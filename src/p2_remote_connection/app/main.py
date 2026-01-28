import logging
from typing import Dict
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel, Field
from app.ros_bridge import start_ros_bridge, get_bridge
from fastapi import WebSocket

import os
from fastapi import Depends, Header, Request
import hmac
 
import time
import secrets
from typing import Optional, Dict, Any

# Conditional import: use terminal_ec2 for EC2 deployment, terminal for robot
if os.getenv("DEPLOYMENT_ENV") == "ec2":
    from .terminal_ec2 import terminal_ws
else:
    from .terminal import terminal_ws

# Authentication
GO2_API_TOKEN = (os.getenv("GO2_API_TOKEN") or "").strip()
if not GO2_API_TOKEN:
    raise RuntimeError("GO2_API_TOKEN is not set (or is empty)")

def require_token(authorization: str = Header(None)) -> str:
    if not GO2_API_TOKEN:
        raise HTTPException(status_code=500, detail="Server missing GO2_API_TOKEN")

    if not authorization:
        raise HTTPException(status_code=401, detail="Missing Authorization header")

    # Accept: "Bearer <token>"
    parts = authorization.split(" ", 1)
    if len(parts) != 2 or parts[0].lower() != "bearer":
        raise HTTPException(status_code=401, detail="Invalid Authorization header format")

    token = parts[1].strip()

    # constant-time compare (safer)
    if not hmac.compare_digest(token, GO2_API_TOKEN):
        raise HTTPException(status_code=403, detail="Invalid token")

    return token

logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")
logger = logging.getLogger(__name__)

app = FastAPI(title="Go2 Remote Actions")

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=False,
    allow_methods=["GET", "POST", "OPTIONS"],
    allow_headers=["Authorization", "Content-Type"],
    max_age=86400,
)

@app.on_event("startup")
async def on_startup():
    start_ros_bridge()
    
@app.get("/health")
async def health():
    return {"ok": True, "status": "ok"}

# Actions
STOP_LATCHED = False
TELEOP_ENABLED = True
ALLOWED_ACTIONS = {
  "sit",
  "stand",
  "stop",
  "standdown",
  "recover"
}

@app.post("/actions/{action}")
async def start_action(action: str, _=Depends(require_token)):
    global STOP_LATCHED

    if STOP_LATCHED and action != "stop":
        raise HTTPException(status_code=423, detail="STOP latched: actions disabled")

    if action not in ALLOWED_ACTIONS:
        raise HTTPException(status_code=404, detail="Action not allowed")
    get_bridge().publish_action(action)
    return {"ok": True, "action": action}

@app.post("/safety/stop")
async def safety_stop(_=Depends(require_token)):
    global STOP_LATCHED, TELEOP_ENABLED
    STOP_LATCHED = True
    TELEOP_ENABLED = False

    # Tell ROS side to disable teleop node behavior
    get_bridge().publish_enabled(False)

    return {"ok": True, "stop_latched": True, "teleop_enabled": False}

@app.post("/safety/resume")
async def safety_resume(_=Depends(require_token)):
    global STOP_LATCHED, TELEOP_ENABLED
    STOP_LATCHED = False
    TELEOP_ENABLED = True

    get_bridge().publish_enabled(True)
    
    return {"ok": True, "stop_latched": False, "teleop_enabled": True}

@app.get("/safety/status")
async def safety_status(_=Depends(require_token)):
    return {"stop_latched": STOP_LATCHED, "teleop_enabled": TELEOP_ENABLED}


# Teleop
class TeleopCommand(BaseModel):
    linear_x: float = Field(0.0)
    linear_y: float = Field(0.0)
    angular_z: float = Field(0.0)

MAX_LIN = 0.6
MAX_ANG = 1.2

def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))

@app.post("/teleop")
async def teleop(cmd: TeleopCommand, request: Request, _=Depends(require_token)):
    if STOP_LATCHED:
        raise HTTPException(status_code=423, detail="STOP latched: teleop disabled")

    if not TELEOP_ENABLED:
        raise HTTPException(status_code=423, detail="Teleop disabled (safety stop)")

    lx = clamp(cmd.linear_x, -MAX_LIN, MAX_LIN)
    ly = clamp(cmd.linear_y, -MAX_LIN, MAX_LIN)
    az = clamp(cmd.angular_z, -MAX_ANG, MAX_ANG)

    get_bridge().publish_teleop(lx, ly, az)
    return {"ok": True, "linear_x": lx, "linear_y": ly, "angular_z": az}


# Terminal websocket
@app.websocket("/ws/terminal")
async def ws_terminal(websocket: WebSocket):
    await terminal_ws(websocket)

