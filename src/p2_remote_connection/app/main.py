import logging
from typing import Dict
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel, Field
from app.ros_bridge import start_ros_bridge, get_bridge
from .terminal import terminal_ws
from fastapi import WebSocket

import os
from fastapi import Depends, Header, Request
import hmac
 
import time
import secrets
from typing import Optional, Dict, Any

# Authentication
GO2_API_TOKEN = os.getenv("GO2_API_TOKEN", "")

if not GO2_API_TOKEN:
    raise RuntimeError("GO2_API_TOKEN is not set")

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
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

@app.on_event("startup")
async def on_startup():
    start_ros_bridge()

@app.get("/health")
async def health(_=Depends(require_token)):
    return {"status": "ok"}

# Actions
STOP_LATCHED = False
ALLOWED_ACTIONS = {"sit", "stand", "stop", "standdown", "recover"}

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
    global STOP_LATCHED
    STOP_LATCHED = True
    get_bridge().publish_teleop(0.0, 0.0, 0.0)
    get_bridge().publish_action("stop")
    return {"ok": True, "stop_latched": True}

@app.post("/safety/resume")
async def safety_resume(_=Depends(require_token)):
    global STOP_LATCHED
    STOP_LATCHED = False
    return {"ok": True, "stop_latched": False}

@app.get("/safety/status")
async def safety_status(_=Depends(require_token)):
    return {"stop_latched": STOP_LATCHED}

# Teleop

# --- Teleop lease (single owner) ---
TELEOP_LEASE_TTL_S = 1.0  # how long before lease expires without messages
_teleop_owner_token: Optional[str] = None
_teleop_owner_since: float = 0.0
_teleop_last_seen: float = 0.0
_teleop_lease_id: Optional[str] = None  # random id per lease

class TeleopCommand(BaseModel):
    linear_x: float = Field(0.0)
    linear_y: float = Field(0.0)
    angular_z: float = Field(0.0)

MAX_LIN = 0.6
MAX_ANG = 1.2

def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))

def _lease_active(now: float) -> bool:
    return (_teleop_owner_token is not None) and ((now - _teleop_last_seen) <= TELEOP_LEASE_TTL_S)

@app.post("/teleop")
async def teleop(cmd: TeleopCommand, request: Request, _=Depends(require_token)):
    global _teleop_last_seen, _teleop_owner_token, _teleop_lease_id

    if STOP_LATCHED:
        raise HTTPException(status_code=423, detail="STOP latched: teleop disabled")

    auth = request.headers.get("authorization", "")
    now = time.time()

    # If no active lease, reject (or allow zeros only — up to you)
    if not _lease_active(now):
        # Lease expired -> clear
        _teleop_owner_token = None
        _teleop_lease_id = None
        get_bridge().publish_teleop(0.0, 0.0, 0.0)
        raise HTTPException(status_code=409, detail="Teleop not started or lease expired")

    if auth != _teleop_owner_token:
        raise HTTPException(status_code=403, detail="Teleop owned by another client")

    # Refresh lease
    _teleop_last_seen = now

    lx = clamp(cmd.linear_x, -MAX_LIN, MAX_LIN)
    ly = clamp(cmd.linear_y, -MAX_LIN, MAX_LIN)
    az = clamp(cmd.angular_z, -MAX_ANG, MAX_ANG)

    get_bridge().publish_teleop(lx, ly, az)
    return {"ok": True, "linear_x": lx, "linear_y": ly, "angular_z": az, "lease_id": _teleop_lease_id}


# Terminal websocket
@app.websocket("/ws/terminal")
async def ws_terminal(websocket: WebSocket):
    await terminal_ws(websocket)

@app.post("/teleop/start")
async def teleop_start(request: Request, _=Depends(require_token)):
    global _teleop_owner_token, _teleop_owner_since, _teleop_last_seen, _teleop_lease_id

    if STOP_LATCHED:
        raise HTTPException(status_code=423, detail="STOP latched: teleop disabled")

    auth = request.headers.get("authorization", "")
    now = time.time()

    if not auth.startswith("Bearer "):
        raise HTTPException(status_code=401, detail="Missing bearer token")

    # If someone owns it and it's still active
    if _lease_active(now):
        if auth == _teleop_owner_token:
            # idempotent re-start
            _teleop_last_seen = now
            return {"ok": True, "lease_id": _teleop_lease_id, "owner": "you", "active": True}

        raise HTTPException(status_code=409, detail="Teleop already owned by another client")

    # Acquire lease
    _teleop_owner_token = auth
    _teleop_owner_since = now
    _teleop_last_seen = now
    _teleop_lease_id = secrets.token_urlsafe(8)

    return {"ok": True, "lease_id": _teleop_lease_id, "active": True}

@app.post("/teleop/stop")
async def teleop_stop(request: Request, _=Depends(require_token)):
    global _teleop_owner_token, _teleop_owner_since, _teleop_last_seen, _teleop_lease_id

    if STOP_LATCHED:
        # do not publish anything while latched
        _teleop_owner_token = None
        _teleop_lease_id = None
        _teleop_owner_since = 0.0
        _teleop_last_seen = 0.0
        return {"ok": True, "stopped": True, "note": "STOP latched: teleop cleared without publishing."}

    auth = request.headers.get("authorization", "")
    now = time.time()

    if not auth.startswith("Bearer "):
        raise HTTPException(status_code=401, detail="Missing bearer token")

    # Only the owner can stop; if lease already expired, treat as already stopped
    if _teleop_owner_token is None or not _lease_active(now):
        _teleop_owner_token = None
        _teleop_lease_id = None
        # send a final stop anyway (safe)
        get_bridge().publish_teleop(0.0, 0.0, 0.0)
        return {"ok": True, "stopped": True, "note": "No active lease (already stopped/expired)."}

    if auth != _teleop_owner_token:
        raise HTTPException(status_code=403, detail="You do not own teleop lease")

    # Final zero + release
    get_bridge().publish_teleop(0.0, 0.0, 0.0)

    _teleop_owner_token = None
    _teleop_lease_id = None
    _teleop_owner_since = 0.0
    _teleop_last_seen = 0.0

    return {"ok": True, "stopped": True}

@app.get("/teleop/status")
async def teleop_status():
    now = time.time()
    active = _lease_active(now)
    return {
        "active": active,
        "lease_id": _teleop_lease_id if active else None,
        "ttl_s": TELEOP_LEASE_TTL_S,
        "age_s": (now - _teleop_owner_since) if active else 0.0,
        "idle_s": (now - _teleop_last_seen) if active else 0.0,
    }

