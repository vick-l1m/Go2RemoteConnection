import asyncio
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
from typing import Optional, Dict, Any, Set 

import json
import gzip
from fastapi.responses import Response
from app.ros_bridge import get_map_store

from dataclasses import dataclass, field

from fastapi.staticfiles import StaticFiles
from .ros_bridge import get_pcd_store 

from app.ros_bridge import get_cam_store

# Conditional import: use terminal_ec2 for EC2 deployment, terminal for robot
if os.getenv("DEPLOYMENT_ENV") == "ec2":
    from .terminal_ec2 import terminal_ws
else:
    from .terminal import terminal_ws


# Authentication
# ------------------------------------------------------------
# Option to disable auth for testing
AUTH_ENABLED = os.getenv("GO2_AUTH_ENABLED", "1") not in ("0", "false", "False")

# Authentication token (ONLY required if auth enabled)
GO2_API_TOKEN = (os.getenv("GO2_API_TOKEN") or "").strip()

if AUTH_ENABLED and not GO2_API_TOKEN:
    raise RuntimeError("GO2_API_TOKEN is not set (or is empty) while GO2_AUTH_ENABLED=1")


def require_token(authorization: str = Header(None)) -> str:
    # 🔓 Auth disabled (dev / sim mode)
    if not AUTH_ENABLED:
        return ""

    if not GO2_API_TOKEN:
        raise HTTPException(status_code=500, detail="Server missing GO2_API_TOKEN")

    if not authorization:
        raise HTTPException(status_code=401, detail="Missing Authorization header")

    # Accept: "Bearer <token>"
    parts = authorization.split(" ", 1)
    if len(parts) != 2 or parts[0].lower() != "bearer":
        raise HTTPException(status_code=401, detail="Invalid Authorization header format")

    token = parts[1].strip()

    if not hmac.compare_digest(token, GO2_API_TOKEN):
        raise HTTPException(status_code=403, detail="Invalid token")

    return token

logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")
logger = logging.getLogger(__name__)

app = FastAPI(title="Go2 Remote Actions")

HERE = os.path.dirname(__file__)
STATIC_DIR = os.path.join(HERE, "static")
app.mount("/app/static", StaticFiles(directory=STATIC_DIR), name="static")

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=False,
    allow_methods=["GET", "POST", "OPTIONS"],
    allow_headers=["Authorization", "Content-Type"],
    expose_headers=[
        "x-map-frame",
        "x-map-resolution",
        "x-map-width",
        "x-map-height",
        "x-map-origin-x",
        "x-map-origin-y",
        "x-map-seq",
        "content-encoding",
    ],
    max_age=86400,
)

@app.on_event("startup")
async def on_startup():
    start_ros_bridge()
    # give ROS bridge the running asyncio loop so it can broadcast map updates
    get_bridge().set_asyncio_loop(asyncio.get_running_loop())

@app.get("/health")
async def health():
    return {"ok": True, "status": "ok"}

SHUTTING_DOWN = False

@app.on_event("shutdown")
async def on_shutdown():
    global SHUTTING_DOWN
    SHUTTING_DOWN = True

# ------------------------------------------------------------
# Actions
# ------------------------------------------------------------
STOP_LATCHED = False
TELEOP_ENABLED = True
ALLOWED_ACTIONS = {
  "sit","stand","stop","standdown","recover",

  # Mode switching
  "mode_movement","mode_posing","mode_actions",

  # Movement-mode commands
  "toggle_freebound","toggle_freeavoid","toggle_crossstep","toggle_freejump",
  "toggle_walkupright","toggle_handstand", "toggle_classicwalk",
  "trot_run","economic_gait","switch_avoid","cycle_speed",
  "stop_move","static_walk", "freewalk",

  # Posing-mode commands
  "damp","balance_stand",

  # Actions-mode commands
  "sit_toggle","hello","stretch","content","heart","scrape",
  "front_pounce","front_jump","front_flip","back_flip", "left_flip", "recovery"
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

@app.get("/config")
async def config():
    return {
        "auth_enabled": AUTH_ENABLED
    }

# ------------------------------------------------------------
# Teleop
# ------------------------------------------------------------
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
    if SHUTTING_DOWN:
        raise HTTPException(status_code=503, detail="Server shutting down")

    if STOP_LATCHED:
        raise HTTPException(status_code=423, detail="STOP latched: teleop disabled")

    if not TELEOP_ENABLED:
        raise HTTPException(status_code=423, detail="Teleop disabled (safety stop)")

    lx = clamp(cmd.linear_x, -MAX_LIN, MAX_LIN)
    ly = clamp(cmd.linear_y, -MAX_LIN, MAX_LIN)
    az = clamp(cmd.angular_z, -MAX_ANG, MAX_ANG)

    get_bridge().publish_teleop(lx, ly, az)
    return {"ok": True, "linear_x": lx, "linear_y": ly, "angular_z": az}

# ------------------------------------------------------------
# Terminal websocket
# ------------------------------------------------------------
@app.websocket("/ws/terminal")
async def ws_terminal(websocket: WebSocket):
    await terminal_ws(websocket)

# ------------------------------------------------------------
# Map2D endpoints
# ------------------------------------------------------------
@app.get("/map2d/full")
async def map2d_full(_=Depends(require_token)):
    store = get_map_store()
    async with store.lock:
        if store.meta is None or store.full_raw is None:
            return Response(status_code=204)

        meta = store.meta
        gz = gzip.compress(store.full_raw, compresslevel=6)

        headers = {
            "X-Map-Frame": meta["frame_id"],
            "X-Map-Resolution": str(meta["resolution"]),
            "X-Map-Width": str(meta["width"]),
            "X-Map-Height": str(meta["height"]),
            "X-Map-Origin-X": str(meta["origin_x"]),
            "X-Map-Origin-Y": str(meta["origin_y"]),
            "X-Map-Seq": str(store.seq),

            # IMPORTANT: do NOT set Content-Encoding gzip (browser auto-decompresses)
            "X-Map-Encoding": "gzip",
            "Content-Type": "application/octet-stream",
        }
        return Response(content=gz, headers=headers)

@app.websocket("/ws/map2d")
async def ws_map2d(websocket: WebSocket):
    # Authenticate BEFORE accept (same style as terminal.py)
    token = websocket.query_params.get("token", "")

    if AUTH_ENABLED:
        if not GO2_API_TOKEN:
            await websocket.close(code=1011)
            return

        if not hmac.compare_digest(token, GO2_API_TOKEN):
            await websocket.close(code=1008)
            return

    await websocket.accept()

    # Optional: if STOP latched, immediately refuse (client will reconnect on RESUME)
    if STOP_LATCHED:
        await websocket.close(code=1013)  # Try again later
        return

    store = get_map_store()

    # Register client
    async with store.lock:
        store.clients.add(websocket)

        # Send latest full map immediately if available
        initial = None
        if store.meta is not None and store.full_raw is not None:
            initial = {
                "t": "f",
                "seq": store.seq,
                "meta": store.meta,
                "gz": gzip.compress(store.full_raw, compresslevel=6),
            }

    try:
        if initial:
            header = initial.copy()
            gz = header.pop("gz")
            await websocket.send_text(json.dumps(header))
            await websocket.send_bytes(gz)

        # Keepalive loop (client sends "ping")
        while True:
            _ = await websocket.receive_text()
            if STOP_LATCHED:
                await websocket.close(code=1013)
                return

    except Exception:
        pass
    finally:
        async with store.lock:
            store.clients.discard(websocket)

# ------------------------------------------------------------
# MoveForward x meters
# ------------------------------------------------------------
class MoveForwardReq(BaseModel):
    meters: float = Field(..., ge=0.0, le=5.0, description="Distance in meters (0–5)")

@app.post("/move_forward")
async def move_forward(req: MoveForwardReq, _=Depends(require_token)):
    if STOP_LATCHED:
        raise HTTPException(status_code=423, detail="STOP latched: move disabled")

    if not TELEOP_ENABLED:
        raise HTTPException(status_code=423, detail="Teleop disabled (safety stop)")

    meters = float(req.meters)
    if meters <= 0.0:
        raise HTTPException(status_code=400, detail="meters must be > 0")

    # ✅ This assumes you add publish_move_forward(meters) in ros_bridge.py
    get_bridge().publish_move_forward(meters)
    return {"ok": True, "meters": meters}

# ------------------------------------------------------------
# Sport command passthrough (button-style)
# ------------------------------------------------------------
class SportCmdReq(BaseModel):
    cmd: str = Field(..., min_length=1)
    enable: Optional[bool] = None
    level: Optional[int] = None
    roll: Optional[float] = None
    pitch: Optional[float] = None
    yaw: Optional[float] = None

@app.post("/sport/{cmd}")
async def sport_action(cmd: str, _=Depends(require_token)):
    if SHUTTING_DOWN:
        raise HTTPException(status_code=503, detail="Server shutting down")
    if STOP_LATCHED:
        raise HTTPException(status_code=423, detail="STOP latched: sport disabled")
    if not TELEOP_ENABLED:
        raise HTTPException(status_code=423, detail="Teleop disabled (safety stop)")

    # allowlist if you want:
    # if cmd not in ALLOWED_SPORT_CMDS:
    #     raise HTTPException(status_code=404, detail="Sport cmd not allowed")

    get_bridge().publish_sport_cmd({"cmd": cmd})
    return {"ok": True, "cmd": cmd}

# ------------------------------------------------------------
# PointCloud store
# ------------------------------------------------------------
@app.websocket("/ws/pcd")
async def ws_pcd(websocket: WebSocket):
    token = websocket.query_params.get("token", "")

    if AUTH_ENABLED:
        if not GO2_API_TOKEN:
            await websocket.close(code=1011); return
        if not hmac.compare_digest(token, GO2_API_TOKEN):
            await websocket.close(code=1008); return

    await websocket.accept()
    if STOP_LATCHED:
        await websocket.close(code=1013); return

    store = get_pcd_store()

    # register + capture initial frame
    async with store.lock:
        store.clients.add(websocket)
        initial_header = None
        initial_gz = None
        if store.meta is not None and store.raw is not None:
            initial_header = {
                "t": "pcd",
                "seq": store.seq,
                "meta": store.meta,
                "n": len(store.raw) // 12,
            }
            initial_gz = gzip.compress(store.raw, compresslevel=6)

    try:
        if initial_header:
            await websocket.send_text(json.dumps(initial_header))
            await websocket.send_bytes(initial_gz)

        while True:
            _ = await websocket.receive_text()  # keepalive ping
            if STOP_LATCHED:
                await websocket.close(code=1013)
                return
    except Exception:
        pass
    finally:
        async with store.lock:
            store.clients.discard(websocket)

# ------------------------------------------------------------
# CAMERA STORE (JPEG bytes)
# ------------------------------------------------------------
@app.websocket("/ws/cam_front")
async def ws_cam_front(websocket: WebSocket):
    token = websocket.query_params.get("token", "")

    if AUTH_ENABLED:
        if not GO2_API_TOKEN:
            await websocket.close(code=1011); return
        if not hmac.compare_digest(token, GO2_API_TOKEN):
            await websocket.close(code=1008); return

    await websocket.accept()
    if STOP_LATCHED:
        await websocket.close(code=1013); return

    store = get_cam_store()

    # register + initial
    async with store.lock:
        store.clients.add(websocket)
        initial_header = None
        initial_jpg = None
        if store.meta is not None and store.jpg is not None:
            initial_header = {
                "t": "cam",
                "seq": store.seq,
                "meta": store.meta,
                "n": len(store.jpg),
            }
            initial_jpg = store.jpg

    try:
        if initial_header:
            await websocket.send_text(json.dumps(initial_header))
            await websocket.send_bytes(initial_jpg)

        while True:
            _ = await websocket.receive_text()  # keepalive ping
            if STOP_LATCHED:
                await websocket.close(code=1013)
                return
    except Exception:
        pass
    finally:
        async with store.lock:
            store.clients.discard(websocket)
