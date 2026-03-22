"""

safety.py
This module defines the API endpoints for safety controls in the Go2 Remote Actions application.

Version: 2.0
Author: Victor Lim

"""

from fastapi import APIRouter, Depends
from app.core.auth import require_token
from app.core.state import state
from app.ros_bridge import get_bridge

router = APIRouter()

@router.post("/safety/stop")
async def safety_stop(_=Depends(require_token)):
    state.stop_latched = True
    state.teleop_enabled = False
    get_bridge().publish_enabled(False)
    return {"ok": True, "stop_latched": True, "teleop_enabled": False}

@router.post("/safety/resume")
async def safety_resume(_=Depends(require_token)):
    state.stop_latched = False
    state.teleop_enabled = True
    get_bridge().publish_enabled(True)
    return {"ok": True, "stop_latched": False, "teleop_enabled": True}

@router.get("/safety/status")
async def safety_status(_=Depends(require_token)):
    return {
        "stop_latched": state.stop_latched,
        "teleop_enabled": state.teleop_enabled,
    }