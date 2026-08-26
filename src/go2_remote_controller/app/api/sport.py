"""

sport.py
This module defines the API endpoint for sport commands in the Go2 Remote Actions application.

Version: 2.0
Author: Victor Lim

"""

from fastapi import APIRouter, Depends, HTTPException
from app.core.auth import require_token
from app.core.state import state
from app.ros_bridge import get_bridge

router = APIRouter()

@router.post("/sport/{cmd}")
async def sport_action(cmd: str, _=Depends(require_token)):
    if state.shutting_down:
        raise HTTPException(status_code=503, detail="Server shutting down")
    if state.stop_latched:
        raise HTTPException(status_code=423, detail="STOP latched: sport disabled")
    if not state.teleop_enabled:
        raise HTTPException(status_code=423, detail="Teleop disabled (safety stop)")

    get_bridge().publish_sport_cmd({"cmd": cmd})
    return {"ok": True, "cmd": cmd}