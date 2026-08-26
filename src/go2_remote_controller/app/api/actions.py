"""

actions.py
This module defines the API endpoints for starting actions in the Go2 Remote Actions application.

Version: 2.0
Author: Victor Lim

"""

from fastapi import APIRouter, Depends, HTTPException
from app.core.auth import require_token
from app.core.state import state, ALLOWED_ACTIONS
from app.ros_bridge import get_bridge

router = APIRouter()

@router.post("/actions/{action}")
async def start_action(action: str, _=Depends(require_token)):
    if state.stop_latched and action != "stop":
        raise HTTPException(status_code=423, detail="STOP latched: actions disabled")

    if action not in ALLOWED_ACTIONS:
        raise HTTPException(status_code=404, detail="Action not allowed")

    get_bridge().publish_action(action)
    return {"ok": True, "action": action}