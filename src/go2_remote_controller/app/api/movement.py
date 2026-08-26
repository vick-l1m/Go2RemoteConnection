"""

movement.py
This module defines the API endpoints for movement commands in the Go2 Remote Actions application.

Version: 2.0
Author: Victor Lim

"""

from fastapi import APIRouter, Depends, HTTPException
from app.core.auth import require_token
from app.core.state import state
from app.models.movement import MoveForwardReq
from app.ros_bridge import get_bridge

router = APIRouter()

@router.post("/move_forward")
async def move_forward(req: MoveForwardReq, _=Depends(require_token)):
    if state.stop_latched:
        raise HTTPException(status_code=423, detail="STOP latched: move disabled")
    if not state.teleop_enabled:
        raise HTTPException(status_code=423, detail="Teleop disabled (safety stop)")

    meters = float(req.meters)
    if meters <= 0.0:
        raise HTTPException(status_code=400, detail="meters must be > 0")

    get_bridge().publish_move_forward(meters)
    return {"ok": True, "meters": meters}