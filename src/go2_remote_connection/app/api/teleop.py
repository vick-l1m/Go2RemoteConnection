"""

teleop.py
This module defines the API endpoint for teleoperation commands in the Go2 Remote Actions application.

Version: 2.0
Author: Victor Lim

"""

from fastapi import APIRouter, Depends, HTTPException
from app.core.auth import require_token
from app.core.config import MAX_LIN, MAX_ANG
from app.core.state import state
from app.models.teleop import TeleopCommand
from app.utils.math_utils import clamp
from app.ros_bridge import get_bridge

router = APIRouter()

@router.post("/teleop")
async def teleop(cmd: TeleopCommand, _=Depends(require_token)):
    if state.shutting_down:
        raise HTTPException(status_code=503, detail="Server shutting down")
    if state.stop_latched:
        raise HTTPException(status_code=423, detail="STOP latched: teleop disabled")
    if not state.teleop_enabled:
        raise HTTPException(status_code=423, detail="Teleop disabled (safety stop)")

    lx = clamp(cmd.linear_x, -MAX_LIN, MAX_LIN)
    ly = clamp(cmd.linear_y, -MAX_LIN, MAX_LIN)
    az = clamp(cmd.angular_z, -MAX_ANG, MAX_ANG)

    get_bridge().publish_teleop(lx, ly, az)
    return {"ok": True, "linear_x": lx, "linear_y": ly, "angular_z": az}