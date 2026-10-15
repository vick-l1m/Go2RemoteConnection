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
from app.core.teleop_arbiter import TeleopArbiter

router = APIRouter()
# One driver at a time: an idle page's zero commands are dropped while another page is
# driving (see teleop_arbiter.py for the 2026-10-15 start/stop chatter this fixes).
arbiter = TeleopArbiter(hold_s=0.5)

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

    if not arbiter.accept(cmd.client, lx, ly, az):
        return {"ok": True, "published": False, "reason": "another client is driving",
                "driver": arbiter.driver}

    get_bridge().publish_teleop(lx, ly, az)
    return {"ok": True, "published": True, "linear_x": lx, "linear_y": ly, "angular_z": az}


@router.get("/teleop/arbiter")
async def teleop_arbiter(_=Depends(require_token)):
    """Who is driving, and how many idle-page zeros were dropped."""
    return arbiter.status()