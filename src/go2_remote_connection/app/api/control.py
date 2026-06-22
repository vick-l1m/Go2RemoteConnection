"""

control.py
Selects the active locomotion controller: Unitree's built-in SportClient gaits
("sport") or the low-level RL policy ("rl", served by go2_rl_policy_node).

Publishing "/web_control_mode" tells go2_rl_policy_node to engage/disengage and
tells web_bridge to idle its SportClient output while the RL policy owns the
motors. The RL node performs the actual sport-service handover (release/recover).

Version: 1.0
Author: Victor Lim

"""

from fastapi import APIRouter, Depends, HTTPException
from app.core.auth import require_token
from app.core.state import state
from app.ros_bridge import get_bridge

router = APIRouter()

VALID_MODES = {"sport", "rl"}


@router.post("/control_mode/{mode}")
async def set_control_mode(mode: str, _=Depends(require_token)):
    if state.shutting_down:
        raise HTTPException(status_code=503, detail="Server shutting down")
    if state.stop_latched:
        raise HTTPException(status_code=423, detail="STOP latched: control mode locked")

    mode = mode.strip().lower()
    if mode not in VALID_MODES:
        raise HTTPException(status_code=400, detail=f"mode must be one of {sorted(VALID_MODES)}")

    bridge = get_bridge()
    # Order matters for safe mutual exclusion:
    #   -> rl  : idle web_bridge FIRST, then tell the RL node to engage.
    #   -> sport: tell the RL node to disengage FIRST, then re-enable web_bridge.
    if mode == "rl":
        bridge.publish_enabled(False)        # web_bridge tick() early-returns -> no SportClient output
        bridge.publish_control_mode("rl")
    else:
        bridge.publish_control_mode("sport")  # RL node ramps down + recovers the sport service
        bridge.publish_enabled(True)          # web_bridge resumes SportClient teleop

    state.control_mode = mode
    return {"ok": True, "control_mode": mode}


@router.get("/control_mode")
async def get_control_mode(_=Depends(require_token)):
    return {"control_mode": state.control_mode}
