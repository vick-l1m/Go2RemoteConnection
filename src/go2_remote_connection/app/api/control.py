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
from app.api.rl_policy import _load_registry, _current_id

router = APIRouter()

VALID_MODES = {"sport", "rl"}


@router.post("/control_mode/{mode}")
async def set_control_mode(mode: str, _=Depends(require_token)):
    if state.shutting_down:
        raise HTTPException(status_code=503, detail="Server shutting down")

    mode = mode.strip().lower()
    if mode not in VALID_MODES:
        raise HTTPException(status_code=400, detail=f"mode must be one of {sorted(VALID_MODES)}")

    # Switching INTO rl is blocked while STOP is latched. Switching back to sport is
    # always allowed -- it is the safe direction and a failsafe out of a stuck state.
    if mode == "rl" and state.stop_latched:
        raise HTTPException(status_code=423, detail="STOP latched: RESUME before engaging RL")

    # Don't release the sport service for a policy the controller can't actually
    # run (needs perception, or its onnx isn't on the robot).
    if mode == "rl":
        default_id, policies = _load_registry()
        by_id = {p["id"]: p for p in policies}
        sel = _current_id(policies, default_id)
        policy = by_id.get(sel)
        if policy is None:
            raise HTTPException(status_code=409, detail="no RL policy selected")
        if not policy["available"]:
            raise HTTPException(status_code=409, detail=f"policy '{sel}' onnx not found on robot")
        if not policy["runnable"] or policy["uses_heightmap"]:
            raise HTTPException(status_code=409,
                                detail=f"policy '{sel}' needs the perception pipeline (not runnable here)")

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
