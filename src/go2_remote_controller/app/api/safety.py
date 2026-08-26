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
    """Genuinely stop all movement: the active controller damps and the robot
    soft-collapses to the ground. Latched until /safety/resume.

    - rl mode    : tell go2_rl_policy_node to ESTOP (damp the motors directly;
                   also disarms the firmware auto-recovery so it stays down).
    - sport mode : tell web_bridge to disarm the firmware auto-recovery, then Damp,
                   so the robot stays down instead of auto-standing.
    """
    bridge = get_bridge()
    state.stop_latched = True
    state.teleop_enabled = False
    bridge.publish_enabled(False)            # stop web_bridge teleop output

    if state.control_mode == "rl":
        bridge.publish_estop(True)           # RL node -> damp (soft collapse)
    else:
        # Disarm the firmware auto-recovery FIRST so the Damp'd robot stays down and
        # the sport service does not auto-stand it; then damp. Re-armed on resume.
        bridge.publish_action("autorecovery_off")
        bridge.publish_action("damp")        # sport service -> Damp (soft collapse)

    return {"ok": True, "stop_latched": True, "teleop_enabled": False,
            "control_mode": state.control_mode}

@router.post("/safety/resume")
async def safety_resume(_=Depends(require_token)):
    """Get back up and ready to move again, in whichever mode is active.

    - rl mode    : clear the ESTOP -> the RL node re-arms the firmware auto-recovery,
                   ramps back to the default pose and resumes the policy (joystick-ready).
    - sport mode : re-arm the firmware auto-recovery, then SportClient.RecoveryStand()
                   to stand up, and re-enable teleop.
    """
    bridge = get_bridge()
    state.stop_latched = False
    state.teleop_enabled = True
    bridge.publish_enabled(True)             # ALWAYS un-gate the enable flag (failsafe)

    if state.control_mode == "rl":
        bridge.publish_estop(False)          # RL node -> stand up under policy, ready
        # web_bridge still idles via its rl_mode_ gate while RL owns the motors
    else:
        # Re-arm the firmware auto-recovery that STOP disarmed, then stand back up.
        bridge.publish_action("autorecovery_on")
        bridge.publish_action("recovery")    # sport service -> RecoveryStand

    return {"ok": True, "stop_latched": False, "teleop_enabled": True,
            "control_mode": state.control_mode}

@router.get("/safety/status")
async def safety_status(_=Depends(require_token)):
    return {
        "stop_latched": state.stop_latched,
        "teleop_enabled": state.teleop_enabled,
    }