"""

rl_policy.py
Lists the RL policies selectable from the RL web page's dropdown and lets the
operator pick which one the low-level controller (go2_rl_policy_node) runs.

The registry (rl_policy/policies.json) is the single source of truth for the
dropdown. Selecting a policy publishes its id on /web_rl_policy; the bridge
forwards it to the controller, which hot-swaps the onnx (idle only). The
controller echoes the policy it actually loaded back to server state.

Version: 1.0
Author: Victor Lim

"""

import json
import pathlib

from fastapi import APIRouter, Depends, HTTPException
from app.core.auth import require_token
from app.core.state import state
from app.ros_bridge import get_bridge

router = APIRouter()

# rl_policy/policies.json lives beside the app package: app/api/ -> ../../rl_policy/
REGISTRY_PATH = pathlib.Path(__file__).resolve().parents[2] / "rl_policy" / "policies.json"


def _load_registry():
    """Read the registry fresh on each request so hand-edits are picked up without
    a server restart. Returns (default_id, [enriched policy dicts])."""
    try:
        data = json.loads(REGISTRY_PATH.read_text())
    except FileNotFoundError:
        raise HTTPException(status_code=500, detail=f"policy registry not found: {REGISTRY_PATH}")
    except (ValueError, OSError) as e:
        raise HTTPException(status_code=500, detail=f"policy registry unreadable: {e}")

    reg_dir = REGISTRY_PATH.parent
    policies = []
    for entry in data.get("policies", []):
        pid = str(entry.get("id", "")).strip()
        if not pid:
            continue
        path = pathlib.Path(entry.get("path", ""))
        if not path.is_absolute():
            path = reg_dir / path
        policies.append({
            "id": pid,
            "name": entry.get("name", pid),
            "model": entry.get("model", ""),
            "type": entry.get("type", "blind"),
            # Absent means velocity: every policy predating the sit/stand task is
            # joystick-driven, so the default keeps old registry entries working.
            "control": entry.get("control", "velocity"),
            "obs_dim": entry.get("obs_dim"),
            "uses_heightmap": bool(entry.get("uses_heightmap", False)),
            "runnable": bool(entry.get("runnable", True)),
            "available": path.exists(),      # onnx present on disk?
        })
    default_id = data.get("default") or (policies[0]["id"] if policies else "")
    return default_id, policies


def _current_id(policies, default_id):
    """The selected policy id, falling back to the registry default when the node
    has not reported one yet."""
    sel = state.rl_policy_id or default_id
    ids = {p["id"] for p in policies}
    return sel if sel in ids else (default_id if default_id in ids else "")


@router.get("/rl/policies")
async def list_policies(_=Depends(require_token)):
    default_id, policies = _load_registry()
    return {
        "policies": policies,
        "default": default_id,
        "current": _current_id(policies, default_id),
    }


@router.get("/rl/policy")
async def get_policy(_=Depends(require_token)):
    default_id, policies = _load_registry()
    return {"current": _current_id(policies, default_id)}


@router.post("/rl/policy/{policy_id}")
async def set_policy(policy_id: str, _=Depends(require_token)):
    if state.shutting_down:
        raise HTTPException(status_code=503, detail="Server shutting down")

    default_id, policies = _load_registry()
    by_id = {p["id"]: p for p in policies}
    policy = by_id.get(policy_id)
    if policy is None:
        raise HTTPException(status_code=404, detail=f"unknown policy '{policy_id}'")

    # Switching the policy is an idle-only operation: it changes the control law,
    # so we forbid it while RL is engaged or the system is STOP-latched. Switch
    # back to sport (or RESUME) first.
    if state.control_mode == "rl":
        raise HTTPException(status_code=423, detail="turn RL off before switching policy")
    if state.stop_latched:
        raise HTTPException(status_code=423, detail="STOP latched: RESUME before switching policy")

    if not policy["available"]:
        raise HTTPException(status_code=409, detail=f"policy '{policy_id}' onnx not found on robot")
    if not policy["runnable"]:
        raise HTTPException(status_code=409,
                            detail=f"policy '{policy_id}' is not runnable on this robot")
    # uses_heightmap is NO LONGER a refusal here. The controller can be fed a live height
    # scan now (go2_rl_bridge_node forwards /go2/height_scan over the UDP link), so
    # whether a perception policy may run depends on whether the camera is publishing
    # right now -- which this endpoint cannot know and must not guess. SELECTING one is
    # harmless; the gate that matters is in go2_rl_policy_node._engage(), which refuses to
    # release the sport service without a scan newer than HEIGHT_SCAN_TIMEOUT and bounces
    # the UI toggle back to sport. Keeping a duplicate static check here would just make
    # a correctly-running perception stack un-selectable.

    state.rl_policy_id = policy_id
    get_bridge().publish_rl_policy(policy_id)
    return {"ok": True, "current": policy_id}


# ---------------------------------------------------------------------------
# Posture (sit/stand) commands for a posture policy.
#
# These are NOT the /actions/{sit,stand} endpoints. Those publish /web_action, which
# web_bridge turns into SportClient.StandDown()/StandUp() -- i.e. they hand the robot
# to the Unitree firmware. Sending one while an RL policy owns the motors would put
# two drivers on the robot at once. This path instead latches a target posture the
# running policy reads as its `posture_command` observation.
# ---------------------------------------------------------------------------

VALID_POSTURES = {"sit", "stand"}


def _posture_policy_selected():
    """(policy dict, current id) for the selected policy, or (None, id)."""
    default_id, policies = _load_registry()
    cur = _current_id(policies, default_id)
    return next((p for p in policies if p["id"] == cur), None), cur


@router.get("/rl/posture")
async def get_posture(_=Depends(require_token)):
    policy, cur = _posture_policy_selected()
    return {
        "posture": state.rl_posture,
        "policy": cur,
        # Whether the RL Actions panel applies to what is loaded. The UI uses this to
        # enable/disable the buttons rather than guessing from the policy name.
        "supported": bool(policy and policy.get("control") == "posture"),
    }


@router.post("/rl/posture/{posture}")
async def set_posture(posture: str, _=Depends(require_token)):
    if state.shutting_down:
        raise HTTPException(status_code=503, detail="Server shutting down")

    posture = posture.strip().lower()
    if posture not in VALID_POSTURES:
        raise HTTPException(status_code=400, detail=f"posture must be one of {sorted(VALID_POSTURES)}")

    if state.stop_latched:
        raise HTTPException(status_code=423, detail="STOP latched: RESUME before commanding a posture")

    # A posture only means something to a running policy. In sport mode the firmware
    # owns the robot and the ordinary Sit/Stand buttons are the right control.
    if state.control_mode != "rl":
        raise HTTPException(status_code=409,
                            detail="RL is off — use the sport Sit/Stand buttons, or turn RL on first")

    policy, cur = _posture_policy_selected()
    if policy is None or policy.get("control") != "posture":
        raise HTTPException(status_code=409,
                            detail=f"policy '{cur}' is velocity-driven; it has no posture command")

    state.rl_posture = posture
    get_bridge().publish_rl_posture(posture)
    return {"ok": True, "posture": posture}
