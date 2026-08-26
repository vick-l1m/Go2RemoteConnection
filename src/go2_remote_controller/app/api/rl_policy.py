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
    if not policy["runnable"] or policy["uses_heightmap"]:
        raise HTTPException(status_code=409,
                            detail=f"policy '{policy_id}' needs the perception pipeline (not runnable here)")

    state.rl_policy_id = policy_id
    get_bridge().publish_rl_policy(policy_id)
    return {"ok": True, "current": policy_id}
