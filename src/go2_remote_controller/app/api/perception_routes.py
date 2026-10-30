"""
perception_routes.py
RealSense + height-map views for the RL web page:

  GET /perception/status            is a RealSense on USB (and USB 2 or 3), is the
                                    driver publishing, per-topic frame rates, grid info,
                                    plus the perception stack the selected policy needs
                                    (``stack_required``, from its deploy.yaml) and the one
                                    the height-map node reports (``stack_running``)
  WS  /ws/height_map                JSON frames: {"t":"height_scan",...} (the policy's
                                    17x11 obs) and {"t":"grid_map",...} (GridMap layers)
  WS  /ws/cam_realsense?stream=     depth | color -- JSON header + JPEG bytes, the
                                    same wire format as /ws/cam_front

Same auth / stop-latch / keepalive shape as robot_model_routes.py and camera_routes.py.

Version: 1.0
Author: Victor Lim
"""

import json
import pathlib

import yaml
from fastapi import APIRouter, Depends, WebSocket

from app.core.auth import require_token
from app.core.state import state
from app.services.realsense_usb import get_realsense_status
from app.services.websocket_auth import authenticate_websocket
from app.ros_bridge import get_perception_status, get_perception_store
from app.api.rl_policy import REGISTRY_PATH, _current_id, _load_registry

router = APIRouter()

_REQUIRED_CACHE: dict = {}      # deploy.yaml path -> (mtime, parsed requirement)


def _policy_requirement(entry: dict) -> dict:
    """The perception stack a policy was trained on, read from its deploy.yaml:
    snapped levels or raw heights, rolling-map memory, camera pipeline."""
    path = pathlib.Path(entry.get("path", ""))
    if not path.is_absolute():
        path = REGISTRY_PATH.parent / path
    deploy = path.parent / "deploy.yaml"
    out = {"policy": entry.get("id"), "name": entry.get("name", entry.get("id")),
           "deploy_found": deploy.exists()}
    if not deploy.exists():
        return out
    mtime = deploy.stat().st_mtime
    cached = _REQUIRED_CACHE.get(str(deploy))
    if cached and cached[0] == mtime:
        return {**out, **cached[1]}
    try:
        d = yaml.safe_load(deploy.read_text()) or {}
    except (OSError, yaml.YAMLError):
        return {**out, "deploy_found": False}
    hs = (d.get("observations") or {}).get("height_scan")
    params = (hs or {}).get("params") or {}
    levels = (params.get("snap") or {}).get("levels")
    memory = params.get("memory")
    req = {
        "height_scan": hs is not None,
        "snap": bool(levels),
        "levels": [round(float(v), 4) for v in levels] if levels else None,
        "memory_s": float(memory.get("memory_s", 2.0)) if memory else 0.0,
        "camera_profile": d.get("camera_profile"),
    }
    _REQUIRED_CACHE[str(deploy)] = (mtime, req)
    return {**out, **req}


@router.get("/perception/status", dependencies=[Depends(require_token)])
async def perception_status():
    status = get_perception_status()
    status["realsense"] = get_realsense_status()
    try:
        default_id, policies = _load_registry()
        cur = _current_id(policies, default_id)
        raw = {str(e.get("id")): e for e in json.loads(REGISTRY_PATH.read_text()).get("policies", [])}
        status["stack_required"] = _policy_requirement(raw[cur]) if cur in raw else None
    except Exception:  # noqa: BLE001 -- the camera status must not fail on a registry problem
        status["stack_required"] = None
    return status


@router.websocket("/ws/height_map")
async def ws_height_map(websocket: WebSocket):
    if not await authenticate_websocket(websocket):
        return

    await websocket.accept()

    if state.stop_latched:
        await websocket.close(code=1013)
        return

    store = get_perception_store()

    async with store.lock:
        store.height_clients.add(websocket)
        initial = [m for m in (store.last_height_scan, store.last_grid_map) if m]

    try:
        for msg in initial:
            await websocket.send_text(json.dumps(msg))

        while True:
            _ = await websocket.receive_text()
            if state.stop_latched:
                await websocket.close(code=1013)
                return

    except Exception:
        pass

    finally:
        async with store.lock:
            store.height_clients.discard(websocket)


@router.websocket("/ws/cam_realsense")
async def ws_cam_realsense(websocket: WebSocket, stream: str = "depth"):
    if not await authenticate_websocket(websocket):
        return

    stream = (stream or "depth").lower()
    if stream not in ("depth", "color"):
        await websocket.close(code=1008)
        return

    await websocket.accept()

    if state.stop_latched:
        await websocket.close(code=1013)
        return

    store = get_perception_store()
    clients = store.depth_clients if stream == "depth" else store.color_clients

    async with store.lock:
        clients.add(websocket)
        initial_header = None
        initial_jpg = None
        cached = store.last_jpg.get(stream)
        if cached is not None:
            jpg, meta = cached
            initial_header = {
                "t": "cam", "stream": stream,
                "seq": store.seq_cam.get(stream, 0),
                "meta": meta, "n": len(jpg),
            }
            initial_jpg = jpg

    try:
        if initial_header:
            await websocket.send_text(json.dumps(initial_header))
            await websocket.send_bytes(initial_jpg)

        while True:
            _ = await websocket.receive_text()
            if state.stop_latched:
                await websocket.close(code=1013)
                return

    except Exception:
        pass

    finally:
        async with store.lock:
            clients.discard(websocket)
