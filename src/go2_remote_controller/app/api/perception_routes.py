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
  POST /perception/photo_mode/on|off  switch the colour stream on/off in the RUNNING
                                    driver (enable_color) to take a picture on the USB 2
                                    depth-only launch. Engaging RL is refused while on
                                    (the USB 2 colour path can stall depth or drop the
                                    camera off the bus, see real_perception_usb2.launch.py)
  POST /perception/photo_memory/{s|policy}  rolling-map memory for the picture (photo
                                    mode only; 0.5-60 s, "policy" restores the policy's)
  GET  /perception/photo.jpg        latest colour frame at full resolution
  POST /perception/scan_snap/{auto|on|off}  height-map snap switch: follow the policy's
                                    deploy.yaml, or force snapped / raw heights (relayed to
                                    go2_rl_bridge_node). Refused while RL is engaged
  POST /perception/stepfield_preset/{name}  stepfield_map_node terrain_preset, live
                                    (lab_blocks / half_cubic / full_cubic / grid_pallet).
                                    Refused while RL is engaged

Same auth / stop-latch / keepalive shape as robot_model_routes.py and camera_routes.py.

Version: 1.0
Author: Victor Lim
"""

import asyncio
import json
import pathlib
import time

import yaml
from fastapi import APIRouter, Depends, HTTPException, Response, WebSocket

from app.core.auth import require_token
from app.core.state import state
from app.services.realsense_usb import get_realsense_status
from app.services.websocket_auth import authenticate_websocket
from app.ros_bridge import get_bridge, get_perception_status, get_perception_store
from app.services.perception_codec import HAVE_CV2, color_image_to_jpeg
from app.api.rl_policy import REGISTRY_PATH, _current_id, _load_registry

router = APIRouter()

#: stepfield_map_node presets offered on the page (go2_perception/stepfield_presets.py).
STEPFIELD_PRESETS = ("lab_blocks", "half_cubic", "full_cubic", "grid_pallet")

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
    status["scan_snap_mode"] = state.scan_snap_mode
    status["stepfield_presets"] = list(STEPFIELD_PRESETS)
    try:
        default_id, policies = _load_registry()
        cur = _current_id(policies, default_id)
        raw = {str(e.get("id")): e for e in json.loads(REGISTRY_PATH.read_text()).get("policies", [])}
        status["stack_required"] = _policy_requirement(raw[cur]) if cur in raw else None
    except Exception:  # noqa: BLE001 -- the camera status must not fail on a registry problem
        status["stack_required"] = None
    return status


_photo_lock = asyncio.Lock()


@router.post("/perception/photo_mode/{onoff}", dependencies=[Depends(require_token)])
async def set_photo_mode(onoff: str):
    onoff = onoff.strip().lower()
    if onoff not in ("on", "off"):
        raise HTTPException(status_code=400, detail="use /perception/photo_mode/on or /off")
    bridge = get_bridge()
    async with _photo_lock:
        if onoff == "on":
            if state.photo_mode:
                return {"ok": True, "photo_mode": True}
            if state.control_mode == "rl":
                raise HTTPException(status_code=423, detail="stop RL before turning photo mode on")
            cur = await asyncio.to_thread(bridge.get_camera_bools, ["enable_color"])
            if cur is None:
                raise HTTPException(status_code=409, detail="RealSense driver (/go2/camera) not reachable")
            if cur["enable_color"]:
                # USB 3 launch: colour is already part of the normal stack, nothing to switch.
                return {"ok": True, "photo_mode": False, "color_already_on": True}
            if state.control_mode == "rl":       # re-check: RL may have engaged during the await
                raise HTTPException(status_code=423, detail="stop RL before turning photo mode on")
            state.photo_mode = True              # block RL BEFORE touching the camera
            ok, why = await asyncio.to_thread(bridge.set_camera_bools, {"enable_color": True})
            if not ok:
                ok_back, _ = await asyncio.to_thread(bridge.set_camera_bools, {"enable_color": False})
                if ok_back:
                    state.photo_mode = False
                raise HTTPException(status_code=502, detail=f"could not enable colour: {why}")
            return {"ok": True, "photo_mode": True}

        if not state.photo_mode:
            return {"ok": True, "photo_mode": False}
        state.photo_memory_s = None
        bridge.publish_map_memory_override(None)
        ok, why = await asyncio.to_thread(bridge.set_camera_bools, {"enable_color": False})
        # An unreachable driver streams nothing, so it is safe to clear; a driver that
        # refused keeps colour on, so RL stays blocked.
        if not ok and why != "unreachable":
            raise HTTPException(status_code=502, detail=f"could not disable colour (RL stays blocked): {why}")
        state.photo_mode = False
        return {"ok": True, "photo_mode": False}


@router.post("/perception/photo_memory/{value}", dependencies=[Depends(require_token)])
async def set_photo_memory(value: str):
    if not state.photo_mode:
        raise HTTPException(status_code=423, detail="turn photo mode on first (RL must not run on a changed memory)")
    if value.strip().lower() == "policy":
        seconds = None
    else:
        try:
            seconds = float(value)
        except ValueError:
            raise HTTPException(status_code=400, detail="memory must be seconds or 'policy'")
        if not 0.5 <= seconds <= 60.0:
            raise HTTPException(status_code=400, detail="memory must be 0.5-60 s")
    state.photo_memory_s = seconds
    get_bridge().publish_map_memory_override(seconds)
    return {"ok": True, "photo_memory_s": seconds}


@router.post("/perception/scan_snap/{mode}", dependencies=[Depends(require_token)])
async def set_scan_snap(mode: str):
    mode = mode.strip().lower()
    if mode not in ("auto", "on", "off"):
        raise HTTPException(status_code=400, detail="use /perception/scan_snap/auto, /on or /off")
    if mode == state.scan_snap_mode:
        return {"ok": True, "scan_snap_mode": mode}
    if state.control_mode == "rl":
        # The switch rebuilds the rolling map: never under a walking policy.
        raise HTTPException(status_code=423, detail="stop RL before changing snapping")
    state.scan_snap_mode = mode
    get_bridge().publish_scan_snap_override(mode)
    return {"ok": True, "scan_snap_mode": mode}


@router.post("/perception/stepfield_preset/{name}", dependencies=[Depends(require_token)])
async def set_stepfield_preset(name: str):
    name = name.strip().lower()
    if name not in STEPFIELD_PRESETS:
        raise HTTPException(status_code=400, detail=f"preset must be one of {', '.join(STEPFIELD_PRESETS)}")
    if state.control_mode == "rl":
        raise HTTPException(status_code=423, detail="stop RL before changing the step preset")
    ok, why = await asyncio.to_thread(get_bridge().set_stepfield_preset, name)
    if not ok:
        if why == "unreachable":
            raise HTTPException(status_code=409, detail="stepfield_map_node not running (GO2_HEIGHTMAP_MODE=stepfield)")
        raise HTTPException(status_code=502, detail=f"preset not set: {why}")
    return {"ok": True, "preset": name}


@router.get("/perception/photo.jpg", dependencies=[Depends(require_token)])
async def photo_jpg():
    store = get_perception_store()
    msg = store.last_color_msg
    if not HAVE_CV2:
        raise HTTPException(status_code=503, detail="cv2 not available on the robot")
    if msg is None or not store.rates["color"].alive():
        raise HTTPException(status_code=404, detail="no colour stream (turn photo mode on)")
    jpg = await asyncio.to_thread(color_image_to_jpeg, bytes(msg.data), msg.height, msg.width,
                                  msg.encoding, msg.step, int(msg.width), 92)
    if jpg is None:
        raise HTTPException(status_code=500, detail=f"cannot encode colour frame ({msg.encoding})")
    name = time.strftime("realsense_%Y%m%d-%H%M%S.jpg")
    return Response(content=jpg, media_type="image/jpeg",
                    headers={"Content-Disposition": f'attachment; filename="{name}"',
                             "Cache-Control": "no-store"})


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
