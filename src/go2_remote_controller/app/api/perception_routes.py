"""
perception_routes.py
RealSense + height-map views for the RL web page:

  GET /perception/status            is a RealSense on USB (and USB 2 or 3), is the
                                    driver publishing, per-topic frame rates, grid info
  WS  /ws/height_map                JSON frames: {"t":"height_scan",...} (the policy's
                                    17x11 obs) and {"t":"grid_map",...} (GridMap layers)
  WS  /ws/cam_realsense?stream=     depth | color -- JSON header + JPEG bytes, the
                                    same wire format as /ws/cam_front

Same auth / stop-latch / keepalive shape as robot_model_routes.py and camera_routes.py.

Version: 1.0
Author: Victor Lim
"""

import json

from fastapi import APIRouter, Depends, WebSocket

from app.core.auth import require_token
from app.core.state import state
from app.services.realsense_usb import get_realsense_status
from app.services.websocket_auth import authenticate_websocket
from app.ros_bridge import get_perception_status, get_perception_store

router = APIRouter()


@router.get("/perception/status", dependencies=[Depends(require_token)])
async def perception_status():
    status = get_perception_status()
    status["realsense"] = get_realsense_status()
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
