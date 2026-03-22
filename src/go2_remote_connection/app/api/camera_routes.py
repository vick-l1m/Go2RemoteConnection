"""

camera_routes.py
This module defines the API endpoints for camera streams in the Go2 Remote Actions application.

Version: 2.0
Author: Victor Lim

"""

from fastapi import APIRouter, WebSocket
import json

from app.core.state import state
from app.services.websocket_auth import authenticate_websocket
from app.ros_bridge import get_cam_store, get_yolo_cam_store

router = APIRouter()


@router.websocket("/ws/cam_front")
async def ws_cam_front(websocket: WebSocket):
    if not await authenticate_websocket(websocket):
        return

    await websocket.accept()

    if state.stop_latched:
        await websocket.close(code=1013)
        return

    store = get_cam_store()

    async with store.lock:
        store.clients.add(websocket)
        initial_header = None
        initial_jpg = None

        if store.meta is not None and store.jpg is not None:
            initial_header = {
                "t": "cam",
                "seq": store.seq,
                "meta": store.meta,
                "n": len(store.jpg),
            }
            initial_jpg = store.jpg

    try:
        if initial_header:
            await websocket.send_text(json.dumps(initial_header))
            await websocket.send_bytes(initial_jpg)

        while True:
            _ = await websocket.receive_text()  # keepalive ping from client
            if state.stop_latched:
                await websocket.close(code=1013)
                return

    except Exception:
        pass

    finally:
        async with store.lock:
            store.clients.discard(websocket)


@router.websocket("/ws/cam_yolo")
async def ws_cam_yolo(websocket: WebSocket):
    if not await authenticate_websocket(websocket):
        return

    await websocket.accept()

    if state.stop_latched:
        await websocket.close(code=1013)
        return

    store = get_yolo_cam_store()

    async with store.lock:
        store.clients.add(websocket)
        initial_header = None
        initial_jpg = None

        if store.meta is not None and store.jpg is not None:
            initial_header = {
                "t": "cam",
                "seq": store.seq,
                "meta": store.meta,
                "n": len(store.jpg),
            }
            initial_jpg = store.jpg

    try:
        if initial_header:
            await websocket.send_text(json.dumps(initial_header))
            await websocket.send_bytes(initial_jpg)

        while True:
            _ = await websocket.receive_text()  # keepalive ping from client
            if state.stop_latched:
                await websocket.close(code=1013)
                return

    except Exception:
        pass

    finally:
        async with store.lock:
            store.clients.discard(websocket)