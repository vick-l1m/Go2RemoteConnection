"""

yolo_routes.py
This module defines the API endpoints for YOLO functionality in the Go2 Remote Actions application.

Version: 2.0
Author: Victor Lim

"""

import json

from fastapi import APIRouter, Depends, HTTPException, WebSocket

from app.core.auth import require_token
from app.core.state import state
from app.services.websocket_auth import authenticate_websocket
from app.services.yolo_service import (
    start_yolo_process,
    stop_yolo_process,
    yolo_status as get_yolo_process_status,
)
from app.ros_bridge import get_yolo_store

router = APIRouter()


@router.websocket("/ws/yolo")
async def ws_yolo(websocket: WebSocket):
    if not await authenticate_websocket(websocket):
        return

    await websocket.accept()

    if state.stop_latched:
        await websocket.close(code=1013)
        return

    store = get_yolo_store()

    async with store.lock:
        store.clients.add(websocket)
        initial = store.last_json

    try:
        if initial:
            if isinstance(initial, (dict, list)):
                await websocket.send_text(json.dumps(initial))
            else:
                await websocket.send_text(initial)

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


@router.post("/yolo/start")
def start_yolo(_=Depends(require_token)):
    if state.stop_latched:
        raise HTTPException(status_code=423, detail="STOP latched: YOLO start disabled")
    return start_yolo_process()


@router.post("/yolo/stop")
def stop_yolo(_=Depends(require_token)):
    return stop_yolo_process()


@router.get("/yolo/status")
def yolo_status(_=Depends(require_token)):
    return get_yolo_process_status()