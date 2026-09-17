"""
robot_model_routes.py
API endpoint streaming live joint/pose telemetry for the 3D robot viewer.

Version: 1.0
Author: Victor Lim
"""

from fastapi import APIRouter, WebSocket
import json

from app.core.state import state
from app.services.websocket_auth import authenticate_websocket
from app.ros_bridge import get_robot_pose_store

router = APIRouter()


@router.websocket("/ws/robot_pose")
async def ws_robot_pose(websocket: WebSocket):
    if not await authenticate_websocket(websocket):
        return

    await websocket.accept()

    if state.stop_latched:
        await websocket.close(code=1013)
        return

    store = get_robot_pose_store()

    async with store.lock:
        store.clients.add(websocket)
        initial = None
        if store.last_joints is not None or store.last_base is not None:
            initial = {
                "t": "pose",
                "seq": store.seq,
                "joints": store.last_joints,
                "base": store.last_base,
            }

    try:
        if initial:
            await websocket.send_text(json.dumps(initial))

        while True:
            _ = await websocket.receive_text()
            if state.stop_latched:
                await websocket.close(code=1013)
                return

    except Exception:
        pass

    finally:
        async with store.lock:
            store.clients.discard(websocket)
