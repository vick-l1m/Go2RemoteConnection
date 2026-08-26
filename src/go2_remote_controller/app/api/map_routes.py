"""

map_routes.py
This module defines the API endpoints for map data in the Go2 Remote Actions application.

Version: 2.0
Author: Victor Lim

"""

import gzip
import json
from fastapi import APIRouter, Depends, Response, WebSocket
from app.core.auth import require_token
from app.core.state import state
from app.services.websocket_auth import authenticate_websocket
from app.ros_bridge import get_map_store

router = APIRouter()

@router.get("/map2d/full")
async def map2d_full(_=Depends(require_token)):
    store = get_map_store()
    async with store.lock:
        if store.meta is None or store.full_raw is None:
            return Response(status_code=204)

        meta = store.meta
        gz = gzip.compress(store.full_raw, compresslevel=6)

        headers = {
            "X-Map-Frame": meta["frame_id"],
            "X-Map-Resolution": str(meta["resolution"]),
            "X-Map-Width": str(meta["width"]),
            "X-Map-Height": str(meta["height"]),
            "X-Map-Origin-X": str(meta["origin_x"]),
            "X-Map-Origin-Y": str(meta["origin_y"]),
            "X-Map-Seq": str(store.seq),
            "X-Map-Encoding": "gzip",
            "Content-Type": "application/octet-stream",
        }
        return Response(content=gz, headers=headers)

@router.websocket("/ws/map2d")
async def ws_map2d(websocket: WebSocket):
    if not await authenticate_websocket(websocket):
        return

    await websocket.accept()

    if state.stop_latched:
        await websocket.close(code=1013)
        return

    store = get_map_store()

    async with store.lock:
        store.clients.add(websocket)
        initial = None
        if store.meta is not None and store.full_raw is not None:
            initial = {
                "t": "f",
                "seq": store.seq,
                "meta": store.meta,
                "gz": gzip.compress(store.full_raw, compresslevel=6),
            }

    try:
        if initial:
            header = initial.copy()
            gz = header.pop("gz")
            await websocket.send_text(json.dumps(header))
            await websocket.send_bytes(gz)

        while True:
            await websocket.receive_text()
            if state.stop_latched:
                await websocket.close(code=1013)
                return
    except Exception:
        pass
    finally:
        async with store.lock:
            store.clients.discard(websocket)