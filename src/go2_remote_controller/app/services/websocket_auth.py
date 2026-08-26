"""

websocket_auth.py
WebSocket authentication service for the Go2 Remote Actions application.

Version 2.0
Author: Victor Lim

"""

import hmac
from fastapi import WebSocket
from app.core.config import AUTH_ENABLED, GO2_API_TOKEN

async def authenticate_websocket(websocket: WebSocket) -> bool:
    token = websocket.query_params.get("token", "")

    if not AUTH_ENABLED:
        return True

    if not GO2_API_TOKEN:
        await websocket.close(code=1011)
        return False

    if not hmac.compare_digest(token, GO2_API_TOKEN):
        await websocket.close(code=1008)
        return False

    return True