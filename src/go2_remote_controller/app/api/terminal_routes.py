"""

terminal_routes.py
This module defines the API endpoints for terminal access in the Go2 Remote Actions application.

Version: 2.0
Author: Victor Lim

"""

import os
from fastapi import APIRouter, WebSocket

router = APIRouter()

if os.getenv("DEPLOYMENT_ENV") == "ec2":
    from app.terminal_ec2 import terminal_ws
else:
    from app.terminal import terminal_ws

@router.websocket("/ws/terminal")
async def ws_terminal(websocket: WebSocket):
    await terminal_ws(websocket)