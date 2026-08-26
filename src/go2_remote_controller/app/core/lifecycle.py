"""

lifecycle.py
Lifecycle event handlers for the Go2 Remote Actions application.

Version 2.0
Author: Victor Lim

"""

import asyncio
from app.ros_bridge import start_ros_bridge, get_bridge
from app.core.state import state

async def on_startup():
    start_ros_bridge()
    get_bridge().set_asyncio_loop(asyncio.get_running_loop())

async def on_shutdown():
    state.shutting_down = True