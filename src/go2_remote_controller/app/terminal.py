"""
terminal.py

Terminal backend implementation for the Go2 web UI.
This module manages PTY-backed shell sessions and exposes a websocket handler
used by the terminal route.

Version 2.0
Author: Victor Lim
"""

import os
import pty
import json
import fcntl
import signal
import struct
import termios
import asyncio

from fastapi import WebSocket, WebSocketDisconnect

from app.services.websocket_auth import authenticate_websocket


class TerminalSession:
    def __init__(self):
        self.pid, self.fd = pty.fork()

        if self.pid == 0:
            os.environ["TERM"] = "xterm-256color"
            os.environ["COLORTERM"] = "truecolor"
            os.environ["LANG"] = os.environ.get("LANG", "C.UTF-8")
            os.environ["LC_ALL"] = os.environ.get("LC_ALL", "C.UTF-8")

            os.execvp("bash", [
                "bash",
                "-lc",
                r"""
                if [ -f /opt/ros/foxy/setup.bash ]; then
                  source /opt/ros/foxy/setup.bash
                elif [ -f /opt/ros/humble/setup.bash ]; then
                  source /opt/ros/humble/setup.bash
                fi

                if [ -f ~/unitree_ros2/install/setup.sh ]; then
                  source ~/unitree_ros2/install/setup.sh
                elif [ -f ~/unitree_ros2/install/setup.bash ]; then
                  source ~/unitree_ros2/install/setup.bash
                fi

                if [ -f ~/go2_ws/Go2RemoteConnection/install/setup.bash ]; then
                  source ~/go2_ws/Go2RemoteConnection/install/setup.bash
                fi

                alias ls='ls --color=auto' 2>/dev/null || true
                cd ~/go2_ws 2>/dev/null || cd ~

                exec bash -i
                """
            ])

    def resize(self, rows: int, cols: int):
        winsz = struct.pack("HHHH", rows, cols, 0, 0)
        fcntl.ioctl(self.fd, termios.TIOCSWINSZ, winsz)

    async def read_loop(self, websocket: WebSocket):
        loop = asyncio.get_running_loop()
        try:
            while True:
                data = await loop.run_in_executor(None, os.read, self.fd, 4096)
                if not data:
                    break
                await websocket.send_text(data.decode(errors="ignore"))
        except Exception:
            pass

    async def write(self, data: str):
        os.write(self.fd, data.encode())


async def terminal_ws(websocket: WebSocket):
    if not await authenticate_websocket(websocket):
        return

    await websocket.accept()

    term = TerminalSession()
    term.resize(rows=30, cols=120)

    reader = asyncio.create_task(term.read_loop(websocket))

    try:
        while True:
            msg = await websocket.receive_text()

            if msg and msg[:1] == "{":
                try:
                    obj = json.loads(msg)

                    if obj.get("kind") == "resize":
                        cols = int(obj.get("cols", 120))
                        rows = int(obj.get("rows", 30))
                        cols = max(20, min(cols, 400))
                        rows = max(5, min(rows, 200))
                        term.resize(rows=rows, cols=cols)
                        continue

                    if "resize" in obj:
                        cols = int(obj["resize"].get("cols", 120))
                        rows = int(obj["resize"].get("rows", 30))
                        cols = max(20, min(cols, 400))
                        rows = max(5, min(rows, 200))
                        term.resize(rows=rows, cols=cols)
                        continue
                except Exception:
                    pass

            await term.write(msg)

    except WebSocketDisconnect:
        reader.cancel()
    finally:
        try:
            reader.cancel()
        except Exception:
            pass

        try:
            os.kill(term.pid, signal.SIGHUP)
        except Exception:
            pass

        try:
            os.close(term.fd)
        except Exception:
            pass