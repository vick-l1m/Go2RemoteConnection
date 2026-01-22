import os
import pty
import asyncio
import json
import fcntl
import struct
import termios
from fastapi import WebSocket, WebSocketDisconnect

class TerminalSession:
    def __init__(self):
        self.pid, self.fd = pty.fork()

        if self.pid == 0:
            os.execvp("bash", [
                "bash",
                "-lc",
                r"""
            if [ -f /opt/ros/foxy/setup.bash ]; then
              source /opt/ros/foxy/setup.bash
            elif [ -f /opt/ros/humble/setup.bash ]; then
              source /opt/ros/humble/setup.bash
            else
              echo "❌ No ROS 2 setup.bash found in /opt/ros"
            fi
            
            # Source Unitree + your overlay (use the correct paths!)
            if [ -f ~/unitree_ros2/install/setup.sh ]; then
              source ~/unitree_ros2/install/setup.sh
            elif [ -f ~/unitree_ros2/install/setup.bash ]; then
              source ~/unitree_ros2/install/setup.bash
            fi
            
            # IMPORTANT: this path in your snippet is wrong — don't use src/.../install/...
            if [ -f ~/p2_ws/P2RemoteConnection/install/setup.bash ]; then
              source ~/p2_ws/P2RemoteConnection/install/setup.bash
            fi
            
            exec bash
            """
            ])

    def resize(self, rows: int, cols: int):
        # winsize: rows, cols, xpix, ypix
        winsz = struct.pack("HHHH", rows, cols, 0, 0)
        fcntl.ioctl(self.fd, termios.TIOCSWINSZ, winsz)

    async def read_loop(self, websocket: WebSocket):
        try:
            loop = asyncio.get_event_loop()
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
    await websocket.accept()
    term = TerminalSession()
    reader = asyncio.create_task(term.read_loop(websocket))

    try:
        while True:
            msg = await websocket.receive_text()

            # Try parse as JSON control message (resize)
            if msg and msg[0] == "{":
                try:
                    obj = json.loads(msg)
                    if "resize" in obj:
                        cols = int(obj["resize"].get("cols", 80))
                        rows = int(obj["resize"].get("rows", 24))
                        cols = max(20, min(cols, 400))
                        rows = max(5, min(rows, 200))
                        term.resize(rows=rows, cols=cols)
                        continue
                except Exception:
                    # Not valid JSON; treat as normal input
                    pass

            await term.write(msg)
    except WebSocketDisconnect:
        reader.cancel()
    finally:
        try:
            os.close(term.fd)
        except Exception:
            pass
