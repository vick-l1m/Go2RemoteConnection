import os
import pty
import asyncio
import json
import fcntl
import struct
import termios
import hmac
from fastapi import WebSocket, WebSocketDisconnect

GO2_API_TOKEN = os.getenv("GO2_API_TOKEN", "")

class TerminalSession:
    def __init__(self):
        self.pid, self.fd = pty.fork()

        if self.pid == 0:
            # Child process - simplified for EC2 (no robot-specific paths)
            os.environ["TERM"] = "xterm-256color"
            os.environ["COLORTERM"] = "truecolor"
            os.environ["LANG"] = os.environ.get("LANG", "C.UTF-8")
            os.environ["LC_ALL"] = os.environ.get("LC_ALL", "C.UTF-8")

            # Simple interactive bash - works on EC2
            os.execvp("bash", ["bash", "-i"])

    def resize(self, rows: int, cols: int):
        winsz = struct.pack("HHHH", rows, cols, 0, 0)
        fcntl.ioctl(self.fd, termios.TIOCSWINSZ, winsz)

    async def read_loop(self, websocket: WebSocket):
        loop = asyncio.get_event_loop()
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
    # Authenticate BEFORE accept
    token = websocket.query_params.get("token", "")

    if not GO2_API_TOKEN:
        await websocket.close(code=1011)
        return

    # constant-time compare
    if not hmac.compare_digest(token, GO2_API_TOKEN):
        await websocket.close(code=1008)
        return

    await websocket.accept()

    term = TerminalSession()

    # Set a sane default size immediately (helps clear + prompt wrapping)
    term.resize(rows=30, cols=120)

    reader = asyncio.create_task(term.read_loop(websocket))

    try:
        while True:
            msg = await websocket.receive_text()

            # Handle resize JSON control message
            if msg and msg[:1] == "{":
                try:
                    obj = json.loads(msg)
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
            os.close(term.fd)
        except Exception:
            pass
