import asyncio
import logging
import signal
from typing import Dict

from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel


logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")
logger = logging.getLogger(__name__)


# Fixed configuration for ROS environment and action commands.
DEFAULT_ROS_SETUP_PATH = "/opt/ros/humble/setup.bash"
DEFAULT_SIT_COMMAND = "ros2 launch your_pkg sit.launch.py"
DEFAULT_STAND_COMMAND = "ros2 launch your_pkg stand.launch.py"


# Fixed configuration for ROS environment and action commands.
DEFAULT_ROS_SETUP_PATH = "/opt/ros/humble/setup.bash"
DEFAULT_SIT_COMMAND = "ros2 launch your_pkg sit.launch.py"
DEFAULT_STAND_COMMAND = "ros2 launch your_pkg stand.launch.py"


# Fixed configuration for ROS environment and action commands.
DEFAULT_ROS_SETUP_PATH = "/opt/ros/humble/setup.bash"
DEFAULT_SIT_COMMAND = "ros2 launch your_pkg sit.launch.py"
DEFAULT_STAND_COMMAND = "ros2 launch your_pkg stand.launch.py"


class ActionStartResponse(BaseModel):
    action: str
    pid: int
    message: str


class CommandManager:
    """Manage allowed robot actions that run as system commands.

    The commands are executed via the shell so ROS 2 launch or action helpers can
    be used (e.g., `ros2 launch ...`). For long-running launches, processes are
    tracked so they can be stopped when the server shuts down.
    """

    def __init__(self, commands: Dict[str, str], ros_setup_path: str | None = None):
        self._commands = commands
        self._ros_setup_path = ros_setup_path
        self._running: Dict[str, asyncio.subprocess.Process] = {}

    def _build_command(self, action: str) -> str:
        base_command = self._commands[action]
        if self._ros_setup_path:
            return f"source {self._ros_setup_path} && {base_command}"
        return base_command

    async def start(self, action: str) -> asyncio.subprocess.Process:
        if action not in self._commands:
            raise KeyError(f"Action '{action}' is not allowed")

        command = self._build_command(action)
        logger.info("Starting action '%s' with command: %s", action, command)
        process = await asyncio.create_subprocess_shell(
            command,
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.PIPE,
            start_new_session=True,
        )

        self._running[action] = process
        asyncio.create_task(self._log_process_output(action, process))
        return process

    async def stop(self, action: str) -> None:
        process = self._running.get(action)
        if process and process.returncode is None:
            logger.info("Stopping action '%s' (pid=%s)", action, process.pid)
            process.send_signal(signal.SIGTERM)
            try:
                await asyncio.wait_for(process.wait(), timeout=10)
            except asyncio.TimeoutError:
                logger.warning("Action '%s' did not stop gracefully; killing", action)
                process.kill()
                await process.wait()
        self._running.pop(action, None)

    async def shutdown(self) -> None:
        await asyncio.gather(*(self.stop(action) for action in list(self._running.keys())))

    async def _log_process_output(self, action: str, process: asyncio.subprocess.Process) -> None:
        assert process.stdout and process.stderr
        while True:
            line = await process.stdout.readline()
            if not line:
                break
            logger.info("[%s][stdout] %s", action, line.decode().rstrip())
        stderr_output = await process.stderr.read()
        if stderr_output:
            logger.warning("[%s][stderr] %s", action, stderr_output.decode().rstrip())
        await process.wait()


def build_command_manager() -> CommandManager:
    ros_setup_path = DEFAULT_ROS_SETUP_PATH
    commands = {
        "sit": DEFAULT_SIT_COMMAND,
        "stand": DEFAULT_STAND_COMMAND,
    }
    return CommandManager(commands=commands, ros_setup_path=ros_setup_path)


app = FastAPI(title="Go2 Remote Actions")
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

command_manager = build_command_manager()


@app.on_event("shutdown")
async def _on_shutdown() -> None:
    await command_manager.shutdown()


@app.get("/health")
async def health() -> dict[str, str]:
    return {"status": "ok"}


@app.post("/actions/{action}", response_model=ActionStartResponse)
async def start_action(action: str) -> ActionStartResponse:
    try:
        process = await command_manager.start(action)
    except KeyError as exc:  # action not allowed
        raise HTTPException(status_code=404, detail=str(exc))
    except Exception as exc:  # unexpected errors launching the command
        logger.exception("Failed to start action '%s'", action)
        raise HTTPException(status_code=500, detail=str(exc))

    return ActionStartResponse(
        action=action,
        pid=process.pid,
        message="Action started; monitor logs for progress",
    )


@app.post("/actions/{action}/stop")
async def stop_action(action: str) -> dict[str, str]:
    try:
        await command_manager.stop(action)
    except Exception as exc:
        logger.exception("Failed to stop action '%s'", action)
        raise HTTPException(status_code=500, detail=str(exc))
    return {"action": action, "status": "stopped"}
