import asyncio
import logging
import signal
from typing import Dict, List, Optional

from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel

# -----------------------------------------------------------------------------
# Logging setup
# -----------------------------------------------------------------------------
logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")
logger = logging.getLogger(__name__)

# -----------------------------------------------------------------------------
# Configuration: environment + command paths
#
# These are the *actual* commands you said work:
#   sit   -> go2_sport_client 3
#   stand -> go2_sport_client 4
#
# We use ABSOLUTE paths so the code does not depend on where you run the server.
# -----------------------------------------------------------------------------

# ROS distro environment
ROS_SETUP_BASH = "/opt/ros/foxy/setup.bash"

# Unitree workspaces on the Go2 (adjust if your paths differ)
UNITREE_ROOT = "/home/unitree/unitree_ros2"
CYCLONEDDS_WS_SETUP = f"{UNITREE_ROOT}/cyclonedds_ws/install/setup.bash"
UNITREE_MAIN_SETUP = f"{UNITREE_ROOT}/install/setup.bash"
EXAMPLE_SETUP = f"{UNITREE_ROOT}/example/install/setup.bash"

# Absolute path to the sport client built in the example workspace
GO2_SPORT_CLIENT = f"{UNITREE_ROOT}/example/install/unitree_ros2_example/bin/go2_sport_client"

# The action -> command mapping (only the final executable invocation goes here!)
COMMANDS: Dict[str, str] = {
    "sit": f"{GO2_SPORT_CLIENT} 3",
    "stand": f"{GO2_SPORT_CLIENT} 4",
}

# Commands to run before every action, in order.
# These ensure:
# - ROS is available
# - CycloneDDS / rmw env is present (if you built it)
# - Unitree packages (unitree_go libs) are on LD_LIBRARY_PATH
# - Example overlay is on top
COMMON_PRE_COMMANDS: List[str] = [
    f"source {ROS_SETUP_BASH}",
    f"source {CYCLONEDDS_WS_SETUP}",
    f"source {UNITREE_MAIN_SETUP}",
    f"source {EXAMPLE_SETUP}",
]

# -----------------------------------------------------------------------------
# API response model (FastAPI uses this for validation + docs)
# -----------------------------------------------------------------------------
class ActionStartResponse(BaseModel):
    action: str
    pid: int
    message: str


# -----------------------------------------------------------------------------
# CommandManager
#
# Starts/stops a small set of whitelisted actions by spawning shell processes.
# This is a "quick prototype" way to bridge HTTP -> robot actions.
# -----------------------------------------------------------------------------
class CommandManager:
    def __init__(self, commands: Dict[str, str], common_pre_commands: Optional[List[str]] = None):
        """
        commands:
            Mapping of action name -> shell command to execute (e.g. "sit" -> "/path/go2_sport_client 3")
        common_pre_commands:
            Shell commands that are executed before every action (e.g. 'source /opt/ros/foxy/setup.bash')
        """
        self._commands = commands
        self._common_pre_commands = common_pre_commands or []

        # Track currently running subprocesses by action name
        self._running: Dict[str, asyncio.subprocess.Process] = {}

    def _build_shell_command(self, action: str) -> str:
        """
        Build one shell string that:
          1) sources environment scripts
          2) runs the action command

        We join with '&&' so that if any step fails (e.g. missing setup.bash),
        the action does NOT run and we see the error in stderr.
        """
        if action not in self._commands:
            raise KeyError(f"Action '{action}' is not allowed")

        parts: List[str] = []
        parts.extend(self._common_pre_commands)
        parts.append(self._commands[action])
        return " && ".join(parts)

    async def start(self, action: str) -> asyncio.subprocess.Process:
        """
        Start an action as a subprocess.

        If the same action is already running, stop it first. This prevents
        multiple overlapping 'stand' processes, etc.
        """
        if action not in self._commands:
            raise KeyError(f"Action '{action}' is not allowed")

        # Stop previous process for this action if it exists
        await self.stop(action)

        cmd = self._build_shell_command(action)
        logger.info("Starting action '%s' with command:\n%s", action, cmd)

        # Use bash explicitly so "source ..." is guaranteed to work
        process = await asyncio.create_subprocess_shell(
            cmd,
            executable="/bin/bash",
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.PIPE,
            start_new_session=True,  # start a new process group (helps with stopping)
        )

        self._running[action] = process

        # Background task to stream logs
        asyncio.create_task(self._log_process_output(action, process))

        return process

    async def stop(self, action: str) -> None:
        """
        Stop a running action process, if present.

        We send SIGTERM first, wait a bit, then SIGKILL if needed.
        If the process already exited, this just cleans up tracking.
        """
        process = self._running.get(action)
        if not process:
            return

        # If still running, try to terminate cleanly
        if process.returncode is None:
            logger.info("Stopping action '%s' (pid=%s)", action, process.pid)
            process.send_signal(signal.SIGTERM)
            try:
                await asyncio.wait_for(process.wait(), timeout=5)
            except asyncio.TimeoutError:
                logger.warning("Action '%s' did not stop gracefully; killing", action)
                process.kill()
                await process.wait()

        # Remove from tracking dictionary
        self._running.pop(action, None)

    async def shutdown(self) -> None:
        """Stop all running actions (called when the server is shutting down)."""
        await asyncio.gather(*(self.stop(a) for a in list(self._running.keys())))

    async def _log_process_output(self, action: str, process: asyncio.subprocess.Process) -> None:
        """
        Stream stdout lines to the server logs.
        Collect stderr at the end and print it as a warning (helps debugging).
        """
        assert process.stdout is not None
        assert process.stderr is not None

        # Stream stdout live (line-by-line)
        while True:
            line = await process.stdout.readline()
            if not line:
                break
            logger.info("[%s][stdout] %s", action, line.decode(errors="replace").rstrip())

        # Read any remaining stderr output
        stderr_output = await process.stderr.read()
        if stderr_output:
            logger.warning("[%s][stderr] %s", action, stderr_output.decode(errors="replace").rstrip())

        await process.wait()
        logger.info("[%s] exited with code %s", action, process.returncode)


# -----------------------------------------------------------------------------
# FastAPI app setup
# -----------------------------------------------------------------------------
app = FastAPI(title="Go2 Remote Actions")

# CORS: for easy testing from a phone browser.
# For safety, you should eventually restrict allow_origins to your phone/laptop origin.
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

command_manager = CommandManager(commands=COMMANDS, common_pre_commands=COMMON_PRE_COMMANDS)


@app.on_event("shutdown")
async def on_shutdown() -> None:
    """Ensure we stop any running processes when the server exits."""
    await command_manager.shutdown()


@app.get("/health")
async def health() -> Dict[str, str]:
    """Simple health check endpoint."""
    return {"status": "ok"}


@app.post("/actions/{action}", response_model=ActionStartResponse)
async def start_action(action: str) -> ActionStartResponse:
    """
    Start a whitelisted action.
    Example:
      POST /actions/sit
      POST /actions/stand
    """
    try:
        process = await command_manager.start(action)
    except KeyError as exc:
        raise HTTPException(status_code=404, detail=str(exc))
    except Exception as exc:
        logger.exception("Failed to start action '%s'", action)
        raise HTTPException(status_code=500, detail=str(exc))

    return ActionStartResponse(
        action=action,
        pid=process.pid,
        message="Action started; check server logs for output",
    )


@app.post("/actions/{action}/stop")
async def stop_action(action: str) -> Dict[str, str]:
    """
    Stop a running action (if any).
    Example:
      POST /actions/sit/stop
    """
    try:
        await command_manager.stop(action)
    except Exception as exc:
        logger.exception("Failed to stop action '%s'", action)
        raise HTTPException(status_code=500, detail=str(exc))

    return {"action": action, "status": "stopped"}
