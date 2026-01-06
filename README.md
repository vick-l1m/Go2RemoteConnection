# P2RemoteConnection

Creating a web access interface to connect to Go2 functions.

## FastAPI backend (prototype)

This backend runs on the robot (or an edge box on the same network) and exposes
HTTP endpoints to start or stop curated actions. Two example actions are wired:
`sit` and `stand`. Each action maps to a shell command so you can run ROS 2
launch files without exposing arbitrary shell access.

### How it works
- A **CommandManager** keeps an allowlist of actions → commands.
- On `POST /actions/{action}`, the manager starts the configured command in a
  new session and returns the process PID immediately. Output is logged to the
  server logs.
- On `POST /actions/{action}/stop`, the manager sends SIGTERM (and SIGKILL if
  needed) to stop the process.
- If `ROS_SETUP_PATH` is set, the server sources that script before running any
  command (e.g., `export ROS_SETUP_PATH=/opt/ros/humble/setup.bash`).

### Configure the commands
Paths are fixed in code for simplicity. Edit `app/main.py` to point to your
actual ROS setup script and launch files:

- `DEFAULT_ROS_SETUP_PATH` (default: `/opt/ros/humble/setup.bash`)
- `DEFAULT_SIT_COMMAND` (default: `ros2 launch your_pkg sit.launch.py`)
- `DEFAULT_STAND_COMMAND` (default: `ros2 launch your_pkg stand.launch.py`)

### Run the server
```bash
pip install -r requirements.txt
uvicorn app.main:app --host 0.0.0.0 --port 8000
```

### Example calls
```bash
# Start sit action
curl -X POST http://localhost:8000/actions/sit

# Stop sit action
curl -X POST http://localhost:8000/actions/sit/stop

# Health check
curl http://localhost:8000/health
```

### Connecting to the robot
Run the server directly on the Go2 (or a nearby edge computer that has network
access to the robot and ROS 2 environment available). A web frontend can call
these endpoints (and a WebSocket terminal later) to trigger the sit/stand
launch files safely through this allowlist-based API.
