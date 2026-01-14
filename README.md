# P2RemoteConnection

Creating a web access interface to connect to Go2 functions.

## FastAPI backend (prototype)

This project runs a small FastAPI server on the Unitree Go2 (or a machine with the same ROS 2/Unitree environment)
and exposes **HTTP endpoints** that trigger a **whitelisted set of shell commands**.

It’s designed for quick experiments: phone → HTTP → Go2 command.

### How it works
- `GET /health`  
  Basic connectivity check (returns `{"status":"ok"}`)

- `POST /actions/{action}`  
  Runs a pre-defined command for that action (example: `sit`, `stand`)

- `POST /actions/{action}/stop`  
  Stops the currently-running process for that action (SIGTERM, then SIGKILL if needed)

You can explore and test everything using FastAPI’s built-in UI:

- `http://<go2-ip>:8000/docs`

### Configure the commands
Set environment variables so the backend can call your actual ROS 2 launch
files or helpers:

```bash
export ROS_SETUP_PATH=/opt/ros/humble/setup.bash
export SIT_COMMAND="ros2 launch your_pkg sit.launch.py"
export STAND_COMMAND="ros2 launch your_pkg stand.launch.py"
```

### Run the server

On the Go2 (or the device running the server)
```bash
pip3 install fastapi uvicorn
pip install -r requirements.txt
cd ~/P2/P2RemoteConnection
uvicorn app.main:app --host 0.0.0.0 --port 8000
```

In a seperate terminal, run the html: 
```bash
cd ~/P2/P2RemoteConnection/app
python3 -m http.server 8081
```

Open the website on a device connected to the same wifi:

<device_ip>::8081/go2_remote.html

## Robot Commands

Commands live in the COMMANDS dictionary in app/main.py:
```bash
COMMANDS: Dict[str, str] = {
    "sit":   f"{GO2_SPORT_CLIENT} 3",
    "stand": f"{GO2_SPORT_CLIENT} 4",
}
```
Each key is the action name used in the URL, and each value is the shell command that runs on the Go2.
Example API call:

  curl -X POST http://<go2-ip>:8000/actions/stand

### Adding a new command:

**1) Ensure the command works manually in a Go2 terminal**
Example:

  /home/unitree/unitree_ros2/example/install/unitree_ros2_example/bin/go2_sport_client 5

**2) Add it to COMMANDS**
Open app/main.py and add a new entry:
```bash
COMMANDS: Dict[str, str] = {
    "sit":   f"{GO2_SPORT_CLIENT} 3",
    "stand": f"{GO2_SPORT_CLIENT} 4",

    # New action example:
    "my_new_action": f"{GO2_SPORT_CLIENT} 5",
}
```
**Tips**
- Keep commands absolute paths where possible.
- Keep them static (don’t take arbitrary user input) to avoid unsafe shell execution.

**3) Restart the server**
Stop the running uvicorn (Ctrl+C) and run again:
```bash
  uvicorn app.main:app --host 0.0.0.0 --port 8000
```

**4) Call the new action**

  curl -X POST http://<go2-ip>:8000/actions/my_new_action

**Adding to the source**
This project runs these scripts before every action (see COMMON_PRE_COMMANDS):
```bash
COMMON_PRE_COMMANDS = [
    f"source {ROS_SETUP_BASH}",
    f"source {CYCLONEDDS_WS_SETUP}",
    f"source {UNITREE_MAIN_SETUP}",
    f"source {EXAMPLE_SETUP}",
]
```
If you add a command that needs extra setup, either:
add another source ... line here, or
make your command a script that handles its own setup.

### Example calls
```bash
# Health check
curl http://localhost:8000/health
# Stand / Sit
curl -X POST http://<go2-ip>:8000/actions/stand
curl -X POST http://<go2-ip>:8000/actions/sit

# Stop (if needed)
curl -X POST http://<go2-ip>:8000/actions/stand/stop
curl -X POST http://<go2-ip>:8000/actions/sit/stop
```