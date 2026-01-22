# P2RemoteConnection

Creating a web access interface to connect to Go2 functions.

## FastAPI backend (prototype)

This project runs a small FastAPI server on the Unitree Go2 (or a machine with the same ROS 2/Unitree environment)
and exposes **HTTP endpoints** that trigger a **whitelisted set of shell commands**.

It’s designed for quick experiments: phone → HTTP → Go2 command.


### Build the workspace

**1. Clone this workspace**
```bash
  git clone https://github.com/vick-l1m/P2RemoteConnection.git
```

On the Go2 (or the device running the server)
```bash
cd ~/P2RemoteConnection/src/p2_remote_connection
pip3 install fastapi uvicorn
pip install -r requirements.txt
source ~/unitree_ros2/install/setup.sh
cd ~/P2RemoteConnection
colcon build
source install/setup.bash
```

### Launch the backend, webpage and ros2 node
Make the command runnable and launch:
```bash
cd ~/P2RemoteConnection
chmod +x start_remote_connection.sh
./start_remote_connection.sh
```

To run on the Issac Sim:
```bash
use_fastrtps
cd ~/P2RemoteConnection
chmod +x start_remote_connection_humble.sh
./start_remote_connection_humble.sh
```

Open the website on a device connected to the same wifi:
Check the device ip address:

```bash
hostname -I
```
Then search up the website:
```bash
<device_ip>::8081/go2_joystick.html

go2: 192.168.123.18/go2_joystick.html
```

## Robot Commands

Each key is the action name used in the URL, and each value is the shell command that runs on the Go2.
Example API call:

  curl -X POST http://<go2-ip>:8000/actions/stand


### How it works
The start up script runs 3 seperate process':

**The backend (FastAPI)**:
```bash
python3 -m uvicorn app.main:app --host 0.0.0.0 --port 8000
```
**The front (static webpage)**:
```bash
python3 -m http.server 8081
```

**The Ros2 bridge to run Go2 commands**
```bash
ros2 run p2_remote_connection web_teleop_bridge
```
The script does the following:
  - Source ROS 2 Foxy
  - Source Unitree Go2 ROS environment
  - Source this workspace’s overlay
  - Export DDS / ROS environment variables explicitly
  - Wait for the Go2 driver node to be available
  - Start:
    - FastAPI backend
    - HTTP static server
    - web_teleop_bridge node

They post to the server through the following:

- `GET /health`  
  Basic connectivity check (returns `{"status":"ok"}`)

- `POST /actions/{action}`  
  Runs a pre-defined command for that action (example: `sit`, `stand`)

- `POST /actions/{action}/stop`  
  Stops the currently-running process for that action (SIGTERM, then SIGKILL if needed)

You can explore and test everything using FastAPI’s built-in UI:

- `http://<go2-ip>:8000/docs`

### Making the script run on startup

1. Create a systemd service (autostart on boot)
- A systemd service was created so the system:
- Starts automatically on robot boot
- Restarts if it crashes
- Runs without any terminal attached
- Logs output to journalctl

**Service File Location**: 
```swift
/etc/systemd/system/p2-remote-connection.service
```
2. Enable and start the service:
```bash
sudo systemctl daemon-reload
sudo systemctl enable p2-remote-connection.service
sudo systemctl start p2-remote-connection.service
```

3. Monitoring the service:
```bash
# Check service status
systemctl status p2-remote-connection.service
# View logs live
journalctl -u p2-remote-connection.service -f
# Restart the system
sudo systemctl restart p2-remote-connection.service
```

4. To make changes to the startup:
```bash
# Edit the file with access
sudo vim /etc/systemd/system/p2-remote-connection.service   
# Reload after edit
sudo systemctl daemon-reload                                
sudo systemctl restart p2-remote-connection.service
# Check that it is still active
systemctl status p2-remote-connection.service --no-pager    
```
5. To turn off the startup and test manually:
```bash
# Stop the service
sudo systemctl stop p2-remote-connection.service
# Confirm the ports are free
sudo ss -ltnp | egrep ':8000|:8081'
# They should show nothing

# To restart
sudo systemctl start p2-remote-connection.service
```

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