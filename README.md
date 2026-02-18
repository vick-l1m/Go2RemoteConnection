# Go2RemoteConnection

Creating a web access interface to connect to Go2 functions.

## FastAPI backend (prototype)

This project runs a small FastAPI server on the Unitree Go2 (or a machine with the same ROS 2/Unitree environment)
and exposes **HTTP endpoints** that trigger a **whitelisted set of shell commands**.

## Prerequisites
- Unitree ROS2: Install the Unitree ROS2 for GO2 robot, please refer to [Unitree ROS2](https://github.com/unitreerobotics/unitree_ros2)
- Unitree SDK2 Python: please refer to [Unitree SDK2 Python](https://github.com/unitreerobotics/unitree_sdk2_python)

## 1. Build the workspace

### 1.1. Clone this workspace
```bash
  mkdir p2_ws
  cd p2_ws
  git clone https://github.com/vick-l1m/Go2RemoteConnection.git
```

On the Go2 (or the device running the server)
```bash
# Install dependancies
pip3 install fastapi uvicorn
sudo apt update
sudo apt install ros-humble-map-msgs

# For YOLO and Camera
sudo apt install -y \
  ros-humble-cv-bridge \
  ros-humble-image-transport \
  ros-humble-sensor-msgs
cd ~/p2_ws/Go2RemoteConnection/src/p2_remote_connection/src/cv/model
python3 model_download.py

# Setup the workspace
cd ~/p2_ws/Go2RemoteConnection/src/p2_remote_connection

pip install -r requirements.txt
source ~/unitree_ros2/install/setup.sh
cd ~/p2_ws/Go2RemoteConnection
colcon build
source install/setup.bash
```

### 1.2. Setup the unique token:
On the Go2, create a file in the robot user’s home directory:

```bash
vim ~/.go2_token
``` 
Put the token inside the file - one line with no spaces

Secure the token file ensureing that it is read-only:
```bash
chmod 600 ~/.go2_token
```
This token will be used to ensure security and that each Go2 has a unique id.

### 1.3. Launch the backend and ros2 node
Make the command runnable and launch:
```bash
cd ~/p2_ws/Go2RemoteConnection
chmod +x start_remote_connection.sh
./start_remote_connection.sh
```

To run on the Issac Sim:
```bash
use_fastrtps
cd ~/p2_ws/Go2RemoteConnection
chmod +x start_remote_connection_humble.sh
./start_remote_connection_humble.sh
```
### 1.4. Connect and run 
Check the device ip address:
```bash
# search for wlan0 ip
ip -br a 
```
Open the website on a device connected to the same wifi: 
```bash
<device_ip>::8081/app/

# Local Go2 (ethernet connected)
192.168.123.18/app/
```

### 1.5. Landing page and login
```/app``` will open a landing page allowing you to choose between: 
1. ```go2_joystick.html```
    - ```sit``` and ```stand``` commands
    - 2 joysticks allowing you to move the go2
    - 3 accessable and toggelable terminals
    - ```stop``` and ```resume``` toggle, latching all movement commands and stopping the web_teleop node

2. ```go2_terminal_only.html```
    - 3 accessable and toggelable terminals
    - ```stop``` and ```resume``` toggle, stops all terminals

### Operating without ```web_teleop_node```
Launching with
```bash
./start_remote_connection terminal
``` 
Will disable the web_teleop node, allowing the ```go2_terminal_only.html``` to run interupted.

### Debugging:
Each node comes with a log, you can read each log using the command
```bash
sed -n '1,200p' /tmp/<node>log

# Eg front_camera_node
sed -n '1,200p' /tmp/front_camera_node.log
```

## 2. Making the script run on startup

### 2.1. Create a systemd service (autostart on boot)
A systemd service can be created so the system
- Starts automatically on robot boot
- Restarts if it crashes
- Runs without any terminal attached
- Logs output to journalctl

**Create the service file**: 
```swift
sudo vim /etc/systemd/system/p2-remote-connection.service
```
### 2.2. Enable and start the service:
```bash
sudo systemctl daemon-reload
sudo systemctl enable p2-remote-connection.service
sudo systemctl start p2-remote-connection.service
```

### 2.3. Monitoring the service:
```bash
# Check service status
systemctl status p2-remote-connection.service
# View logs live
journalctl -u p2-remote-connection.service -f
# Restart the system
sudo systemctl restart p2-remote-connection.service
```

### 2.4. To make changes to the startup:
```bash
# Edit the file with access
sudo vim /etc/systemd/system/p2-remote-connection.service   
# Reload after edit
sudo systemctl daemon-reload                                
sudo systemctl restart p2-remote-connection.service
# Check that it is still active
systemctl status p2-remote-connection.service --no-pager    
```

### 2.5. To turn off the startup and test manually:
```bash
# Stop the service
sudo systemctl stop p2-remote-connection.service
# Confirm the ports are free
sudo ss -ltnp | egrep ':8000|:8081'
# They should show nothing

# To restart
sudo systemctl restart p2-remote-connection.service
```

## 3. How it works
The start up script: ```./start_remote_connection.sh``` runs 3 seperate process':

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

### 4. Features
#### 4.1. Login gate / UI lock
- Shows login Overlay
- Stores ROBOT_BASE_URL (where the robot’s API is) and AUTH_TOKEN (your password token)
- Saves them into localStorage so the browser remembers them

#### 4.2. Teleop joystick
- Renders two canvas joysticks
- Converts joystick position into a command {linear_x, linear_y, angular_z}
- Sends that command to the robot API at /teleop 20 times per second while the stick is active

#### 4.3. Terminal
- Creates an xterm.js terminal
- Opens a websocket to the robot API at /ws/terminal?token=...
- Streams keystrokes to backend, and streams output back to the browser
- Up to 3 terminals can be accessed at a time

## 5. Authentication Overview

### 5.1. Unique Token

Each Go2 has a **unique secret string** (“token” / “password”) that must be provided by the user before the website will send robot commands.

The browser sends this token with each API request using an HTTP header:

- `Authorization: Bearer <token>`

The backend checks whether the supplied token matches the Go2’s expected token.

### 5.2. How the token is used

#### Uvicorn Backend
All endpoints depend on the ```require_token``` dependancy, meaning that you cannot run any actions without entering the correct token.

#### Fontend (WebUI) login
When the user opens the web UI, the UI is locked and a login overlay is shown.

Once the correct token is entered, the UI is accessable. 
The frontend stores the last successful token + URL in ```localStorage```, so that the same device does not have to continuously log-in.

The logout button clears this storage.
