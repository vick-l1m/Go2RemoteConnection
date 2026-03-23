#!/usr/bin/env bash
set -eo pipefail
# NOTE: we intentionally do NOT enable 'set -u' until after sourcing ROS

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# If the script is in workspace root:
if [ -d "$WS_DIR/src/go2_remote_connection" ]; then
  PKG_DIR="$WS_DIR/src/go2_remote_connection"
# If the script is inside the package (e.g. .../src/go2_remote_connection):
elif [ -d "$WS_DIR/app" ] && [ -d "$WS_DIR/src" ]; then
  PKG_DIR="$WS_DIR"
else
  echo "[run_all] ❌ Can't locate go2_remote_connection package from $WS_DIR"
  exit 1
fi

API_HOST="0.0.0.0"
API_PORT="8000"

# Only override HOME under systemd (when HOME may be empty)
if [ -z "${HOME:-}" ] || [ "$HOME" = "/" ]; then
  export HOME="/home/unitree"
fi

get_best_ip() {
  ip route get 1.1.1.1 2>/dev/null | awk '/src/ {for(i=1;i<=NF;i++) if($i=="src"){print $(i+1); exit}}' || true
}

# ----------------------------
# UI selection by CLI arg
# ----------------------------
MODE="${1:-joystick}"   # joystick | terminal | movement

case "$MODE" in
  terminal|joystick|movement|"")
    ;;
  *)
    echo "[run_all] ❌ Unknown mode: '$MODE'"
    echo "Usage:"
    echo "  $0                # serve joystick UI + bridge"
    echo "  $0 joystick       # serve joystick UI + bridge"
    echo "  $0 terminal       # serve terminal-only UI (no bridge)"
    echo "  $0 movement       # serve movement UI + bridge"
    exit 1
    ;;
esac

pids=()
declare -A pid_names
declare -A pid_logs

register_pid() {
  local pid="$1"
  local name="$2"
  local log_path="${3:-}"
  pids+=("$pid")
  pid_names["$pid"]="$name"
  pid_logs["$pid"]="$log_path"
}

print_log_hint() {
  local log_path="${1:-}"
  [ -z "$log_path" ] && return 0
  echo "[run_all] Check the log with:"
  echo "  sed -n '1,200p' $log_path"
  echo "[run_all] Or follow it live with:"
  echo "  tail -f $log_path"
}

cleanup() {
  echo ""
  echo "[run_all] Stopping processes..."
  for pid in "${pids[@]:-}"; do
    if kill -0 "$pid" 2>/dev/null; then
      kill "$pid" 2>/dev/null || true
    fi
  done
  sleep 0.5 || true
  for pid in "${pids[@]:-}"; do
    if kill -0 "$pid" 2>/dev/null; then
      kill -9 "$pid" 2>/dev/null || true
    fi
  done
}
trap cleanup EXIT INT TERM

ok_or_die() {
  local name="$1"
  local pid="$2"
  local log_path="${3:-}"

  if ! kill -0 "$pid" 2>/dev/null; then
    echo "[run_all] ❌ $name failed to start"
    print_log_hint "$log_path"
    cleanup
    exit 1
  fi
}

echo "[run_all] Workspace: $WS_DIR"
echo "[run_all] Mode: $MODE"

# --- Source ROS + Unitree stack safely ---
set +u

# 1) Source ROS 2 environment (Foxy preferred, fallback to Humble)
if [ -f /opt/ros/foxy/setup.bash ]; then
  echo "[env] Sourcing ROS 2 Foxy"
  source /opt/ros/foxy/setup.bash
elif [ -f /opt/ros/humble/setup.bash ]; then
  echo "[env] Sourcing ROS 2 Humble"
  source /opt/ros/humble/setup.bash
else
  echo "[env] ❌ No ROS 2 setup.bash found in /opt/ros"
  exit 1
fi

# 2) Unitree environment
if [ -f "$HOME/unitree_ros2/install/setup.sh" ]; then
  echo "[run_all] Sourcing Unitree env: $HOME/unitree_ros2/install/setup.sh"
  source "$HOME/unitree_ros2/install/setup.sh"
elif [ -f "$HOME/unitree_ros2/install/setup.bash" ]; then
  echo "[run_all] Sourcing Unitree env: $HOME/unitree_ros2/install/setup.bash"
  source "$HOME/unitree_ros2/install/setup.bash"
else
  echo "[run_all] WARNING: Unitree env not found at $HOME/unitree_ros2/install/setup.(sh|bash)"
fi

# 3) Your overlay
if [ -f "$WS_DIR/install/setup.bash" ]; then
  echo "[run_all] Sourcing overlay: $WS_DIR/install/setup.bash"
  source "$WS_DIR/install/setup.bash"
else
  echo "[run_all] WARNING: overlay not found: $WS_DIR/install/setup.bash (did you colcon build?)"
fi

set -u

# --- Make runtime deterministic under systemd ---
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"

# ----------------------------
# Turn on/off Authentication
# ----------------------------
export GO2_AUTH_ENABLED=0
export GO2_API_TOKEN=""
echo "[run_all] 🔓 Auth disabled (GO2_AUTH_ENABLED=0)"

# ----------------------------
# Load Go2 API token (ONLY if auth is enabled)
# ----------------------------
TOKEN_FILE="${GO2_TOKEN_FILE:-$HOME/.go2_token}"

if [ "$GO2_AUTH_ENABLED" = "1" ] || [ "$GO2_AUTH_ENABLED" = "true" ]; then
  if [ -f "$TOKEN_FILE" ]; then
    export GO2_API_TOKEN="$(tr -d '\r\n' < "$TOKEN_FILE")"
  else
    echo "[run_all] ❌ ERROR: $TOKEN_FILE not found (GO2_AUTH_ENABLED=1)"
    exit 1
  fi

  if [ -z "${GO2_API_TOKEN:-}" ]; then
    echo "[run_all] ❌ ERROR: GO2_API_TOKEN is empty (token file: $TOKEN_FILE)"
    exit 1
  fi

  echo "[run_all] GO2_API_TOKEN loaded"
else
  export GO2_API_TOKEN=""
  echo "[run_all] 🔓 Auth disabled (GO2_AUTH_ENABLED=0): skipping token load"
fi

# ----------------------------
# 0a) Start L1 -> /map2d
# ----------------------------
echo "[run_all] Starting flatten_l1_data (L1 -> /map2d)..."
ros2 run go2_remote_connection flatten_l1_data \
  --ros-args \
  -p cloud_topic:=/utlidar/cloud_base \
  -p map2d_topic:=/map2d \
  -p min_height_rel:=-5.0 \
  -p max_height_rel:=5.0 \
  -p tick_rate_hz:=5.0 \
  -p update_radius_m:=20.0 \
  -p decay_per_tick:=100 \
  -p max_occ:=100 \
  -p resolution:=0.1 \
  > /tmp/flatten_l1_data.log 2>&1 &

FLATTEN_PID=$!
register_pid "$FLATTEN_PID" "flatten_l1_data" "/tmp/flatten_l1_data.log"

sleep 0.3
ok_or_die "flatten_l1_data" "$FLATTEN_PID" "/tmp/flatten_l1_data.log"

# ------------------------------------------------------------
# 0b) Front camera capture node + ROS bridge
# ------------------------------------------------------------
GO2_WS_DIR="$HOME/go2_ws/Go2RemoteConnection"
PKG_NAME="go2_remote_connection"
PKG_INSTALL_DIR="$GO2_WS_DIR/install/$PKG_NAME/lib/$PKG_NAME"

VENV_PYTHON="$HOME/venvs/unitree_sdk2_python/bin/python3"
UNITREE_SDK_SRC="$HOME/unitree_sdk2_python"
VENV_SITE="$HOME/venvs/unitree_sdk2_python/lib/python3.10/site-packages"
EXISTING_PP="${PYTHONPATH:-}"
COMBINED_PP="$UNITREE_SDK_SRC:$VENV_SITE:$EXISTING_PP"

export PYTHONNOUSERSITE="1"
export PYTHONPATH="$COMBINED_PP"

HOSTNAME_LOWER="$(hostname | tr '[:upper:]' '[:lower:]')"

detect_laptop_interface() {
  local target_ip="${1:-192.168.123.161}"
  local out iface
  out="$(ip route get "$target_ip" 2>/dev/null)" || return 1
  iface="$(awk '/dev/ {for(i=1;i<=NF;i++) if($i=="dev") {print $(i+1); exit}}' <<< "$out")"
  [ -n "$iface" ] && [ "$iface" != "lo" ] && echo "$iface"
}

if [[ "$HOSTNAME_LOWER" == *unitree* || "$HOSTNAME_LOWER" == *jetson* || "$HOSTNAME_LOWER" == go2* ]]; then
  DEFAULT_UNITREE_IFACE="enP8p1s0"
else
  DEFAULT_UNITREE_IFACE="$(detect_laptop_interface 192.168.123.161)"
  DEFAULT_UNITREE_IFACE="${DEFAULT_UNITREE_IFACE:-enp0s31f6}"
fi

export UNITREE_IFACE="${UNITREE_IFACE:-$DEFAULT_UNITREE_IFACE}"
export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"${UNITREE_IFACE}\" priority=\"default\" multicast=\"default\" /></Interfaces></General></Domain></CycloneDDS>"

FRONT_CAMERA_CAPTURE_EXE="$PKG_INSTALL_DIR/front_camera_capture.py"
FRONT_CAMERA_BRIDGE_EXE="$PKG_INSTALL_DIR/front_camera_ros_bridge.py"

export GO2_CAM_JPEG_QUALITY=40
export GO2_CAM_SCALE=0.5

echo "[run_all] Using UNITREE_IFACE=$UNITREE_IFACE"
echo "[run_all] Using VENV_PYTHON=$VENV_PYTHON"

echo "[run_all] Starting front_camera_capture ..."
"$VENV_PYTHON" "$FRONT_CAMERA_CAPTURE_EXE" \
  > /tmp/front_camera_capture.log 2>&1 &

front_cam_capture_PID=$!
register_pid "$front_cam_capture_PID" "front_camera_capture" "/tmp/front_camera_capture.log"

sleep 2.0
ok_or_die "front_camera_capture" "$front_cam_capture_PID" "/tmp/front_camera_capture.log"

echo "[run_all] Starting front_camera_ros_bridge (/front_camera/image_raw)..."
"$VENV_PYTHON" "$FRONT_CAMERA_BRIDGE_EXE" --ros-args -r __node:=front_camera_node \
  > /tmp/front_camera_node.log 2>&1 &

front_cam_node_PID=$!
register_pid "$front_cam_node_PID" "front_camera_ros_bridge" "/tmp/front_camera_node.log"

sleep 2.0
ok_or_die "front_camera_ros_bridge" "$front_cam_node_PID" "/tmp/front_camera_node.log"

# ----------------------------
# 1) Start FastAPI backend
# ----------------------------
echo "[run_all] Starting FastAPI (uvicorn) on :$API_PORT ..."

cd "$PKG_DIR"
python3 -m uvicorn app.main:app --host "$API_HOST" --port "$API_PORT" \
  > /tmp/go2_fastapi.log 2>&1 &

API_PID=$!
register_pid "$API_PID" "FastAPI" "/tmp/go2_fastapi.log"

sleep 0.8
ok_or_die "FastAPI" "$API_PID" "/tmp/go2_fastapi.log"

# ----------------------------
# 2) Wait for Unitree sport topics before starting bridge
# ----------------------------
echo "[run_all] Waiting for Unitree sport topics..."
for i in {1..30}; do
  if ros2 topic list 2>/dev/null | grep -q "^/api/sport/request$"; then
    echo "[run_all] Unitree Sport API is up."
    break
  fi
  sleep 1
done

# ----------------------------
# 3) Start ROS nodes depending on MODE
# ----------------------------
kill_conflicting_nodes() {
  echo "[run_all] Killing conflicting motion nodes (if any)..."
  pkill -f "ros2 run go2_remote_connection web_teleop_bridge" || true
  pkill -f "ros2 run go2_remote_connection web_advanced_bridge" || true
  pkill -f "ros2 run go2_remote_connection advanced_gamepad_controller_web" || true
  pkill -f "go2_remote_connection.*web_teleop_bridge" || true
  pkill -f "go2_remote_connection.*web_advanced_bridge" || true
  pkill -f "go2_remote_connection.*advanced_gamepad_controller_web" || true
  pkill -f "ros2 run go2_remote_connection web_bridge" || true
  pkill -f "go2_remote_connection.*web_bridge" || true
  sleep 0.3
}

kill_conflicting_nodes

if [ "$MODE" = "terminal" ]; then
  echo "[run_all] Terminal mode: skipping motion nodes ✅"
else
  echo "[run_all] Starting web_bridge (for ALL web UIs)"
  ros2 run go2_remote_connection web_bridge > /tmp/web_bridge.log 2>&1 &
  WEB_BRIDGE_PID=$!
  register_pid "$WEB_BRIDGE_PID" "web_bridge" "/tmp/web_bridge.log"

  echo "[run_all] Starting move_forward_meters_node ..."
  ros2 run go2_remote_connection move_forward_meters_node > /tmp/move_forward_meters.log 2>&1 &
  MOVE_PID=$!
  register_pid "$MOVE_PID" "move_forward_meters_node" "/tmp/move_forward_meters.log"

  sleep 0.5
  ok_or_die "web_bridge" "$WEB_BRIDGE_PID" "/tmp/web_bridge.log"
  ok_or_die "move_forward_meters_node" "$MOVE_PID" "/tmp/move_forward_meters.log"
fi

echo ""
echo "[run_all] ✅ All started."

HOST_IP="$(get_best_ip || true)"
if [ -z "$HOST_IP" ]; then
  HOST_IP="$(hostname -I 2>/dev/null | awk '{print $1}')"
fi
HOST_IP="${HOST_IP:-127.0.0.1}"

echo "[run_all] Home: http://$HOST_IP:$API_PORT/"
case "$MODE" in
  terminal)
    echo "[run_all] UI:   http://$HOST_IP:$API_PORT/terminal"
    ;;
  movement)
    echo "[run_all] UI:   http://$HOST_IP:$API_PORT/movement"
    ;;
  *)
    echo "[run_all] UI:   http://$HOST_IP:$API_PORT/joystick"
    ;;
esac

echo "[run_all] API:  http://$HOST_IP:$API_PORT"
echo "[run_all] Press Ctrl+C to stop everything."
echo ""

echo "See logs with:"
echo "  sed -n '1,200p' /tmp/go2_fastapi.log"
echo "  sed -n '1,200p' /tmp/flatten_l1_data.log"
echo "  sed -n '1,200p' /tmp/front_camera_capture.log"
echo "  sed -n '1,200p' /tmp/front_camera_node.log"
echo "  sed -n '1,200p' /tmp/web_bridge.log"
echo "  sed -n '1,200p' /tmp/move_forward_meters.log"

set +e

while true; do
  for pid in "${pids[@]}"; do
    if ! kill -0 "$pid" 2>/dev/null; then
      wait "$pid"
      rc=$?
      name="${pid_names[$pid]:-Child process}"
      log_path="${pid_logs[$pid]:-}"

      echo "[run_all] ❌ $name exited with code $rc"
      print_log_hint "$log_path"
      exit "$rc"
    fi
  done
  sleep 1
done