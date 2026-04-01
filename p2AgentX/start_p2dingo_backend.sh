#!/usr/bin/env bash
set -eo pipefail
# NOTE: we intentionally do NOT enable 'set -u' until after sourcing ROS

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

if [ ! -d "$WS_DIR/src/go2_remote_connection" ]; then
  echo "[run_p2dingo] ❌ Can't locate go2_remote_connection package from $WS_DIR"
  exit 1
fi

PKG_DIR="$WS_DIR/src/go2_remote_connection"

API_HOST="0.0.0.0"
API_PORT="8200"

# Only override HOME under systemd (when HOME may be empty)
if [ -z "${HOME:-}" ] || [ "$HOME" = "/" ]; then
  export HOME="/home/unitree"
fi

get_best_ip() {
  ip route get 1.1.1.1 2>/dev/null | awk '/src/ {for(i=1;i<=NF;i++) if($i=="src"){print $(i+1); exit}}' || true
}

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
  echo "[run_p2dingo] Check the log with:"
  echo "  sed -n '1,200p' $log_path"
  echo "[run_p2dingo] Or follow it live with:"
  echo "  tail -f $log_path"
}

cleanup() {
  echo ""
  echo "[run_p2dingo] Stopping processes..."
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
    echo "[run_p2dingo] ❌ $name failed to start"
    print_log_hint "$log_path"
    cleanup
    exit 1
  fi
}

echo "[run_p2dingo] Script dir: $SCRIPT_DIR"
echo "[run_p2dingo] Workspace: $WS_DIR"
echo "[run_p2dingo] Package dir: $PKG_DIR"
echo "[run_p2dingo] Mode: p2dingo"

# --- Source ROS + Unitree stack safely ---
set +u

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

if [ -f "$HOME/unitree_ros2/install/setup.sh" ]; then
  echo "[run_p2dingo] Sourcing Unitree env: $HOME/unitree_ros2/install/setup.sh"
  source "$HOME/unitree_ros2/install/setup.sh"
elif [ -f "$HOME/unitree_ros2/install/setup.bash" ]; then
  echo "[run_p2dingo] Sourcing Unitree env: $HOME/unitree_ros2/install/setup.bash"
  source "$HOME/unitree_ros2/install/setup.bash"
else
  echo "[run_p2dingo] WARNING: Unitree env not found at $HOME/unitree_ros2/install/setup.(sh|bash)"
fi

if [ -f "$WS_DIR/install/setup.bash" ]; then
  echo "[run_p2dingo] Sourcing overlay: $WS_DIR/install/setup.bash"
  source "$WS_DIR/install/setup.bash"
else
  echo "[run_p2dingo] WARNING: overlay not found: $WS_DIR/install/setup.bash (did you colcon build?)"
fi

set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"

# ----------------------------
# Auth off by default; can enable later
# ----------------------------
export GO2_AUTH_ENABLED="${GO2_AUTH_ENABLED:-0}"
export GO2_TOKEN_FILE="${GO2_TOKEN_FILE:-$HOME/go2_token}"

if [ "$GO2_AUTH_ENABLED" = "1" ] || [ "$GO2_AUTH_ENABLED" = "true" ]; then
  if [ -f "$GO2_TOKEN_FILE" ]; then
    export GO2_API_TOKEN="$(tr -d '\r\n' < "$GO2_TOKEN_FILE")"
  else
    echo "[run_p2dingo] ❌ ERROR: $GO2_TOKEN_FILE not found (GO2_AUTH_ENABLED=1)"
    exit 1
  fi

  if [ -z "${GO2_API_TOKEN:-}" ]; then
    echo "[run_p2dingo] ❌ ERROR: GO2_API_TOKEN is empty (token file: $GO2_TOKEN_FILE)"
    exit 1
  fi

  echo "[run_p2dingo] 🔐 Auth enabled"
else
  export GO2_API_TOKEN=""
  echo "[run_p2dingo] 🔓 Auth disabled (GO2_AUTH_ENABLED=0)"
fi

# ----------------------------
# 0a) Start L1 -> /map2d
# ----------------------------
echo "[run_p2dingo] Starting flatten_l1_data (L1 -> /map2d)..."
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
  > /tmp/flatten_l1_data_p2dingo.log 2>&1 &

FLATTEN_PID=$!
register_pid "$FLATTEN_PID" "flatten_l1_data" "/tmp/flatten_l1_data_p2dingo.log"

sleep 0.3
ok_or_die "flatten_l1_data" "$FLATTEN_PID" "/tmp/flatten_l1_data_p2dingo.log"

# ------------------------------------------------------------
# 0b) Front camera capture node + ROS bridge
# ------------------------------------------------------------
GO2_WS_DIR="$WS_DIR"
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

echo "[run_p2dingo] Using UNITREE_IFACE=$UNITREE_IFACE"
echo "[run_p2dingo] Using VENV_PYTHON=$VENV_PYTHON"

echo "[run_p2dingo] Starting front_camera_capture ..."
"$VENV_PYTHON" "$FRONT_CAMERA_CAPTURE_EXE" \
  > /tmp/front_camera_capture_p2dingo.log 2>&1 &

FRONT_CAM_CAPTURE_PID=$!
register_pid "$FRONT_CAM_CAPTURE_PID" "front_camera_capture" "/tmp/front_camera_capture_p2dingo.log"

sleep 2.0
ok_or_die "front_camera_capture" "$FRONT_CAM_CAPTURE_PID" "/tmp/front_camera_capture_p2dingo.log"

echo "[run_p2dingo] Starting front_camera_ros_bridge (/front_camera/image_raw)..."
"$VENV_PYTHON" "$FRONT_CAMERA_BRIDGE_EXE" --ros-args -r __node:=front_camera_node \
  > /tmp/front_camera_node_p2dingo.log 2>&1 &

FRONT_CAM_NODE_PID=$!
register_pid "$FRONT_CAM_NODE_PID" "front_camera_ros_bridge" "/tmp/front_camera_node_p2dingo.log"

sleep 2.0
ok_or_die "front_camera_ros_bridge" "$FRONT_CAM_NODE_PID" "/tmp/front_camera_node_p2dingo.log"

# ----------------------------
# 1) Start P2Dingo-only FastAPI backend
# ----------------------------
echo "[run_p2dingo] Starting FastAPI (uvicorn) on :$API_PORT ..."
cd "$PKG_DIR"
python3 -m uvicorn app.p2dingo_main:app --host "$API_HOST" --port "$API_PORT" \
  > /tmp/go2_fastapi_p2dingo.log 2>&1 &

API_PID=$!
register_pid "$API_PID" "FastAPI-P2Dingo" "/tmp/go2_fastapi_p2dingo.log"

sleep 0.8
ok_or_die "FastAPI-P2Dingo" "$API_PID" "/tmp/go2_fastapi_p2dingo.log"

# ----------------------------
# 2) Wait for Unitree sport topics before starting bridge
# ----------------------------
echo "[run_p2dingo] Waiting for Unitree sport topics..."
SPORT_READY=0
for i in {1..30}; do
  if ros2 topic list 2>/dev/null | grep -q "^/api/sport/request$"; then
    echo "[run_p2dingo] Unitree Sport API is up."
    SPORT_READY=1
    break
  fi
  sleep 1
done

kill_conflicting_nodes() {
  echo "[run_p2dingo] Killing conflicting motion nodes (if any)..."
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

if [ "$SPORT_READY" -eq 1 ]; then
  echo "[run_p2dingo] Starting web_bridge ..."
  ros2 run go2_remote_connection web_bridge > /tmp/web_bridge_p2dingo.log 2>&1 &
  WEB_BRIDGE_PID=$!
  register_pid "$WEB_BRIDGE_PID" "web_bridge" "/tmp/web_bridge_p2dingo.log"

  echo "[run_p2dingo] Starting move_forward_meters_node ..."
  ros2 run go2_remote_connection move_forward_meters_node > /tmp/move_forward_meters_p2dingo.log 2>&1 &
  MOVE_PID=$!
  register_pid "$MOVE_PID" "move_forward_meters_node" "/tmp/move_forward_meters_p2dingo.log"

  sleep 0.5
  ok_or_die "web_bridge" "$WEB_BRIDGE_PID" "/tmp/web_bridge_p2dingo.log"
  ok_or_die "move_forward_meters_node" "$MOVE_PID" "/tmp/move_forward_meters_p2dingo.log"
else
  echo "[run_p2dingo] ⚠️  Unitree sport topics not found after timeout; continuing without robot motion backend."
  echo "[run_p2dingo] ⚠️  Skipping web_bridge and move_forward_meters_node because Unitree sport topics are unavailable."
fi

echo ""
echo "[run_p2dingo] ✅ P2Dingo backend started."

HOST_IP="$(get_best_ip || true)"
if [ -z "$HOST_IP" ]; then
  HOST_IP="$(hostname -I 2>/dev/null | awk '{print $1}')"
fi
HOST_IP="${HOST_IP:-127.0.0.1}"

echo "[run_p2dingo] Page: http://$HOST_IP:$API_PORT/"
echo "[run_p2dingo] Press Ctrl+C to stop everything."
echo ""

echo "See logs with:"
echo "  sed -n '1,200p' /tmp/go2_fastapi_p2dingo.log"
echo "  sed -n '1,200p' /tmp/flatten_l1_data_p2dingo.log"
echo "  sed -n '1,200p' /tmp/front_camera_capture_p2dingo.log"
echo "  sed -n '1,200p' /tmp/front_camera_node_p2dingo.log"
echo "  sed -n '1,200p' /tmp/web_bridge_p2dingo.log"
echo "  sed -n '1,200p' /tmp/move_forward_meters_p2dingo.log"

set +e

while true; do
  for pid in "${pids[@]}"; do
    if ! kill -0 "$pid" 2>/dev/null; then
      wait "$pid"
      rc=$?
      name="${pid_names[$pid]:-Child process}"
      log_path="${pid_logs[$pid]:-}"

      echo "[run_p2dingo] ❌ $name exited with code $rc"
      print_log_hint "$log_path"
      exit "$rc"
    fi
  done
  sleep 1
done