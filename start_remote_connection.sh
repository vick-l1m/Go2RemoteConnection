#!/usr/bin/env bash
set -eo pipefail
# NOTE: we intentionally do NOT enable 'set -u' until after sourcing ROS

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$WS_DIR/src/p2_remote_connection"

API_HOST="0.0.0.0"
API_PORT="8000"
UI_PORT="8081"

# Only override HOME under systemd (when HOME may be empty)
if [ -z "${HOME:-}" ] || [ "$HOME" = "/" ]; then
  export HOME="/home/unitree"
fi

get_best_ip() {
  # Try to get the source IP used to reach the internet (works on most Linux)
  ip route get 1.1.1.1 2>/dev/null | awk '/src/ {for(i=1;i<=NF;i++) if($i=="src"){print $(i+1); exit}}'
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
    exit 1
    ;;
esac

pids=()

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
  if ! kill -0 "$pid" 2>/dev/null; then
    echo "[run_all] Error occurred ❌ ($name failed to start)"
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

# 2) Unitree environment (IMPORTANT on Go2)
if [ -f "$HOME/unitree_ros2/install/setup.sh" ]; then
  echo "[run_all] Sourcing Unitree env: $HOME/unitree_ros2/install/setup.sh"
  source "$HOME/unitree_ros2/install/setup.sh"
elif [ -f "$HOME/unitree_ros2/install/setup.bash" ]; then
  echo "[run_all] Sourcing Unitree env: $HOME/unitree_ros2/install/setup.bash"
  source "$HOME/unitree_ros2/install/setup.bash"
else
  echo "[run_all] WARNING: Unitree env not found at $HOME/unitree_ros2/install/setup.(sh|bash)"
fi

# 3) Your overlay (P2RemoteConnection)
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
# export CYCLONEDDS_URI="file://$HOME/unitree_ros2/cyclonedds.xml"

# ----------------------------
# Turn on/off Authentication (set BEFORE token handling)
# ----------------------------
export GO2_AUTH_ENABLED="${GO2_AUTH_ENABLED:-0}"   # default: 0 for your testing
# export GO2_AUTH_ENABLED="1"                      # enable auth

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
  # Auth disabled: do NOT read token file
  export GO2_API_TOKEN=""
  echo "[run_all] 🔓 Auth disabled (GO2_AUTH_ENABLED=0): skipping token load"
fi

# ----------------------------
# 0a) Start L1 -> 2D map generator (flatten_l1_data) and 3D point cloud streamer (flatten_l1_data with different params)
# ----------------------------
echo "[run_all] Starting flatten_l1_data (L1 -> /map2d)..."
ros2 run p2_remote_connection flatten_l1_data \
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
pids+=("$FLATTEN_PID")

echo "[run_all] Starting L1_pcd_web_packer (L1 -> /pcd/xyz32)..."
ros2 run p2_remote_connection L1_pcd_web_packer \
  --ros-args \
  -p cloud_topic:=/utlidar/cloud_deskewed \
  -p target_frame:=odom \
  -p publish_hz:=5.0 \
  -p voxel_size:=0.10 \
  -p max_points:=20000 \
  -p min_range:=0.25 \
  -p max_range:=25.0 \
  -p min_z:=-1.0 \
  -p max_z:=3.0 \
  > /tmp/L1_pcd_web_packer.log 2>&1 &

PCD_PID=$!
pids+=("$PCD_PID")

sleep 0.3
ok_or_die "flatten_l1_data" "$FLATTEN_PID"
ok_or_die "L1_pcd_web_packer" "$PCD_PID"

# ------------------------------------------------------------
# 0b) Front camera capture node, Image -> JPEG compressed bridge
# ------------------------------------------------------------
chmod +x "$PKG_DIR/src/front_camera_node.py" "$PKG_DIR/src/image_to_compressed_bridge.py"

echo "[run_all] Starting front camera node (/front_camera/image_raw)..."
python3 "$PKG_DIR/src/front_camera_node.py" \
  > /tmp/front_camera_node.log 2>&1 &

front_cam_node_PID=$!
pids+=("$front_cam_node_PID")
sleep 2.0
ok_or_die "front_camera_node" "$front_cam_node_PID"

echo "[run_all] Starting image_to_compressed_bridge (/front_camera/image_raw -> /web/front_cam/compressed)..."
python3 "$PKG_DIR/src/image_to_compressed_bridge.py" --ros-args \
  -p jpeg_quality:=80 \
  > /tmp/image_to_compressed_bridge.log 2>&1 &

front_cam_bridge_PID=$!
pids+=("$front_cam_bridge_PID")
sleep 1.0
ok_or_die "image_to_compressed_bridge" "$front_cam_bridge_PID"

# ----------------------------
# 1) Start FastAPI backend
# ----------------------------
echo "[run_all] Starting FastAPI (uvicorn) on :$API_PORT ..."
cd "$PKG_DIR"
python3 -m uvicorn app.main:app --host "$API_HOST" --port "$API_PORT" &
API_PID=$!
pids+=("$API_PID")

sleep 0.5
if ! kill -0 "$API_PID" 2>/dev/null; then
  echo "[run_all] ❌ FastAPI failed to start (check logs above)."
  exit 1
fi

# ----------------------------
# 2) Start static file server for frontend
# ----------------------------
echo "[run_all] Starting http.server on :$UI_PORT ..."
cd "$PKG_DIR"
python3 -m http.server "$UI_PORT" &
UI_PID=$!
pids+=("$UI_PID")

# ----------------------------
# 3) Wait for unitree sport topics before starting bridge (kept the same)
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
# 4) Start ROS nodes depending on MODE
# ----------------------------
kill_conflicting_nodes() {
  echo "[run_all] Killing conflicting motion nodes (if any)..."
  pkill -f "ros2 run p2_remote_connection web_teleop_bridge" || true
  pkill -f "ros2 run p2_remote_connection web_advanced_bridge" || true
  pkill -f "ros2 run p2_remote_connection advanced_gamepad_controller_web" || true
  pkill -f "p2_remote_connection.*web_teleop_bridge" || true
  pkill -f "p2_remote_connection.*web_advanced_bridge" || true
  pkill -f "p2_remote_connection.*advanced_gamepad_controller_web" || true
  pkill -f "ros2 run p2_remote_connection web_bridge" || true
  pkill -f "p2_remote_connection.*web_bridge" || true
  sleep 0.3
}

kill_conflicting_nodes
if [ "$MODE" = "terminal" ]; then
  echo "[run_all] Terminal mode: skipping motion nodes ✅"
else
  echo "[run_all] Starting web_bridge (for ALL web UIs)"
  ros2 run p2_remote_connection web_bridge &
  pids+=("$!")

  # optional helper if you still use it from API
  ros2 run p2_remote_connection move_forward_meters_node &
  pids+=("$!")
fi

# Give them a moment to crash if they will
sleep 0.5

# Validate they are alive
ok_or_die "FastAPI" "$API_PID"
ok_or_die "UI server" "$UI_PID"

echo ""
echo "[run_all] ✅ All started."

HOST_IP="$(get_best_ip)"
if [ -z "$HOST_IP" ]; then
  HOST_IP="$(hostname -I 2>/dev/null | awk '{print $1}')"
fi
HOST_IP="${HOST_IP:-127.0.0.1}"

if [ "$MODE" = "terminal" ]; then
  echo "[run_all] UI:  http://$HOST_IP:$UI_PORT/app/go2_terminal_only.html"
elif [ "$MODE" = "movement" ]; then
  echo "[run_all] UI:  http://$HOST_IP:$UI_PORT/app/go2_movement_controller.html"
else
  echo "[run_all] UI:  http://$HOST_IP:$UI_PORT/app/go2_joystick.html"
fi


echo "[run_all] API: http://$HOST_IP:$API_PORT"
echo "[run_all] Press Ctrl+C to stop everything."
echo ""

wait -n
echo "[run_all] A process exited; shutting down..."
exit 0
