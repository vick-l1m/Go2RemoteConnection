#!/usr/bin/env bash
set -eo pipefail
# NOTE: intentionally do NOT enable 'set -u' until after sourcing ROS

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# If the script is in workspace root:
if [ -d "$WS_DIR/src/go2_remote_connection" ]; then
  PKG_DIR="$WS_DIR/src/go2_remote_connection"
# If the script is inside the package:
elif [ -d "$WS_DIR/app" ] && [ -d "$WS_DIR/src" ]; then
  PKG_DIR="$WS_DIR"
else
  echo "[run_movement_only] ❌ Can't locate go2_remote_connection package from $WS_DIR"
  exit 1
fi

API_HOST="0.0.0.0"
API_PORT="8000"
UI_PORT="8081"

# Only override HOME under systemd (when HOME may be empty)
if [ -z "${HOME:-}" ] || [ "$HOME" = "/" ]; then
  export HOME="/home/unitree"
fi

get_best_ip() {
  ip route get 1.1.1.1 2>/dev/null | awk '/src/ {for(i=1;i<=NF;i++) if($i=="src"){print $(i+1); exit}}'
}

pids=()

cleanup() {
  echo ""
  echo "[run_movement_only] Stopping processes..."
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
    echo "[run_movement_only] ❌ $name failed to start"
    cleanup
    exit 1
  fi
}

echo "[run_movement_only] Workspace: $WS_DIR"

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
  echo "[run_movement_only] Sourcing Unitree env: $HOME/unitree_ros2/install/setup.sh"
  source "$HOME/unitree_ros2/install/setup.sh"
elif [ -f "$HOME/unitree_ros2/install/setup.bash" ]; then
  echo "[run_movement_only] Sourcing Unitree env: $HOME/unitree_ros2/install/setup.bash"
  source "$HOME/unitree_ros2/install/setup.bash"
else
  echo "[run_movement_only] WARNING: Unitree env not found at $HOME/unitree_ros2/install/setup.(sh|bash)"
fi

# 3) Your overlay
if [ -f "$WS_DIR/install/setup.bash" ]; then
  echo "[run_movement_only] Sourcing overlay: $WS_DIR/install/setup.bash"
  source "$WS_DIR/install/setup.bash"
else
  echo "[run_movement_only] WARNING: overlay not found: $WS_DIR/install/setup.bash (did you colcon build?)"
fi

set -u

# --- Make runtime deterministic ---
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
# export CYCLONEDDS_URI="file://$HOME/unitree_ros2/cyclonedds.xml"

# ----------------------------
# Authentication
# ----------------------------
export GO2_AUTH_ENABLED="${GO2_AUTH_ENABLED:-0}"   # default off for testing
# export GO2_AUTH_ENABLED="1"

TOKEN_FILE="${GO2_TOKEN_FILE:-$HOME/.go2_token}"

if [ "$GO2_AUTH_ENABLED" = "1" ] || [ "$GO2_AUTH_ENABLED" = "true" ]; then
  if [ -f "$TOKEN_FILE" ]; then
    export GO2_API_TOKEN="$(tr -d '\r\n' < "$TOKEN_FILE")"
  else
    echo "[run_movement_only] ❌ ERROR: $TOKEN_FILE not found (GO2_AUTH_ENABLED=1)"
    exit 1
  fi

  if [ -z "${GO2_API_TOKEN:-}" ]; then
    echo "[run_movement_only] ❌ ERROR: GO2_API_TOKEN is empty (token file: $TOKEN_FILE)"
    exit 1
  fi

  echo "[run_movement_only] GO2_API_TOKEN loaded"
else
  export GO2_API_TOKEN=""
  echo "[run_movement_only] 🔓 Auth disabled (GO2_AUTH_ENABLED=0): skipping token load"
fi

# ----------------------------
# Start FastAPI backend
# ----------------------------
echo "[run_movement_only] Starting FastAPI (uvicorn) on :$API_PORT ..."
cd "$PKG_DIR"
python3 -m uvicorn app.main:app --host "$API_HOST" --port "$API_PORT" \
  > /tmp/go2_fastapi.log 2>&1 &

API_PID=$!
pids+=("$API_PID")
sleep 0.8
ok_or_die "FastAPI" "$API_PID"

# ----------------------------
# Start static file server
# ----------------------------
echo "[run_movement_only] Starting http.server on :$UI_PORT ..."
cd "$PKG_DIR"
python3 -m http.server "$UI_PORT" \
  > /tmp/go2_ui_server.log 2>&1 &

UI_PID=$!
pids+=("$UI_PID")
sleep 0.5
ok_or_die "UI server" "$UI_PID"

# ----------------------------
# Wait for Unitree sport topics
# ----------------------------
echo "[run_movement_only] Waiting for Unitree sport topics..."
for i in {1..30}; do
  if ros2 topic list 2>/dev/null | grep -q "^/api/sport/request$"; then
    echo "[run_movement_only] Unitree Sport API is up."
    break
  fi
  sleep 1
done

# ----------------------------
# Kill conflicting nodes
# ----------------------------
kill_conflicting_nodes() {
  echo "[run_movement_only] Killing conflicting motion nodes (if any)..."
  pkill -f "ros2 run go2_remote_connection web_teleop_bridge" || true
  pkill -f "ros2 run go2_remote_connection web_advanced_bridge" || true
  pkill -f "ros2 run go2_remote_connection advanced_gamepad_controller_web" || true
  pkill -f "go2_remote_connection.*web_teleop_bridge" || true
  pkill -f "go2_remote_connection.*web_advanced_bridge" || true
  pkill -f "go2_remote_connection.*advanced_gamepad_controller_web" || true
  pkill -f "ros2 run go2_remote_connection web_bridge" || true
  pkill -f "go2_remote_connection.*web_bridge" || true
  pkill -f "ros2 run go2_remote_connection move_forward_meters_node" || true
  pkill -f "go2_remote_connection.*move_forward_meters_node" || true
  sleep 0.3
}

kill_conflicting_nodes

# ----------------------------
# Start movement-related ROS nodes only
# ----------------------------
echo "[run_movement_only] Starting web_bridge ..."
ros2 run go2_remote_connection web_bridge \
  > /tmp/web_bridge.log 2>&1 &

WEB_BRIDGE_PID=$!
pids+=("$WEB_BRIDGE_PID")
sleep 0.5
ok_or_die "web_bridge" "$WEB_BRIDGE_PID"

echo "[run_movement_only] Starting move_forward_meters_node ..."
ros2 run go2_remote_connection move_forward_meters_node \
  > /tmp/move_forward_meters_node.log 2>&1 &

MOVE_FORWARD_PID=$!
pids+=("$MOVE_FORWARD_PID")
sleep 0.5
ok_or_die "move_forward_meters_node" "$MOVE_FORWARD_PID"

echo ""
echo "[run_movement_only] ✅ All movement-related processes started."

HOST_IP="$(get_best_ip)"
if [ -z "$HOST_IP" ]; then
  HOST_IP="$(hostname -I 2>/dev/null | awk '{print $1}')"
fi
HOST_IP="${HOST_IP:-127.0.0.1}"

echo "[run_movement_only] UI:  http://$HOST_IP:$UI_PORT/app/go2_movement_controller.html"
echo "[run_movement_only] API: http://$HOST_IP:$API_PORT"
echo "[run_movement_only] Press Ctrl+C to stop everything."
echo ""

wait
echo "[run_movement_only] A process exited; shutting down..."
exit 0