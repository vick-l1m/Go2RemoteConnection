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
    echo "  $0                # serve joystick UI + SIM bridge"
    echo "  $0 joystick       # serve joystick UI + SIM bridge"
    echo "  $0 terminal       # serve terminal-only UI (no bridge)"
    echo "  $0 movement       # serve movement UI + SIM bridge"
    exit 1
    ;;
esac

pids=()
declare -A pid_names
declare -A pid_logs
declare -A pid_critical
declare -A pid_reported

# register_pid <pid> <name> [log_path] [critical]
#   critical=1 (default): if this process dies, tear the whole stack down.
#   critical=0          : non-essential (e.g. lidar/camera); warn but keep running.
register_pid() {
  local pid="$1"
  local name="$2"
  local log_path="${3:-}"
  local critical="${4:-1}"
  pids+=("$pid")
  pid_names["$pid"]="$name"
  pid_logs["$pid"]="$log_path"
  pid_critical["$pid"]="$critical"
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

# warn_if_dead: for non-essential nodes (lidar/camera). Warns but never exits.
warn_if_dead() {
  local name="$1"
  local pid="$2"
  local log_path="${3:-}"

  if ! kill -0 "$pid" 2>/dev/null; then
    echo "[run_all] ⚠️  $name failed to start — continuing without it (non-essential)."
    print_log_hint "$log_path"
    return 1
  fi
  return 0
}

echo "[run_all] Workspace: $WS_DIR"
echo "[run_all] Mode: $MODE"

# --- Source ROS + overlay safely (SIM: skip Unitree stack) ---
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

# SIM: intentionally skip Unitree ROS2 stack

# The colcon-built overlay may live in this repo or in the dedicated
# build workspace (~/go2_ws/Go2RemoteConnection). Try both.
OVERLAY_SOURCED=0
for overlay in "$WS_DIR/install/setup.bash" "$HOME/go2_ws/Go2RemoteConnection/install/setup.bash"; do
  if [ -f "$overlay" ]; then
    echo "[run_all] Sourcing overlay: $overlay"
    source "$overlay"
    OVERLAY_SOURCED=1
    break
  fi
done
if [ "$OVERLAY_SOURCED" -eq 0 ]; then
  echo "[run_all] WARNING: no go2_remote_connection overlay found (did you colcon build?)"
fi

set -u

# SIM: use FastRTPS and avoid CycloneDDS interface pinning
unset CYCLONEDDS_URI
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

# ----------------------------
# Auth: always disabled for sim (no token required)
# ----------------------------
export GO2_AUTH_ENABLED="0"
export GO2_API_TOKEN=""
echo "[run_all] 🔓 Auth disabled (sim mode)"

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
register_pid "$FLATTEN_PID" "flatten_l1_data" "/tmp/flatten_l1_data.log" 0

sleep 0.3
warn_if_dead "flatten_l1_data (lidar)" "$FLATTEN_PID" "/tmp/flatten_l1_data.log" || true

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

kill_conflicting_nodes() {
  echo "[run_all] Killing conflicting motion nodes (if any)..."
  pkill -f "ros2 run go2_remote_connection web_teleop_bridge_sim" || true
  pkill -f "go2_remote_connection.*web_teleop_bridge_sim" || true
  pkill -f "ros2 run go2_remote_connection web_teleop_bridge" || true
  pkill -f "go2_remote_connection.*web_teleop_bridge" || true
  pkill -f "ros2 run go2_remote_connection web_bridge" || true
  pkill -f "go2_remote_connection.*web_bridge" || true
  sleep 0.3
}

kill_conflicting_nodes

# ----------------------------
# 2) Start SIM bridge node
# ----------------------------
# web_teleop_bridge_sim is standalone — it only needs the web UI topics and
# publishes /robot{N}/cmd_vel. No simulator node needs to be running first.
if [ "$MODE" = "terminal" ]; then
  echo "[run_all] Terminal mode: skipping motion nodes ✅"
else
  echo "[run_all] Starting web_teleop_bridge_sim..."
  SIM_BRIDGE_BIN="$WS_DIR/install/go2_remote_connection/lib/go2_remote_connection/web_teleop_bridge_sim"
  if [ -x "$SIM_BRIDGE_BIN" ]; then
    "$SIM_BRIDGE_BIN" --ros-args -p robot_index:=0 > /tmp/web_teleop_bridge_sim.log 2>&1 &
  else
    ros2 run go2_remote_connection web_teleop_bridge_sim --ros-args -p robot_index:=0 \
      > /tmp/web_teleop_bridge_sim.log 2>&1 &
  fi
  SIM_BRIDGE_PID=$!
  register_pid "$SIM_BRIDGE_PID" "web_teleop_bridge_sim" "/tmp/web_teleop_bridge_sim.log"

  sleep 0.5
  ok_or_die "web_teleop_bridge_sim" "$SIM_BRIDGE_PID" "/tmp/web_teleop_bridge_sim.log"

  # ----------------------------
  # 3) Start cmd_vel file relay (for Isaac Lab / Python 3.11 which can't load rclpy)
  # ----------------------------
  CMD_VEL_RELAY="$PKG_DIR/src/cmd_vel_file_relay.py"
  if [ -f "$CMD_VEL_RELAY" ]; then
    echo "[run_all] Starting cmd_vel_file_relay -> /tmp/go2_cmd_vel ..."
    python3 "$CMD_VEL_RELAY" --output /tmp/go2_cmd_vel \
      > /tmp/cmd_vel_file_relay.log 2>&1 &
    RELAY_PID=$!
    register_pid "$RELAY_PID" "cmd_vel_file_relay" "/tmp/cmd_vel_file_relay.log"
    sleep 0.3
    ok_or_die "cmd_vel_file_relay" "$RELAY_PID" "/tmp/cmd_vel_file_relay.log"
  fi
fi

echo ""
echo "[run_all] ✅ Frontend/backend started."

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
echo "  sed -n '1,200p' /tmp/web_teleop_bridge_sim.log"
echo "  sed -n '1,200p' /tmp/cmd_vel_file_relay.log"

set +e

while true; do
  for pid in "${pids[@]}"; do
    if ! kill -0 "$pid" 2>/dev/null; then
      # Only report each dead pid once.
      [ -n "${pid_reported[$pid]:-}" ] && continue
      wait "$pid" 2>/dev/null
      rc=$?
      name="${pid_names[$pid]:-Child process}"
      log_path="${pid_logs[$pid]:-}"
      pid_reported["$pid"]=1

      if [ "${pid_critical[$pid]:-1}" = "1" ]; then
        echo "[run_all] ❌ $name exited with code $rc — shutting down."
        print_log_hint "$log_path"
        exit "$rc"
      else
        echo "[run_all] ⚠️  $name (non-essential) exited with code $rc — continuing without it."
        print_log_hint "$log_path"
      fi
    fi
  done
  sleep 1
done
