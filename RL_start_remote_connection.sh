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
# UI: this launcher is RL-only — it serves the standalone RL_sim_to_real
# page (/rl_sim_to_real), which carries the RL control buttons and shows
# no navigation to the other web pages. No mode selection.
# ----------------------------
MODE="rl_sim_to_real"

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
  echo "[run_all] Sourcing Unitree env: $HOME/unitree_ros2/install/setup.sh"
  source "$HOME/unitree_ros2/install/setup.sh"
elif [ -f "$HOME/unitree_ros2/install/setup.bash" ]; then
  echo "[run_all] Sourcing Unitree env: $HOME/unitree_ros2/install/setup.bash"
  source "$HOME/unitree_ros2/install/setup.bash"
else
  echo "[run_all] WARNING: Unitree env not found at $HOME/unitree_ros2/install/setup.(sh|bash)"
fi

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

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"

# ----------------------------
# Turn on/off Authentication
# ----------------------------
export GO2_AUTH_ENABLED="${GO2_AUTH_ENABLED:-0}"
export GO2_TOKEN_FILE="${GO2_TOKEN_FILE:-$HOME/go2_token}"

if [ "$GO2_AUTH_ENABLED" = "1" ] || [ "$GO2_AUTH_ENABLED" = "true" ]; then
  if [ -f "$GO2_TOKEN_FILE" ]; then
    export GO2_API_TOKEN="$(tr -d '\r\n' < "$GO2_TOKEN_FILE")"
  else
    echo "[run_all] ❌ ERROR: $GO2_TOKEN_FILE not found (GO2_AUTH_ENABLED=1)"
    exit 1
  fi

  if [ -z "${GO2_API_TOKEN:-}" ]; then
    echo "[run_all] ❌ ERROR: GO2_API_TOKEN is empty (token file: $GO2_TOKEN_FILE)"
    exit 1
  fi

  echo "[run_all] 🔐 Auth enabled"
else
  export GO2_API_TOKEN=""
  echo "[run_all] 🔓 Auth disabled (GO2_AUTH_ENABLED=0)"
fi

# ------------------------------------------------------------
# Runtime env needed by the RL policy node (interface, venv python,
# DDS config). No lidar/camera/perception nodes are started — this
# launcher runs the movement-related stack only.
# ------------------------------------------------------------
GO2_WS_DIR="$HOME/go2_ws/Go2RemoteConnection"
PKG_NAME="go2_remote_connection"

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

echo "[run_all] Using UNITREE_IFACE=$UNITREE_IFACE"
echo "[run_all] Using VENV_PYTHON=$VENV_PYTHON"

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
SPORT_READY=0
for i in {1..30}; do
  if ros2 topic list 2>/dev/null | grep -q "^/api/sport/request$"; then
    echo "[run_all] Unitree Sport API is up."
    SPORT_READY=1
    break
  fi
  sleep 1
done

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

if [ "$SPORT_READY" -eq 1 ]; then
  echo "[run_all] Starting web_bridge (for the joystick UI)"
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

  # ----------------------------
  # 2b) Low-level RL policy node (enabled by default; disable with GO2_RL_POLICY=0)
  #     Starts IDLE; only drives motors once 'RL' is selected on the joystick page.
  # ----------------------------
  if [ "${GO2_RL_POLICY:-1}" = "1" ]; then
    RL_NODE="$GO2_WS_DIR/src/$PKG_NAME/rl_policy/go2_rl_policy_node.py"
    RL_BRIDGE="$GO2_WS_DIR/src/$PKG_NAME/rl_policy/go2_rl_bridge_node.py"
    RL_EXTRA_ARGS=""
    # GO2_RL_DRY_RUN=1 -> compute obs/action and publish lowcmd with kp=kd=0 (no torque)
    [ "${GO2_RL_DRY_RUN:-0}" = "1" ] && RL_EXTRA_ARGS="--dry-run"

    # The RL controller is split into two processes because unitree_sdk2py and
    # rmw_cyclonedds cannot both own DDS domain 0 in one process (see the node's
    # header). The bridge is pure rclpy (ROS <-> localhost UDP); the controller is
    # pure SDK. Start the bridge first so its keepalive is flowing before RL engages.
    echo "[run_all] Starting go2_rl_bridge_node (ROS<->UDP bridge for the RL controller) ..."
    "$VENV_PYTHON" "$RL_BRIDGE" \
      > /tmp/go2_rl_bridge.log 2>&1 &
    RL_BRIDGE_PID=$!
    register_pid "$RL_BRIDGE_PID" "go2_rl_bridge_node" "/tmp/go2_rl_bridge.log"
    sleep 0.5
    ok_or_die "go2_rl_bridge_node" "$RL_BRIDGE_PID" "/tmp/go2_rl_bridge.log"

    echo "[run_all] Starting go2_rl_policy_node (idle until 'RL' selected in the UI) ${RL_EXTRA_ARGS}..."
    "$VENV_PYTHON" "$RL_NODE" --net "$UNITREE_IFACE" --no-prompt $RL_EXTRA_ARGS \
      > /tmp/go2_rl_policy.log 2>&1 &
    RL_PID=$!
    register_pid "$RL_PID" "go2_rl_policy_node" "/tmp/go2_rl_policy.log"
    sleep 1.5
    ok_or_die "go2_rl_policy_node" "$RL_PID" "/tmp/go2_rl_policy.log"
  else
    echo "[run_all] go2_rl_policy_node NOT started (GO2_RL_POLICY=0)."
  fi
else
  echo "[run_all] ⚠️  Unitree sport topics not found after timeout; continuing without robot motion backend."
  echo "[run_all] ⚠️  Skipping web_bridge, move_forward_meters_node and go2_rl_policy_node because Unitree sport topics are unavailable."
fi

echo ""
echo "[run_all] ✅ Frontend/backend started."

HOST_IP="$(get_best_ip || true)"
if [ -z "$HOST_IP" ]; then
  HOST_IP="$(hostname -I 2>/dev/null | awk '{print $1}')"
fi
HOST_IP="${HOST_IP:-127.0.0.1}"

echo "[run_all] UI:   http://$HOST_IP:$API_PORT/rl_sim_to_real   (standalone RL joystick page)"
echo "[run_all] API:  http://$HOST_IP:$API_PORT"
echo "[run_all] Press Ctrl+C to stop everything."
echo ""

echo "See logs with:"
echo "  sed -n '1,200p' /tmp/go2_fastapi.log"
echo "  sed -n '1,200p' /tmp/web_bridge.log"
echo "  sed -n '1,200p' /tmp/move_forward_meters.log"
echo "  sed -n '1,200p' /tmp/go2_rl_bridge.log"
echo "  sed -n '1,200p' /tmp/go2_rl_policy.log"

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