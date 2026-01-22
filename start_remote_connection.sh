#!/usr/bin/env bash
set -eo pipefail
# NOTE: we intentionally do NOT enable 'set -u' until after sourcing ROS

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$WS_DIR/src/p2_remote_connection"

API_HOST="0.0.0.0"
API_PORT="8000"
UI_PORT="8081"

# Make sure HOME is set correctly under systemd
export HOME="/home/unitree"

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

echo "[run_all] Workspace: $WS_DIR"

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
# Use whichever exists on your system:
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

# If you have a CycloneDDS config file, set it explicitly (optional)
# export CYCLONEDDS_URI="file://$HOME/unitree_ros2/cyclonedds.xml"

# ----------------------------
# Load Go2 API token
# ----------------------------
if [ -f "$HOME/.go2_token" ]; then
  export GO2_API_TOKEN="$(cat "$HOME/.go2_token")"
else
  echo "[run_all] ❌ ERROR: ~/.go2_token not found"
  exit 1
fi

echo "[run_all] GO2_API_TOKEN loaded"


# 1) Start FastAPI backend
echo "[run_all] Starting FastAPI (uvicorn) on :$API_PORT ..."
cd "$PKG_DIR"
python3 -m uvicorn app.main:app --host "$API_HOST" --port "$API_PORT" &
pids+=("$!")
API_PID=$!
sleep 0.5
if ! kill -0 "$API_PID" 2>/dev/null; then
  echo "[run_all] ❌ FastAPI failed to start (check logs above)."
  exit 1
fi

# 2) Start static file server for frontend
echo "[run_all] Starting http.server on :$UI_PORT ..."
python3 -m http.server "$UI_PORT" &
pids+=("$!")


# 3) Wait for unitree sport topics before starting bridge (prevents “no motion after boot”)
echo "[run_all] Waiting for Unitree sport topics..."
for i in {1..30}; do
  if ros2 topic list 2>/dev/null | grep -q "^/api/sport/request$"; then
    echo "[run_all] sport API is up."
    break
  fi
  sleep 1
done

# 4) Start bridge node (prefer launching the binary directly)
BRIDGE_BIN="$WS_DIR/install/p2_remote_connection/lib/p2_remote_connection/web_teleop_bridge"
if [ -x "$BRIDGE_BIN" ]; then
  echo "[run_all] Starting bridge: $BRIDGE_BIN"
  "$BRIDGE_BIN" &
  pids+=("$!")
else
  echo "[run_all] Starting ROS2 bridge via ros2 run (fallback)"
  cd "$WS_DIR"
  ros2 run p2_remote_connection web_teleop_bridge &
  pids+=("$!")
fi

echo ""
echo "[run_all] ✅ All started."
echo "[run_all] UI:  http://<this-machine-ip>:$UI_PORT/app/go2_joystick.html"
echo "[run_all] API: http://<this-machine-ip>:$API_PORT"
echo "[run_all] Press Ctrl+C to stop everything."
echo ""

wait -n
echo "[run_all] A process exited; shutting down..."
exit 0

