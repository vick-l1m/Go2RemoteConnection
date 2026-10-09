#!/usr/bin/env bash
set -eo pipefail
# NOTE: we intentionally do NOT enable 'set -u' until after sourcing ROS

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# If the script is in workspace root:
if [ -d "$WS_DIR/src/go2_remote_controller" ]; then
  PKG_DIR="$WS_DIR/src/go2_remote_controller"
# If the script is inside the package (e.g. .../src/go2_remote_controller):
elif [ -d "$WS_DIR/app" ] && [ -d "$WS_DIR/src" ]; then
  PKG_DIR="$WS_DIR"
else
  echo "[run_all] ❌ Can't locate go2_remote_controller package from $WS_DIR"
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
    [ -n "$pid" ] || continue
    if kill -0 "$pid" 2>/dev/null; then
      kill "$pid" 2>/dev/null || true
    fi
  done
  # `ros2 launch` (perception: realsense driver + heightmap_node) needs a few seconds
  # to shut its children down cleanly -- SIGKILLing the launch process after 0.5 s
  # orphans realsense2_camera_node, which then keeps the camera open and blocks the
  # next start. Non-critical pids get up to 5 s; everything else keeps the quick path.
  local deadline=$((SECONDS + 5))
  while [ "$SECONDS" -lt "$deadline" ]; do
    local waiting=0
    for pid in "${pids[@]:-}"; do
      [ -n "$pid" ] || continue
      if [ "${pid_critical[$pid]:-1}" = "0" ] && kill -0 "$pid" 2>/dev/null; then
        waiting=1
      fi
    done
    [ "$waiting" -eq 0 ] && break
    sleep 0.25 || true
  done
  sleep 0.5 || true
  for pid in "${pids[@]:-}"; do
    [ -n "$pid" ] || continue
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
  echo "[run_all] WARNING: no go2_remote_controller overlay found (did you colcon build?)"
fi

# RealSense: the Jetson's working realsense2_camera is a SOURCE build in ~/ros2_ws
# (4.57.6 against the installed librealsense 2.57); /opt/ros/humble's apt node dies on
# librealsense2.so.2.58. Sourcing it AFTER /opt/ros makes it win the ament lookup (same
# as robot_env.sh). Guarded, so a laptop without it is unaffected.
if [ -f "$HOME/ros2_ws/install/setup.bash" ]; then
  echo "[run_all] Sourcing ~/ros2_ws overlay (realsense2_camera)"
  source "$HOME/ros2_ws/install/setup.bash"
fi

# The OUTER Go2_RL_workflow workspace, if this checkout is a submodule of it. It builds
# go2_msgs (the HeightScan type) and go2_perception (heightmap_node) -- neither of which
# this submodule contains. Without it go2_rl_bridge_node cannot import go2_msgs, forwards
# no height scan, and every PERCEPTION policy refuses to engage; blind policies are
# unaffected either way. Sourcing an overlay only puts packages on the path, it starts
# nothing, so this is safe when perception is not in use.
for outer in "$WS_DIR/../install/setup.bash" "$HOME/Go2_RL_workflow/install/setup.bash"; do
  if [ -f "$outer" ]; then
    echo "[run_all] Sourcing outer workspace overlay (go2_msgs/go2_perception): $outer"
    source "$outer"
    break
  fi
done

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
# The RL controller + bridge run as raw Python from the source tree (not via
# `ros2 run`), so anchor their paths to THIS checkout's package dir ($PKG_DIR,
# auto-detected above). This makes the launcher work both inside the
# Go2_RL_workflow project and as a standalone Go2RemoteConnection repo, wherever
# it is cloned -- no hard-coded ~/go2_ws path.
# To force the RL nodes to run from a DIFFERENT checkout, set GO2_WS_DIR to that
# repo root (the dir containing src/go2_remote_controller) before launching.
PKG_NAME="go2_remote_controller"
if [ -n "${GO2_WS_DIR:-}" ]; then
  RL_PKG_DIR="$GO2_WS_DIR/src/$PKG_NAME"
else
  RL_PKG_DIR="$PKG_DIR"
fi

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
# 0) Perception: RealSense D435i + height map, auto-started when a camera is on USB
# ----------------------------
# The rough-terrain (uses_heightmap) policies need /go2/height_scan, built by the outer
# workspace's go2_perception heightmap_node from the RealSense. Which launch is right
# depends on the USB link the camera landed on:
#   USB 3 (>= 5000 Mb/s)  real_perception.launch.py       depth + colour + driver cloud
#   USB 2 (480 Mb/s)      real_perception_usb2.launch.py  depth ONLY; heightmap_node
#                                                         back-projects the depth image
#                                                         (the colour+cloud path stalls
#                                                         and drops the camera off the bus)
# Presence + speed come from sysfs (never opens the device, so no fight with the
# driver); the web page's /perception/status reads the same sysfs independently.
#   GO2_PERCEPTION=auto (default) detect + pick | usb3 | usb2 force a launch | 0 never
#   GO2_HEIGHT_EMPTY_FILL=-1.0    value for cells the camera cannot see. MUST equal the
#                                 selected policy's deploy.yaml unobserved_value (-1.0
#                                 for recent exports, 0.0 for older ones); the policy
#                                 node logs a mismatch but runs anyway.
GO2_PERCEPTION="${GO2_PERCEPTION:-auto}"
GO2_HEIGHT_EMPTY_FILL="${GO2_HEIGHT_EMPTY_FILL:--1.0}"
PERCEPTION_STARTED=0
PERCEPTION_LOG="/tmp/go2_perception.log"

RS_FOUND=0; RS_SPEED_MBPS=""; RS_USB=""; RS_PRODUCT=""
# detect_realsense: sysfs scan. Sets RS_FOUND/RS_USB(2|3)/RS_SPEED_MBPS/RS_PRODUCT.
# Mirror of app/services/realsense_usb.py -- keep the two in step.
detect_realsense() {
  RS_FOUND=0; RS_SPEED_MBPS=""; RS_USB=""; RS_PRODUCT=""
  local d prod speed
  for d in /sys/bus/usb/devices/*; do
    [ -f "$d/idVendor" ] || continue
    [ "$(cat "$d/idVendor" 2>/dev/null)" = "8086" ] || continue
    prod="$(cat "$d/product" 2>/dev/null || true)"
    case "$prod" in *[Rr]eal[Ss]ense*) ;; *) continue ;; esac
    speed="$(cat "$d/speed" 2>/dev/null || true)"
    RS_FOUND=1; RS_PRODUCT="$prod"; RS_SPEED_MBPS="${speed%%.*}"
    if [ -n "$RS_SPEED_MBPS" ] && [ "$RS_SPEED_MBPS" -ge 5000 ] 2>/dev/null; then
      RS_USB=3
    else
      RS_USB=2
    fi
    return 0
  done
  return 1
}

# The VIP-Rescue vip-realsense container (restart=unless-stopped, so it comes up on
# every boot) opens the camera itself and publishes under /camera/*, NOT /go2/camera/*.
# Reusing it is impossible: heightmap_node would subscribe a depth topic nobody
# publishes. It has to be stopped to free the camera.
vip_realsense_running() {
  command -v docker >/dev/null 2>&1 && docker ps --format '{{.Names}}' 2>/dev/null | grep -q '^vip-realsense$'
}

# Something already owns the camera under OUR topic names? (a previous RL_start or a
# manual launch) -- only ONE process may open a RealSense.
realsense_driver_running() {
  timeout 6 ros2 topic list 2>/dev/null | grep -q '^/go2/camera/depth/image_rect_raw$'
}

# A MESSAGE must arrive, not just the topic exist: heightmap_node advertises
# /go2/height_scan the moment it starts, depth or no depth, so `ros2 topic list`
# reported "publishing" for a scan that never carried a single frame.
height_scan_flowing() {
  timeout 6 ros2 topic echo --once /go2/height_scan go2_msgs/msg/HeightScan \
    --field header.stamp >/dev/null 2>&1
}

# Launch files are run BY PATH: go2_bringup depends on the sim/controller packages and
# need not be built on the robot, which is exactly why both launches take camera_config.
PERCEPTION_LAUNCH_DIR=""
for cand in "$WS_DIR/../src/go2_bringup/launch" "$HOME/Go2_RL_workflow/src/go2_bringup/launch"; do
  if [ -f "$cand/real_perception.launch.py" ]; then
    PERCEPTION_LAUNCH_DIR="$(cd "$cand" && pwd)"
    break
  fi
done
CAMERA_YAML=""
[ -n "$PERCEPTION_LAUNCH_DIR" ] && CAMERA_YAML="$PERCEPTION_LAUNCH_DIR/../config/camera.yaml"

# start_heightmap_only <usb 2|3>: the driver is already up (someone else's), so add only
# the missing half: heightmap_node (+ the base->camera_link mount TF from camera.yaml).
start_heightmap_only() {
  local usb="$1" mode tf_vals
  if [ "$usb" = "3" ]; then mode="pointcloud"; else mode="depth_image"; fi
  echo "[run_all] RealSense driver already running -> starting heightmap_node only (input_mode=$mode)"
  tf_vals="$(python3 - "$CAMERA_YAML" <<'PY' 2>/dev/null || true
import math, sys, yaml
c = (yaml.safe_load(open(sys.argv[1])) or {}).get("camera", {})
t = c.get("translation", [0.28, 0.0, 0.10])
print(t[0], t[1], t[2],
      math.radians(float(c.get("roll_deg", 0.0))),
      math.radians(float(c.get("pitch_deg", 30.0))),
      math.radians(float(c.get("yaw_deg", 0.0))))
PY
)"
  if [ -n "$tf_vals" ]; then
    # shellcheck disable=SC2086
    set -- $tf_vals
    ros2 run tf2_ros static_transform_publisher --x "$1" --y "$2" --z "$3" \
      --roll "$4" --pitch "$5" --yaw "$6" --frame-id base --child-frame-id camera_link \
      > /tmp/go2_camera_tf.log 2>&1 &
    register_pid "$!" "camera mount TF" "/tmp/go2_camera_tf.log" 0
    echo "[run_all] Published base->camera_link mount TF from $CAMERA_YAML (assumed; the running driver does not publish it)"
  else
    echo "[run_all] ⚠️  Could not read $CAMERA_YAML for the mount TF; heightmap_node will wait on TF."
  fi
  ros2 run go2_perception heightmap_node --ros-args \
    -p input_mode:="$mode" \
    -p pointcloud_topic:=/go2/camera/depth/color/points \
    -p depth_topic:=/go2/camera/depth/image_rect_raw \
    -p camera_info_topic:=/go2/camera/depth/camera_info \
    -p base_frame:=base -p size_x:=1.6 -p size_y:=1.0 -p resolution:=0.1 -p center_x:=0.6 \
    -p empty_fill:="$GO2_HEIGHT_EMPTY_FILL" -p gridmap_every_n:=3 \
    > "$PERCEPTION_LOG" 2>&1 &
  register_pid "$!" "heightmap_node" "$PERCEPTION_LOG" 0
  PERCEPTION_STARTED=1
  sleep 2.0
  warn_if_dead "heightmap_node" "$!" "$PERCEPTION_LOG" || true
}

start_perception() {
  local usb="$1" launch_file
  if [ "$usb" = "3" ]; then
    launch_file="real_perception.launch.py"
  else
    launch_file="real_perception_usb2.launch.py"
  fi
  echo "[run_all] Launching $launch_file (empty_fill=$GO2_HEIGHT_EMPTY_FILL -- must match the policy's unobserved_value)"
  ros2 launch "$PERCEPTION_LAUNCH_DIR/$launch_file" \
    camera_config:="$CAMERA_YAML" rviz:=false empty_fill:="$GO2_HEIGHT_EMPTY_FILL" \
    > "$PERCEPTION_LOG" 2>&1 &
  local pid=$!
  register_pid "$pid" "perception (realsense USB $usb + heightmap)" "$PERCEPTION_LOG" 0
  PERCEPTION_STARTED=1
  sleep 3.0
  warn_if_dead "perception ($launch_file)" "$pid" "$PERCEPTION_LOG" || true
}

if [ "$GO2_PERCEPTION" = "0" ]; then
  echo "[run_all] Perception disabled (GO2_PERCEPTION=0)."
else
  if detect_realsense; then
    if [ "$RS_USB" = "3" ]; then
      echo "[run_all] RealSense on USB 3 ($RS_SPEED_MBPS Mb/s): $RS_PRODUCT"
    else
      echo "[run_all] ⚠️  RealSense on USB 2 ($RS_SPEED_MBPS Mb/s): $RS_PRODUCT -- depth-only mode; use a USB 3 port + cable for colour + cloud"
    fi
  else
    echo "[run_all] No RealSense on USB."
  fi

  PERCEPTION_USB=""
  case "$GO2_PERCEPTION" in
    usb3) PERCEPTION_USB=3 ;;
    usb2) PERCEPTION_USB=2 ;;
    auto) [ "$RS_FOUND" -eq 1 ] && PERCEPTION_USB="$RS_USB" ;;
    *) echo "[run_all] ⚠️  Unknown GO2_PERCEPTION='$GO2_PERCEPTION' (auto|usb3|usb2|0); treating as auto."
       [ "$RS_FOUND" -eq 1 ] && PERCEPTION_USB="$RS_USB" ;;
  esac

  if [ -n "$PERCEPTION_USB" ] && vip_realsense_running; then
    if [ "${GO2_STOP_VIP_REALSENSE:-0}" = "1" ]; then
      echo "[run_all] Stopping the vip-realsense container to free the camera (GO2_STOP_VIP_REALSENSE=1)."
      echo "[run_all]   It stays stopped until reboot; 'docker start vip-realsense' brings it back."
      docker stop vip-realsense >/dev/null 2>&1 || true
      sleep 2
    else
      echo "[run_all] ❌ The vip-realsense Docker container owns the RealSense. It publishes /camera/*,"
      echo "[run_all]    not /go2/camera/*, so the height map would get no depth. Perception SKIPPED."
      echo "[run_all]    Free the camera:  docker stop vip-realsense   (or rerun with GO2_STOP_VIP_REALSENSE=1)"
      PERCEPTION_USB=""
    fi
  fi

  if [ -z "$PERCEPTION_USB" ]; then
    echo "[run_all] Perception not started (blind policies only)."
  elif [ -z "$PERCEPTION_LAUNCH_DIR" ] || [ ! -f "$CAMERA_YAML" ]; then
    echo "[run_all] ⚠️  Perception launch files not found (looked beside $WS_DIR and in ~/Go2_RL_workflow/src/go2_bringup/launch); skipping."
  elif ! ros2 pkg prefix realsense2_camera >/dev/null 2>&1; then
    echo "[run_all] ⚠️  realsense2_camera not found in this ROS environment (source ~/ros2_ws, see robot_env.sh); perception skipped."
  elif ! ros2 pkg prefix go2_perception >/dev/null 2>&1; then
    echo "[run_all] ⚠️  go2_perception not found (build the outer Go2_RL_workflow workspace); perception skipped."
  elif realsense_driver_running; then
    if height_scan_flowing; then
      echo "[run_all] Perception already running (driver + /go2/height_scan) -- reusing it."
    else
      start_heightmap_only "$PERCEPTION_USB"
    fi
  else
    start_perception "$PERCEPTION_USB"
  fi
fi

# ----------------------------
# Session recording (GO2_RECORD=1)
# ----------------------------
# Two halves, one folder (sessions/<UTC>_rl/):
#   flight/  the policy node's own per-step log -- the exact obs vector, raw and
#            commanded actions, scan + scan age, events (flight_recorder.py). Written
#            only while the policy drives. This is what the evaluation replays.
#   bag/     rosbag2 of robot state, motor commands, height scan + map, TF, operator
#            input, logs -- for RViz replay and anything the policy did not see.
# Evaluate afterwards (on the PC):
#   python3 src/go2_remote_controller/tools/eval_rl_flight.py sessions/<...>/flight/<run>
# GO2_RECORD_DEPTH=1 also bags the raw depth/cloud (~14 MB/s; mind the Jetson's disk).
RECORD_SESSION_DIR=""
if [ "${GO2_RECORD:-0}" = "1" ]; then
  RECORD_SESSION_DIR="$WS_DIR/sessions/$(date -u +%Y%m%d-%H%M%S)_rl"
  mkdir -p "$RECORD_SESSION_DIR/flight"
  export GO2_RL_RECORD_DIR="$RECORD_SESSION_DIR/flight"
  BAG_TOPICS="/lowstate /lowcmd /go2/height_scan /go2/local_heightmap /go2/mount_monitor \
/tf /tf_static /go2/camera/depth/camera_info /web_teleop /web_control_mode /web_estop \
/web_rl_policy /web_rl_active_policy /rosout"
  if [ "${GO2_RECORD_DEPTH:-0}" = "1" ]; then
    BAG_TOPICS="$BAG_TOPICS /go2/camera/depth/image_rect_raw /go2/camera/depth/color/points"
  fi
  # rosbag2 finalises its metadata on SIGINT; cleanup() sends SIGTERM. The wrapper
  # translates, so stopping RL_start leaves a bag that opens without a reindex.
  # shellcheck disable=SC2086
  bash -c 'ros2 bag record -o "$1" $2 & b=$!; trap "kill -INT $b 2>/dev/null; wait $b" TERM INT; wait $b' \
    _ "$RECORD_SESSION_DIR/bag" "$BAG_TOPICS" > /tmp/go2_record.log 2>&1 &
  register_pid "$!" "session bag" "/tmp/go2_record.log" 0
  echo "[run_all] 🔴 Recording session -> $RECORD_SESSION_DIR (bag + policy flight log)"
fi

# ----------------------------
# 1) Start FastAPI backend
# ----------------------------
echo "[run_all] Starting FastAPI (uvicorn) on :$API_PORT ..."
cd "$PKG_DIR"
if ! python3 -c "import rclpy" >/dev/null 2>&1; then
  echo "[run_all] ❌ 'python3' ($(command -v python3)) can't import rclpy." >&2
  echo "[run_all]    A conda env (e.g. env_isaaclab) is likely active in this shell and is" >&2
  echo "[run_all]    shadowing the system Python 3.10 that ROS 2 Humble's rclpy needs." >&2
  echo "[run_all]    Run 'conda deactivate' (possibly more than once) and try again." >&2
  exit 1
fi
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
  pkill -f "ros2 run go2_remote_controller web_teleop_bridge" || true
  pkill -f "ros2 run go2_remote_controller web_advanced_bridge" || true
  pkill -f "ros2 run go2_remote_controller advanced_gamepad_controller_web" || true
  pkill -f "go2_remote_controller.*web_teleop_bridge" || true
  pkill -f "go2_remote_controller.*web_advanced_bridge" || true
  pkill -f "go2_remote_controller.*advanced_gamepad_controller_web" || true
  pkill -f "ros2 run go2_remote_controller web_bridge" || true
  pkill -f "go2_remote_controller.*web_bridge" || true
  sleep 0.3
}

kill_conflicting_nodes

if [ "$SPORT_READY" -eq 1 ]; then
  echo "[run_all] Starting web_bridge (for the joystick UI)"
  ros2 run go2_remote_controller web_bridge > /tmp/web_bridge.log 2>&1 &
  WEB_BRIDGE_PID=$!
  register_pid "$WEB_BRIDGE_PID" "web_bridge" "/tmp/web_bridge.log"

  echo "[run_all] Starting move_forward_meters_node ..."
  ros2 run go2_remote_controller move_forward_meters_node > /tmp/move_forward_meters.log 2>&1 &
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
    RL_NODE="$RL_PKG_DIR/rl_policy/go2_rl_policy_node.py"
    RL_BRIDGE="$RL_PKG_DIR/rl_policy/go2_rl_bridge_node.py"
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

if [ "$PERCEPTION_STARTED" -eq 1 ]; then
  echo "[run_all] Waiting for /go2/height_scan ..."
  SCAN_OK=0
  # Each check blocks up to 6 s waiting for a message, so 5 tries is ~35 s worst case.
  for i in {1..5}; do
    if height_scan_flowing; then SCAN_OK=1; break; fi
    sleep 1
  done
  if [ "$SCAN_OK" -eq 1 ]; then
    echo "[run_all] /go2/height_scan is publishing -- perception policies can engage."
  else
    echo "[run_all] ⚠️  No /go2/height_scan message received; perception policies will refuse to engage until it flows."
    echo "[run_all]    Usual causes: no depth reaching heightmap_node, or no /lowstate attitude (it drops"
    echo "[run_all]    frames rather than publish an unlevelled scan) -- the log below says which."
    print_log_hint "$PERCEPTION_LOG"
  fi
fi

echo ""
echo "[run_all] ✅ Frontend/backend started."
if [ -n "$RECORD_SESSION_DIR" ]; then
  echo "[run_all] 🔴 Recording to $RECORD_SESSION_DIR -- policy steps are logged while RL drives."
fi

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
echo "  sed -n '1,200p' /tmp/go2_perception.log"

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