#!/usr/bin/env bash
# Start the RL half of the VIP-Rescue "RL drive" nav stack on the robot host.
#
# This is RL_start_remote_connection.sh with the settings the omni30 mem6 usb2 policy
# needs, so the nav-side adapter (go2_control_cpp::CmdVelToGo2, inside vip-web) can
# engage it. Nothing here is new: the controller, bridge, perception, calibration and
# recording are the unchanged RL stack. The nav adapter selects the policy itself
# (/web_rl_policy) and switches modes (/web_control_mode); no web page needed.
#
# Usage:  ./nav_rl_start.sh [VAR=value ...]        (same VAR=value settings as RL_start)
#   e.g.  ./nav_rl_start.sh GO2_RL_DRY_RUN=1       gantry dry run, no torque
#         ./nav_rl_start.sh GO2_RECORD=1 GO2_RECORD_NAME=nav_rl
#         ./nav_rl_start.sh GO2_STEPFIELD_PRESET=half_cubic
#
# Prerequisites on the robot:
#   * vip-web + vip-livox running (docker compose up -d livox web). NOT vip-realsense:
#     this script stops it (GO2_STOP_VIP_REALSENSE=1) because the policy's height scan
#     needs the camera under /go2/camera/*.
#   * the outer Go2_RL_workflow workspace built + sourced for go2_msgs/go2_perception
#     (RL_start handles sourcing; see its header).
set -eo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Defaults for THIS policy (deploy.yaml: camera_profile usb2, stepfield snap levels
# 0/0.15/0.3, 6 s rolling-map memory, unobserved_value -1.0). Any VAR=value argument
# overrides the matching default.
DEFAULTS=(
  GO2_PERCEPTION=usb2
  GO2_HEIGHTMAP_MODE=stepfield
  GO2_MAP=1
  GO2_STOP_VIP_REALSENSE=1
  GO2_CALIBRATE=1
  GO2_HEIGHT_EMPTY_FILL=-1.0
  GO2_STEPFIELD_PRESET=lab_blocks
)

declare -A given
for arg in "$@"; do
  case "$arg" in
    -h|--help)
      sed -n '2,20p' "$0" | sed 's/^# \{0,1\}//'
      echo; echo "--- RL_start_remote_connection.sh settings ---"
      exec "$HERE/RL_start_remote_connection.sh" --help ;;
    [A-Za-z_]*=*) given["${arg%%=*}"]=1 ;;
  esac
done

args=()
for d in "${DEFAULTS[@]}"; do
  [ -z "${given[${d%%=*}]:-}" ] && args+=("$d")
done
args+=("$@")

cat <<EOF
[nav_rl] Starting the RL stack for nav drive with: ${args[*]}
[nav_rl]
[nav_rl] Then, inside vip-web:
[nav_rl]   ros2 launch go2_control_cpp minimal_bt_launch_rl.py use_sim_time:=false
[nav_rl] Switch the nav stack's drive backend (from any ROS shell on domain 0):
[nav_rl]   ros2 topic pub -1 /go2/drive_backend std_msgs/String "{data: rl}"
[nav_rl]   ros2 topic pub -1 /go2/drive_backend std_msgs/String "{data: sport}"
[nav_rl]   ros2 topic pub -1 /go2/drive_estop   std_msgs/Bool   "{data: true}"
[nav_rl]   ros2 topic echo /go2/drive_status
[nav_rl]
EOF

exec "$HERE/RL_start_remote_connection.sh" "${args[@]}"
