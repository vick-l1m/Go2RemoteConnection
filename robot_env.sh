# robot_env.sh — DDS + workspace environment for talking to the REAL Go2.
#
# Source this in EVERY fresh shell (each SSH session) BEFORE running standalone
# ros2 commands against the live robot — `ros2 launch go2_bringup ...`,
# `ros2 topic list/echo/hz`, `ros2 bag record`, or the record_*.sh scripts.
#
# It mirrors the DDS block of RL_start_remote_connection.sh so your ad-hoc shells
# land on the SAME domain / RMW / network interface as the robot stack. Without
# it, two shells (or the launch vs. your query shell) end up on different DDS
# settings, can't discover each other, and `ros2 topic list` shows nothing even
# though the nodes are publishing.
#
# This is the ROBOT env. It is NOT the same as recording/viewer_env.sh, which is
# for local bag replay on a laptop (loopback `lo`). Do not mix them.
#
# Usage (every new terminal):
#   source ~/Go2_RL_workflow/Go2RemoteConnection/robot_env.sh

set +u   # ROS setup files reference unbound vars; disable nounset while sourcing

# 1. ROS 2 + overlays — so the `ros2` CLI and the unitree_go / go2_* message
#    types exist in this shell (deserialising /lowstate etc. needs them).
[ -f /opt/ros/humble/setup.bash ] && source /opt/ros/humble/setup.bash
[ -f "$HOME/unitree_ros2/install/setup.bash" ] && source "$HOME/unitree_ros2/install/setup.bash"
[ -f "$HOME/Go2_RL_workflow/go2_rl_workflow/install/setup.bash" ] \
    && source "$HOME/Go2_RL_workflow/go2_rl_workflow/install/setup.bash"

# 2. DDS domain + vendor — MUST match the robot stack, or discovery fails.
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"

# 3. Pin CycloneDDS to the NIC that carries the Go2 traffic (same hostname logic
#    as RL_start_remote_connection.sh). Override by pre-setting UNITREE_IFACE.
if [ -z "${UNITREE_IFACE:-}" ]; then
  _host="$(hostname | tr '[:upper:]' '[:lower:]')"
  case "$_host" in
    *unitree*|*jetson*|go2*) UNITREE_IFACE="enP8p1s0" ;;   # the Jetson's Go2 NIC
    *) UNITREE_IFACE="$(ip route get 192.168.123.161 2>/dev/null \
         | awk '/dev/{for(i=1;i<=NF;i++) if($i=="dev"){print $(i+1); exit}}')"
       UNITREE_IFACE="${UNITREE_IFACE:-enp0s31f6}" ;;       # laptop fallback
  esac
fi
export UNITREE_IFACE
export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"${UNITREE_IFACE}\" priority=\"default\" multicast=\"default\" /></Interfaces></General></Domain></CycloneDDS>"

# 4. Reset the ros2 CLI daemon so `ros2 topic/node list` re-reads THIS config
#    instead of a cached view from an earlier (mismatched) shell.
# Bounded so a wedged daemon or slow ros2-CLI entry-point scan can't hang the
# source. If it times out the daemon still gets replaced on the next ros2 call.
timeout 15 ros2 daemon stop >/dev/null 2>&1 || true

echo "[robot_env] ROS_DOMAIN_ID=$ROS_DOMAIN_ID  RMW=$RMW_IMPLEMENTATION  UNITREE_IFACE=$UNITREE_IFACE  (daemon reset)"
