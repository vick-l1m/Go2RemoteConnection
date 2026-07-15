# viewer_env.sh — DDS config for LOCAL rosbag replay + RViz (no robot attached).
#
# The unitree_ros2 overlay sets CYCLONEDDS_URI to bind DDS to the robot's wired
# NIC (enp0s31f6). With no robot connected that interface can't carry traffic, so
# local nodes (bag player -> lowstate_to_jointstate -> robot_state_publisher/RViz)
# never discover each other. This overrides DDS to use loopback so they do.
#
# Usage — in EVERY terminal, AFTER sourcing ROS 2 + the unitree_ros2 overlay
# (so this overrides the overlay's CYCLONEDDS_URI):
#     source /opt/ros/humble/setup.bash
#     source ~/unitree_ros2/install/setup.bash
#     source src/go2_remote_connection/recording/viewer_env.sh
#
# To talk to the REAL robot again, just re-source the overlay (or open a fresh
# terminal) — do NOT source this file.
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0
# Use multicast SPDP discovery over loopback (reliable, unlike unicast-peer
# discovery which is flaky with ParticipantIndex=auto). Loopback is not
# multicast-capable by default, so enable it once per boot:
#     sudo ip link set lo multicast on
# AllowMulticast=true is REQUIRED: Cyclone's "default" heuristic disables
# multicast on loopback even when the MULTICAST flag is set, which breaks
# cross-process discovery.
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo" priority="default" multicast="true"/></Interfaces><AllowMulticast>true</AllowMulticast></General></Domain></CycloneDDS>'

# The ros2 CLI daemon caches the DDS graph and does NOT pick up a changed
# CYCLONEDDS_URI on its own. If it started under a different config (e.g. the
# robot overlay), `ros2 node list` / `ros2 topic echo` report a stale/empty
# graph even though node-to-node data flows fine. Reset it so the next ros2
# command restarts it under THIS config.
ros2 daemon stop >/dev/null 2>&1 || true

if ip link show lo 2>/dev/null | grep -qw MULTICAST; then
  echo "[viewer_env] DDS -> loopback (lo) multicast, domain 0. Daemon reset. Robot NIC ignored."
else
  echo "[viewer_env] ⚠️  'lo' is NOT multicast-capable — discovery will fail."
  echo "[viewer_env]     Run once, then re-source this file:  sudo ip link set lo multicast on"
fi
