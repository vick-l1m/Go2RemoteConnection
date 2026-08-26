# topics.sh — shared topic sets for the recording modules.
# Sourced by record_session.sh (joints) and record_camera.sh (camera) so the
# topic lists have a single source of truth. Not executable on its own.

# --- Unitree state (override for a different unitree_ros2 install) ----------
JOINT_STATE_TOPICS="${GO2_STATE_TOPICS:-/lowstate /sportmodestate /wirelesscontroller /lowcmd}"

# --- Web joystick + drive-mode + safety context ----------------------------
WEB_TOPICS="/web_teleop /web_control_mode /web_estop /web_teleop_enabled /web_sport_cmd"

# --- Camera module ----------------------------------------------------------
# Raw depth cloud (reprocessable) + the sim-matched processed height scan +
# RGB/intrinsics + TF (needed to rebuild the cloud in the base frame offline).
#
# The realsense2_camera pointcloud filter publishes on <ns>/depth/color/points;
# with real_perception.launch.py the namespace is /go2/camera. Override the cloud
# topic via GO2_CLOUD_TOPIC if your driver namespace differs (verify with
# `ros2 topic list`). Sim publishes /go2/camera/depth/points (no color) — set
# GO2_CLOUD_TOPIC accordingly if you ever record from the sim instead.
GO2_CLOUD_TOPIC="${GO2_CLOUD_TOPIC:-/go2/camera/depth/color/points}"

# RGB is handy for debugging but roughly doubles disk; set GO2_RECORD_RGB=0 to drop it.
_RGB_TOPICS="/go2/camera/color/image_raw /go2/camera/color/camera_info"
[ "${GO2_RECORD_RGB:-1}" = "0" ] && _RGB_TOPICS=""

CAMERA_TOPICS="${GO2_CAMERA_TOPICS:-$GO2_CLOUD_TOPIC /go2/height_scan /go2/local_heightmap /go2/camera/depth/camera_info $_RGB_TOPICS /tf /tf_static}"
