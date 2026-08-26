"""view_session.launch.py — visualise a recorded Go2 session in RViz.

Brings up the pieces that turn a played-back bag into a moving robot:
  * lowstate_to_jointstate  — /lowstate (LowState) -> /joint_states
  * robot_state_publisher   — /joint_states + URDF -> TF for the RobotModel
  * sportmodestate_to_tf    — /sportmodestate -> odom->base_link (base moves in
                              space, not on the spot); disable with base_motion:=false
  * rviz2                    — the viewer

It does NOT play the bag — run that yourself in another terminal so you control
play/pause/rate:
    ros2 bag play sessions/<UTCdate>_<name>/bag --loop

Usage (source ROS 2 Humble first):
    ros2 launch go2_remote_viz view_session.launch.py

The robot model comes from the go2_description package in the main repo — it is
the single source of truth for the Go2's kinematics, and its meshes are resolved
through package:// rather than absolute paths. Build that package alongside this
one, or set GO2_DESCRIPTION_SHARE to its share directory. Override the model
entirely with urdf:=/path/to/other.urdf.

RViz loads view_session.rviz (fixed frame odom, Go2 RobotModel + camera RGB +
depth cloud displays), so the robot walks across a fixed ground grid. The camera
displays stay empty for joint-only sessions and populate automatically for
sessions recorded with --camera (/tf_static from the bag places them). Override
with rviz_config:=/path/to/other.rviz.
"""

import os
import sys

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

_HERE = os.path.dirname(os.path.abspath(__file__))


def _default_urdf():
    """Locate go2_description's URDF, with a clear failure instead of a silent one.

    This package intentionally does not carry its own copy of the robot model:
    two copies drift, and the one that used to live here had absolute mesh paths
    baked in for a single machine.
    """
    override = os.environ.get("GO2_DESCRIPTION_SHARE")
    if override:
        return os.path.join(override, "urdf", "go2.urdf")
    try:
        return os.path.join(get_package_share_directory("go2_description"), "urdf", "go2.urdf")
    except PackageNotFoundError:
        print("[view_session] go2_description not found on the ament path.\n"
              "  Build it from the main repo (colcon build --packages-select go2_description)\n"
              "  and source that workspace, or set GO2_DESCRIPTION_SHARE=<its share dir>,\n"
              "  or pass urdf:=/path/to/go2.urdf.", file=sys.stderr)
        return ""


def generate_launch_description():
    urdf = LaunchConfiguration("urdf")
    rviz = LaunchConfiguration("rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    base_motion = LaunchConfiguration("base_motion")

    return LaunchDescription([
        DeclareLaunchArgument(
            "urdf",
            default_value=_default_urdf(),
            description="Absolute path to the Go2 URDF for robot_state_publisher. "
                        "Defaults to go2_description's urdf/go2.urdf.",
        ),
        DeclareLaunchArgument("rviz", default_value="true", description="Launch rviz2."),
        DeclareLaunchArgument(
            "base_motion", default_value="true",
            description="Broadcast odom->base_link from /sportmodestate so the base moves "
                        "through space. Set false to view the robot on the spot at the origin.",
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=os.path.join(_HERE, "view_session.rviz"),
            description="RViz config (Go2 RobotModel + camera displays, fixed frame base_link).",
        ),

        # /lowstate -> /joint_states (plain rclpy script, no package install needed)
        ExecuteProcess(
            cmd=["python3", os.path.join(_HERE, "lowstate_to_jointstate.py")],
            output="screen",
        ),

        # /sportmodestate -> TF odom->base_link, so the base moves through space
        # (not just legs on the spot). Silent for bags without /sportmodestate.
        ExecuteProcess(
            cmd=["python3", os.path.join(_HERE, "sportmodestate_to_tf.py")],
            output="screen",
            condition=IfCondition(base_motion),
        ),

        # /joint_states + URDF -> TF
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": ParameterValue(Command(["cat ", urdf]), value_type=str),
            }],
        ),

        # the viewer (skip with rviz:=false, e.g. on a headless machine)
        Node(
            package="rviz2",
            executable="rviz2",
            output="screen",
            arguments=["-d", rviz_config],
            condition=IfCondition(rviz),
        ),
    ])
