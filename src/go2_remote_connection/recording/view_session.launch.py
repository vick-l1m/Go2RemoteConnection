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

Usage (source ROS 2 Humble + the unitree_ros2 overlay first):
    ros2 launch src/go2_remote_connection/recording/view_session.launch.py

The URDF defaults to the self-contained description/go2/go2.urdf in this repo;
override with urdf:=/path/to/other.urdf if needed.

RViz loads view_session.rviz (fixed frame odom, Go2 RobotModel + camera RGB +
depth cloud displays), so the robot walks across a fixed ground grid. The camera
displays stay empty for joint-only sessions and populate automatically for
sessions recorded with --camera (/tf_static from the bag places them). Override
with rviz_config:=/path/to/other.rviz.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

_HERE = os.path.dirname(os.path.abspath(__file__))


def generate_launch_description():
    urdf = LaunchConfiguration("urdf")
    rviz = LaunchConfiguration("rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    base_motion = LaunchConfiguration("base_motion")

    return LaunchDescription([
        DeclareLaunchArgument(
            "urdf",
            default_value=os.path.normpath(
                os.path.join(_HERE, "..", "..", "..", "description", "go2", "go2.urdf")
            ),
            description="Absolute path to the Go2 URDF for robot_state_publisher. "
                        "Defaults to the self-contained description/go2/go2.urdf.",
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
