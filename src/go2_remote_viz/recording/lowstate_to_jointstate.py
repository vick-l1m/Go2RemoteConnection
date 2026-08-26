#!/usr/bin/env python3
"""lowstate_to_jointstate.py — bridge Unitree /lowstate -> sensor_msgs/JointState.

The recorder captures per-joint angles as ``unitree_go/msg/LowState`` on
``/lowstate`` (motor_state[i].q). RViz's RobotModel display can't animate from
that; it needs a name-keyed ``sensor_msgs/JointState`` on ``/joint_states`` which
``robot_state_publisher`` converts to TF for the Go2 URDF links.

This node does exactly that map. Run it alongside ``ros2 bag play`` (which
re-publishes /lowstate) and robot_state_publisher, then watch the Go2 articulate
in RViz. Pure rclpy + std message deps that come with the unitree_ros2 overlay,
so no colcon rebuild — just source ROS 2 Humble + the overlay and run it.

    python3 lowstate_to_jointstate.py            # /lowstate -> /joint_states

The first 12 motor_state entries are the leg joints in SDK order (single source
of truth: record_session.sh session_metadata.yaml `sdk_joint_order`). We append
"_joint" to match the Go2 URDF joint names.
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import JointState
from unitree_go.msg import LowState

# SDK motor_state[0..11] order -> URDF joint names (SDK name + "_joint").
# Mirror of record_session.sh `sdk_joint_order`.
JOINT_NAMES = [
    "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
    "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
    "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
    "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
]


class LowStateToJointState(Node):
    def __init__(self):
        super().__init__("lowstate_to_jointstate")
        self.declare_parameter("in_topic", "/lowstate")
        self.declare_parameter("out_topic", "/joint_states")
        in_topic = self.get_parameter("in_topic").value
        out_topic = self.get_parameter("out_topic").value

        self.pub = self.create_publisher(JointState, out_topic, 10)
        self.create_subscription(LowState, in_topic, self._on_lowstate, 10)
        self.get_logger().info(f"bridging {in_topic} (LowState) -> {out_topic} (JointState)")

    def _on_lowstate(self, msg: LowState):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = JOINT_NAMES
        js.position = [float(msg.motor_state[i].q) for i in range(len(JOINT_NAMES))]
        js.velocity = [float(msg.motor_state[i].dq) for i in range(len(JOINT_NAMES))]
        js.effort = [float(msg.motor_state[i].tau_est) for i in range(len(JOINT_NAMES))]
        self.pub.publish(js)


def main():
    rclpy.init()
    node = LowStateToJointState()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
