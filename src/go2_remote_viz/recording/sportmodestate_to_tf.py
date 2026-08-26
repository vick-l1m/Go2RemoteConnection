#!/usr/bin/env python3
"""sportmodestate_to_tf.py — broadcast odom->base_link TF from /sportmodestate.

robot_state_publisher only builds TF *inside* the robot (base_link -> legs) from
/joint_states, so on its own the Go2 walks on the spot at the origin. This node
adds the missing piece: where the base is in the world. It reads the Go2 state
estimator's base pose from /sportmodestate (unitree_go/SportModeState) and
broadcasts it as an odom -> base_link transform, so with RViz fixed frame `odom`
the robot moves through space (and leaves a TF trail of where it has been).

The pose is the robot's onboard odometry — it drifts over a long session (no
loop closure), but it is faithful for visualising a recorded run.

Only sessions recorded with record_session.sh contain /sportmodestate; camera-
only bags (record_camera.sh) do not, so this node simply stays silent for those.

    python3 sportmodestate_to_tf.py     # /sportmodestate -> TF odom->base_link
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from unitree_go.msg import SportModeState


class SportModeStateToTF(Node):
    def __init__(self):
        super().__init__("sportmodestate_to_tf")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value

        self.br = TransformBroadcaster(self)
        self.create_subscription(SportModeState, "/sportmodestate", self._on_state, 10)
        self.get_logger().info(
            f"broadcasting TF {self.odom_frame} -> {self.base_frame} from /sportmodestate"
        )

    def _on_state(self, msg: SportModeState):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame

        t.transform.translation.x = float(msg.position[0])
        t.transform.translation.y = float(msg.position[1])
        t.transform.translation.z = float(msg.position[2])

        # Unitree quaternion order is [w, x, y, z]; ROS wants x, y, z, w.
        qw, qx, qy, qz = (float(v) for v in msg.imu_state.quaternion)
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.br.sendTransform(t)


def main():
    rclpy.init()
    node = SportModeStateToTF()
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
