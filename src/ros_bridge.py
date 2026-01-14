import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class WebRosBridge(Node):
    def __init__(self):
        super().__init__("web_ros_bridge")
        self.pub_twist = self.create_publisher(Twist, "/web_teleop", 10)
        self.pub_action = self.create_publisher(String, "/web_action", 10)

    def publish_teleop(self, vx: float, vy: float, vyaw: float):
        msg = Twist()
        msg.linear.x = float(vx)
        msg.linear.y = float(vy)
        msg.angular.z = float(vyaw)
        self.pub_twist.publish(msg)

    def publish_action(self, action: str):
        m = String()
        m.data = action
        self.pub_action.publish(m)

_bridge = None

def start_ros_bridge():
    global _bridge
    rclpy.init(args=None)
    _bridge = WebRosBridge()

    # Spin in background thread
    t = threading.Thread(target=rclpy.spin, args=(_bridge,), daemon=True)
    t.start()
    return _bridge

def get_bridge() -> WebRosBridge:
    global _bridge
    if _bridge is None:
        raise RuntimeError("ROS bridge not started")
    return _bridge
