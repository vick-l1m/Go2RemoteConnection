#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2

class ImageToCompressedBridge(Node):
    def __init__(self):
        super().__init__("image_to_compressed_bridge")
        self.bridge = CvBridge()

        # Match your existing web bridge default:
        self.in_topic  = "/front_camera/image_raw"
        self.out_topic = "/web/front_cam/compressed"

        # ✅ INPUT QoS: must match /front_camera/image_raw publisher (RELIABLE)
        in_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # ✅ OUTPUT QoS: best-effort is fine for camera streaming
        out_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=2,
        )

        self.pub = self.create_publisher(CompressedImage, self.out_topic, out_qos)
        self.sub = self.create_subscription(Image, self.in_topic, self.cb, in_qos)

        # JPEG quality (0-100). 70–85 is a good starting point.
        self.declare_parameter("jpeg_quality", 80)
        self.get_logger().info(f"Bridging {self.in_topic} -> {self.out_topic} (JPEG)")

    def cb(self, msg: Image):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

            q = int(self.get_parameter("jpeg_quality").value)
            ok, enc = cv2.imencode(".jpg", cv_img, [int(cv2.IMWRITE_JPEG_QUALITY), q])
            if not ok:
                return

            out = CompressedImage()
            out.header = msg.header
            out.format = "jpeg"
            out.data = enc.tobytes()
            self.pub.publish(out)

        except Exception as e:
            self.get_logger().warn(f"Bridge encode failed: {e}")

def main():
    rclpy.init()
    node = ImageToCompressedBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
