import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge  # Convert OpenCV image to ROS2 Image message
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.go2.video.video_client import VideoClient
import cv2
import numpy as np
import os
import xml.etree.ElementTree as ET
import yolo
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from ament_index_python.packages import get_package_share_directory

class YOLONode(Node):
    def __init__(self):
        super().__init__("yolo_node")
        self.declare_parameter("image_topic", "/front_camera/image_raw")
        self.image_topic = self.get_parameter("image_topic").value
        self.bridge = CvBridge()

        image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE
        )
        # Input subscription QoS: must match camera publisher (BEST_EFFORT)
        self.subscriber_ = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            qos_profile_sensor_data
        )

        # Output publisher QoS: RELIABLE is fine for RViz
        output_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE
        )

        self.publisher_ = self.create_publisher(
            Image,
            "/yolo_depth/image_raw",
            output_qos
        )

        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info(f"YOLO node subscribed to {self.image_topic}")

        
    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        detections = yolo.detection(image)
        yolo.draw_detections(image, detections)
        self.publisher_.publish(self.bridge.cv2_to_imgmsg(image, encoding="bgr8"))

    def timer_callback(self):
        current_image_topic = self.get_parameter("image_topic").value

        if current_image_topic != self.image_topic:
            old_topic = self.image_topic
            self.image_topic = current_image_topic
            self.destroy_subscription(self.subscriber_)
            self.subscriber_ = self.create_subscription(
                Image,
                self.image_topic,
                self.image_callback,
                qos_profile_sensor_data
            )
            self.get_logger().info(
                f"Image topic changed from {old_topic} to {self.image_topic}"
            )

def main(args=None):
    pkg_share = get_package_share_directory("go2_remote_connection")
    yolo.model_path = os.path.join(pkg_share, "model", "yolov8n.pt")

    if not yolo.check_model_exists():
        print(f"Model not found at: {yolo.model_path}")
        exit(1)
    else:
        print(f"Model found at: {yolo.model_path}")
    
    rclpy.init(args=args)
    yolo_node = YOLONode()
    
    try:
        rclpy.spin(yolo_node)
    except KeyboardInterrupt:
        yolo_node.get_logger().info("Shutting down YOLO node...")
    except Exception as e:
        yolo_node.get_logger().error(f"Unexpected error: {e}")
    finally:
        try:
            yolo_node.destroy_node()
            rclpy.shutdown()
        except Exception as e:
            # Ignore shutdown errors as they're common when interrupting
            pass

if __name__ == "__main__":
    main()