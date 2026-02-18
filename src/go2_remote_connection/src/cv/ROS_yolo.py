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
import json
from std_msgs.msg import String

import os, sys
THIS_DIR = os.path.dirname(os.path.abspath(__file__))
if THIS_DIR not in sys.path:
    sys.path.insert(0, THIS_DIR)

import yolo  # yolo.py must be in the same directory
yolo.model_path = os.path.join(THIS_DIR, "model", "yolov8n.pt")

def make_json_safe(x):
    import numpy as np
    # numpy scalars -> Python scalars
    if isinstance(x, (np.integer, np.floating)):
        return x.item()
    # numpy arrays -> lists
    if isinstance(x, np.ndarray):
        return x.tolist()
    # dict -> recurse
    if isinstance(x, dict):
        return {k: make_json_safe(v) for k, v in x.items()}
    # list/tuple -> recurse
    if isinstance(x, (list, tuple)):
        return [make_json_safe(v) for v in x]
    # fallback
    return x

class YOLONode(Node):
    def __init__(self):
        super().__init__("yolo_node")
        self.declare_parameter("image_topic", "/front_camera/image_raw")
        self.image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        self.subscriber_ = self.create_subscription(Image, self.image_topic, self.image_callback, 10)
        self.publisher_ = self.create_publisher(Image, "/yolo_depth/image_raw", 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.bridge = CvBridge()
        self.det_pub = self.create_publisher(String, "/yolo/detections", 10)

    def image_callback(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            detections = yolo.detection(image)
    
            payload = {
                "stamp": {"sec": int(msg.header.stamp.sec), "nanosec": int(msg.header.stamp.nanosec)},
                "frame_id": msg.header.frame_id,
                "detections": make_json_safe(detections),
            }
            self.det_pub.publish(String(data=json.dumps(payload)))
    
            yolo.draw_detections(image, detections)
            self.publisher_.publish(self.bridge.cv2_to_imgmsg(image, encoding="bgr8"))
        except Exception as e:
            self.get_logger().error(f"YOLO callback failed: {e}")

    def timer_callback(self):
        current_image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        if current_image_topic != self.image_topic:
            self.image_topic = current_image_topic
            self.subscriber_.destroy()
            self.subscriber_ = self.create_subscription(Image, self.image_topic, self.image_callback, 10)
            self.get_logger().info(f"Image topic changed to {self.image_topic}")

def main(args=None):
    if not yolo.check_model_exists():
        print("Model not found, please run model_download.py to download the model")
        exit(1)
    else:
        print("Model found")
    
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