#!/usr/bin/env python3
import os

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy, qos_profile_sensor_data
from ament_index_python.packages import get_package_share_directory

import yolo


class YOLONode(Node):
    def __init__(self):
        super().__init__("yolo_node")

        self.declare_parameter("image_topic", "/front_camera/image_raw")
        self.declare_parameter("jpeg_quality", 70)

        self.image_topic = self.get_parameter("image_topic").value
        self.jpeg_quality = int(self.get_parameter("jpeg_quality").value)

        self.bridge = CvBridge()

        stream_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.subscriber_ = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            qos_profile_sensor_data,
        )

        self.raw_pub = self.create_publisher(
            Image,
            "/yolo_depth/image_raw",
            stream_qos,
        )

        self.compressed_pub = self.create_publisher(
            CompressedImage,
            "/web/yolo_cam/compressed",
            stream_qos,
        )

        self.timer = self.create_timer(0.2, self.timer_callback)

        self.get_logger().info(f"YOLO node subscribed to {self.image_topic}")
        self.get_logger().info(f"YOLO compressed output: /web/yolo_cam/compressed")
        self.get_logger().info(f"YOLO JPEG quality: {self.jpeg_quality}")

    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        detections = yolo.detection(image)
        yolo.draw_detections(image, detections)

        stamp = self.get_clock().now().to_msg()

        # Publish raw annotated image only if someone wants it
        if self.raw_pub.get_subscription_count() > 0:
            raw_msg = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
            raw_msg.header.stamp = stamp
            raw_msg.header.frame_id = "front_camera"
            self.raw_pub.publish(raw_msg)

        # Publish compressed annotated image for web streaming
        if self.compressed_pub.get_subscription_count() > 0:
            ok, encoded = cv2.imencode(
                ".jpg",
                image,
                [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
            )
            if ok:
                comp_msg = CompressedImage()
                comp_msg.header.stamp = stamp
                comp_msg.header.frame_id = "front_camera"
                comp_msg.format = "jpeg"
                comp_msg.data = encoded.tobytes()
                self.compressed_pub.publish(comp_msg)

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
                qos_profile_sensor_data,
            )
            self.get_logger().info(
                f"Image topic changed from {old_topic} to {self.image_topic}"
            )


def main(args=None):
    pkg_share = get_package_share_directory("go2_remote_controller")
    yolo.model_path = os.path.join(pkg_share, "model", "yolov8n.pt")

    if not yolo.check_model_exists():
        print(f"Model not found at: {yolo.model_path}")
        raise FileNotFoundError(yolo.model_path)
    else:
        print(f"Model found at: {yolo.model_path}")

    rclpy.init(args=args)
    node = YOLONode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down YOLO node...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()