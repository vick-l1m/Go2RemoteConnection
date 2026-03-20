#!/usr/bin/env python3
import socket
import struct
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image, CompressedImage


HOST = "127.0.0.1"
PORT = 9998


def recv_exact(sock: socket.socket, size: int) -> bytes:
    chunks = []
    remaining = size
    while remaining > 0:
        chunk = sock.recv(remaining)
        if not chunk:
            raise ConnectionError("Socket closed while receiving frame")
        chunks.append(chunk)
        remaining -= len(chunk)
    return b"".join(chunks)


def connect_with_retry(host, port, retries=100, delay=0.1):
    for _ in range(retries):
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            sock.connect((host, port))
            return sock
        except ConnectionRefusedError:
            sock.close()
            time.sleep(delay)
    raise RuntimeError(f"Could not connect to capture process at {host}:{port}")


class FrontCameraRosBridge(Node):
    def __init__(self):
        super().__init__("front_camera_node")

        self.bridge = CvBridge()

        # Raw image QoS for ROS camera consumers like YOLO
        raw_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Compressed stream QoS for web/video consumers
        compressed_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.raw_pub = self.create_publisher(
            Image,
            "/front_camera/image_raw",
            raw_qos,
        )

        self.compressed_pub = self.create_publisher(
            CompressedImage,
            "/web/front_cam/compressed",
            compressed_qos,
        )

        self.get_logger().info(f"Connecting to capture process at {HOST}:{PORT} ...")
        self.sock = connect_with_retry(HOST, PORT)
        self.get_logger().info("Connected to capture process")

        # Run as fast as practical without artificially limiting to 30 Hz.
        # A very small timer period keeps latency low.
        self.timer = self.create_timer(0.001, self.capture_callback)
        self.get_logger().info(
            "Front camera ROS bridge started "
            "(/front_camera/image_raw and /web/front_cam/compressed)"
        )

        self.frame_count = 0
        self.last_log_time = time.time()

    def capture_callback(self):
        try:
            header = recv_exact(self.sock, 4)
            (size,) = struct.unpack("!I", header)
            payload = recv_exact(self.sock, size)

            stamp = self.get_clock().now().to_msg()

            # Publish compressed JPEG directly for web streaming
            compressed_msg = CompressedImage()
            compressed_msg.header.stamp = stamp
            compressed_msg.header.frame_id = "front_camera"
            compressed_msg.format = "jpeg"
            compressed_msg.data = payload
            self.compressed_pub.publish(compressed_msg)

            # Decode once for raw ROS image consumers like YOLO
            image_data = np.frombuffer(payload, dtype=np.uint8)
            image = cv2.imdecode(image_data, cv2.IMREAD_COLOR)
            if image is not None:
                ros_image = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
                ros_image.header.stamp = stamp
                ros_image.header.frame_id = "front_camera"
                self.raw_pub.publish(ros_image)
            else:
                self.get_logger().warn("Failed to decode JPEG frame from capture process")

            self.frame_count += 1
            now = time.time()
            if now - self.last_log_time >= 5.0:
                fps = self.frame_count / (now - self.last_log_time)
                self.get_logger().info(f"Publishing at ~{fps:.1f} FPS")
                self.frame_count = 0
                self.last_log_time = now

        except Exception as e:
            self.get_logger().error(f"Bridge receive/publish error: {e}")

    def destroy_node(self):
        try:
            self.sock.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FrontCameraRosBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()