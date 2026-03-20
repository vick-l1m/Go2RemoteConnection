#!/usr/bin/env python3
import socket
import struct

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


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

def connect_with_retry(host, port, retries=50, delay=0.1):
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    for _ in range(retries):
        try:
            sock.connect((host, port))
            return sock
        except ConnectionRefusedError:
            import time
            time.sleep(delay)
    raise RuntimeError(f"Could not connect to capture process at {host}:{port}")

class FrontCameraRosBridge(Node):
    def __init__(self):
        super().__init__("front_camera_node")

        self.publisher_ = self.create_publisher(
            Image,
            "/front_camera/image_raw",
            qos_profile_sensor_data,
        )
        self.bridge = CvBridge()

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.get_logger().info(f"Connecting to capture process at {HOST}:{PORT} ...")
        self.sock = connect_with_retry(HOST, PORT)
        self.get_logger().info("Connected to capture process")

        self.timer = self.create_timer(1.0 / 30.0, self.capture_callback)
        self.get_logger().info("Front camera ROS bridge started")

    def capture_callback(self):
        try:
            header = recv_exact(self.sock, 4)
            (size,) = struct.unpack("!I", header)
            payload = recv_exact(self.sock, size)

            image_data = np.frombuffer(payload, dtype=np.uint8)
            image = cv2.imdecode(image_data, cv2.IMREAD_COLOR)
            if image is None:
                self.get_logger().warn("Failed to decode frame from capture process")
                return

            ros_image = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
            ros_image.header.stamp = self.get_clock().now().to_msg()
            ros_image.header.frame_id = "front_camera"
            self.publisher_.publish(ros_image)

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