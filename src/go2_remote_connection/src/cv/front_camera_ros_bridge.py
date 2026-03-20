#!/usr/bin/env python3
import socket
import struct
import threading
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
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


def connect_with_retry(host: str, port: int, retries: int = 100, delay: float = 0.1) -> socket.socket:
    last_err = None
    for _ in range(retries):
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            sock.connect((host, port))
            return sock
        except Exception as e:
            last_err = e
            try:
                sock.close()
            except Exception:
                pass
            time.sleep(delay)
    raise RuntimeError(f"Could not connect to capture process at {host}:{port}: {last_err}")


class FrontCameraRosBridge(Node):
    def __init__(self):
        super().__init__("front_camera_node")

        self.bridge = CvBridge()
        self.running = True
        self.sock = None
        self.recv_thread = None

        stream_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.raw_pub = self.create_publisher(
            Image,
            "/front_camera/image_raw",
            stream_qos,
        )

        self.compressed_pub = self.create_publisher(
            CompressedImage,
            "/web/front_cam/compressed",
            stream_qos,
        )

        self.get_logger().info(f"Connecting to capture process at {HOST}:{PORT} ...")
        self.sock = connect_with_retry(HOST, PORT)
        self.get_logger().info("Connected to capture process")

        self.frame_count = 0
        self.last_log_time = time.time()

        self.recv_thread = threading.Thread(target=self._recv_loop, daemon=True)
        self.recv_thread.start()

        self.get_logger().info(
            "Front camera ROS bridge started "
            "(/web/front_cam/compressed always, /front_camera/image_raw on demand)"
        )

    def _recv_loop(self):
        while self.running:
            try:
                header = recv_exact(self.sock, 4)
                (size,) = struct.unpack("!I", header)

                if size <= 0:
                    continue

                payload = recv_exact(self.sock, size)
                stamp = self.get_clock().now().to_msg()

                # Publish compressed JPEG immediately for lowest latency
                compressed_msg = CompressedImage()
                compressed_msg.header.stamp = stamp
                compressed_msg.header.frame_id = "front_camera"
                compressed_msg.format = "jpeg"
                compressed_msg.data = payload
                self.compressed_pub.publish(compressed_msg)

                # Only pay decode cost if someone actually subscribes to raw frames
                if self.raw_pub.get_subscription_count() > 0:
                    image_data = np.frombuffer(payload, dtype=np.uint8)
                    image = cv2.imdecode(image_data, cv2.IMREAD_COLOR)

                    if image is not None:
                        ros_image = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
                        ros_image.header.stamp = stamp
                        ros_image.header.frame_id = "front_camera"
                        self.raw_pub.publish(ros_image)
                    else:
                        self.get_logger().warn("Failed to decode JPEG frame for raw image publish")

                self.frame_count += 1
                now = time.time()
                if now - self.last_log_time >= 5.0:
                    fps = self.frame_count / (now - self.last_log_time)
                    raw_subs = self.raw_pub.get_subscription_count()
                    comp_subs = self.compressed_pub.get_subscription_count()
                    self.get_logger().info(
                        f"Publishing at ~{fps:.1f} FPS "
                        f"(raw subs={raw_subs}, compressed subs={comp_subs})"
                    )
                    self.frame_count = 0
                    self.last_log_time = now

            except ConnectionError as e:
                if self.running:
                    self.get_logger().error(f"Camera socket connection lost: {e}")
                break
            except Exception as e:
                if self.running:
                    self.get_logger().error(f"Bridge receive/publish error: {e}")
                # small pause prevents tight error loop
                time.sleep(0.01)

        self.running = False

    def destroy_node(self):
        self.running = False

        try:
            if self.sock is not None:
                self.sock.shutdown(socket.SHUT_RDWR)
        except Exception:
            pass

        try:
            if self.sock is not None:
                self.sock.close()
        except Exception:
            pass

        if self.recv_thread is not None and self.recv_thread.is_alive():
            self.recv_thread.join(timeout=1.0)

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