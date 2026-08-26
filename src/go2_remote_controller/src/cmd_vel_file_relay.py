#!/usr/bin/env python3
"""
Subscribes to /robot0/cmd_vel and writes the latest (vx vy vyaw) to
/tmp/go2_cmd_vel as a single space-separated line.

This bridges the Python-3.10 ROS2 world into any Python 3.11 process
(e.g. Isaac Lab) that can't load rclpy's C extension directly.

Usage (called automatically by start_remote_connection_sim.sh):
  python3 cmd_vel_file_relay.py [--output /tmp/go2_cmd_vel]
"""

import argparse
import os
import tempfile

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CmdVelFileRelay(Node):
    def __init__(self, output_path: str):
        super().__init__("cmd_vel_file_relay")
        self._output = output_path
        self._dir = os.path.dirname(os.path.abspath(output_path))
        self.create_subscription(Twist, "/robot0/cmd_vel", self._cb, 10)
        self.get_logger().info(f"Relaying /robot0/cmd_vel -> {self._output}")

    def _cb(self, msg: Twist):
        # Atomic write: write to a temp file then rename so readers never see a partial line
        fd, tmp = tempfile.mkstemp(dir=self._dir, prefix=".cmd_vel_tmp")
        try:
            with os.fdopen(fd, "w") as f:
                f.write(f"{msg.linear.x} {msg.linear.y} {msg.angular.z}\n")
            os.replace(tmp, self._output)
        except Exception:
            try:
                os.unlink(tmp)
            except OSError:
                pass


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", default="/tmp/go2_cmd_vel")
    args = parser.parse_args()

    rclpy.init()
    node = CmdVelFileRelay(args.output)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
