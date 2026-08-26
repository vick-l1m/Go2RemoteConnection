#!/usr/bin/env python3
import math
import time
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate


def clamp(v: int, lo: int, hi: int) -> int:
    return lo if v < lo else hi if v > hi else v


class MockMap2DPublisher(Node):
    """
    Publishes:
      - /map2d            nav_msgs/OccupancyGrid (latched)
      - /map2d_updates    map_msgs/OccupancyGridUpdate (ROI patches)

    The map is an int8 grid in [-1..100]. We use:
      0   free
      100 occupied
    """

    def __init__(self):
        super().__init__("mock_map2d_publisher")

        # --- Parameters ---
        self.declare_parameter("frame_id", "map2d")
        self.declare_parameter("resolution", 0.1)        # m/cell
        self.declare_parameter("width", 400)             # cells
        self.declare_parameter("height", 400)            # cells
        self.declare_parameter("origin_x", -20.0)        # meters
        self.declare_parameter("origin_y", -20.0)        # meters
        self.declare_parameter("tick_hz", 5.0)
        self.declare_parameter("full_publish_hz", 0.5)   # occasionally re-latch full map

        self.frame_id = self.get_parameter("frame_id").value
        self.res = float(self.get_parameter("resolution").value)
        self.w = int(self.get_parameter("width").value)
        self.h = int(self.get_parameter("height").value)
        self.ox = float(self.get_parameter("origin_x").value)
        self.oy = float(self.get_parameter("origin_y").value)
        self.tick_hz = float(self.get_parameter("tick_hz").value)
        self.full_pub_hz = float(self.get_parameter("full_publish_hz").value)

        # --- QoS to match your real publisher ---
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        upd_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.pub_full = self.create_publisher(OccupancyGrid, "/map2d", map_qos)
        self.pub_upd = self.create_publisher(OccupancyGridUpdate, "/map2d_updates", upd_qos)

        # --- Internal map buffer (int8) ---
        # Initialize as free (0). If you want unknown space, set to -1.
        self.grid = [0] * (self.w * self.h)

        # Build a base “world”
        self._draw_border(thickness=2, val=100)
        self._draw_rect(60, 60, 40, 120, 100)   # obstacle 1
        self._draw_rect(250, 80, 90, 40, 100)   # obstacle 2
        self._draw_rect(140, 260, 120, 25, 100) # obstacle 3

        # Moving obstacle state
        self.t = 0.0
        self.mo_size = 14
        self.mo_prev = None  # previous ROI (x,y,w,h) for clean updates

        # Pre-build OccupancyGrid message shell
        self.full_msg = OccupancyGrid()
        self.full_msg.header.frame_id = self.frame_id
        self.full_msg.info.resolution = self.res
        self.full_msg.info.width = self.w
        self.full_msg.info.height = self.h
        self.full_msg.info.origin.position.x = self.ox
        self.full_msg.info.origin.position.y = self.oy
        self.full_msg.info.origin.orientation.w = 1.0

        # Publish initial full map (latched)
        self.publish_full()

        # Timers
        self.create_timer(1.0 / self.tick_hz, self.tick)

        if self.full_pub_hz > 0.0:
            self.create_timer(1.0 / self.full_pub_hz, self.publish_full)

        self.get_logger().info(
            f"Mock map publisher up: /map2d ({self.w}x{self.h}, res={self.res}) and /map2d_updates"
        )

    def idx(self, x: int, y: int) -> int:
        return y * self.w + x

    def _draw_border(self, thickness: int, val: int):
        for y in range(self.h):
            for x in range(self.w):
                if x < thickness or x >= self.w - thickness or y < thickness or y >= self.h - thickness:
                    self.grid[self.idx(x, y)] = val

    def _draw_rect(self, x: int, y: int, w: int, h: int, val: int):
        x0 = clamp(x, 0, self.w - 1)
        y0 = clamp(y, 0, self.h - 1)
        x1 = clamp(x + w - 1, 0, self.w - 1)
        y1 = clamp(y + h - 1, 0, self.h - 1)
        for yy in range(y0, y1 + 1):
            base = yy * self.w
            for xx in range(x0, x1 + 1):
                self.grid[base + xx] = val

    def _clear_rect(self, x: int, y: int, w: int, h: int, val: int = 0):
        x0 = clamp(x, 0, self.w - 1)
        y0 = clamp(y, 0, self.h - 1)
        x1 = clamp(x + w - 1, 0, self.w - 1)
        y1 = clamp(y + h - 1, 0, self.h - 1)
        for yy in range(y0, y1 + 1):
            base = yy * self.w
            for xx in range(x0, x1 + 1):
                self.grid[base + xx] = val

    def publish_full(self):
        # Copy current grid into OccupancyGrid (list of int8)
        self.full_msg.header.stamp = self.get_clock().now().to_msg()
        self.full_msg.data = list(int(v) for v in self.grid)
        self.pub_full.publish(self.full_msg)

    def publish_update_roi(self, x: int, y: int, w: int, h: int):
        # Clamp ROI to map
        x0 = clamp(x, 0, self.w - 1)
        y0 = clamp(y, 0, self.h - 1)
        x1 = clamp(x + w - 1, 0, self.w - 1)
        y1 = clamp(y + h - 1, 0, self.h - 1)
        ww = (x1 - x0) + 1
        hh = (y1 - y0) + 1

        msg = OccupancyGridUpdate()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.x = x0
        msg.y = y0
        msg.width = ww
        msg.height = hh

        data = []
        for yy in range(y0, y0 + hh):
            base = yy * self.w
            data.extend(self.grid[base + x0 : base + x0 + ww])

        msg.data = [int(v) for v in data]
        self.pub_upd.publish(msg)

    def tick(self):
        # Update moving obstacle path (circle-ish)
        self.t += 1.0 / self.tick_hz

        # previous moving obstacle cleared
        if self.mo_prev is not None:
            px, py, pw, ph = self.mo_prev
            # clear (set back to free)
            self._clear_rect(px, py, pw, ph, val=0)
            # but keep borders/obstacles intact: re-draw static geometry that might overlap
            self._draw_border(thickness=2, val=100)
            self._draw_rect(60, 60, 40, 120, 100)
            self._draw_rect(250, 80, 90, 40, 100)
            self._draw_rect(140, 260, 120, 25, 100)
            # publish update for old ROI (so client sees it cleared)
            self.publish_update_roi(px, py, pw, ph)

        # compute new pos
        cx = int(self.w * 0.5 + math.cos(self.t * 0.8) * (self.w * 0.25))
        cy = int(self.h * 0.5 + math.sin(self.t * 1.1) * (self.h * 0.20))
        x = clamp(cx - self.mo_size // 2, 3, self.w - self.mo_size - 3)
        y = clamp(cy - self.mo_size // 2, 3, self.h - self.mo_size - 3)

        # draw new moving obstacle
        self._draw_rect(x, y, self.mo_size, self.mo_size, 100)
        self.mo_prev = (x, y, self.mo_size, self.mo_size)

        # publish update for new ROI
        self.publish_update_roi(x, y, self.mo_size, self.mo_size)


def main():
    rclpy.init()
    node = MockMap2DPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
