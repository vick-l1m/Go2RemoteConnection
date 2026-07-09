"""
ros_bridge.py

This module provides the ROS 2 <-> web bridge for the Go2 Remote Actions app.
It owns:
- shared websocket data stores for map, camera, YOLO detections, and YOLO camera
- the ROS 2 node that publishes commands from the web UI
- ROS 2 subscriptions that push data into the websocket stores
- lifecycle helpers for starting and accessing the bridge

Version 2.0
Author: Victor Lim
"""

import asyncio
import gzip
import json
import threading
from dataclasses import dataclass, field
from typing import Any, Dict, Optional, Set

import rclpy
from fastapi import WebSocket
from geometry_msgs.msg import Twist
from map_msgs.msg import OccupancyGridUpdate
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool, Float32
from std_msgs.msg import String as RosString

from app.core.state import state


# ============================================================
# 2D MAP STORE
# ============================================================
@dataclass
class MapStore:
    meta: Optional[Dict[str, Any]] = None
    full_raw: Optional[bytes] = None
    seq: int = 0
    clients: Set[WebSocket] = None
    lock: asyncio.Lock = None

    def __post_init__(self):
        if self.clients is None:
            self.clients = set()
        if self.lock is None:
            self.lock = asyncio.Lock()


_map_store = MapStore()


async def _broadcast_map_payload(payload: dict):
    dead = []

    async with _map_store.lock:
        clients = list(_map_store.clients)

    for ws in clients:
        try:
            header = payload.copy()
            gz = header.pop("gz")
            await ws.send_text(json.dumps(header))
            await ws.send_bytes(gz)
        except Exception:
            dead.append(ws)

    if dead:
        async with _map_store.lock:
            for ws in dead:
                _map_store.clients.discard(ws)


def _i8_list_to_bytes(data) -> bytes:
    return bytes((d & 0xFF) for d in data)


def get_map_store() -> MapStore:
    return _map_store


# ============================================================
# CAMERA STORE (JPEG bytes)
# ============================================================
@dataclass
class CameraStore:
    meta: Optional[Dict[str, Any]] = None
    jpg: Optional[bytes] = None
    seq: int = 0
    clients: Set[WebSocket] = None
    lock: asyncio.Lock = None

    def __post_init__(self):
        if self.clients is None:
            self.clients = set()
        if self.lock is None:
            self.lock = asyncio.Lock()


_cam_store = CameraStore()


def get_cam_store() -> CameraStore:
    return _cam_store


# ============================================================
# YOLO DETECTIONS STORE (JSON text)
# ============================================================
@dataclass
class YoloStore:
    lock: asyncio.Lock = field(default_factory=asyncio.Lock)
    clients: Set[WebSocket] = field(default_factory=set)
    seq: int = 0
    last_json: Optional[str] = None


_yolo_store: Optional[YoloStore] = None


def get_yolo_store() -> YoloStore:
    global _yolo_store
    if _yolo_store is None:
        _yolo_store = YoloStore()
    return _yolo_store


# ============================================================
# YOLO CAMERA STORE (JPEG bytes)
# ============================================================
_yolo_cam_store = CameraStore()


def get_yolo_cam_store() -> CameraStore:
    return _yolo_cam_store


# ============================================================
# ROS <-> WEB BRIDGE NODE
# ============================================================
class WebRosBridge(Node):
    def __init__(self):
        super().__init__("web_ros_bridge")

        # ---------------- Publishers ----------------
        self.pub_twist = self.create_publisher(Twist, "/web_teleop", 10)
        self.pub_action = self.create_publisher(RosString, "/web_action", 10)
        self.pub_enabled = self.create_publisher(Bool, "/web_teleop_enabled", 1)
        self.pub_move_forward = self.create_publisher(Float32, "/move_forward_meters", 10)
        self.pub_sport_cmd = self.create_publisher(RosString, "/web_sport_cmd", 10)
        self.pub_control_mode = self.create_publisher(RosString, "/web_control_mode", 1)
        self.pub_rl_policy = self.create_publisher(RosString, "/web_rl_policy", 1)
        self.pub_estop = self.create_publisher(Bool, "/web_estop", 1)

        # ---------------- Map subscriptions ----------------
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

        self.declare_parameter("map_topic", "/map2d")
        self.declare_parameter("map_updates_topic", "/map2d_updates")

        map_topic = self.get_parameter("map_topic").value
        map_updates_topic = self.get_parameter("map_updates_topic").value

        self.sub_map_full = self.create_subscription(
            OccupancyGrid, map_topic, self._on_map_full, map_qos
        )
        self.sub_map_upd = self.create_subscription(
            OccupancyGridUpdate, map_updates_topic, self._on_map_update, upd_qos
        )

        # ---------------- Front camera subscription ----------------
        self.declare_parameter("front_cam_topic", "/web/front_cam/compressed")
        cam_topic = self.get_parameter("front_cam_topic").value

        cam_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=2,
        )

        self.sub_front_cam = self.create_subscription(
            CompressedImage, cam_topic, self._on_front_cam, cam_qos
        )

        # ---------------- YOLO subscriptions ----------------
        self.sub_yolo = self.create_subscription(
            RosString,
            "/yolo/detections",
            self._on_yolo_detections,
            10,
        )

        self.declare_parameter("yolo_cam_topic", "/web/yolo_cam/compressed")
        yolo_cam_topic = self.get_parameter("yolo_cam_topic").value

        self.sub_yolo_cam = self.create_subscription(
            CompressedImage, yolo_cam_topic, self._on_yolo_cam, cam_qos
        )

        # ---------------- RL policy liveness watchdog ----------------
        # If we are in RL control mode but go2_rl_policy_node stops sending its
        # heartbeat (crash / kill), auto-revert to sport so web_bridge un-gates and
        # the robot stays drivable. Covers the hard-crash case the node's own
        # SIGINT/SIGTERM handler cannot.
        self._rl_last_hb: Optional[float] = None
        self._rl_mode_since: Optional[float] = None
        self.create_subscription(Bool, "/web_rl_heartbeat", self._on_rl_heartbeat, 10)
        self.create_timer(0.5, self._rl_watchdog)

        # ---------------- RL active-policy feedback ----------------
        # The RL node echoes the policy it actually loaded here; mirror it into
        # server state so GET /rl/policy reflects reality (incl. a rejected switch).
        self.create_subscription(RosString, "/web_rl_active_policy", self._on_rl_active_policy, 1)

        # AsyncIO loop provided by FastAPI
        self._loop: Optional[asyncio.AbstractEventLoop] = None

    def set_asyncio_loop(self, loop: asyncio.AbstractEventLoop):
        self._loop = loop

    # ---------------- RL liveness watchdog ----------------
    def _on_rl_heartbeat(self, msg: Bool):
        self._rl_last_hb = self.get_clock().now().nanoseconds * 1e-9

    def _on_rl_active_policy(self, msg: RosString):
        pid = msg.data.strip()
        if pid:
            state.rl_policy_id = pid

    def _rl_watchdog(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if state.control_mode != "rl":
            self._rl_mode_since = None
            return
        if self._rl_mode_since is None:
            self._rl_mode_since = now            # just entered rl; start the grace window
        # Use the most recent of (last heartbeat, mode-entry) so a freshly-engaged
        # node gets a grace period to start beating before we judge it dead.
        last = self._rl_last_hb if self._rl_last_hb is not None else self._rl_mode_since
        if now - last > 2.0:
            self.get_logger().warn("RL node heartbeat lost; auto-reverting to sport mode")
            state.control_mode = "sport"
            self.publish_control_mode("sport")   # clears web_bridge rl_mode_ gate
            self.publish_enabled(True)           # re-enable sport teleop
            self._rl_mode_since = None
            self._rl_last_hb = None

    # ---------------- Publish helpers ----------------
    def _ok_to_publish(self) -> bool:
        try:
            return rclpy.ok()
        except Exception:
            return False

    def publish_teleop(self, linear_x: float, linear_y: float, angular_z: float):
        if not self._ok_to_publish():
            return

        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.linear.y = float(linear_y)
        msg.angular.z = float(angular_z)
        self.pub_twist.publish(msg)

    def publish_action(self, action: str):
        if not self._ok_to_publish():
            return

        msg = RosString()
        msg.data = str(action)
        self.pub_action.publish(msg)

    def publish_enabled(self, enabled: bool):
        if not self._ok_to_publish():
            return

        msg = Bool()
        msg.data = bool(enabled)
        self.pub_enabled.publish(msg)

    def publish_move_forward(self, meters: float):
        if not self._ok_to_publish():
            return

        msg = Float32()
        msg.data = float(meters)
        self.pub_move_forward.publish(msg)

    def publish_sport_cmd(self, payload):
        if not self._ok_to_publish():
            return

        msg = RosString()
        if isinstance(payload, str):
            msg.data = payload
        else:
            msg.data = json.dumps(payload)
        self.pub_sport_cmd.publish(msg)

    def publish_control_mode(self, mode: str):
        if not self._ok_to_publish():
            return

        msg = RosString()
        msg.data = str(mode)
        self.pub_control_mode.publish(msg)

    def publish_rl_policy(self, policy_id: str):
        if not self._ok_to_publish():
            return

        msg = RosString()
        msg.data = str(policy_id)
        self.pub_rl_policy.publish(msg)

    def publish_estop(self, engaged: bool):
        if not self._ok_to_publish():
            return

        msg = Bool()
        msg.data = bool(engaged)
        self.pub_estop.publish(msg)

    # ---------------- Map callbacks ----------------
    def _on_map_full(self, msg: OccupancyGrid):
        raw = _i8_list_to_bytes(msg.data)

        meta = {
            "frame_id": msg.header.frame_id,
            "resolution": float(msg.info.resolution),
            "width": int(msg.info.width),
            "height": int(msg.info.height),
            "origin_x": float(msg.info.origin.position.x),
            "origin_y": float(msg.info.origin.position.y),
        }

        async def update_and_broadcast():
            async with _map_store.lock:
                _map_store.meta = meta
                _map_store.full_raw = raw
                _map_store.seq += 1
                seq = _map_store.seq

            await _broadcast_map_payload(
                {
                    "t": "f",
                    "seq": seq,
                    "meta": meta,
                    "gz": gzip.compress(raw, compresslevel=6),
                }
            )

        if self._loop:
            asyncio.run_coroutine_threadsafe(update_and_broadcast(), self._loop)

    def _on_map_update(self, msg: OccupancyGridUpdate):
        raw = _i8_list_to_bytes(msg.data)
        x = int(msg.x)
        y = int(msg.y)
        w = int(msg.width)
        h = int(msg.height)

        if self._loop is None:
            return

        def _schedule():
            async def _broadcast_only():
                async with _map_store.lock:
                    _map_store.seq += 1
                    seq = _map_store.seq

                await _broadcast_map_payload(
                    {
                        "t": "u",
                        "seq": seq,
                        "x": x,
                        "y": y,
                        "w": w,
                        "h": h,
                        "gz": gzip.compress(raw, compresslevel=6),
                    }
                )

            asyncio.create_task(_broadcast_only())

        self._loop.call_soon_threadsafe(_schedule)

    # ---------------- Front camera callback ----------------
    def _on_front_cam(self, msg: CompressedImage):
        store = get_cam_store()

        jpg = bytes(msg.data)
        meta = {
            "stamp": {
                "sec": int(msg.header.stamp.sec),
                "nanosec": int(msg.header.stamp.nanosec),
            },
            "frame_id": msg.header.frame_id,
            "format": msg.format,
        }

        async def fanout(clients):
            header = {"t": "cam", "seq": store.seq, "meta": store.meta, "n": len(store.jpg)}
            dead = []

            for ws in list(clients):
                try:
                    await ws.send_text(json.dumps(header))
                    await ws.send_bytes(store.jpg)
                except Exception:
                    dead.append(ws)

            if dead:
                async with store.lock:
                    for ws in dead:
                        store.clients.discard(ws)

        async def update_and_send():
            async with store.lock:
                store.seq += 1
                store.jpg = jpg
                store.meta = meta
                clients = set(store.clients)

            await fanout(clients)

        if self._loop:
            asyncio.run_coroutine_threadsafe(update_and_send(), self._loop)

    # ---------------- YOLO detections callback ----------------
    def _on_yolo_detections(self, msg: RosString):
        store = get_yolo_store()
        data = msg.data

        async def fanout():
            async with store.lock:
                store.seq += 1
                store.last_json = data
                clients = list(store.clients)

            dead = []
            for ws in clients:
                try:
                    await ws.send_text(data)
                except Exception:
                    dead.append(ws)

            if dead:
                async with store.lock:
                    for ws in dead:
                        store.clients.discard(ws)

        if self._loop:
            asyncio.run_coroutine_threadsafe(fanout(), self._loop)

    # ---------------- YOLO camera callback ----------------
    def _on_yolo_cam(self, msg: CompressedImage):
        store = get_yolo_cam_store()

        jpg = bytes(msg.data)
        meta = {
            "stamp": {
                "sec": int(msg.header.stamp.sec),
                "nanosec": int(msg.header.stamp.nanosec),
            },
            "frame_id": msg.header.frame_id,
            "format": msg.format,
        }

        async def fanout(clients):
            header = {"t": "cam", "seq": store.seq, "meta": store.meta, "n": len(store.jpg)}
            dead = []

            for ws in list(clients):
                try:
                    await ws.send_text(json.dumps(header))
                    await ws.send_bytes(store.jpg)
                except Exception:
                    dead.append(ws)

            if dead:
                async with store.lock:
                    for ws in dead:
                        store.clients.discard(ws)

        async def update_and_send():
            async with store.lock:
                store.seq += 1
                store.jpg = jpg
                store.meta = meta
                clients = set(store.clients)

            await fanout(clients)

        if self._loop:
            asyncio.run_coroutine_threadsafe(update_and_send(), self._loop)


# ============================================================
# LIFECYCLE HELPERS
# ============================================================
_bridge: Optional[WebRosBridge] = None


def start_ros_bridge():
    global _bridge

    if _bridge is not None:
        return _bridge

    rclpy.init(args=None)
    _bridge = WebRosBridge()

    t = threading.Thread(target=rclpy.spin, args=(_bridge,), daemon=True)
    t.start()

    _bridge.publish_enabled(True)
    return _bridge


def get_bridge() -> WebRosBridge:
    if _bridge is None:
        raise RuntimeError("ROS bridge not started")
    return _bridge