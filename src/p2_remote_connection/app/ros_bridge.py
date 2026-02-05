import threading
import asyncio
import gzip
import json
from dataclasses import dataclass
from typing import Optional, Set, Dict, Any

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate
from fastapi import WebSocket
from std_msgs.msg import Bool


@dataclass
class MapStore:
    meta: Optional[Dict[str, Any]] = None     # {frame_id,resolution,width,height,origin_x,origin_y}
    full_raw: Optional[bytes] = None          # int8 raw bytes (len = width*height)
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
    """
    Send 2 frames: JSON header then gzipped binary bytes.
    """
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


class WebRosBridge(Node):
    def __init__(self):
        super().__init__("web_ros_bridge")

        # Publishers
        self.pub_twist = self.create_publisher(Twist, "/web_teleop", 10)
        self.pub_action = self.create_publisher(String, "/web_action", 10)
        self.pub_enabled = self.create_publisher(Bool, "/web_teleop_enabled", 1)
        self.move_forward_pub = self.node.create_publisher(Float32, "/move_forward_meters", 10)

        # Map subscriptions
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,  # match your /map2d publisher
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        upd_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.sub_map_full = self.create_subscription(
            OccupancyGrid, "/map2d", self._on_map_full, map_qos
        )
        self.sub_map_upd = self.create_subscription(
            OccupancyGridUpdate, "/map2d_updates", self._on_map_update, upd_qos
        )

        self._loop: Optional[asyncio.AbstractEventLoop] = None

    def set_asyncio_loop(self, loop: asyncio.AbstractEventLoop):
        """
        Call this once from FastAPI startup so ROS callbacks can schedule WS broadcasts.
        """
        self._loop = loop

    # ---------------- Teleop / Action ----------------
    def publish_teleop(self, vx: float, vy: float, vyaw: float):
        msg = Twist()
        msg.linear.x = float(vx)
        msg.linear.y = float(vy)
        msg.angular.z = float(vyaw)
        self.pub_twist.publish(msg)

    def publish_action(self, action: str):
        m = String()
        m.data = action
        self.pub_action.publish(m)

    # ---------------- Move forward helper ----------------
    def publish_move_forward(self, meters: float):
        msg = Float32()
        msg.data = float(meters)
        self.move_forward_pub.publish(msg)

    # ---------------- Map callbacks ----------------
    def _on_map_full(self, msg: OccupancyGrid):
        raw = _i8_list_to_bytes(msg.data)
        meta = {
            "frame_id": str(msg.header.frame_id),
            "resolution": float(msg.info.resolution),
            "width": int(msg.info.width),
            "height": int(msg.info.height),
            "origin_x": float(msg.info.origin.position.x),
            "origin_y": float(msg.info.origin.position.y),
        }

        if self._loop is None:
            # FastAPI hasn't set loop yet; just store the latest.
            _map_store.meta = meta
            _map_store.full_raw = raw
            _map_store.seq += 1
            return

        def _schedule():
            async def _set_and_broadcast():
                async with _map_store.lock:
                    _map_store.meta = meta
                    _map_store.full_raw = raw
                    _map_store.seq += 1
                    seq = _map_store.seq
                payload = {
                    "t": "f",
                    "seq": seq,
                    "meta": meta,
                    "gz": gzip.compress(raw, compresslevel=6),
                }
                await _broadcast_map_payload(payload)

            asyncio.create_task(_set_and_broadcast())

        self._loop.call_soon_threadsafe(_schedule)

    def _on_map_update(self, msg: OccupancyGridUpdate):
        # ignore updates until we have a full map
        if _map_store.meta is None or _map_store.full_raw is None:
            return

        raw = _i8_list_to_bytes(msg.data)
        x, y = int(msg.x), int(msg.y)
        w, h = int(msg.width), int(msg.height)

        if self._loop is None:
            return

        def _schedule():
            async def _broadcast_only():
                async with _map_store.lock:
                    _map_store.seq += 1
                    seq = _map_store.seq
                payload = {
                    "t": "u",
                    "seq": seq,
                    "x": x, "y": y, "w": w, "h": h,
                    "gz": gzip.compress(raw, compresslevel=6),
                }
                await _broadcast_map_payload(payload)

            asyncio.create_task(_broadcast_only())

        self._loop.call_soon_threadsafe(_schedule)

    def publish_enabled(self, enabled: bool):
        b = Bool()
        b.data = bool(enabled)
        self.pub_enabled.publish(b)

_bridge = None


def start_ros_bridge():
    """
    Starts ROS once and spins in a background thread.
    """
    global _bridge
    if _bridge is not None:
        return _bridge

    rclpy.init(args=None)
    _bridge = WebRosBridge()

    t = threading.Thread(target=rclpy.spin, args=(_bridge,), daemon=True)
    t.start()

    # default: enabled
    _bridge.publish_enabled(True)

    return _bridge


def get_bridge() -> WebRosBridge:
    global _bridge
    if _bridge is None:
        raise RuntimeError("ROS bridge not started")
    return _bridge


def get_map_store() -> MapStore:
    return _map_store
