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
from std_msgs.msg import String as RosString
from std_msgs.msg import Bool, Float32, UInt8MultiArray
from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate
from fastapi import WebSocket
from sensor_msgs.msg import CompressedImage



# ============================================================
# 2D MAP STORE
# ============================================================
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


# ============================================================
# 3D POINTCLOUD STORE (xyz32)
# ============================================================
@dataclass
class PointCloudStore:
    meta: Optional[Dict[str, Any]] = None     # {frame_id, fmt, stride, ...}
    raw: Optional[bytes] = None               # packed float32 xyz (len = n*12)
    seq: int = 0
    clients: Set[WebSocket] = None
    lock: asyncio.Lock = None

    def __post_init__(self):
        if self.clients is None:
            self.clients = set()
        if self.lock is None:
            self.lock = asyncio.Lock()

_pcd_store = PointCloudStore()

async def _broadcast_pcd_payload(payload: dict):
    dead = []
    async with _pcd_store.lock:
        clients = list(_pcd_store.clients)

    for ws in clients:
        try:
            header = payload.copy()
            gz = header.pop("gz")
            await ws.send_text(json.dumps(header))
            await ws.send_bytes(gz)
        except Exception:
            dead.append(ws)

    if dead:
        async with _pcd_store.lock:
            for ws in dead:
                _pcd_store.clients.discard(ws)


# ============================================================
# CAMERA STORE (JPEG bytes)
# ============================================================
@dataclass
class CameraStore:
    meta: Optional[Dict[str, Any]] = None   # {frame_id, format}
    jpg: Optional[bytes] = None            # raw JPEG bytes
    seq: int = 0
    clients: Set[WebSocket] = None
    lock: asyncio.Lock = None

    def __post_init__(self):
        if self.clients is None:
            self.clients = set()
        if self.lock is None:
            self.lock = asyncio.Lock()

_cam_store = CameraStore()

async def _broadcast_cam_payload(payload: dict):
    dead = []
    async with _cam_store.lock:
        clients = list(_cam_store.clients)

    for ws in clients:
        try:
            header = payload.copy()
            jpg = header.pop("jpg")
            await ws.send_text(json.dumps(header))
            await ws.send_bytes(jpg)
        except Exception:
            dead.append(ws)

    if dead:
        async with _cam_store.lock:
            for ws in dead:
                _cam_store.clients.discard(ws)

def get_cam_store() -> CameraStore:
    return _cam_store

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
        self.move_forward_pub = self.create_publisher(Float32, "/move_forward_meters", 10)
        self.pub_sport_cmd = self.create_publisher(RosString, "/web_sport_cmd", 10)

        # ---------------- Map subscriptions ----------------
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,  # full map latched
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

        # ---------------- PointCloud subscriptions ----------------
        self.declare_parameter("pcd_xyz_topic", "/pcd/xyz32")
        self.declare_parameter("pcd_meta_topic", "/pcd/meta")

        pcd_xyz_topic = self.get_parameter("pcd_xyz_topic").value
        pcd_meta_topic = self.get_parameter("pcd_meta_topic").value

        pcd_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=2,
        )

        self.sub_pcd_meta = self.create_subscription(
            RosString, pcd_meta_topic, self._on_pcd_meta, 10
        )
        self.sub_pcd_xyz = self.create_subscription(
            UInt8MultiArray, pcd_xyz_topic, self._on_pcd_xyz32, pcd_qos
        )

        self._pcd_meta_cache = {"frame_id": "unknown", "fmt": "xyz32", "stride": 12}
        
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

        # AsyncIO loop provided by FastAPI
        self._loop: Optional[asyncio.AbstractEventLoop] = None

    def set_asyncio_loop(self, loop: asyncio.AbstractEventLoop):
        self._loop = loop

    # ---------------- Teleop / Action ----------------
    def _ok_to_publish(self) -> bool:
        try:
            return rclpy.ok()
        except Exception:
            return False

    def publish_teleop(self, vx: float, vy: float, vyaw: float):
        if not self._ok_to_publish():
            return
        msg = Twist()
        msg.linear.x = float(vx)
        msg.linear.y = float(vy)
        msg.angular.z = float(vyaw)
        try:
            self.pub_twist.publish(msg)
        except Exception:
            return

    def publish_action(self, action: str):
        if not self._ok_to_publish():
            return
        m = RosString()
        m.data = action
        self.pub_action.publish(m)

    def publish_move_forward(self, meters: float):
        if not self._ok_to_publish():
            return
        msg = Float32()
        msg.data = float(meters)
        self.move_forward_pub.publish(msg)

    def publish_enabled(self, enabled: bool):
        if not self._ok_to_publish():
            return
        b = Bool()
        b.data = bool(enabled)
        self.pub_enabled.publish(b)

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

                await _broadcast_map_payload({
                    "t": "f",
                    "seq": seq,
                    "meta": meta,
                    "gz": gzip.compress(raw, compresslevel=6),
                })

            asyncio.create_task(_set_and_broadcast())

        self._loop.call_soon_threadsafe(_schedule)

    def _on_map_update(self, msg: OccupancyGridUpdate):
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

                await _broadcast_map_payload({
                    "t": "u",
                    "seq": seq,
                    "x": x, "y": y, "w": w, "h": h,
                    "gz": gzip.compress(raw, compresslevel=6),
                })

            asyncio.create_task(_broadcast_only())

        self._loop.call_soon_threadsafe(_schedule)

    # ---------------- PointCloud callbacks ----------------
    def _on_pcd_meta(self, msg: RosString):
        try:
            meta = json.loads(msg.data)
            if not isinstance(meta, dict):
                raise ValueError("meta not dict")
        except Exception:
            meta = {"frame_id": str(msg.data), "fmt": "xyz32", "stride": 12}

        self._pcd_meta_cache = meta
        _pcd_store.meta = meta

    def _on_pcd_xyz32(self, msg: UInt8MultiArray):
        raw = bytes(msg.data)
        n = len(raw) // 12
        if n <= 0:
            return
    
        meta = dict(getattr(self, "_pcd_meta_cache", {"frame_id": "unknown"}))
        meta.setdefault("fmt", "xyz32")
        meta.setdefault("stride", 12)
    
        if self._loop is None:
            _pcd_store.meta = meta
            _pcd_store.raw = raw
            _pcd_store.seq += 1
            return
    
        def _schedule():
            async def _set_and_broadcast():
                async with _pcd_store.lock:
                    _pcd_store.meta = meta
                    _pcd_store.raw = raw
                    _pcd_store.seq += 1
                    seq = _pcd_store.seq
    
                payload = {
                    "t": "pcd",
                    "seq": seq,
                    "meta": meta,
                    "n": n,
                    "gz": gzip.compress(raw, compresslevel=6),
                }
                await _broadcast_pcd_payload(payload)
    
            asyncio.create_task(_set_and_broadcast())
    
        self._loop.call_soon_threadsafe(_schedule)

    # ---------------- Front camera callback ----------------
    def _on_front_cam(self, msg: CompressedImage):
        store = get_cam_store()
    
        jpg = bytes(msg.data)   # JPEG payload
        meta = {
            "stamp": {"sec": int(msg.header.stamp.sec), "nanosec": int(msg.header.stamp.nanosec)},
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
            for ws in dead:
                clients.discard(ws)
    
        async def update_and_send():
            async with store.lock:
                store.seq += 1
                store.jpg = jpg
                store.meta = meta
                clients = set(store.clients)  # snapshot under lock
    
            # send outside lock
            await fanout(clients)
    
        # schedule into FastAPI event loop
        if self._loop:
            asyncio.run_coroutine_threadsafe(update_and_send(), self._loop)


# ============================================================
# LIFECYCLE HELPERS
# ============================================================
_bridge = None

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

def get_map_store() -> MapStore:
    return _map_store

def get_pcd_store() -> PointCloudStore:
    return _pcd_store
