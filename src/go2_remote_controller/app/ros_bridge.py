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
import time
from dataclasses import dataclass, field
from typing import Any, Dict, Optional, Set, Tuple

import rclpy
from fastapi import WebSocket
from geometry_msgs.msg import Twist
from map_msgs.msg import OccupancyGridUpdate
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Bool, Float32
from std_msgs.msg import String as RosString
from unitree_go.msg import LowState, SportModeState

from app.core.state import state
from app.services.perception_codec import (HAVE_CV2, color_image_to_jpeg,
                                           depth_image_to_jpeg, unpack_grid_map_layer)
from app.services.rate_meter import RateMeter

# Perception message types live in the OUTER Go2_RL_workflow workspace (go2_msgs) and
# in ros-humble-grid-map-msgs. Neither is required for the blind-policy page, so both
# imports are guarded exactly like go2_rl_bridge_node.py's: without them the matching
# subscription is skipped and /perception/status says so.
try:
    from go2_msgs.msg import HeightScan  # type: ignore
    HAVE_HEIGHT_SCAN = True
except Exception:  # pragma: no cover - depends on the sourced overlays
    HeightScan = None
    HAVE_HEIGHT_SCAN = False
try:
    from grid_map_msgs.msg import GridMap  # type: ignore
    HAVE_GRID_MAP = True
except Exception:  # pragma: no cover
    GridMap = None
    HAVE_GRID_MAP = False


# SDK motor_state[0..11] order -> URDF joint names (SDK name + "_joint").
# Mirror of go2_remote_viz/recording/lowstate_to_jointstate.py JOINT_NAMES —
# keep both lists in sync if the SDK ordering ever changes.
ROBOT_JOINT_NAMES = [
    "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
    "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
    "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
    "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
]


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
# ROBOT POSE STORE (joint angles + base pose, for the live 3D viewer)
# ============================================================
@dataclass
class RobotPoseStore:
    lock: asyncio.Lock = field(default_factory=asyncio.Lock)
    clients: Set[WebSocket] = field(default_factory=set)
    seq: int = 0
    last_joints: Optional[Dict[str, float]] = None
    last_base: Optional[Dict[str, Any]] = None


_robot_pose_store: Optional[RobotPoseStore] = None


def get_robot_pose_store() -> RobotPoseStore:
    global _robot_pose_store
    if _robot_pose_store is None:
        _robot_pose_store = RobotPoseStore()
    return _robot_pose_store


# ============================================================
# PERCEPTION STORE (RealSense previews + height map for the RL page)
# ============================================================
# Real-robot topics from go2_bringup/launch/real_perception*.launch.py. The USB 2
# launch streams depth only, so the colour topic may simply never publish.
RS_DEPTH_TOPIC = "/go2/camera/depth/image_rect_raw"
RS_COLOR_TOPIC = "/go2/camera/color/image_raw"
HEIGHT_SCAN_TOPIC = "/go2/height_scan"
GRID_MAP_TOPIC = "/go2/local_heightmap"
PREVIEW_MIN_PERIOD_S = 0.1        # JPEG encode cap per stream (10 Hz)
HEIGHT_MAP_BROADCAST_HZ = 15.0


@dataclass
class PerceptionStore:
    lock: asyncio.Lock = field(default_factory=asyncio.Lock)
    height_clients: Set[WebSocket] = field(default_factory=set)
    depth_clients: Set[WebSocket] = field(default_factory=set)
    color_clients: Set[WebSocket] = field(default_factory=set)
    seq_height: int = 0
    seq_cam: Dict[str, int] = field(default_factory=lambda: {"depth": 0, "color": 0})
    last_height_scan: Optional[Dict[str, Any]] = None
    last_grid_map: Optional[Dict[str, Any]] = None
    last_jpg: Dict[str, Optional[Tuple[bytes, Dict[str, Any]]]] = field(
        default_factory=lambda: {"depth": None, "color": None})
    rates: Dict[str, RateMeter] = field(default_factory=lambda: {
        "depth": RateMeter(), "color": RateMeter(),
        "height_scan": RateMeter(), "grid_map": RateMeter(),
    })
    scan_info: Optional[Dict[str, Any]] = None


_perception_store: Optional[PerceptionStore] = None


def get_perception_store() -> PerceptionStore:
    global _perception_store
    if _perception_store is None:
        _perception_store = PerceptionStore()
    return _perception_store


def get_perception_status() -> Dict[str, Any]:
    """JSON for GET /perception/status (the route adds the sysfs USB scan)."""
    st = get_perception_store()
    rates = {k: round(m.hz(), 1) for k, m in st.rates.items()}
    ages = {k: (None if m.age_s() is None else round(m.age_s(), 2)) for k, m in st.rates.items()}
    depth_alive = st.rates["depth"].alive()
    color_alive = st.rates["color"].alive()
    # Which launch is running: the USB 3 launch publishes colour, the USB 2 (depth-only)
    # launch cannot. Inferred from live topics, so it reports reality, not the link speed.
    mode = "usb3" if color_alive else ("usb2" if depth_alive else None)
    return {
        "driver_up": depth_alive or color_alive,
        "mode": mode,
        "height_scan_flowing": st.rates["height_scan"].alive(),
        "rates": rates,
        "ages_s": ages,
        "height_scan": st.scan_info,
        "topics": {
            "depth": RS_DEPTH_TOPIC, "color": RS_COLOR_TOPIC,
            "height_scan": HEIGHT_SCAN_TOPIC, "grid_map": GRID_MAP_TOPIC,
        },
        "available": {
            "height_scan_msg": HAVE_HEIGHT_SCAN,
            "grid_map_msg": HAVE_GRID_MAP,
            "cv2": HAVE_CV2,
        },
        "bridge_started": _bridge is not None,
    }


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
        self.pub_rl_posture = self.create_publisher(RosString, "/web_rl_posture", 1)
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

        # ---------------- Robot pose (joint angles + base pose) ----------------
        # Feeds the live 3D robot viewer on the web pages. /lowstate publishes far
        # faster than any UI needs, so callbacks only cache the latest sample —
        # a separate timer below does the actual (rate-limited) broadcast.
        self._last_joints: Optional[Dict[str, float]] = None
        self._last_base: Optional[Dict[str, Any]] = None

        self.sub_lowstate = self.create_subscription(
            LowState, "/lowstate", self._on_lowstate, cam_qos
        )
        self.sub_sportmodestate = self.create_subscription(
            SportModeState, "/sportmodestate", self._on_sportmodestate, cam_qos
        )
        self.create_timer(1.0 / 25.0, self._broadcast_robot_pose)

        # ---------------- RealSense previews + height map (RL page) ----------------
        # Raw images are subscribed whenever the driver publishes so the page can show
        # frame rates; JPEG encoding only happens while a /ws/cam_realsense client is
        # watching that stream, capped at 10 Hz. The height scan / GridMap callbacks
        # just cache; a 15 Hz timer broadcasts, like the robot pose above.
        self.declare_parameter("rs_depth_topic", RS_DEPTH_TOPIC)
        self.declare_parameter("rs_color_topic", RS_COLOR_TOPIC)
        self.declare_parameter("height_scan_topic", HEIGHT_SCAN_TOPIC)
        self.declare_parameter("grid_map_topic", GRID_MAP_TOPIC)
        rs_depth_topic = self.get_parameter("rs_depth_topic").value
        rs_color_topic = self.get_parameter("rs_color_topic").value
        height_scan_topic = self.get_parameter("height_scan_topic").value
        grid_map_topic = self.get_parameter("grid_map_topic").value

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._rs_last_encode: Dict[str, float] = {"depth": 0.0, "color": 0.0}
        self._hm_pending: Dict[str, bool] = {"height_scan": False, "grid_map": False}
        self.sub_rs_depth = self.create_subscription(
            Image, rs_depth_topic, lambda m: self._on_rs_image("depth", m), sensor_qos)
        self.sub_rs_color = self.create_subscription(
            Image, rs_color_topic, lambda m: self._on_rs_image("color", m), sensor_qos)
        if HAVE_HEIGHT_SCAN:
            self.sub_height_scan = self.create_subscription(
                HeightScan, height_scan_topic, self._on_height_scan, sensor_qos)
        else:
            self.get_logger().warn(
                "go2_msgs not importable: /go2/height_scan view disabled "
                "(source the outer Go2_RL_workflow install/setup.bash)")
        if HAVE_GRID_MAP:
            self.sub_grid_map = self.create_subscription(
                GridMap, grid_map_topic, self._on_grid_map, sensor_qos)
        else:
            self.get_logger().warn(
                "grid_map_msgs not importable: /go2/local_heightmap columns view disabled")
        if not HAVE_CV2:
            self.get_logger().warn("cv2 not importable: RealSense image preview disabled")
        self.create_timer(1.0 / HEIGHT_MAP_BROADCAST_HZ, self._broadcast_height_map)

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

    def publish_rl_posture(self, posture: str):
        if not self._ok_to_publish():
            return

        msg = RosString()
        msg.data = str(posture)
        self.pub_rl_posture.publish(msg)

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

    # ---------------- Robot pose callbacks ----------------
    def _on_lowstate(self, msg: LowState):
        # Mirror of go2_remote_viz/recording/lowstate_to_jointstate.py's mapping.
        self._last_joints = {
            name: float(msg.motor_state[i].q)
            for i, name in enumerate(ROBOT_JOINT_NAMES)
        }

    def _on_sportmodestate(self, msg: SportModeState):
        # Mirror of go2_remote_viz/recording/sportmodestate_to_tf.py's mapping.
        # Unitree quaternion order is [w, x, y, z]; ROS/three.js want x, y, z, w.
        qw, qx, qy, qz = (float(v) for v in msg.imu_state.quaternion)
        self._last_base = {
            "position": [float(msg.position[0]), float(msg.position[1]), float(msg.position[2])],
            "quaternion": [qx, qy, qz, qw],
        }

    def _broadcast_robot_pose(self):
        if self._last_joints is None and self._last_base is None:
            return

        store = get_robot_pose_store()
        joints = self._last_joints
        base = self._last_base

        async def fanout():
            async with store.lock:
                store.seq += 1
                store.last_joints = joints
                store.last_base = base
                seq = store.seq
                clients = list(store.clients)

            if not clients:
                return

            data = json.dumps({"t": "pose", "seq": seq, "joints": joints, "base": base})
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


    # ---------------- RealSense preview callbacks ----------------
    def _on_rs_image(self, stream: str, msg: Image):
        store = get_perception_store()
        store.rates[stream].tick()

        clients = store.depth_clients if stream == "depth" else store.color_clients
        if not clients or not HAVE_CV2:
            return
        now = time.monotonic()
        if now - self._rs_last_encode[stream] < PREVIEW_MIN_PERIOD_S:
            return
        self._rs_last_encode[stream] = now

        data = bytes(msg.data)
        if stream == "depth":
            jpg = depth_image_to_jpeg(data, msg.height, msg.width, msg.encoding,
                                      msg.step, bool(msg.is_bigendian))
        else:
            jpg = color_image_to_jpeg(data, msg.height, msg.width, msg.encoding, msg.step)
        if jpg is None:
            return
        meta = {
            "stamp": {"sec": int(msg.header.stamp.sec), "nanosec": int(msg.header.stamp.nanosec)},
            "frame_id": msg.header.frame_id,
            "encoding": msg.encoding,
            "width": int(msg.width),
            "height": int(msg.height),
        }

        async def update_and_send():
            async with store.lock:
                store.seq_cam[stream] += 1
                store.last_jpg[stream] = (jpg, meta)
                seq = store.seq_cam[stream]
                targets = list(clients)
            header = {"t": "cam", "stream": stream, "seq": seq, "meta": meta, "n": len(jpg)}
            dead = []
            for ws in targets:
                try:
                    await ws.send_text(json.dumps(header))
                    await ws.send_bytes(jpg)
                except Exception:
                    dead.append(ws)
            if dead:
                async with store.lock:
                    for ws in dead:
                        clients.discard(ws)

        if self._loop:
            asyncio.run_coroutine_threadsafe(update_and_send(), self._loop)

    # ---------------- Height map callbacks ----------------
    def _on_height_scan(self, msg):
        store = get_perception_store()
        store.rates["height_scan"].tick()
        heights = [round(float(h), 3) for h in msg.heights]
        info = {
            "num_x": int(msg.num_x), "num_y": int(msg.num_y),
            "resolution": float(msg.resolution),
            "center_x": float(msg.center_x), "center_y": float(msg.center_y),
            "offset": float(msg.offset),
            "clip_min": float(msg.clip_min), "clip_max": float(msg.clip_max),
            "unobserved_value": float(msg.unobserved_value),
            "unobserved_count": int(msg.unobserved_count),
            "frame_id": msg.header.frame_id,
        }
        store.scan_info = info
        store.last_height_scan = {
            "t": "height_scan",
            "stamp": {"sec": int(msg.header.stamp.sec), "nanosec": int(msg.header.stamp.nanosec)},
            **info,
            "heights": heights,
        }
        self._hm_pending["height_scan"] = True

    def _on_grid_map(self, msg):
        store = get_perception_store()
        store.rates["grid_map"].tick()
        layers = list(msg.layers)
        if not layers or not msg.data:
            return
        dims = msg.data[0].layout.dim
        if len(dims) < 2:
            return
        n_y, n_x = int(dims[0].size), int(dims[1].size)   # column_index, row_index
        out: Dict[str, Any] = {
            "t": "grid_map",
            "stamp": {"sec": int(msg.header.stamp.sec), "nanosec": int(msg.header.stamp.nanosec)},
            "frame_id": msg.header.frame_id,
            "num_x": n_x, "num_y": n_y,
            "resolution": float(msg.info.resolution),
            "length_x": float(msg.info.length_x), "length_y": float(msg.info.length_y),
            "pose_x": float(msg.info.pose.position.x),
            "pose_y": float(msg.info.pose.position.y),
            "layers": {},
        }
        for name in ("elevation", "min_elevation"):
            if name in layers:
                try:
                    out["layers"][name] = unpack_grid_map_layer(
                        msg.data[layers.index(name)].data, n_x, n_y)
                except ValueError as e:
                    self.get_logger().warn(f"grid_map layer {name}: {e}", throttle_duration_sec=5.0)
        if "elevation" not in out["layers"]:
            return
        store.last_grid_map = out
        self._hm_pending["grid_map"] = True

    def _broadcast_height_map(self):
        store = get_perception_store()
        msgs = []
        if self._hm_pending["height_scan"] and store.last_height_scan:
            self._hm_pending["height_scan"] = False
            msgs.append(store.last_height_scan)
        if self._hm_pending["grid_map"] and store.last_grid_map:
            self._hm_pending["grid_map"] = False
            msgs.append(store.last_grid_map)
        if not msgs or not store.height_clients:
            return

        async def fanout():
            async with store.lock:
                store.seq_height += 1
                clients = list(store.height_clients)
            if not clients:
                return
            payloads = [json.dumps(m) for m in msgs]
            dead = []
            for ws in clients:
                try:
                    for p in payloads:
                        await ws.send_text(p)
                except Exception:
                    dead.append(ws)
            if dead:
                async with store.lock:
                    for ws in dead:
                        store.height_clients.discard(ws)

        if self._loop:
            asyncio.run_coroutine_threadsafe(fanout(), self._loop)


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