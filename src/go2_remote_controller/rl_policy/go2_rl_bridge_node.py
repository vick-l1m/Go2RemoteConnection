#!/usr/bin/env python3
"""ROS 2 <-> UDP bridge for the Go2 low-level RL controller.

The RL controller (``go2_rl_policy_node.py``) is a pure-``unitree_sdk2py``
process with NO rclpy, because the Unitree SDK and rmw_cyclonedds cannot both
create DDS domain 0 in one process (see that file's header). This node is the
other half of the split: it is pure rclpy (no Unitree SDK), exactly like
``web_bridge``, so it coexists with the rest of the ROS stack without any DDS
conflict. It relays the web control topics to/from the controller over a
localhost UDP link.

ROS -> controller (forwarded as JSON UDP datagrams):
    SUB /web_teleop        geometry_msgs/Twist   -> {"t":"teleop","vx","vy","wz"}
    SUB /web_control_mode  std_msgs/String       -> {"t":"mode","mode":"sport"|"rl"}
    SUB /web_rl_policy     std_msgs/String       -> {"t":"policy","id":str}
    SUB /web_rl_posture    std_msgs/String       -> {"t":"posture","stand":bool}
    SUB /web_estop         std_msgs/Bool         -> {"t":"estop","on":bool}
    (plus a 10 Hz {"t":"ping"} keepalive so the controller can deadman this link)
    SUB /go2/height_scan   go2_msgs/HeightScan   -> binary frame (see height_scan_wire)

The height scan is the one message on this link that is NOT JSON. It is 187 floats at
camera rate, so it goes over as a packed binary frame with a magic prefix the receiver
tests before trying json.loads(); see height_scan_wire.py for the format and why.
It is also the one subscription that may be unavailable: go2_msgs is built by the
OUTER Go2_RL_workflow workspace, which RL_start_remote_connection.sh does not source.
When the import fails this node logs it once and runs exactly as before, so the blind
(flat) policies are unaffected -- only perception policies refuse to engage.

controller -> ROS (received over UDP, republished):
    {"t":"heartbeat"}          -> PUB /web_rl_heartbeat    std_msgs/Bool(true)  (~5 Hz)
    {"t":"policy_out","id"}    -> PUB /web_rl_active_policy std_msgs/String      (loaded policy)
    {"t":"mode_out","mode"}    -> PUB /web_control_mode     std_msgs/String      (shutdown un-gate)
    {"t":"enabled","val"}      -> PUB /web_teleop_enabled   std_msgs/Bool        (shutdown un-gate)
"""

import argparse
import json
import os
import socket
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String

import height_scan_wire

# go2_msgs lives in the outer Go2_RL_workflow colcon workspace, which the RL launcher
# does not source (it sources only this submodule's overlay). Import defensively so a
# robot without the perception workspace built still runs the blind policies, and says
# clearly why a perception policy will not engage.
try:
    from go2_msgs.msg import HeightScan
    HEIGHT_SCAN_AVAILABLE = True
    HEIGHT_SCAN_IMPORT_ERROR = ""
except ImportError as _e:  # pragma: no cover - depends on the robot's built overlay
    HeightScan = None
    HEIGHT_SCAN_AVAILABLE = False
    HEIGHT_SCAN_IMPORT_ERROR = str(_e)

DEF_UDP_HOST = "127.0.0.1"
DEF_CTRL_PORT = 47811    # the controller listens here (web -> control)
DEF_BRIDGE_PORT = 47812  # this node listens here (control -> web)

# Critical edge-triggered commands (mode/estop) are sent a few times to survive
# the rare localhost UDP drop; they are idempotent on the controller side.
CRITICAL_REPEAT = 3

#: Default topic carrying the perception height scan (go2_perception/heightmap_node).
DEF_HEIGHT_SCAN_TOPIC = "/go2/height_scan"


class Go2RLBridge(Node):
    def __init__(self, udp_host, ctrl_port, bridge_port, height_scan_topic=DEF_HEIGHT_SCAN_TOPIC):
        super().__init__("go2_rl_bridge_node")

        self._ctrl_addr = (udp_host, ctrl_port)
        self._tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._rx.bind((udp_host, bridge_port))
        self._rx.settimeout(0.5)
        self._stop = False

        # ROS -> controller
        self.create_subscription(Twist, "/web_teleop", self._on_teleop, 10)
        self.create_subscription(String, "/web_control_mode", self._on_mode, 10)
        self.create_subscription(String, "/web_rl_policy", self._on_policy, 10)
        self.create_subscription(String, "/web_rl_posture", self._on_posture, 10)
        self.create_subscription(Bool, "/web_estop", self._on_estop, 10)

        # Perception -> controller. Depth 1: a height scan is only ever useful fresh,
        # and queueing stale terrain behind a momentary stall is worse than dropping it
        # (the controller deadmans on scan age anyway).
        self._scan_seq = 0
        self._scan_logged = False
        if HEIGHT_SCAN_AVAILABLE and height_scan_topic:
            self.create_subscription(HeightScan, height_scan_topic, self._on_height_scan, 1)
            self.get_logger().info(f"forwarding height scans from {height_scan_topic}")
        elif not HEIGHT_SCAN_AVAILABLE:
            self.get_logger().warn(
                f"go2_msgs not importable ({HEIGHT_SCAN_IMPORT_ERROR}); height scans will NOT "
                "be forwarded, so perception policies cannot engage. Build and source the "
                "outer Go2_RL_workflow workspace (colcon build --packages-select go2_msgs "
                "go2_perception) to enable them. Blind policies are unaffected.")

        # controller -> ROS
        self._pub_hb = self.create_publisher(Bool, "/web_rl_heartbeat", 10)
        self._pub_active_policy = self.create_publisher(String, "/web_rl_active_policy", 1)
        self._pub_mode_out = self.create_publisher(String, "/web_control_mode", 1)
        self._pub_enabled_out = self.create_publisher(Bool, "/web_teleop_enabled", 1)

        # 10 Hz keepalive so the controller's bridge-link deadman stays satisfied
        # even when the joystick is idle (no /web_teleop traffic).
        self.create_timer(0.1, lambda: self._send({"t": "ping"}))

        self._rxthread = threading.Thread(target=self._udp_rx_loop, daemon=True)
        self._rxthread.start()

        self.get_logger().info(
            f"go2_rl_bridge_node ready: ROS <-> udp {udp_host} "
            f"(control:{ctrl_port} <- / bridge:{bridge_port} ->)")

    def _send(self, obj, repeat=1):
        data = json.dumps(obj).encode("utf-8")
        for _ in range(repeat):
            try:
                self._tx.sendto(data, self._ctrl_addr)
            except OSError:
                break

    # ---- ROS -> controller ------------------------------------------- #
    def _on_teleop(self, msg: Twist):
        self._send({"t": "teleop", "vx": msg.linear.x, "vy": msg.linear.y, "wz": msg.angular.z})

    def _on_mode(self, msg: String):
        new = msg.data.strip().lower()
        if new in ("sport", "rl"):
            self._send({"t": "mode", "mode": new}, repeat=CRITICAL_REPEAT)

    def _on_policy(self, msg: String):
        pid = msg.data.strip()
        if pid:
            self._send({"t": "policy", "id": pid}, repeat=CRITICAL_REPEAT)

    def _on_posture(self, msg: String):
        # Edge-triggered and latched on the controller, so repeat it like mode/estop:
        # a dropped datagram here leaves the robot holding the previous posture with
        # the UI showing the new one.
        want = msg.data.strip().lower()
        if want in ("sit", "stand"):
            self._send({"t": "posture", "stand": want == "stand"}, repeat=CRITICAL_REPEAT)

    def _on_estop(self, msg: Bool):
        self._send({"t": "estop", "on": bool(msg.data)}, repeat=CRITICAL_REPEAT)

    def _on_height_scan(self, msg):
        """Forward one scan as a binary frame (see height_scan_wire).

        Sent raw and unsorted: heightmap_node already produced RayCaster order with the
        training clip/offset applied, and re-touching either here is the silent scramble
        HeightScan.msg warns about. Never repeated -- a dropped frame is replaced by the
        next one microseconds later, and the controller's staleness deadman covers a
        real outage.
        """
        try:
            frame = height_scan_wire.encode(
                msg.heights,
                num_x=msg.num_x,
                num_y=msg.num_y,
                resolution=msg.resolution,
                center_x=msg.center_x,
                center_y=msg.center_y,
                unobserved_count=msg.unobserved_count,
                seq=self._scan_seq,
            )
        except height_scan_wire.HeightScanWireError as e:
            # Throttled: a geometry fault repeats at camera rate.
            self.get_logger().error(f"height scan not encodable, dropping: {e}", throttle_duration_sec=5.0)
            return
        self._scan_seq = (self._scan_seq + 1) & 0xFFFFFFFF
        if not self._scan_logged:
            self._scan_logged = True
            self.get_logger().info(
                f"first height scan: {msg.num_x}x{msg.num_y} @ {msg.resolution:.3g} m, "
                f"centre (+{msg.center_x:.2g}, {msg.center_y:+.2g}) m, "
                f"{msg.unobserved_count}/{msg.num_x * msg.num_y} cells unobserved, "
                f"{len(frame)} B/frame")
        try:
            self._tx.sendto(frame, self._ctrl_addr)
        except OSError:
            pass

    # ---- controller -> ROS ------------------------------------------- #
    def _udp_rx_loop(self):
        while not self._stop:
            try:
                data, _ = self._rx.recvfrom(4096)
            except socket.timeout:
                continue
            except OSError:
                if self._stop:
                    break
                continue
            try:
                msg = json.loads(data.decode("utf-8"))
                t = msg.get("t")
            except (ValueError, AttributeError):
                continue
            if t == "heartbeat":
                self._pub_hb.publish(Bool(data=True))
            elif t == "policy_out":
                self._pub_active_policy.publish(String(data=str(msg.get("id") or "")))
            elif t == "mode_out":
                self._pub_mode_out.publish(String(data=str(msg.get("mode", "sport"))))
            elif t == "enabled":
                self._pub_enabled_out.publish(Bool(data=bool(msg.get("val", True))))

    def destroy_node(self):
        self._stop = True
        try:
            self._rx.close()
        except Exception:  # noqa: BLE001
            pass
        super().destroy_node()


def main():
    ap = argparse.ArgumentParser(description="ROS<->UDP bridge for the Go2 RL controller")
    ap.add_argument("--udp-host", default=os.environ.get("GO2_RL_UDP_HOST", DEF_UDP_HOST))
    ap.add_argument("--ctrl-port", type=int,
                    default=int(os.environ.get("GO2_RL_CTRL_PORT", DEF_CTRL_PORT)))
    ap.add_argument("--bridge-port", type=int,
                    default=int(os.environ.get("GO2_RL_BRIDGE_PORT", DEF_BRIDGE_PORT)))
    ap.add_argument("--height-scan-topic",
                    default=os.environ.get("GO2_RL_HEIGHT_SCAN_TOPIC", DEF_HEIGHT_SCAN_TOPIC),
                    help="go2_msgs/HeightScan topic to forward to the controller "
                         "(empty string disables forwarding)")
    args = ap.parse_args()

    rclpy.init()
    node = Go2RLBridge(args.udp_host, args.ctrl_port, args.bridge_port, args.height_scan_topic)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.try_shutdown()
        except Exception:  # noqa: BLE001
            pass


if __name__ == "__main__":
    main()
