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
    SUB /web_estop         std_msgs/Bool         -> {"t":"estop","on":bool}
    (plus a 10 Hz {"t":"ping"} keepalive so the controller can deadman this link)

controller -> ROS (received over UDP, republished):
    {"t":"heartbeat"}          -> PUB /web_rl_heartbeat    std_msgs/Bool(true)  (~5 Hz)
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

DEF_UDP_HOST = "127.0.0.1"
DEF_CTRL_PORT = 47811    # the controller listens here (web -> control)
DEF_BRIDGE_PORT = 47812  # this node listens here (control -> web)

# Critical edge-triggered commands (mode/estop) are sent a few times to survive
# the rare localhost UDP drop; they are idempotent on the controller side.
CRITICAL_REPEAT = 3


class Go2RLBridge(Node):
    def __init__(self, udp_host, ctrl_port, bridge_port):
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
        self.create_subscription(Bool, "/web_estop", self._on_estop, 10)

        # controller -> ROS
        self._pub_hb = self.create_publisher(Bool, "/web_rl_heartbeat", 10)
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

    def _on_estop(self, msg: Bool):
        self._send({"t": "estop", "on": bool(msg.data)}, repeat=CRITICAL_REPEAT)

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
    args = ap.parse_args()

    rclpy.init()
    node = Go2RLBridge(args.udp_host, args.ctrl_port, args.bridge_port)
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
