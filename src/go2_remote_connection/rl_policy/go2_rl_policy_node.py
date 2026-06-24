#!/usr/bin/env python3
"""Low-level RL locomotion controller for the Unitree Go2.

Runs the flat-terrain policy trained in Isaac Lab (RSL-RL PPO,
``Isaac-Velocity-Flat-Unitree-Go2-v0``) directly on the robot's motors, driven
by the Go2RemoteConnection web joystick. Sits alongside the existing
``web_bridge`` (Unitree SportClient) path; the operator switches between them
from the website.

Topics (ROS 2 / rclpy):
    SUB  /web_teleop        geometry_msgs/Twist   joystick (vx=linear.x, vy=linear.y, wz=angular.z)
    SUB  /web_control_mode  std_msgs/String       "sport" | "rl"

Robot (unitree_sdk2py / DDS):
    SUB  rt/lowstate   LowState_   IMU (quat, gyro) + per-joint q, dq
    PUB  rt/lowcmd     LowCmd_     joint position targets @ 50 Hz with PD gains
    MotionSwitcherClient / SportClient   release / recover the high-level sport service

Control mode handover:
    sport -> rl : StandDown (crouch) -> ReleaseMode (kill sport svc) -> ramp to
                  default pose under PD -> run policy @ 50 Hz.
    rl -> sport : ramp command to 0, PD-hold default -> SelectMode + BalanceStand
                  (sport svc takes over) -> stop publishing lowcmd.

DEPLOYMENT CONTRACT (must match training -- see sim_to_real_deployment_plan.md):
    rate 50 Hz; q_target = q_default + 0.25*action; kp=25, kd=0.5;
    obs = [base_lin_vel(3), base_ang_vel(3), proj_gravity(3), cmd(3),
           q-q_default(12), dq(12), last_action(12)]  (48 dims, no normalization).

base_lin_vel (obs 0:3) is NOT observable in low-level mode; with the un-retrained
48-dim policy it is fed zeros (``--lin-vel-mode zero``). Expect a rougher gait
than sim -- this build validates the pipeline, not gait quality.

!!! TEST ON A GANTRY FIRST, feet off the ground, E-stop in hand. !!!
"""

import argparse
import json
import pathlib
import signal
import threading
import time

import numpy as np
import onnxruntime as ort

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String

from unitree_sdk2py.core.channel import (
    ChannelFactoryInitialize,
    ChannelPublisher,
    ChannelSubscriber,
)
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.utils.thread import RecurrentThread
from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient
from unitree_sdk2py.go2.sport.sport_client import SportClient

# ----------------------------------------------------------------------------
# Constants
# ----------------------------------------------------------------------------
HERE = pathlib.Path(__file__).resolve().parent

# unitree_legged_const
PosStopF = 2.146e9
VelStopF = 16000.0

# SDK joint index order: rt/lowstate motor_state[i] and rt/lowcmd motor_cmd[i].
SDK_JOINTS = [
    "FR_hip", "FR_thigh", "FR_calf",
    "FL_hip", "FL_thigh", "FL_calf",
    "RR_hip", "RR_thigh", "RR_calf",
    "RL_hip", "RL_thigh", "RL_calf",
]

# Default joint angles by name (env.yaml init_state.joint_pos). Single source of
# truth; per-order arrays are derived from this so they cannot drift apart.
Q_DEFAULT_BY_NAME = {
    "FL_hip": 0.1, "RL_hip": 0.1, "FR_hip": -0.1, "RR_hip": -0.1,
    "FL_thigh": 0.8, "FR_thigh": 0.8, "RL_thigh": 1.0, "RR_thigh": 1.0,
    "FL_calf": -1.5, "FR_calf": -1.5, "RL_calf": -1.5, "RR_calf": -1.5,
}

# Deployment contract (from env.yaml / agent.yaml).
CONTROL_DT = 0.02      # 50 Hz policy rate (sim dt 0.005 * decimation 4)
ACTION_SCALE = 0.25
KP, KD = 25.0, 0.5     # trained actuator gains
RAMP_KP, RAMP_KD = 40.0, 4.0   # firmer during the stand-up ramp
RAMP_TIME = 2.0        # s, measured pose -> default pose
DISENGAGE_HOLD = 1.0   # s, PD-hold default before handing back to sport
CMD_TIMEOUT = 0.5      # s, deadman on /web_teleop
CMD_CLIP = 1.0         # training command range +/-1
DAMP_KD = 3.0          # damping fallback (soft collapse) on fault

# Control-phase state machine.
#   SPORT     idle, the Unitree sport service owns the robot (node publishes nothing)
#   ENGAGE    ramping measured pose -> default pose under PD, then -> RL_RUN
#   RL_RUN    policy active @ 50 Hz
#   DISENGAGE PD-hold default while handing back to the sport service
#   ESTOP     emergency stop: continuously damp (soft collapse), latched until RESUME
SPORT, ENGAGE, RL_RUN, DISENGAGE, ESTOP = "SPORT", "ENGAGE", "RL_RUN", "DISENGAGE", "ESTOP"


def projected_gravity(quat_wxyz):
    """Body-frame gravity unit vector from the IMU quaternion (w, x, y, z).

    Matches Isaac Lab's ``projected_gravity_b`` = quat_rotate_inverse(q, [0,0,-1]).
    Upright -> (0, 0, -1).
    """
    w, x, y, z = quat_wxyz
    g = np.array([2 * (x * z - w * y),
                  2 * (y * z + w * x),
                  1 - 2 * (x * x + y * y)], dtype=np.float32)
    return -g


def load_isaac_joints(path):
    """Read the Isaac-Lab joint order (strip '_joint'); fall back to the documented default."""
    default = ["FL_hip", "FR_hip", "RL_hip", "RR_hip",
               "FL_thigh", "FR_thigh", "RL_thigh", "RR_thigh",
               "FL_calf", "FR_calf", "RL_calf", "RR_calf"]
    try:
        names = json.loads(pathlib.Path(path).read_text())["joint_names"]
        names = [n.replace("_joint", "") for n in names]
        assert sorted(names) == sorted(SDK_JOINTS), "joint set mismatch"
        return names, False
    except Exception as e:  # noqa: BLE001
        print(f"[WARN] could not load {path} ({e}); using documented default order. "
              "VERIFY with dump_isaac_joint_order.py before ground tests.")
        return default, True


class Go2RLPolicyNode(Node):
    def __init__(self, args):
        super().__init__("go2_rl_policy_node")
        self.dry_run = args.dry_run
        self.handover = not args.no_handover
        self.lin_vel_mode = args.lin_vel_mode

        # ---- policy + joint remap ----------------------------------------
        self.session = ort.InferenceSession(args.policy, providers=["CPUExecutionProvider"])
        self.in_name = self.session.get_inputs()[0].name
        self.obs_dim = int(self.session.get_inputs()[0].shape[1])
        isaac_joints, used_default = load_isaac_joints(args.joint_names)
        self.isaac_joints = isaac_joints
        self.isaac_from_sdk = [SDK_JOINTS.index(n) for n in isaac_joints]   # lowstate -> obs order
        self.sdk_from_isaac = [isaac_joints.index(n) for n in SDK_JOINTS]   # action  -> lowcmd order
        self.q_default_isaac = np.array([Q_DEFAULT_BY_NAME[n] for n in isaac_joints], np.float32)
        self.q_default_sdk = np.array([Q_DEFAULT_BY_NAME[n] for n in SDK_JOINTS], np.float32)
        self.get_logger().info(
            f"policy={args.policy} obs_dim={self.obs_dim} "
            f"joint_order={'DEFAULT(verify!)' if used_default else 'joint_names.json'} "
            f"dry_run={self.dry_run} handover={self.handover} lin_vel={self.lin_vel_mode}")

        # ---- shared state ------------------------------------------------
        self._lock = threading.Lock()
        self._sport_lock = threading.Lock()
        self.low_state = None
        self.cmd = np.zeros(3, np.float32)
        self.last_cmd_t = 0.0
        self.last_action = np.zeros(12, np.float32)
        self.phase = SPORT
        self.requested_mode = "sport"
        self.start_pos_sdk = self.q_default_sdk.copy()
        self.ramp_t = 0.0
        self._recover_pending = False
        self._wake = threading.Event()
        self._stop = False
        self._t0 = time.monotonic()

        # ---- DDS lowcmd/lowstate ----------------------------------------
        self.crc = CRC()
        self.low_cmd = unitree_go_msg_dds__LowCmd_()
        self._init_low_cmd()
        self.lowcmd_pub = ChannelPublisher("rt/lowcmd", LowCmd_)
        self.lowcmd_pub.Init()
        self.lowstate_sub = ChannelSubscriber("rt/lowstate", LowState_)
        self.lowstate_sub.Init(self._on_lowstate, 10)

        # ---- sport service control --------------------------------------
        self.sc = SportClient(); self.sc.SetTimeout(5.0); self.sc.Init()
        self.msc = MotionSwitcherClient(); self.msc.SetTimeout(5.0); self.msc.Init()

        # ---- ROS subscriptions ------------------------------------------
        self.create_subscription(Twist, "/web_teleop", self._on_teleop, 10)
        self.create_subscription(String, "/web_control_mode", self._on_mode, 10)
        self.create_subscription(Bool, "/web_estop", self._on_estop, 10)

        # Failsafe publishers: a liveness heartbeat (so the web stack can detect a
        # dead node and auto-revert to sport), and one-shot un-gate signals sent on
        # shutdown so web_bridge is never left latched into RL mode.
        self._pub_hb = self.create_publisher(Bool, "/web_rl_heartbeat", 10)
        self._pub_mode_out = self.create_publisher(String, "/web_control_mode", 1)
        self._pub_enabled_out = self.create_publisher(Bool, "/web_teleop_enabled", 1)
        self._tick = 0

        # ---- threads -----------------------------------------------------
        self._tworker = threading.Thread(target=self._transition_worker, daemon=True)
        self._tworker.start()
        self.ctrl_thread = RecurrentThread(interval=CONTROL_DT, target=self._control_step,
                                           name="rl_control")
        self.ctrl_thread.Start()
        self.get_logger().info("go2_rl_policy_node ready (mode=sport, idle).")

    # ------------------------------------------------------------------ #
    # Callbacks
    # ------------------------------------------------------------------ #
    def _on_lowstate(self, msg: LowState_):
        self.low_state = msg

    def _on_teleop(self, msg: Twist):
        with self._lock:
            self.cmd = np.array([msg.linear.x, msg.linear.y, msg.angular.z], np.float32)
            self.last_cmd_t = self._now()

    def _on_mode(self, msg: String):
        new = msg.data.strip().lower()
        if new not in ("sport", "rl"):
            return
        with self._lock:
            self.requested_mode = new
        self._wake.set()

    def _on_estop(self, msg: Bool):
        """Emergency stop from the web STOP button (only meaningful when RL owns the robot)."""
        if msg.data:
            with self._lock:
                # Only damp if the policy is (or was) driving; if the sport service owns
                # the robot (SPORT phase) the sport-side Damp handles it, not us.
                if self.phase in (ENGAGE, RL_RUN, ESTOP):
                    self.phase = ESTOP
                    self.get_logger().warn("ESTOP: damping motors (soft collapse)")
        else:
            with self._lock:
                if self.phase == ESTOP:
                    self._recover_pending = True
            self._wake.set()

    # ------------------------------------------------------------------ #
    # Transition worker (blocking SDK calls live here, never in the 50 Hz loop)
    # ------------------------------------------------------------------ #
    def _transition_worker(self):
        while not self._stop:
            self._wake.wait(timeout=0.5)
            self._wake.clear()
            if self._stop:
                break
            with self._lock:
                req, phase, recover = self.requested_mode, self.phase, self._recover_pending
            if recover and phase == ESTOP:
                self._recover()
            elif req == "rl" and phase == SPORT:
                self._engage()
            elif req == "sport" and phase == RL_RUN:
                self._disengage()

    def _engage(self):
        self.get_logger().info("ENGAGE: handing control from sport service to RL policy")
        if self.handover and not self.dry_run:
            with self._sport_lock:
                self.sc.StopMove()
                self.sc.StandDown()          # crouch via sport svc -> stable low pose
            time.sleep(1.5)
            self._release_sport()            # kill sport svc; robot now low + limp briefly
        with self._lock:
            self.start_pos_sdk = self._read_q_sdk()
            self.last_action = np.zeros(12, np.float32)
            self.ramp_t = 0.0
            self.phase = ENGAGE              # control loop now ramps -> RL_RUN

    def _disengage(self):
        self.get_logger().info("DISENGAGE: handing control from RL policy back to sport service")
        with self._lock:
            self.phase = DISENGAGE           # control loop PD-holds default
        time.sleep(DISENGAGE_HOLD)
        if self.handover and not self.dry_run:
            self._select_sport()             # restart sport svc
            with self._sport_lock:
                self.sc.BalanceStand()       # sport svc takes the weight
            time.sleep(0.5)
        with self._lock:
            self.phase = SPORT               # stop publishing lowcmd

    def _recover(self):
        """Stand back up under the RL policy after an ESTOP (RESUME button).

        The sport service was already released on first engage, so this only ramps
        the (collapsed) measured pose back to the default pose, then resumes the policy.
        """
        self.get_logger().info("RECOVER: standing back up under RL policy")
        with self._lock:
            self._recover_pending = False
            self.start_pos_sdk = self._read_q_sdk()
            self.last_action = np.zeros(12, np.float32)
            self.ramp_t = 0.0
            self.phase = ENGAGE          # control loop ramps -> RL_RUN

    def _release_sport(self):
        status, result = self.msc.CheckMode()
        for _ in range(10):
            if not result or not result.get("name"):
                break
            self.msc.ReleaseMode()
            time.sleep(0.5)
            status, result = self.msc.CheckMode()
        self.get_logger().info(f"sport service released (mode now: {result})")

    def _select_sport(self):
        self.msc.SelectMode("normal")
        time.sleep(1.0)
        self.get_logger().info("sport service selected (normal)")

    # ------------------------------------------------------------------ #
    # 50 Hz control loop
    # ------------------------------------------------------------------ #
    def _control_step(self):
        if self.low_state is None:
            return
        self._tick += 1
        if self._tick % 10 == 0:             # ~5 Hz liveness heartbeat
            hb = Bool()
            hb.data = True
            self._pub_hb.publish(hb)
        with self._lock:
            phase = self.phase
        try:
            if phase == SPORT:
                return                       # sport svc owns the robot
            elif phase == ESTOP:
                self._damp()                 # emergency stop: motors passive, soft collapse
            elif phase == ENGAGE:
                self._ramp_step()
            elif phase == RL_RUN:
                self._policy_step()
            elif phase == DISENGAGE:
                self._publish(self.q_default_sdk, KP, KD)
        except Exception as e:               # noqa: BLE001 - never let the loop die mid-flight
            self.get_logger().error(f"control step fault: {e}; damping")
            self._damp()

    def _ramp_step(self):
        self.ramp_t += CONTROL_DT
        alpha = min(self.ramp_t / RAMP_TIME, 1.0)
        q = (1 - alpha) * self.start_pos_sdk + alpha * self.q_default_sdk
        self._publish(q, RAMP_KP, RAMP_KD)
        if alpha >= 1.0:
            with self._lock:
                self.phase = RL_RUN
            self.get_logger().info("RL_RUN: policy active")

    def _policy_step(self):
        obs = self._build_obs()
        if not np.all(np.isfinite(obs)):
            self.get_logger().error("non-finite obs; damping")
            self._damp()
            return
        action = self.session.run(None, {self.in_name: obs[None]})[0][0]
        if not np.all(np.isfinite(action)):
            self.get_logger().error("non-finite action; damping")
            self._damp()
            return
        with self._lock:
            self.last_action = action.astype(np.float32)
        q_target_isaac = self.q_default_isaac + ACTION_SCALE * action
        q_target_sdk = q_target_isaac[self.sdk_from_isaac]
        self._publish(q_target_sdk, KP, KD)

    def _build_obs(self):
        ls = self.low_state
        q_sdk = np.array([ls.motor_state[i].q for i in range(12)], np.float32)
        dq_sdk = np.array([ls.motor_state[i].dq for i in range(12)], np.float32)
        q_isaac = q_sdk[self.isaac_from_sdk]
        dq_isaac = dq_sdk[self.isaac_from_sdk]
        gyro = np.array(ls.imu_state.gyroscope, np.float32)
        proj_g = projected_gravity(ls.imu_state.quaternion)
        with self._lock:
            timed_out = (self._now() - self.last_cmd_t) > CMD_TIMEOUT
            cmd = np.zeros(3, np.float32) if timed_out else np.clip(self.cmd, -CMD_CLIP, CMD_CLIP)
            last_a = self.last_action.copy()
        lin_vel = np.zeros(3, np.float32)    # lin_vel_mode == "zero"
        parts = []
        if self.obs_dim == 48:               # 48-dim policy includes base_lin_vel
            parts.append(lin_vel)
        parts += [gyro, proj_g, cmd, q_isaac - self.q_default_isaac, dq_isaac, last_a]
        return np.concatenate(parts).astype(np.float32)

    # ------------------------------------------------------------------ #
    # Low-level command helpers
    # ------------------------------------------------------------------ #
    def _init_low_cmd(self):
        self.low_cmd.head[0] = 0xFE
        self.low_cmd.head[1] = 0xEF
        self.low_cmd.level_flag = 0xFF
        self.low_cmd.gpio = 0
        for i in range(20):
            m = self.low_cmd.motor_cmd[i]
            m.mode = 0x01
            m.q = PosStopF
            m.kp = 0.0
            m.dq = VelStopF
            m.kd = 0.0
            m.tau = 0.0

    def _publish(self, q_target_sdk, kp, kd):
        if self.dry_run:
            kp, kd = 0.0, 0.0                # compute everything, apply no torque
        for i in range(12):
            m = self.low_cmd.motor_cmd[i]
            m.mode = 0x01
            m.q = float(q_target_sdk[i])
            m.dq = 0.0
            m.kp = float(kp)
            m.kd = float(kd)
            m.tau = 0.0
        self.low_cmd.crc = self.crc.Crc(self.low_cmd)
        self.lowcmd_pub.Write(self.low_cmd)

    def _damp(self):
        for i in range(12):
            m = self.low_cmd.motor_cmd[i]
            m.mode = 0x01
            m.q = 0.0
            m.dq = 0.0
            m.kp = 0.0
            m.kd = DAMP_KD
            m.tau = 0.0
        self.low_cmd.crc = self.crc.Crc(self.low_cmd)
        self.lowcmd_pub.Write(self.low_cmd)

    def _read_q_sdk(self):
        ls = self.low_state
        return np.array([ls.motor_state[i].q for i in range(12)], np.float32)

    def _now(self):
        return time.monotonic() - self._t0

    def shutdown(self):
        """Safe teardown: never leave the robot stranded.

        Idempotent (signal handler + finally may both call it). If the policy was
        driving (sport service released), recover the sport service so the robot
        stands rather than going limp; then un-gate web_bridge so sport teleop
        works again even if FastAPI never learned the node died.
        """
        if getattr(self, "_shutdown_done", False):
            return
        self._shutdown_done = True
        self._stop = True
        self._wake.set()
        with self._lock:
            phase = self.phase
        engaged = phase in (ENGAGE, RL_RUN, DISENGAGE, ESTOP)

        # 1. stop the 50 Hz loop so it no longer publishes lowcmd
        try:
            self.ctrl_thread.Wait(timeout=1.5)
        except Exception:  # noqa: BLE001
            pass

        # 2. if we released the sport service, bring it back (else the robot is
        #    left with no controller = limp). Falls back to a damp on failure.
        if engaged and self.handover and not self.dry_run:
            try:
                self.get_logger().warn("shutdown: recovering Unitree sport service")
                self._select_sport()
                with self._sport_lock:
                    self.sc.BalanceStand()
            except Exception as e:  # noqa: BLE001
                self.get_logger().error(f"shutdown: sport recovery failed ({e}); damping")
                self._damp()

        # 3. un-gate web_bridge so sport teleop is usable again
        try:
            m = String(); m.data = "sport"
            self._pub_mode_out.publish(m)
            b = Bool(); b.data = True
            self._pub_enabled_out.publish(b)
            self.get_logger().info("shutdown: published sport mode + re-enabled web_bridge")
        except Exception:  # noqa: BLE001
            pass


def main():
    ap = argparse.ArgumentParser(description="Go2 low-level RL policy controller")
    ap.add_argument("--net", default="", help="DDS network interface to the robot (e.g. eth0)")
    ap.add_argument("--policy", default=str(HERE / "policy.onnx"))
    ap.add_argument("--joint-names", default=str(HERE / "joint_names.json"))
    ap.add_argument("--lin-vel-mode", default="zero", choices=["zero"],
                    help="source for base_lin_vel obs (48-dim policy). 'zero' = un-retrained test.")
    ap.add_argument("--dry-run", action="store_true",
                    help="compute obs/action and publish lowcmd with kp=kd=0 (no torque) -- "
                         "validate the pipeline on a powered robot with limp motors")
    ap.add_argument("--no-handover", action="store_true",
                    help="skip sport-service release/recover (gantry-only; YOU ensure no sport svc)")
    ap.add_argument("--no-prompt", action="store_true",
                    help="skip the interactive confirmation (for background launch). The node still "
                         "stays idle until 'rl' is selected; the UI confirm dialog is the human gate.")
    args = ap.parse_args()

    print("WARNING: low-level control. Ensure the robot is on a gantry / clear area, E-stop ready.")
    print("The node starts IDLE (sport mode) and only drives motors once 'rl' is selected.")
    if not args.no_prompt:
        input("Press Enter to start...")

    if args.net:
        ChannelFactoryInitialize(0, args.net)
    else:
        ChannelFactoryInitialize(0)

    rclpy.init()
    node = Go2RLPolicyNode(args)

    # Catch Ctrl-C (SIGINT) AND `kill`/launcher cleanup (SIGTERM) so the node always
    # runs its safe shutdown (recover sport service + un-gate web_bridge) instead of
    # dying and stranding the robot. (A hard SIGKILL/-9 can't be trapped -> power-cycle.)
    def _graceful(signum, _frame):
        try:
            node.get_logger().warn(f"signal {signum}: safe shutdown")
        except Exception:  # noqa: BLE001
            pass
        node.shutdown()
        try:
            rclpy.try_shutdown()
        except Exception:  # noqa: BLE001
            pass

    signal.signal(signal.SIGINT, _graceful)
    signal.signal(signal.SIGTERM, _graceful)

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.shutdown()           # idempotent; no-op if the signal handler ran
        try:
            node.destroy_node()
        except Exception:  # noqa: BLE001
            pass
        try:
            rclpy.try_shutdown()
        except Exception:  # noqa: BLE001
            pass


if __name__ == "__main__":
    main()
