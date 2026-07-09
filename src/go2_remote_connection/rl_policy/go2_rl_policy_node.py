#!/usr/bin/env python3
"""Low-level RL locomotion controller for the Unitree Go2 (SDK-only process).

Runs the flat-terrain policy trained in Isaac Lab (RSL-RL PPO,
``Isaac-Velocity-Flat-Unitree-Go2-v0``) directly on the robot's motors, driven
by the Go2RemoteConnection web joystick. Sits alongside the existing
``web_bridge`` (Unitree SportClient) path; the operator switches between them
from the website.

PROCESS SPLIT (why this file has no rclpy):
    The Unitree SDK and ROS 2's rmw_cyclonedds both use CycloneDDS, and in one
    process they share a single ``libddsc`` (same SONAME -> one instance). Both
    insist on *creating* DDS domain 0 (the robot's low-level interface is fixed
    there, and the SDK's ChannelFactory has no "join" path), so whichever calls
    ``dds_create_domain(0)`` second fails ("Precondition Not Met"). They cannot
    coexist in one process on this platform. So the RL controller is split in two:

      * THIS process (``go2_rl_policy_node.py``) -- pure ``unitree_sdk2py``, no
        rclpy: the 50 Hz control loop, policy inference, sport-service handover,
        and all safety fallbacks. Talks to the bridge over localhost UDP.
      * ``go2_rl_bridge_node.py`` -- pure rclpy: subscribes the web topics and
        forwards them here over UDP; relays our heartbeat / un-gate signals back
        onto ROS. (Pure ROS, exactly like ``web_bridge`` -- no SDK, no conflict.)

Web control (via the bridge, localhost UDP JSON datagrams):
    IN   teleop   {vx,vy,wz}      joystick (from /web_teleop)
    IN   mode     {"sport"|"rl"}  control-mode request (from /web_control_mode)
    IN   policy   {id: str}       hot-swap the active policy, idle only (from /web_rl_policy)
    IN   estop    {on: bool}      emergency stop / resume (from /web_estop)
    IN   ping                     bridge liveness (10 Hz)
    OUT  heartbeat                5 Hz liveness (bridge -> /web_rl_heartbeat)
    OUT  policy_out {id: str}     the policy actually loaded now (-> /web_rl_active_policy)
    OUT  mode_out {"sport"}       on shutdown, un-gate web_bridge (-> /web_control_mode)
    OUT  enabled  {val: bool}     on shutdown, re-enable web_bridge (-> /web_teleop_enabled)

Robot (unitree_sdk2py / DDS):
    SUB  rt/lowstate   LowState_   IMU (quat, gyro) + per-joint q, dq
    PUB  rt/lowcmd     LowCmd_     joint position targets @ 50 Hz with PD gains
    MotionSwitcherClient / SportClient   release / recover the high-level sport service

Control mode handover:
    sport -> rl : StandDown (crouch) -> ReleaseMode (kill sport svc) -> ramp to
                  default pose under PD -> run policy @ 50 Hz.
    rl -> sport : ramp command to 0, PD-hold default -> SelectMode + BalanceStand
                  (sport svc takes over) -> stop publishing lowcmd.
    flip (rl)   : if the robot ends up inverted while the policy runs, stop driving
                  lowcmd -> SelectMode + Damp + RecoveryStand + BalanceStand (the
                  sport svc rights it) -> idle in sport mode (operator re-engages RL).
                  Detected from body-frame gravity + a low-angular-velocity gate.

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
import logging
import os
import pathlib
import signal
import socket
import threading
import time

import numpy as np
import onnxruntime as ort

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
CMD_TIMEOUT = 0.5      # s, deadman on the joystick command
CMD_CLIP = 1.0         # training command range +/-1
DAMP_KD = 3.0          # damping fallback (soft collapse) on fault
BRIDGE_TIMEOUT = 1.0   # s, deadman on the ROS bridge link (heartbeat/ping/teleop)

# Flip auto-recovery (RL_RUN only). Body-frame gravity z is -1 upright and flips to
# +1 inverted (see projected_gravity); fire only once the robot has *settled* upside
# down (low |gyro|), after a debounce, then hand back to the sport service for a
# RecoveryStand. A cooldown stops it re-triggering on the same tumble.
FLIP_PROJ_G_Z = 0.7      # proj_gravity[2] threshold for "upside down" (>0 = inverted)
FLIP_GYRO_MAX = 2.5      # rad/s, max |gyro| to count as settled (reject mid-tumble)
FLIP_DEBOUNCE = 0.75     # s, inverted+settled must hold continuously before firing
FLIP_COOLDOWN = 5.0      # s, suppress re-trigger after a recovery completes
FLIP_RECOVERY_WAIT = 5.0 # s, let the sport-service RecoveryStand maneuver finish

# localhost UDP link to the rclpy bridge (go2_rl_bridge_node.py).
DEF_UDP_HOST = "127.0.0.1"
DEF_CTRL_PORT = 47811  # this process listens here (web -> control)
DEF_BRIDGE_PORT = 47812  # the bridge listens here (control -> web)

# Control-phase state machine.
#   SPORT     idle, the Unitree sport service owns the robot (node publishes nothing)
#   ENGAGE    ramping measured pose -> default pose under PD, then -> RL_RUN
#   RL_RUN    policy active @ 50 Hz
#   DISENGAGE PD-hold default while handing back to the sport service
#   ESTOP     emergency stop: continuously damp (soft collapse), latched until RESUME
#   RECOVER   auto flip-recovery: hand back to the sport svc + RecoveryStand -> SPORT
SPORT, ENGAGE, RL_RUN, DISENGAGE, ESTOP, RECOVER = \
    "SPORT", "ENGAGE", "RL_RUN", "DISENGAGE", "ESTOP", "RECOVER"


class _StdLogger:
    """Minimal logger shim so the control logic keeps its ``get_logger().info(...)``
    call sites unchanged now that this class is no longer an rclpy ``Node``."""

    def __init__(self, name):
        self._l = logging.getLogger(name)

    def info(self, m):
        self._l.info(m)

    def warn(self, m):
        self._l.warning(m)

    def warning(self, m):
        self._l.warning(m)

    def error(self, m):
        self._l.error(m)


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


def load_registry(path, logger=None):
    """Read the policy registry (policies.json) selectable from the web dropdown.

    Returns ``(policies_by_id, default_id)``. On any failure returns ``({}, None)``
    so the node falls back to the bare ``--policy`` file (legacy behaviour).
    """
    def _warn(m):
        (logger.warn if logger else print)(m)
    try:
        data = json.loads(pathlib.Path(path).read_text())
        by_id = {}
        for entry in data.get("policies", []):
            pid = str(entry.get("id", "")).strip()
            if pid:
                by_id[pid] = entry
        default_id = data.get("default") or (next(iter(by_id), None))
        return by_id, default_id
    except FileNotFoundError:
        _warn(f"policy registry {path} not found; using --policy file only")
        return {}, None
    except Exception as e:  # noqa: BLE001
        _warn(f"could not parse policy registry {path} ({e}); using --policy file only")
        return {}, None


class Go2RLPolicyController:
    def __init__(self, args):
        self._logger = _StdLogger("go2_rl_policy")
        self.dry_run = args.dry_run
        self.handover = not args.no_handover
        self.flip_recovery = True    # auto flip-recovery is always armed while the policy runs
        self.lin_vel_mode = args.lin_vel_mode

        # ---- joint remap -------------------------------------------------
        isaac_joints, used_default = load_isaac_joints(args.joint_names)
        self.isaac_joints = isaac_joints
        self.isaac_from_sdk = [SDK_JOINTS.index(n) for n in isaac_joints]   # lowstate -> obs order
        self.sdk_from_isaac = [isaac_joints.index(n) for n in SDK_JOINTS]   # action  -> lowcmd order
        self.q_default_isaac = np.array([Q_DEFAULT_BY_NAME[n] for n in isaac_joints], np.float32)
        self.q_default_sdk = np.array([Q_DEFAULT_BY_NAME[n] for n in SDK_JOINTS], np.float32)

        # ---- policy registry + initial policy ----------------------------
        # The onnx is hot-swappable at runtime (only while idle) so the operator can
        # pick a policy from the web dropdown without restarting the node. The
        # registry (policies.json) is the source of truth for that dropdown.
        self.registry_path = pathlib.Path(args.policies)
        self.policies, self.default_policy_id = load_registry(self.registry_path, self.get_logger())
        self._policy_lock = threading.Lock()      # guards the session swap on hot-reload
        self.session = None
        self.in_name = None
        self.obs_dim = None
        self.active_policy_id = None
        start_entry = self.policies.get(self.default_policy_id) if self.default_policy_id else None
        if start_entry is None or not self._load_policy_entry(start_entry):
            # No usable registry default -> fall back to the bare --policy file.
            self._load_policy_file(args.policy)
            self.active_policy_id = self.default_policy_id
        self.get_logger().info(
            f"active_policy={self.active_policy_id} obs_dim={self.obs_dim} "
            f"joint_order={'DEFAULT(verify!)' if used_default else 'joint_names.json'} "
            f"dry_run={self.dry_run} handover={self.handover} lin_vel={self.lin_vel_mode} "
            f"flip_recovery={self.flip_recovery}")

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
        self._flip_pending = False             # set by the loop, consumed by the worker
        self._flip_since = None                # monotonic t when inversion first held
        self._flip_cooldown_until = 0.0        # suppress re-trigger after a recovery
        self._wake = threading.Event()
        self._stop = False
        self._t0 = time.monotonic()
        self.last_bridge_t = self._now()   # bridge-link deadman (updated on any datagram)

        # ---- localhost UDP link to the rclpy bridge ----------------------
        self._bridge_addr = (args.udp_host, args.bridge_port)
        self._tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._rx.bind((args.udp_host, args.ctrl_port))
        self._rx.settimeout(0.5)
        self.get_logger().info(
            f"bridge link: listening on {args.udp_host}:{args.ctrl_port}, "
            f"sending to {args.udp_host}:{args.bridge_port}")

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

        self._tick = 0

        # ---- threads -----------------------------------------------------
        self._rxthread = threading.Thread(target=self._udp_rx_loop, daemon=True)
        self._rxthread.start()
        self._tworker = threading.Thread(target=self._transition_worker, daemon=True)
        self._tworker.start()
        self.ctrl_thread = RecurrentThread(interval=CONTROL_DT, target=self._control_step,
                                           name="rl_control")
        self.ctrl_thread.Start()
        self._send({"t": "policy_out", "id": self.active_policy_id})   # tell the web which policy is loaded
        self.get_logger().info("go2_rl_policy_node ready (mode=sport, idle).")

    def get_logger(self):
        return self._logger

    # ------------------------------------------------------------------ #
    # Bridge link (localhost UDP JSON datagrams)
    # ------------------------------------------------------------------ #
    def _send(self, obj):
        """Best-effort send of one JSON datagram to the bridge (never raises)."""
        try:
            self._tx.sendto(json.dumps(obj).encode("utf-8"), self._bridge_addr)
        except OSError:
            pass

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
            self.last_bridge_t = self._now()   # any datagram proves the bridge is alive
            try:
                msg = json.loads(data.decode("utf-8"))
                t = msg.get("t")
            except (ValueError, AttributeError):
                continue
            if t == "teleop":
                self._set_teleop(msg.get("vx", 0.0), msg.get("vy", 0.0), msg.get("wz", 0.0))
            elif t == "mode":
                self._set_mode(msg.get("mode", ""))
            elif t == "policy":
                self._set_policy(msg.get("id", ""))
            elif t == "estop":
                self._set_estop(bool(msg.get("on", False)))
            elif t == "ping":
                pass   # liveness only; last_bridge_t already updated above

    # ------------------------------------------------------------------ #
    # Web-command handlers (driven by the bridge over UDP)
    # ------------------------------------------------------------------ #
    def _on_lowstate(self, msg: LowState_):
        self.low_state = msg

    def _set_teleop(self, vx, vy, wz):
        with self._lock:
            self.cmd = np.array([vx, vy, wz], np.float32)
            self.last_cmd_t = self._now()

    def _set_mode(self, mode):
        new = str(mode).strip().lower()
        if new not in ("sport", "rl"):
            return
        with self._lock:
            self.requested_mode = new
        self._wake.set()

    # ------------------------------------------------------------------ #
    # Policy selection (hot-swap the onnx, idle only)
    # ------------------------------------------------------------------ #
    def _resolve_policy_path(self, entry):
        """Resolve a registry entry's onnx path (relative paths are anchored at the
        registry file's directory, i.e. rl_policy/)."""
        p = pathlib.Path(entry.get("path", ""))
        if not p.is_absolute():
            p = self.registry_path.resolve().parent / p
        return p

    def _load_policy_file(self, path):
        """Build a fresh ORT session and swap it in. Assigns only after a successful
        build so a bad file never leaves us with a half-loaded session. Raises on error."""
        sess = ort.InferenceSession(str(path), providers=["CPUExecutionProvider"])
        with self._policy_lock:
            self.session = sess
            self.in_name = sess.get_inputs()[0].name
            self.obs_dim = int(sess.get_inputs()[0].shape[1])

    def _load_policy_entry(self, entry):
        """Load the onnx described by a registry entry. Returns True on success and
        leaves the current policy untouched on any failure (unknown/unrunnable/missing)."""
        pid = entry.get("id")
        if not entry.get("runnable", True) or entry.get("uses_heightmap", False):
            self.get_logger().warn(
                f"policy '{pid}' needs a height scan this node cannot provide; not loading")
            return False
        path = self._resolve_policy_path(entry)
        if not path.exists():
            self.get_logger().warn(f"policy '{pid}' onnx not found at {path}; not loading")
            return False
        try:
            self._load_policy_file(path)
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"policy '{pid}' failed to load ({e}); keeping previous policy")
            return False
        exp = entry.get("obs_dim")
        if exp is not None and int(exp) != self.obs_dim:
            self.get_logger().warn(
                f"policy '{pid}' onnx obs_dim {self.obs_dim} != registry {exp}")
        self.active_policy_id = pid
        self.get_logger().info(f"loaded policy '{pid}' from {path} (obs_dim={self.obs_dim})")
        return True

    def _set_policy(self, pid):
        """Web dropdown asked to switch the active policy. Allowed only while idle
        (SPORT): swapping the onnx mid-run would change the control law under load.
        Always echoes the *actual* active policy back to the web so the UI stays honest."""
        pid = str(pid).strip()
        entry = self.policies.get(pid)
        if entry is None:
            self.get_logger().warn(f"unknown policy id '{pid}'; ignoring")
        else:
            with self._lock:
                phase = self.phase
            if phase != SPORT:
                self.get_logger().warn(
                    f"policy switch to '{pid}' ignored: only allowed while idle (phase={phase})")
            elif pid != self.active_policy_id:
                self._load_policy_entry(entry)
        self._send({"t": "policy_out", "id": self.active_policy_id})

    def _set_estop(self, on):
        """Emergency stop from the web STOP button (only meaningful when RL owns the robot)."""
        if on:
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
                flip = self._flip_pending
            # A flip recovery is already latched by the 50 Hz loop (phase == RECOVER);
            # run it first and to completion -- it is a single, safe, blocking sequence.
            if flip and phase == RECOVER:
                self._recover_flip()
            # A "sport" request always wins -- it is the safe direction and the
            # failsafe out of any engaged state. Handle it from every phase where
            # we still own the motors, not just RL_RUN, so toggling RL off never
            # strands the robot with the sport service released.
            elif req == "sport" and phase in (ENGAGE, RL_RUN):
                self._disengage()
            elif req == "sport" and phase == ESTOP:
                self._disengage_from_estop()
            elif recover and phase == ESTOP:
                self._recover()
            elif req == "rl" and phase == SPORT:
                self._engage()

    def _engage(self):
        # Defence in depth: the web backend already blocks engaging a policy that
        # needs perception, but never release the sport service for one we cannot
        # feed. Revert the web toggle to sport instead of stranding the robot.
        entry = self.policies.get(self.active_policy_id)
        if entry is not None and (not entry.get("runnable", True) or entry.get("uses_heightmap", False)):
            self.get_logger().error(
                f"refusing to engage '{self.active_policy_id}': needs a height scan this "
                "node cannot provide; staying in sport mode")
            with self._lock:
                self.requested_mode = "sport"
            self._send({"t": "mode_out", "mode": "sport"})   # bounce the web toggle back to sport
            self._send({"t": "enabled", "val": True})
            return
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

    def _disengage_from_estop(self):
        """Hand control back to the sport service from an ESTOP (soft collapse).

        Reached when the operator hits STOP while RL owns the robot and then
        toggles RL off. Unlike _disengage(), we do NOT PD-hold the default pose
        first: the robot is collapsed on the ground, so snapping it to the default
        stance would be a violent lurch. Instead we stop driving lowcmd at once and
        recover the sport service so it listens to SportClient commands again. The
        robot stays down (STOP is still latched) until the operator RESUMEs, which
        stands it back up via the sport service's RecoveryStand -- exactly how STOP
        already behaves in plain sport mode.
        """
        self.get_logger().info("DISENGAGE (from ESTOP): handing control back to sport service")
        with self._lock:
            self._recover_pending = False    # cancel any pending RL stand-up
            self.phase = SPORT               # 50 Hz loop stops publishing lowcmd now
        if self.handover and not self.dry_run:
            self._select_sport()             # restart sport svc -> sport teleop usable again

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

    def _recover_flip(self):
        """Auto-recover from an inverted robot while the policy was running.

        Detected by the 50 Hz loop (see _flip_detected), which has already set
        phase == RECOVER so the loop has stopped driving lowcmd. Here we run the
        one-shot handoff: recover the sport service (released on engage), then
        Damp -> RecoveryStand -> BalanceStand so the sport controller rights and
        stands the robot. We deliberately land in *sport* mode (not RL): a flip is
        a fault, so the operator re-engages RL when ready rather than us diving
        straight back into the policy that just tipped over.
        """
        self.get_logger().warn("FLIP RECOVERY: handing back to sport service for RecoveryStand")
        with self._lock:
            self._flip_pending = False
            self.requested_mode = "sport"    # stay idle after recovery; no auto re-engage
            self.phase = RECOVER             # (already set by the loop) loop drives nothing
        time.sleep(0.1)                      # let the 50 Hz loop observe RECOVER and go quiet
        if self.handover and not self.dry_run:
            self._select_sport()             # restart the sport svc (released on engage)
            with self._sport_lock:
                self.sc.Damp()               # relax from the collapsed/inverted pose
            time.sleep(0.5)
            with self._sport_lock:
                self.sc.RecoveryStand()      # flip upright and stand
            time.sleep(FLIP_RECOVERY_WAIT)   # let the maneuver finish before settling
            with self._sport_lock:
                self.sc.BalanceStand()       # settle into balance stand
            time.sleep(0.5)
        # Reflect sport mode on the web UI and re-enable web_bridge teleop.
        self._send({"t": "mode_out", "mode": "sport"})
        self._send({"t": "enabled", "val": True})
        with self._lock:
            self.phase = SPORT               # sport svc owns the robot; loop stays quiet
            self._flip_since = None
            self._flip_cooldown_until = self._now() + FLIP_COOLDOWN
        self.get_logger().info(
            "FLIP RECOVERY complete: standing under sport service (mode=sport, re-select RL to resume)")

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
        if self._tick % 10 == 0:             # ~5 Hz liveness heartbeat to the bridge
            self._send({"t": "heartbeat"})
        with self._lock:
            phase = self.phase
        # Bridge-link deadman: if the rclpy bridge (our only path to the web STOP
        # button and mode switch) goes silent while we own the motors, fail safe
        # to ESTOP -- a soft collapse -- rather than keep driving blind. Mirrors
        # the web-side watchdog that reverts to sport when our heartbeat stops.
        if phase in (ENGAGE, RL_RUN) and (self._now() - self.last_bridge_t) > BRIDGE_TIMEOUT:
            self.get_logger().error("bridge link lost; ESTOP (soft collapse)")
            with self._lock:
                self.phase = ESTOP
            phase = ESTOP
        try:
            if phase == SPORT:
                return                       # sport svc owns the robot
            elif phase == ESTOP:
                self._damp()                 # emergency stop: motors passive, soft collapse
            elif phase == ENGAGE:
                self._ramp_step()
            elif phase == RL_RUN:
                if self.flip_recovery and self._flip_detected():
                    self.get_logger().warn(
                        "flip detected (robot inverted and settled); starting auto-recovery")
                    with self._lock:
                        self.phase = RECOVER      # stop driving lowcmd this instant
                        self._flip_pending = True
                    self._wake.set()              # transition worker runs the sequence
                    return
                self._policy_step()
            elif phase == DISENGAGE:
                self._publish(self.q_default_sdk, KP, KD)
            elif phase == RECOVER:
                return                            # sport svc (being) restored; do not drive
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

    def _flip_detected(self):
        """True once the robot has been inverted *and settled* for FLIP_DEBOUNCE.

        proj_gravity[2] is -1 upright and swings positive when the base rolls/pitches
        past ~90 deg; > FLIP_PROJ_G_Z means clearly upside down. The |gyro| gate rejects
        transient inversions mid-tumble (we want to recover only once it has come to
        rest on its back). A cooldown after each recovery stops repeat firing on the
        same event. Called only from RL_RUN, where low_state is guaranteed non-None.
        """
        if self._now() < self._flip_cooldown_until:
            self._flip_since = None
            return False
        ls = self.low_state
        proj_g = projected_gravity(ls.imu_state.quaternion)
        gyro = np.asarray(ls.imu_state.gyroscope, np.float32)
        inverted = proj_g[2] > FLIP_PROJ_G_Z and float(np.linalg.norm(gyro)) < FLIP_GYRO_MAX
        if not inverted:
            self._flip_since = None              # reset the debounce window
            return False
        now = self._now()
        if self._flip_since is None:
            self._flip_since = now               # start the debounce window
            return False
        return (now - self._flip_since) >= FLIP_DEBOUNCE

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

        # 3. un-gate web_bridge so sport teleop is usable again (via the bridge)
        try:
            self._send({"t": "mode_out", "mode": "sport"})
            self._send({"t": "enabled", "val": True})
            self.get_logger().info("shutdown: published sport mode + re-enabled web_bridge")
        except Exception:  # noqa: BLE001
            pass
        try:
            self._rx.close()
        except Exception:  # noqa: BLE001
            pass


def main():
    ap = argparse.ArgumentParser(description="Go2 low-level RL policy controller (SDK-only)")
    ap.add_argument("--net", default="", help="DDS network interface to the robot (e.g. eth0)")
    ap.add_argument("--policy", default=str(HERE / "policy.onnx"),
                    help="fallback onnx if the registry has no usable default")
    ap.add_argument("--policies", default=str(HERE / "policies.json"),
                    help="policy registry (JSON) listing the web-selectable policies")
    ap.add_argument("--joint-names", default=str(HERE / "joint_names.json"))
    ap.add_argument("--lin-vel-mode", default="zero", choices=["zero"],
                    help="source for base_lin_vel obs (48-dim policy). 'zero' = un-retrained test.")
    ap.add_argument("--dry-run", action="store_true",
                    help="compute obs/action and publish lowcmd with kp=kd=0 (no torque) -- "
                         "validate the pipeline on a powered robot with limp motors")
    ap.add_argument("--no-handover", action="store_true",
                    help="skip sport-service release/recover (gantry-only; YOU ensure no sport svc)")
    ap.add_argument("--no-flip-recovery", action="store_true",
                    help="disable automatic flip detection + RecoveryStand while the policy runs "
                         "(detection is RL_RUN-only; leave ON for free-standing ground tests)")
    ap.add_argument("--no-prompt", action="store_true",
                    help="skip the interactive confirmation (for background launch). The node still "
                         "stays idle until 'rl' is selected; the UI confirm dialog is the human gate.")
    ap.add_argument("--udp-host", default=os.environ.get("GO2_RL_UDP_HOST", DEF_UDP_HOST),
                    help="localhost address for the rclpy bridge link")
    ap.add_argument("--ctrl-port", type=int,
                    default=int(os.environ.get("GO2_RL_CTRL_PORT", DEF_CTRL_PORT)),
                    help="UDP port this process listens on (web -> control)")
    ap.add_argument("--bridge-port", type=int,
                    default=int(os.environ.get("GO2_RL_BRIDGE_PORT", DEF_BRIDGE_PORT)),
                    help="UDP port the bridge listens on (control -> web)")
    args = ap.parse_args()

    logging.basicConfig(level=logging.INFO, format="[%(levelname)s] [%(name)s] %(message)s")

    print("WARNING: low-level control. Ensure the robot is on a gantry / clear area, E-stop ready.")
    print("The node starts IDLE (sport mode) and only drives motors once 'rl' is selected.")
    if not args.no_prompt:
        input("Press Enter to start...")

    # Pure-SDK process: no rclpy here, so the SDK is the only CycloneDDS user and
    # can safely own domain 0. (The ROS side lives in go2_rl_bridge_node.py.)
    if args.net:
        ChannelFactoryInitialize(0, args.net)
    else:
        ChannelFactoryInitialize(0)

    controller = Go2RLPolicyController(args)
    stop_event = threading.Event()

    # Catch Ctrl-C (SIGINT) AND `kill`/launcher cleanup (SIGTERM) so the node always
    # runs its safe shutdown (recover sport service + un-gate web_bridge) instead of
    # dying and stranding the robot. (A hard SIGKILL/-9 can't be trapped -> power-cycle.)
    def _graceful(signum, _frame):
        controller.get_logger().warn(f"signal {signum}: safe shutdown")
        stop_event.set()

    signal.signal(signal.SIGINT, _graceful)
    signal.signal(signal.SIGTERM, _graceful)

    try:
        stop_event.wait()
    finally:
        controller.shutdown()           # idempotent; safe if called more than once


if __name__ == "__main__":
    main()
