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
    Every value comes from the active policy's deploy.yaml (see deploy_contract.py):
    control rate, q_target = q_default + action_scale*action, kp/kd, the observation
    term list *and each term's scale and clip*, and the command envelope.

    Observation scales are NOT optional. Stock Isaac Lab's Go2 velocity task sets no
    scales, so the first 48-dim policies ran correctly on raw sensor values and this
    file used to say "no normalization". The unitree_rl_lab-derived tasks
    (go2-velocity, go2-tap, flat-dr) carry scale=0.2 on base_ang_vel and scale=0.05 on
    joint_vel_rel; feeding those raw puts the gyro 5x and the joint velocity 20x
    outside the training distribution, the actions saturate, and the motors fault out.
    _apply_obs_transform() is the clip->scale step unitree_rl_lab's C++ deploy does in
    ObservationTermCfg::add(); do not remove it.

base_lin_vel (obs 0:3) is NOT observable in low-level mode. A policy whose contract
still declares it is fed zeros (``--lin-vel-mode zero``) and the node says so loudly at
load: those three numbers are the velocity-tracking inputs, so the policy is running
outside its training distribution on exactly the term its dominant reward optimised.
The fix is on the training side -- drop base_lin_vel from the policy observation group
and keep it for the critic only (docs/plans/sim_to_real_deployment_plan.md Phase 0).
Every policy trained that way (``go2-velocity`` and its descendants; the perception
task ``stepfield-spec-unitree``) simply has no such term and runs at full fidelity.

PERCEPTION POLICIES (height_scan). A rough policy's observation ends with a 187-cell
terrain scan produced on the ROS side by go2_perception/heightmap_node from the D435i
depth cloud. This process has no rclpy to subscribe with, so the scan crosses the same
localhost UDP link as the joystick, as a packed binary frame (see height_scan_wire.py).
The scan is treated as a safety-critical input, not a nice-to-have:

  * engaging a policy whose contract declares ``height_scan`` is REFUSED unless a scan
    has arrived within HEIGHT_SCAN_TIMEOUT, and
  * if the scan goes stale while the policy is driving, the node ESTOPs (soft collapse)
    the same way it does when the bridge link dies.

Feeding a stale or absent scan instead would hand the policy a frozen or flat view of
terrain it is actively stepping onto, which is worse than stopping.

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

import height_scan_wire

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

# Deployment contract.
#
# These were hand-transcribed from env.yaml / agent.yaml. They are now superseded at
# runtime by each policy's own deploy.yaml, which training writes from the live env
# (see deploy_contract.py). A policy without a deploy.yaml is refused rather than run
# on the values below -- every one of them fails silently when wrong: the robot walks,
# badly, and the policy gets blamed.
#
# CONTROL_DT is still used for the control loop's nominal period before any policy is
# loaded (stand-up ramp, damping); the active contract overrides it once loaded.
CONTROL_DT = 0.02      # 50 Hz policy rate (sim dt 0.005 * decimation 4)
ACTION_SCALE = 0.25    # fallback only -- see self.action_scale
KP, KD = 25.0, 0.5     # fallback only -- see self.kp / self.kd
RAMP_KP, RAMP_KD = 40.0, 4.0   # firmer during the stand-up ramp
RAMP_TIME = 2.0        # s, measured pose -> default pose
BLEND_TIME = 0.3       # s, policy authority + gains fade in at the ENGAGE -> RL_RUN handover
DISENGAGE_HOLD = 1.0   # s, PD-hold default before handing back to sport
CMD_TIMEOUT = 0.5      # s, deadman on the joystick command
CMD_CLIP = 1.0         # training command range +/-1
DAMP_KD = 3.0          # damping fallback (soft collapse) on fault
BRIDGE_TIMEOUT = 1.0   # s, deadman on the ROS bridge link (heartbeat/ping/teleop)

# Deadman on the perception height scan, for policies whose contract declares one.
# Deliberately tighter than BRIDGE_TIMEOUT: a joystick that stops updating leaves the
# robot walking on its last command, but terrain that stops updating leaves it stepping
# onto ground it can no longer see. heightmap_node runs at the D435i's frame rate
# (30 Hz nominal), so 0.4 s is ~12 missed frames -- past any plausible hiccup.
HEIGHT_SCAN_TIMEOUT = 0.4

# Action guard. A correctly-fed policy outputs |a| of roughly 1-2.5 here; the exported
# onnx is the policy *mean* and is unbounded, and deploy.yaml's action clip is +-100,
# so nothing downstream limits it. ACTION_CLIP caps a single step at a 1.25 rad offset
# from the default pose (0.25 * 5.0). Sustained clipping is not an aggressive gait, it
# is a wrong observation -- a contract mismatch measured at |a|=28 on flat-dr -- so
# after ACTION_SAT_STEPS consecutive clipped steps the node stops driving instead of
# grinding the motors into a fault.
ACTION_CLIP = 5.0
ACTION_SAT_STEPS = 25  # 0.5 s at 50 Hz

# Orientation guard. Training terminates the episode at 0.8 rad of tilt, so past that
# the policy is extrapolating; unitree_rl_lab's C++ deploy drops to Passive at 1.0 rad
# (isaaclab::mdp::bad_orientation). Unlike the flip detector below this does not wait
# for the robot to settle -- the point is to stop driving *during* the fall.
TILT_ABORT_RAD = 1.0
TILT_ABORT_DEBOUNCE = 0.2   # s, must hold continuously (rejects one noisy IMU frame)

# Flip auto-recovery (RL_RUN only). Body-frame gravity z is -1 upright and flips to
# +1 inverted (see projected_gravity); fire only once the robot has *settled* upside
# down (low |gyro|), after a debounce, then hand back to the sport service for a
# RecoveryStand. A cooldown stops it re-triggering on the same tumble.
FLIP_PROJ_G_Z = 0.7      # proj_gravity[2] threshold for "upside down" (>0 = inverted)
FLIP_GYRO_MAX = 2.5      # rad/s, max |gyro| to count as settled (reject mid-tumble)
FLIP_DEBOUNCE = 0.75     # s, inverted+settled must hold continuously before firing
FLIP_COOLDOWN = 5.0      # s, suppress re-trigger after a recovery completes
FLIP_RECOVERY_WAIT = 7.0 # s, let the sport-service self-right (roll off back) + stand finish

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


from deploy_contract import DeployContract, DeployContractError  # noqa: E402


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
        # Provisional: every one of these is replaced by the active policy's
        # deploy.yaml in _load_policy_file(). They exist so the stand-up ramp and the
        # damping path have sane values before any policy is loaded.
        self.q_default_isaac = np.array([Q_DEFAULT_BY_NAME[n] for n in isaac_joints], np.float32)
        self.q_default_sdk = np.array([Q_DEFAULT_BY_NAME[n] for n in SDK_JOINTS], np.float32)
        self.contract = None

        # ---- perception height scan (forwarded by the bridge) -------------
        # Declared HERE, above the first _load_policy_entry() call below, because
        # _load_policy_file() reads scan_geom to check a perception policy's expected
        # scan width against the grid the perception stack actually publishes.
        # Guarded by _scan_lock rather than the main _lock: it is written by the UDP rx
        # thread at camera rate and read by the control loop at 50 Hz, and it must not
        # contend with the command/phase state those two already share.
        self._scan_lock = threading.Lock()
        self.height_scan = None            # np.float32 (N,), already clipped/offset
        self.last_scan_t = 0.0             # monotonic t of the most recent valid frame
        self.scan_geom = None              # dict from height_scan_wire.decode (minus values)
        self._scan_seq_prev = None
        self._scan_drops = 0               # frames lost on the UDP link (seq gaps)
        self._scan_bad = 0                 # frames rejected as malformed
        self._scan_logged = False
        self.action_scale = ACTION_SCALE
        self.kp, self.kd = KP, KD
        self.include_base_lin_vel = True
        # Observation layout + per-term transforms. All replaced by the active policy's
        # deploy.yaml in _load_policy_file(); these provisional values describe the stock
        # 48-dim Isaac Lab layout, which is unscaled and unclipped.
        self.obs_terms = ["base_lin_vel", "base_ang_vel", "projected_gravity",
                          "velocity_commands", "joint_pos_rel", "joint_vel_rel", "last_action"]
        self.obs_scales = {}
        self.obs_clips = {}
        # Per-axis command envelope (lin_vel_x, lin_vel_y, ang_vel_z). The contract's
        # ranges replace these; CMD_CLIP is only the no-contract fallback.
        self.cmd_lo = np.full(3, -CMD_CLIP, np.float32)
        self.cmd_hi = np.full(3, CMD_CLIP, np.float32)
        # Replaced by the active policy's contract in _load_policy_file(); None means the
        # policy carries no gait_phase term and the clock is never read.
        self.gait_phase_period = None
        self.gait_phase_dt = CONTROL_DT
        self.gait_phase = 0.0

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
            # Let this raise: starting with no policy, or with one whose deployment
            # contract is missing, is not something to recover from quietly.
            try:
                self._load_policy_file(args.policy)
            except DeployContractError as e:
                self.get_logger().error(
                    f"Cannot start: {e}\n"
                    "Every runnable policy needs a deploy.yaml beside its .onnx. Re-export "
                    "it from a training run made with a current training/scripts/train.py."
                )
                raise
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
        # Posture target for sit/stand policies: 1.0 = stand, 0.0 = sit. Starts at
        # stand because _engage() ramps the robot to the policy's default pose, which
        # is a standing pose -- handing a freshly engaged policy a "sit" command would
        # make it fight the ramp it just finished. Unlike the joystick this has NO
        # deadman: a posture is a latched state, and zeroing it on silence would drop
        # a sitting robot's command back to stand and stand it up unasked.
        self.posture_cmd = 1.0
        self.last_action = np.zeros(12, np.float32)
        self.phase = SPORT
        self.requested_mode = "sport"
        self._prev_sport_mode = "normal"       # sport mode to restore on disengage (set at release)
        self.start_pos_sdk = self.q_default_sdk.copy()
        self.ramp_t = 0.0
        self.blend_t = 0.0                     # policy-authority fade-in, see _blend_alpha()
        self._recover_pending = False
        self._flip_pending = False             # set by the loop, consumed by the worker
        self._flip_since = None                # monotonic t when inversion first held
        self._flip_cooldown_until = 0.0        # suppress re-trigger after a recovery
        self._tilt_since = None                # monotonic t when excess tilt first held
        self._sat_steps = 0                    # consecutive steps with a clipped action
        # STOP-always-wins plumbing. _sport_owns tracks who currently holds the
        # motors so ESTOP/RESUME damp/stand through the right controller. _abort_recover
        # is raised by STOP to tear down an in-flight flip recovery; _estop_pending asks
        # the worker to run the sport-side ESTOP damp (kill firmware auto-recovery + Damp).
        self._sport_owns = True                # sport service owns the motors at startup (idle)
        self._abort_recover = False            # STOP pressed mid flip-recovery -> bail out
        self._estop_pending = False            # worker: run the sport-side ESTOP damp
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
        # Arm the Go2's built-in fall/flip auto-recovery so the sport service self-
        # rights the robot (rolls off its back and stands) whenever it owns the
        # motors. Our flip handler relies on this rather than a bare RecoveryStand,
        # which cannot roll the robot over from a full inversion.
        self._set_auto_recovery(True)

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
                data, _ = self._rx.recvfrom(height_scan_wire.MAX_DATAGRAM)
            except socket.timeout:
                continue
            except OSError:
                if self._stop:
                    break
                continue
            self.last_bridge_t = self._now()   # any datagram proves the bridge is alive
            # Height scans are binary, everything else is JSON. Tested by magic prefix
            # first because a float array is not valid UTF-8 and would otherwise cost a
            # failed decode per camera frame.
            if height_scan_wire.is_height_scan(data):
                self._on_height_scan_frame(data)
                continue
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
            elif t == "posture":
                self._set_posture(bool(msg.get("stand", True)))
            elif t == "estop":
                self._set_estop(bool(msg.get("on", False)))
            elif t == "ping":
                pass   # liveness only; last_bridge_t already updated above

    def _on_height_scan_frame(self, data):
        """Store one height-scan frame from the bridge.

        A malformed frame is DROPPED, not substituted: the previous scan stays, and if
        no valid frame arrives the staleness deadman stops the robot. Filling in zeros
        would read to the policy as "flat ground straight ahead", which on a stepfield
        is the most dangerous thing this node could invent.
        """
        try:
            frame = height_scan_wire.decode(data)
        except height_scan_wire.HeightScanWireError as e:
            self._scan_bad += 1
            if self._scan_bad in (1, 10, 100) or self._scan_bad % 1000 == 0:
                self.get_logger().warn(f"bad height-scan frame ({self._scan_bad} so far): {e}")
            return

        heights = frame.pop("heights")
        seq = frame["seq"]
        if self._scan_seq_prev is not None:
            gap = (seq - self._scan_seq_prev - 1) & 0xFFFFFFFF
            # Only count small forward gaps as drops; a large one means the bridge
            # restarted and its counter went back to zero.
            if 0 < gap < 1000:
                self._scan_drops += gap
        self._scan_seq_prev = seq

        with self._scan_lock:
            self.height_scan = heights
            self.last_scan_t = self._now()
            self.scan_geom = frame

        if not self._scan_logged:
            self._scan_logged = True
            self.get_logger().info(
                f"height scan online: {frame['num_x']}x{frame['num_y']} = {heights.size} cells "
                f"@ {frame['resolution']:.3g} m, centre +{frame['center_x']:.2g} m fwd, "
                f"{frame['unobserved_count']} unobserved")

    def _height_scan_for_obs(self):
        """(scan, age_s) for the control loop, or (None, inf) if nothing has arrived."""
        with self._scan_lock:
            if self.height_scan is None:
                return None, float("inf")
            return self.height_scan, self._now() - self.last_scan_t

    def _height_scan_ready(self):
        """True when a policy that needs terrain may be driven right now."""
        _, age = self._height_scan_for_obs()
        return age <= HEIGHT_SCAN_TIMEOUT

    # ------------------------------------------------------------------ #
    # Web-command handlers (driven by the bridge over UDP)
    # ------------------------------------------------------------------ #
    def _on_lowstate(self, msg: LowState_):
        self.low_state = msg

    def _set_teleop(self, vx, vy, wz):
        # The joystick UI's raw lateral/yaw axes do not match the body-frame
        # convention training used (Isaac Lab standard: +x forward, +y LEFT,
        # +yaw counter-clockwise/left -- see go2_velocity_env_cfg.py's
        # UniformLevelVelocityCommandCfg). web_bridge.cpp already negates the
        # same two axes ("VY_SCALE * -vy", "VYAW_SCALE * -vyaw") before handing
        # them to SportClient.Move() for exactly this reason; do the same here
        # so `velocity_commands` matches what the policy was trained against.
        # vx needs no correction -- forward/back agrees in both conventions,
        # which is why only left/right and yaw were ever reported as flipped.
        with self._lock:
            self.cmd = np.array([vx, -vy, -wz], np.float32)
            self.last_cmd_t = self._now()

    def _set_posture(self, stand: bool):
        """Latch the sit/stand target for a posture policy.

        A no-op for velocity policies: they carry no ``posture_command`` observation,
        so ``_build_obs`` never reads this. Accepting it regardless keeps the bridge
        protocol uniform and means the web panel does not need to know which kind of
        policy is loaded.
        """
        with self._lock:
            self.posture_cmd = 1.0 if stand else 0.0
        # Say so when the loaded policy cannot act on this, rather than logging as if
        # it had: a velocity policy accepts the value and then ignores it.
        if "posture_command" not in self.obs_terms:
            self.get_logger().warn(
                f"posture '{'stand' if stand else 'sit'}' ignored: the loaded policy is "
                "velocity-driven (no posture_command in its deploy.yaml).")
            return
        self.get_logger().info(f"posture -> {'STAND' if stand else 'SIT'}")

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
        """Build a fresh ORT session + its deployment contract and swap both in.

        Assigns only after everything is built, so a bad file never leaves a
        half-loaded session. Raises on error -- including a missing deploy.yaml, which
        is treated as "this policy cannot be run", not "run it on defaults".
        """
        path = pathlib.Path(path)
        contract = DeployContract.load(str(path.parent))

        sess = ort.InferenceSession(str(path), providers=["CPUExecutionProvider"])
        onnx_dim = int(sess.get_inputs()[0].shape[1])
        if onnx_dim != contract.obs_dim:
            raise DeployContractError(
                f"{path.name}: onnx expects {onnx_dim} observations but "
                f"{contract.source} describes {contract.obs_dim} "
                f"({' + '.join(f'{t}:{contract.obs_widths[t]}' for t in contract.obs_terms)}). "
                "The contract and the exported policy are from different runs."
            )

        # The joint order the contract was exported under must match the order this
        # node builds observations in, or every joint is fed the wrong number.
        if contract.joint_ids_map is not None and len(contract.joint_ids_map) != len(self.isaac_joints):
            raise DeployContractError(
                f"{contract.source}: joint_ids_map has {len(contract.joint_ids_map)} entries, "
                f"this node maps {len(self.isaac_joints)} joints"
            )
        if len(contract.default_joint_pos) != len(self.isaac_joints):
            raise DeployContractError(
                f"{contract.source}: default_joint_pos has {len(contract.default_joint_pos)} "
                f"entries, expected {len(self.isaac_joints)}"
            )

        # A perception policy is only loadable if the scan it expects matches the grid
        # the perception stack produces. Width is the part the contract records; the
        # geometry (resolution / centre) is checked against the live frame below.
        if contract.uses_height_scan:
            want = contract.obs_widths["height_scan"]
            geom = self.scan_geom
            if geom is not None and geom["num_x"] * geom["num_y"] != want:
                raise DeployContractError(
                    f"{contract.source}: policy expects a {want}-cell height scan but the "
                    f"perception stack is publishing {geom['num_x']}x{geom['num_y']} = "
                    f"{geom['num_x'] * geom['num_y']}. Reconcile heightmap_node's grid "
                    "params with the env the policy trained under."
                )

        if contract.include_base_lin_vel:
            self.get_logger().warn(
                f"{contract.source} declares base_lin_vel, which the Go2 cannot measure in "
                f"low-level mode -- feeding ZEROS for obs 0:3 (--lin-vel-mode "
                f"{self.lin_vel_mode}). The policy is running outside its training "
                "distribution on its velocity-tracking inputs; expect degraded tracking. "
                "Retrain without the term (sim_to_real_deployment_plan.md Phase 0).")

        with self._policy_lock:
            self.session = sess
            self.in_name = sess.get_inputs()[0].name
            self.obs_dim = onnx_dim
            self.contract = contract
            self.action_scale = contract.action_scale
            self.kp, self.kd = contract.kp, contract.kd
            self.include_base_lin_vel = contract.include_base_lin_vel
            # Observation layout and the per-term clip/scale training applied. Without
            # these the vector is the right width and the wrong magnitude, which is the
            # one failure this whole contract exists to prevent.
            self.obs_terms = list(contract.obs_terms)
            self.obs_scales = dict(contract.obs_scales)
            self.obs_clips = dict(contract.obs_clips)
            ranges = contract.command_ranges
            if ranges is not None:
                self.cmd_lo = np.asarray(ranges[0], np.float32)
                self.cmd_hi = np.asarray(ranges[1], np.float32)
            else:
                self.get_logger().warn(
                    f"{contract.source}: no commands.base_velocity.ranges; clamping the "
                    f"joystick to +-{CMD_CLIP} on every axis instead")
                self.cmd_lo = np.full(3, -CMD_CLIP, np.float32)
                self.cmd_hi = np.full(3, CMD_CLIP, np.float32)
            # Gait clock, for policies trained with a gait_phase observation. Swapping
            # policies restarts it: a new policy's cycle has nothing to do with where the
            # previous one happened to be.
            self.gait_phase_period = contract.gait_phase_period
            self.gait_phase_dt = contract.step_dt
            self.gait_phase = 0.0
            # default_joint_pos is recorded in Isaac order, same as our obs vector
            self.q_default_isaac = np.asarray(contract.default_joint_pos, np.float32)
            self.q_default_sdk = self.q_default_isaac[self.sdk_from_isaac]

    def _load_policy_entry(self, entry):
        """Load the onnx described by a registry entry. Returns True on success and
        leaves the current policy untouched on any failure (unknown/unrunnable/missing)."""
        pid = entry.get("id")
        # NOTE: uses_heightmap is no longer a refusal. This node CAN be fed a height scan
        # now (the bridge forwards one over the UDP link), so whether a perception policy
        # may run is a question about live data, not about the policy. Loading is
        # harmless -- it builds an onnx session and reads the contract -- and the real
        # gate is in _engage(), which refuses to release the sport service without a
        # fresh scan. 'runnable' still means "this launcher cannot run it at all".
        if not entry.get("runnable", True):
            self.get_logger().warn(f"policy '{pid}' is marked not runnable; not loading")
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
        # policies.json's obs_dim is hand-maintained; deploy.yaml is generated. Keep
        # reporting a mismatch so the registry gets corrected, but the contract wins.
        exp = entry.get("obs_dim")
        if exp is not None and int(exp) != self.obs_dim:
            self.get_logger().warn(
                f"policy '{pid}' onnx obs_dim {self.obs_dim} != policies.json {exp} "
                "(stale registry entry -- the onnx and its deploy.yaml agree)")
        self.active_policy_id = pid
        self.get_logger().info(
            f"loaded policy '{pid}' from {path} — {self.contract!r}")
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
        """Emergency stop from the web STOP button. STOP always wins: the motors go
        damp no matter what the node was doing -- including mid flip-recovery -- and
        nothing tries to stand up again until RESUME."""
        if on:
            with self._lock:
                # STOP must damp the motors from EVERY phase -- there is no state in which
                # the robot should keep its stance after STOP. RECOVER is included so an
                # in-flight flip recovery is aborted; SPORT is included so that even when
                # the policy is idle and the sport service owns the motors (RL selected
                # but not engaged), the ESTOP still damps (via SportClient.Damp in
                # _estop_damp) instead of leaving the robot standing.
                if self.phase == RECOVER:
                    self._abort_recover = True       # tear down the in-flight flip recovery
                self.phase = ESTOP
                self._recover_pending = False        # cancel any queued stand-up
                self._flip_pending = False           # cancel a not-yet-started flip recovery
                self._estop_pending = True           # worker: kill auto-recovery + sport Damp
                self.get_logger().warn("ESTOP: damping motors (soft collapse)")
            self._wake.set()
        else:
            with self._lock:
                if self.phase == ESTOP:
                    self._recover_pending = True
            self._wake.set()

    def _estop_damp(self):
        """Worker-side half of an ESTOP. Runs the blocking SDK work the rx thread and
        50 Hz loop must not: disarm the Go2's built-in auto-recovery so the firmware
        stops trying to stand the robot up, and -- if the sport service currently owns
        the motors (e.g. we STOPped mid flip-recovery) -- damp through it. The 50 Hz
        loop separately drives a lowcmd damp for the case where the policy owns them.
        Latched: nothing stands up again until RESUME clears the ESTOP."""
        with self._lock:
            self._estop_pending = False
            self._abort_recover = False         # abort consumed; don't trip the next recovery
            sport_owns = self._sport_owns
        self._set_auto_recovery(False)          # stop the firmware from auto-standing
        if sport_owns and not self.dry_run:
            self.get_logger().warn("ESTOP: sport service owns the motors -> SportClient.Damp()")
            with self._sport_lock:
                self.sc.Damp()

    def _recover_aborted(self):
        with self._lock:
            return self._abort_recover

    def _sleep_or_abort(self, total):
        """Sleep up to ``total`` s, returning True the instant a STOP requests an abort.
        Lets a blocking recovery bail out promptly instead of ignoring STOP for seconds."""
        end = self._now() + total
        while self._now() < end:
            if self._recover_aborted():
                return True
            time.sleep(0.05)
        return self._recover_aborted()

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
                estop = self._estop_pending
            # STOP wins over everything: complete the sport-side damp (kill firmware
            # auto-recovery, and Damp through the sport service if it owns the motors).
            # The 50 Hz loop already drives a lowcmd damp for the case where we own them.
            if estop:
                self._estop_damp()
                continue
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
        refusal = None
        if entry is not None and not entry.get("runnable", True):
            refusal = "it is marked not runnable in the registry"
        elif self.contract is not None and self.contract.uses_height_scan:
            # The gate that matters for perception: never release the sport service for a
            # policy that steps on terrain it cannot currently see. Checked here, at the
            # single point where control is handed over, rather than trusting a static
            # registry flag to predict whether the camera is alive.
            scan, age = self._height_scan_for_obs()
            if scan is None:
                refusal = (
                    "it needs a height scan and none has arrived. Start the perception "
                    "stack (ros2 launch go2_bringup real_perception.launch.py rviz:=false) "
                    "and check /tmp/go2_rl_bridge.log for 'forwarding height scans'")
            elif age > HEIGHT_SCAN_TIMEOUT:
                refusal = f"its height scan is stale ({age:.2f} s old; limit {HEIGHT_SCAN_TIMEOUT} s)"
        if refusal is not None:
            self.get_logger().error(
                f"refusing to engage '{self.active_policy_id}': {refusal}; staying in sport mode")
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
            self.gait_phase = 0.0        # fresh episode: restart the gait clock
            # The ramp below drives the robot to the policy's DEFAULT pose, which is a
            # standing one. Re-latch stand so a posture policy is not asked to sit the
            # instant it takes over from a ramp that just stood it up.
            self.posture_cmd = 1.0
            self.ramp_t = 0.0
            self._sat_steps = 0          # and fresh guards, so a previous run's
            self._tilt_since = None      # saturation/tilt history cannot trip this one
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
        """Stand back up after an ESTOP (RESUME button) -- the only path that stands
        the robot after STOP damped it. Re-arms the firmware auto-recovery that STOP
        disabled, then stands up through whichever controller owns the motors:

        - RL owns them (STOPped mid-policy, sport still released from engage): ramp the
          collapsed pose back to the default pose and resume the policy.
        - Sport owns them (STOPped during/after a flip recovery): RecoveryStand through
          the sport service and settle in sport mode, mirroring the flip-recovery landing.
        """
        self._set_auto_recovery(True)           # re-arm the firmware auto-recovery STOP killed
        with self._lock:
            self._recover_pending = False
            sport_owns = self._sport_owns
        if sport_owns:
            self.get_logger().info("RECOVER: standing back up under sport service")
            if self.handover and not self.dry_run:
                with self._sport_lock:
                    self.sc.RecoveryStand()
                time.sleep(FLIP_RECOVERY_WAIT)
                with self._sport_lock:
                    self.sc.BalanceStand()
                time.sleep(0.5)
            self._send({"t": "mode_out", "mode": "sport"})
            self._send({"t": "enabled", "val": True})
            with self._lock:
                self.requested_mode = "sport"
                self.phase = SPORT
            return
        self.get_logger().info("RECOVER: standing back up under RL policy")
        with self._lock:
            self.start_pos_sdk = self._read_q_sdk()
            self.last_action = np.zeros(12, np.float32)
            self.gait_phase = 0.0        # fresh episode: restart the gait clock
            # The ramp below drives the robot to the policy's DEFAULT pose, which is a
            # standing one. Re-latch stand so a posture policy is not asked to sit the
            # instant it takes over from a ramp that just stood it up.
            self.posture_cmd = 1.0
            self.ramp_t = 0.0
            self._sat_steps = 0          # and fresh guards, so a previous run's
            self._tilt_since = None      # saturation/tilt history cannot trip this one
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
            # STOP may have fired between the loop latching RECOVER and us getting here.
            # Check-and-set atomically so a concurrent ESTOP is never clobbered: if the
            # abort is already up, bail without touching phase (it is ESTOP by now).
            if self._abort_recover:
                return
            self._flip_pending = False
            self.requested_mode = "sport"    # stay idle after recovery; no auto re-engage
            self.phase = RECOVER             # (already set by the loop) loop drives nothing
        time.sleep(0.1)                      # let the 50 Hz loop observe RECOVER and go quiet
        if self.handover and not self.dry_run:
            # Between each blocking step, bail the instant STOP asks for an abort so the
            # robot goes damp instead of finishing the stand-up. _estop_damp (worker) then
            # kills firmware auto-recovery and Damps; the phase is already ESTOP.
            if self._recover_aborted():
                return
            self._select_sport()             # restart the sport svc (released on engage)
            if self._recover_aborted():
                return
            self._set_auto_recovery(True)    # re-arm built-in recovery (release may clear it)
            if self._recover_aborted():
                return
            # Let the sport service self-right: with auto-recovery armed it rolls the
            # robot off its back and stands on its own. We do NOT Damp first -- limp
            # motors would stop the firmware from righting itself. RecoveryStand is a
            # nudge that also covers the fell-on-its-side case.
            with self._sport_lock:
                self.sc.RecoveryStand()
            if self._sleep_or_abort(FLIP_RECOVERY_WAIT):   # let the roll-upright + stand finish
                return
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
        # Remember the mode that was active so we can restore *that exact* mode on
        # disengage. Hard-coding "normal" is fragile -- the name varies by firmware
        # (e.g. "normal" vs "ai"), and selecting the wrong one silently no-ops,
        # leaving the robot released and limp.
        if result and result.get("name"):
            self._prev_sport_mode = result["name"]
        self.get_logger().info(
            f"releasing sport service (was: {result}, will restore '{self._prev_sport_mode}')")
        for _ in range(10):
            if not result or not result.get("name"):
                break
            self.msc.ReleaseMode()
            time.sleep(0.5)
            status, result = self.msc.CheckMode()
        with self._lock:
            self._sport_owns = False          # sport svc released -> the policy owns lowcmd now
        self.get_logger().info(f"sport service released (mode now: {result})")

    def _select_sport(self, mode=None):
        """Bring the Unitree sport service back after a release, and VERIFY it.

        Symmetric to _release_sport: fire SelectMode, then poll CheckMode until a
        mode is actually active (non-empty name), retrying a few times. Restores the
        mode captured at release time (falls back to 'normal'). Returns True on
        success. Logs loudly on failure so a limp robot is diagnosable, not silent."""
        name = mode or getattr(self, "_prev_sport_mode", None) or "normal"
        result = None
        for attempt in range(6):
            self.msc.SelectMode(name)
            time.sleep(1.0)
            _status, result = self.msc.CheckMode()
            if result and result.get("name"):
                with self._lock:
                    self._sport_owns = True   # sport svc back in control of the motors
                self.get_logger().info(f"sport service selected (mode now: {result})")
                return True
            self.get_logger().warn(
                f"sport service still down after SelectMode('{name}') "
                f"attempt {attempt + 1}/6 (CheckMode={result}); retrying")
        self.get_logger().error(
            f"sport service FAILED to restart after SelectMode('{name}'); robot is likely "
            "limp -- recover with the Unitree remote (L2+A / damp+stand) or power-cycle")
        return False

    def _set_auto_recovery(self, enabled):
        """Toggle the Go2's built-in fall/flip auto-recovery (SportClient AutoRecoverySet).

        When on, the sport service automatically rights the robot after a fall --
        including rolling it off its back -- which a one-shot RecoveryStand cannot do.
        Best-effort: logs and swallows errors (e.g. if the SDK build lacks the API)."""
        try:
            with self._sport_lock:
                self.sc.AutoRecoverySet(bool(enabled))
                got = self.sc.AutoRecoveryGet()
            self.get_logger().info(f"auto-recovery set to {enabled} (readback: {got})")
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f"could not set auto-recovery ({e}); relying on RecoveryStand")

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
        # Perception deadman, same failure response for the same reason: while a rough
        # policy owns the motors, terrain that stops updating means it is stepping onto
        # ground it can no longer see. Holding the last scan would keep it walking
        # confidently off the edge of what it knows.
        if phase in (ENGAGE, RL_RUN) and self.contract is not None and self.contract.uses_height_scan:
            _scan, age = self._height_scan_for_obs()
            if age > HEIGHT_SCAN_TIMEOUT:
                self.get_logger().error(
                    f"height scan stale ({age:.2f} s > {HEIGHT_SCAN_TIMEOUT} s); ESTOP (soft collapse)")
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
                tilted = self._tilt_abort_detected()
                if self.flip_recovery and (tilted or self._flip_detected()):
                    self.get_logger().warn(
                        "tilt past the trained envelope; starting auto-recovery" if tilted
                        else "flip detected (robot inverted and settled); starting auto-recovery")
                    with self._lock:
                        self.phase = RECOVER      # stop driving lowcmd this instant
                        self._flip_pending = True
                    self._wake.set()              # transition worker runs the sequence
                    return
                self._policy_step()
            elif phase == DISENGAGE:
                self._publish(self.q_default_sdk, self.kp, self.kd)
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
                self.blend_t = 0.0           # fade the policy in from here
            self.get_logger().info(
                f"RL_RUN: policy active (fading in over {BLEND_TIME:g} s)")

    def _blend_alpha(self):
        """Policy authority in [0, 1], faded in over BLEND_TIME at the handover.

        _ramp_step leaves the robot static at q_default under stiff gains
        (RAMP_KP/RAMP_KD = 40/4). Entering RL_RUN used to change two things in the same
        control step: the target jumped to the policy's first action -- a real move,
        measured at 13.5 deg on the worst joint for flat-dr, because q_default is the
        action *origin* and not a pose the policy was ever rewarded for holding -- and
        the gains dropped to the trained 25/0.5, an 8x cut in damping. Stepping both at
        once is what makes the handover snap. This fades them in together instead.

        Smoothstep, not linear: zero slope at both ends, so neither the start of the
        blend nor its completion is itself a corner.

        Attenuating the *action* is exactly an interpolation of the joint target,
        because q_default is the offset the action is added to::

            q_def + s*(alpha*a)  ==  (1 - alpha)*q_def + alpha*(q_def + s*a)

        so there is one knob rather than two that can disagree.
        """
        if self.blend_t >= BLEND_TIME:
            return 1.0
        self.blend_t += CONTROL_DT           # the control thread's own period
        if self.blend_t >= BLEND_TIME:
            self.get_logger().info("RL_RUN: policy at full authority")
            return 1.0
        u = self.blend_t / BLEND_TIME
        return float(u * u * (3.0 - 2.0 * u))

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
        peak = float(np.abs(action).max())
        if peak > ACTION_CLIP:
            self._sat_steps += 1
            if self._sat_steps == 1 or self._sat_steps % 10 == 0:
                self.get_logger().warn(
                    f"action saturating (|a|max={peak:.1f} > {ACTION_CLIP}); clipping "
                    f"[{self._sat_steps} step(s)]")
            if self._sat_steps >= ACTION_SAT_STEPS:
                self.get_logger().error(
                    f"action saturated for {self._sat_steps} steps (|a|max={peak:.1f}); "
                    "ESTOP. This is what a wrong observation looks like, not a hot gait -- "
                    f"check the obs layout/scales against {getattr(self.contract, 'source', '?')}")
                with self._lock:
                    self.phase = ESTOP
                self._damp()
                return
            action = np.clip(action, -ACTION_CLIP, ACTION_CLIP)
        else:
            self._sat_steps = 0
        # Fade-in only; the guards above ran on the *raw* action, so a saturating
        # policy is caught from the first step rather than being masked by a small alpha.
        alpha = self._blend_alpha()
        action = alpha * action
        # Feed back what was actually commanded, not what the policy asked for. In
        # training last_action is the action that was applied, and keeping that invariant
        # through the blend is what stops the policy reacting to a move it never made.
        with self._lock:
            self.last_action = action.astype(np.float32)
        q_target_isaac = self.q_default_isaac + self.action_scale * action
        q_target_sdk = q_target_isaac[self.sdk_from_isaac]
        # Gains ride the same curve, RAMP_KP/KD -> the trained kp/kd.
        self._publish(q_target_sdk,
                      RAMP_KP + alpha * (self.kp - RAMP_KP),
                      RAMP_KD + alpha * (self.kd - RAMP_KD))

    def _tilt_abort_detected(self):
        """True while the base has been tilted past TILT_ABORT_RAD for the debounce.

        The complement of _flip_detected(): that one waits for the robot to come to rest
        upside down, so during a fall -- exactly when driving the motors does the damage
        -- it never fires. This fires *during* the fall, at the angle unitree_rl_lab's
        C++ deploy drops to Passive (isaaclab::mdp::bad_orientation, 1.0 rad), which is
        already past the 0.8 rad the training env terminates an episode at. Beyond that
        the policy is extrapolating, so handing the robot to the sport service to stand
        itself up beats letting it keep commanding joints.

        Shares the flip cooldown so a recovery is not re-triggered by its own motion.
        Called only from RL_RUN, where low_state is guaranteed non-None.
        """
        if self._now() < self._flip_cooldown_until:
            self._tilt_since = None
            return False
        proj_g = projected_gravity(self.low_state.imu_state.quaternion)
        # |proj_g| is 1, so -proj_g[2] is cos(tilt from upright); clamp for acos safety.
        tilt = float(np.arccos(np.clip(-proj_g[2], -1.0, 1.0)))
        if tilt <= TILT_ABORT_RAD:
            self._tilt_since = None
            return False
        now = self._now()
        if self._tilt_since is None:
            self._tilt_since = now
            return False
        return (now - self._tilt_since) >= TILT_ABORT_DEBOUNCE

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

    def _apply_obs_transform(self, name, values):
        """Clip then scale one observation term, exactly as training did.

        Isaac Lab's ObservationManager applies noise -> clip -> scale, and
        unitree_rl_lab's C++ deploy repeats the clip -> scale half on the robot
        (ObservationTermCfg::add). There is no noise on hardware, so this is the whole
        transform. Both halves come from the policy's own deploy.yaml; a term the
        contract left unscaled/unclipped is passed straight through, which is why the
        stock 48-dim policies are unaffected by this step.
        """
        clip = self.obs_clips.get(name)
        if clip is not None:
            values = np.clip(values, clip[0], clip[1])
        scale = self.obs_scales.get(name)
        if scale is not None:
            values = values * np.asarray(scale, np.float32)
        return np.asarray(values, np.float32)

    def _clip_cmd(self, cmd):
        """Clamp the joystick to the per-axis envelope the command was sampled from.

        The Go2 tasks train on +-1.0 in x and yaw but only +-0.4 laterally, so the old
        symmetric +-CMD_CLIP handed a full-stick strafe 2.5x outside anything the policy
        had seen. Mirrors the per-axis std::clamp in the C++ velocity_commands term.
        """
        return np.clip(cmd, self.cmd_lo, self.cmd_hi).astype(np.float32)

    def _build_obs(self):
        ls = self.low_state
        q_sdk = np.array([ls.motor_state[i].q for i in range(12)], np.float32)
        dq_sdk = np.array([ls.motor_state[i].dq for i in range(12)], np.float32)
        q_isaac = q_sdk[self.isaac_from_sdk]
        dq_isaac = dq_sdk[self.isaac_from_sdk]
        with self._lock:
            timed_out = (self._now() - self.last_cmd_t) > CMD_TIMEOUT
            cmd = np.zeros(3, np.float32) if timed_out else self._clip_cmd(self.cmd)
            last_a = self.last_action.copy()
            posture = self.posture_cmd
        # Keyed by term name and emitted in the contract's own order, rather than
        # positionally: which terms are present *and in what order* is recorded in
        # deploy.yaml, and a reordered observation is well-formed and silently wrong.
        raw = {
            "base_lin_vel": np.zeros(3, np.float32),   # lin_vel_mode == "zero"
            "base_ang_vel": np.asarray(ls.imu_state.gyroscope, np.float32),
            "projected_gravity": projected_gravity(ls.imu_state.quaternion),
            "velocity_commands": cmd,
            "joint_pos_rel": q_isaac - self.q_default_isaac,
            "joint_vel_rel": dq_isaac,
            "last_action": last_a,
            "posture_command": np.array([posture], np.float32),
        }
        if "height_scan" in self.obs_terms:
            scan, _age = self._height_scan_for_obs()
            # Freshness is enforced by the control loop's deadman before we get here, so
            # by this point a scan exists. The guard is for the impossible case only --
            # and it raises rather than zero-fills, because a silent flat-ground scan is
            # the failure this whole path is built to prevent.
            if scan is None:
                raise RuntimeError("policy needs a height scan but none has ever arrived")
            raw["height_scan"] = scan
        parts = []
        for name in self.obs_terms:
            # gait_phase is a clock, not a sensor: advance it exactly once per control
            # step, and only when the contract says the policy carries it.
            value = self._advance_gait_phase() if name == "gait_phase" else raw[name]
            parts.append(self._apply_obs_transform(name, value))
        return np.concatenate(parts).astype(np.float32)

    def _advance_gait_phase(self):
        """(sin, cos) of the gait clock, advanced one control step.

        Ported from ``REGISTER_OBSERVATION(gait_phase)`` in unitree_rl_lab's own deploy
        stack (deploy/include/isaaclab/envs/mdp/observations/observations.h), which is the
        reference for what a gait-conditioned policy expects on hardware. An accumulator,
        not the training side's ``episode_length_buf * step_dt``: there is no episode
        counter here, and the two agree while this is called once per control step. Like
        upstream it advances *before* emitting, so the first sample sits one step in
        rather than at phase 0, and _engage() zeroes it the way the C++ env's reset() does.
        """
        self.gait_phase = (self.gait_phase + self.gait_phase_dt / self.gait_phase_period) % 1.0
        angle = self.gait_phase * 2.0 * np.pi
        return np.array([np.sin(angle), np.cos(angle)], np.float32)

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
