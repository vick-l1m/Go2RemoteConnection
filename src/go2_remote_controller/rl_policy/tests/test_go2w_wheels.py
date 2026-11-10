#!/usr/bin/env python3
"""Go2-W wheel handling in go2_rl_policy_node: lowcmd slot writes and obs widths.

Runs the node's publish / observation helpers on a controller built WITHOUT its
constructor (no DDS, no UDP, no onnx), with the joint tables set exactly as
_load_policy_file() sets them for a Go2 policy, a Go2-W locked policy and a Go2-W
drive policy. Needs the robot venv (unitree_sdk2py is imported by the node)::

    ~/venvs/unitree_sdk2_python/bin/python rl_policy/tests/test_go2w_wheels.py

Plain asserts, no pytest (the venv has none).
"""

import os
import sys
import threading
import types

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

import go2_rl_policy_node as node  # noqa: E402

# Isaac breadth-first order -> SDK slot (export_deploy_cfg's joint_ids_map) for the
# Go2 and the Go2-W (legs, then FL/FR/RL/RR wheel axles onto slots 13/12/15/14).
GO2_MAP = [3, 0, 9, 6, 4, 1, 10, 7, 5, 2, 11, 8]
GO2W_MAP = GO2_MAP + [13, 12, 15, 14]


class _Motor:
    def __init__(self):
        self.mode = self.q = self.dq = self.kp = self.kd = self.tau = 0.0


class _LowCmd:
    def __init__(self):
        self.motor_cmd = [_Motor() for _ in range(20)]
        self.crc = 0


class _Pub:
    def __init__(self):
        self.n = 0

    def Write(self, _msg):
        self.n += 1


class _Contract:
    """Only what _joint_obs reads."""

    def __init__(self, widths):
        self.obs_widths = widths
        self.obs_history = {}


def make(robot, n_isaac, leg_ids, wheel_ids, widths, wheel_gains=(25.0, 0.5), wheel_scale=0.0):
    c = node.Go2RLPolicyController.__new__(node.Go2RLPolicyController)
    c._logger = node._StdLogger("test")
    c.dry_run = False
    c.robot = robot
    c.sdk_joints = node.SDK_JOINTS_BY_ROBOT[robot]
    c.n_sdk = len(c.sdk_joints)
    jmap = (GO2W_MAP if n_isaac == 16 else GO2_MAP)
    c.isaac_joints = [c.sdk_joints[s] for s in jmap]
    c.isaac_from_sdk = list(jmap)
    c.leg_isaac = list(leg_ids)
    c.leg_sdk = [jmap[i] for i in leg_ids]
    c.wheel_sdk = [s for s in range(c.n_sdk) if s not in set(c.leg_sdk)]
    c.driven_isaac = list(wheel_ids)
    c.driven_sdk = [jmap[i] for i in wheel_ids]
    c.wheel_kp, c.wheel_kd = wheel_gains
    c.wheel_scale = wheel_scale
    c.action_dim = len(leg_ids) + len(wheel_ids)
    c.q_default_isaac = np.array([node.Q_DEFAULT_BY_NAME[n] for n in c.isaac_joints], np.float32)
    c.q_default_sdk = c._to_sdk(c.q_default_isaac)
    c.wheel_hold_q_sdk = np.zeros(c.n_sdk, np.float32)
    c.contract = _Contract(widths)
    c.low_cmd = _LowCmd()
    c.crc = types.SimpleNamespace(Crc=lambda _m: 0)
    c.lowcmd_pub = _Pub()
    c._lock = threading.Lock()
    return c


def test_go2_unchanged():
    c = make("go2", 12, range(12), [], {"joint_pos_rel": 12, "joint_vel_rel": 12})
    assert c.wheel_sdk == [] and c.n_sdk == 12
    q = np.arange(12, dtype=np.float32)
    c._publish(q, 25.0, 0.5)
    for i in range(12):
        m = c.low_cmd.motor_cmd[i]
        assert (m.q, m.kp, m.kd, m.dq) == (float(i), 25.0, 0.5, 0.0)
    for i in range(12, 20):                      # never touched
        assert c.low_cmd.motor_cmd[i].kp == 0.0 and c.low_cmd.motor_cmd[i].q == 0.0
    assert c._joint_obs("joint_vel_rel", np.ones(12)).shape == (12,)


def test_go2_policy_on_go2w_brakes_the_wheels():
    """A 12-joint contract on --robot go2w: legs as on the Go2, wheels position-held
    at the angle captured on engage."""
    c = make("go2w", 12, range(12), [], {"joint_pos_rel": 12, "joint_vel_rel": 12},
             wheel_gains=(node.WHEEL_HOLD_KP, node.WHEEL_HOLD_KD))
    assert c.wheel_sdk == [12, 13, 14, 15] and c.driven_sdk == []
    c.wheel_hold_q_sdk[12:16] = [7.0, 8.0, 9.0, 10.0]     # wheels mid-turn at engage
    q = c.q_default_sdk.copy()
    c._publish(q, 25.0, 0.5)
    for i, hold in zip(range(12, 16), [7.0, 8.0, 9.0, 10.0]):
        m = c.low_cmd.motor_cmd[i]
        assert (m.q, m.kp, m.kd, m.dq) == (hold, node.WHEEL_HOLD_KP, node.WHEEL_HOLD_KD, 0.0)
    for i in range(12):
        assert c.low_cmd.motor_cmd[i].kp == 25.0
    # _to_sdk fills the slots no Isaac joint maps to
    assert c.q_default_sdk.shape == (16,) and np.all(c.q_default_sdk[12:] == 0.0)


def test_go2w_locked_policy():
    c = make("go2w", 16, range(12), [], {"joint_pos_rel": 12, "joint_vel_rel": 12, "last_action": 12})
    assert c.action_dim == 12 and c.wheel_sdk == [12, 13, 14, 15] and c.driven_sdk == []
    dq_isaac = np.arange(16, dtype=np.float32)
    assert c._joint_obs("joint_vel_rel", dq_isaac).tolist() == list(range(12))   # legs only
    assert c._joint_obs("joint_pos_rel", dq_isaac).shape == (12,)


def test_go2w_drive_policy():
    c = make("go2w", 16, range(12), [12, 13, 14, 15],
             {"joint_pos_rel": 12, "joint_vel_rel": 16, "last_action": 16},
             wheel_gains=(0.0, 1.0), wheel_scale=8.0)
    assert c.action_dim == 16
    assert c.driven_sdk == [13, 12, 15, 14]              # Isaac FL,FR,RL,RR -> SDK slots
    dq_isaac = np.arange(16, dtype=np.float32)
    assert c._joint_obs("joint_vel_rel", dq_isaac).shape == (16,)
    assert c._joint_obs("joint_pos_rel", dq_isaac).shape == (12,)

    # The action split _policy_step performs: legs -> q, wheels -> dq on their slots.
    action = np.zeros(16, np.float32)
    action[12:] = [0.5, -0.5, 0.25, 0.0]                 # FL, FR, RL, RR in rad/s / 8
    n_leg = 12
    q_target_sdk = c.q_default_sdk.copy()
    q_target_sdk[c.leg_sdk] = c.q_default_isaac[c.leg_isaac] + 0.25 * action[:n_leg]
    dq_sdk = np.zeros(16, np.float32)
    dq_sdk[c.driven_sdk] = c.wheel_scale * action[n_leg:]
    c._publish(q_target_sdk, 25.0, 0.5, dq_sdk=dq_sdk)
    fr, fl, rr, rl = (c.low_cmd.motor_cmd[i] for i in (12, 13, 14, 15))
    assert (fl.dq, fr.dq, rl.dq, rr.dq) == (4.0, -4.0, 2.0, 0.0)
    for m in (fr, fl, rr, rl):
        assert (m.q, m.kp, m.kd, m.mode) == (0.0, 0.0, 1.0, 1)
    for i in range(12):
        assert c.low_cmd.motor_cmd[i].kp == 25.0 and c.low_cmd.motor_cmd[i].dq == 0.0

    # dry run: no torque anywhere, wheels included
    c.dry_run = True
    c._publish(q_target_sdk, 25.0, 0.5, dq_sdk=dq_sdk)
    for i in range(16):
        m = c.low_cmd.motor_cmd[i]
        assert m.kp == 0.0 and m.kd == 0.0 and m.dq == 0.0
    c.dry_run = False

    # damp: every slot limp, wheels with the Passive kd
    c._damp()
    for i in range(16):
        m = c.low_cmd.motor_cmd[i]
        assert m.kp == 0.0
        assert m.kd == (node.WHEEL_DAMP_KD if i >= 12 else node.DAMP_KD)


def test_sdk_wheel_slots_match_the_training_names():
    """The node's slot names must agree with training's GO2W_JOINT_SDK_NAMES, which is
    what joint_ids_map is written against."""
    want = ["FR_foot", "FL_foot", "RR_foot", "RL_foot"]
    assert node.SDK_WHEEL_JOINTS == want
    assert node.SDK_JOINTS_BY_ROBOT["go2w"][12:] == want
    assert node.SDK_JOINTS_BY_ROBOT["go2w"][:12] == node.SDK_JOINTS


if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for fn in fns:
        fn()
        print(f"ok  {fn.__name__}")
    print(f"{len(fns)} passed")
