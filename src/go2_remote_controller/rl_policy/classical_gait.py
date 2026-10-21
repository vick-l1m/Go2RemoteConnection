"""Classical (non-learned) Go2 gait for the real robot -- numpy, no torch, no ROS.

A line-for-line numpy port of ``training/go2_training/classical/gait.py`` (the torch
controller the residual-RL tasks are built on) for one robot. The step numbering in
:meth:`ClassicalGait.step` matches that file's so the two can be diffed by eye, and the
parent workspace's ``training/tests/test_classical_port_parity.py`` runs both on the
same inputs and fails on any drift. **Change both together.**

Inputs per 20 ms tick are what the robot measures -- velocity command, projected
gravity and gyro (IMU), joint positions and velocities (encoders) -- in Isaac leg order
(FL, FR, RL, RR x hip/thigh/calf). Output: 12 joint position targets in the same order,
for a PD with the gains in the gait's JSON (``kp``/``kd``).

Loaded by ``go2_rl_policy_node`` for a ``policies.json`` entry of ``"kind": "classical"``
whose ``path`` is a JSON of ``GaitParams`` overrides (``classical/tuned/trot.json`` from
the training side).
"""

from __future__ import annotations

import json
import math
from dataclasses import asdict, dataclass, fields

import numpy as np

# --- kinematics (training/go2_training/classical/kinematics.py) ---------------------
LEGS = ("FL", "FR", "RL", "RR")
THIGH_LEN = 0.213
CALF_LEN = 0.213
HIP_OFFSET = 0.0955
HIP = np.array([[0.1934, 0.0465, 0.0], [0.1934, -0.0465, 0.0], [-0.1934, 0.0465, 0.0], [-0.1934, -0.0465, 0.0]])
SIDE = np.array([1.0, -1.0, 1.0, -1.0])
Q_LO = np.array([[-1.0472, -1.5708, -2.7227], [-1.0472, -1.5708, -2.7227],
                 [-1.0472, -0.5236, -2.7227], [-1.0472, -0.5236, -2.7227]])
Q_HI = np.array([[1.0472, 3.4907, -0.83776], [1.0472, 3.4907, -0.83776],
                 [1.0472, 4.5379, -0.83776], [1.0472, 4.5379, -0.83776]])

GAITS = {
    "trot": (0.0, 0.5, 0.5, 0.0),
    "walk": (0.25, 0.75, 0.0, 0.5),
    "pace": (0.0, 0.5, 0.0, 0.5),
    "bound": (0.0, 0.0, 0.5, 0.5),
    "pronk": (0.0, 0.0, 0.0, 0.0),
}


def joint_names_leg_order() -> list[str]:
    return [f"{leg}_{j}_joint" for leg in LEGS for j in ("hip", "thigh", "calf")]


def forward_kinematics(q: np.ndarray) -> np.ndarray:
    """(4, 3) joint angles -> (4, 3) foot positions relative to each hip, body frame."""
    hip, thigh, calf = q[:, 0], q[:, 1], q[:, 2]
    x = -THIGH_LEN * np.sin(thigh) - CALF_LEN * np.sin(thigh + calf)
    lz = -THIGH_LEN * np.cos(thigh) - CALF_LEN * np.cos(thigh + calf)
    ly = SIDE * HIP_OFFSET
    ch, sh = np.cos(hip), np.sin(hip)
    return np.stack((x, ly * ch - lz * sh, ly * sh + lz * ch), axis=-1)


def inverse_kinematics(p: np.ndarray) -> np.ndarray:
    """(4, 3) foot positions relative to each hip -> (4, 3) joint angles (knee back)."""
    x, y, z = p[:, 0], p[:, 1], p[:, 2]
    a, b, L = THIGH_LEN, CALF_LEN, HIP_OFFSET
    e2 = np.maximum(y * y + z * z, L * L + 1e-6)
    lz = -np.sqrt(e2 - L * L)
    hip = np.arctan2(z, y) - np.arctan2(lz, SIDE * L)
    hip = np.arctan2(np.sin(hip), np.cos(hip))
    d = np.clip(np.sqrt(x * x + lz * lz), abs(a - b) + 1e-3, a + b - 1e-3)
    calf = np.arccos(np.clip((a * a + b * b - d * d) / (2.0 * a * b), -1.0, 1.0)) - math.pi
    theta = np.arctan2(-x, -lz)
    thigh = theta - np.arctan2(b * np.sin(calf), a + b * np.cos(calf))
    return np.stack((hip, thigh, calf), axis=-1)


def jacobian(q: np.ndarray, eps: float = 1e-4) -> np.ndarray:
    """(4, 3) -> (4, 3, 3) d(foot)/d(joints), central differences (as the torch side)."""
    cols = []
    for j in range(3):
        dq = np.zeros_like(q)
        dq[:, j] = eps
        cols.append((forward_kinematics(q + dq) - forward_kinematics(q - dq)) / (2.0 * eps))
    return np.stack(cols, axis=-1)


# --- parameters (training/go2_training/classical/gait.py GaitParams) ----------------
@dataclass
class GaitParams:
    freq: float = 2.5
    freq_gain: float = 0.0
    duty: float = 0.5
    swing_height: float = 0.08
    height: float = 0.30
    foot_x: float = 0.0
    foot_y: float = 0.11
    k_vel: float = 0.17
    k_wz: float = 0.1
    odom_alpha: float = 0.2
    k_att: float = 0.5
    max_dx: float = 0.18
    max_dy: float = 0.12
    fb_max: float = 0.12
    cmd_accel: float = 3.0
    act_speed: float = 0.08
    act_tau: float = 0.15
    cmd_deadband: float = 0.05
    err_deadband: float = 0.25
    k_h: float = 0.0
    ground_swing: float = 0.0
    h_corr_max: float = 0.08
    force_mode: float = 1.0
    f_kv: float = 3.0
    f_amax: float = 3.0
    f_kh: float = 60.0
    f_dh: float = 12.0
    f_att_kp: float = 40.0
    f_att_kd: float = 3.0
    com_x: float = 0.02
    f_att_ki: float = 60.0
    f_att_imax: float = 15.0
    f_kh_i: float = 30.0
    f_h_imax: float = 3.0
    f_yaw_kd: float = 2.0
    mu: float = 0.5
    tau_max: float = 20.0
    ff_mass: float = 15.0
    ff_kp: float = 25.0
    ff_ramp: float = 0.15
    ff_hip: float = 0.0


PARAM_NAMES = tuple(f.name for f in fields(GaitParams))


def _min_jerk(s):
    s = np.clip(s, 0.0, 1.0)
    return s * s * s * (10.0 + s * (-15.0 + 6.0 * s))


def _lift_profile(s):
    return np.where(s < 0.5, _min_jerk(2.0 * s), _min_jerk(2.0 - 2.0 * s))


def _rotate(v: np.ndarray, axis: np.ndarray, angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return v * c + np.cross(np.broadcast_to(axis, v.shape), v) * s + axis * (v @ axis)[:, None] * (1.0 - c)


class ClassicalGait:
    """One robot's classical gait. ``step()`` every control tick."""

    def __init__(self, params: GaitParams | dict | None = None, gait: str = "trot"):
        if gait not in GAITS:
            raise ValueError(f"Unknown gait {gait!r}; choose from {sorted(GAITS)}")
        if isinstance(params, dict):
            unknown = set(params) - set(PARAM_NAMES)
            if unknown:
                raise KeyError(f"Unknown gait parameter(s) {sorted(unknown)}")
            params = GaitParams(**params)
        self.p = params or GaitParams()
        self.gait = gait
        self.offsets = np.asarray(GAITS[gait], dtype=float)
        self.reset()

    @classmethod
    def from_json(cls, path: str) -> tuple["ClassicalGait", dict]:
        """Load ``{"params": {...}, "gait": "trot", "kp": .., "kd": ..}``; returns the gait
        and the whole JSON (the caller reads kp/kd from it)."""
        with open(path) as f:
            cfg = json.load(f)
        return cls(cfg.get("params", {}), cfg.get("gait", "trot")), cfg

    # -------------------------------------------------------------------- state
    def nominal_feet(self) -> np.ndarray:
        p = self.p
        return np.stack((np.full(4, p.foot_x), p.foot_y * SIDE, np.full(4, -(p.height + self.h_corr))), axis=-1)

    def reset(self) -> None:
        self.phase = 0.0
        self.v_ref = np.zeros(3)
        self.activity = 0.0
        self.h_corr = 0.0
        self.v_est = np.zeros(2)
        self.ground = -self.p.height
        self.h_dot = 0.0
        self.tau_ff = np.zeros((4, 3))
        self.att_int = np.zeros(2)
        self.h_int = 0.0
        nominal = self.nominal_feet()
        self.p_td = nominal.copy()
        self.p_lo = nominal.copy()
        self.p_cmd = nominal.copy()
        self.prev_stance = self.leg_phase() < self.p.duty

    def leg_phase(self) -> np.ndarray:
        return np.remainder(self.phase + self.offsets, 1.0)

    def phase_obs(self) -> np.ndarray:
        a = 2.0 * math.pi * self.phase
        return np.array([math.sin(a), math.cos(a)])

    # ------------------------------------------------------------------- forces
    def _stance_forces(self, share, v_xy, wz, projected_gravity, joint_pos, ang_vel_b, dt) -> np.ndarray:
        """Body wrench -> support-foot forces -> feedforward joint torque (Nm), (4, 3).
        See the torch controller's ``_stance_forces`` for the derivation."""
        p = self.p
        up = -np.asarray(projected_gravity, dtype=float)
        up = up / np.linalg.norm(up)
        r = forward_kinematics(joint_pos) + HIP
        r[:, 0] -= p.com_x  # moments about the CoM, not the base origin
        m = p.ff_mass
        a_xy = np.clip(p.f_kv * (v_xy - self.v_est), -p.f_amax, p.f_amax)
        h_err = p.height - (-self.ground)
        self.h_int = float(np.clip(self.h_int + p.f_kh_i * h_err * dt, -p.f_h_imax, p.f_h_imax))
        a_up = p.f_kh * h_err - p.f_dh * self.h_dot + self.h_int
        F = m * (9.81 * up + a_up * up)
        F[:2] += m * a_xy
        tilt = np.cross(np.array([0.0, 0.0, 1.0]), up)
        self.att_int = np.clip(self.att_int + p.f_att_ki * tilt[:2] * dt, -p.f_att_imax, p.f_att_imax)
        T = p.f_att_kp * tilt
        T[:2] += self.att_int
        T[:2] -= p.f_att_kd * np.asarray(ang_vel_b, dtype=float)[:2]
        T[2] = p.f_yaw_kd * (wz - ang_vel_b[2])
        W = np.concatenate((F, T))
        A = np.zeros((6, 12))
        for i in range(4):
            rx, ry, rz = r[i]
            A[0:3, 3 * i:3 * i + 3] = np.eye(3)
            A[3:6, 3 * i:3 * i + 3] = np.array([[0.0, -rz, ry], [rz, 0.0, -rx], [-ry, rx, 0.0]])
        d = np.repeat(share, 3) + 1e-6
        AD = A * d[None, :]
        M = AD @ A.T + 1e-4 * np.eye(6)
        f = (AD.T @ np.linalg.solve(M, W)).reshape(4, 3)
        fn = np.maximum(f @ up, 0.0)
        ft = f - (f @ up)[:, None] * up[None, :]
        ft_norm = np.maximum(np.linalg.norm(ft, axis=-1), 1e-9)
        ft = ft * np.minimum(p.mu * fn / ft_norm, 1.0)[:, None]
        f = fn[:, None] * up[None, :] + ft
        tau = -(np.swapaxes(jacobian(joint_pos), -1, -2) @ f[..., None])[..., 0]
        return np.clip(tau, -p.tau_max, p.tau_max)

    # --------------------------------------------------------------------- step
    def step(
        self,
        command: np.ndarray,
        dt: float,
        projected_gravity: np.ndarray | None = None,
        joint_pos: np.ndarray | None = None,
        joint_vel: np.ndarray | None = None,
        ang_vel_b: np.ndarray | None = None,
        lin_vel_b: np.ndarray | None = None,
    ) -> np.ndarray:
        """Advance by ``dt``; returns (4, 3) joint targets, leg order. Arrays as in the
        torch controller without the batch dimension (joint_pos/vel are (4, 3))."""
        p = self.p
        command = np.asarray(command, dtype=float)

        # 1. Deadbanded, rate-limited velocity reference.
        command = np.where(np.abs(command) < p.cmd_deadband, 0.0, command)
        step_max = p.cmd_accel * dt
        self.v_ref = self.v_ref + np.clip(command - self.v_ref, -step_max, step_max)
        v_xy, wz = self.v_ref[:2], self.v_ref[2]

        # 2. Velocity feedback (Raibert) from leg odometry (or a given true velocity).
        if lin_vel_b is None and joint_pos is not None and joint_vel is not None and ang_vel_b is not None:
            p_foot = forward_kinematics(joint_pos) + HIP
            foot_vel = (jacobian(joint_pos) @ joint_vel[..., None])[..., 0]
            base_vel = -(foot_vel + np.cross(np.broadcast_to(ang_vel_b, p_foot.shape), p_foot))[:, :2]
            lp_prev = self.leg_phase()
            st_prev = lp_prev < p.duty
            s_prev = lp_prev / p.duty
            wt = st_prev * np.clip(np.minimum(s_prev, 1.0 - s_prev) / 0.2, 0.0, 1.0)
            wt = wt + (1.0 - self.activity)
            v_meas = (base_vel * wt[:, None]).sum(0) / max(wt.sum(), 1e-6)
            self.v_est = self.v_est + p.odom_alpha * (v_meas - self.v_est)
            lin_meas = self.v_est
        elif lin_vel_b is not None:
            lin_meas = np.asarray(lin_vel_b, dtype=float)[:2]
        else:
            lin_meas = None
        if lin_meas is not None:
            err = lin_meas - v_xy
            fb = np.clip(p.k_vel * err, -p.fb_max, p.fb_max)
        else:
            err = np.zeros(2)
            fb = np.zeros(2)

        # 3. Stepping activity.
        speed = np.linalg.norm(v_xy) + 0.25 * abs(wz)
        disturbed = max(np.linalg.norm(err) - p.err_deadband, 0.0)
        drive = speed + float(p.k_vel > 0) * disturbed
        target = min(max(drive / p.act_speed, 0.0), 1.0)
        alpha = min(max(dt / p.act_tau, 0.0), 1.0)
        self.activity += alpha * (target - self.activity)

        # 4. Gait clock.
        freq = p.freq + p.freq_gain * np.linalg.norm(v_xy)
        self.phase = float(np.remainder(self.phase + freq * dt, 1.0))
        lp = self.leg_phase()
        stance = lp < p.duty
        s_st = lp / p.duty
        s_sw = (lp - p.duty) / (1.0 - p.duty)
        t_st = p.duty / freq

        # 5. Ground velocity under each foot: v + w x r.
        nominal = self.nominal_feet()
        r = HIP + nominal
        v_leg = np.stack((v_xy[0] - wz * r[:, 1], v_xy[1] + wz * r[:, 0]), axis=-1)

        # 6. Raibert foothold, clamped to the workspace box.
        reach = np.array([p.max_dx, p.max_dy])
        off = v_leg * (0.5 * t_st) + fb
        if ang_vel_b is not None:
            wz_err = ang_vel_b[2] - wz
            off = off + p.k_wz * wz_err * np.stack((-r[:, 1], r[:, 0]), axis=-1)
        foothold = nominal[:, :2] + np.clip(off, -reach, reach)

        # 7. Phase transitions latch where the foot actually was.
        touchdown = stance & ~self.prev_stance
        liftoff = ~stance & self.prev_stance
        self.p_td = np.where(touchdown[:, None], self.p_cmd, self.p_td)
        self.p_lo = np.where(liftoff[:, None], self.p_cmd, self.p_lo)
        self.prev_stance = stance

        # 8. Foot paths.
        st_xy = self.p_td[:, :2] - v_leg * (s_st[:, None] * t_st)
        st_xy = nominal[:, :2] + np.clip(st_xy - nominal[:, :2], -reach, reach)
        sw_xy = self.p_lo[:, :2] + (foothold - self.p_lo[:, :2]) * _min_jerk(s_sw)[:, None]
        xy = np.where(stance[:, None], st_xy, sw_xy)
        lift = p.swing_height * self.activity * _lift_profile(s_sw)
        z = nominal[:, 2] + np.where(stance, 0.0, lift)
        foot = np.concatenate((xy, z[:, None]), axis=-1)
        self.p_cmd = foot

        # 9. Attitude levelling.
        if projected_gravity is not None:
            up = -np.asarray(projected_gravity, dtype=float)
            up = up / np.linalg.norm(up)
            axis = np.array([-up[1], up[0], 0.0])
            sin_t = np.linalg.norm(axis)
            angle = math.atan2(sin_t, up[2])
            axis = axis / max(sin_t, 1e-9)
            foot = _rotate(HIP + foot, axis, -p.k_att * angle) - HIP

        # 9b. Ground-relative swing + height hold.
        if joint_pos is not None and projected_gravity is not None:
            up = -np.asarray(projected_gravity, dtype=float)
            up = up / np.linalg.norm(up)
            depth = (forward_kinematics(joint_pos) + HIP) @ up
            w = stance + (1.0 - self.activity) * (~stance)
            ground = float((depth * w).sum() / max(w.sum(), 1e-6))
            if dt > 0:
                self.h_dot += 0.3 * ((self.ground - ground) / dt - self.h_dot)
            self.ground = ground
            h_err = p.height - (-self.ground)
            self.h_corr = float(np.clip(self.h_corr + p.k_h * h_err * dt, -p.h_corr_max, p.h_corr_max))
            along = (HIP + foot) @ up
            want = self.ground + lift
            delta = np.where(stance, 0.0, np.maximum(want - along, 0.0)) * p.ground_swing
            foot = foot + up[None, :] * delta[:, None]

        # 10. IK.
        q = inverse_kinematics(foot)

        # 11. Support: load share ramps over the ends of stance; all legs when standing.
        ramp = np.clip(np.minimum(s_st, 1.0 - s_st) / p.ff_ramp, 0.0, 1.0)
        act = self.activity
        share = act * np.where(stance, ramp, 0.0) + (1.0 - act)
        forced = p.force_mode > 0.5
        have = projected_gravity is not None and joint_pos is not None and joint_vel is not None and ang_vel_b is not None
        if have and forced:
            self.tau_ff = self._stance_forces(share, v_xy, wz, projected_gravity, joint_pos, ang_vel_b, dt)
        else:
            self.tau_ff = np.zeros((4, 3))
            forced = False

        # 11b. Position-only alternative (force_mode 0).
        if not forced:
            n_support = 4.0 * (act * p.duty + (1.0 - act))
            load = p.ff_mass * 9.81 * share / n_support
            force = np.zeros_like(foot)
            force[:, 2] = -load
            tau = (np.swapaxes(jacobian(q), -1, -2) @ force[..., None])[..., 0]
            tau[:, 0] *= p.ff_hip
            q = q + tau / p.ff_kp

        return np.minimum(np.maximum(q, Q_LO), Q_HI)
