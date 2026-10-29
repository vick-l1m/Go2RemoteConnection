"""'Move X metres' for the RL page: walk a set distance forward (or back) and stop.

Pure numpy, no SDK, no ROS (unit tested without either). ``go2_rl_policy_node`` owns one
:class:`DistanceMove`, feeds it every 50 Hz control tick, and while it is driving takes
its ``(vx, wz)`` in place of the joystick -- through the policy's own command shaping
when RL is engaged, or as ``SportClient.Move`` when the sport service owns the robot.

Why the distance is measured here and not from ``/utlidar/robot_odom`` (what the sport
``move_forward_meters_node`` uses): while an RL policy is engaged the Go2 publishes no
odometry at all -- ``rt/sportmodestate`` and the lidar odom stop
(``docs/measurements/go2_localisation_while_engaged.md``). What survives is LowState, so
the distance comes from leg odometry: a stance foot does not move in the world, so

    v_base = -(J(q) q_dot + omega x p_foot)        averaged over the stance feet

(the same estimate as ``go2_perception/leg_odometry.py`` and the classical gait's
Raibert feedback; robot bags: 3-6 cm median drift over 2 s). The same estimator runs in
sport mode too, so both modes stop on the same measurement.

Progress is the displacement along the heading the robot had when the move started, so
a sideways drift does not count toward the goal. The move holds that heading: it returns
a yaw-rate correction the node applies unless the policy's own world anchor already does.
"""

from __future__ import annotations

import math

import numpy as np

from classical_gait import HIP, forward_kinematics, jacobian

MOVE_SPEED = 0.3          # m/s, default walking speed (capped by the policy's envelope)
MIN_SPEED = 0.05          # m/s, below this a policy cannot do the move at all
MAX_DISTANCE = 5.0        # m, per request (matches the sport "Move X meters" box)
GOAL_TOL = 0.02           # m, "arrived" when this close
CONTACT_FORCE_MIN = 20.0  # LowState.foot_force (~N) for a stance foot; = leg_odom_node on the robot
HEADING_GAIN = 1.0        # wz = gain * heading error (the eval / CommandAnchor law)
HEADING_RATE_MAX = 0.5    # rad/s cap on that correction
TIMEOUT_FACTOR = 2.0      # give up after this many times the nominal walk time ...
TIMEOUT_MARGIN = 3.0      # ... plus this many seconds (stuck on an obstacle, policy stalled)
SETTLE_TIME = 1.0         # s of odometry after the stop, so the reported distance includes the coast
TELEOP_OVERRIDE = 0.05    # |joystick| on any axis that cancels a running move

# LowState legs are in SDK order (FR, FL, RR, RL); the kinematics use FL, FR, RL, RR.
SDK_TO_LEG = np.array([1, 0, 3, 2])

IDLE, DRIVING, SETTLING, DONE, CANCELLED, TIMEOUT = (
    "idle", "driving", "settling", "done", "cancelled", "timeout")


def quat_to_rot(w: float, x: float, y: float, z: float) -> np.ndarray:
    """Body -> world rotation matrix from a ``(w, x, y, z)`` quaternion."""
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def stance_velocity(q_leg: np.ndarray, dq_leg: np.ndarray, gyro, contacts) -> np.ndarray | None:
    """Base linear velocity in the body frame from the feet in contact, or None if none are.

    ``q_leg``/``dq_leg`` are ``(4, 3)`` in leg order (FL, FR, RL, RR x hip, thigh, calf).
    """
    contacts = np.asarray(contacts, dtype=bool)
    if not contacts.any():
        return None
    q_leg = np.asarray(q_leg, dtype=np.float64)
    p_foot = forward_kinematics(q_leg) + HIP
    v_rel = (jacobian(q_leg) @ np.asarray(dq_leg, dtype=np.float64)[..., None])[..., 0]
    w = np.broadcast_to(np.asarray(gyro, dtype=np.float64), p_foot.shape)
    return -(v_rel + np.cross(w, p_foot))[contacts].mean(0)


def contacts_from_sdk_force(foot_force_sdk, threshold: float = CONTACT_FORCE_MIN) -> np.ndarray:
    """Stance mask in leg order from ``LowState.foot_force`` (SDK order)."""
    return np.asarray(foot_force_sdk, dtype=np.float64)[SDK_TO_LEG] > threshold


def sdk_to_leg(v12) -> np.ndarray:
    """12-vector in LowState motor order -> ``(4, 3)`` leg order."""
    return np.asarray(v12, dtype=np.float64).reshape(4, 3)[SDK_TO_LEG]


def _wrap(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class DistanceMove:
    """One distance goal at a time: ``start`` -> ``update`` every tick -> ``command``.

    ``state`` is ``idle`` until the first goal, ``driving`` while it walks, ``settling``
    for :data:`SETTLE_TIME` after it stops (odometry keeps running so the final distance
    includes the coast), then ``done``. ``cancel`` and the timeout end it early.
    """

    def __init__(self):
        self.state = IDLE
        self.target = 0.0          # signed metres requested
        self.speed = 0.0           # m/s magnitude
        self.progress = 0.0        # signed metres along the start heading
        self.why = ""
        self._dir = 1.0
        self._yaw0 = 0.0
        self._heading = np.array([1.0, 0.0])
        self._t0 = 0.0
        self._t_end = 0.0
        self._deadline = 0.0
        self._v_world = np.zeros(3)

    @property
    def driving(self) -> bool:
        return self.state == DRIVING

    @property
    def measuring(self) -> bool:
        return self.state in (DRIVING, SETTLING)

    def start(self, meters: float, speed: float, yaw: float, t: float) -> None:
        """Begin a move of ``meters`` (negative = backwards) at ``speed`` m/s from heading ``yaw``."""
        self.target = float(meters)
        self.speed = float(speed)
        self._dir = 1.0 if meters >= 0.0 else -1.0
        self._yaw0 = float(yaw)
        self._heading = np.array([math.cos(yaw), math.sin(yaw)])
        self.progress = 0.0
        self._v_world = np.zeros(3)
        self._t0 = t
        self._deadline = t + TIMEOUT_FACTOR * abs(meters) / max(speed, MIN_SPEED) + TIMEOUT_MARGIN
        self.why = ""
        self.state = DRIVING

    def cancel(self, why: str, t: float) -> bool:
        """Stop driving now. Returns True if a move was driving (i.e. something changed)."""
        if self.state == DRIVING:
            self.state, self.why, self._t_end = CANCELLED, why, t
            return True
        if self.state == SETTLING:       # already stopped; just finish the report early
            self.state = DONE
        return False

    def update(self, t: float, dt: float, q_sdk, dq_sdk, gyro, quat_wxyz, foot_force_sdk) -> None:
        """Integrate one tick of leg odometry and advance the state machine.

        Joint and foot arrays are straight from LowState (SDK order).
        """
        if not self.measuring:
            return
        v_b = stance_velocity(sdk_to_leg(q_sdk), sdk_to_leg(dq_sdk), gyro,
                              contacts_from_sdk_force(foot_force_sdk))
        if v_b is not None:                   # all feet in the air: hold the last velocity
            self._v_world = quat_to_rot(*quat_wxyz) @ v_b
        self.progress += float(self._v_world[:2] @ self._heading) * dt
        if self.state == DRIVING:
            if self._dir * self.progress >= abs(self.target) - GOAL_TOL:
                self.state, self._t_end = SETTLING, t
            elif t >= self._deadline:
                self.state, self._t_end = TIMEOUT, t
                self.why = f"timed out after {t - self._t0:.1f} s"
        elif self.state == SETTLING and t - self._t_end >= SETTLE_TIME:
            self.state = DONE

    def command(self, yaw: float, hold_heading: bool = True) -> tuple[float, float]:
        """``(vx, wz)`` body-frame command while driving, else zeros.

        ``wz`` steers back to the start heading; pass ``hold_heading=False`` when the
        policy's own world anchor already holds it (a second hold would fight it).
        """
        if self.state != DRIVING:
            return 0.0, 0.0
        wz = 0.0
        if hold_heading:
            wz = HEADING_GAIN * _wrap(self._yaw0 - yaw)
            wz = max(-HEADING_RATE_MAX, min(HEADING_RATE_MAX, wz))
        return self._dir * self.speed, wz

    def status(self) -> dict:
        return {
            "state": self.state,
            "target": round(self.target, 3),
            "travelled": round(self.progress, 3),
            "speed": round(self.speed, 3),
            "why": self.why,
        }
