"""rl_policy/distance_move.py: leg odometry + the move state machine, no robot needed."""

import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "rl_policy"))

from classical_gait import HIP, inverse_kinematics  # noqa: E402
import distance_move as dm  # noqa: E402

DT = 0.02
FEET_HOME = np.array([[0.0, 0.0955, -0.30], [0.0, -0.0955, -0.30],
                      [0.0, 0.0955, -0.30], [0.0, -0.0955, -0.30]]) + HIP
STANCE = [50.0, 50.0, 50.0, 50.0]


def quat_yaw(yaw):
    return (math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2))


def leg_to_sdk(a):
    return np.asarray(a).reshape(4, 3)[dm.SDK_TO_LEG].reshape(12)   # the swap is its own inverse


def joints_for_base(offset_b):
    """Joint angles (SDK order) with every foot pinned while the base sits at ``offset_b``."""
    return leg_to_sdk(inverse_kinematics(FEET_HOME - offset_b - HIP))


def drive(move, yaw=0.0, v_scale=1.0, steps=2000, stop_when_idle=True):
    """Walk the synthetic robot at whatever the move commands (times ``v_scale``)."""
    x, t = 0.0, 0.0
    q_prev = joints_for_base(np.zeros(3))
    for _ in range(steps):
        vx, _wz = move.command(yaw)
        # Feet stay pinned for a 0.2 m stride, then "step": reset the stance reference.
        x_new = x + v_scale * vx * DT
        local = (x_new % 0.2) if x_new >= 0 else -((-x_new) % 0.2)
        local_prev = local - v_scale * vx * DT
        q_prev = joints_for_base(np.array([local_prev, 0.0, 0.0]))
        q = joints_for_base(np.array([local, 0.0, 0.0]))
        dq = (q - q_prev) / DT
        x, t = x_new, t + DT
        move.update(t, DT, q, dq, [0.0, 0.0, 0.0], quat_yaw(yaw), STANCE)
        if stop_when_idle and not move.measuring:
            break
    return x, t


@pytest.mark.parametrize("meters", [1.0, -0.6, 2.5])
def test_reaches_goal_both_directions(meters):
    move = dm.DistanceMove()
    move.start(meters, 0.3, 0.0, 0.0)
    x, _ = drive(move)
    assert move.state == dm.DONE
    assert move.progress == pytest.approx(meters, abs=0.03)
    assert x == pytest.approx(meters, abs=0.03)       # the odometry matched the true walk


def test_command_sign_and_heading_hold():
    move = dm.DistanceMove()
    move.start(-1.0, 0.25, 0.0, 0.0)
    vx, wz = move.command(0.2)                 # drifted 0.2 rad left -> steer right
    assert vx == pytest.approx(-0.25)
    assert wz == pytest.approx(-0.2)
    assert move.command(0.2, hold_heading=False) == (pytest.approx(-0.25), 0.0)
    assert move.command(3.0)[1] == pytest.approx(-dm.HEADING_RATE_MAX)


def test_progress_is_along_start_heading():
    """Walking at a 90-degree yaw counts body-forward motion, not world x."""
    move = dm.DistanceMove()
    move.start(0.5, 0.3, math.pi / 2, 0.0)
    drive(move, yaw=math.pi / 2)
    assert move.state == dm.DONE
    assert move.progress == pytest.approx(0.5, abs=0.03)


def test_timeout_when_stuck():
    move = dm.DistanceMove()
    move.start(1.0, 0.3, 0.0, 0.0)
    drive(move, v_scale=0.0)                    # commanded, but the robot never moves
    assert move.state == dm.TIMEOUT
    assert "timed out" in move.why


def test_cancel_stops_command():
    move = dm.DistanceMove()
    move.start(1.0, 0.3, 0.0, 0.0)
    assert move.cancel("joystick override", 0.1)
    assert move.state == dm.CANCELLED
    assert move.command(0.0) == (0.0, 0.0)
    assert not move.cancel("again", 0.2)


def test_no_contacts_holds_last_velocity():
    assert dm.stance_velocity(np.zeros((4, 3)), np.zeros((4, 3)), np.zeros(3), [False] * 4) is None
