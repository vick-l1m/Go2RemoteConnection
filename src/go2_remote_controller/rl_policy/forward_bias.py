"""The forward-bias command mapping, in one place for both sim and the robot.

A forward-biased policy is trained to redirect by *turning*, not by strafing. That bias
is not a reward -- it is a property of the command distribution the policy is trained
against: a large yaw command never arrives together with a large forward speed, because
forward speed is gated by how far the body still has to turn.

    gate(err) = max(0, cos(err))          err = heading error, radians

    err      0 deg   30 deg   45 deg   60 deg   90 deg   120 deg
    gate     1.00     0.87     0.71     0.50     0.00     0.00

Two callers must agree on that formula or the policy is fed commands it never saw:

* **Training** -- :class:`go2_training.mdp.commands.ForwardBiasedVelocityCommand` samples
  a world-frame heading target, measures ``err`` against the base's yaw, and gates the
  sampled forward speed with it. It applies the formula in torch, over 4096 envs at once.
* **Deployment** -- the RL bridge node turns the operator's left-stick vector into the
  same three numbers. There is no heading target and no yaw estimate on that side: the
  stick vector is *already* expressed in the body frame, so ``atan2(vy, vx)`` **is** the
  heading error. That is why the robot needs no IMU yaw integration and no latch --
  holding the stick left reads as a standing heading error, so the robot keeps turning
  left until the stick is centred.

:func:`forward_gate` is the scalar reference implementation of the shared formula, and
``tests/test_forward_bias.py`` asserts the torch path in the command term agrees with it
across the full angle range. Keep them in sync by changing this file, not that one.

Pure Python -- ``math`` only, no torch, no Isaac -- so the deployment side (ROS 2 Humble,
Python 3.10, no Isaac Lab) can import it as-is.
"""

from __future__ import annotations

import math

__all__ = ["forward_gate", "stick_to_velocity_command", "wrap_to_pi"]


def wrap_to_pi(angle: float) -> float:
    """Wrap an angle in radians to ``[-pi, pi]`` (scalar ``isaaclab.utils.math.wrap_to_pi``)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def forward_gate(heading_error: float) -> float:
    """Fraction of the commanded speed that survives a heading error of ``heading_error``.

    ``max(0, cos(err))``: full speed when already facing the target, zero at 90 degrees or
    beyond (turn in place), a smooth arc in between. Clamped at zero rather than allowed
    negative on purpose -- a target behind the robot must make it *turn around*, not walk
    backwards into it. Reverse is a separate, explicitly commanded mode (see
    :func:`stick_to_velocity_command` and the command term's ``rel_reverse_envs``).
    """
    return max(0.0, math.cos(heading_error))


def stick_to_velocity_command(
    stick_x: float,
    stick_y: float,
    yaw_stick: float = 0.0,
    *,
    forward_max: float,
    reverse_max: float,
    yaw_max: float,
    stiffness: float = 1.0,
) -> tuple[float, float, float]:
    """Map an operator's sticks to the ``(vx, vy, wz)`` a forward-biased policy expects.

    This is the deployment half of the contract above -- what the policy node should feed
    the observation instead of passing ``Twist`` through unchanged. Stateless: no yaw
    estimate, no latched heading target, so the operator keeps direct authority (centre
    the stick and the turn stops immediately).

    Args:
        stick_x: forward/back component of the move stick, in m/s, body frame (``linear.x``).
        stick_y: left/right component of the move stick, in m/s, body frame (``linear.y``).
        yaw_stick: the separate yaw request, in rad/s (``angular.z``). Converted to an
            equivalent heading error and *added* to the one the move stick asks for, rather
            than added to the returned yaw -- see below.
        forward_max: largest forward speed the policy was trained on (deploy.yaml
            ``commands.base_velocity.ranges.lin_vel_x`` upper bound).
        reverse_max: largest *magnitude* of trained reverse speed, as a positive number.
        yaw_max: yaw-rate clip, matching the trained ``ang_vel_z`` bound.
        stiffness: heading-error -> yaw-rate gain, matching the trained
            ``heading_control_stiffness``.

    Returns:
        ``(vx, vy, wz)``, with ``vy`` always 0.0 -- a forward-biased policy has no lateral
        channel, and feeding it one would be an observation it was never trained on.

    Behaviour, for a stick pushed to full deflection:

    ==================  ===============  =========  ==========================
    stick direction     heading error    vx         wz
    ==================  ===============  =========  ==========================
    forward             0 deg            forward    0            walks straight
    forward-left 45     +45 deg          0.71x      +0.79 x k    arcs left
    left                +90 deg          0.0        +yaw_max     turns in place
    back                0 deg (folded)   -reverse   0            backs up straight
    back-left 135       -45 deg          -0.71x     -0.79 x k    backs up curving
    ==================  ===============  =========  ==========================

    **Why the yaw stick enters as a heading error.** Adding it to the returned ``wz``
    instead would break the one property the policy was trained on: that a large yaw
    command never arrives with a fast walk. Full forward plus full yaw would hand it
    ``(1.2, 0, 1.2)``, a pair the gate never produces in training, which is exactly the
    kind of out-of-distribution command that makes a transferred policy stumble. Entering
    through the error instead keeps the pair on the trained curve -- the robot arcs, with
    forward speed cut by the same cosine -- and a yaw stick pushed on its own still spins
    in place, because ``speed`` is then zero and only ``wz`` survives.
    """
    speed = math.hypot(stick_x, stick_y)
    reverse = False

    if speed < 1e-6:
        # No direction asked for: the yaw stick alone, which is a turn in place.
        error = 0.0
    else:
        theta = math.atan2(stick_y, stick_x)
        if abs(theta) <= math.pi / 2.0:
            # Stick points into the forward half-plane: turn toward it and walk forward.
            error = theta
        else:
            # Stick points behind: back up ALONG it rather than spinning 180 degrees to
            # face it. The relevant alignment error is then to the robot's reverse axis,
            # which is what folding by pi measures -- a straight-back stick gives error 0,
            # hence no yaw, which is what an operator expects from pulling straight back.
            error = wrap_to_pi(theta - math.pi)
            reverse = True

    # The operator's yaw request, as the heading error that would produce it. Both
    # contributions are then subject to the same gate, so the returned pair stays on the
    # curve the policy was trained against.
    #
    # Clamped to the trained envelope BEFORE it becomes an angle, and that order is
    # load-bearing: an out-of-envelope request (a UI whose yaw axis scales past yaw_max)
    # would otherwise convert to more than pi radians of error and wrap to the OPPOSITE
    # sign -- full right stick spinning the robot left. Clamping first also keeps the sum
    # below pi (|error| <= pi/2 after the fold, plus at most yaw_max/stiffness), so the
    # wrap below can only ever be a no-op.
    yaw_request = max(-yaw_max, min(yaw_max, yaw_stick))
    error = wrap_to_pi(error + yaw_request / stiffness)

    if reverse:
        forward = max(-speed * forward_gate(error), -reverse_max)
    else:
        forward = min(speed * forward_gate(error), forward_max)

    yaw = max(-yaw_max, min(yaw_max, stiffness * error))
    return forward, 0.0, yaw
