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

__all__ = [
    "CommandAnchor",
    "forward_gate",
    "stick_to_cone_command",
    "stick_to_omni_command",
    "stick_to_velocity_command",
    "wrap_to_pi",
    "yaw_from_quat",
]


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


def stick_to_cone_command(
    stick_x: float,
    stick_y: float,
    yaw_stick: float = 0.0,
    *,
    forward_max: float,
    cone_half_angle: float,
    lateral_max: float,
    yaw_max: float,
) -> tuple[float, float, float]:
    """Project an operator's sticks into the FORWARD CONE a cone-trained policy expects.

    The deployment half of :class:`go2_training.mdp.ForwardConeVelocityCommand`. That term
    samples uniformly inside a wedge; this projects an arbitrary stick onto the same wedge,
    so every command the policy sees on the robot is one its training distribution covers.

    Projection, not clamping, and the difference is the whole point. deploy.yaml reports a
    per-axis BOX, and a box clamp admits its corners: ``(vx, vy) = (0.0, 0.57)`` has both
    axes in range and is a 90 degree strafe the policy was never trained on. Projecting
    keeps the operator's intent -- the *nearest* direction the policy actually knows --
    rather than producing a command that is in-range on paper and out-of-distribution in
    fact.

    Args:
        stick_x: forward/back component of the move stick, m/s, body frame (``linear.x``).
        stick_y: left/right component of the move stick, m/s, body frame (``linear.y``).
        yaw_stick: the separate yaw request, rad/s (``angular.z``). Passed through, clipped.
        forward_max: the cone's radius -- the trained forward speed ceiling (deploy.yaml
            ``commands.base_velocity.ranges.lin_vel_x`` upper bound).
        cone_half_angle: half-angle of the trained wedge, radians, off the body +x axis.
        lateral_max: the trained ``lin_vel_y`` bound, as a positive number. A backstop only:
            the cone already implies ``forward_max * sin(cone_half_angle)``, and this keeps
            the result inside the exported envelope if the two ever disagree.
        yaw_max: yaw-rate clip, matching the trained ``ang_vel_z`` bound.

    Returns:
        ``(vx, vy, wz)`` with ``vx >= 0`` and ``|atan2(vy, vx)| <= cone_half_angle``.

    Behaviour, for a stick pushed to full deflection, at a 45 degree cone:

    ==================  ==========================================================
    stick direction     result
    ==================  ==========================================================
    forward             full speed straight ahead -- unchanged
    forward-left 30     full speed at 30 deg -- inside the cone, passed through
    left (90 deg)       full speed at 45 deg: the nearest direction in the cone
    back                STOPPED (vx = vy = 0). The cone has no reverse; backing up
                        is the one direction with no depth coverage at all, so the
                        honest answer is to refuse it rather than fake it.
    back + yaw stick    turns in place -- yaw survives, the linear command does not
    ==================  ==========================================================

    **Yaw is passed through, not folded into the direction** -- unlike the forward-biased
    mapping, where a lateral stick *is* a turn request. Here the lateral channel is real:
    the policy can strafe diagonally, so the move stick keeps meaning "travel this way" and
    the yaw stick keeps meaning "turn". A cone-trained policy saw the two sampled
    independently, so passing them through independently is what matches its training.

    Rejecting a rearward stick rather than folding it to a turn is deliberate for the same
    reason: the training distribution contains no rearward command at all, so there is no
    "nearest trained direction" behind the robot. An operator who wants to turn around uses
    the yaw stick, which still works at a standstill.
    """
    wz = max(-yaw_max, min(yaw_max, yaw_stick))

    speed = math.hypot(stick_x, stick_y)
    if speed < 1e-6:
        return 0.0, 0.0, wz

    theta = math.atan2(stick_y, stick_x)
    if abs(theta) > math.pi / 2.0:
        # Rearward: outside the half-plane the cone lives in. Stop the linear command and
        # leave the yaw stick alone, so the operator can still spin to face where they want
        # to go and then drive forward.
        return 0.0, 0.0, wz

    theta = max(-cone_half_angle, min(cone_half_angle, theta))
    speed = min(speed, forward_max)

    vx = speed * math.cos(theta)
    vy = speed * math.sin(theta)
    # Backstop against an exported lateral bound tighter than the cone implies; clamping vy
    # alone would rotate the direction, so scale both and keep the bearing.
    if lateral_max > 0.0 and abs(vy) > lateral_max:
        scale = lateral_max / abs(vy)
        vx *= scale
        vy *= scale
    return vx, vy, wz


def stick_to_omni_command(
    vx: float,
    vy: float,
    wz: float,
    speed_max: float,
    yaw_max: float,
) -> tuple[float, float, float]:
    """Project the move stick onto the trained omni DISC (``style: omni_mixed``).

    Such a policy (``go2_training.mdp.MixedDirectionVelocityCommand``) trained on every
    direction at a FIXED speed, so the trained envelope is a disc of radius
    ``speed_max`` -- which the exported box cannot express. A per-axis clamp fails the
    same two ways the cone taught us: the box corner (full x AND full y, a 0.42 m/s
    diagonal at speed_max 0.3) is a speed the policy never saw, and a fixed-speed export
    makes ``lin_vel_x`` the range ``[0.3, 0.3]``, which a per-axis clamp turns into
    "always drive forward" and the robot can never stand (the 2026-10-28 cone bug).
    Here: keep the stick's DIRECTION, cap its magnitude at ``speed_max``; a centred
    stick stays (0, 0) -- standing is in the trained distribution (``rel_standing_envs``).
    Yaw is an independent box, clamped.
    """
    speed = math.hypot(vx, vy)
    if speed_max > 0.0 and speed > speed_max:
        scale = speed_max / speed
        vx *= scale
        vy *= scale
    wz = max(-yaw_max, min(yaw_max, wz))
    return vx, vy, wz


def yaw_from_quat(w: float, x: float, y: float, z: float) -> float:
    """Heading (yaw about world z), radians, from a ``(w, x, y, z)`` quaternion."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class CommandAnchor:
    """The deployment half of ``world_anchored=True`` (``TerrainScaledVelocityCommand``).

    In training, an anchored command is sampled in the body frame and then pinned to the
    WORLD heading the body had at that moment; the anchor turns only with the commanded
    yaw rate, and every step the pinned ``(vx, vy)`` is re-expressed in the current body
    frame (``go2_training.utils.forward_cone.rotate_to_body``). A robot that yaws without
    being told to SEES the command swing the other way -- drift left 20 deg and "forward"
    reads as forward-right. That swing is the policy's only heading feedback, so it learns
    to lean on it.

    A node that forwards the raw body-frame stick removes the feedback: the command never
    swings, and the policy's small turning bias goes uncorrected. The climb30 finetune spun
    left at ~1 rad/s on the robot (session 20261028-035126) with the stick centred on yaw,
    and went straight only while a rightward stick happened to stand in for the swing.

    The stick plays the part of the resample. A change of the linear command by more than
    ``retarget_tol`` m/s re-pins it at the current heading -- "forward" means where the nose
    points when the stick moves -- and a held stick keeps its anchor, so drift shows up as
    the same swing the policy was trained on. The yaw stick never re-pins: as in training,
    it advances the anchor heading instead, so commanded turns are not read as drift.

    HEADING HOLD (``hold_gain`` > 0, off by default). The swing alone was not enough on the
    robot: the climb30 finetune still turned ~90 deg left within 1.5 s of a push and kept
    spinning at zero command (2026-10-28), while in Isaac it only wanders (~0.03 rad/s with
    no correction). Every Isaac eval of it ran with a heading hold, ``wz = 1.0 * (target -
    heading)`` clipped to +-1 rad/s (eval_fixed_rung.py), so that is the law used here:
    while the yaw stick is centred the returned ``wz`` steers back to the anchor heading.
    The hold's own correction does not advance the anchor -- only the operator's yaw does --
    so the hold and the swing pull toward the same heading. Releasing the yaw stick re-pins
    at the current heading, so the robot holds where the operator stopped turning it.
    ``max_swing`` caps the swing (a cone policy never trained on a command outside its
    cone; uncapped, a 90 deg drift became a pure sideways command and froze it).
    """

    def __init__(self, retarget_tol: float = 0.08, hold_gain: float = 0.0,
                 hold_max: float = 1.0, max_swing: float | None = None,
                 yaw_deadband: float = 0.05):
        self.retarget_tol = retarget_tol
        self.hold_gain = hold_gain
        self.hold_max = hold_max
        self.max_swing = max_swing
        self.yaw_deadband = yaw_deadband
        self._was_turning = False
        self.reset()

    def reset(self) -> None:
        """Forget the anchor; the next :meth:`apply` re-pins at the current heading."""
        self._cmd: tuple[float, float] | None = None
        self._heading = 0.0

    def apply(
        self, vx: float, vy: float, wz: float, yaw: float, dt: float
    ) -> tuple[float, float, float]:
        """Anchor the body-frame ``(vx, vy, wz)`` and return it in the current body frame.

        Args:
            vx, vy, wz: the command as shaped for the policy (cone projection, clip, ...).
            yaw: the base's current heading, radians.
            dt: control period, seconds -- the anchor turns by ``wz * dt`` each call.

        Same order as training: re-pin on a new command, then advance the anchor by the
        commanded yaw, then rotate. The rotated ``(vx, vy)`` can leave the trained box or
        cone (training's observation did too) unless ``max_swing`` caps it. With
        ``hold_gain`` > 0 and the yaw stick centred, ``wz`` is the heading hold instead.
        """
        turning = abs(wz) > self.yaw_deadband
        if self._was_turning and not turning:
            self.reset()                  # yaw stick released: hold the heading it left
        self._was_turning = turning
        if self._cmd is None or math.hypot(vx - self._cmd[0], vy - self._cmd[1]) > self.retarget_tol:
            self._cmd = (vx, vy)
            self._heading = yaw
        self._heading = wrap_to_pi(self._heading + wz * dt)
        err = wrap_to_pi(self._heading - yaw)
        swing = err if self.max_swing is None else max(-self.max_swing, min(self.max_swing, err))
        c, s = math.cos(swing), math.sin(swing)
        ax, ay = self._cmd
        if self.hold_gain > 0.0 and not turning:
            wz = max(-self.hold_max, min(self.hold_max, self.hold_gain * err))
        return c * ax - s * ay, s * ax + c * ay, wz
