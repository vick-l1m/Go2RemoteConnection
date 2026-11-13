"""NIST run records for real stepfield runs -- pure numpy, no ROS (unit tested without it).

The real-robot half of the evaluation upgrade (``docs/eval_upgrade_plan_NIST_Agarwal.md`` in
the outer repo, section 7.1): one record per engaged run, in the SAME field names as the sim
evaluators' per-episode records (``training/go2_training/utils/episode_log.py``), so a real
row drops straight into ``scores.csv`` with ``sim = real``.

What is measured, and how:

* ``outcome``  -- the OPERATOR's call (pass / head / side / stuck / other), given on the
  command line. Without one the record says what odometry suggests (``outcome_source`` =
  ``auto``): pass if the base got past the pass line, other if the body tipped past
  ``FALL_TILT_DEG``, else stuck. Use the operator's call in the thesis.
* distance -- while a policy is engaged the Go2 publishes no odometry, so the track is leg
  odometry from LowState (``rl_policy/distance_move.py``: stance-foot velocity, the estimator
  the node's "move X m" uses; 3-6 cm median drift over 2 s on robot bags). ``x_reached_m`` is
  the furthest displacement ALONG THE HEADING AT THE COMMAND ONSET, from where the robot was
  then (the protocol's spawn, 1.2 m before row 0). Over a 2-6 m lane expect a few percent of
  error; it is an estimate, the outcome is the operator's.
* ``time_to_pass_s`` -- from the command onset to the base crossing the pass line. On the lab
  lanes the pass line is the far edge of the last row plus ``RUN_OUT_MIN`` (0.6 m: all four
  feet across); the sims use the tile exit 7.8 m from the spawn, which the lab floor does not
  have, so real and sim pass times are not the same distance -- compare speeds, not times.
* ``head_touches`` is not measurable on the robot (no head contact sensor): null.
"""

from __future__ import annotations

import math

import numpy as np

from distance_move import contacts_from_sdk_force, quat_to_rot, sdk_to_leg, stance_velocity

FIELDS = (
    "run_id", "train_seed", "eval_seed", "sim", "lane_id", "tile_id", "episode_idx",
    "outcome", "time_to_pass_s", "time_to_end_s", "x_reached_m", "forward_dist_m",
    "lane_fraction_completed", "falls", "head_touches", "mean_speed_mps",
    "drift_heading_m_per_m", "drift_crab_m_per_m",
)
OUTCOMES = ("pass", "head", "side", "stuck", "other")
#: Same defaults as the final-eval layout (eval_stepfields_layout.RUN_OUT_MIN, RUN_IN).
RUN_OUT_MIN = 0.60
RUN_IN = 1.20
CELL = 0.30
#: Body tilt (deg) past which an auto outcome calls the run a fall.
FALL_TILT_DEG = 45.0
#: Forward command (m/s) that marks the start of the scored walk.
CMD_ONSET = 0.05
MAX_DT = 0.1


def yaw_of(R: np.ndarray) -> float:
    return math.atan2(R[1, 0], R[0, 0])


def track(t, q_sdk, dq_sdk, gyro, quat_wxyz, foot_force_sdk) -> dict:
    """Leg-odometry track over a window of LowState samples (arrays, first axis = time).

    Returns per-sample arrays: ``x`` / ``y`` along / across the heading of the FIRST sample,
    ``tilt_deg`` (angle between body z and world z), and the cumulative ``fwd`` (forward
    body-frame distance), ``dy_heading`` / ``dy_crab`` (lateral drift from pointing off the
    start heading / from walking sideways), as eval_fixed_rung sums them in the sims."""
    t = np.asarray(t, float)
    n = len(t)
    pos = np.zeros((n, 2))
    fwd = np.zeros(n)
    dyh = np.zeros(n)
    dyc = np.zeros(n)
    tilt = np.zeros(n)
    yaw0 = None
    p = np.zeros(2)
    f = dh = dc = 0.0
    for i in range(n):
        R = quat_to_rot(*quat_wxyz[i])
        tilt[i] = math.degrees(math.acos(max(-1.0, min(1.0, R[2, 2]))))
        yaw = yaw_of(R)
        if yaw0 is None:
            yaw0 = yaw
        if i > 0:
            dt = t[i] - t[i - 1]
            v_b = stance_velocity(sdk_to_leg(q_sdk[i]), sdk_to_leg(dq_sdk[i]), np.asarray(gyro[i], float),
                                  contacts_from_sdk_force(foot_force_sdk[i]))
            if v_b is not None and 0.0 < dt <= MAX_DT:
                v_w = R @ np.asarray(v_b, float)
                c, s = math.cos(-yaw0), math.sin(-yaw0)
                p = p + dt * np.array([c * v_w[0] - s * v_w[1], s * v_w[0] + c * v_w[1]])
                rel = yaw - yaw0
                f += float(v_b[0]) * dt
                dh += float(v_b[0]) * math.sin(rel) * dt
                dc += float(v_b[1]) * math.cos(rel) * dt
        pos[i] = p
        fwd[i], dyh[i], dyc[i] = f, dh, dc
    return {"t": t, "x": pos[:, 0], "y": pos[:, 1], "tilt_deg": tilt, "fwd": fwd,
            "dy_heading": dyh, "dy_crab": dyc}


def lane_geometry(rows: int, run_in: float = RUN_IN, cell: float = CELL) -> tuple[float, float]:
    """``(lane end x, pass line x)`` from the spawn for a lane of ``rows`` rows."""
    end = run_in + rows * cell
    return end, end + RUN_OUT_MIN


def run_record(tr: dict, *, lane_id: str, lane_end_x: float, pass_x: float, run_id: str,
               episode_idx: int, outcome: str | None = None, t_onset: float | None = None,
               t_end: float | None = None) -> dict:
    """One record from a :func:`track`. The scored window is ``[t_onset, t_end]`` (default:
    the whole track); positions are re-zeroed at the onset (the spawn)."""
    t = tr["t"]
    lo = 0 if t_onset is None else int(np.searchsorted(t, t_onset))
    hi = len(t) if t_end is None else int(np.searchsorted(t, t_end, side="right"))
    lo = min(lo, max(0, hi - 1))
    sl = slice(lo, hi)
    x = tr["x"][sl] - tr["x"][lo]
    tt = t[sl] - t[lo]
    x_reached = float(x.max()) if len(x) else 0.0
    crossed = np.flatnonzero(x >= pass_x)
    t_cross = float(tt[crossed[0]]) if crossed.size else None
    source = "operator"
    if outcome is None:
        source = "auto"
        if crossed.size:
            outcome = "pass"
        elif len(tr["tilt_deg"][sl]) and float(tr["tilt_deg"][sl].max()) > FALL_TILT_DEG:
            outcome = "other"
        else:
            outcome = "stuck"
    if outcome not in OUTCOMES:
        raise ValueError(f"outcome {outcome!r} not in {OUTCOMES}")
    dur = float(tt[-1]) if len(tt) else 0.0
    dist = float(tr["fwd"][hi - 1] - tr["fwd"][lo]) if hi > lo else 0.0
    dyh = float(tr["dy_heading"][hi - 1] - tr["dy_heading"][lo]) if hi > lo else 0.0
    dyc = float(tr["dy_crab"][hi - 1] - tr["dy_crab"][lo]) if hi > lo else 0.0
    frac = 1.0 if outcome == "pass" else (min(1.0, max(0.0, x_reached / lane_end_x)) if lane_end_x > 0 else None)

    def r(v, nd=3):
        return None if v is None or not math.isfinite(v) else round(float(v), nd)

    rec = {
        "run_id": run_id, "train_seed": None, "eval_seed": None, "sim": "real", "lane_id": lane_id,
        "tile_id": "lab", "episode_idx": int(episode_idx), "outcome": outcome,
        "time_to_pass_s": r(t_cross if t_cross is not None else dur, 2) if outcome == "pass" else None,
        "time_to_end_s": r(dur, 2), "x_reached_m": r(x_reached), "forward_dist_m": r(dist),
        "lane_fraction_completed": r(frac), "falls": int(outcome in ("head", "other")),
        "head_touches": None,
        "mean_speed_mps": r(dist / dur if dur > 0 else None),
        "drift_heading_m_per_m": r(dyh / dist if dist > 0.05 else None, 4),
        "drift_crab_m_per_m": r(dyc / dist if dist > 0.05 else None, 4),
    }
    out = {k: rec[k] for k in FIELDS}
    out["outcome_source"] = source          # extra key: operator or auto (not a sim field)
    return out


def command_onset(t, cmd) -> float | None:
    """Time of the first forward command above ``CMD_ONSET`` (the flight log's ``cmd``)."""
    cmd = np.asarray(cmd, float)
    if cmd.ndim != 2 or len(cmd) == 0:
        return None
    idx = np.flatnonzero(np.abs(cmd[:, 0]) > CMD_ONSET)
    return float(np.asarray(t, float)[idx[0]]) if idx.size else None
