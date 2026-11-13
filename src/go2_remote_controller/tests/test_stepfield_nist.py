"""NIST run records for real stepfield runs (tools/stepfield_nist.py)."""

import os
import sys

import numpy as np
import pytest

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(PKG, "tools"))
sys.path.insert(0, os.path.join(PKG, "rl_policy"))

import stepfield_nist as N  # noqa: E402

STAND_Q_SDK = np.tile([0.0, 0.8, -1.5], 4)


def _synthetic_track(x_end: float, n: int = 500, tilt_max: float = 0.0) -> dict:
    t = np.linspace(0.0, 20.0, n)
    x = np.linspace(0.0, x_end, n)
    return {"t": t, "x": x, "y": np.zeros(n), "tilt_deg": np.full(n, tilt_max), "fwd": x.copy(),
            "dy_heading": 0.01 * x, "dy_crab": -0.02 * x}


def test_fields_match_the_sim_records():
    assert N.FIELDS[0] == "run_id" and len(N.FIELDS) == len(set(N.FIELDS)) == 18
    outer = os.path.join(PKG, "..", "..", "..", "training")
    if os.path.isdir(os.path.join(outer, "go2_training")):
        sys.path.insert(0, outer)
        from go2_training.utils import episode_log
        assert N.FIELDS == episode_log.FIELDS
        assert N.OUTCOMES == episode_log.OUTCOMES


def test_standing_robot_does_not_move():
    n = 100
    t = np.arange(n) * 0.002
    tr = N.track(t, np.tile(STAND_Q_SDK, (n, 1)), np.zeros((n, 12)), np.zeros((n, 3)),
                 np.tile([1.0, 0.0, 0.0, 0.0], (n, 1)), np.full((n, 4), 80.0))
    assert abs(tr["x"][-1]) < 1e-9 and abs(tr["y"][-1]) < 1e-9
    assert tr["tilt_deg"].max() == pytest.approx(0.0, abs=1e-6)


def test_auto_outcomes():
    end, pass_x = N.lane_geometry(8)
    assert (end, pass_x) == pytest.approx((3.6, 4.2))
    r = N.run_record(_synthetic_track(5.0), lane_id="t1a", lane_end_x=end, pass_x=pass_x, run_id="p", episode_idx=0)
    assert r["outcome"] == "pass" and r["outcome_source"] == "auto"
    assert r["time_to_pass_s"] == pytest.approx(20.0 * 4.2 / 5.0, abs=0.1)
    assert r["lane_fraction_completed"] == 1.0
    r = N.run_record(_synthetic_track(1.8), lane_id="t1a", lane_end_x=end, pass_x=pass_x, run_id="p", episode_idx=1)
    assert r["outcome"] == "stuck" and r["lane_fraction_completed"] == pytest.approx(0.5)
    assert r["time_to_pass_s"] is None and r["falls"] == 0
    r = N.run_record(_synthetic_track(1.0, tilt_max=80.0), lane_id="t1a", lane_end_x=end, pass_x=pass_x,
                     run_id="p", episode_idx=2)
    assert r["outcome"] == "other" and r["falls"] == 1


def test_operator_outcome_wins_and_window():
    end, pass_x = N.lane_geometry(8)
    r = N.run_record(_synthetic_track(5.0), lane_id="t1a", lane_end_x=end, pass_x=pass_x, run_id="p",
                     episode_idx=0, outcome="side")
    assert r["outcome"] == "side" and r["outcome_source"] == "operator" and r["time_to_pass_s"] is None
    # A window starting at t = 10 s re-zeroes x at the onset.
    r = N.run_record(_synthetic_track(5.0), lane_id="t1a", lane_end_x=end, pass_x=pass_x, run_id="p",
                     episode_idx=0, t_onset=10.0)
    assert r["x_reached_m"] == pytest.approx(2.5, abs=0.02) and r["outcome"] == "stuck"
    assert r["drift_heading_m_per_m"] == pytest.approx(0.01) and r["head_touches"] is None
    with pytest.raises(ValueError):
        N.run_record(_synthetic_track(1.0), lane_id="x", lane_end_x=1, pass_x=2, run_id="p",
                     episode_idx=0, outcome="crossed")


def test_command_onset():
    t = np.arange(5) * 0.02
    cmd = np.array([[0, 0, 0], [0.01, 0, 0], [0.3, 0, 0], [0.3, 0, 0], [0, 0, 0]], float)
    assert N.command_onset(t, cmd) == pytest.approx(0.04)
    assert N.command_onset(t, np.zeros((5, 3))) is None
