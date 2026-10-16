#!/usr/bin/env python3
"""Analyse one recorded stepfield session (record_stepfield_run.sh): did the robot see the
blocks, was the scan it saw trustworthy, and did it lift for them before touching them?

    python3 src/go2_remote_controller/tools/analyze_stepfield_session.py sessions/<UTC>_<name>
        [--train_flat -0.158] [--lookahead 2.0] [--out DIR]

Reads the session's ``bag/`` (rosbag2) and, when present, every ``flight/<run>``. Writes
``analysis/summary.txt`` and ``analysis/report.png`` into the session and prints the
summary. Needs rosbag2_py + the unitree_go and go2_msgs message packages on this machine
(the same requirement as recording/bag_to_dataset.py).

Sections:

  1. SCAN HEALTH   rate, age (receive time - header stamp), observed cells, holes inside
                   the trained mask and extra cells outside it, flat-floor median of the
                   visible cells against the training value, and the near->far slope of
                   the row medians as a tilt in degrees. A slope is a mount or levelling
                   error (2026-10-15: a 10.6 deg mount error read as a 0/15 field with
                   the block at training's FLAT value). Also /go2/mount_monitor's live
                   mount fit against the camera.yaml the session was recorded with.
  2. BODY          roll/pitch from /lowstate, foot contacts from foot_force, swing apex
                   height of each front foot (leg FK, same as eval_rl_flight.py), and
                   "bump" events: pitch dips > BUMP_DEG within 0.3 s.
  3. ANTICIPATION  for every front-foot swing: the tallest rise the scan showed in the
                   visible band during the preceding ``--lookahead`` seconds, against
                   that swing's apex. A perceptive policy lifts higher when a block was
                   seen; a blind one shows no slope. Reported as the regression slope
                   (cm of apex per cm of seen block) and the apex split for swings with
                   and without a block in view. The same for bumps: how many were
                   preceded by a block in view, i.e. were avoidable.
  4. FLIGHT        per engaged run: steps, policy, scan age inside the policy node,
                   and a pointer to eval_rl_flight.py for the counterfactual replay.

Everything here is open-loop evidence from one run. Pair it with the sim probe
(eval_fixed_rung.py --scan_override flat) before concluding anything about the policy.
"""

from __future__ import annotations

import argparse
import glob
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "rl_policy"))

BUMP_DEG = 4.0           # pitch dip (deg) within BUMP_WIN s that counts as hitting a block
BUMP_WIN = 0.3
CONTACT_N = 20.0         # foot_force threshold (raw SDK units, ~N)
FRONT_FEET = (0, 1)      # SDK order FR, FL
BLOCK_SEEN_M = 0.06      # a rise above the floor median this big counts as a block in view


# ---------------------------------------------------------------- bag reading
def read_bag(bag_dir: str, topics: set[str]):
    from rclpy.serialization import deserialize_message
    from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
    from rosidl_runtime_py.utilities import get_message

    reader = SequentialReader()
    reader.open(StorageOptions(uri=bag_dir, storage_id=""), ConverterOptions("", ""))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    out = {t: [] for t in topics}
    cls = {}
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        if topic not in topics:
            continue
        if topic not in cls:
            cls[topic] = get_message(types[topic])
        out[topic].append((t_ns * 1e-9, deserialize_message(data, cls[topic])))
    return out


def stamp_s(header) -> float:
    return header.stamp.sec + header.stamp.nanosec * 1e-9


# ---------------------------------------------------------------- kinematics
HIP_X, HIP_Y, THIGH_Y, LINK, FOOT_R = 0.1934, 0.0465, 0.0955, 0.213, 0.022
SDK_LEGS = ((+1, -1), (+1, +1), (-1, -1), (-1, +1))   # FR, FL, RR, RL


def _rx(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def _ry(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def feet_sdk(q12):
    out = []
    for (sx, sy), (qh, qt, qc) in zip(SDK_LEGS, np.asarray(q12, float).reshape(4, 3)):
        leg = (np.array([0, sy * THIGH_Y, 0]) + _ry(qt) @ [0, 0, -LINK] + _ry(qt + qc) @ [0, 0, -LINK])
        out.append(np.array([sx * HIP_X, sy * HIP_Y, 0]) + _rx(qh) @ leg)
    return np.array(out)


def up_in_base(q):
    w, x, y, z = q
    return np.array([2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)])


def rpy_deg(q):
    w, x, y, z = q
    roll = np.degrees(np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)))
    pitch = np.degrees(np.arcsin(np.clip(2 * (w * y - z * x), -1, 1)))
    return roll, pitch


# ---------------------------------------------------------------- scan stats
def trained_mask(session_dir: str, n: int):
    """Trained FOV mask: from a flight run's meta if present, else from training's
    camera_fov with the session's camera.yaml, else None."""
    for meta_p in glob.glob(os.path.join(session_dir, "flight", "*", "meta.json")):
        with open(meta_p) as f:
            m = json.load(f)
        for k in ("trained_mask", "mask"):
            if k in m and m[k] is not None and len(m[k]) == n:
                return np.asarray(m[k], dtype=float) > 0
    try:
        import importlib.util
        p = os.path.expanduser("~/Go2_RL_workflow/training/go2_training/utils/camera_fov.py")
        spec = importlib.util.spec_from_file_location("camera_fov", p)
        cf = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cf)
        yaml_p = os.path.join(session_dir, "camera.yaml")
        fov = cf.load_camera_fov(yaml_p if os.path.isfile(yaml_p) else None)
        return cf.build_fov_mask(fov, (1.6, 1.0), 0.1, 0.6, -0.35) > 0
    except Exception as e:  # noqa: BLE001
        print(f"[analyze] no trained mask available ({e}); coverage stats will be mask-free")
        return None


def scan_frame_stats(h, unobs, nx, ny, cx, res, mask):
    seen = np.abs(h - unobs) > 1e-6
    d = {"observed": int(seen.sum())}
    if mask is not None:
        d["holes"] = int((~seen & mask).sum())
        d["extra"] = int((seen & ~mask).sum())
        use = seen & mask
    else:
        use = seen
    vals = h[use]
    if vals.size < 5:
        d.update(median=np.nan, slope_deg=np.nan, max_rise=np.nan)
        return d
    d["median"] = float(np.median(vals))
    g = h.reshape(ny, nx)
    gs = use.reshape(ny, nx)
    xs = cx - (nx - 1) * res / 2 + np.arange(nx) * res
    rows = np.array([np.median(g[:, ix][gs[:, ix]]) if gs[:, ix].sum() >= 3 else np.nan
                     for ix in range(nx)])
    ok = np.isfinite(rows)
    if ok.sum() >= 3:
        k = np.polyfit(xs[ok], rows[ok], 1)[0]          # d(value)/dx; value up = terrain down
        d["slope_deg"] = float(np.degrees(np.arctan(-k)))
    else:
        d["slope_deg"] = np.nan
    # tallest rise above the floor median anywhere in the visible band
    d["max_rise"] = float(np.max(d["median"] - vals))
    return d


# ---------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("session")
    ap.add_argument("--train_flat", type=float, default=-0.158,
                    help="scan value training reads on flat ground")
    ap.add_argument("--lookahead", type=float, default=2.0,
                    help="seconds before a swing/bump in which a seen block counts")
    ap.add_argument("--out", default=None, help="output dir (default <session>/analysis)")
    args = ap.parse_args()

    sess = os.path.abspath(args.session)
    out_dir = args.out or os.path.join(sess, "analysis")
    os.makedirs(out_dir, exist_ok=True)
    lines = [f"=== stepfield session {os.path.basename(sess)} ==="]
    meta_p = os.path.join(sess, "session_metadata.yaml")
    if os.path.isfile(meta_p):
        with open(meta_p) as f:
            lines += ["  " + ln.rstrip() for ln in f if ln.startswith("  ") and ":" in ln][:8]

    bag = read_bag(os.path.join(sess, "bag"),
                   {"/go2/height_scan", "/go2/mount_monitor", "/lowstate", "/web_control_mode"})

    # ---- 1. scan health
    scans = bag["/go2/height_scan"]
    if not scans:
        lines.append("\n[1] SCAN: no /go2/height_scan in the bag -- perception was not running")
        t_scan = np.zeros(0)
        max_rise = np.zeros(0)
    else:
        m0 = scans[0][1]
        nx, ny, cx, res, unobs = int(m0.num_x), int(m0.num_y), float(m0.center_x), float(m0.resolution), float(m0.unobserved_value)
        mask = trained_mask(sess, nx * ny)
        t_scan = np.array([t for t, _ in scans])
        age = np.array([t - stamp_s(m.header) for t, m in scans])
        st = [scan_frame_stats(np.asarray(m.heights, dtype=np.float32), unobs, nx, ny, cx, res, mask)
              for _, m in scans]
        med = np.array([s["median"] for s in st])
        slope = np.array([s["slope_deg"] for s in st])
        max_rise = np.array([s["max_rise"] for s in st])
        obs_n = np.array([s["observed"] for s in st])
        dt = np.diff(t_scan)
        lines.append(
            f"\n[1] SCAN HEALTH  {len(scans)} frames over {t_scan[-1] - t_scan[0]:.1f} s  "
            f"rate {1 / np.median(dt):.1f} Hz (gaps > 0.4 s: {int((dt > 0.4).sum())})  "
            f"age med {1000 * np.median(age):.0f} / p95 {1000 * np.percentile(age, 95):.0f} ms")
        lines.append(f"    grid {nx}x{ny} @ {res} m, centre x {cx:+.2f}, unobserved token {unobs}")
        lines.append(f"    observed cells med {np.median(obs_n):.0f}"
                     + (f"  holes med {np.median([s['holes'] for s in st]):.0f}  "
                        f"extra med {np.median([s['extra'] for s in st]):.0f}" if mask is not None else ""))
        lines.append(f"    flat-floor median {np.nanmedian(med):+.3f}  (training {args.train_flat:+.3f}, "
                     f"offset {100 * (np.nanmedian(med) - args.train_flat):+.1f} cm of apparent floor drop)")
        clear_view = max_rise < 0.03
        tilt_txt = (f"{np.nanmedian(slope[clear_view]):+.1f} deg over {int(clear_view.sum())} block-free frames"
                    if clear_view.sum() >= 5 else "n/a (a block was in view in every frame; tilt is only "
                    "meaningful on bare floor)")
        lines.append(f"    near->far tilt {tilt_txt} (|tilt| > 3 deg means the mount or levelling is wrong; "
                     f"see /go2/mount_monitor below)")
        lines.append(f"    frames with a block in view (rise > {100 * BLOCK_SEEN_M:.0f} cm): "
                     f"{100 * np.nanmean(max_rise > BLOCK_SEEN_M):.0f}%, tallest {100 * np.nanmax(max_rise):.0f} cm")
    mm = bag["/go2/mount_monitor"]
    if mm:
        a = np.array([list(m.data) for _, m in mm])
        if a.shape[1] >= 6:
            d_pitch, d_roll = a[:, 1] - a[:, 3], a[:, 0] - a[:, 2]
            lines.append(f"    mount monitor: pitch fit {np.median(a[:, 1]):.2f} vs camera.yaml {a[0, 3]:.2f} "
                         f"(d {np.median(d_pitch):+.2f} deg), roll d {np.median(d_roll):+.2f} deg, "
                         f"{len(mm)} reports")
            if abs(np.median(d_pitch)) > 1.0:
                lines.append("    !! mount differs from camera.yaml by > 1 deg: re-run calibrate_camera_mount")

    # ---- 2. body
    ls = bag["/lowstate"]
    if not ls:
        lines.append("\n[2] BODY: no /lowstate in the bag")
        t_ls = np.zeros(0)
        pitch = np.zeros(0)
        bumps = []
        swings = []
    else:
        t_ls = np.array([t for t, _ in ls])
        quat = np.array([list(m.imu_state.quaternion) for _, m in ls])
        q = np.array([[m.motor_state[i].q for i in range(12)] for _, m in ls])
        ff = np.array([list(m.foot_force)[:4] for _, m in ls], dtype=float)
        rp = np.array([rpy_deg(qq) for qq in quat])
        roll, pitch = rp[:, 0], rp[:, 1]
        up = np.array([up_in_base(qq) for qq in quat])
        feet = np.array([feet_sdk(qq) for qq in q])                 # (T, 4, 3) base frame
        foot_h = np.einsum("tfk,tk->tf", feet, up)                  # height along gravity, base-relative
        # Clearance of each foot above ITS OWN last stance height (lift-off point), so a
        # foot stepping off a block does not read block height + lift. Stance height is
        # held while the foot is in contact and frozen through the swing.
        contact_all = ff > CONTACT_N
        stance_h = np.empty_like(foot_h)
        last = foot_h[0].copy()
        for i in range(len(foot_h)):
            last = np.where(contact_all[i], foot_h[i], last)
            stance_h[i] = last
        clear = foot_h - stance_h
        contact = contact_all
        lines.append(f"\n[2] BODY  {len(ls)} lowstate samples over {t_ls[-1] - t_ls[0]:.1f} s  "
                     f"roll rms {np.sqrt(np.mean(roll ** 2)):.1f} deg  pitch rms {np.sqrt(np.mean(pitch ** 2)):.1f} deg")
        # bumps: pitch drop > BUMP_DEG inside BUMP_WIN
        hz = len(ls) / max(1e-6, t_ls[-1] - t_ls[0])
        w = max(1, int(BUMP_WIN * hz))
        bumps = []
        i = 0
        while i + w < len(pitch):
            if pitch[i] - pitch[i:i + w].min() > BUMP_DEG:
                bumps.append(t_ls[i])
                i += w
            else:
                i += 1
        # front-foot swings: contact off -> on, apex = max clearance in between
        swings = []
        for f in FRONT_FEET:
            c = contact[:, f]
            starts = np.flatnonzero(~c[1:] & c[:-1]) + 1
            ends = np.flatnonzero(c[1:] & ~c[:-1]) + 1
            for s0 in starts:
                e = ends[ends > s0]
                if e.size == 0:
                    break
                e0 = e[0]
                if 0.05 < t_ls[e0] - t_ls[s0] < 1.0:
                    swings.append((t_ls[s0], t_ls[e0], float(clear[s0:e0, f].max())))
        lines.append(f"    front-foot swings {len(swings)}  apex med {100 * np.median([s[2] for s in swings]) if swings else float('nan'):.1f} cm  "
                     f"bumps (pitch dip > {BUMP_DEG:.0f} deg in {BUMP_WIN} s) {len(bumps)}")

    # ---- 3. anticipation
    if scans and swings:
        def seen_before(t):
            sel = (t_scan >= t - args.lookahead) & (t_scan <= t)
            return float(np.nanmax(max_rise[sel])) if sel.any() else np.nan
        seen = np.array([seen_before(s[0]) for s in swings])
        apex = np.array([s[2] for s in swings])
        ok = np.isfinite(seen)
        lines.append(f"\n[3] ANTICIPATION (block seen in the {args.lookahead:.1f} s before a swing vs its apex)")
        if ok.sum() >= 10:
            k, b = np.polyfit(seen[ok], apex[ok], 1)
            r = np.corrcoef(seen[ok], apex[ok])[0, 1]
            with_b, without = apex[ok & (seen > BLOCK_SEEN_M)], apex[ok & (seen <= BLOCK_SEEN_M)]
            lines.append(f"    slope {k:+.2f} cm apex per cm of seen block  r = {r:+.2f}  (n = {int(ok.sum())})")
            lines.append(f"    apex with a block in view {100 * np.median(with_b) if with_b.size else float('nan'):.1f} cm (n={with_b.size})  "
                         f"without {100 * np.median(without) if without.size else float('nan'):.1f} cm (n={without.size})")
            verdict = ("LIFTS FOR SEEN BLOCKS" if (k > 0.3 and r > 0.3) else
                       "weak anticipation" if (k > 0.1 and r > 0.15) else "NO anticipation: apex independent of the scan")
            lines.append(f"    verdict: {verdict}")
        else:
            lines.append("    too few swings with scan coverage to fit")
        if bumps:
            avoidable = sum(1 for t in bumps if np.isfinite(seen_before(t)) and seen_before(t) > BLOCK_SEEN_M)
            lines.append(f"    bumps {len(bumps)}, of which {avoidable} had a block in view beforehand (avoidable)")

    # ---- 4. flight
    runs = sorted(glob.glob(os.path.join(sess, "flight", "*", "meta.json")))
    lines.append(f"\n[4] FLIGHT runs: {len(runs)}")
    for mp in runs:
        rd = os.path.dirname(mp)
        try:
            from flight_recorder import load
            meta, st, ev = load(rd)
            n = len(st.get("t", []))
            sa = st.get("scan_age")
            lines.append(f"    {os.path.basename(rd)}: {n} steps ({n / 50:.0f} s), policy {meta.get('policy')}, "
                         f"scan age med {1000 * np.median(sa) if sa is not None and len(sa) else float('nan'):.0f} ms, "
                         f"{len(ev)} events")
        except Exception as e:  # noqa: BLE001
            lines.append(f"    {os.path.basename(rd)}: could not load ({e})")
        lines.append(f"      counterfactual replay: python3 {os.path.relpath(os.path.join(HERE, 'eval_rl_flight.py'))} {os.path.relpath(rd)}")

    text = "\n".join(lines)
    print(text)
    with open(os.path.join(out_dir, "summary.txt"), "w") as f:
        f.write(text + "\n")

    # ---- report.png
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(4, 1, figsize=(12, 11), sharex=True)
        starts = ([t_scan[0]] if len(t_scan) else []) + ([t_ls[0]] if len(t_ls) else [])
        t0 = min(starts) if starts else 0.0
        if len(t_scan):
            ax[0].plot(t_scan - t0, med, lw=0.8, label="visible-cell median")
            ax[0].axhline(args.train_flat, color="k", ls="--", lw=0.8, label="training flat")
            ax[0].set_ylabel("scan value"); ax[0].legend(loc="upper right"); ax[0].invert_yaxis()
            ax[1].plot(t_scan - t0, 100 * max_rise, lw=0.8, label="tallest rise in view (cm)")
            ax[1].plot(t_scan - t0, slope, lw=0.8, label="near->far tilt (deg)")
            ax[1].legend(loc="upper right")
        if len(t_ls):
            ax[2].plot(t_ls - t0, pitch, lw=0.8, label="pitch (deg)")
            ax[2].plot(t_ls - t0, roll, lw=0.8, label="roll (deg)")
            for tb in bumps:
                ax[2].axvline(tb - t0, color="r", alpha=0.3, lw=0.8)
            ax[2].legend(loc="upper right")
            for f, name in zip(FRONT_FEET, ("FR", "FL")):
                ax[3].plot(t_ls - t0, 100 * clear[:, f], lw=0.8, label=f"{name} clearance (cm)")
            ax[3].legend(loc="upper right"); ax[3].set_xlabel("s")
        fig.suptitle(os.path.basename(sess))
        fig.tight_layout()
        fig.savefig(os.path.join(out_dir, "report.png"), dpi=110)
        print(f"[analyze] wrote {out_dir}/summary.txt and report.png")
    except Exception as e:  # noqa: BLE001
        print(f"[analyze] no report.png ({e}); summary.txt written")


if __name__ == "__main__":
    main()
