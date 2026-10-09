#!/usr/bin/env python3
"""Evaluate one recorded RL engagement: what did the policy see, and what set it off?

    python3 src/go2_remote_controller/tools/eval_rl_flight.py sessions/<...>/flight/<run>
    python3 .../eval_rl_flight.py <run> --onnx path/to/policy.onnx

Input is a flight-recorder directory (rl_policy/flight_recorder.py; GO2_RECORD=1 in
RL_start). Writes ``report.png`` and ``summary.txt`` into it and prints the summary.

What it checks:
  1. TIMELINE   duration, phases, ESTOP / saturation / tilt events, and the ONSET --
                the first step that looks like a freak-out (action past the clip, or
                body roll/pitch past 20 deg).
  2. REPLAY     the recorded obs through the same ONNX must reproduce the recorded
                actions. If not, the recording (or the policy file) is not what ran.
  3. SCAN       per step: observed cells, flat-floor value, near->far slope, cells
                outside the trained mask (from the raw scan), scan age -- and the
                correlation of slope with body pitch (non-zero = levelling not working).
  4. COUNTERFACTUAL  the SAME steps re-run with the scan replaced by
                  ideal  a perfect flat floor at the height the LEGS measure
                         (forward kinematics + IMU), in every trained cell
                  blind  every cell unobserved
                last_action is fed back from the counterfactual's own actions, and the
                commanded swing-foot height is compared -- "does the scan make it lift
                its legs?" in centimetres.
                If flat calms the actions down where the real scan saturated them, the
                scan is what the policy reacted to. Open-loop: the states are the real
                ones, only the scan input changes, so read it as "how much did the scan
                drive the action at THIS moment", not as a simulation of another run.
"""

import argparse
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "rl_policy"))
from flight_recorder import load  # noqa: E402

FREAKOUT_TILT_DEG = 20.0


def rpy_from_wxyz(q):
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    roll = np.degrees(np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)))
    pitch = np.degrees(np.arcsin(np.clip(2 * (w * y - z * x), -1, 1)))
    return roll, pitch


def scan_slices(meta):
    """Slices of every history frame of height_scan in the obs vector, oldest first."""
    off = 0
    for name in meta["obs_terms"]:
        w = int(meta["obs_widths"][name])
        if name == "height_scan":
            h = int(meta.get("obs_history", {}).get(name, 1) or 1)
            per = w // h
            return [slice(off + i * per, off + (i + 1) * per) for i in range(h)]
        off += w
    return []


def resolve_onnx(meta, override):
    if override:
        return override
    p = meta.get("policy_path")
    if p and os.path.exists(p):
        return p
    if p:  # recorded on the robot: map to this checkout's copy of the same policy
        local = os.path.join(HERE, "..", "rl_policy", "policies",
                             os.path.basename(os.path.dirname(p)), os.path.basename(p))
        if os.path.exists(local):
            return os.path.abspath(local)
    return None


def cell_x(meta, n):
    g = meta.get("scan_geom") or {}
    nx, ny = int(g.get("num_x", 17)), int(g.get("num_y", 11))
    res, cx = float(g.get("resolution", 0.1)), float(g.get("center_x", 0.6))
    if nx * ny != n:
        return None
    xs = cx - (nx - 1) * res / 2 + np.arange(nx) * res
    return np.tile(xs, ny)  # RayCaster order: x fastest


# ---- Go2 kinematics (src/go2_description/urdf/go2.urdf; same as go2_perception's
#      mount_calibration, copied so this tool runs from the submodule alone) ----
HIP_X, HIP_Y, THIGH_Y, LINK, FOOT_R = 0.1934, 0.0465, 0.0955, 0.213, 0.022
SDK_LEGS = ((+1, -1), (+1, +1), (-1, -1), (-1, +1))   # FR, FL, RR, RL: (x sign, y sign)


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


def up_in_base(quat_wxyz):
    w, x, y, z = quat_wxyz
    # third row of R_world_base = world z expressed in base
    return np.array([2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)])


def base_heights(q_sdk, quats):
    """Base height above the floor per step: lowest foot along gravity + foot radius."""
    return np.array([-(feet_sdk(q) @ up_in_base(qt)).min() + FOOT_R for q, qt in zip(q_sdk, quats)])


def term_slice(meta, term):
    off = 0
    for name in meta["obs_terms"]:
        w = int(meta["obs_widths"][name])
        if name == term:
            return slice(off, off + w)
        off += w
    return None


def isaac_sdk_map(st, meta):
    """(sdk index per Isaac joint, q_default in Isaac order) recovered from the log:
    joint_pos_rel (Isaac order) = q_sdk[perm] - q_default. None if it does not fit."""
    sl = term_slice(meta, "joint_pos_rel")
    if sl is None or "q" not in st:
        return None
    jpr, q = st["obs"][:, sl], st["q"]
    perm, qdef = [], []
    for j in range(12):
        off = q - jpr[:, j:j + 1]
        i = int(np.argmin(off.std(axis=0)))
        # 0.05 rad: the recorder samples /lowstate (500 Hz) a few ms after the obs
        # was built, so the match is close, not exact.
        if off[:, i].std() > 0.05:
            return None
        perm.append(i)
        qdef.append(float(off[:, i].mean()))
    return np.array(perm), np.array(qdef)


def swing_height(actions, alpha, clip, meta, mapping, quats):
    """Commanded height of the highest foot above the lowest, per step (m)."""
    perm, qdef = mapping
    scale = float(meta.get("action_scale", 0.25))
    out = np.zeros(len(actions))
    for k, (a, al, qt) in enumerate(zip(actions, alpha, quats)):
        q_isaac = qdef + scale * al * np.clip(a, -clip, clip)
        q_sdk = np.zeros(12)
        q_sdk[perm] = q_isaac
        z = feet_sdk(q_sdk) @ up_in_base(qt)
        out[k] = z.max() - z.min()
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("run", help="flight-recorder run directory")
    ap.add_argument("--onnx", help="policy.onnx (default: resolved from meta.json)")
    ap.add_argument("--no-plot", action="store_true")
    args = ap.parse_args()

    meta, st, events = load(args.run)
    out = []
    say = out.append
    n = len(st.get("t", []))
    if n == 0:
        print("no steps recorded")
        return 1
    t = st["t"]
    dt = float(meta.get("control_dt", 0.02))
    clip = float(meta.get("action_clip", 5.0))
    say(f"run: {args.run}")
    say(f"policy: {meta.get('policy_id')}  dry_run={meta.get('dry_run')}  steps={n} "
        f"({t[-1]:.1f} s, {n / max(t[-1], 1e-9):.1f} Hz)")

    # ---- 1. timeline ----
    roll, pitch = rpy_from_wxyz(st["quat"])
    amax = np.abs(st["action_raw"]).max(axis=1)
    bad = (amax > clip) | (np.abs(roll) > FREAKOUT_TILT_DEG) | (np.abs(pitch) > FREAKOUT_TILT_DEG)
    onset = int(np.argmax(bad)) if bad.any() else None
    say("\n== events")
    for e in events:
        if e["kind"] in ("phase", "start", "stop"):
            continue
        extra = {k: v for k, v in e.items() if k not in ("kind", "t", "wall")}
        say(f"  {e['t']:7.2f} s  {e['kind']:10s} {extra}")
    say(f"  saturating steps: {int((amax > clip).sum())}/{n}   max |a| {amax.max():.2f} (clip {clip})")
    say(f"  body roll  [{roll.min():+.1f}, {roll.max():+.1f}] deg   pitch [{pitch.min():+.1f}, {pitch.max():+.1f}] deg")
    say(f"  ONSET: {'none' if onset is None else f'step {onset} at {t[onset]:.2f} s (|a| {amax[onset]:.2f}, roll {roll[onset]:+.1f}, pitch {pitch[onset]:+.1f})'}")

    # ---- 2. replay ----
    onnx = resolve_onnx(meta, args.onnx)
    sess = None
    if onnx is None:
        say("\n== replay: policy.onnx not found (pass --onnx); skipping replay + counterfactuals")
    else:
        import onnxruntime as ort
        sess = ort.InferenceSession(onnx, providers=["CPUExecutionProvider"])
        name = sess.get_inputs()[0].name

        batch = sess.get_inputs()[0].shape[0]

        def run(obs):
            obs = obs.astype(np.float32)
            if isinstance(batch, int) and batch == 1:   # exported with a fixed batch of 1
                return np.concatenate([sess.run(None, {name: o[None]})[0] for o in obs])
            return sess.run(None, {name: obs})[0]

        a_rep = run(st["obs"])
        err = float(np.abs(a_rep - st["action_raw"]).max())
        say(f"\n== replay through {os.path.relpath(onnx)}: max |a_replay - a_recorded| = {err:.2e}"
            + ("  OK" if err < 1e-3 else "  MISMATCH -- this is not the policy that ran"))

    # ---- 3. scan ----
    slices = scan_slices(meta)
    unobs = meta.get("scan_unobserved")
    scan_obs = None
    if slices and unobs is not None:
        scan_obs = st["obs"][:, slices[-1]]                        # newest frame
        seen = np.abs(scan_obs - unobs) > 1e-6
        nseen = seen.sum(axis=1)
        med = np.array([np.median(r[s]) if s.any() else np.nan for r, s in zip(scan_obs, seen)])
        xs = cell_x(meta, scan_obs.shape[1])
        slope = np.full(n, np.nan)
        if xs is not None:
            for i in range(n):
                s = seen[i]
                if s.sum() > 10 and np.ptp(xs[s]) > 0.2:
                    slope[i] = np.polyfit(xs[s], scan_obs[i, s], 1)[0]
        age = st.get("scan_age", np.full(n, np.nan))
        say("\n== scan (what the policy saw, after masking)")
        say(f"  observed cells {np.median(nseen):.0f} (min {nseen.min()})   value median "
            f"{np.nanmedian(med):+.3f}, p5-p95 [{np.nanpercentile(med, 5):+.3f}, {np.nanpercentile(med, 95):+.3f}]")
        say(f"  near->far slope median {np.nanmedian(slope):+.3f}/m, p5-p95 "
            f"[{np.nanpercentile(slope, 5):+.3f}, {np.nanpercentile(slope, 95):+.3f}]  (flat floor: 0;"
            f" 0.1/m ~ 6 deg tilt)")
        ok = np.isfinite(slope)
        if ok.sum() > 20 and np.std(pitch[ok]) > 0.5:
            r = float(np.corrcoef(slope[ok], pitch[ok])[0, 1])
            say(f"  slope vs body pitch correlation {r:+.2f}"
                + ("  <-- scan tilts with the body: levelling NOT working" if abs(r) > 0.5 else "  (levelling OK)"))
        say(f"  scan age median {1000 * np.median(age):.0f} ms, p95 {1000 * np.percentile(age, 95):.0f} ms, "
            f"max {1000 * age.max():.0f} ms")
        mask = meta.get("scan_mask")
        raw = st.get("scan_raw")
        if mask is not None and raw is not None and raw.shape[1] == len(mask):
            m = np.asarray(mask, bool)
            extra = (~m & (np.abs(raw - unobs) > 1e-6)).sum(axis=1)
            holes = (m & (np.abs(raw - unobs) <= 1e-6)).sum(axis=1)
            say(f"  raw scan: {np.median(extra):.0f} cells outside the trained mask (blanked), "
                f"{np.median(holes):.0f} holes inside it (median per frame)")
        if onset is not None:
            w = slice(max(0, onset - int(1.0 / dt)), onset + 1)
            say(f"  1 s before onset: value {np.nanmedian(med[w]):+.3f}  slope {np.nanmedian(slope[w]):+.3f}/m"
                f"  observed {np.median(nseen[w]):.0f}  age max {1000 * age[w].max():.0f} ms")

    # ---- 4. counterfactuals ----
    # The truth the scan should have shown: flat floor at the base height the LEGS say
    # (forward kinematics on the recorded joints, levelled by the IMU), per step.
    h_base = base_heights(st["q"], st["quat"])
    expected = h_base - 0.5
    raw = st.get("scan_raw")
    if raw is not None and raw.ndim == 2 and raw.shape[1]:
        # Cells the CAMERA saw (raw scan, before masking/filling), inside the trained mask
        # when there is one -- the same set for masked and full-scan policies.
        wire_unobs = float((meta.get("scan_geom") or {}).get("unobserved", -1.0))
        ok = np.abs(raw - wire_unobs) > 1e-6
        if meta.get("scan_mask") is not None:
            ok &= np.asarray(meta["scan_mask"], bool)[None, :]
        floor_med = np.array([np.median(r[k]) if k.any() else np.nan for r, k in zip(raw, ok)])
        err = floor_med - expected      # value = base_z - hit_z - 0.5: more negative = higher floor
        say(f"\n== floor check vs leg kinematics (base height median {np.median(h_base):.3f} m)")
        say(f"  scan reads the floor {-100 * np.nanmedian(err):+.1f} cm vs the legs (+ = floor looks "
            f"HIGHER, like a platform ahead); first second {-100 * np.nanmedian(err[: int(1 / dt)]):+.1f} cm")
    cf = {}
    lift = {}
    if sess is not None and slices:
        sl_la = term_slice(meta, "last_action")
        m = meta.get("scan_mask")
        full = unobs is None or m is None
        m = np.ones(slices[-1].stop - slices[-1].start, bool) if m is None else np.asarray(m, bool)
        fill_unobs = np.float32(0.0 if unobs is None else unobs)
        alpha = st.get("alpha", np.ones(n))
        mapping = isaac_sdk_map(st, meta)

        def rollout(scan_fn):
            """Re-run every step with the scan from scan_fn(i); last_action is the
            counterfactual's OWN previous action, as the node would have fed it back."""
            acts = np.zeros_like(st["action_raw"])
            for k in range(n):
                o = st["obs"][k].copy()
                if scan_fn is not None:
                    sc = scan_fn(k)
                    for s_ in slices:
                        o[s_] = sc
                    if k > 0 and sl_la is not None:
                        o[sl_la] = alpha[k - 1] * np.clip(acts[k - 1], -clip, clip)
                acts[k] = run(o[None])[0]
            return acts

        variants = [("recorded", None),
                    ("ideal", lambda k: np.where(m, np.float32(expected[k]), fill_unobs).astype(np.float32))]
        if not full:
            variants.append(("blind", lambda k: np.full(m.size, fill_unobs, np.float32)))
        say("\n== counterfactual (recorded states; scan swapped; last_action fed back from the "
            "counterfactual itself)")
        say("  ideal = flat floor at the leg-kinematics height in every trained cell")
        say(f"  {'scan':8s} {'saturating':>11s} {'mean|a|':>8s} {'max|a|':>7s} {'mean|a-a_rec|':>14s}"
            f" {'swing foot cm':>14s}")
        for label, fn in variants:
            a = run(st["obs"]) if fn is None else rollout(fn)
            cf[label] = np.abs(a).max(axis=1)
            if mapping is not None:
                lift[label] = swing_height(a, alpha, clip, meta, mapping, st["quat"])
            lf = (f"{100 * np.median(lift[label]):5.1f} / {100 * np.percentile(lift[label], 95):4.1f}"
                  if label in lift else "     -")
            say(f"  {label:8s} {int((cf[label] > clip).sum()):>6d}/{n:<4d} {np.abs(a).mean():8.3f} "
                f"{cf[label].max():7.2f} {np.abs(a - st['action_raw']).mean():14.3f} {lf:>14s}")
        if lift:
            say("  swing foot = commanded height of the highest foot above the lowest, median / p95")
            if "ideal" in lift:
                d = 100 * (np.median(lift["recorded"]) - np.median(lift["ideal"]))
                say(f"  -> the real scan makes it lift its feet {d:+.1f} cm vs a perfect flat scan")
        a0 = {k: v[0] for k, v in cf.items()}
        say("  first step (clean state, no feedback yet) max|a|: "
            + "  ".join(f"{k} {v:.2f}" for k, v in a0.items()))
        if a0["recorded"] > 1.0 and a0["recorded"] > 2.0 * a0["ideal"]:
            say(f"  -> the SCAN alone takes the very first action from {a0['ideal']:.2f} to "
                f"{a0['recorded']:.2f}: the policy reacted to what it saw from step 0")
        if onset is not None and t[onset] < 1.0:
            say("  (onset within 1 s of engage: after it the recorded states are already off "
                "the rails, so later steps cannot be judged open-loop -- trust the first step)")
        elif onset is not None:
            w = slice(max(0, onset - int(0.5 / dt)), min(n, onset + int(0.5 / dt)))
            say("  around onset (+-0.5 s) max|a|: "
                + "  ".join(f"{k} {v[w].max():.2f}" for k, v in cf.items()))
            if cf["recorded"][w].max() > clip and cf["ideal"][w].max() <= clip:
                say("  -> with a perfect scan it would NOT have saturated here: the SCAN drove it")
            elif cf["ideal"][w].max() > clip:
                say("  -> it saturates even with a perfect scan: look at the OTHER inputs "
                    "(proprioception, command) or the dynamics")

    text = "\n".join(out)
    print(text)
    with open(os.path.join(args.run, "summary.txt"), "w") as f:
        f.write(text + "\n")

    # ---- plot ----
    if not args.no_plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        rows = 5 if scan_obs is not None else 3
        fig, ax = plt.subplots(rows, 1, figsize=(12, 2.2 * rows), sharex=True)
        ax[0].plot(t, roll, label="roll")
        ax[0].plot(t, pitch, label="pitch")
        ax[0].set_ylabel("body deg")
        ax[0].legend(loc="upper left")
        ax[1].plot(t, amax, label="recorded")
        for k in ("ideal", "blind"):
            if k in cf:
                ax[1].plot(t, cf[k], label=f"{k} scan", alpha=0.7)
        ax[1].axhline(clip, color="k", ls="--", lw=0.8)
        ax[1].set_ylabel("max |action|")
        ax[1].legend(loc="upper left")
        ax[2].plot(t, st["cmd"][:, 0], label="vx")
        ax[2].plot(t, st["cmd"][:, 2], label="wz")
        ax[2].set_ylabel("command")
        ax[2].legend(loc="upper left")
        if scan_obs is not None:
            ax[3].plot(t, med, label="value")
            ax[3].plot(t, slope, label="slope /m")
            ax[3].set_ylabel("scan")
            ax[3].legend(loc="upper left")
            ax[4].plot(t, 1000 * st["scan_age"])
            ax[4].set_ylabel("scan age ms")
        for e in events:
            if e["kind"] in ("estop", "recover", "fault"):
                for a in ax:
                    a.axvline(e["t"], color="r", lw=0.8)
        if onset is not None:
            for a in ax:
                a.axvline(t[onset], color="m", ls=":", lw=1)
        ax[-1].set_xlabel("s since engage")
        fig.suptitle(f"{meta.get('policy_id')}  ({os.path.basename(os.path.normpath(args.run))})")
        fig.tight_layout()
        png = os.path.join(args.run, "report.png")
        fig.savefig(png, dpi=110)
        print(f"\nplot: {png}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
