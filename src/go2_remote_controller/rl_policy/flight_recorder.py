"""Flight recorder for the RL policy node: what the policy saw and did, every step.

A rosbag shows the robot's state and the height scan, but not the policy's own
inputs and outputs -- those exist only inside go2_rl_policy_node (the obs vector is
assembled there, the actions go out over the Unitree SDK, not ROS). When the robot
"freaks out", the question is WHICH input it reacted to, and only the exact obs
vector can answer it: tools/eval_rl_flight.py replays it through the same ONNX,
including counterfactuals (the same steps with a perfect flat-floor scan).

Enabled by ``GO2_RL_RECORD_DIR`` (RL_start sets it with ``GO2_RECORD=1``). One
directory per engagement::

    <dir>/<YYYYmmdd-HHMMSS>_<policy>/
        meta.json        policy, contract layout, mask, gains, clip
        steps_0000.npz   per-step arrays, one chunk per CHUNK_STEPS (30 s at 50 Hz)
        events.jsonl     phase changes, ESTOP reasons, saturation, tilt aborts

Chunks are written by a background thread: an np.savez on the Jetson takes longer
than the 20 ms control period, and the control loop must never wait on a disk.
Chunking also bounds what a crash or a SIGKILL can lose to the last 30 s.
"""

import json
import os
import queue
import threading
import time

import numpy as np

CHUNK_STEPS = 1500

FIELDS = ("t", "wall", "obs", "action_raw", "action_cmd", "q_target_sdk", "kp", "kd",
          "alpha", "q", "dq", "quat", "gyro", "cmd", "scan_raw", "scan_age")


class FlightRecorder:
    """Per-engagement step log. All public methods are cheap and never raise."""

    def __init__(self, root: str, logger=None):
        self.root = os.path.expanduser(root)
        self.log = logger
        self.dir = None
        self._rows = []
        self._chunk = 0
        self._t0 = None
        self._q = queue.Queue()
        self._writer = threading.Thread(target=self._write_loop, name="flight-recorder",
                                        daemon=True)
        self._writer.start()

    @classmethod
    def from_env(cls, logger=None):
        root = os.environ.get("GO2_RL_RECORD_DIR", "").strip()
        if not root:
            return None
        rec = cls(root, logger)
        if logger is not None:
            logger.info(f"flight recorder ON -> {rec.root}")
        return rec

    @property
    def active(self) -> bool:
        return self.dir is not None

    # ---- lifecycle ----
    def start(self, policy_id: str, meta: dict):
        """Open a new engagement directory (closes any open one first)."""
        self.stop("restart")
        safe = "".join(c if c.isalnum() or c in "-_." else "_" for c in (policy_id or "policy"))
        d = os.path.join(self.root, f"{time.strftime('%Y%m%d-%H%M%S')}_{safe}")
        try:
            os.makedirs(d, exist_ok=True)
            with open(os.path.join(d, "meta.json"), "w") as f:
                json.dump(_jsonable(dict(meta, policy_id=policy_id,
                                         started=time.strftime("%Y-%m-%dT%H:%M:%S%z"))),
                          f, indent=1)
        except OSError as e:
            self._warn(f"flight recorder: cannot open {d}: {e}")
            return
        self.dir, self._rows, self._chunk, self._t0 = d, [], 0, time.monotonic()
        self.event("start", policy=policy_id)
        if self.log is not None:
            self.log.info(f"flight recorder: recording to {d}")

    def stop(self, reason: str = "stop"):
        if self.dir is None:
            return
        self.event("stop", reason=reason)
        self._flush()
        self.dir = None

    def close(self):
        """Flush and wait for the writer (shutdown)."""
        self.stop("shutdown")
        self._q.put(None)
        self._writer.join(timeout=5.0)

    # ---- data ----
    def step(self, **row):
        if self.dir is None:
            return
        row.setdefault("t", time.monotonic() - self._t0)
        row.setdefault("wall", time.time())
        self._rows.append(row)
        if len(self._rows) >= CHUNK_STEPS:
            self._flush()

    def event(self, kind: str, **info):
        if self.dir is None:
            return
        rec = dict(info, kind=kind, t=time.monotonic() - (self._t0 or time.monotonic()),
                   wall=time.time())
        self._q.put(("event", self.dir, rec))

    # ---- internals ----
    def _flush(self):
        if not self._rows:
            return
        rows, self._rows = self._rows, []
        self._q.put(("chunk", self.dir, self._chunk, rows))
        self._chunk += 1

    def _write_loop(self):
        while True:
            job = self._q.get()
            if job is None:
                return
            try:
                if job[0] == "event":
                    _, d, rec = job
                    with open(os.path.join(d, "events.jsonl"), "a") as f:
                        f.write(json.dumps(_jsonable(rec)) + "\n")
                else:
                    _, d, idx, rows = job
                    arrays = {}
                    for k in FIELDS:
                        vals = [r.get(k) for r in rows]
                        if any(v is None for v in vals):
                            continue
                        arrays[k] = np.asarray(vals, dtype=np.float32 if k != "wall" else np.float64)
                    np.savez(os.path.join(d, f"steps_{idx:04d}.npz"), **arrays)
            except Exception as e:  # noqa: BLE001 -- recording must never take the node down
                self._warn(f"flight recorder write failed: {e}")

    def _warn(self, msg):
        if self.log is not None:
            self.log.warn(msg)


def _jsonable(x):
    if isinstance(x, dict):
        return {str(k): _jsonable(v) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [_jsonable(v) for v in x]
    if isinstance(x, np.ndarray):
        return x.tolist()
    if isinstance(x, (np.floating, np.integer, np.bool_)):
        return x.item()
    return x


def load(run_dir: str):
    """(meta, steps dict of concatenated arrays, events list) for one engagement."""
    with open(os.path.join(run_dir, "meta.json")) as f:
        meta = json.load(f)
    chunks = sorted(p for p in os.listdir(run_dir) if p.startswith("steps_") and p.endswith(".npz"))
    steps = {}
    for p in chunks:
        with np.load(os.path.join(run_dir, p)) as z:
            for k in z.files:
                steps.setdefault(k, []).append(z[k])
    steps = {k: np.concatenate(v) for k, v in steps.items()}
    events = []
    ev = os.path.join(run_dir, "events.jsonl")
    if os.path.exists(ev):
        with open(ev) as f:
            events = [json.loads(line) for line in f if line.strip()]
    return meta, steps, events
