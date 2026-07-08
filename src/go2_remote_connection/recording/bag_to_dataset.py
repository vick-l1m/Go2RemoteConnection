#!/usr/bin/env python3
"""Convert a recorded Go2 driving-session rosbag2 into an aligned Parquet dataset.

Reads a bag produced by ``record_session.sh`` and emits one time-aligned row per
control step (default 50 Hz, matching the deployment contract). Each row carries
the joint state (name-keyed), IMU, foot forces, base pose/velocity, the active
drive mode and the web-joystick command at that instant. Two downstream uses:

    * RL baseline  — behaviour-cloning / offline comparison against a policy.
    * Sim replay   — play the base-pose + joint trajectory back in Isaac Lab.

WHY THIS RUNS UNDER ROS 2 HUMBLE (not env_isaaclab):
    Deserialising ``unitree_go/msg/LowState`` needs the message definitions that
    only exist in the unitree_ros2 overlay. Output is still Parquet, loadable by
    the Python-3.11 training harness. Prereqs in the ROS 2 env:
        pip install pandas pyarrow
    then source ROS 2 Humble + the unitree_ros2 overlay before running.

Joint columns are keyed by NAME (q_FR_hip, dq_FL_calf, ...), so the SDK vs Isaac
ordering never leaks into the dataset — reorder by name in training as needed.

Usage:
    python3 bag_to_dataset.py <bag_dir> --out dataset.parquet [--rate 50] [--csv]
"""
import argparse
import pathlib

import numpy as np
import pandas as pd

from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

# ---------------------------------------------------------------------------
# Contract (mirror of go2_rl_policy_node.py — keep in sync).
# ---------------------------------------------------------------------------
SDK_JOINTS = [
    "FR_hip", "FR_thigh", "FR_calf",
    "FL_hip", "FL_thigh", "FL_calf",
    "RR_hip", "RR_thigh", "RR_calf",
    "RL_hip", "RL_thigh", "RL_calf",
]
FOOT_ORDER = ["FR", "FL", "RR", "RL"]
Q_DEFAULT = {
    "FL_hip": 0.1, "RL_hip": 0.1, "FR_hip": -0.1, "RR_hip": -0.1,
    "FL_thigh": 0.8, "FR_thigh": 0.8, "RL_thigh": 1.0, "RR_thigh": 1.0,
    "FL_calf": -1.5, "FR_calf": -1.5, "RL_calf": -1.5, "RR_calf": -1.5,
}

# Topics we know how to extract; anything else in the bag is ignored.
LOWSTATE_TOPICS = {"/lowstate", "lowstate", "rt/lowstate"}
SPORT_TOPICS = {"/sportmodestate", "sportmodestate", "rt/sportmodestate"}
TELEOP_TOPICS = {"/web_teleop"}
MODE_TOPICS = {"/web_control_mode"}
ESTOP_TOPICS = {"/web_estop"}


def projected_gravity(quat_wxyz):
    """Body-frame gravity unit vector from IMU quaternion (w,x,y,z). Upright -> (0,0,-1)."""
    w, x, y, z = quat_wxyz
    g = np.array([2 * (x * z - w * y),
                  2 * (y * z + w * x),
                  1 - 2 * (x * x + y * y)], dtype=np.float64)
    return -g


def read_bag(bag_dir):
    """Yield (topic, ros_msg, t_ns) for every message, deserialised by type."""
    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=str(bag_dir), storage_id=""),   # auto-detect sqlite3/mcap
        ConverterOptions(input_serialization_format="cdr",
                         output_serialization_format="cdr"),
    )
    type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}
    msg_cls = {name: get_message(tp) for name, tp in type_map.items()}
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        cls = msg_cls.get(topic)
        if cls is None:
            continue
        yield topic, deserialize_message(data, cls), t_ns


def collect(bag_dir):
    """First pass: bucket messages per stream as (t_ns, payload)."""
    streams = {"lowstate": [], "sport": [], "teleop": [], "mode": [], "estop": []}
    for topic, msg, t_ns in read_bag(bag_dir):
        if topic in LOWSTATE_TOPICS:
            ms = msg.motor_state
            row = {}
            for i, name in enumerate(SDK_JOINTS):
                row[f"q_{name}"] = ms[i].q
                row[f"dq_{name}"] = ms[i].dq
                row[f"tau_{name}"] = ms[i].tau_est
                row[f"qrel_{name}"] = ms[i].q - Q_DEFAULT[name]
            imu = msg.imu_state
            quat = list(imu.quaternion)               # (w,x,y,z)
            row.update(quat_w=quat[0], quat_x=quat[1], quat_y=quat[2], quat_z=quat[3])
            row.update(gyro_x=imu.gyroscope[0], gyro_y=imu.gyroscope[1], gyro_z=imu.gyroscope[2])
            row.update(acc_x=imu.accelerometer[0], acc_y=imu.accelerometer[1], acc_z=imu.accelerometer[2])
            pg = projected_gravity(quat)
            row.update(proj_g_x=pg[0], proj_g_y=pg[1], proj_g_z=pg[2])
            for j, foot in enumerate(FOOT_ORDER):
                row[f"foot_force_{foot}"] = msg.foot_force[j]
            streams["lowstate"].append((t_ns, row))
        elif topic in SPORT_TOPICS:
            row = {
                "base_x": msg.position[0], "base_y": msg.position[1], "base_z": msg.position[2],
                "base_vx": msg.velocity[0], "base_vy": msg.velocity[1], "base_vz": msg.velocity[2],
                "base_yaw_speed": msg.yaw_speed,
            }
            streams["sport"].append((t_ns, row))
        elif topic in TELEOP_TOPICS:
            streams["teleop"].append((t_ns, {
                "cmd_vx": msg.linear.x, "cmd_vy": msg.linear.y, "cmd_wz": msg.angular.z}))
        elif topic in MODE_TOPICS:
            streams["mode"].append((t_ns, {"control_mode": msg.data}))
        elif topic in ESTOP_TOPICS:
            streams["estop"].append((t_ns, {"estop": bool(msg.data)}))
    return streams


def zoh(sorted_stream, t_ns, default):
    """Zero-order hold: latest sample at or before t_ns (for command/mode signals)."""
    lo, hi, idx = 0, len(sorted_stream) - 1, -1
    while lo <= hi:
        mid = (lo + hi) // 2
        if sorted_stream[mid][0] <= t_ns:
            idx, lo = mid, mid + 1
        else:
            hi = mid - 1
    return dict(default) if idx < 0 else sorted_stream[idx][1]


def build_dataframe(streams, rate_hz):
    """Resample everything onto lowstate timestamps, then decimate to rate_hz."""
    low = sorted(streams["lowstate"], key=lambda x: x[0])
    if not low:
        raise SystemExit("no lowstate messages in bag — nothing to export")
    for k in ("sport", "teleop", "mode", "estop"):
        streams[k].sort(key=lambda x: x[0])

    t0 = low[0][0]
    step_ns = int(1e9 / rate_hz)
    next_emit = t0
    rows = []
    for t_ns, jrow in low:
        if t_ns < next_emit:      # decimate lowstate (~500 Hz) down to rate_hz
            continue
        next_emit = t_ns + step_ns
        row = {"t": (t_ns - t0) / 1e9}
        row.update(jrow)
        row.update(zoh(streams["sport"], t_ns,
                       {"base_x": np.nan, "base_y": np.nan, "base_z": np.nan,
                        "base_vx": np.nan, "base_vy": np.nan, "base_vz": np.nan,
                        "base_yaw_speed": np.nan}))
        row.update(zoh(streams["teleop"], t_ns, {"cmd_vx": 0.0, "cmd_vy": 0.0, "cmd_wz": 0.0}))
        row.update(zoh(streams["mode"], t_ns, {"control_mode": "unknown"}))
        row.update(zoh(streams["estop"], t_ns, {"estop": False}))
        rows.append(row)
    return pd.DataFrame(rows)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag_dir", help="rosbag2 directory (contains metadata.yaml)")
    ap.add_argument("--out", help="output .parquet path (default: <bag>/../dataset.parquet)")
    ap.add_argument("--rate", type=float, default=50.0, help="resample rate Hz (default 50)")
    ap.add_argument("--csv", action="store_true", help="also write a .csv sidecar for inspection")
    args = ap.parse_args()

    bag_dir = pathlib.Path(args.bag_dir).resolve()
    out = pathlib.Path(args.out) if args.out else bag_dir.parent / "dataset.parquet"

    df = build_dataframe(collect(bag_dir), args.rate)
    df.to_parquet(out, index=False)
    print(f"[export] {len(df)} rows @ {args.rate:g} Hz, {len(df.columns)} cols -> {out}")
    if not df.empty:
        span = df["t"].iloc[-1]
        modes = df["control_mode"].value_counts().to_dict()
        print(f"[export] duration {span:.1f}s | drive-mode rows: {modes}")
    if args.csv:
        csv_path = out.with_suffix(".csv")
        df.to_csv(csv_path, index=False)
        print(f"[export] csv sidecar -> {csv_path}")


if __name__ == "__main__":
    main()
