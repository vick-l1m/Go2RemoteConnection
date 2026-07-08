# Session recording — Go2 driving data for RL baselines & sim replay

Capture a real Go2 driving session (joint states + web-joystick commands + drive
mode) to a rosbag2, then export an aligned Parquet dataset. Two downstream uses:

- **RL baseline** — offline comparison / behaviour cloning against a policy.
- **Sim replay** — play the base-pose + joint trajectory back in Isaac Lab.

The recording is **read-only** — it never commands the robot. Drive as normal
(Unitree sport mode and/or the RL policy) from the web joystick while it runs.

## 1. Record (on the Go2 / Jetson)

Source ROS 2 Humble + the `unitree_ros2` overlay first (or just run the script —
it sources them, mirroring `RL_start_remote_connection.sh`). Then:

```bash
cd ~/go2_ws/Go2RemoteConnection      # or wherever this repo lives
./record_session.sh --name flat_walk_01 --mode both \
    --terrain "lab floor" --notes "figure-8, comfortable pace"
# ... drive the robot from the web joystick ... Ctrl+C to stop.
```

`--mode` is `sport` | `rl` | `both`. For `sport`/`rl` a one-shot
`/web_control_mode` marker is published at start so the bag is self-describing
even though the web app latches that topic. In `both`, rely on the live
`/web_control_mode` switches captured during driving.

Output lands in `sessions/<UTCdate>_<name>/`:
- `bag/` — the rosbag2 (raw, replayable, ground truth)
- `session_metadata.yaml` — drive mode, terrain, notes, git SHA, and the joint
  order / gains / `q_default` contract needed to interpret it later.

### Recorded topics
| topic | type | why |
|---|---|---|
| `/lowstate` | `unitree_go/msg/LowState` | per-joint q, dq, tau_est; IMU quat/gyro/accel; foot forces |
| `/sportmodestate` | `unitree_go/msg/SportModeState` | base position, velocity, yaw rate (sim replay + `base_lin_vel` obs) |
| `/wirelesscontroller` | `unitree_go/msg/WirelessController` | native remote sticks/buttons (bonus context) |
| `/lowcmd` | `unitree_go/msg/LowCmd` | commanded joint targets when the RL policy drives |
| `/web_teleop` | `geometry_msgs/Twist` | joystick command: vx=linear.x, vy=linear.y, wz=angular.z |
| `/web_control_mode` | `std_msgs/String` | active drive mode `"sport"｜"rl"` |
| `/web_estop`, `/web_teleop_enabled`, `/web_sport_cmd` | — | safety / context |

Override the state-topic names for a different `unitree_ros2` install via
`GO2_STATE_TOPICS="..."`, and add extra topics with `--topics "/a /b"`.

## 2. Export to Parquet

Runs in the **ROS 2 Humble env** (deserialising `unitree_go` messages needs the
overlay's message definitions — `env_isaaclab` doesn't have them). One-time:

```bash
pip install pandas pyarrow      # into the ROS 2 Python
```

Then:

```bash
python3 src/go2_remote_connection/recording/bag_to_dataset.py \
    sessions/<UTCdate>_<name>/bag \
    --out sessions/<UTCdate>_<name>/dataset.parquet --rate 50 --csv
```

The resulting Parquet loads directly in the Python-3.11 training harness
(`pd.read_parquet`). One time-aligned row per control step (default 50 Hz):

- `t` — seconds from session start
- `control_mode`, `estop` — drive-mode / safety tag per row
- `cmd_vx`, `cmd_vy`, `cmd_wz` — joystick command (zero-order hold)
- `q_<joint>`, `dq_<joint>`, `tau_<joint>`, `qrel_<joint>` — **name-keyed** joint
  state (`qrel = q - q_default`); reorder by name to any policy's joint order
- `quat_{w,x,y,z}`, `gyro_{x,y,z}`, `acc_{x,y,z}`, `proj_g_{x,y,z}` — IMU
- `foot_force_{FR,FL,RR,RL}`
- `base_{x,y,z}`, `base_v{x,y,z}`, `base_yaw_speed` — from sportmodestate

Joint columns are keyed by name, so the SDK↔Isaac ordering never leaks into the
dataset. `session_metadata.yaml` records the mapping and gains for reference.

## Before your first real record
1. `ros2 topic list` must show `/lowstate` and `/sportmodestate` — confirms the
   `unitree_ros2` bridge is up and `RMW_IMPLEMENTATION` + `ROS_DOMAIN_ID` match.
2. Confirm sport-mode driving works normally first (this repo's web joystick).
