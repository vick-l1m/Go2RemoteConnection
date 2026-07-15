# Session recording — Go2 driving data for RL baselines & sim replay

Capture a real Go2 driving session (joint states + web-joystick commands + drive
mode) to a rosbag2, then export an aligned Parquet dataset. Two downstream uses:

- **RL baseline** — offline comparison / behaviour cloning against a policy.
- **Sim replay** — play the base-pose + joint trajectory back in Isaac Lab.

The recording is **read-only** — it never commands the robot. Drive as normal
(Unitree sport mode and/or the RL policy) from the web joystick while it runs.

The **camera is its own module** ([topics.sh](topics.sh) `CAMERA_TOPICS`): record
it alone with `record_camera.sh`, or fold it into a joint-state session with
`record_session.sh --camera` (one time-aligned bag). Both capture the raw D435i
depth cloud **and** the sim-matched `/go2/height_scan` (the 187-cell mask the
rough policy trained on).

## 1. Record (on the Go2 / Jetson)

Source ROS 2 Humble + the `unitree_ros2` overlay first (or just run the script —
it sources them, mirroring `RL_start_remote_connection.sh`). Then:

```bash
cd ~/Go2_RL_workflow/Go2RemoteConnection      # or wherever this repo lives
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

## 1b. Record camera (perception)

You have the RealSense **driver only**, so first bring up the full perception
stack — pointcloud filter, `/go2/camera` namespace, camera TF, and the height
map — on the robot (from the `go2_rl_workflow` overlay):

```bash
ros2 launch go2_bringup real_perception.launch.py rviz:=false
# verify the cloud + mask are live:
ros2 topic list | grep -E "go2/camera/depth|go2/height_scan"
```

This mirrors `sim_perception.launch.py` using the same `camera.yaml`, so
`/go2/height_scan` carries the **same 187-cell mask** as the sim rough policy.
If the cloud topic name differs from `/go2/camera/depth/color/points`, pass
`cloud_topic:=...` to the launch and set `GO2_CLOUD_TOPIC=...` for the recorders.

Then record — camera alone:
```bash
./record_camera.sh --name stepfield_01 --terrain "cubic stepfield"
```
…or camera **with** joint states in one time-aligned bag:
```bash
./record_session.sh --name perc_walk_01 --camera --terrain "cubic stepfield"
```

Recorded camera topics: `$GO2_CLOUD_TOPIC` (raw cloud, reprocessable),
`/go2/height_scan` (processed mask), `/go2/local_heightmap`,
`/go2/camera/color/image_raw` + `camera_info` (set `GO2_RECORD_RGB=0` to skip),
and `/tf` + `/tf_static` (to rebuild the cloud in the base frame offline).

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
- `hs_000`…`hs_186` — the `/go2/height_scan` mask (camera sessions only; absent
  otherwise). Same layout/ordering as the rough policy's obs suffix. The raw
  depth cloud is **not** exported — it stays in the bag for offline reprocessing.

Joint columns are keyed by name, so the SDK↔Isaac ordering never leaks into the
dataset. `session_metadata.yaml` records the mapping and gains for reference.

## 3. Replay a session in RViz

Watch a recorded session play back with the Go2 articulating (and camera, for
`--camera` sessions). `ros2 bag play` just re-publishes the recorded topics; the
viewer turns them into a moving robot:

- `lowstate_to_jointstate.py` — `/lowstate` (`unitree_go/LowState`) → name-keyed
  `sensor_msgs/JointState` on `/joint_states` (RViz can't animate from `LowState`).
- `robot_state_publisher` + the self-contained URDF at
  `description/go2/go2.urdf` → TF *inside* the robot (base_link → legs).
- `sportmodestate_to_tf.py` — `/sportmodestate` → `odom` → `base_link` TF, so the
  base **moves through space** instead of walking on the spot. It's the Go2's
  onboard odometry (drifts over a long run, no loop closure). Disable with
  `base_motion:=false` to pin the robot at the origin. Camera-only bags have no
  `/sportmodestate`, so the node stays silent for those.
- `view_session.rviz` — RobotModel + camera RGB + depth-cloud displays, fixed
  frame `odom` (so the robot traverses a fixed ground grid).

`view_session.launch.py` wires all three (+ rviz2) together.

### One-time host setup (until reboot)
Local replay needs the ROS 2 processes to discover each other over **loopback**.
The `unitree_ros2` overlay pins DDS to the robot NIC, and loopback isn't
multicast-capable by default, so enable it once:

```bash
sudo ip link set lo multicast on
```

### Every terminal
Source ROS 2 + the overlay, then `viewer_env.sh` (**last**, so it overrides the
overlay's DDS config for local loopback and resets the stale `ros2` daemon):

```bash
cd ~/Go2_RL_workflow/Go2RemoteConnection
source /opt/ros/humble/setup.bash
source ~/unitree_ros2/install/setup.bash
source src/go2_remote_connection/recording/viewer_env.sh   # prints "... multicast, domain 0. Daemon reset."
```

### Run it (two terminals, both sourced as above)
```bash
# Terminal A — viewer (bridge + robot_state_publisher + RViz)
ros2 launch src/go2_remote_connection/recording/view_session.launch.py

# Terminal B — play the bag (Terminal A does NOT play it)
ros2 bag play sessions/<UTCdate>_<name>/bag --loop
```
Handy: `-r 0.5` (half speed), `-p` (start paused; SPACE=play, s=step). If a live
Go2 is on the same domain, add `-x '/lowcmd'` so replayed commands never reach it.

### Verify (third sourced terminal)
```bash
ros2 node list                        # rviz, robot_state_publisher, lowstate_to_jointstate, rosbag2_player
ros2 topic echo /joint_states --once  # 12 named joints with positions
```

### Gotchas learned the hard way
- **`ros2 node list` empty / `echo` "Could not determine the type"** — the `ros2`
  CLI daemon cached a stale DDS config. `viewer_env.sh` runs `ros2 daemon stop`
  on source; if you changed configs mid-session, run `ros2 daemon stop` manually.
- **Nothing discovers anything** — re-source `viewer_env.sh` in *every* terminal
  (including the launch) and confirm it prints `multicast, domain 0`, not the
  ⚠️ "not multicast-capable" warning (→ redo the `sudo ip link` step).
- **Fixed frame** is `odom` (robot moves in space). Switch to `base_link` in
  RViz — or launch with `base_motion:=false` — to view it on the spot. Note the
  URDF root is `base_link`, not `base`.
- **Camera displays empty** — the session was recorded without `--camera`; only
  joints are in the bag. They populate automatically for `--camera` sessions.

## Before your first real record
1. `ros2 topic list` must show `/lowstate` and `/sportmodestate` — confirms the
   `unitree_ros2` bridge is up and `RMW_IMPLEMENTATION` + `ROS_DOMAIN_ID` match.
2. Confirm sport-mode driving works normally first (this repo's web joystick).
