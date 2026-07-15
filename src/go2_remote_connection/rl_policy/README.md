# RL Policy Low-Level Control (`rl_policy/`)

Runs the Isaac-Lab flat-terrain RL policy on the Go2's motors, driven by the web
joystick, as an alternative to the built-in Unitree SportClient gaits. Switch
between them with the **RL: OFF/ON** button on the joystick page.

Full design + rationale: `Go2_RL_workflow/sim_to_real_deployment_plan.md`.

> ⚠️ This is **low-level joint control**. A wrong joint map, gain, or sign can
> make the robot lunge. **Test on a gantry first**, feet off the ground, E-stop
> in hand. This build uses the un-retrained 48-dim policy with `base_lin_vel`
> fed zeros — expect a rougher gait than sim. It validates the *pipeline*.

## Files

| File | What |
|------|------|
| `go2_rl_policy_node.py` | the controller (pure `unitree_sdk2py`, **no rclpy**): `rt/lowstate` → 48-dim obs → `policy.onnx` → `rt/lowcmd` @ 50 Hz, plus sport-service handover and safety fallbacks. Talks to the bridge over localhost UDP. |
| `go2_rl_bridge_node.py` | the ROS↔UDP bridge (pure rclpy, **no SDK**): forwards `/web_teleop`, `/web_control_mode`, `/web_estop` to the controller and republishes its heartbeat / un-gate signals. |
| `policy.onnx` | exported flat policy (copy of `logs/rsl_rl/unitree_go2_flat/<ts>/exported/policy.onnx`) |
| `joint_names.json` | Isaac-Lab joint order — **verify before ground tests** |
| `dump_isaac_joint_order.py` | run in the Isaac Lab env to regenerate `joint_names.json` from the real env |

### Why two processes?

`unitree_sdk2py` and ROS 2's `rmw_cyclonedds` both use CycloneDDS, and in a
single process they share one `libddsc`. Both insist on **creating** DDS domain 0
(the robot's low-level interface is fixed there, and the SDK's `ChannelFactory`
has no "join" path), so whichever calls `dds_create_domain(0)` second dies with
`Precondition Not Met`. They cannot coexist in one process. So the controller is
SDK-only (owns domain 0) and the bridge is ROS-only (like `web_bridge`); they
exchange the web commands over a localhost UDP link (`127.0.0.1`, ports 47811/47812,
override with `GO2_RL_CTRL_PORT` / `GO2_RL_BRIDGE_PORT` / `GO2_RL_UDP_HOST`). If the
bridge link goes silent while RL is driving, the controller fails safe to a damp
(soft collapse).

## One-time setup

The node runs in the SDK venv (`~/venvs/unitree_sdk2_python`), which already has
`unitree_sdk2py`, `rclpy`, and `numpy`. Add ONNX Runtime:

```bash
~/venvs/unitree_sdk2_python/bin/pip install onnxruntime    # aarch64 wheel on the Jetson
```

Verify the joint order matches your trained env (it usually does, but a mismatch
silently drives the wrong leg):

```bash
# in the Isaac Lab env, on the training machine
conda activate env_isaaclab && cd ~/IsaacLab
python ~/Go2_RL_workflow/Go2RemoteConnection/src/go2_remote_connection/rl_policy/dump_isaac_joint_order.py
# copy the regenerated joint_names.json back next to policy.onnx
```

## Bring-up (gantry first — do not skip a step)

**0. Robot on a stand, feet off the ground. E-stop in hand.**

Standalone bring-up runs **two** processes: the SDK controller and the ROS
bridge. The `ros2 topic pub` commands reach the controller *through* the bridge.

**1. Dry run** — validate obs/inference/remap with no torque (motors limp):

```bash
cd ~/Go2_RL_workflow/Go2RemoteConnection/src/go2_remote_connection/rl_policy
# terminal A — the ROS<->UDP bridge (pure rclpy):
~/venvs/unitree_sdk2_python/bin/python3 go2_rl_bridge_node.py
# terminal B — the SDK controller:
~/venvs/unitree_sdk2_python/bin/python3 go2_rl_policy_node.py --net <iface> --dry-run --no-handover --no-prompt
# terminal C — publish a mode + a command and watch B's logs:
ros2 topic pub -1 /web_control_mode std_msgs/String "{data: 'rl'}"
ros2 topic pub -r 10 /web_teleop geometry_msgs/Twist "{linear: {x: 0.3}}"
```

In `--dry-run`, `kp=kd=0` so nothing moves; you're confirming the node reads
state, builds a finite 48-dim obs, and produces sane targets at 50 Hz.

**2. Powered hold** — drop `--dry-run`. Keep `--no-handover` only if you have
manually stopped the sport service; otherwise omit it so the node releases it:

```bash
# terminal A — bridge (leave running from step 1)
~/venvs/unitree_sdk2_python/bin/python3 go2_rl_bridge_node.py
# terminal B — controller
~/venvs/unitree_sdk2_python/bin/python3 go2_rl_policy_node.py --net <iface> --no-prompt
ros2 topic pub -1 /web_control_mode std_msgs/String "{data: 'rl'}"
```

Expect: crouch → ramp to default pose → policy holds a stance (it settles ~0.2 rad
off `q_default`; a small adjustment is normal). **No buzzing, no divergence** is
the go/no-go gate. Send tiny commands (`x: 0.2`) and watch coordinated stepping
**in the air**.

**3. Full web integration** — start the stack with the node enabled:

```bash
GO2_RL_POLICY=1 ./RL_start_remote_connection.sh
```

Open the joystick page, press **RL: ON** (confirm dialog), drive with the
joystick. Press **RL: OFF** to hand back to the Unitree sport gaits.

**4. Ground** — only after 1–3 pass, in a clear open area, low speed, short bursts.

## Flags

| Flag | Use |
|------|-----|
| `--net <iface>` | DDS interface to the robot (`UNITREE_IFACE`, e.g. `enP8p1s0` on Jetson) |
| `--dry-run` | compute everything, publish `kp=kd=0` (no torque) |
| `--no-handover` | skip sport-service release/recover — you guarantee no sport service is running |
| `--no-prompt` | skip the Enter confirmation (used by the launcher; node still idles until `rl`) |

## Safety behaviour

- Starts **idle** in `sport` phase; never publishes `rt/lowcmd` until `rl` is selected.
- Joystick silence > 0.5 s → command zeroed (deadman).
- Non-finite obs/action, or any control-loop exception → **damping** command
  (`kp=0, kd=3`) for a soft, controlled collapse rather than a hard cut.
- **Web STOP** (`/safety/stop`) genuinely stops all movement: in RL mode the node
  enters the latched `ESTOP` phase and damps the motors (soft collapse to the
  ground); in sport mode it issues `SportClient.Damp()`. Mode changes are blocked
  while latched.
- **Web RESUME** (`/safety/resume`) gets the robot back up: in RL mode the node
  ramps from the collapsed pose back to the default pose and resumes the policy
  (joystick-ready); in sport mode it issues `RecoveryStand()` and re-enables teleop.
- The **physical E-stop** remains the true hardware cut for any emergency.

### Crash / exit recovery (so the robot is never stranded)

- **Ctrl-C or `kill` (SIGINT/SIGTERM)** — the node traps the signal and runs a safe
  shutdown: if it had released the sport service it **recovers it** (`SelectMode("normal")`
  + `BalanceStand`), then publishes `sport` mode + re-enables `web_bridge`. The robot
  stands and the website drives it again. **Always prefer pressing RL: OFF**, but a
  Ctrl-C will no longer strand the robot.
- **Hard crash (SIGKILL / segfault)** — the node publishes a 5 Hz heartbeat; the web
  bridge watchdog notices it stop and, after ~2 s, **auto-reverts to sport** so
  `web_bridge` un-gates. Note: if the policy had released the sport service, the web
  side cannot revive it — **power-cycle the robot** to restore sport in that case.
- The UI can always force normal mode: `POST /control_mode/sport` works even while
  STOP is latched, and RESUME always re-enables `web_bridge`.

## Known limitation (this build)

`base_lin_vel` (obs 0:3) is fed zeros because it is not observable in low-level
mode and we did **not** retrain. The policy was trained expecting a real value,
so the gait will be rougher and may drift. To fix properly: retrain a 45-dim
policy without `base_lin_vel` (Phase 0 in the deployment plan) and drop in the
new `policy.onnx` — the node auto-detects a 45-dim model and omits the term.
