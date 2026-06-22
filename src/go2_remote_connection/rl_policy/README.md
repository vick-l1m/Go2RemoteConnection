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
| `go2_rl_policy_node.py` | the controller: `rt/lowstate` → 48-dim obs → `policy.onnx` → `rt/lowcmd` @ 50 Hz, plus sport-service handover and safety fallbacks |
| `policy.onnx` | exported flat policy (copy of `logs/rsl_rl/unitree_go2_flat/<ts>/exported/policy.onnx`) |
| `joint_names.json` | Isaac-Lab joint order — **verify before ground tests** |
| `dump_isaac_joint_order.py` | run in the Isaac Lab env to regenerate `joint_names.json` from the real env |

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
python ~/go2_ws/Go2RemoteConnection/src/go2_remote_connection/rl_policy/dump_isaac_joint_order.py
# copy the regenerated joint_names.json back next to policy.onnx
```

## Bring-up (gantry first — do not skip a step)

**0. Robot on a stand, feet off the ground. E-stop in hand.**

**1. Dry run** — validate obs/inference/remap with no torque (motors limp):

```bash
cd ~/go2_ws/Go2RemoteConnection/src/go2_remote_connection/rl_policy
~/venvs/unitree_sdk2_python/bin/python3 go2_rl_policy_node.py --net <iface> --dry-run --no-handover
# in another shell, publish a mode + a command and watch the logs:
ros2 topic pub -1 /web_control_mode std_msgs/String "{data: 'rl'}"
ros2 topic pub -r 10 /web_teleop geometry_msgs/Twist "{linear: {x: 0.3}}"
```

In `--dry-run`, `kp=kd=0` so nothing moves; you're confirming the node reads
state, builds a finite 48-dim obs, and produces sane targets at 50 Hz.

**2. Powered hold** — drop `--dry-run`. Keep `--no-handover` only if you have
manually stopped the sport service; otherwise omit it so the node releases it:

```bash
~/venvs/unitree_sdk2_python/bin/python3 go2_rl_policy_node.py --net <iface>
ros2 topic pub -1 /web_control_mode std_msgs/String "{data: 'rl'}"
```

Expect: crouch → ramp to default pose → policy holds a stance (it settles ~0.2 rad
off `q_default`; a small adjustment is normal). **No buzzing, no divergence** is
the go/no-go gate. Send tiny commands (`x: 0.2`) and watch coordinated stepping
**in the air**.

**3. Full web integration** — start the stack with the node enabled:

```bash
GO2_RL_POLICY=1 ./start_remote_connection.sh
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

## Known limitation (this build)

`base_lin_vel` (obs 0:3) is fed zeros because it is not observable in low-level
mode and we did **not** retrain. The policy was trained expecting a real value,
so the gait will be rougher and may drift. To fix properly: retrain a 45-dim
policy without `base_lin_vel` (Phase 0 in the deployment plan) and drop in the
new `policy.onnx` — the node auto-detects a 45-dim model and omits the term.
