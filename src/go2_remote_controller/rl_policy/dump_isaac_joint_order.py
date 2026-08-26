"""Dump the Isaac-Lab articulation joint order to joint_names.json.

Run this in the ISAAC LAB env (env_isaaclab), NOT on the robot. It loads the
same flat task the policy was trained on and prints/writes the exact joint
order that the policy's action vector and joint observations use. Copy the
resulting joint_names.json next to policy.onnx.

The order matters: Isaac Lab orders joints by *type* (all hips, all thighs,
all calves), which is NOT the Unitree SDK order. go2_rl_policy_node.py uses
this file to build the SDK<->Isaac index remap. A wrong order silently moves
the wrong leg.

Usage (from ~/IsaacLab):
    conda activate env_isaaclab
    python dump_isaac_joint_order.py
"""

import json
import pathlib

from isaaclab.app import AppLauncher

# Headless: we only need to parse the articulation, not render.
app_launcher = AppLauncher({"headless": True})
simulation_app = app_launcher.app

import gymnasium as gym  # noqa: E402  (must follow AppLauncher)
import isaaclab_tasks  # noqa: E402,F401  (registers Isaac-* tasks)
from isaaclab_tasks.utils import parse_env_cfg  # noqa: E402

TASK = "Isaac-Velocity-Flat-Unitree-Go2-v0"

env_cfg = parse_env_cfg(TASK, num_envs=1)
env = gym.make(TASK, cfg=env_cfg)
robot = env.unwrapped.scene["robot"]
joint_names = list(robot.data.joint_names)

print("Isaac-Lab joint order:")
for i, n in enumerate(joint_names):
    print(f"  {i:2d}  {n}")

out = pathlib.Path(__file__).with_name("joint_names.json")
out.write_text(json.dumps({"joint_names": joint_names}, indent=2))
print(f"\nwrote {out}")

env.close()
simulation_app.close()
