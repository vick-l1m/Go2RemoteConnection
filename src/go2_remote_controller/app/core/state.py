"""
state.py

This module defines the RuntimeState dataclass to manage the application's runtime state, including flags for shutdown, stop latching, and teleoperation status. It also defines a set of allowed actions that can be performed by the robot.

Version 2.0
Author: Victor Lim

"""
import os
from dataclasses import dataclass, field

@dataclass
class RuntimeState:
    shutting_down: bool = False
    stop_latched: bool = False
    teleop_enabled: bool = True
    control_mode: str = "sport"   # "sport" (Unitree SportClient) | "rl" (low-level RL policy)
    rl_policy_id: str = ""        # id of the selected/loaded RL policy (see rl_policy/policies.json)
    # Photo mode: the RealSense colour stream switched on in the running driver (for a
    # picture on the USB 2 depth-only launch). Engaging RL is refused while it is on.
    photo_mode: bool = False
    # Rolling-map memory override (s) for a photo, only while photo_mode; None = policy's.
    photo_memory_s: "float | None" = None
    # Height-map snap switch (RL page): "auto" follows the policy's deploy.yaml, "on"/"off"
    # force snapped/raw. Starts from RL_start's GO2_SCAN_SNAP (auto/1/0).
    scan_snap_mode: str = {"1": "on", "0": "off"}.get(os.environ.get("GO2_SCAN_SNAP", ""), "auto")
    rl_posture: str = "stand"     # "stand" | "sit" — posture target for a sit/stand RL policy
    # Latest "Move X m" report from go2_rl_policy_node (/web_rl_move_status):
    # {state, target, travelled, speed, why, stamp}. state: idle | driving | settling |
    # done | cancelled | timeout | refused.
    rl_move: dict = field(default_factory=lambda: {"state": "idle"})

state = RuntimeState()

ALLOWED_ACTIONS = {
    "sit","stand","stop","standdown","recover",
    "mode_movement","mode_posing","mode_actions",
    "toggle_freebound","toggle_freeavoid","toggle_crossstep","toggle_freejump",
    "toggle_walkupright","toggle_handstand","toggle_classicwalk",
    "trot_run","economic_gait","switch_avoid","cycle_speed",
    "stop_move","static_walk","freewalk",
    "damp","balance_stand",
    "sit_toggle","hello","stretch","content","heart","scrape",
    "front_pounce","front_jump","front_flip","back_flip","left_flip","recovery",
}