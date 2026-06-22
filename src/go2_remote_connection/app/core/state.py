"""

state.py

This module defines the RuntimeState dataclass to manage the application's runtime state, including flags for shutdown, stop latching, and teleoperation status. It also defines a set of allowed actions that can be performed by the robot.

Version 2.0
Author: Victor Lim

"""
from dataclasses import dataclass

@dataclass
class RuntimeState:
    shutting_down: bool = False
    stop_latched: bool = False
    teleop_enabled: bool = True
    control_mode: str = "sport"   # "sport" (Unitree SportClient) | "rl" (low-level RL policy)

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