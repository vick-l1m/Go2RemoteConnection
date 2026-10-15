"""

teleop.py
This module defines the TeleopCommand Pydantic model for representing teleoperation commands sent to the robot, including linear and angular velocity components.

Version 2.0
Author: Victor Lim

"""

from pydantic import BaseModel, Field

class TeleopCommand(BaseModel):
    linear_x: float = Field(0.0)
    linear_y: float = Field(0.0)
    angular_z: float = Field(0.0)
    # Per-page id (shared.js attaches it). The /teleop arbiter uses it to drop an idle
    # page's zeros while another page is driving; empty = legacy sender, always published.
    client: str = Field("")