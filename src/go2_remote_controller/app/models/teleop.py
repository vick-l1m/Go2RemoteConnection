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