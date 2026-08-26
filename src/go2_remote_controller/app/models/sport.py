"""

sport.py
Data models for the sport API endpoints in the Go2 Remote Actions application.

Version 2.0
Author: Victor Lim

"""

from typing import Optional
from pydantic import BaseModel, Field

class SportCmdReq(BaseModel):
    cmd: str = Field(..., min_length=1)
    enable: Optional[bool] = None
    level: Optional[int] = None
    roll: Optional[float] = None
    pitch: Optional[float] = None
    yaw: Optional[float] = None