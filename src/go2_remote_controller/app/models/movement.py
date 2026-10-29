from pydantic import BaseModel, Field


class MoveForwardReq(BaseModel):
    meters: float = Field(
        ...,
        ge=0.0,
        le=5.0,
        description="Distance in meters (0–5)",
    )


class RlMoveReq(BaseModel):
    meters: float = Field(
        ...,
        ge=-5.0,
        le=5.0,
        description="Signed distance in meters along the current heading (+ forward, - back)",
    )