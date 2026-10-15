"""Which joystick is driving? One client at a time.

Every web page with a joystick posts /teleop every 120 ms, idle or not, because the
robot-side dead-man wants a steady stream. With two pages open (a phone and a laptop, or
two tabs) the idle one posts zeros between the driver's commands, and the policy node,
which takes the newest Twist, sees the command flip 0.75 -> 0 -> 0.75 several times a
second. On 2026-10-15 (sessions/20261015-094538_field10_a) that start/stop chatter put
the RL policy on the floor within 1.5 s of the first step -- the counterfactual replay
had already cleared the height scan.

Rule: the client that most recently sent a NON-ZERO command is the driver. Its commands,
zero or not, always go through (releasing the stick must stop the robot). Another
client's ZERO is dropped while the driver has been heard from within ``hold_s``; another
client's NON-ZERO always goes through (last writer wins, as before) and takes over as
driver. A client with no id behaves as before (always published).

Pure Python, no FastAPI, so it is unit tested directly.
"""

from __future__ import annotations

import time


class TeleopArbiter:
    def __init__(self, hold_s: float = 0.5, clock=time.monotonic):
        self.hold_s = float(hold_s)
        self._clock = clock
        self.driver: str | None = None
        self.driver_t: float = 0.0
        self.dropped: int = 0
        self.contested: int = 0

    def accept(self, client: str, lx: float, ly: float, az: float) -> bool:
        """True if this command should be published."""
        if not client:
            return True
        now = self._clock()
        is_zero = max(abs(lx), abs(ly), abs(az)) < 1e-6
        other_driving = (self.driver is not None and self.driver != client
                         and (now - self.driver_t) < self.hold_s)
        if is_zero:
            if other_driving:
                self.dropped += 1
                return False
            if self.driver == client:
                self.driver_t = now
            return True
        if other_driving:
            self.contested += 1
        self.driver, self.driver_t = client, now
        return True

    def status(self) -> dict:
        return {"driver": self.driver, "driver_age_s": (self._clock() - self.driver_t) if self.driver else None,
                "dropped_zero_cmds": self.dropped, "contested_cmds": self.contested}
