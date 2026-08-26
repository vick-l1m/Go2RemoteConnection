"""

config.py
Config variables for the Go2 Remote Actions application.

Version 2.0

Author: Victor Lim
"""

import os

AUTH_ENABLED = os.getenv("GO2_AUTH_ENABLED", "1") not in ("0", "false", "False")
GO2_API_TOKEN = (os.getenv("GO2_API_TOKEN") or "").strip()
DEPLOYMENT_ENV = os.getenv("DEPLOYMENT_ENV", "").strip().lower()

MAX_LIN = 0.6
MAX_ANG = 1.2