"""

auth.py
This module defines the authentication mechanism for the Go2 Remote Actions application.

Version: 2.0
Author: Victor Lim

"""

import hmac

from fastapi import Header, HTTPException

from app.core.config import AUTH_ENABLED, GO2_API_TOKEN


if AUTH_ENABLED and not GO2_API_TOKEN:
    raise RuntimeError("GO2_API_TOKEN is not set (or is empty) while GO2_AUTH_ENABLED=1")


def require_token(authorization: str = Header(None)) -> str:
    """
    HTTP Bearer token dependency.

    - If auth is disabled, allow all requests.
    - If auth is enabled, require:
        Authorization: Bearer <token>
    """
    if not AUTH_ENABLED:
        return ""

    if not GO2_API_TOKEN:
        raise HTTPException(status_code=500, detail="Server missing GO2_API_TOKEN")

    if not authorization:
        raise HTTPException(status_code=401, detail="Missing Authorization header")

    parts = authorization.split(" ", 1)
    if len(parts) != 2 or parts[0].lower() != "bearer":
        raise HTTPException(status_code=401, detail="Invalid Authorization header format")

    token = parts[1].strip()
    if not hmac.compare_digest(token, GO2_API_TOKEN):
        raise HTTPException(status_code=403, detail="Invalid token")

    return token