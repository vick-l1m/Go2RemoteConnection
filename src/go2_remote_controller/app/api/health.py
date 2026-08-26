"""

health.py
This module defines the health check endpoint for the Go2 Remote Actions application.

Version: 2.0
Author: Victor Lim

"""


from fastapi import APIRouter

router = APIRouter()

@router.get("/health")
async def health():
    return {"ok": True, "status": "ok"}