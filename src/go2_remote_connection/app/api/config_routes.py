"""

config_routes.py
Config routes for the Go2 Remote Actions application.

Version: 2.0
Author: Victor Lim


"""

from fastapi import APIRouter
from app.core.config import AUTH_ENABLED

router = APIRouter()

@router.get("/config")
async def config():
    return {"auth_enabled": AUTH_ENABLED}