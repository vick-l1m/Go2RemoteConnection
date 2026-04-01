"""
main.py

This module initializes the FastAPI application, sets up CORS middleware,
mounts static files, and includes all API routers for the Go2 Remote Actions
application. It also defines startup and shutdown event handlers to manage
application lifecycle events.

Includes P2Dingo Backend Connection 

Version 2.1
Author: Victor Lim
"""

import os
from fastapi import FastAPI, Depends, Header, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse, JSONResponse
from fastapi.staticfiles import StaticFiles

from app.core.lifecycle import on_startup, on_shutdown
from app.api.health import router as health_router
from app.api.config_routes import router as config_router
from app.api.actions import router as actions_router
from app.api.safety import router as safety_router
from app.api.teleop import router as teleop_router
from app.api.movement import router as movement_router
from app.api.sport import router as sport_router
from app.api.map_routes import router as map_router
from app.api.camera_routes import router as camera_router
from app.api.yolo_routes import router as yolo_router
from app.api.terminal_routes import router as terminal_router


def _truthy(v: str) -> bool:
    return str(v).strip().lower() in {"1", "true", "yes", "on"}


GO2_AUTH_ENABLED = _truthy(os.getenv("GO2_AUTH_ENABLED", "0"))
GO2_TOKEN_FILE = os.path.expanduser(os.getenv("GO2_TOKEN_FILE", "~/go2_token"))
GO2_API_TOKEN = os.getenv("GO2_API_TOKEN", "")

if GO2_AUTH_ENABLED and not GO2_API_TOKEN and os.path.isfile(GO2_TOKEN_FILE):
    with open(GO2_TOKEN_FILE, "r", encoding="utf-8") as f:
        GO2_API_TOKEN = f.read().strip()


def require_token_if_enabled(authorization: str | None = Header(default=None)) -> None:
    if not GO2_AUTH_ENABLED:
        return

    if not GO2_API_TOKEN:
        raise HTTPException(status_code=500, detail="Auth enabled but token not configured")

    if not authorization:
        raise HTTPException(status_code=401, detail="Missing Authorization header")

    parts = authorization.split(" ", 1)
    if len(parts) != 2 or parts[0].lower() != "bearer":
        raise HTTPException(status_code=401, detail="Invalid Authorization header format")

    token = parts[1].strip()
    if token != GO2_API_TOKEN:
        raise HTTPException(status_code=403, detail="Invalid token")


app = FastAPI(title="Go2 Remote Actions")

HERE = os.path.dirname(__file__)
STATIC_DIR = os.path.join(HERE, "static")
PAGES_DIR = os.path.join(STATIC_DIR, "pages")

app.mount("/app/static", StaticFiles(directory=STATIC_DIR), name="static")


@app.get("/")
async def index_page():
    return FileResponse(os.path.join(PAGES_DIR, "index.html"))

@app.get("/joystick")
async def joystick_page():
    return FileResponse(os.path.join(PAGES_DIR, "go2_joystick.html"))


@app.get("/movement")
async def movement_page():
    return FileResponse(os.path.join(PAGES_DIR, "go2_movement_controller.html"))


@app.get("/map")
async def map_page():
    return FileResponse(os.path.join(PAGES_DIR, "go2_map_viewer.html"))


@app.get("/terminal")
async def terminal_page():
    return FileResponse(os.path.join(PAGES_DIR, "go2_terminal_only.html"))


@app.get("/other")
async def other_page():
    return FileResponse(os.path.join(PAGES_DIR, "go2_other.html"))


@app.get("/camera")
async def camera_page():
    return FileResponse(os.path.join(PAGES_DIR, "go2_front_camera.html"))


@app.get("/config")
async def config():
    return JSONResponse({
        "auth_enabled": GO2_AUTH_ENABLED,
        "deployment_env": os.getenv("DEPLOYMENT_ENV", "robot"),
        "default_page": "/p2dingo",
    })


app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=False,
    allow_methods=["GET", "POST", "OPTIONS"],
    allow_headers=["Authorization", "Content-Type"],
    expose_headers=[
        "x-map-frame",
        "x-map-resolution",
        "x-map-width",
        "x-map-height",
        "x-map-origin-x",
        "x-map-origin-y",
        "x-map-seq",
        "content-encoding",
    ],
    max_age=86400,
)

app.add_event_handler("startup", on_startup)
app.add_event_handler("shutdown", on_shutdown)

app.include_router(health_router)
app.include_router(config_router)
app.include_router(actions_router)
app.include_router(safety_router)
app.include_router(teleop_router)
app.include_router(movement_router)
app.include_router(sport_router)
app.include_router(map_router)
app.include_router(camera_router)
app.include_router(yolo_router)
app.include_router(terminal_router)