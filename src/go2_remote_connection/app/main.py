"""
main.py

This module initializes the FastAPI application, sets up CORS middleware,
mounts static files, and includes all API routers for the Go2 Remote Actions
application. It also defines startup and shutdown event handlers to manage
application lifecycle events.

Version 2.0
Author: Victor Lim
"""

import os

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse
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