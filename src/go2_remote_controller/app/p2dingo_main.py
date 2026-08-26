"""
p2dingo_main.py

FastAPI entrypoint for the P2Dingo-only deployment.
Serves only the P2Dingo HTML page as the main UI while keeping the same API routers.

Author: Victor Lim
Version 1.1
"""

import os

from fastapi import FastAPI
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

app = FastAPI(title="P2Dingo Remote Actions")

HERE = os.path.dirname(__file__)
STATIC_DIR = os.path.join(HERE, "static")
PAGES_DIR = os.path.join(STATIC_DIR, "pages")

app.mount("/app/static", StaticFiles(directory=STATIC_DIR), name="static")


@app.get("/")
async def root_page():
    return FileResponse(os.path.join(PAGES_DIR, "P2Dingo_controller.html"))


@app.get("/config")
async def config_page():
    auth_enabled = str(os.getenv("GO2_AUTH_ENABLED", "0")).strip().lower() in {
        "1", "true", "yes", "on"
    }
    return JSONResponse(
        {
            "auth_enabled": auth_enabled,
            "default_page": "/",
            "site_name": "P2Dingo Controller",
        }
    )


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