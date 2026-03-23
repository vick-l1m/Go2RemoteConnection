"""
yolo_service.py
This module manages the YOLO process for the Go2 Remote Actions application.

Version: 2.1
Author: Victor Lim
"""

import os
import signal
import subprocess
import time
from typing import Optional

from app.ros_bridge import get_yolo_store, get_yolo_cam_store

YOLO_PROC: Optional[subprocess.Popen] = None
YOLO_LOGF = None


def yolo_script_path() -> str:
    home = os.path.expanduser("~")
    return os.path.join(
        home,
        "go2_ws",
        "Go2RemoteConnection",
        "src",
        "go2_remote_connection",
        "src",
        "cv",
        "ROS_yolo.py",
    )


def venv_python_path() -> str:
    home = os.path.expanduser("~")
    return os.path.join(home, "venvs", "unitree_sdk2_python", "bin", "python3")


def _clear_yolo_state():
    yolo_store = get_yolo_store()
    yolo_cam_store = get_yolo_cam_store()

    yolo_store.last_json = None
    yolo_store.seq = 0

    yolo_cam_store.meta = None
    yolo_cam_store.jpg = None
    yolo_cam_store.seq = 0


def start_yolo_process():
    global YOLO_PROC, YOLO_LOGF

    if YOLO_PROC is not None and YOLO_PROC.poll() is None:
        return {"ok": True, "message": "YOLO already running", "pid": YOLO_PROC.pid}

    YOLO_PROC = None
    _clear_yolo_state()

    env = os.environ.copy()
    home = os.path.expanduser("~")
    unitree_sdk_src = os.path.join(home, "unitree_sdk2_python")
    venv_site = os.path.join(home, "venvs", "unitree_sdk2_python", "lib", "python3.10", "site-packages")
    existing_pp = env.get("PYTHONPATH", "")
    env["PYTHONNOUSERSITE"] = "1"
    env["PYTHONPATH"] = f"{unitree_sdk_src}:{venv_site}:{existing_pp}"

    log_path = "/tmp/ROS_yolo.log"
    YOLO_LOGF = open(log_path, "ab", buffering=0)

    YOLO_PROC = subprocess.Popen(
        [venv_python_path(), yolo_script_path()],
        stdout=YOLO_LOGF,
        stderr=subprocess.STDOUT,
        env=env,
        preexec_fn=os.setsid,
    )

    return {"ok": True, "message": "YOLO started", "pid": YOLO_PROC.pid}


def stop_yolo_process():
    global YOLO_PROC, YOLO_LOGF

    stopped = False

    if YOLO_PROC is not None and YOLO_PROC.poll() is None:
        try:
            pgid = os.getpgid(YOLO_PROC.pid)
            os.killpg(pgid, signal.SIGTERM)
            stopped = True

            for _ in range(10):
                if YOLO_PROC.poll() is not None:
                    break
                time.sleep(0.1)

            if YOLO_PROC.poll() is None:
                os.killpg(pgid, signal.SIGKILL)
        except Exception:
            try:
                YOLO_PROC.terminate()
                stopped = True
            except Exception:
                pass

    YOLO_PROC = None

    if YOLO_LOGF is not None:
        try:
            YOLO_LOGF.close()
        except Exception:
            pass
        YOLO_LOGF = None

    try:
        subprocess.run(
            ["pkill", "-f", yolo_script_path()],
            check=False,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
    except Exception:
        pass

    _clear_yolo_state()

    return {"ok": True, "message": "YOLO stopped", "stopped": stopped}


def yolo_status():
    running = YOLO_PROC is not None and YOLO_PROC.poll() is None
    pid = YOLO_PROC.pid if running else None
    return {"ok": True, "running": running, "pid": pid}