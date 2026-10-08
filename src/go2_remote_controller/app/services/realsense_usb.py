"""
realsense_usb.py
Is a RealSense plugged in, and on what kind of USB link?

Answered from Linux sysfs alone (/sys/bus/usb/devices), never by opening the
device: librealsense is not needed, it works before/without the ROS driver, and
it cannot fight realsense2_camera for the camera. Mirrors detect_realsense() in
RL_start_remote_connection.sh -- keep the two in step.

Link speed is what matters on the robot: the D435i on a 480 Mb/s (USB 2) port
cannot carry colour + point cloud, so the launcher picks the depth-only
real_perception_usb2.launch.py there (5000 Mb/s and up = USB 3, full stack).

Version: 1.0
Author: Victor Lim
"""

import os
import threading
import time
from typing import Any, Dict, Optional

INTEL_VENDOR_ID = "8086"
SYSFS_USB_ROOT = os.getenv("GO2_SYSFS_USB_ROOT", "/sys/bus/usb/devices")
_RESCAN_S = 2.0


def _read(path: str) -> Optional[str]:
    try:
        with open(path, "r", encoding="utf-8", errors="replace") as f:
            return f.read().strip()
    except OSError:
        return None


def classify_speed(speed_mbps: Optional[int]) -> Optional[str]:
    """480 -> 'USB 2'; 5000/10000/20000 -> 'USB 3'; 12/1.5 -> 'USB 1'; None -> None."""
    if speed_mbps is None:
        return None
    if speed_mbps >= 5000:
        return "USB 3"
    if speed_mbps >= 480:
        return "USB 2"
    return "USB 1"


def scan_realsense(root: str = SYSFS_USB_ROOT) -> Dict[str, Any]:
    """Scan sysfs once. Returns a JSON-ready dict; ``connected`` False when absent."""
    result: Dict[str, Any] = {
        "connected": False,
        "product": None,
        "serial": None,
        "speed_mbps": None,
        "usb_class": None,
        "usb_version": None,
        "sysfs_path": None,
    }
    try:
        entries = sorted(os.listdir(root))
    except OSError:
        return result

    for name in entries:
        dev = os.path.join(root, name)
        if _read(os.path.join(dev, "idVendor")) != INTEL_VENDOR_ID:
            continue
        product = _read(os.path.join(dev, "product")) or ""
        if "realsense" not in product.lower():
            continue
        speed_raw = _read(os.path.join(dev, "speed"))
        speed = None
        if speed_raw:
            try:
                speed = int(float(speed_raw))
            except ValueError:
                speed = None
        result.update({
            "connected": True,
            "product": product,
            "serial": _read(os.path.join(dev, "serial")),
            "speed_mbps": speed,
            "usb_class": classify_speed(speed),
            "usb_version": (_read(os.path.join(dev, "version")) or "").strip() or None,
            "sysfs_path": dev,
        })
        return result
    return result


_cache_lock = threading.Lock()
_cache: Optional[Dict[str, Any]] = None
_cache_t = 0.0


def get_realsense_status() -> Dict[str, Any]:
    """Cached scan_realsense(): rescans at most every 2 s. Thread-safe (the ROS
    spin thread and the asyncio loop both call it)."""
    global _cache, _cache_t
    now = time.monotonic()
    with _cache_lock:
        if _cache is None or (now - _cache_t) >= _RESCAN_S:
            _cache = scan_realsense()
            _cache_t = now
        return dict(_cache)
