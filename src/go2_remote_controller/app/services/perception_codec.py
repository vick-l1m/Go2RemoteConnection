"""
perception_codec.py
Pure numpy/cv2 helpers behind the RealSense web views -- no rclpy, so they are
unit tested directly:

* depth_image_to_jpeg / color_image_to_jpeg: raw sensor_msgs/Image bytes -> small JPEG
  for the page's image box (same JSON-header + bytes wire format as /ws/cam_front).
* unpack_grid_map_layer: grid_map_msgs packing -> x-fastest flat list, the same
  order go2_msgs/HeightScan.heights uses, so the browser indexes both alike.

cv2 is optional at import: the status endpoint reports its absence and the image
box explains it, instead of the whole bridge failing to start.

Version: 1.0
Author: Victor Lim
"""

from typing import List, Optional, Tuple

import numpy as np

try:
    import cv2  # type: ignore
    HAVE_CV2 = True
except Exception:  # pragma: no cover - environment dependent
    cv2 = None
    HAVE_CV2 = False

DEPTH_MIN_MM = 200
DEPTH_MAX_MM = 4000
PREVIEW_WIDTH = 320
JPEG_QUALITY = 60


def _resize_to_width(img: np.ndarray, width: int) -> np.ndarray:
    h, w = img.shape[:2]
    if w <= width:
        return img
    scale = width / float(w)
    return cv2.resize(img, (width, max(1, int(round(h * scale)))), interpolation=cv2.INTER_AREA)


def _encode(img_bgr: np.ndarray, quality: int) -> Optional[bytes]:
    ok, buf = cv2.imencode(".jpg", img_bgr, [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)])
    return buf.tobytes() if ok else None


def depth_to_u16(data: bytes, height: int, width: int, encoding: str, step: int,
                 is_bigendian: bool = False) -> Optional[np.ndarray]:
    """sensor_msgs/Image depth payload -> (H, W) uint16 millimetres (16UC1 as-is,
    32FC1 metres converted). None for encodings we do not know."""
    enc = encoding.lower()
    if enc in ("16uc1", "mono16"):
        dt = np.dtype(">u2" if is_bigendian else "<u2")
        row = step // 2 if step else width
        arr = np.frombuffer(data, dtype=dt)
        if arr.size < height * row:
            return None
        return arr[: height * row].reshape(height, row)[:, :width].astype(np.uint16, copy=False)
    if enc == "32fc1":
        dt = np.dtype(">f4" if is_bigendian else "<f4")
        row = step // 4 if step else width
        arr = np.frombuffer(data, dtype=dt)
        if arr.size < height * row:
            return None
        m = arr[: height * row].reshape(height, row)[:, :width]
        m = np.nan_to_num(m, nan=0.0, posinf=0.0, neginf=0.0)
        return np.clip(m * 1000.0, 0, 65535).astype(np.uint16)
    return None


def colorize_depth(depth_mm: np.ndarray, dmin: int = DEPTH_MIN_MM,
                   dmax: int = DEPTH_MAX_MM) -> np.ndarray:
    """uint16 mm -> BGR uint8 with a turbo ramp (near = red/yellow, far = blue);
    zero / out-of-range pixels black. Needs cv2."""
    valid = (depth_mm >= dmin) & (depth_mm <= dmax)
    norm = np.zeros(depth_mm.shape, dtype=np.uint8)
    span = float(max(dmax - dmin, 1))
    scaled = (depth_mm.astype(np.float32) - dmin) * (255.0 / span)
    norm[valid] = np.clip(scaled[valid], 0, 255).astype(np.uint8)
    cmap = getattr(cv2, "COLORMAP_TURBO", cv2.COLORMAP_JET)
    bgr = cv2.applyColorMap(255 - norm, cmap)   # invert so near is warm
    bgr[~valid] = 0
    return bgr


def depth_image_to_jpeg(data: bytes, height: int, width: int, encoding: str, step: int,
                        is_bigendian: bool = False, preview_width: int = PREVIEW_WIDTH,
                        quality: int = JPEG_QUALITY) -> Optional[bytes]:
    if not HAVE_CV2:
        return None
    depth = depth_to_u16(data, height, width, encoding, step, is_bigendian)
    if depth is None:
        return None
    return _encode(_resize_to_width(colorize_depth(depth), preview_width), quality)


def color_image_to_jpeg(data: bytes, height: int, width: int, encoding: str, step: int,
                        preview_width: int = PREVIEW_WIDTH,
                        quality: int = JPEG_QUALITY) -> Optional[bytes]:
    if not HAVE_CV2:
        return None
    enc = encoding.lower()
    channels = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4, "mono8": 1}.get(enc)
    if channels is None:
        return None
    row = step if step else width * channels
    arr = np.frombuffer(data, dtype=np.uint8)
    if arr.size < height * row:
        return None
    img = arr[: height * row].reshape(height, row)[:, : width * channels]
    img = img.reshape(height, width, channels) if channels > 1 else img.reshape(height, width)
    if enc == "rgb8":
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    elif enc == "rgba8":
        img = cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
    elif enc == "bgra8":
        img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
    elif enc == "mono8":
        img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    return _encode(_resize_to_width(img, preview_width), quality)


def unpack_grid_map_layer(flat, n_x: int, n_y: int) -> List[Optional[float]]:
    """Inverse of heightmap_node._gridmap_layer.

    The node's grid is ``(n_x, n_y)`` indexed ``[ix, iy]``; it stores
    ``grid[::-1, ::-1]`` column-major (order="F"), i.e. rows = x descending,
    cols = y descending (grid_map convention; dim[0]=column_index=n_y,
    dim[1]=row_index=n_x). Returns the values x-fastest, ``out[iy*n_x + ix]``,
    the same order as HeightScan.heights, with NaN -> None for JSON.
    go2_viz/terrain_columns.grid_from_layer is the same inverse.
    """
    arr = np.asarray(flat, dtype=np.float64)
    if arr.size != n_x * n_y:
        raise ValueError(f"layer has {arr.size} values, expected {n_x}*{n_y}")
    grid = arr.reshape((n_x, n_y), order="F")[::-1, ::-1]      # grid[ix, iy]
    out: List[Optional[float]] = []
    for v in grid.T.reshape(-1):                                 # iy outer, ix inner
        out.append(None if not np.isfinite(v) else round(float(v), 4))
    return out


def pack_grid_map_layer(grid: np.ndarray) -> np.ndarray:
    """heightmap_node's packing, for tests: grid ``[ix, iy]`` -> flat Fortran-order flipped."""
    return grid[::-1, ::-1].flatten(order="F")
