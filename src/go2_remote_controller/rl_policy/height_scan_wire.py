"""Wire format for carrying a height scan across the RL controller's UDP link.

WHY THIS EXISTS. The RL controller is split in two processes because
``unitree_sdk2py`` and ``rmw_cyclonedds`` cannot both create DDS domain 0 in one
process (see ``go2_rl_policy_node.py``'s header). The perception half of a rough
policy's observation is produced on the ROS side -- ``go2_perception/heightmap_node``
turns the D435i depth cloud into a ``go2_msgs/HeightScan`` -- but consumed on the SDK
side, where there is no rclpy to subscribe with. So the scan has to cross the same
localhost UDP link the joystick already uses.

WHY NOT JSON, like every other message on that link. The control messages are a
handful of scalars a few times a second; a height scan is 187 floats at the camera's
frame rate. JSON-encoding it costs roughly 2.5 kB and a parse per frame on a Jetson
that is also running inference at 50 Hz, for numbers that are already a packed float
array. This is a fixed-layout binary frame instead: one ``struct.pack``, one
``np.frombuffer``, no per-element work.

FRAMING. Datagrams on this link are otherwise JSON objects, so every frame here starts
with a magic prefix the receiver can test before attempting ``json.loads``. JSON can
never begin with these bytes, so the two are unambiguous on one socket.

    offset  type      field
    0       4s        magic  b"HS01"
    4       uint16    num_x            cells along +x (forward)
    6       uint16    num_y            cells along +y (left)
    8       float32   resolution       cell size [m]
    12      float32   center_x         grid centre ahead of the base [m]
    16      float32   center_y         grid centre left of the base [m]
    20      uint32    unobserved_count cells the camera could not fill
    24      uint32    seq              sender's frame counter (drop detection)
    28      float32[] heights          num_x * num_y, x varying fastest

The geometry travels WITH the values on purpose. ``HeightScan.msg`` makes the point:
a scan whose resolution, extent or centre offset differs from training fails silently
-- the robot walks, badly, and nothing reports a mismatch. Carrying the geometry lets
the receiver check it against what the policy trained on and refuse instead.

Values are NOT transformed here. ``heightmap_node`` has already applied the training
pipeline (``clip(sensor_z - hit_z - offset, clip_min, clip_max)``) and filled
unobservable cells with ``unobserved_value``; a consumer that offsets or clips again
is feeding the policy something it never saw.

No ROS and no numpy import at module scope beyond numpy itself, so both halves of the
split can import it -- the bridge under rclpy, the controller under the SDK venv.
"""

from __future__ import annotations

import struct

import numpy as np

MAGIC = b"HS01"

#: ``struct`` layout of the fixed header, network-independent little-endian.
_HEADER = struct.Struct("<4sHHfffII")
HEADER_SIZE = _HEADER.size  # 28

#: Largest grid this codec will encode or accept. The trained grid is 17 x 11 = 187;
#: the cap exists so a malformed length field cannot make the receiver allocate wildly.
MAX_CELLS = 4096

#: Datagram buffer the receiver should use. MAX_CELLS floats plus the header, rounded
#: up -- so a future finer grid does not silently truncate at the socket.
MAX_DATAGRAM = HEADER_SIZE + MAX_CELLS * 4 + 64


class HeightScanWireError(ValueError):
    """Raised when a datagram is not a decodable height-scan frame."""


def is_height_scan(data: bytes) -> bool:
    """True if ``data`` looks like a height-scan frame rather than a JSON message.

    Cheap enough to call on every datagram before the JSON path.
    """
    return len(data) >= HEADER_SIZE and data[:4] == MAGIC


def encode(
    heights,
    *,
    num_x: int,
    num_y: int,
    resolution: float,
    center_x: float,
    center_y: float,
    unobserved_count: int = 0,
    seq: int = 0,
) -> bytes:
    """Pack one height scan into a single datagram.

    ``heights`` must already be in RayCaster order (``heights[iy * num_x + ix]``,
    x varying fastest). It is not re-sorted here -- re-sorting is exactly the silent
    scramble ``HeightScan.msg`` warns consumers not to do.
    """
    values = np.asarray(heights, dtype=np.float32).reshape(-1)
    if values.size != num_x * num_y:
        raise HeightScanWireError(
            f"heights has {values.size} values but the grid is {num_x}x{num_y} = {num_x * num_y}"
        )
    if values.size > MAX_CELLS:
        raise HeightScanWireError(f"grid of {values.size} cells exceeds MAX_CELLS={MAX_CELLS}")
    header = _HEADER.pack(
        MAGIC,
        int(num_x),
        int(num_y),
        float(resolution),
        float(center_x),
        float(center_y),
        int(unobserved_count) & 0xFFFFFFFF,
        int(seq) & 0xFFFFFFFF,
    )
    # tobytes() on a C-contiguous float32 array is already little-endian on every
    # platform this runs on (x86-64 laptop, aarch64 Jetson).
    return header + np.ascontiguousarray(values).tobytes()


def decode(data: bytes) -> dict:
    """Unpack a datagram produced by :func:`encode`.

    Returns a dict with the geometry fields plus ``heights`` (a float32 ndarray) and
    ``seq``. Raises :class:`HeightScanWireError` on anything malformed -- the caller
    is expected to drop the frame and keep its previous scan, not to guess.
    """
    if not is_height_scan(data):
        raise HeightScanWireError("datagram is not a height-scan frame")
    magic, num_x, num_y, resolution, center_x, center_y, unobserved, seq = _HEADER.unpack_from(data)
    del magic  # already checked by is_height_scan
    cells = num_x * num_y
    if cells == 0 or cells > MAX_CELLS:
        raise HeightScanWireError(f"implausible grid {num_x}x{num_y} ({cells} cells)")
    expected = HEADER_SIZE + cells * 4
    if len(data) != expected:
        raise HeightScanWireError(
            f"frame is {len(data)} bytes but a {num_x}x{num_y} grid needs exactly {expected}"
        )
    heights = np.frombuffer(data, dtype="<f4", count=cells, offset=HEADER_SIZE)
    return {
        "num_x": num_x,
        "num_y": num_y,
        "resolution": float(resolution),
        "center_x": float(center_x),
        "center_y": float(center_y),
        "unobserved_count": int(unobserved),
        "seq": int(seq),
        # Copy: np.frombuffer views the datagram, which the socket reuses.
        "heights": np.array(heights, dtype=np.float32),
    }
