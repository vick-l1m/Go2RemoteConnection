"""Grid-map unpack round-trip and image -> JPEG encoding (no ROS)."""

import numpy as np
import pytest

from app.services import perception_codec as pc
from app.services.rate_meter import RateMeter


def test_grid_map_round_trip_is_x_fastest():
    n_x, n_y = 17, 11
    grid = np.arange(n_x * n_y, dtype=np.float32).reshape(n_x, n_y)   # grid[ix, iy]
    grid[3, 4] = np.nan
    flat = pc.pack_grid_map_layer(grid)
    out = pc.unpack_grid_map_layer(flat, n_x, n_y)
    assert len(out) == n_x * n_y
    for iy in range(n_y):
        for ix in range(n_x):
            v = out[iy * n_x + ix]
            if (ix, iy) == (3, 4):
                assert v is None
            else:
                assert v == pytest.approx(float(grid[ix, iy]))


def test_grid_map_unpack_matches_go2_viz_inverse():
    # Same inverse as go2_viz/terrain_columns.grid_from_layer (rows = n_x, cols = n_y).
    n_x, n_y = 4, 3
    grid = np.random.default_rng(0).random((n_x, n_y)).astype(np.float32)
    flat = pc.pack_grid_map_layer(grid)
    viz = np.asarray(flat, dtype=np.float32).reshape((n_x, n_y), order="F")[::-1, ::-1]
    assert np.allclose(viz, grid)


def test_grid_map_size_mismatch():
    with pytest.raises(ValueError):
        pc.unpack_grid_map_layer([0.0] * 5, 2, 2)


def test_depth_to_u16_16uc1_little_endian():
    h, w = 2, 3
    img = np.array([[0, 500, 1000], [1500, 2000, 65535]], dtype="<u2")
    out = pc.depth_to_u16(img.tobytes(), h, w, "16UC1", step=w * 2)
    assert out.shape == (h, w)
    assert out[1, 1] == 2000


def test_depth_to_u16_32fc1_metres():
    img = np.array([[0.5, np.nan]], dtype="<f4")
    out = pc.depth_to_u16(img.tobytes(), 1, 2, "32FC1", step=8)
    assert out[0, 0] == 500 and out[0, 1] == 0


def test_depth_to_u16_unknown_encoding():
    assert pc.depth_to_u16(b"\0" * 8, 1, 2, "rgb8", step=6) is None


@pytest.mark.skipif(not pc.HAVE_CV2, reason="cv2 not installed")
def test_depth_image_to_jpeg():
    h, w = 60, 80
    depth = np.linspace(0, 5000, h * w, dtype=np.float32).astype("<u2").reshape(h, w)
    jpg = pc.depth_image_to_jpeg(depth.tobytes(), h, w, "16UC1", step=w * 2,
                                 preview_width=40)
    assert jpg is not None and jpg[:2] == b"\xff\xd8"      # JPEG SOI marker


@pytest.mark.skipif(not pc.HAVE_CV2, reason="cv2 not installed")
def test_color_image_to_jpeg_rgb8():
    h, w = 30, 40
    rgb = np.zeros((h, w, 3), dtype=np.uint8)
    rgb[..., 0] = 255
    jpg = pc.color_image_to_jpeg(rgb.tobytes(), h, w, "rgb8", step=w * 3, preview_width=20)
    assert jpg is not None and jpg[:2] == b"\xff\xd8"


def test_color_unknown_encoding():
    assert pc.color_image_to_jpeg(b"", 1, 1, "yuv422", step=2) is None


def test_rate_meter():
    m = RateMeter(window_s=10.0)
    assert m.hz() == 0.0 and m.age_s() is None and not m.alive()
    t0 = 1000.0
    for i in range(11):
        m.tick(t0 + i * 0.1)
    now = t0 + 1.0
    assert m.hz(now) == pytest.approx(10.0, rel=0.05)
    assert m.age_s(now) == pytest.approx(0.0, abs=1e-9)
    assert m.alive(2.0, now) and not m.alive(2.0, now + 5.0)
    assert m.hz(now + 20.0) == 0.0            # everything aged out of the window
