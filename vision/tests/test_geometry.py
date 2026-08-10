"""Camera geometry — tested exactly, because it is exact.

The controller-level tests can only check "did the coupling get smaller".
Here the answer has a closed form: applying an attitude and then undoing it
must return the original pixel. Round-trip tests catch sign errors, degree/
radian mix-ups and rotate-about-the-wrong-point bugs that a tolerance-based
integration test will happily pass.
"""
from __future__ import annotations

import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from pursuit.config import CameraConfig                          # noqa: E402
from pursuit.geometry import (distance_from_height,              # noqa: E402
                              height_px_for_distance, level_box,
                              level_point)

W, H = 1280, 720
CX, CY = W / 2, H / 2


def focal():
    return CameraConfig().focal_px(W)


def apply_attitude(u, v, roll_deg, pitch_deg, f):
    """Forward model: where a fixed point APPEARS when the drone is tilted.

    Written independently of `level_point` (which is the inverse) so the pair
    can be checked against each other rather than against themselves.
    """
    du, dv = u - CX, v - CY
    # Nose-down moves fixed objects up the image.
    dv -= f * math.tan(math.radians(pitch_deg))
    # Banking right makes the world appear to rotate left.
    r = math.radians(-roll_deg)
    du, dv = du * math.cos(r) - dv * math.sin(r), du * math.sin(r) + dv * math.cos(r)
    return CX + du, CY + dv


# ------------------------------------------------------------------- lens --

def test_focal_length_matches_the_field_of_view():
    cam = CameraConfig(hfov_deg=90.0)
    # At 90 degrees horizontal, the focal length is exactly half the width.
    assert cam.focal_px(W) == pytest.approx(W / 2, rel=1e-6)

    cam = CameraConfig(hfov_deg=65.0)
    f = cam.focal_px(W)
    # ...and the edge of the frame is at half the FOV from the axis.
    assert math.degrees(math.atan((W / 2) / f)) == pytest.approx(32.5, abs=1e-6)


# -------------------------------------------------------------- round trip --

@pytest.mark.parametrize("roll,pitch", [
    (0, 0), (0, 12), (0, -12), (20, 0), (-20, 0),
    (15, 10), (-25, -8), (5, 30),
])
@pytest.mark.parametrize("point", [(CX, CY), (100, 80), (1200, 650), (CX, 100)])
def test_levelling_inverts_the_attitude_exactly(roll, pitch, point):
    f = focal()
    seen = apply_attitude(point[0], point[1], roll, pitch, f)
    back = level_point(seen[0], seen[1], roll, pitch, f, CX, CY)
    assert back[0] == pytest.approx(point[0], abs=1e-6)
    assert back[1] == pytest.approx(point[1], abs=1e-6)


def test_level_attitude_changes_nothing():
    f = focal()
    assert level_point(400.0, 300.0, 0.0, 0.0, f, CX, CY) == (400.0, 300.0)


def test_nose_down_correction_pushes_the_point_back_down():
    """Sign check, stated the way the bug was described: pitching forward makes
    the subject appear higher, so levelling must move them back down."""
    f = focal()
    _, v = level_point(CX, CY, 0.0, 15.0, f, CX, CY)
    assert v > CY
    assert v - CY == pytest.approx(f * math.tan(math.radians(15.0)), abs=1e-6)


def test_extreme_pitch_does_not_blow_up():
    f = focal()
    for pitch in (89.0, 90.0, 120.0, -90.0):
        u, v = level_point(CX, CY, 0.0, pitch, f, CX, CY)
        assert math.isfinite(u) and math.isfinite(v)


# --------------------------------------------------------------- box level --

def test_levelling_a_box_preserves_its_size():
    """The box height is the distance cue. Re-fitting an axis-aligned box
    around a rotated one would inflate it with the roll angle, and the drone
    would think the subject stepped closer every time it banked."""
    box = np.array([500.0, 200.0, 620.0, 560.0], dtype=np.float32)
    out = level_box(box, 25.0, 10.0, focal(), W, H)
    assert (out[2] - out[0]) == pytest.approx(box[2] - box[0], abs=1e-4)
    assert (out[3] - out[1]) == pytest.approx(box[3] - box[1], abs=1e-4)


def test_levelling_a_box_moves_its_centre_like_a_point():
    box = np.array([500.0, 200.0, 620.0, 560.0], dtype=np.float32)
    f = focal()
    out = level_box(box, 18.0, -7.0, f, W, H)
    want = level_point((box[0] + box[2]) / 2, (box[1] + box[3]) / 2,
                       18.0, -7.0, f, CX, CY)
    assert (out[0] + out[2]) / 2 == pytest.approx(want[0], abs=1e-4)
    assert (out[1] + out[3]) / 2 == pytest.approx(want[1], abs=1e-4)


# ----------------------------------------------------------------- ranging --

def test_distance_from_height_is_the_pinhole_relation():
    f = focal()
    # 1.75m person whose box is 500px tall.
    d = distance_from_height(500.0, 1.75, f)
    assert d == pytest.approx(1.75 * f / 500.0, rel=1e-9)
    # Twice as far -> half the pixels.
    assert distance_from_height(250.0, 1.75, f) == pytest.approx(2 * d, rel=1e-9)


def test_range_and_height_are_inverses():
    f = focal()
    for dist in (1.0, 2.5, 3.0, 8.0, 20.0):
        px = height_px_for_distance(dist, 1.80, f)
        assert distance_from_height(px, 1.80, f) == pytest.approx(dist, rel=1e-6)


def test_degenerate_inputs_do_not_raise():
    f = focal()
    assert distance_from_height(0.0, 1.75, f) == float("inf")
    assert distance_from_height(500.0, 0.0, f) == float("inf")
    assert height_px_for_distance(0.0, 1.75, f) == 0.0
