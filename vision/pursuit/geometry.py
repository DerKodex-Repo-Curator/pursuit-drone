"""Turning image coordinates into something the control loop can trust.

The camera is bolted to the airframe with no gimbal, so every degree the drone
tilts moves the whole image. Left uncorrected that is not a small error — it is
a feedback loop:

    chase forward  ->  pitch nose-down  ->  camera tilts down  ->  the subject
    appears HIGHER in frame  ->  altitude loop reads "target too high"  ->
    climb  ->  ...

At a 14 degree pitch limit and a 65 degree lens, that apparent shift is around
200 pixels — larger than any real movement the subject makes, and in the wrong
direction. The same coupling exists between roll and yaw.

The fix is to ask where the target *would* be if the drone were level, which is
a pure function of the measured attitude and the lens. The telemetry packet
already carries roll and pitch, so nothing new has to be measured.

Convention (matching control.py and the telemetry packet):
    roll  + = right wing down
    pitch + = nose down
    image x right, y down, origin top-left
"""
from __future__ import annotations

import math

import numpy as np


def level_point(u: float, v: float, roll_deg: float, pitch_deg: float,
                focal_px: float, cx: float, cy: float) -> tuple[float, float]:
    """Where (u, v) would appear if the airframe were level.

    Pitch is a translation along the image's vertical axis and roll is a
    rotation about the principal point. Both are exact for a pinhole camera;
    the tangent is what makes pitch non-linear at large angles, which is why
    this is not just `v + k * pitch`.
    """
    du, dv = u - cx, v - cy

    # Undo roll first: it is a rotation of the image about the optical axis.
    # Tilting the aircraft right makes the world appear to rotate left, so the
    # correction rotates back by +roll.
    if roll_deg:
        r = math.radians(roll_deg)
        cos_r, sin_r = math.cos(r), math.sin(r)
        du, dv = du * cos_r - dv * sin_r, du * sin_r + dv * cos_r

    # Undo pitch: nose-down moves fixed objects UP the image, so add the shift
    # back. Guard the tangent — at 90 degrees the projection is undefined and
    # the aircraft has bigger problems.
    if pitch_deg:
        p = math.radians(max(-80.0, min(80.0, pitch_deg)))
        dv += focal_px * math.tan(p)

    return cx + du, cy + dv


def level_box(xyxy: np.ndarray, roll_deg: float, pitch_deg: float,
              focal_px: float, frame_w: int, frame_h: int) -> np.ndarray:
    """Attitude-correct a box by its centre, preserving its size.

    Only the centre is transformed. A rotation would also shear the box, but
    the box is an axis-aligned approximation of a person to begin with, and its
    *height* is the distance cue — re-fitting an axis-aligned box around a
    rotated one inflates that height with the roll angle and would make the
    drone think the subject moved closer every time it banked.
    """
    cx_img, cy_img = frame_w * 0.5, frame_h * 0.5
    x1, y1, x2, y2 = (float(v) for v in xyxy)
    w, h = x2 - x1, y2 - y1

    u, v = (x1 + x2) * 0.5, (y1 + y2) * 0.5
    u2, v2 = level_point(u, v, roll_deg, pitch_deg, focal_px, cx_img, cy_img)

    return np.array([u2 - w * 0.5, v2 - h * 0.5,
                     u2 + w * 0.5, v2 + h * 0.5], dtype=np.float32)


def distance_from_height(box_h_px: float, subject_h_m: float,
                         focal_px: float) -> float:
    """Pinhole range estimate: distance = real height * focal / pixel height.

    Far better than a box-height *fraction* because it is a physical quantity —
    it does not change when you switch camera resolution or lens, and a
    setpoint expressed in metres means the same thing on every build.

    Accurate to roughly the accuracy of `subject_h_m` and the detector's box,
    which is to say ±10-20%. Good enough to hold a standoff, not good enough to
    navigate by.
    """
    if box_h_px <= 1.0 or subject_h_m <= 0.0:
        return float("inf")
    return (subject_h_m * focal_px) / box_h_px


def height_px_for_distance(distance_m: float, subject_h_m: float,
                           focal_px: float) -> float:
    """Inverse of `distance_from_height` — the box height a given range implies.

    Lets the controller keep working in pixels (where its gains are tuned)
    while the *setpoint* is specified in metres.
    """
    if distance_m <= 1e-3 or subject_h_m <= 0.0:
        return 0.0
    return (subject_h_m * focal_px) / distance_m
