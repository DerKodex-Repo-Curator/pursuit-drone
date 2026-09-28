"""Controller behaviour, especially the signs and the limits.

A sign error here is not a subtle bug — it is a drone that accelerates away
from the person it is supposed to follow. These tests pin every axis's
direction against the convention written in control.py's docstring.
"""
from __future__ import annotations

import os
import sys
from dataclasses import replace

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from pursuit.config import CameraConfig, ControlConfig          # noqa: E402
from pursuit.control import FollowController, clamp, soft_deadband  # noqa: E402
from pursuit.lock import LockResult, LockState                  # noqa: E402
from pursuit.protocol import FLAG_FAILSAFE, FLAG_LOCKED         # noqa: E402

W, H = 1280, 720


def locked(x1, y1, x2, y2, source="face"):
    return LockResult(LockState.LOCKED, track=None,
                      xyxy=np.array([x1, y1, x2, y2], dtype=np.float32),
                      confidence=0.8, source=source, target_name="t",
                      commanding=True)


def box_at(cx, cy, h_frac=0.45, aspect=0.4):
    bh = h_frac * H
    bw = bh * aspect
    return locked(cx - bw / 2, cy - bh / 2, cx + bw / 2, cy + bh / 2)


def settle(ctl, lock, n=40, dt=1 / 30):
    cmd = None
    for _ in range(n):
        cmd = ctl.update(lock, W, H, dt)
    return cmd


# ------------------------------------------------------------------- helpers --

def test_soft_deadband_is_continuous():
    # A hard deadband steps from 0 to `width` and makes the drone twitch.
    assert soft_deadband(0.03, 0.05) == 0.0
    assert soft_deadband(0.0500001, 0.05) == pytest.approx(0.0, abs=1e-5)
    assert soft_deadband(1.0, 0.05) == pytest.approx(1.0)
    assert soft_deadband(-1.0, 0.05) == pytest.approx(-1.0)


def test_clamp():
    assert clamp(5, 0, 1) == 1
    assert clamp(-5, 0, 1) == 0
    assert clamp(0.5, 0, 1) == 0.5


# --------------------------------------------------------------------- signs --

def test_target_right_yaws_right():
    ctl = FollowController(ControlConfig())
    cmd = settle(ctl, box_at(W * 0.85, H * 0.5))
    assert cmd.yaw_rate > 5.0, "target on the right must command a right yaw"


def test_target_left_yaws_left():
    ctl = FollowController(ControlConfig())
    cmd = settle(ctl, box_at(W * 0.15, H * 0.5))
    assert cmd.yaw_rate < -5.0


def test_target_too_small_pitches_forward():
    # Small in frame = far away = close the distance = nose down.
    ctl = FollowController(ControlConfig())
    cmd = settle(ctl, box_at(W / 2, H / 2, h_frac=0.20))
    assert cmd.pitch > 1.0


def test_target_too_large_pitches_back():
    ctl = FollowController(ControlConfig())
    cmd = settle(ctl, box_at(W / 2, H / 2, h_frac=0.80))
    assert cmd.pitch < -1.0


def test_target_high_in_frame_climbs():
    """Counter-intuitive but correct: raising a forward-facing camera makes a
    fixed object appear lower, so a target sitting high means climb."""
    ctl = FollowController(ControlConfig())
    start = ctl.alt_sp
    # Small and high, so the box clears the top edge — a full-height box up
    # here would be clipped, and the guard would correctly freeze altitude.
    cmd = settle(ctl, box_at(W / 2, H * 0.15, h_frac=0.20))
    assert cmd.alt_sp > start
    assert ctl.vertical_valid


def test_target_low_in_frame_descends():
    ctl = FollowController(ControlConfig())
    start = ctl.alt_sp
    # Small and low, so the box stays clear of the bottom edge — otherwise the
    # clipped-box guard (correctly) refuses to act on the centroid at all.
    cmd = settle(ctl, box_at(W / 2, H * 0.85, h_frac=0.20))
    assert cmd.alt_sp < start
    assert ctl.vertical_valid


def test_clipped_box_freezes_altitude():
    """A box running off the bottom has a centroid biased upward — its legs
    are missing. Acting on it descends, which clips more of the subject, which
    descends further. The guard has to break that loop."""
    ctl = FollowController(ControlConfig())
    start = ctl.alt_sp
    # Subject close to the camera: box extends past the bottom of the frame.
    clipped = locked(W / 2 - 150, H * 0.15, W / 2 + 150, H + 200)
    for _ in range(120):
        cmd = ctl.update(clipped, W, H, 1 / 30)
    assert not ctl.vertical_valid
    assert cmd.alt_sp == pytest.approx(start, abs=1e-6), \
        "altitude ran away on a vertically clipped box"


def test_clipped_at_the_top_also_freezes():
    ctl = FollowController(ControlConfig())
    start = ctl.alt_sp
    clipped = locked(W / 2 - 150, -80, W / 2 + 150, H * 0.7)
    for _ in range(120):
        cmd = ctl.update(clipped, W, H, 1 / 30)
    assert not ctl.vertical_valid
    assert cmd.alt_sp == pytest.approx(start, abs=1e-6)


def test_clipping_does_not_disable_yaw_or_pitch():
    """Only the vertical axis is unreliable — the horizontal centroid and the
    box height are still perfectly good, so following must continue."""
    ctl = FollowController(ControlConfig())
    clipped = locked(W * 0.75, H * 0.15, W * 0.98, H + 200)
    cmd = settle(ctl, clipped)
    assert not ctl.vertical_valid
    assert abs(cmd.yaw_rate) > 5.0, "stopped steering because of a clipped box"


# --------------------------------------------------------------------- limits --

def test_outputs_respect_configured_limits():
    cfg = ControlConfig()
    ctl = FollowController(cfg)
    # Hard against the frame edge and tiny: saturates yaw and pitch together.
    cmd = settle(ctl, box_at(W - 10, H / 2, h_frac=0.02), n=200)
    assert abs(cmd.yaw_rate) <= cfg.yaw_max + 1e-6
    assert abs(cmd.pitch) <= cfg.pitch_max + 1e-6
    assert cfg.alt_min - 1e-6 <= cmd.alt_sp <= cfg.alt_max + 1e-6


def test_slew_rate_is_enforced():
    cfg = ControlConfig()
    ctl = FollowController(cfg)
    dt = 1 / 30
    prev = 0.0
    lock = box_at(W - 10, H / 2)          # step input, maximum demand
    for _ in range(10):
        cmd = ctl.update(lock, W, H, dt)
        assert abs(cmd.yaw_rate - prev) <= cfg.slew_deg_per_s * dt + 1e-6
        prev = cmd.yaw_rate


def test_centred_target_commands_almost_nothing():
    ctl = FollowController(ControlConfig())
    cfg = ControlConfig()
    aim_y = H * 0.5 * (1.0 + cfg.aim_y_bias)
    cmd = settle(ctl, box_at(W / 2, aim_y, h_frac=cfg.target_height_frac))
    assert abs(cmd.yaw_rate) < 1.0
    assert abs(cmd.pitch) < 1.0


def test_no_derivative_spike_on_first_frame():
    """The D term must not see the whole error as one frame's change."""
    ctl = FollowController(ControlConfig())
    first = ctl.update(box_at(W * 0.9, H * 0.5), W, H, 1 / 30)
    # With kd active on a cold start this would blow straight to the limit.
    assert abs(first.yaw_rate) < ControlConfig().yaw_max


# ------------------------------------------------------------------ failsafe --

def test_not_commanding_relaxes_to_neutral():
    cfg = ControlConfig(search_enabled=False)
    ctl = FollowController(cfg)
    settle(ctl, box_at(W * 0.9, H * 0.5))
    assert abs(ctl.yaw.output) > 5.0

    lost = LockResult(LockState.LOST, commanding=False)
    for _ in range(60):
        cmd = ctl.update(lost, W, H, 1 / 30)
    assert cmd.yaw_rate == pytest.approx(0.0, abs=1e-6)
    assert cmd.pitch == pytest.approx(0.0, abs=1e-6)
    assert cmd.flags & FLAG_FAILSAFE


# ---------------------------------------------------------------- searching --

def test_search_sweeps_toward_where_they_went():
    cfg = ControlConfig()
    ctl = FollowController(cfg)
    settle(ctl, box_at(W * 0.9, H * 0.5))        # last seen to the RIGHT

    lost = LockResult(LockState.LOST, commanding=False)
    for _ in range(30):
        cmd = ctl.update(lost, W, H, 1 / 30)
    assert cmd.yaw_rate > 0, "searched away from where the subject went"
    assert abs(cmd.yaw_rate) == pytest.approx(cfg.search_yaw_rate, abs=1e-6)
    # Pitch must NOT search — flying forward toward a subject you cannot see
    # is how you meet a wall.
    assert cmd.pitch == pytest.approx(0.0, abs=1e-6)


def test_search_follows_the_other_direction_too():
    ctl = FollowController(ControlConfig())
    settle(ctl, box_at(W * 0.1, H * 0.5))        # last seen to the LEFT
    lost = LockResult(LockState.LOST, commanding=False)
    for _ in range(30):
        cmd = ctl.update(lost, W, H, 1 / 30)
    assert cmd.yaw_rate < 0


def test_never_locked_never_searches():
    """A drone that has not yet seen its target must sit still, not spin
    around hunting for whoever happens to be nearby."""
    ctl = FollowController(ControlConfig())
    searching = LockResult(LockState.SEARCHING, commanding=False)
    for _ in range(60):
        cmd = ctl.update(searching, W, H, 1 / 30)
    assert cmd.yaw_rate == pytest.approx(0.0, abs=1e-6)


def test_search_gives_up_after_the_timeout(monkeypatch):
    """An unbounded sweep is just a slow spin."""
    import pursuit.control as control_mod
    cfg = ControlConfig(search_timeout_s=1.0)
    ctl = FollowController(cfg)
    settle(ctl, box_at(W * 0.9, H * 0.5))

    now = [1000.0]
    monkeypatch.setattr(control_mod.time, "monotonic", lambda: now[0])

    lost = LockResult(LockState.LOST, commanding=False)
    for _ in range(10):
        cmd = ctl.update(lost, W, H, 1 / 30)
    assert abs(cmd.yaw_rate) > 1.0, "did not start searching"

    now[0] += cfg.search_timeout_s + 0.5
    for _ in range(120):
        cmd = ctl.update(lost, W, H, 1 / 30)
    assert cmd.yaw_rate == pytest.approx(0.0, abs=1e-6), \
        "kept spinning past the search timeout"


def test_altitude_is_held_not_dropped_when_target_lost():
    ctl = FollowController(ControlConfig())
    settle(ctl, box_at(W / 2, H * 0.2))       # climb a bit
    held = ctl.alt_sp
    lost = LockResult(LockState.LOST, commanding=False)
    for _ in range(90):
        cmd = ctl.update(lost, W, H, 1 / 30)
    assert cmd.alt_sp == pytest.approx(held, abs=1e-6)


def test_coasting_decays_the_command():
    """A Kalman prediction is credible for a few frames, then fiction."""
    cfg = ControlConfig()
    ctl = FollowController(cfg)
    lock = box_at(W * 0.9, H * 0.5)
    settle(ctl, lock)
    strong = abs(ctl.yaw.output)

    box = box_at(W * 0.9, H * 0.5)
    for miss in range(1, 25):
        coast = LockResult(LockState.COASTING, xyxy=box.xyxy, confidence=0.5,
                           source="predicted", miss_frames=miss,
                           commanding=True)
        cmd = ctl.update(coast, W, H, 1 / 30)
    assert abs(cmd.yaw_rate) < strong * 0.5


def test_locked_flag_only_when_locked():
    ctl = FollowController(ControlConfig())
    cmd = ctl.update(box_at(W / 2, H / 2), W, H, 1 / 30)
    assert cmd.flags & FLAG_LOCKED

    coast = LockResult(LockState.COASTING, xyxy=box_at(W / 2, H / 2).xyxy,
                       source="predicted", miss_frames=2, commanding=True)
    cmd = ctl.update(coast, W, H, 1 / 30)
    assert not (cmd.flags & FLAG_LOCKED)


# --------------------------------------------------- attitude compensation --

def test_pitching_forward_does_not_command_a_climb():
    """The coupling bug this whole feature exists for.

    Camera bolted to the airframe: nose-down tilt makes a fixed subject appear
    HIGHER in frame, the altitude loop reads "target too high" and climbs — so
    every forward chase became a climb, and every deceleration a descent.
    """
    cfg = ControlConfig()
    cam = CameraConfig()
    focal = cam.focal_px(W)

    # Subject perfectly framed with the drone level.
    aim_y = H * 0.5 * (1.0 + cfg.aim_y_bias)
    level = box_at(W / 2, aim_y, h_frac=0.35)

    # Now the drone pitches 12 deg nose-down to chase. The subject has not
    # moved, but the image says they rose by focal*tan(12deg) pixels.
    import math
    shift = focal * math.tan(math.radians(12.0))
    apparent = box_at(W / 2, aim_y - shift, h_frac=0.35)

    uncorrected = FollowController(cfg, cam)
    corrected = FollowController(cfg, cam)
    start = uncorrected.alt_sp

    for _ in range(90):
        uncorrected.update(apparent, W, H, 1 / 30)
        corrected.update(apparent, W, H, 1 / 30, attitude=(0.0, 12.0))

    assert uncorrected.alt_sp > start + 0.05, \
        "test is not reproducing the coupling it is meant to check"
    assert corrected.alt_sp == pytest.approx(start, abs=0.02), \
        f"attitude correction failed: {corrected.alt_sp:.3f} vs {start:.3f}"


def test_roll_does_not_command_a_spurious_yaw():
    import math
    cfg = ControlConfig()
    cam = CameraConfig()

    # Subject on the vertical centreline but well below the optical axis. The
    # offset is measured from the PRINCIPAL POINT, which is what roll rotates
    # about — measuring it from the aim point instead silently produces a
    # different geometry from the one the correction implements.
    roll = 20.0
    dv = 200.0
    r = math.radians(-roll)
    ru = -dv * math.sin(r)
    rv = dv * math.cos(r)
    apparent = box_at(W / 2 + ru, H / 2 + rv, h_frac=0.30)

    uncorrected = FollowController(cfg, cam)
    corrected = FollowController(cfg, cam)
    for _ in range(60):
        u = uncorrected.update(apparent, W, H, 1 / 30)
        c = corrected.update(apparent, W, H, 1 / 30, attitude=(roll, 0.0))

    assert abs(u.yaw_rate) > 5.0, "test is not reproducing the roll coupling"
    assert abs(c.yaw_rate) < abs(u.yaw_rate) * 0.35, \
        "roll correction did not remove the spurious yaw"


def test_attitude_is_optional():
    """No telemetry (bench testing, link down) must still work, just coupled."""
    ctl = FollowController(ControlConfig(), CameraConfig())
    cmd = settle(ctl, box_at(W * 0.8, H / 2))
    assert cmd.yaw_rate > 0
    assert ctl.attitude_applied == (0.0, 0.0)


# ------------------------------------------------------------------- lead --

class _Moving:
    """Minimal stand-in for a Track carrying a Kalman velocity."""
    def __init__(self, vx, vy, hits=10):
        self.mean = np.array([0, 0, 0, 0, vx, vy, 0, 0], dtype=np.float32)
        self.hits = hits


def _walk_across(ctrl, v_px_per_frame=16.7, n=25):
    """Run a PHYSICALLY MOVING subject through a controller.

    The subject is placed so that it crosses the frame centre exactly on the
    final frame. At that instant the positional error is zero, so whatever yaw
    is still commanded is the lead term and nothing else — which is the only
    way to isolate lead without pinning the box.

    An earlier version of this test did pin the box at centre while handing the
    controller a track that CLAIMED 16.7 px/frame of motion. That state cannot
    occur: ByteTrack's velocity is derived from the box actually moving. The
    target Kalman filter measures the pixels rather than believing the claim,
    and correctly reported no motion — which read as a regression and was
    really the fixture being unphysical.
    """
    out = None
    for i in range(n):
        cx = W / 2 - v_px_per_frame * (n - 1 - i)
        box = box_at(cx, H / 2)
        lock = LockResult(LockState.LOCKED,
                          track=_Moving(v_px_per_frame, 0.0),
                          xyxy=box.xyxy, confidence=0.9, source="face",
                          commanding=True)
        out = ctrl.update(lock, W, H, 1 / 30)
    return out


def test_lead_aims_ahead_of_a_moving_subject():
    # 16.7 px/frame is a person walking 1.5 m/s at 3 m through a 65-degree lens
    # at 30fps — the case the feature exists for. Anything much slower is
    # swallowed by the deadband, which is correct: a 20 px aim offset on a
    # 1280 px frame is not worth moving an aircraft for.
    lead = _walk_across(FollowController(ControlConfig()))
    none = _walk_across(FollowController(replace(ControlConfig(),
                                                 lead_time_s=0.0)))

    # Both see the subject dead centre on the final frame, so positional error
    # is identical and zero. The difference is entirely the lead term.
    assert lead.yaw_rate > none.yaw_rate + 3.0, "lead compensation did nothing"
    assert lead.yaw_rate > 3.0


def test_lead_falls_back_to_tracker_velocity_when_the_filter_is_off():
    """The pre-filter path is still there and still works, so use_target_kf is
    a real switch rather than a one-way door."""
    off = replace(ControlConfig(), use_target_kf=False)
    moved = _walk_across(FollowController(off))
    still = _walk_across(FollowController(replace(off, lead_time_s=0.0)))
    assert moved.yaw_rate > still.yaw_rate + 3.0


def test_a_stationary_subject_gets_no_lead():
    ctrl = FollowController(ControlConfig())
    box = box_at(W / 2, H / 2)
    lock = LockResult(LockState.LOCKED, track=_Moving(0.0, 0.0),
                      xyxy=box.xyxy, confidence=0.9, source="face",
                      commanding=True)
    out = None
    for _ in range(40):
        out = ctrl.update(lock, W, H, 1 / 30)
    assert abs(out.yaw_rate) < 1.0
    assert abs(ctrl.lead_px[0]) < 5.0


def test_lead_is_bounded():
    cfg = ControlConfig(lead_max_px=100.0)
    ctl = FollowController(cfg)
    wild = LockResult(LockState.LOCKED, track=_Moving(9999.0, 9999.0),
                      xyxy=box_at(W / 2, H / 2).xyxy, source="face",
                      commanding=True)
    for _ in range(20):
        ctl.update(wild, W, H, 1 / 30)
    assert abs(ctl.lead_px[0]) <= cfg.lead_max_px + 1e-6
    assert abs(ctl.lead_px[1]) <= cfg.lead_max_px + 1e-6


def test_new_tracks_are_not_led_on():
    """A fresh track's zeroed velocity means 'unknown', not 'stationary'."""
    ctl = FollowController(ControlConfig())
    fresh = LockResult(LockState.LOCKED, track=_Moving(50.0, 0.0, hits=1),
                       xyxy=box_at(W / 2, H / 2).xyxy, source="face",
                       commanding=True)
    ctl.update(fresh, W, H, 1 / 30)
    assert ctl.lead_px == (0.0, 0.0)


# ---------------------------------------------------------- metric standoff --

def test_metric_standoff_holds_a_real_distance():
    cfg = ControlConfig(standoff_m=3.0, use_metric_standoff=True)
    cam = CameraConfig()
    ctl = FollowController(cfg, cam, subject_height_m=1.75)
    focal = cam.focal_px(W)

    # Box height that corresponds to exactly 3m — should command ~nothing.
    at_range = (1.75 * focal) / 3.0
    lock = locked(W / 2 - 60, H / 2 - at_range / 2,
                  W / 2 + 60, H / 2 + at_range / 2)
    cmd = settle(ctl, lock)
    assert abs(cmd.pitch) < 1.5
    assert ctl.range_m == pytest.approx(3.0, rel=0.05)

    # Twice as far -> half the pixel height -> move forward.
    far = (1.75 * focal) / 6.0
    ctl2 = FollowController(cfg, cam, subject_height_m=1.75)
    cmd = settle(ctl2, locked(W / 2 - 30, H / 2 - far / 2,
                              W / 2 + 30, H / 2 + far / 2))
    assert cmd.pitch > 1.0
    assert ctl2.range_m == pytest.approx(6.0, rel=0.05)


def test_falls_back_to_height_fraction_without_a_known_height():
    cfg = ControlConfig(use_metric_standoff=True)
    ctl = FollowController(cfg, CameraConfig(), subject_height_m=None)
    cmd = settle(ctl, box_at(W / 2, H / 2, h_frac=0.20))
    assert ctl.range_m is None
    assert cmd.pitch > 1.0, "fallback standoff stopped working"


def test_every_emitted_command_is_transmittable():
    """Whatever the controller produces must survive packing.

    The packer rejects non-finite values, so this asserts the control maths
    cannot produce a NaN — via a zero-size box, a degenerate dt, or otherwise.
    """
    ctl = FollowController(ControlConfig())
    cases = [
        box_at(W / 2, H / 2),
        locked(0, 0, 0, 0),                       # zero-area box
        locked(-50, -50, W + 50, H + 50),          # larger than the frame
        box_at(0, 0, h_frac=0.001),
    ]
    for lock in cases:
        for dt in (1e-9, 1 / 30, 10.0):
            cmd = ctl.update(lock, W, H, dt)
            cmd.pack()                              # raises if non-finite


# ------------------------------------------------------- identity re-verify --

def _reverify_ctl(**over):
    cfg = ControlConfig(**over)
    ctl = FollowController(cfg)
    ctl.subject_height_m = 1.75          # metric standoff needs enrolment height
    return cfg, ctl


def _lock_at(cx, cy, source, h_frac=0.45, aspect=0.4):
    """box_at() with a choosable source — its own is hard-coded to face."""
    bh = h_frac * H
    bw = bh * aspect
    return locked(cx - bw / 2, cy - bh / 2, cx + bw / 2, cy + bh / 2,
                  source=source)


def _run(ctl, lock, seconds, dt=1 / 30):
    cmd = None
    for _ in range(int(seconds / dt)):
        cmd = ctl.update(lock, W, H, dt)
    return cmd


def test_body_only_lock_closes_in_to_reverify_the_face():
    """Face works to ~2.4m, standoff is 3.0m: held at standoff the lock is
    body-only and an ID drift can never be caught. After long enough on the
    body model alone the controller must shrink the standoff into face range."""
    cfg, ctl = _reverify_ctl()
    _run(ctl, _lock_at(W / 2, H / 2, "face"), 1.0)
    assert not ctl.reverifying
    _run(ctl, _lock_at(W / 2, H / 2, "body"),
         cfg.reverify_after_s + 1.0)
    assert ctl.reverifying
    assert ctl.standoff_used_m == pytest.approx(cfg.reverify_standoff_m)


def test_a_fresh_face_confirmation_ends_the_excursion():
    cfg, ctl = _reverify_ctl()
    _run(ctl, _lock_at(W / 2, H / 2, "body"),
         cfg.reverify_after_s + 1.0)
    assert ctl.reverifying
    ctl.update(_lock_at(W / 2, H / 2, "face"), W, H, 1 / 30)
    assert not ctl.reverifying
    assert ctl.standoff_used_m == pytest.approx(cfg.standoff_m)


def test_reverify_gives_up_rather_than_pressing_closer_forever():
    """If the face never becomes readable (turned away, masked), backing off
    beats hovering at 2m indefinitely — and the clock restarts, so the next
    attempt happens rather than never."""
    cfg, ctl = _reverify_ctl()
    body = _lock_at(W / 2, H / 2, "body")
    _run(ctl, body, cfg.reverify_after_s + 1.0)
    assert ctl.reverifying
    _run(ctl, body, cfg.reverify_give_up_s + 1.0)
    assert not ctl.reverifying
    assert ctl.standoff_used_m == pytest.approx(cfg.standoff_m)
    # ...and the cycle re-arms rather than staying given-up forever
    _run(ctl, body, cfg.reverify_after_s + 1.0)
    assert ctl.reverifying


def test_reverify_disabled_is_exactly_the_old_behaviour():
    cfg, ctl = _reverify_ctl(reverify_enabled=False)
    _run(ctl, _lock_at(W / 2, H / 2, "body"), 60.0)
    assert not ctl.reverifying
    assert ctl.standoff_used_m == pytest.approx(cfg.standoff_m)


def test_coasting_does_not_age_the_face_clock():
    """Prediction frames carry no identity evidence either way; only body-
    confirmed COMMANDING frames should advance the clock, or a long coast
    would trigger a close approach at whoever gets re-acquired."""
    cfg, ctl = _reverify_ctl()
    coast = LockResult(LockState.COASTING, commanding=False)
    _run(ctl, coast, cfg.reverify_after_s + 2.0)
    assert not ctl.reverifying
