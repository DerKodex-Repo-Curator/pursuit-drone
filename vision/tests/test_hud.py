"""The preview window.

A HUD is the one part of this stack whose correctness cannot be asserted —
whether it READS well is a judgement, made by looking at it. What can be
asserted is everything around that judgement: that it renders at all in every
state the pipeline can hand it, that it never quietly corrupts the frame the
rest of the pipeline is still holding, and that its layout invariants hold at
sizes nobody tests by hand.

That is not hypothetical. Writing these caught two crashes that only fire in
states the developer does not happen to be in while looking at the window:
a marker constant that does not exist in this OpenCV build, and a narrow
capture whose header drew the mode pill straight over the target's name.
"""
from __future__ import annotations

import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from pursuit import hud as hud_mod                              # noqa: E402
from pursuit.hud import CANVAS_MIN_W, DECK_H, HEAD_H, Hud       # noqa: E402
from pursuit.lock import LockResult, LockState                  # noqa: E402
from pursuit.pipeline import FrameResult, Timings               # noqa: E402
from pursuit.protocol import FLAG_ARMED, Command, Telemetry     # noqa: E402


class _Track:
    def __init__(self, tid, xyxy, is_target, identity=None, score=0.0):
        self.track_id = tid
        self.xyxy = np.array(xyxy, np.float32)
        self.is_target = is_target
        self.identity = identity
        self.identity_score = score


class _Face:
    def __init__(self, xyxy):
        self.xyxy = np.array(xyxy, np.float32)


class _Stats:
    sent = 4812
    received = 3204
    rejected = 2
    send_errors = 0
    rtt_ms = 19.4


class _Link:
    def __init__(self, enabled=True, up=True, telemetry=True):
        self.enabled = enabled
        self.link_up = up
        self.stats = _Stats()
        self.telemetry = Telemetry(vbat=3.74, alt=3.28, roll=-11.4,
                                   pitch=6.2, yaw=142.0, loop_hz=996,
                                   state=3) if telemetry else None


class _Mode:
    value = "FOLLOW"


class _Arbiter:
    mode = _Mode()
    overrides = 0


class _Pilot:
    connected = True


def _result(state, w=1280, h=720, box=(498, 168, 622, 600)):
    frame = np.full((h, w, 3), 90, np.uint8)
    tracks = [_Track(3, box, True, "paing", 0.94),
              _Track(7, (868, 168, 992, 600), False, "sina", 0.81)]
    faces = [_Face((524, 172, 596, 258))]
    lock = LockResult(
        state=state,
        xyxy=(np.array(box, np.float32)
              if state is not LockState.SEARCHING else None),
        confidence=0.94,
        source="predicted" if state is LockState.COASTING else "face",
        target_name="paing",
        miss_frames=7 if state is LockState.COASTING else 0,
        commanding=state is LockState.LOCKED,
    )
    t = Timings()
    t.fps, t.detect, t.face, t.total = 25.4, 18.2, 11.9, 39.4
    cmd = Command(yaw_rate=-46.2, pitch=8.4, alt_sp=3.6, flags=FLAG_ARMED)
    return FrameResult(frame, [], tracks, faces, lock, cmd, t, 0)


@pytest.mark.parametrize("state", list(LockState))
def test_every_lock_state_renders(state):
    """The states a developer is least often in are the ones that crash."""
    view = Hud().draw(_result(state), link=_Link(), arbiter=_Arbiter(),
                      pilot=_Pilot(), outgoing=Command(flags=FLAG_ARMED))
    assert view.dtype == np.uint8
    assert view.ndim == 3


@pytest.mark.parametrize("link,arbiter,pilot", [
    (None, None, None),                      # webcam only, no aircraft
    (_Link(enabled=False), None, None),      # link constructed but disabled
    (_Link(up=False), _Arbiter(), _Pilot()),  # aircraft silent
    (_Link(telemetry=False), _Arbiter(), _Pilot()),   # up, nothing back yet
])
def test_degraded_link_states_render(link, arbiter, pilot):
    """Every one of these is a real state on a bench, and three of them are
    what you see BEFORE the aircraft exists."""
    Hud().draw(_result(LockState.LOCKED), link=link, arbiter=arbiter,
               pilot=pilot)


def test_canvas_is_the_video_plus_its_chrome():
    r = _result(LockState.LOCKED)
    view = Hud().draw(r, link=_Link())
    assert view.shape[0] == HEAD_H + 720 + DECK_H
    assert view.shape[1] == 1280


def test_a_narrow_capture_is_pillarboxed_rather_than_relaid_out():
    """One deck layout, always. A second layout for small frames is a second
    thing to get wrong, and it is the one nobody looks at."""
    r = _result(LockState.LOCKED, w=640, h=360, box=(200, 90, 300, 320))
    view = Hud().draw(r, link=_Link(), arbiter=_Arbiter(), pilot=_Pilot())
    assert view.shape[1] == CANVAS_MIN_W
    assert view.shape[0] == HEAD_H + 360 + DECK_H


def test_the_frame_is_never_drawn_on():
    """The pipeline still holds this array — the tracker's appearance model
    reads from it. Annotating in place would feed the HUD's own overlay back
    into re-identification, which is a failure that would look like the model
    drifting rather than like a drawing bug."""
    r = _result(LockState.LOCKED)
    before = r.frame.copy()
    Hud().draw(r, link=_Link())
    assert np.array_equal(r.frame, before)


def test_the_locked_subject_is_the_brightest_thing_on_screen():
    """The spotlight is the whole reason "which person" is answerable at a
    glance, so it is worth pinning rather than trusting to a colour choice."""
    r = _result(LockState.LOCKED)
    view = Hud().draw(r, link=_Link())
    x1, y1, x2, y2 = (int(v) for v in r.lock.xyxy)
    inside = view[HEAD_H + y1 + 40:HEAD_H + y2 - 40, x1 + 20:x2 - 20]
    outside = view[HEAD_H + y1 + 40:HEAD_H + y2 - 40, 40:200]
    assert inside.mean() > outside.mean() * 1.8


def test_the_spotlight_can_be_turned_off():
    r = _result(LockState.LOCKED)
    hud = Hud()
    hud.spotlight = False
    view = hud.draw(r, link=_Link())
    x1, y1, x2, y2 = (int(v) for v in r.lock.xyxy)
    outside = view[HEAD_H + y1 + 40:HEAD_H + y2 - 40, 40:200]
    assert outside.mean() > 60          # undimmed grey 90 background


def test_acquisition_settles_rather_than_animating_forever():
    """The brackets converge over a handful of frames. A lock held for a long
    time must look identical frame to frame, or the window develops a shimmer
    that reads as instability in the tracker."""
    hud = Hud()
    r = _result(LockState.LOCKED)
    # Long enough for the trail ring to fill as well: while it is still
    # growing, every frame re-fades the whole trail and no two frames match.
    for _ in range(70):
        a = hud.draw(r, link=_Link())
    b = hud.draw(r, link=_Link())
    assert np.array_equal(a, b)


def test_losing_the_target_clears_the_history():
    """Stale trail and confidence history from a previous subject would draw a
    path the aircraft is not following."""
    hud = Hud()
    for _ in range(10):
        hud.draw(_result(LockState.LOCKED), link=_Link())
    assert hud._trail and hud._conf
    hud.draw(_result(LockState.SEARCHING), link=_Link())
    assert not hud._trail and not hud._conf


def test_a_long_target_name_cannot_push_the_header_into_the_right_block():
    """The left and right header blocks are laid out from opposite edges. If
    the left one is not clipped against the right, a long name prints
    underneath the mode pill."""
    r = _result(LockState.LOCKED, w=940, h=400)
    r.lock.target_name = "a" * 60
    Hud().draw(r, link=_Link(), arbiter=_Arbiter(), pilot=_Pilot())


def test_the_altitude_meter_does_not_saturate_at_a_normal_setpoint():
    """It used to be signed about a hard-coded 1.2 m, which was reasonable
    while the rangefinder capped the aircraft at 2 m and became nonsense the
    moment a barometer took the ceiling to 9.5."""
    hud = Hud()
    deck = np.full((DECK_H, 1280, 3), hud_mod.INK, np.uint8)
    r = _result(LockState.LOCKED)
    r.command.alt_sp = 6.0
    hud._draw_deck(deck, r, _Link(), _Arbiter(), _Link().telemetry)
    # The saturation dot is the only RED thing in the command column.
    col = deck[:, :320]
    red = np.all(np.abs(col.astype(int) - np.array(hud_mod.RED)) < 12, axis=2)
    assert not red.any()


def test_setpoints_beyond_the_ceiling_still_flag():
    hud = Hud()
    deck = np.full((DECK_H, 1280, 3), hud_mod.INK, np.uint8)
    r = _result(LockState.LOCKED)
    r.command.alt_sp = 40.0
    hud._draw_deck(deck, r, _Link(), _Arbiter(), _Link().telemetry)
    col = deck[:, :320]
    red = np.all(np.abs(col.astype(int) - np.array(hud_mod.RED)) < 12, axis=2)
    assert red.any()
