"""Annotated preview.

The point of this window is not to look pretty — it is to make a wrong decision
obvious at a glance while you are standing in front of the camera and cannot
read a terminal. So the things that get the most visual weight are the ones
that are dangerous when wrong: which person is locked, whether the lock is
being held on a face or on colour alone, and what the drone is actually being
told to do.

Setpoints are drawn as signed bars rather than printed as numbers. A number
tells you the value; a bar tells you instantly that yaw has been pinned to its
limit for the last two seconds, which is the thing you need to notice.
"""
from __future__ import annotations

import cv2
import numpy as np

from .lock import LockState
from .protocol import FLAG_ARMED, Command

# BGR. Restrained on purpose.
#
# The default instinct for an overlay like this is saturated green-on-black
# with translucent slabs behind everything — which is what every generated
# "AI vision" mockup looks like, and it reads as a screensaver rather than an
# instrument. Real flight OSDs, machine-vision inspection tools and broadcast
# overlays all do the opposite: near-white text, hairline rules, ONE accent
# reserved for state, and nothing decorative competing with the picture.
WHITE = (238, 240, 242)     # slightly warm off-white, not clinical 255
GREY = (150, 152, 155)
DIM = (105, 107, 110)
GREEN = (120, 200, 130)     # desaturated — an instrument green, not a laser
AMBER = (90, 175, 225)
RED = (80, 85, 220)
BLUE = (205, 165, 105)
BG = (22, 22, 24)
RULE = (70, 72, 76)         # hairline separators

STATE_COLOUR = {
    LockState.SEARCHING: GREY,
    LockState.LOCKED: GREEN,
    LockState.COASTING: AMBER,
    LockState.LOST: RED,
}

FONT = cv2.FONT_HERSHEY_DUPLEX
# SIMPLEX is the single-stroke plotter font every OpenCV demo ships with, and
# it is the main reason an overlay looks like an OpenCV demo. DUPLEX draws the
# same glyphs with a proper two-stroke weight and reads as typeset.


def _text(img, s, org, scale=0.45, colour=WHITE, thick=1, shadow=True):
    """Text with a centred outline, not an offset drop-shadow.

    This used to draw the black pass at (x+1, y+1) with thickness+1. At these
    scales that is not a shadow — it is a second, misaligned copy of every
    glyph, and the whole HUD reads as doubled. An outline goes at the SAME
    origin, thicker, underneath, so the coloured fill sits centred inside it.
    """
    if shadow:
        cv2.putText(img, s, org, FONT, scale, (0, 0, 0), thick + 2,
                    cv2.LINE_AA)
    cv2.putText(img, s, org, FONT, scale, colour, thick, cv2.LINE_AA)


def _panel(img, x, y, w, h, alpha=0.86):
    """Translucent backing so text stays readable over a bright scene."""
    x2, y2 = min(x + w, img.shape[1]), min(y + h, img.shape[0])
    x, y = max(x, 0), max(y, 0)
    if x2 <= x or y2 <= y:
        return
    # Near-opaque and genuinely dark, with a lit top edge. The old version was
    # a 55%-grey wash: it dimmed the picture without ever becoming a surface,
    # so text sat in fog. A panel should either be a surface or not exist.
    roi = img[y:y2, x:x2]
    roi[:] = cv2.addWeighted(roi, 1.0 - alpha,
                             np.full_like(roi, BG, dtype=np.uint8), alpha, 0)
    cv2.line(img, (x, y), (x2 - 1, y), RULE, 1, cv2.LINE_AA)


def _corner_box(img, xyxy, colour, thick=1, frac=0.22):
    """Corner brackets only.

    Long thin brackets read as a measurement reticle. A full rectangle reads
    as a sticker, and a short thick bracket reads as a game HUD. 22% of each
    edge is about where it stops looking decorative.
    """
    x1, y1, x2, y2 = (int(round(v)) for v in xyxy)
    cx = max(6, int((x2 - x1) * frac))
    cy = max(6, int((y2 - y1) * frac))
    for px, dx in ((x1, 1), (x2, -1)):
        for py, dy in ((y1, 1), (y2, -1)):
            cv2.line(img, (px, py), (px + dx * cx, py), colour, thick,
                     cv2.LINE_AA)
            cv2.line(img, (px, py), (px, py + dy * cy), colour, thick,
                     cv2.LINE_AA)


def _bar(img, x, y, w, h, value, limit, colour, label):
    """Signed bar centred on zero, with a tick at the limit."""
    _panel(img, x, y, w, h, 0.62)
    cx = x + w // 2
    # Quarter-scale ticks, so a deflection can be read as a fraction of the
    # limit rather than just "some bar". Centre tick full height, quarters
    # short — the same convention an analogue meter uses.
    for f in (-1.0, -0.5, 0.5, 1.0):
        tx = cx + int((w // 2 - 3) * f)
        cv2.line(img, (tx, y + h - 6), (tx, y + h - 3), RULE, 1)
    cv2.line(img, (cx, y + 3), (cx, y + h - 3), DIM, 1)

    frac = 0.0 if limit <= 0 else max(-1.0, min(1.0, value / limit))
    span = int((w // 2 - 3) * frac)
    if abs(span) >= 1:
        cv2.rectangle(img, (cx, y + 4), (cx + span, y + h - 7), colour, -1)
    # Flag saturation: a pinned axis is the signature of a runaway loop.
    if abs(frac) > 0.98:
        cv2.rectangle(img, (x, y), (x + w - 1, y + h - 1), RED, 1)
    _text(img, label, (x + 4, y + h - 5), 0.38, WHITE)


class Hud:
    def __init__(self, aim_bias: float = 0.12,
                 show_all_tracks: bool = True):
        # Must match ControlConfig.aim_y_bias, or the crosshair will sit
        # somewhere the controller is not actually steering toward — which
        # would make a correctly-working loop look broken.
        self.aim_bias = aim_bias
        self.show_all_tracks = show_all_tracks
        self.show_faces = True
        self._trail: list[tuple[int, int]] = []

    def draw(self, result, link=None, armed: bool = False,
             paused: bool = False, arbiter=None, pilot=None,
             outgoing=None) -> np.ndarray:
        img = result.frame.copy()
        h, w = img.shape[:2]
        lock = result.lock
        cmd: Command = result.command
        colour = STATE_COLOUR[lock.state]

        # --- other people ----------------------------------------------------
        if self.show_all_tracks:
            for tr in result.tracks:
                if tr.is_target:
                    continue
                _corner_box(img, tr.xyxy, DIM, 1)
                x1, y1 = int(tr.xyxy[0]), int(tr.xyxy[1])
                # Name them if the gallery recognised them; a bare track id is
                # only useful when it has not.
                if tr.identity:
                    tag = f"{tr.identity} {tr.identity_score:.2f}  not target"
                else:
                    tag = f"#{tr.track_id}"
                _text(img, tag, (x1 + 3, y1 - 6), 0.42, DIM)

        # --- faces -----------------------------------------------------------
        if self.show_faces:
            for face in result.faces:
                x1, y1, x2, y2 = (int(v) for v in face.xyxy)
                cv2.rectangle(img, (x1, y1), (x2, y2), (70, 120, 160), 1)

        # --- aim point -------------------------------------------------------
        aim_x = w // 2
        aim_y = int(h * 0.5 * (1.0 + self.aim_bias))
        cv2.drawMarker(img, (aim_x, aim_y), (200, 200, 200),
                       cv2.MARKER_CROSS, 18, 1, cv2.LINE_AA)

        # --- the target ------------------------------------------------------
        if lock.has_target:
            x1, y1, x2, y2 = (int(round(v)) for v in lock.xyxy)
            tcx, tcy = (x1 + x2) // 2, (y1 + y2) // 2

            dashed = lock.source == "predicted"
            _corner_box(img, lock.xyxy, colour, 2 if not dashed else 1)
            if dashed:
                cv2.rectangle(img, (x1, y1), (x2, y2), colour, 1, cv2.LINE_4)

            # Error vector: how far the drone thinks it is off aim.
            cv2.line(img, (aim_x, aim_y), (tcx, tcy), colour, 1, cv2.LINE_AA)
            cv2.circle(img, (tcx, tcy), 4, colour, -1, cv2.LINE_AA)

            self._trail.append((tcx, tcy))
            if len(self._trail) > 40:
                self._trail.pop(0)

            tag = f"{lock.target_name or 'target'}  {lock.source}"
            if lock.confidence > 0:
                tag += f" {lock.confidence:.2f}"
            (tw, th), _ = cv2.getTextSize(tag, FONT, 0.45, 1)
            _panel(img, x1, max(0, y1 - th - 10), tw + 10, th + 8, 0.6)
            _text(img, tag, (x1 + 5, max(th, y1 - 8)), 0.45, colour)
        else:
            self._trail.clear()

        for i in range(1, len(self._trail)):
            fade = int(60 + 140 * i / len(self._trail))
            cv2.line(img, self._trail[i - 1], self._trail[i],
                     (fade, fade, fade), 1, cv2.LINE_AA)

        # --- state banner -----------------------------------------------------
        _panel(img, 0, 0, w, 34, 0.6)
        _text(img, lock.state.value, (12, 23), 0.62, colour, 2)

        sub = f"target: {lock.target_name or 'ANY'}"
        if lock.state is LockState.COASTING:
            sub += f"   miss {lock.miss_frames}f"
        _text(img, sub, (135, 22), 0.45, GREY)

        t = result.timings
        _text(img, f"{t.fps:4.1f} fps", (w - 250, 22), 0.45, WHITE)
        _text(img, f"det {t.detect:.0f} face {t.face:.0f} tot {t.total:.0f}ms",
              (w - 180, 22), 0.42, GREY)

        # --- setpoints --------------------------------------------------------
        bx, by, bw, bh = 10, h - 96, 210, 20
        _bar(img, bx, by, bw, bh, cmd.yaw_rate, 120.0, BLUE,
             f"yaw {cmd.yaw_rate:+6.1f} d/s")
        _bar(img, bx, by + 24, bw, bh, cmd.pitch, 14.0, BLUE,
             f"pitch {cmd.pitch:+5.1f} deg")
        _bar(img, bx, by + 48, bw, bh, cmd.alt_sp - 1.2, 1.0, BLUE,
             f"alt {cmd.alt_sp:4.2f} m")

        # --- link -------------------------------------------------------------
        rx = w - 210
        _panel(img, rx, h - 96, 200, 88, 0.55)
        if link is None or not link.enabled:
            _text(img, "LINK  offline (test mode)", (rx + 8, h - 76), 0.42, GREY)
        else:
            up = link.link_up
            _text(img, f"LINK  {'up' if up else 'NO TELEMETRY'}",
                  (rx + 8, h - 76), 0.44, GREEN if up else RED)
            s = link.stats
            _text(img, f"tx {s.sent}  rx {s.received}  "
                       f"bad {s.rejected}  err {s.send_errors}",
                  (rx + 8, h - 60), 0.38, GREY)
            tel = link.telemetry
            if tel is not None:
                vcol = RED if tel.vbat < 3.5 else (
                    AMBER if tel.vbat < 3.7 else GREEN)
                _text(img, f"{tel.state_name}  {tel.vbat:.2f}V",
                      (rx + 8, h - 44), 0.42, vcol)
                _text(img, f"alt {tel.alt:4.2f}m  loop {tel.loop_hz}Hz",
                      (rx + 8, h - 28), 0.38, GREY)

        # --- arm / mode -------------------------------------------------------
        if paused:
            _text(img, "PAUSED", (w // 2 - 46, h // 2), 0.9, AMBER, 2)

        if arbiter is not None:
            live = bool(outgoing is not None and outgoing.flags & FLAG_ARMED)
            mode = arbiter.mode.value
            # Mode colour says who is flying: green when the autonomy has the
            # controls, blue when the pilot does. Nobody should ever have to
            # guess which of those is true.
            mcol = GREEN if mode == "FOLLOW" else BLUE
            _panel(img, w - 168, 42, 158, 46, 0.6)
            _text(img, mode, (w - 160, 62), 0.6, mcol, 2)
            _text(img, "MOTORS LIVE" if live else "safe",
                  (w - 160, 80), 0.46, RED if live else DIM,
                  2 if live else 1)
            if pilot is not None and not pilot.connected:
                _text(img, "NO PILOT INPUT", (w - 300, 62), 0.5, RED, 2)
            if getattr(arbiter, "overrides", 0):
                _text(img, f"overrides {arbiter.overrides}",
                      (w - 300, 80), 0.4, AMBER)
            _text(img, "[1] manual  [2] assist  [3] follow   "
                       "hold SPACE = deadman   wasd/qe/ik = fly   [x] quit",
                  (12, h - 10), 0.4, GREY)
        else:
            arm_col = GREEN if armed else DIM
            _panel(img, w - 92, 42, 82, 24, 0.55)
            _text(img, "ARMED" if armed else "SAFE", (w - 84, 59), 0.5,
                  arm_col, 2)
            _text(img, "[space] arm   [r] reset   [t] tracks   [q] quit",
                  (12, h - 10), 0.4, GREY)
        return img
