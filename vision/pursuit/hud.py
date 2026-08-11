"""Annotated preview.

The point of this window is not to look pretty — it is to make a wrong decision
obvious at a glance while you are standing in front of the camera and cannot
read a terminal. So the things that get the most visual weight are the ones
that are dangerous when wrong: which person is locked, whether the lock is
being held on a face or on colour alone, and what the drone is actually being
told to do.

WHAT CHANGED, AND WHY IT IS NOT DECORATION
------------------------------------------
The previous version printed values around the edges of the video. Everything
was legible and nothing was ORGANISED: the eye had no idea where to look
first, and a value's meaning was carried entirely by its own text. That is the
difference between a debug overlay and an instrument.

This version composites the video into a designed surface — a header that
answers "what is it doing", a deck that answers "what is it commanding and
what is coming back", and the picture in between kept as clean as possible.
Four things carry most of the improvement, and each is functional:

  spotlight   Everything outside the locked box is darkened. The single most
              important fact on screen is WHICH PERSON, and now it is the
              brightest thing on screen rather than one bracket among several.
  acquisition The brackets converge onto the target over ~6 frames instead of
              appearing. A lock that snaps in is visibly a NEW lock; one that
              is merely held looks different. That distinction used to require
              reading the state text.
  tape        Altitude is a vertical scale with a pointer and a setpoint bug,
              not a number. A number tells you where you are; a tape tells you
              instantly that you are 3 m under the commanded height and that
              the ceiling is close.
  horizon     A real artificial horizon off the telemetry. Roll and pitch as
              two printed numbers require arithmetic to interpret; an attitude
              you can see is the whole reason aircraft have this instrument.

Setpoints stay as signed bars rather than numbers, for the same reason: a bar
tells you instantly that yaw has been pinned to its limit for two seconds.

Colour is still restrained on purpose. The default instinct for an overlay
like this is saturated green-on-black with translucent slabs behind
everything, which reads as a screensaver. Real flight OSDs, machine-vision
inspection tools and broadcast graphics all do the opposite: near-white text,
hairline rules, ONE accent reserved for state, and nothing decorative
competing with the picture.
"""
from __future__ import annotations

import math

import cv2
import numpy as np

from .lock import LockState
from .protocol import FLAG_ARMED, Command

# BGR throughout.
INK = (17, 18, 21)          # chrome — near black, very slightly blue
PANEL = (28, 30, 34)        # raised surface inside the chrome
WHITE = (238, 240, 242)     # slightly warm off-white, not clinical 255
GREY = (152, 154, 158)
DIM = (104, 106, 111)
RULE = (58, 60, 66)         # hairline separators
GREEN = (120, 200, 130)     # desaturated — an instrument green, not a laser
AMBER = (90, 175, 225)
RED = (80, 85, 220)
STEEL = (198, 168, 116)     # instrument scales: tapes, ladders, reticle
SKY = (72, 56, 40)          # attitude indicator, above the horizon
GROUND = (34, 44, 62)       # and below it

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

HEAD_H = 50
DECK_H = 138
# The deck's four columns need this much width before the labels start
# colliding. Rather than carry a second layout for narrow captures — which is
# a second thing to get right and the one nobody looks at — a small frame is
# PILLARBOXED into a canvas this wide. One layout, always correct, and the
# side chrome reads as deliberate rather than as a bug.
CANVAS_MIN_W = 940


# --------------------------------------------------------------- primitives

def _text(img, s, org, scale=0.45, colour=WHITE, thick=1, shadow=True):
    """Text with a centred outline, not an offset drop-shadow.

    An offset black pass at (x+1, y+1) is not a shadow at these sizes — it is
    a second, misaligned copy of every glyph, and the whole HUD reads as
    doubled. The outline goes at the SAME origin, thicker, underneath, so the
    coloured fill sits centred inside it.
    """
    if shadow:
        cv2.putText(img, s, org, FONT, scale, (0, 0, 0), thick + 2,
                    cv2.LINE_AA)
    cv2.putText(img, s, org, FONT, scale, colour, thick, cv2.LINE_AA)


def _tw(s, scale=0.45, thick=1):
    return cv2.getTextSize(s, FONT, scale, thick)[0][0]


def _label(img, s, org, colour=DIM, scale=0.33, track=2, shadow=True):
    """Letterspaced small caps.

    Tracking is what separates a label from a value at a glance, and it is
    doing real work here: it lets a 0.33-scale label sit directly above a
    0.5-scale number without the two reading as one run of text.
    """
    x, y = org
    for ch in s.upper():
        if ch == " ":
            x += 4 + track
            continue
        _text(img, ch, (int(x), int(y)), scale, colour, 1, shadow)
        x += cv2.getTextSize(ch, FONT, scale, 1)[0][0] + track
    return int(x - org[0])


def _pill(img, x, y, h, text, fg, bg, scale=0.5, pad=11):
    """Filled capsule. Returns its width so callers can lay out left to right."""
    tw, th = cv2.getTextSize(text, FONT, scale, 1)[0]
    w = tw + pad * 2
    r = h // 2
    cv2.rectangle(img, (x + r, y), (x + w - r, y + h), bg, -1)
    cv2.circle(img, (x + r, y + r), r, bg, -1, cv2.LINE_AA)
    cv2.circle(img, (x + w - r, y + r), r, bg, -1, cv2.LINE_AA)
    _text(img, text, (x + pad, y + (h + th) // 2), scale, fg, 1, shadow=False)
    return w


def _panel(img, x, y, w, h, alpha=0.86):
    """Translucent backing so text stays readable over a bright scene."""
    x2, y2 = min(x + w, img.shape[1]), min(y + h, img.shape[0])
    x, y = max(x, 0), max(y, 0)
    if x2 <= x or y2 <= y:
        return
    # Near-opaque and genuinely dark, with a lit top edge. A 55%-grey wash
    # dims the picture without ever becoming a surface, so text sits in fog.
    # A panel should either be a surface or not exist.
    roi = img[y:y2, x:x2]
    roi[:] = cv2.addWeighted(roi, 1.0 - alpha,
                             np.full_like(roi, PANEL, dtype=np.uint8),
                             alpha, 0)
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


def _spotlight(img, xyxy, keep=0.52, pad=18):
    """Darken everything except the locked subject.

    The most important fact on screen is which person the aircraft has, and a
    coloured bracket competes with every other bracket for attention. Light
    does not compete — it is pre-attentive. One full-frame scale plus one ROI
    copy, well under a millisecond at 720p.

    `keep` is a RATIO, so how much of the background survives depends on how
    much there was. At 0.38 a well-lit room still reads; a dim one goes to
    black and you lose the scene entirely, which is worse than not having the
    effect. 0.52 keeps the subject clearly separated in both. [s] turns it off.
    """
    x1, y1, x2, y2 = (int(round(v)) for v in xyxy)
    h, w = img.shape[:2]
    x1 = max(0, x1 - pad); y1 = max(0, y1 - pad)
    x2 = min(w, x2 + pad); y2 = min(h, y2 + pad)
    if x2 <= x1 or y2 <= y1:
        return
    subject = img[y1:y2, x1:x2].copy()
    cv2.convertScaleAbs(img, dst=img, alpha=keep, beta=0)
    img[y1:y2, x1:x2] = subject


def _reticle(img, cx, cy, colour, gap=7, arm=13, phase=0.0):
    """Aim point.

    A gapped cross rather than a solid one: the gap is where the subject goes,
    and a solid marker sits on top of the thing it is pointing at. The tick
    ring breathes slowly while searching so a stalled pipeline looks stalled.
    """
    for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
        cv2.line(img, (cx + dx * gap, cy + dy * gap),
                 (cx + dx * (gap + arm), cy + dy * (gap + arm)),
                 colour, 1, cv2.LINE_AA)
    r = int(gap + 4 + 2.5 * math.sin(phase))
    cv2.circle(img, (cx, cy), r, colour, 1, cv2.LINE_AA)


def _sparkline(img, x, y, w, h, values, colour, lo=0.0, hi=1.0):
    """Confidence over time.

    A single confidence number cannot distinguish "steady 0.7" from "0.95
    collapsing to 0.7", and those call for opposite actions.

    One sample per pixel, anchored to the RIGHT edge. Stretching however many
    samples exist across the full width — the obvious implementation — makes
    the time axis rescale every frame until the buffer fills, so a constant
    confidence draws a subtly different line each time and the whole chip
    shimmers for the first few seconds of every lock. It also means a given
    horizontal distance does not mean a fixed number of frames, which is the
    only thing a sparkline is for.
    """
    n = min(len(values), int(w))
    if n < 2:
        return
    span = max(hi - lo, 1e-6)
    x0 = int(x + w) - n
    pts = [(x0 + i,
            int(y + h - h * min(1.0, max(0.0, (v - lo) / span))))
           for i, v in enumerate(values[-n:])]
    cv2.polylines(img, [np.array(pts, np.int32)], False, colour, 1,
                  cv2.LINE_AA)


def _meter(img, x, y, w, label, text, value, limit, colour, signed=True):
    """One command axis: label, track, right-aligned value.

    Laid out on a measured grid so nothing can collide as the numbers change
    width. Signed axes fill from the centre because their zero is a real
    place the aircraft can be; altitude fills from the left because zero is
    the ground, and a bar centred on 4.75 m would be meaningless.
    """
    _label(img, label, (x, y + 5), DIM, 0.32)
    tx = x + 46
    tw = max(30, w - 46 - 92)
    cy = y
    cv2.line(img, (tx, cy), (tx + tw, cy), RULE, 3, cv2.LINE_AA)

    if signed:
        cx = tx + tw // 2
        for f in (-1.0, -0.5, 0.5, 1.0):
            px = int(cx + (tw // 2) * f)
            cv2.line(img, (px, cy - 5), (px, cy + 5), RULE, 1, cv2.LINE_AA)
        cv2.line(img, (cx, cy - 7), (cx, cy + 7), DIM, 1, cv2.LINE_AA)
        frac = 0.0 if limit <= 0 else max(-1.0, min(1.0, value / limit))
        span = int((tw // 2) * frac)
        end = cx + span
        if abs(span) >= 1:
            cv2.line(img, (cx, cy), (end, cy), colour, 5, cv2.LINE_AA)
        pinned = abs(frac) > 0.98
    else:
        for f in (0.25, 0.5, 0.75):
            px = int(tx + tw * f)
            cv2.line(img, (px, cy - 5), (px, cy + 5), RULE, 1, cv2.LINE_AA)
        frac = 0.0 if limit <= 0 else max(0.0, min(1.0, value / limit))
        end = tx + int(tw * frac)
        if end > tx:
            cv2.line(img, (tx, cy), (end, cy), colour, 5, cv2.LINE_AA)
        pinned = frac > 0.98

    if pinned:
        # A pinned axis is the signature of a runaway loop, so it is the one
        # thing on this row allowed to use the alarm colour.
        cv2.circle(img, (end, cy), 4, RED, -1, cv2.LINE_AA)

    _text(img, text, (x + w - _tw(text, 0.44), cy + 5), 0.44, WHITE)


def _horizon(img, cx, cy, r, roll, pitch, live=True):
    """Artificial horizon, drawn from telemetry roll and pitch.

    The aircraft symbol is fixed and the world rotates behind it — the
    convention every real attitude indicator uses, and the opposite of what
    feels natural when you first draw one. Getting it backwards produces an
    instrument that looks right and reads exactly wrong.
    """
    px_per_deg = r / 26.0
    cv2.circle(img, (cx, cy), r, PANEL, -1, cv2.LINE_AA)
    if not live:
        cv2.circle(img, (cx, cy), r, RULE, 1, cv2.LINE_AA)
        _label(img, "no data", (cx - 26, cy + 4), DIM, 0.3)
        return

    a = math.radians(-roll)
    ca, sa = math.cos(a), math.sin(a)

    def place(dx, dy):
        return (int(cx + dx * ca - dy * sa), int(cy + dx * sa + dy * ca))

    # Two-tone disc. Ladder lines alone give you the attitude only after you
    # have counted them; sky over ground gives it to you before you have
    # finished looking, which is the entire reason this instrument exists.
    n = 2 * r + 2
    if cx - r - 1 >= 0 and cy - r - 1 >= 0 \
            and cx + r + 1 <= img.shape[1] and cy + r + 1 <= img.shape[0]:
        disc = np.full((n, n, 3), SKY, np.uint8)
        hz = pitch * px_per_deg
        quad = [(-3 * r, hz), (3 * r, hz), (3 * r, 3 * r), (-3 * r, 3 * r)]
        pts = np.array([(int(r + 1 + dx * ca - dy * sa),
                         int(r + 1 + dx * sa + dy * ca)) for dx, dy in quad],
                       np.int32)
        cv2.fillPoly(disc, [pts], GROUND, cv2.LINE_AA)
        mask = np.zeros((n, n), np.uint8)
        cv2.circle(mask, (r + 1, r + 1), r, 255, -1, cv2.LINE_AA)
        roi = img[cy - r - 1:cy + r + 1, cx - r - 1:cx + r + 1]
        np.copyto(roi, disc, where=(mask > 127)[..., None])
    cv2.circle(img, (cx, cy), r, RULE, 1, cv2.LINE_AA)

    for deg in (-20, -10, 0, 10, 20):
        dy = (pitch - deg) * px_per_deg
        if abs(dy) > r - 3:
            continue
        half = r * 0.92 if deg == 0 else r * 0.34
        half = math.sqrt(max(0.0, (r - 2) ** 2 - dy * dy)) if deg == 0 \
            else min(half, math.sqrt(max(0.0, (r - 2) ** 2 - dy * dy)))
        col = STEEL if deg == 0 else DIM
        if deg == 0:
            cv2.line(img, place(-half, dy), place(half, dy), col, 1,
                     cv2.LINE_AA)
        else:
            gap = r * 0.10
            cv2.line(img, place(-half, dy), place(-gap, dy), col, 1,
                     cv2.LINE_AA)
            cv2.line(img, place(gap, dy), place(half, dy), col, 1,
                     cv2.LINE_AA)

    # Fixed aircraft symbol, on top of everything.
    cv2.line(img, (cx - 15, cy), (cx - 5, cy), AMBER, 2, cv2.LINE_AA)
    cv2.line(img, (cx + 5, cy), (cx + 15, cy), AMBER, 2, cv2.LINE_AA)
    cv2.circle(img, (cx, cy), 2, AMBER, -1, cv2.LINE_AA)
    # Roll pointer against a fixed arc.
    top = place(0, -(r - 4))
    cv2.line(img, (cx, cy - r + 1), top, STEEL, 1, cv2.LINE_AA)


def _alt_tape(img, x, y, h, alt, setpoint, ceiling, live):
    """Vertical altitude scale with a pointer and a setpoint bug.

    Span is fixed rather than auto-ranging: a scale whose gradations move is
    not a scale, and the whole value of a tape is that a given distance always
    means the same number of metres.
    """
    w = 58
    span_m = 5.0
    _panel(img, x, y, w, h, 0.80)
    cv2.line(img, (x, y), (x, y + h), RULE, 1)
    mid = y + h // 2
    px_per_m = h / span_m
    centre = alt if live else setpoint

    # Gradations stay clear of the label at the top and the setpoint readout
    # at the bottom. A scale mark landing on a printed number is the kind of
    # collision that only shows up at one particular altitude.
    top, bot = y + 24, y + h - 34
    for tenth in range(int((centre - span_m / 2) * 2) - 1,
                       int((centre + span_m / 2) * 2) + 2):
        m = tenth / 2.0
        if m < 0:
            continue
        py = int(mid - (m - centre) * px_per_m)
        if not (top <= py <= bot):
            continue
        major = abs(m - round(m)) < 1e-6
        cv2.line(img, (x + w - (14 if major else 8), py), (x + w - 2, py),
                 STEEL if major else RULE, 1, cv2.LINE_AA)
        if major:
            s = f"{m:.0f}"
            _text(img, s, (x + w - 18 - _tw(s, 0.34), py + 4), 0.34, GREY)

    # Ceiling: the height above which the aircraft will refuse to be
    # commanded. Worth seeing before you fly into it, not after.
    if ceiling is not None:
        py = int(mid - (ceiling - centre) * px_per_m)
        if y + 2 <= py <= y + h - 2:
            cv2.line(img, (x + 2, py), (x + w - 2, py), RED, 1, cv2.LINE_AA)
            _label(img, "ceil", (x + 4, py - 4), RED, 0.28)

    # Commanded height, as a bug on the scale. A triangle pointing AT the
    # tape, so it reads as "go here" rather than as another gradation.
    py = int(mid - (setpoint - centre) * px_per_m)
    if y + 2 <= py <= y + h - 2:
        cv2.fillPoly(img, [np.array([(x + 3, py - 6), (x + 3, py + 6),
                                     (x + 14, py)], np.int32)], AMBER,
                     cv2.LINE_AA)

    # Current height: a fixed pointer at the centre, because a tape's whole
    # point is that the pointer does not move and the world does.
    col = WHITE if live else DIM
    cv2.line(img, (x, mid), (x + w, mid), col, 1, cv2.LINE_AA)
    val = f"{centre:4.2f}"
    bw = _tw(val, 0.46) + 12
    cv2.rectangle(img, (x - bw, mid - 11), (x, mid + 11), INK, -1)
    cv2.rectangle(img, (x - bw, mid - 11), (x, mid + 11), col, 1, cv2.LINE_AA)
    _text(img, val, (x - bw + 6, mid + 5), 0.46, col, 1, shadow=False)

    _label(img, "alt m", (x + 6, y + 15), GREY, 0.3)
    cv2.line(img, (x, y + 20), (x + w, y + 20), RULE, 1)
    # The commanded height, spelled out under the tape. The bug says where;
    # this says how far, which is the number you argue with.
    cv2.line(img, (x, y + h - 30), (x + w, y + h - 30), RULE, 1)
    _label(img, "sp", (x + 6, y + h - 18), DIM, 0.3)
    _text(img, f"{setpoint:.2f}", (x + 24, y + h - 17), 0.42, AMBER)


# -------------------------------------------------------------------- HUD

class Hud:
    def __init__(self, aim_bias: float = 0.12,
                 show_all_tracks: bool = True):
        # Must match ControlConfig.aim_y_bias, or the crosshair will sit
        # somewhere the controller is not actually steering toward — which
        # would make a correctly-working loop look broken.
        self.aim_bias = aim_bias
        self.show_all_tracks = show_all_tracks
        self.show_faces = True
        self.spotlight = True
        # The firmware clamps the commanded height to this when a barometer is
        # fitted (ALT_CEILING_BARO_M). Shown so the limit is visible in the
        # air rather than discovered by hitting it.
        self.alt_ceiling = 9.5
        # Display aspect (width/height) to pad the canvas out to, or None.
        # Set by the runner once it knows the screen, so a full-screen window
        # is filled with THIS module's chrome rather than with whatever grey
        # the window manager pads a letterbox with — and without stretching
        # the picture to get there, which would put the aim point and the
        # error vector out of proportion with the video they are drawn over.
        self.fill_aspect: float | None = None
        self._trail: list[tuple[int, int]] = []
        self._conf: list[float] = []
        self._acq = 1.0             # bracket convergence, 0..1
        self._prev_state = None
        self._t = 0

    # ---------------------------------------------------------------- video

    def _draw_video(self, img, result, telemetry):
        h, w = img.shape[:2]
        lock = result.lock
        colour = STATE_COLOUR[lock.state]

        if self.spotlight and lock.has_target:
            _spotlight(img, lock.xyxy)

        if self.show_all_tracks:
            for tr in result.tracks:
                if tr.is_target:
                    continue
                _corner_box(img, tr.xyxy, DIM, 1)
                x1, y1 = int(tr.xyxy[0]), int(tr.xyxy[1])
                # Name them if the gallery recognised them; a bare track id is
                # only useful when it has not.
                if tr.identity:
                    tag = f"{tr.identity} {tr.identity_score:.2f}"
                    sub = "NOT TARGET"
                else:
                    tag = f"#{tr.track_id}"
                    sub = "UNKNOWN"
                _text(img, tag, (x1 + 3, y1 - 18), 0.42, GREY)
                _label(img, sub, (x1 + 3, y1 - 6), DIM, 0.3)

        if self.show_faces:
            for face in result.faces:
                x1, y1, x2, y2 = (int(v) for v in face.xyxy)
                cv2.rectangle(img, (x1, y1), (x2, y2), (78, 96, 118), 1,
                              cv2.LINE_AA)

        aim_x = w // 2
        aim_y = int(h * 0.5 * (1.0 + self.aim_bias))
        # The ring breathes only while there is nothing to aim at, so a frozen
        # pipeline is visible as a frozen reticle. Breathing under a held lock
        # would just be a shimmer, and a HUD that moves when nothing is
        # happening trains you to stop believing it when something is.
        _reticle(img, aim_x, aim_y, GREY if lock.has_target else colour,
                 phase=0.0 if lock.has_target else self._t * 0.18)

        if lock.has_target:
            x1, y1, x2, y2 = (int(round(v)) for v in lock.xyxy)
            tcx, tcy = (x1 + x2) // 2, (y1 + y2) // 2

            # Acquisition: brackets converge from outside over ~6 frames. A
            # lock that snaps in is visibly a NEW lock rather than a held one,
            # which previously required reading the state text.
            grow = (1.0 - self._acq) * 0.55
            gx, gy = (x2 - x1) * grow, (y2 - y1) * grow
            box = (x1 - gx, y1 - gy, x2 + gx, y2 + gy)

            dashed = lock.source == "predicted"
            _corner_box(img, box, colour, 1 if dashed else 2)
            if dashed:
                # Coasting on a prediction. Drawn as a thin full rectangle so
                # it is obviously a different KIND of box, not a dimmer one.
                cv2.rectangle(img, (x1, y1), (x2, y2), colour, 1, cv2.LINE_4)

            cv2.line(img, (aim_x, aim_y), (tcx, tcy), colour, 1, cv2.LINE_AA)
            cv2.circle(img, (tcx, tcy), 4, colour, -1, cv2.LINE_AA)

            self._trail.append((tcx, tcy))
            if len(self._trail) > 44:
                self._trail.pop(0)

            self._chip(img, lock, x1, y1, x2, colour)
        else:
            self._trail.clear()

        for i in range(1, len(self._trail)):
            fade = int(55 + 150 * i / len(self._trail))
            cv2.line(img, self._trail[i - 1], self._trail[i],
                     (fade, fade, fade), 1, cv2.LINE_AA)

        # Tall enough to read, short enough that the gradations are close
        # together — a tape stretched over the full frame height has 5 m
        # spread across 660 px and stops looking like a scale at all.
        th = max(200, min(380, h - 80))
        _alt_tape(img, w - 78, (h - th) // 2, th,
                  telemetry.alt if telemetry else 0.0,
                  result.command.alt_sp, self.alt_ceiling,
                  telemetry is not None)

    def _chip(self, img, lock, x1, y1, x2, colour):
        """Identity card for the locked subject."""
        name = (lock.target_name or "TARGET").upper()
        src = lock.source.upper()
        cw = max(178, _tw(name, 0.52) + 96)
        ch = 52
        cy = y1 - ch - 8
        if cy < 0:                       # subject at the top of frame
            cy = y1 + 8
        # Stay clear of the altitude tape as well as the frame edge — the tape
        # is a fixed 78 px strip and the chip would otherwise slide under it
        # whenever the subject drifts right.
        cx = max(0, min(x1, img.shape[1] - cw - 86))
        _panel(img, cx, cy, cw, ch, 0.85)
        cv2.line(img, (cx, cy), (cx, cy + ch), colour, 3)

        _text(img, name, (cx + 10, cy + 21), 0.52, WHITE)
        _label(img, src, (cx + 10, cy + 40), colour, 0.3)

        if lock.confidence > 0:
            s = f"{lock.confidence:.2f}"
            _text(img, s, (cx + cw - _tw(s, 0.46) - 10, cy + 21), 0.46,
                  colour)
            _sparkline(img, cx + cw - 64, cy + 29, 54, 14, self._conf,
                       colour, 0.0, 1.0)

    # --------------------------------------------------------------- header

    def _draw_header(self, bar, result, link, arbiter, pilot, outgoing):
        """Header.

        The RIGHT block is laid out first, from the edge inwards, and the left
        block is then clipped against wherever it ended. Laying out both from
        their own edge and hoping they meet in the middle is how a HUD ends up
        with the target's name printed underneath a mode pill — which is
        exactly what happened on a 640-wide capture.
        """
        w = bar.shape[1]
        lock = result.lock
        colour = STATE_COLOUR[lock.state]

        cv2.line(bar, (0, HEAD_H - 2), (w, HEAD_H - 2), colour, 2)

        # --- right, edge inwards ------------------------------------------
        rx = w - 16
        t = result.timings
        s = f"{t.fps:.1f}"
        rx -= _tw(s, 0.62)
        _text(bar, s, (rx, 34), 0.62, WHITE)
        rx -= 36
        _label(bar, "fps", (rx, 34), DIM, 0.3)
        rx -= 18
        cv2.line(bar, (rx, 14), (rx, HEAD_H - 12), RULE, 1)

        s = f"det {t.detect:.0f}   face {t.face:.0f}   tot {t.total:.0f} ms"
        rx -= _tw(s, 0.4) + 18
        _text(bar, s, (rx, 34), 0.4, GREY)
        rx -= 18
        cv2.line(bar, (rx, 14), (rx, HEAD_H - 12), RULE, 1)
        rx -= 16

        live = bool(outgoing is not None and outgoing.flags & FLAG_ARMED)
        # Whether the motors can turn is the single most consequential fact in
        # this window, so it is the loudest thing on the right and it is never
        # abbreviated.
        txt = "MOTORS LIVE" if live else "SAFE"
        rx -= _tw(txt, 0.5) + 22
        _pill(bar, rx, 14, 24, txt, WHITE if live else DIM,
              RED if live else PANEL, 0.5)
        rx -= 12

        if arbiter is not None:
            mode = arbiter.mode.value
            mcol = GREEN if mode == "FOLLOW" else STEEL
            rx -= _tw(mode, 0.5) + 22
            _pill(bar, rx, 14, 24, mode, INK, mcol, 0.5)
            rx -= 12

            if pilot is not None and not pilot.connected:
                s = "NO PILOT INPUT"
                rx -= _tw(s, 0.44) + 12
                _text(bar, s, (rx, 34), 0.44, RED)
                rx -= 12

        # --- left, up to that limit ---------------------------------------
        cv2.rectangle(bar, (16, 18), (28, 30), colour, -1)
        cv2.rectangle(bar, (16, 18), (28, 30), WHITE, 1)
        x = 38 + _label(bar, "pursuit-1", (38, 30), WHITE, 0.42, 3)
        x += 14
        cv2.line(bar, (x, 14), (x, HEAD_H - 12), RULE, 1)
        x += 16

        x += _pill(bar, x, 14, 24, lock.state.value, INK, colour, 0.5) + 18

        name = (lock.target_name or "ANY").upper()
        if x + _tw(name, 0.5) + 16 < rx:
            _label(bar, "target", (x, 21), DIM, 0.3)
            _text(bar, name, (x, 38), 0.5, WHITE)
            x += max(88, _tw(name, 0.5) + 24)

        if lock.state is LockState.COASTING and x + 70 < rx:
            _label(bar, "miss", (x, 21), DIM, 0.3)
            _text(bar, f"{lock.miss_frames}f", (x, 38), 0.5, AMBER)

    # ----------------------------------------------------------------- deck

    def _draw_deck(self, deck, result, link, arbiter, telemetry):
        w = deck.shape[1]
        cmd: Command = result.command
        cv2.line(deck, (0, 0), (w, 0), RULE, 1)

        # Pillarboxing guarantees at least CANVAS_MIN_W, so there is exactly
        # one deck layout and it is the one that gets looked at.
        cw = w // 4

        def col(i):
            return i * cw, 0

        for i in range(1, 4):
            cv2.line(deck, (i * cw, 8), (i * cw, DECK_H - 26), RULE, 1)

        # --- 1. what the vision stack is asking for -----------------------
        x0, y0 = col(0)
        _label(deck, "command", (x0 + 16, y0 + 20), DIM, 0.32)
        bx, bw = x0 + 16, cw - 32
        _meter(deck, bx, y0 + 44, bw, "yaw",
               f"{cmd.yaw_rate:+.1f} d/s", cmd.yaw_rate, 120.0, STEEL)
        _meter(deck, bx, y0 + 70, bw, "pitch",
               f"{cmd.pitch:+.1f} deg", cmd.pitch, 14.0, STEEL)
        # Unsigned, against the ceiling. The old version was signed about a
        # hard-coded 1.2 m, which was fair when the rangefinder capped the
        # aircraft at 2 m and became nonsense the moment a barometer took it
        # to 9.5: every normal setpoint pinned the bar and flagged saturation.
        _meter(deck, bx, y0 + 96, bw, "alt",
               f"{cmd.alt_sp:.2f} m", cmd.alt_sp, self.alt_ceiling, STEEL,
               signed=False)

        # --- 2. what the aircraft says it is doing ------------------------
        x0, y0 = col(1)
        _label(deck, "attitude", (x0 + 16, y0 + 20), DIM, 0.32)
        r = 40
        hx, hy = x0 + 16 + r, y0 + 40 + r
        _horizon(deck, hx, hy, r,
                 telemetry.roll if telemetry else 0.0,
                 telemetry.pitch if telemetry else 0.0,
                 telemetry is not None)
        tx = hx + r + 18
        for i, (lbl, val) in enumerate((
                ("roll", telemetry.roll if telemetry else None),
                ("pitch", telemetry.pitch if telemetry else None),
                ("yaw", telemetry.yaw if telemetry else None))):
            ty = y0 + 50 + i * 26
            _label(deck, lbl, (tx, ty), DIM, 0.3)
            s = f"{val:+6.1f}" if val is not None else "  --  "
            _text(deck, s, (tx + 46, ty), 0.48,
                  WHITE if val is not None else DIM)

        # --- 3. the aircraft itself ---------------------------------------
        x0, y0 = col(2)
        _label(deck, "aircraft", (x0 + 16, y0 + 20), DIM, 0.32)
        px = x0 + 16
        if telemetry is None:
            _pill(deck, px, y0 + 30, 24, "NO TELEMETRY", DIM, PANEL, 0.46)
        else:
            _pill(deck, px, y0 + 30, 24, telemetry.state_name, INK, STEEL,
                  0.46)
            # Battery as a bar over the usable pack range, not a bare number:
            # 3.72 V means nothing until you know it sits just above the
            # limiter's threshold.
            v = telemetry.vbat
            vcol = RED if v < 3.5 else (AMBER if v < 3.7 else GREEN)
            frac = max(0.0, min(1.0, (v - 3.3) / (4.2 - 3.3)))
            bw2 = cw - 110
            by = y0 + 74
            cv2.line(deck, (px, by), (px + bw2, by), RULE, 6, cv2.LINE_AA)
            if frac > 0:
                cv2.line(deck, (px, by), (px + int(bw2 * frac), by), vcol, 6,
                         cv2.LINE_AA)
            # The point at which the firmware starts clamping throttle.
            lx = px + int(bw2 * (3.6 - 3.3) / 0.9)
            cv2.line(deck, (lx, by - 6), (lx, by + 6), RED, 1, cv2.LINE_AA)
            _text(deck, f"{v:.2f} V", (px + bw2 + 12, by + 5), 0.46, vcol)

            _label(deck, "loop", (px, y0 + 104), DIM, 0.3)
            _text(deck, f"{telemetry.loop_hz} Hz", (px + 46, y0 + 104), 0.44,
                  WHITE if telemetry.loop_hz > 800 else AMBER)
            _label(deck, "alt", (px + 130, y0 + 104), DIM, 0.3)
            _text(deck, f"{telemetry.alt:.2f} m", (px + 168, y0 + 104), 0.44,
                  WHITE)

        # --- 4. the link ---------------------------------------------------
        x0, y0 = col(3)
        _label(deck, "link", (x0 + 16, y0 + 20), DIM, 0.32)
        px = x0 + 16
        if link is None or not link.enabled:
            _pill(deck, px, y0 + 30, 24, "OFFLINE — TEST MODE", DIM, PANEL,
                  0.44)
        else:
            up = link.link_up
            _pill(deck, px, y0 + 30, 24, "UP" if up else "NO TELEMETRY",
                  INK if up else WHITE, GREEN if up else RED, 0.46)
            s = link.stats
            # Round trip, top right of the column. It is not a diagnostic —
            # the controller's lead compensation is derived from it, and the
            # stability boundary without attitude compensation is ~17 ms, so
            # this number is the difference between settling and oscillating.
            rtt = getattr(s, "rtt_ms", 0.0) or 0.0
            _label(deck, "rtt", (x0 + cw - 96, y0 + 48), DIM, 0.3)
            _text(deck, f"{rtt:.0f} ms" if rtt else "--",
                  (x0 + cw - 62, y0 + 48), 0.46,
                  WHITE if rtt < 40 else AMBER)

            grid = (("tx", s.sent, WHITE), ("rx", s.received, WHITE),
                    ("bad", s.rejected, AMBER if s.rejected else DIM),
                    ("err", s.send_errors, RED if s.send_errors else DIM))
            for i, (lbl, val, c) in enumerate(grid):
                gx = px + (i % 2) * 118
                gy = y0 + 82 + (i // 2) * 26
                _label(deck, lbl, (gx, gy), DIM, 0.3)
                _text(deck, str(val), (gx + 34, gy), 0.44, c)

        # --- key hints ------------------------------------------------------
        if arbiter is not None:
            keys = ("[1] manual   [2] assist   [3] follow   "
                    "hold SPACE deadman   wasd/qe/ik fly   "
                    "[f] fullscreen   [s] spotlight   [x] quit")
        else:
            keys = ("[space] arm   [r] reset lock   [t] tracks   "
                    "[f] fullscreen   [s] spotlight   [p] pause   [q] quit")
        cv2.line(deck, (0, DECK_H - 22), (w, DECK_H - 22), RULE, 1)
        _label(deck, keys, (16, DECK_H - 7), DIM, 0.32, 1)

    # ----------------------------------------------------------------- draw

    def draw(self, result, link=None, armed: bool = False,
             paused: bool = False, arbiter=None, pilot=None,
             outgoing=None) -> np.ndarray:
        self._t += 1
        lock = result.lock

        # Track confidence history and the acquisition animation.
        if lock.state is LockState.LOCKED:
            if self._prev_state is not LockState.LOCKED:
                self._acq = 0.0
            self._acq = min(1.0, self._acq + 0.18)
        else:
            self._acq = 1.0
        self._prev_state = lock.state

        if lock.has_target:
            self._conf.append(float(lock.confidence))
            if len(self._conf) > 90:
                self._conf.pop(0)
        else:
            self._conf.clear()

        telemetry = link.telemetry if (link is not None and link.enabled) \
            else None

        video = result.frame.copy()
        self._draw_video(video, result, telemetry)

        h, w = video.shape[:2]
        cw = max(w, CANVAS_MIN_W)
        if self.fill_aspect:
            cw = max(cw, int(round((HEAD_H + h + DECK_H) * self.fill_aspect)))
        xo = (cw - w) // 2
        canvas = np.full((HEAD_H + h + DECK_H, cw, 3), INK, np.uint8)
        canvas[HEAD_H:HEAD_H + h, xo:xo + w] = video
        if xo:
            # Hairlines down both sides of the picture, so a pillarboxed frame
            # reads as inset rather than as the window failing to fill.
            for lx in (xo - 1, xo + w):
                cv2.line(canvas, (lx, HEAD_H), (lx, HEAD_H + h), RULE, 1)

        # `armed` is the keyboard path's only authority — it has no arbiter and
        # may pass no outgoing packet — so it is folded into the same flag the
        # header reads, rather than shown separately somewhere else.
        cmd_shown = outgoing if outgoing is not None else result.command
        if outgoing is None and armed:
            cmd_shown = Command(yaw_rate=cmd_shown.yaw_rate,
                                pitch=cmd_shown.pitch,
                                alt_sp=cmd_shown.alt_sp,
                                flags=cmd_shown.flags | FLAG_ARMED)

        self._draw_header(canvas[:HEAD_H], result, link, arbiter, pilot,
                          cmd_shown)
        self._draw_deck(canvas[HEAD_H + h:], result, link, arbiter, telemetry)

        if paused:
            s = "PAUSED"
            tw = _tw(s, 1.0, 2)
            _pill(canvas, (cw - tw) // 2 - 18, HEAD_H + h // 2 - 22, 44, s,
                  INK, AMBER, 1.0, 18)

        return canvas
