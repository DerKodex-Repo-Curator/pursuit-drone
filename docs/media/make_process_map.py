#!/usr/bin/env python3
"""PURSUIT-1 process mind-map — the life of one decision, photons to props.

Distinct from pursuit_mindmap.png, which is the static system view. This one
is the RUNTIME PATH: a single camera frame becoming four motor commands, laid
out as a left-to-right serpentine so the eye follows the flow.

    docs/media/.../python make_process_map.py

Pure matplotlib, no external assets, so it regenerates anywhere the vision
venv runs and the numbers live in one place. The visual polish — the glow
haloes, the flow dots, the numbered badges, the graded background — is all
done with layered patches rather than a rasteriser, so there is no new
dependency to break.
"""
from __future__ import annotations

import math

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Circle, FancyBboxPatch, PathPatch, Polygon
from matplotlib.path import Path

# ---- palette --------------------------------------------------------------
INK     = "#eef1f6"
MUTE    = "#9aa3b2"
FAINT   = "#5b6472"
NODE    = "#151c28"
NODE_HI = "#26303f"   # top-edge glass highlight
PURPLE  = "#a394f5"   # perception, on the laptop
BLUE    = "#5cb0f0"   # the radio link
GREEN   = "#54c88f"   # flight, on the aircraft
AMBER   = "#e6b048"   # the safety net

def _rgb(h):
    h = h.lstrip("#")
    return tuple(int(h[i:i + 2], 16) / 255 for i in (0, 2, 4))

W, H = 26.0, 15.5
NW, NH = 3.75, 2.0

# ---- the process, in order ------------------------------------------------
# (num, title, role, subtitle, colour, row)   row 0 top, 1 turn, 2 bottom
STAGES = [
    ("01", "SEE",       "capture",  "OV2640 to JPEG over WiFi\n800x600, blur-proof exposure", PURPLE, 0),
    ("02", "DETECT",    "find",     "YOLO11n, person only\nconf 0.15 - measured, not taste",  PURPLE, 0),
    ("03", "TRACK",     "persist",  "ByteTrack, 3-pass\nIDs survive an occlusion",            PURPLE, 0),
    ("04", "IDENTIFY",  "is it them","SCRFD to ArcFace 512-d\nface veto <0.15; body sustains", PURPLE, 0),
    ("05", "AIM",       "servo",    "PD, attitude-compensated\nloop gain 1.31 through the airframe", PURPLE, 0),
    ("06", "SEND",      "radio",    "30-byte packet\nCRC-16, 30 Hz, x3 redundant",            BLUE,   1),
    ("07", "VERIFY",    "trust no one","magic / ver / CRC / sequence\nARM authority held each packet", GREEN, 2),
    ("08", "ESTIMATE",  "attitude", "ICM-42688 at 1 kHz\nMadgwick 6-axis",                    GREEN,  2),
    ("09", "STABILISE", "cascade",  "angle P to rate PID\nD on measurement, no kick",         GREEN,  2),
    ("10", "DRIVE",     "motors",   "desaturating mixer\n32 kHz staggered PWM x4",            GREEN,  2),
]

Y = {0: 11.7, 1: 8.0, 2: 3.7}


def layout():
    top = [i for i, s in enumerate(STAGES) if s[5] == 0]
    turn = [i for i, s in enumerate(STAGES) if s[5] == 1]
    bot = [i for i, s in enumerate(STAGES) if s[5] == 2]
    xs = {}
    left, right = 2.9, W - 2.9
    for k, i in enumerate(top):
        xs[i] = (left + (right - left) * k / (len(top) - 1), Y[0])
    for i in turn:
        xs[i] = (right, Y[1])
    for k, i in enumerate(bot):
        xs[i] = (right - (right - left) * k / (len(bot) - 1), Y[2])
    return xs


def background(ax):
    """A soft radial lift behind the content, plus a faint technical dot grid.

    Flat black reads as a slide template; a gentle vignette gives the canvas
    depth and pulls the eye to the middle of the flow without ever becoming a
    gradient you notice.
    """
    res = 300
    aspect = W / H
    yy, xx = np.mgrid[0:res, 0:int(res * aspect)]
    nx, ny = xx / xx.max(), yy / yy.max()
    d = np.sqrt((nx - 0.5) ** 2 + ((ny - 0.55) * 1.15) ** 2)
    g = np.clip(1 - d * 1.05, 0, 1) ** 1.4
    c_hi, c_lo = np.array(_rgb("#1a2330")), np.array(_rgb("#070a10"))
    img = c_lo[None, None, :] + g[..., None] * (c_hi - c_lo)[None, None, :]
    ax.imshow(img, extent=[0, W, 0, H], origin="lower", aspect="auto",
              zorder=0, interpolation="bilinear")
    gx, gy = np.meshgrid(np.arange(1, W, 0.62), np.arange(1, H, 0.62))
    ax.scatter(gx, gy, s=1.4, c="#ffffff", alpha=0.045, zorder=1,
               edgecolors="none")


def glow(ax, x, y, w, h, colour, z):
    """A soft bloom behind a node — layered rounded rects, no rasteriser."""
    for grow, alpha in ((0.62, 0.05), (0.40, 0.06), (0.20, 0.08)):
        ax.add_patch(FancyBboxPatch(
            (x - w / 2 - grow, y - h / 2 - grow), w + 2 * grow, h + 2 * grow,
            boxstyle="round,pad=0.02,rounding_size=0.30",
            linewidth=0, facecolor=colour, alpha=alpha, zorder=z))


def node(ax, x, y, num, title, role, sub, colour):
    glow(ax, x, y, NW, NH, colour, 2)
    ax.add_patch(FancyBboxPatch(
        (x - NW / 2, y - NH / 2), NW, NH,
        boxstyle="round,pad=0.02,rounding_size=0.20",
        linewidth=1.5, edgecolor=colour, facecolor=NODE, zorder=4))
    # glassy top-edge highlight
    ax.add_patch(FancyBboxPatch(
        (x - NW / 2 + 0.12, y + NH / 2 - 0.14), NW - 0.24, 0.10,
        boxstyle="round,pad=0,rounding_size=0.05",
        linewidth=0, facecolor=NODE_HI, alpha=0.7, zorder=5))
    # left accent tab
    ax.add_patch(FancyBboxPatch(
        (x - NW / 2, y - NH / 2 + 0.12), 0.11, NH - 0.24,
        boxstyle="round,pad=0,rounding_size=0.05",
        linewidth=0, facecolor=colour, zorder=5))
    # numbered badge, top-right
    bx, by = x + NW / 2 - 0.46, y + NH / 2 - 0.46
    ax.add_patch(Circle((bx, by), 0.30, facecolor=colour, edgecolor="none",
                        zorder=6))
    ax.text(bx, by, num, color="#0b0f16", fontsize=11, fontweight="bold",
            ha="center", va="center", zorder=7)
    # text block
    lx = x - NW / 2 + 0.36
    ax.text(lx, y + 0.50, title, color=INK, fontsize=16, fontweight="bold",
            va="center", ha="left", zorder=7)
    ax.text(lx, y + 0.06, role.upper(), color=colour, fontsize=8.5,
            fontweight="bold", va="center", ha="left", zorder=7,
            family="monospace")
    ax.text(lx, y - 0.52, sub, color=MUTE, fontsize=9.5, va="center",
            ha="left", zorder=7, linespacing=1.5)


def bezier(p0, c1, c2, p1, n=60):
    t = np.linspace(0, 1, n)[:, None]
    return ((1 - t) ** 3 * p0 + 3 * (1 - t) ** 2 * t * c1
            + 3 * (1 - t) * t ** 2 * c2 + t ** 3 * p1)


def link(ax, p0, p1, colour):
    (x0, y0), (x1, y1) = p0, p1
    if abs(y0 - y1) < 0.5:
        s = np.array([x0 + NW / 2, y0]); e = np.array([x1 - NW / 2, y1])
        mid = (s[0] + e[0]) / 2
        c1, c2 = np.array([mid, s[1]]), np.array([mid, e[1]])
    else:
        s = np.array([x0, y0 - NH / 2]); e = np.array([x1, y1 + NH / 2])
        midy = (s[1] + e[1]) / 2
        c1, c2 = np.array([s[0], midy]), np.array([e[0], midy])
    pts = bezier(s, c1, c2, e)
    codes = [Path.MOVETO, Path.CURVE4, Path.CURVE4, Path.CURVE4]
    path = Path([s, c1, c2, e], codes)
    # glow pass, then the crisp line on top
    ax.add_patch(PathPatch(path, edgecolor=colour, facecolor="none",
                           linewidth=6, alpha=0.10, zorder=1))
    ax.add_patch(PathPatch(path, edgecolor=colour, facecolor="none",
                           linewidth=1.8, alpha=0.75, zorder=2))
    # flow dots — data moving along the wire
    for tt in (0.30, 0.52, 0.74):
        pt = pts[int(tt * (len(pts) - 1))]
        ax.add_patch(Circle(pt, 0.055, facecolor=colour, edgecolor="none",
                            alpha=0.9, zorder=3))
    # arrowhead
    d = pts[-1] - pts[-5]; ang = math.atan2(d[1], d[0]); a = 0.20
    ax.add_patch(Polygon([
        e,
        e - a * np.array([math.cos(ang - 0.5), math.sin(ang - 0.5)]),
        e - a * np.array([math.cos(ang + 0.5), math.sin(ang + 0.5)]),
    ], closed=True, color=colour, zorder=3))


def main():
    fig, ax = plt.subplots(figsize=(W, H))
    ax.set_xlim(0, W); ax.set_ylim(0, H); ax.axis("off")
    background(ax)

    ax.text(1.0, H - 0.7, "PURSUIT-1   ·   THE RUNTIME PATH", color=BLUE,
            fontsize=12.5, fontweight="bold", family="monospace")
    ax.text(1.0, H - 1.45, "One camera frame becomes four motor commands",
            color=INK, fontsize=24, fontweight="bold")
    ax.plot([1.02, 9.6], [H - 1.95, H - 1.95], color=BLUE, linewidth=2.2,
            alpha=0.8)

    ax.text(2.9, Y[0] + NH / 2 + 0.62, "ON THE LAPTOP  ·  perception",
            color=PURPLE, fontsize=11.5, fontweight="bold")
    ax.text(W - 2.9, Y[2] - NH / 2 - 0.78,
            "ON THE AIRCRAFT  ·  1 kHz flight loop", color=GREEN,
            fontsize=11.5, fontweight="bold", ha="right")

    xs = layout()
    order = list(range(len(STAGES)))
    for a, b in zip(order, order[1:]):
        link(ax, xs[a], xs[b], STAGES[b][4])
    for i, (num, title, role, sub, colour, _) in enumerate(STAGES):
        node(ax, xs[i][0], xs[i][1], num, title, role, sub, colour)

    # the safety net, as a pill under the flight lane
    ax.add_patch(FancyBboxPatch(
        (W / 2 - 8.6, 1.15), 17.2, 0.72,
        boxstyle="round,pad=0.02,rounding_size=0.36",
        linewidth=1.2, edgecolor=AMBER, facecolor="#1c1708", alpha=0.9,
        zorder=3))
    ax.text(W / 2, 1.51,
            "always, underneath:   watchdog 400 ms → level off · 3 s → land"
            "      ·      altitude: rangefinder + barometer, fused",
            color=AMBER, fontsize=10.5, ha="center", va="center", zorder=4)

    ax.plot([1.0, W - 1.0], [0.68, 0.68], color="#232c38", linewidth=1)
    ax.text(1.0, 0.34, "github.com/Pai-88/pursuit-drone", color=FAINT,
            fontsize=10, family="monospace")
    ax.text(W - 1.0, 0.34, "never flown · boards in fabrication",
            color=FAINT, fontsize=10, ha="right")

    out = __file__.rsplit("/", 1)[0] + "/pursuit_process_map.png"
    fig.savefig(out, dpi=175, facecolor="#070a10", bbox_inches="tight",
                pad_inches=0.2)
    print("wrote", out)


if __name__ == "__main__":
    main()
