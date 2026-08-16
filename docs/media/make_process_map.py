#!/usr/bin/env python3
"""PURSUIT-1 process mind-map — the life of one decision, photons to props.

The RUNTIME PATH: a single camera frame becoming four motor commands, laid
out as a left-to-right serpentine so the eye follows the flow. Light,
editorial style to match the carousel — flat cards, crisp borders, one accent
per lane, nothing decorative competing with the content.

    docs/media/.../python make_process_map.py

Pure matplotlib, no external assets; the numbers live in one place.
"""
from __future__ import annotations

import math

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Circle, FancyBboxPatch, PathPatch, Polygon
from matplotlib.path import Path

# ---- light palette --------------------------------------------------------
BG     = "#ffffff"
INK    = "#18202c"
MUTE   = "#5a6472"
FAINT  = "#9aa3b0"
CARD   = "#ffffff"
INDIGO = "#4f46e5"   # perception, on the laptop
AMBER  = "#c2740c"   # the radio link
GREEN  = "#1f9d57"   # flight, on the aircraft
RED    = "#c0392b"   # the safety net

W, H = 30.0, 16.0
NW, NH = 3.5, 2.05

# ---- the process, in order ------------------------------------------------
# (num, title, role, subtitle, colour, row)   row 0 top, 1 turn, 2 bottom
STAGES = [
    ("01", "CAPTURE",   "see",         "OV2640 to JPEG over WiFi\n800x600, blur-proof exposure", INDIGO, 0),
    ("02", "DETECT",    "find",        "YOLO11n, person only\nconf 0.15 - measured, not taste",  INDIGO, 0),
    ("03", "TRACK",     "persist",     "ByteTrack, 3-pass\nIDs survive an occlusion",            INDIGO, 0),
    ("04", "IDENTIFY",  "is it them",  "ArcFace 512-d + body ReID\nface veto <0.15",             INDIGO, 0),
    ("05", "LOCK",      "commit",      "4-state machine\ncoasts through an occlusion",           INDIGO, 0),
    ("06", "AIM",       "servo",       "PD, attitude-compensated\nloop gain 1.31 thru the frame",INDIGO, 0),
    ("07", "ARBITRATE", "who flies",   "manual / assist / follow\nhold-to-fly deadman",          INDIGO, 0),
    ("08", "TRANSMIT",  "link",        "30-byte packet\nCRC-16, 30 Hz, x3 redundant",            AMBER,  1),
    ("09", "VALIDATE",  "trust no one","magic / ver / CRC / seq\nARM held every packet",         GREEN,  2),
    ("10", "ESTIMATE",  "attitude",    "ICM-42688 at 1 kHz\nMadgwick 6-axis",                    GREEN,  2),
    ("11", "FUSE",      "one height",  "rangefinder + barometer\nblended across the handover",   GREEN,  2),
    ("12", "STABILISE", "cascade",     "angle P to rate PID\nD on measurement, no kick",         GREEN,  2),
    ("13", "PROTECT",   "stay alive",  "battery sag limiter\n400 ms level off, 3 s land",        GREEN,  2),
    ("14", "DRIVE",     "motors",      "desaturating mixer\n32 kHz staggered PWM x4",            GREEN,  2),
]

Y = {0: 12.3, 1: 8.2, 2: 3.7}


def layout():
    top = [i for i, s in enumerate(STAGES) if s[5] == 0]
    turn = [i for i, s in enumerate(STAGES) if s[5] == 1]
    bot = [i for i, s in enumerate(STAGES) if s[5] == 2]
    xs = {}
    left, right = 2.7, W - 2.7
    for k, i in enumerate(top):
        xs[i] = (left + (right - left) * k / (len(top) - 1), Y[0])
    for i in turn:
        xs[i] = (right, Y[1])
    for k, i in enumerate(bot):
        xs[i] = (right - (right - left) * k / (len(bot) - 1), Y[2])
    return xs


def node(ax, x, y, num, title, role, sub, colour):
    ax.add_patch(FancyBboxPatch(
        (x - NW / 2, y - NH / 2), NW, NH,
        boxstyle="round,pad=0.02,rounding_size=0.16",
        linewidth=1.4, edgecolor=colour, facecolor=CARD, zorder=4))
    ax.add_patch(FancyBboxPatch(
        (x - NW / 2, y - NH / 2 + 0.10), 0.10, NH - 0.20,
        boxstyle="round,pad=0,rounding_size=0.04",
        linewidth=0, facecolor=colour, zorder=5))
    bx, by = x + NW / 2 - 0.44, y + NH / 2 - 0.44
    ax.add_patch(Circle((bx, by), 0.29, facecolor=colour, edgecolor="none",
                        zorder=6))
    ax.text(bx, by, num, color="#ffffff", fontsize=10.5, fontweight="bold",
            ha="center", va="center", zorder=7)
    lx = x - NW / 2 + 0.36
    ax.text(lx, y + 0.52, title, color=INK, fontsize=15.5, fontweight="bold",
            va="center", ha="left", zorder=7)
    ax.text(lx, y + 0.10, role.upper(), color=colour, fontsize=8.5,
            fontweight="bold", va="center", ha="left", zorder=7,
            family="monospace")
    ax.text(lx, y - 0.50, sub, color=MUTE, fontsize=9.5, va="center",
            ha="left", zorder=7, linespacing=1.5)


def link(ax, p0, p1, colour):
    (x0, y0), (x1, y1) = p0, p1
    if abs(y0 - y1) < 0.5:
        s = np.array([x0 + NW / 2, y0]); e = np.array([x1 - NW / 2, y1])
        m = (s[0] + e[0]) / 2
        c1, c2 = np.array([m, s[1]]), np.array([m, e[1]])
    else:
        s = np.array([x0, y0 - NH / 2]); e = np.array([x1, y1 + NH / 2])
        my = (s[1] + e[1]) / 2
        c1, c2 = np.array([s[0], my]), np.array([e[0], my])
    path = Path([s, c1, c2, e],
                [Path.MOVETO, Path.CURVE4, Path.CURVE4, Path.CURVE4])
    ax.add_patch(PathPatch(path, edgecolor=colour, facecolor="none",
                           linewidth=1.6, alpha=0.55, zorder=2))
    d = e - c2; ang = math.atan2(d[1], d[0]); a = 0.19
    ax.add_patch(Polygon([
        e,
        e - a * np.array([math.cos(ang - 0.5), math.sin(ang - 0.5)]),
        e - a * np.array([math.cos(ang + 0.5), math.sin(ang + 0.5)]),
    ], closed=True, color=colour, alpha=0.9, zorder=3))


def hero(ax):
    """Central identity + headline stats, filling the space the U flows around.

    The lanes answer "how does it work"; this answers "what is it" and gives
    the picture a focal point instead of a dead middle band. Every number here
    is measured — the count is `pio test -e native` + the vision and sim
    suites, not a claim.
    """
    x0, x1 = 6.6, W - 6.6
    y0, y1 = 6.1, 9.9
    cx = (x0 + x1) / 2
    ax.add_patch(FancyBboxPatch(
        (x0, y0), x1 - x0, y1 - y0,
        boxstyle="round,pad=0.02,rounding_size=0.22",
        linewidth=1.6, edgecolor="#d3d8e0", facecolor="#f7f8fb", zorder=3))
    ax.add_patch(FancyBboxPatch(
        (x0, y1 - 0.14), x1 - x0, 0.14,
        boxstyle="round,pad=0,rounding_size=0.05",
        linewidth=0, facecolor=INDIGO, zorder=4))

    ax.text(cx, 9.25, "PCB-AS-AIRFRAME  ·  PERSON-FOLLOWING MICRO-DRONE",
            color=INDIGO, fontsize=11, fontweight="bold", ha="center",
            family="monospace", zorder=5)
    ax.text(cx, 8.45, "PURSUIT-1", color=INK, fontsize=34, fontweight="bold",
            ha="center", va="center", zorder=5)
    ax.text(cx, 7.65,
            "A 92 mm circuit board that is its own airframe — it follows one "
            "enrolled person by face re-identification.",
            color=MUTE, fontsize=11.5, ha="center", va="center", zorder=5)

    stats = [("66 g", "all-up weight"), ("337", "tests, all green"),
             ("1 kHz", "flight control loop"), ("25 fps", "vision, on a laptop")]
    for i, (big, lab) in enumerate(stats):
        sx = x0 + (x1 - x0) * (i + 0.5) / len(stats)
        ax.text(sx, 6.95, big, color=INDIGO, fontsize=21, fontweight="bold",
                ha="center", va="center", zorder=5)
        ax.text(sx, 6.42, lab.upper(), color=FAINT, fontsize=8.5, ha="center",
                va="center", zorder=5, family="monospace")
        if i:
            ax.plot([sx - (x1 - x0) / len(stats) / 2] * 2, [6.35, 7.05],
                    color="#dfe3ea", linewidth=1, zorder=4)


def main():
    fig, ax = plt.subplots(figsize=(W, H))
    fig.patch.set_facecolor(BG); ax.set_facecolor(BG)
    ax.set_xlim(0, W); ax.set_ylim(0, H); ax.axis("off")

    ax.text(1.0, H - 0.7, "PURSUIT-1   ·   THE RUNTIME PATH", color=INDIGO,
            fontsize=12.5, fontweight="bold", family="monospace")
    ax.text(1.0, H - 1.5, "One camera frame becomes four motor commands",
            color=INK, fontsize=25, fontweight="bold")

    ax.text(2.7, Y[0] + NH / 2 + 0.62, "ON THE LAPTOP  ·  perception + control",
            color=INDIGO, fontsize=12, fontweight="bold")
    ax.text(W - 2.7, Y[2] - NH / 2 - 0.78,
            "ON THE AIRCRAFT  ·  1 kHz flight loop", color=GREEN,
            fontsize=12, fontweight="bold", ha="right")

    hero(ax)

    xs = layout()
    order = list(range(len(STAGES)))
    for a, b in zip(order, order[1:]):
        link(ax, xs[a], xs[b], STAGES[b][4])
    for i, (num, title, role, sub, colour, _) in enumerate(STAGES):
        node(ax, xs[i][0], xs[i][1], num, title, role, sub, colour)

    ax.plot([1.0, W - 1.0], [0.68, 0.68], color="#e2e6ec", linewidth=1)
    ax.text(1.0, 0.34, "github.com/Pai-88/pursuit-drone", color=FAINT,
            fontsize=10, family="monospace")
    ax.text(W - 1.0, 0.34, "never flown · boards in fabrication",
            color=FAINT, fontsize=10, ha="right")

    out = __file__.rsplit("/", 1)[0] + "/pursuit_process_map.png"
    fig.savefig(out, dpi=170, facecolor=BG, bbox_inches="tight",
                pad_inches=0.25)
    print("wrote", out)


if __name__ == "__main__":
    main()
