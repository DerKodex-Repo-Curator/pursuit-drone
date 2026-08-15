#!/usr/bin/env python3
"""PURSUIT-1 process mind-map — the life of one decision, photons to props.

Distinct from pursuit_mindmap.png, which is the static system view. This one
is the RUNTIME PATH: a single camera frame becoming four motor commands, laid
out as a left-to-right serpentine so the eye follows the flow.

    docs/media/.../python make_process_map.py

Deliberately no external assets — pure matplotlib, so it regenerates anywhere
the vision venv runs and the numbers can be refreshed in one place.
"""
from __future__ import annotations

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, PathPatch
from matplotlib.path import Path

# ---- palette (matches the dark system map) --------------------------------
BG      = "#0b0f16"
NODE    = "#141b26"
INK     = "#e8eaed"
MUTE    = "#8b93a0"
RULE    = "#2a333f"
PURPLE  = "#9b8cf0"   # perception, on the laptop
BLUE    = "#5aa9e6"   # the radio link
GREEN   = "#4fbf87"   # flight, on the aircraft
AMBER   = "#e0a83c"   # the safety net

W, H = 26.0, 15.5     # inches; dpi below sets the pixel size

# ---- the process, in order ------------------------------------------------
# (title, subtitle, colour, row)  row 0 = top lane, 1 = turn, 2 = bottom lane
STAGES = [
    ("SEE",        "OV2640 to JPEG over WiFi\n800x600, blur-proof exposure", PURPLE, 0),
    ("DETECT",     "YOLO11n, person only\nconf 0.15 - measured, not taste",  PURPLE, 0),
    ("TRACK",      "ByteTrack, 3-pass\nIDs survive an occlusion",            PURPLE, 0),
    ("IDENTIFY",   "SCRFD to ArcFace 512-d\nface veto <0.15; body sustains", PURPLE, 0),
    ("AIM",        "PD visual servo\nattitude-compensated (gain 1.31)",      PURPLE, 0),
    ("SEND",       "30-byte packet\nCRC-16, 30 Hz, x3 redundant",            BLUE,   1),
    ("VERIFY",     "magic / ver / CRC / sequence\nARM authority held each packet", GREEN, 2),
    ("ESTIMATE",   "ICM-42688 at 1 kHz\nMadgwick 6-axis",                    GREEN,  2),
    ("STABILISE",  "angle P to rate PID\nD on measurement, no kick",         GREEN,  2),
    ("DRIVE",      "desaturating mixer\n32 kHz staggered PWM to 4 motors",   GREEN,  2),
]

# where each row sits vertically, and the node size
Y = {0: 11.6, 1: 8.0, 2: 3.9}
NW, NH = 3.6, 1.9

def layout():
    """Assign an (x, y) centre to every stage along the serpentine."""
    top = [i for i, s in enumerate(STAGES) if s[3] == 0]
    turn = [i for i, s in enumerate(STAGES) if s[3] == 1]
    bot = [i for i, s in enumerate(STAGES) if s[3] == 2]

    xs = {}
    left, right = 2.6, W - 2.6
    # top lane, left -> right
    for k, i in enumerate(top):
        xs[i] = (left + (right - left) * k / (len(top) - 1), Y[0])
    # the turn node sits at the far right, mid height
    for i in turn:
        xs[i] = (right, Y[1])
    # bottom lane, right -> left (so the flow doubles back)
    for k, i in enumerate(bot):
        xs[i] = (right - (right - left) * k / (len(bot) - 1), Y[2])
    return xs


def node(ax, x, y, title, sub, colour):
    box = FancyBboxPatch(
        (x - NW / 2, y - NH / 2), NW, NH,
        boxstyle="round,pad=0.02,rounding_size=0.18",
        linewidth=1.4, edgecolor=colour, facecolor=NODE, zorder=3)
    ax.add_patch(box)
    # a colour tab down the left edge, like the system map's accent
    ax.add_patch(FancyBboxPatch(
        (x - NW / 2, y - NH / 2), 0.10, NH,
        boxstyle="round,pad=0,rounding_size=0.06",
        linewidth=0, facecolor=colour, zorder=4))
    ax.text(x - NW / 2 + 0.32, y + 0.42, title, color=INK, fontsize=15,
            fontweight="bold", va="center", ha="left", zorder=5)
    ax.text(x - NW / 2 + 0.32, y - 0.34, sub, color=MUTE, fontsize=9.5,
            va="center", ha="left", zorder=5, linespacing=1.5)


def link(ax, p0, p1, colour):
    """A smooth connector between two node centres, entering/leaving edges."""
    (x0, y0), (x1, y1) = p0, p1
    # leave the right edge / enter the left edge for same-row hops; for the
    # vertical turn, leave the bottom and enter the top.
    if abs(y0 - y1) < 0.5:                       # same lane
        sx, sy = x0 + NW / 2, y0
        ex, ey = x1 - NW / 2, y1
        c1, c2 = (sx + (ex - sx) * 0.5, sy), (sx + (ex - sx) * 0.5, ey)
    else:                                        # the drop between lanes
        sx, sy = x0, y0 - NH / 2
        ex, ey = x1, y1 + NH / 2
        c1, c2 = (sx, (sy + ey) / 2), (ex, (sy + ey) / 2)
    path = Path([(sx, sy), c1, c2, (ex, ey)],
                [Path.MOVETO, Path.CURVE4, Path.CURVE4, Path.CURVE4])
    ax.add_patch(PathPatch(path, edgecolor=colour, facecolor="none",
                           linewidth=1.6, alpha=0.55, zorder=2))
    # a small arrowhead at the destination, so direction is unmistakable
    import math
    ang = math.atan2(ey - c2[1], ex - c2[0])
    a = 0.16
    ax.add_patch(plt.Polygon([
        (ex, ey),
        (ex - a * math.cos(ang - 0.5), ey - a * math.sin(ang - 0.5)),
        (ex - a * math.cos(ang + 0.5), ey - a * math.sin(ang + 0.5)),
    ], closed=True, color=colour, alpha=0.8, zorder=3))


def main():
    fig, ax = plt.subplots(figsize=(W, H))
    fig.patch.set_facecolor(BG)
    ax.set_facecolor(BG)
    ax.set_xlim(0, W)
    ax.set_ylim(0, H)
    ax.axis("off")

    # header
    ax.text(1.0, H - 0.7, "PURSUIT-1  ·  THE RUNTIME PATH",
            color=BLUE, fontsize=12, fontweight="bold", family="monospace")
    ax.text(1.0, H - 1.35, "One camera frame becomes four motor commands",
            color=INK, fontsize=22, fontweight="bold")

    # lane captions
    ax.text(1.0, Y[0] + NH / 2 + 0.55, "ON THE LAPTOP  —  perception",
            color=PURPLE, fontsize=11, fontweight="bold")
    ax.text(W - 1.0, Y[2] - NH / 2 - 0.75,
            "ON THE AIRCRAFT  —  1 kHz flight loop",
            color=GREEN, fontsize=11, fontweight="bold", ha="right")

    xs = layout()
    order = list(range(len(STAGES)))
    for a, b in zip(order, order[1:]):
        link(ax, xs[a], xs[b], STAGES[b][2])
    for i, (title, sub, colour, _) in enumerate(STAGES):
        node(ax, xs[i][0], xs[i][1], title, sub, colour)

    # the safety net, sitting under the flight lane
    ax.text(W / 2, 1.5,
            "underneath, always:  watchdog 400 ms → level off, 3 s → land"
            "      ·      altitude: rangefinder + barometer, fused",
            color=AMBER, fontsize=10.5, ha="center", style="italic")

    # footer
    ax.plot([1.0, W - 1.0], [0.75, 0.75], color=RULE, linewidth=1)
    ax.text(1.0, 0.4, "github.com/Pai-88/pursuit-drone", color=MUTE,
            fontsize=10, family="monospace")
    ax.text(W - 1.0, 0.4, "never flown · boards in fabrication",
            color=MUTE, fontsize=10, ha="right")

    out = __file__.rsplit("/", 1)[0] + "/pursuit_process_map.png"
    fig.savefig(out, dpi=170, facecolor=BG, bbox_inches="tight", pad_inches=0.25)
    print("wrote", out)


if __name__ == "__main__":
    main()
