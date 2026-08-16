#!/usr/bin/env python3
"""PURSUIT-1 process map — how it works, in six steps a non-engineer can hold.

    docs/media/.../python make_process_map.py

There is a longer, denser version of this story in pursuit_mindmap.png and in
the README. This one is deliberately the OPPOSITE: the fewest boxes that still
tell the truth, in plain words, so somebody scrolling past understands the
whole machine in about ten seconds.

Emoji are rendered through PIL from Apple Color Emoji and composited as
images. Matplotlib cannot draw colour emoji itself — its default font has no
glyphs for most of them and silently draws empty boxes — so drawing them as
text would produce tofu on someone else's machine and nobody would notice
until it was already posted.
"""
from __future__ import annotations

import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Circle, FancyBboxPatch, FancyArrow, Polygon
from PIL import Image, ImageDraw, ImageFont

# ---- light palette, matching the LinkedIn carousel ------------------------
BG     = "#ffffff"
INK    = "#141b26"
MUTE   = "#5a6472"
FAINT  = "#98a1ae"
CARD   = "#ffffff"
INDIGO = "#4f46e5"    # the laptop half
AMBER  = "#c2740c"    # the radio between them
GREEN  = "#1f9d57"    # the aircraft half

EMOJI_FONT = "/System/Library/Fonts/Apple Color Emoji.ttc"

W, H = 30.0, 17.2
NW, NH = 6.6, 3.4

# ---- six steps, plus the radio that joins the two halves ------------------
# (emoji, title, plain-English line, colour, lane)
STEPS = [
    ("📷", "SEE",
     "A camera on the drone streams\nvideo to a laptop over WiFi", INDIGO, 0),
    ("🧠", "RECOGNISE",
     "Find every person, then check the face.\nIs this the one it was shown?",
     INDIGO, 0),
    ("🎯", "DECIDE",
     "Work out which way to turn and\nhow far to move to stay with them",
     INDIGO, 0),
    ("🛡️", "CHECK",
     "The drone refuses anything corrupted,\nrepeated, or unsafe", GREEN, 2),
    ("⚖️", "BALANCE",
     "Correct its own tilt a thousand\ntimes a second so it stays level",
     GREEN, 2),
    ("🌀", "SPIN",
     "Four motors at four different speeds.\nThat is the whole of how it moves",
     GREEN, 2),
]

Y = {0: 13.2, 2: 4.0}
XS = [5.2, 14.6, 24.0]
RADIO = (24.0, 8.6)


# ---- emoji -----------------------------------------------------------------

def emoji_rgba(ch: str, box: int = 220) -> np.ndarray | None:
    """One emoji as a tight RGBA array, or None if the font cannot draw it.

    Apple Color Emoji is an sbix font with fixed strike sizes; 160 is one of
    them, and asking for an arbitrary size silently degrades. Returning None
    rather than a blank lets the caller fall back visibly instead of shipping
    an invisible hole in the picture.
    """
    if not os.path.exists(EMOJI_FONT):
        return None
    try:
        font = ImageFont.truetype(EMOJI_FONT, 160)
        img = Image.new("RGBA", (box, box), (0, 0, 0, 0))
        ImageDraw.Draw(img).text((box // 2, box // 2), ch, font=font,
                                 embedded_color=True, anchor="mm")
        arr = np.asarray(img)
        if arr[..., 3].max() == 0:
            return None
        ys, xs = np.nonzero(arr[..., 3])
        return arr[ys.min():ys.max() + 1, xs.min():xs.max() + 1]
    except Exception:
        return None


def draw_emoji(ax, ch, x, y, size, fallback_colour):
    arr = emoji_rgba(ch)
    if arr is None:
        ax.add_patch(Circle((x, y), size / 2, facecolor=fallback_colour,
                            alpha=0.15, zorder=8))
        return
    h, w = arr.shape[:2]
    half_w = size / 2
    half_h = size / 2 * (h / w)
    ax.imshow(arr, extent=[x - half_w, x + half_w, y - half_h, y + half_h],
              aspect="auto", zorder=8, interpolation="lanczos")


# ---- pieces ----------------------------------------------------------------

def card(ax, x, y, ch, title, line, colour):
    ax.add_patch(FancyBboxPatch(
        (x - NW / 2, y - NH / 2), NW, NH,
        boxstyle="round,pad=0.02,rounding_size=0.28",
        linewidth=1.8, edgecolor=colour, facecolor=CARD, zorder=4))
    ax.add_patch(FancyBboxPatch(
        (x - NW / 2, y + NH / 2 - 0.13), NW, 0.13,
        boxstyle="round,pad=0,rounding_size=0.05",
        linewidth=0, facecolor=colour, zorder=5))
    draw_emoji(ax, ch, x, y + 0.82, 1.30, colour)
    ax.text(x, y - 0.38, title, color=INK, fontsize=23, fontweight="bold",
            ha="center", va="center", zorder=7)
    ax.text(x, y - 1.16, line, color=MUTE, fontsize=12, ha="center",
            va="center", zorder=7, linespacing=1.6)


def radio_card(ax, x, y):
    w, h = 5.8, 2.3
    ax.add_patch(FancyBboxPatch(
        (x - w / 2, y - h / 2), w, h,
        boxstyle="round,pad=0.02,rounding_size=0.24",
        linewidth=1.8, edgecolor=AMBER, facecolor="#fffaf1", zorder=4))
    draw_emoji(ax, "📡", x, y + 0.46, 1.05, AMBER)
    ax.text(x, y - 0.42, "30 bytes, 30 times a second", color=AMBER,
            fontsize=12.5, fontweight="bold", ha="center", va="center",
            zorder=7)


def arrow(ax, x0, y0, x1, y1, colour):
    ax.annotate("", xy=(x1, y1), xytext=(x0, y0), zorder=3,
                arrowprops=dict(arrowstyle="-|>,head_width=0.22,head_length=0.45",
                                color=colour, linewidth=2.4, alpha=0.85,
                                shrinkA=0, shrinkB=0))


def hero(ax):
    x0, x1, y0, y1 = 2.0, 19.6, 6.8, 10.4
    cx = (x0 + x1) / 2
    ax.add_patch(FancyBboxPatch(
        (x0, y0), x1 - x0, y1 - y0,
        boxstyle="round,pad=0.02,rounding_size=0.30",
        linewidth=1.6, edgecolor="#dfe3ea", facecolor="#f7f8fb", zorder=3))
    ax.add_patch(FancyBboxPatch(
        (x0, y1 - 0.15), x1 - x0, 0.15,
        boxstyle="round,pad=0,rounding_size=0.05",
        linewidth=0, facecolor=INDIGO, zorder=4))

    ax.text(cx, 9.72, "PURSUIT-1", color=INK, fontsize=38, fontweight="bold",
            ha="center", va="center", zorder=5)
    ax.text(cx, 8.82,
            "A 92 mm circuit board that is its own airframe.\n"
            "It follows one person — the one it was shown.",
            color=MUTE, fontsize=13, ha="center", va="center", zorder=5,
            linespacing=1.6)

    stats = [("66 g", "all-up weight"), ("337", "tests, all green"),
             ("1 kHz", "balance loop"), ("25 fps", "vision")]
    for i, (big, lab) in enumerate(stats):
        sx = x0 + (x1 - x0) * (i + 0.5) / len(stats)
        ax.text(sx, 7.70, big, color=INDIGO, fontsize=21, fontweight="bold",
                ha="center", va="center", zorder=5)
        ax.text(sx, 7.20, lab.upper(), color=FAINT, fontsize=8.5, ha="center",
                va="center", zorder=5, family="monospace")
        if i:
            ax.plot([sx - (x1 - x0) / len(stats) / 2] * 2, [7.12, 7.85],
                    color="#e3e7ed", linewidth=1, zorder=4)


def main():
    fig, ax = plt.subplots(figsize=(W, H))
    fig.patch.set_facecolor(BG)
    ax.set_facecolor(BG)
    ax.set_xlim(0, W)
    ax.set_ylim(0, H)
    ax.axis("off")

    ax.text(1.2, H - 0.9, "PURSUIT-1  ·  HOW IT WORKS", color=INDIGO,
            fontsize=13, fontweight="bold", family="monospace")
    ax.text(1.2, H - 1.85, "A drone that follows one person, in six steps",
            color=INK, fontsize=27, fontweight="bold")

    # Right-aligned on purpose: left-aligned they land under the title and
    # over the footer rule respectively, which is exactly what happened first.
    ax.text(XS[-1] + NW / 2, Y[0] + NH / 2 + 0.5, "ON THE LAPTOP",
            color=INDIGO, fontsize=12.5, fontweight="bold", ha="right")
    ax.text(XS[-1] + NW / 2, Y[2] - NH / 2 - 0.62, "ON THE DRONE",
            color=GREEN, fontsize=12.5, fontweight="bold", ha="right")

    hero(ax)

    top = [s for s in STEPS if s[4] == 0]
    bot = [s for s in STEPS if s[4] == 2]

    for i, (ch, t, line, colour, _) in enumerate(top):
        card(ax, XS[i], Y[0], ch, t, line, colour)
    # bottom flows right -> left, so the numbering keeps travelling forward
    for i, (ch, t, line, colour, _) in enumerate(bot):
        card(ax, XS[len(XS) - 1 - i], Y[2], ch, t, line, colour)

    radio_card(ax, *RADIO)

    # top lane, left to right
    for a, b in zip(XS, XS[1:]):
        arrow(ax, a + NW / 2 + 0.12, Y[0], b - NW / 2 - 0.12, Y[0], INDIGO)
    # down through the radio
    arrow(ax, RADIO[0], Y[0] - NH / 2 - 0.12, RADIO[0], RADIO[1] + 1.27, AMBER)
    arrow(ax, RADIO[0], RADIO[1] - 1.27, RADIO[0], Y[2] + NH / 2 + 0.12, GREEN)
    # bottom lane, right to left
    for a, b in zip(XS[::-1], XS[::-1][1:]):
        arrow(ax, a - NW / 2 - 0.12, Y[2], b + NW / 2 + 0.12, Y[2], GREEN)

    ax.plot([1.2, W - 1.2], [0.95, 0.95], color="#e6eaef", linewidth=1)
    ax.text(1.2, 0.52, "github.com/Pai-88/pursuit-drone", color=FAINT,
            fontsize=11, family="monospace")
    ax.text(W - 1.2, 0.52, "never flown · boards in fabrication",
            color=FAINT, fontsize=11, ha="right")

    out = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       "pursuit_process_map.png")
    fig.savefig(out, dpi=150, facecolor=BG, bbox_inches="tight",
                pad_inches=0.3)
    print("wrote", out)


if __name__ == "__main__":
    main()
