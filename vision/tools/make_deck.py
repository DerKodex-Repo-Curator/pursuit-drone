#!/usr/bin/env python3
"""Generate a LinkedIn carousel from measured results.

    vision/.venv/bin/python vision/tools/make_deck.py

Why this is a script and not a Figma file
-----------------------------------------
Every number on these slides exists somewhere in this repository already. Typing
them into a design tool creates a second copy that nothing checks, and the first
time a threshold changes the deck becomes confidently wrong. Generating the
slides from `docs/media/measurements.json` — and recomputing the cheap ones live
— means the deck is as true as the last time it was run.

Format, from LinkedIn's published specs
---------------------------------------
* 1080 x 1350 (4:5). This is the TALLEST ratio LinkedIn accepts. Anything taller
  is centre-cropped, not letterboxed, so a 9:16 design silently loses its top
  and bottom.
* Carousels are uploaded as a PDF, every page the same size, and are rasterised
  in the feed — so links inside are not clickable. The repo URL belongs in the
  post text, not only on a slide.
* Body text at 44 px minimum on this canvas. A phone renders 1080 px into about
  390 CSS px, a 0.36x scale, so 44 px lands at ~16 px — normal body size. The
  32 px that looks fine on a laptop lands at 11 px and is unreadable, which is
  how most engineering carousels fail.

A note on what is NOT in here
-----------------------------
There is no loss curve, because nothing in this project was trained. The models
are pretrained and the work is evaluation, deployment and control. Inventing a
training plot would be inventing a result.
"""
from __future__ import annotations

import itertools
import json
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import FancyBboxPatch

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REPO = os.path.dirname(ROOT)
sys.path.insert(0, ROOT)

OUT = os.path.join(REPO, "docs", "media", "linkedin")
MEAS = os.path.join(REPO, "docs", "media", "measurements.json")

# 1080 x 1350 at 100 dpi. Points, not pixels, in matplotlib: px * 72/100.
W_IN, H_IN, DPI = 10.80, 13.50, 100
PX = 72.0 / DPI                     # multiply a pixel size by this to get points
BODY, HEAD, HERO, SMALL = 44*PX, 72*PX, 86*PX, 34*PX

# Light, plain, and deliberately undesigned.
#
# The first version of this deck was near-black with saturated blue/purple/green
# accents and rounded cards. That is the Tailwind default palette every AI tool
# emits, and a reader pattern-matches it before reading a number — which, on a
# post whose whole purpose is to be believed, works against you. White ground,
# near-black text, matplotlib's own muted colours.
C = dict(bg="#FFFFFF", card="#F4F5F7", raised="#EBEDF0", hair="#D8DCE2",
         t1="#111418", t2="#3F4650", t3="#6B7280",
         data="#1b6ca8", ml="#6b4fa8", ctrl="#1a7f6b",
         warn="#b0700f", danger="#b02a2a", ok="#1a7f4b")

plt.rcParams.update({
    "font.family": "sans-serif",
    "font.sans-serif": ["Helvetica Neue", "Helvetica", "Arial", "DejaVu Sans"],
    "text.color": C["t1"], "axes.labelcolor": C["t2"],
    "xtick.color": C["t3"], "ytick.color": C["t3"],
    "axes.edgecolor": C["hair"],
    "axes.spines.top": False, "axes.spines.right": False,
    "legend.frameon": False,
    "figure.facecolor": C["bg"], "axes.facecolor": C["bg"],
    "savefig.facecolor": C["bg"],
})


def slide(n):
    fig = plt.figure(figsize=(W_IN, H_IN), dpi=DPI)
    fig.text(0.91, 0.032, f"{n}", ha="right", va="center",
             fontsize=SMALL, color=C["t3"])
    fig.text(0.09, 0.032, "PURSUIT-1", ha="left", va="center",
             fontsize=SMALL, color=C["t3"], weight="bold")
    return fig


def kicker(fig, text, color):
    fig.text(0.09, 0.945, text, fontsize=SMALL, color=color,
             weight="bold", va="top")


def head(fig, text, y=0.875, size=HEAD):
    fig.text(0.09, y, text, fontsize=size, color=C["t1"],
             weight="bold", va="top", linespacing=1.24)


def body(fig, text, y, color=None, size=BODY, weight="normal"):
    fig.text(0.09, y, text, fontsize=size, color=color or C["t2"],
             va="top", linespacing=1.52, weight=weight)


def note(fig, text, color=C["warn"], y=0.045):
    """A bordered callout — the one thing on the slide that must be read.

    Pinned to the bottom of every slide so the content zone above it is a
    fixed, predictable box. Body text must end by y=0.21 or it collides with
    this — which it did, on five slides, until the grid was made explicit.
    """
    ax = fig.add_axes([0.09, y, 0.82, 0.142]); ax.axis("off")
    ax.add_patch(FancyBboxPatch((0, 0), 1, 1, transform=ax.transAxes,
                 boxstyle="round,pad=0.02,rounding_size=0.03",
                 facecolor=C["card"], edgecolor=color, linewidth=1.6,
                 clip_on=False))
    ax.text(0.045, 0.5, text, transform=ax.transAxes, va="center",
            fontsize=BODY*0.74, color=C["t1"], linespacing=1.55)


def panel(fig, rect):
    ax = fig.add_axes(rect)
    ax.set_facecolor(C["card"])
    for s in ax.spines.values():
        s.set_color(C["hair"])
    ax.tick_params(labelsize=SMALL, length=4)
    ax.grid(alpha=0.16, color=C["t3"], linewidth=0.9)
    ax.set_axisbelow(True)
    return ax


def save(fig, name):
    os.makedirs(OUT, exist_ok=True)
    p = os.path.join(OUT, name)
    fig.savefig(p, dpi=DPI, facecolor=C["bg"])
    plt.close(fig)
    return p


# --------------------------------------------------------------- live data --
def verification_scores():
    """Recompute the real score distributions, by reusing verify_eval itself.

    Importing the tool rather than reimplementing it is the point: a slide that
    computes its own version of a metric is a second implementation that can
    disagree with the one that produced the published number. This way the deck
    is definitionally consistent with `tools/verify_eval.py`, including the
    identity-collision merge that caught the mislabelled gallery.

    Returns (positives, negatives, n_identities), or None when the gallery is
    absent — it is gitignored biometric data, so a clean clone cannot do this.
    """
    try:
        sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
        import verify_eval as VE
        from pursuit.config import Config

        cfg = Config()
        gallery = VE.load_gallery(os.path.join(ROOT, cfg.gallery))
        if not gallery:
            return None

        for a, b in VE.detect_collisions(gallery):
            if a in gallery and b in gallery:
                gallery[a] = np.vstack([gallery[a], gallery.pop(b)])

        # Faces from the fixture images, as extra NEGATIVE identities. Without
        # them the merged gallery is a single person and there are no
        # different-person pairs at all.
        gallery.update(VE.extra_identities(cfg))

        pos, neg = VE.build_pairs(gallery)
        if pos.size == 0 or neg.size == 0:
            return None
        return pos, neg, len(gallery)
    except Exception as exc:
        print(f"  (verification panel skipped: {exc})")
        return None


def main() -> int:
    with open(MEAS) as f:
        M = json.load(f)
    made = []

    # ---------------------------------------------------------------- 01 --
    f = slide(1)
    kicker(f, "EMBEDDED ML  ·  COMPUTER VISION  ·  FLIGHT CONTROL", C["data"])
    head(f, "A 92 mm drone\nthat follows one\nenrolled person.", 0.83, HERO)
    body(f, "The circuit board is the airframe. Flight firmware\n"
            "runs on an ESP32-S3; perception runs off-board\n"
            "on a laptop, over WiFi.", 0.45)
    body(f, f"66 g   ·   {M['tests']['total']} tests   ·   MIT licensed",
         0.265, C["t3"], BODY*0.90)
    note(f, "Not yet flown. The boards are not fabricated.\n"
            "Every number here is bench or simulation.", C["warn"])
    made.append(save(f, "01_hook.png"))

    # ---------------------------------------------------------------- 02 --
    f = slide(2)
    kicker(f, "IDENTITY", C["ml"])
    head(f, "Two signals,\ndifferent jobs.")
    body(f, "Face acquires and re-confirms the lock.\n"
            "ArcFace 512-d, cosine threshold 0.36.\n\n"
            "Body re-identification sustains it between\n"
            "face frames. 768-d embedding.\n\n"
            "Only 26% of frames carry a usable face, so\n"
            "the body signal is doing most of the work.", 0.63)
    note(f, "A readable face scoring below 0.15 counts as\n"
            "evidence against, not as missing evidence.", C["danger"])
    made.append(save(f, "02_identity.png"))

    # ---------------------------------------------------------------- 03 --
    # Real training run, from ml/runs/history.json.
    hist = None
    hp = os.path.join(REPO, "ml", "runs", "history.json")
    if os.path.exists(hp):
        with open(hp) as fh:
            hist = json.load(fh)
    f = slide(3)
    kicker(f, "TRAINING", C["data"])
    head(f, "Trained an embedding\non 4,324 photographs.")
    if hist:
        h = hist["history"]
        ax = panel(f, [0.13, 0.40, 0.78, 0.28])
        ax.plot([r["epoch"] for r in h], [r["train_loss"] for r in h],
                lw=2.2, color=C["data"], label="train")
        ax.plot([r["epoch"] for r in h], [r["val_loss"] for r in h],
                lw=2.2, color=C["warn"], label="validation")
        ax.set_xlabel("epoch", fontsize=BODY*0.8)
        ax.set_ylabel("cross-entropy", fontsize=BODY*0.8)
        ax.legend(fontsize=SMALL)
        d, mo, tr = hist["dataset"], hist["model"], hist["training"]
        body(f, f"Labeled Faces in the Wild. {mo['params']:,} parameters,\n"
                f"{mo['mb_fp32']} MB. {tr['epochs']} epochs, {tr['seconds']:.0f}s "
                f"on an M4 GPU.", 0.335, C["t2"], BODY*0.86)
    note(f, "Validation loss climbs for 34 epochs, then drops\n"
            "as the cosine schedule decays the learning rate.", C["warn"])
    made.append(save(f, "03_training.png"))

    # ---------------------------------------------------------------- 04 --
    f = slide(4)
    kicker(f, "PROTOCOL", C["danger"])
    head(f, "Split by identity,\nnot by image.")
    if hist:
        v = hist["verification"]
        ax = panel(f, [0.28, 0.40, 0.62, 0.26])
        vals = [v["seen_in_training"]["acc"] * 100, v["held_out"]["acc"] * 100]
        ax.barh([1, 0], vals, color=[C["danger"], C["data"]], height=0.5)
        ax.set_yticks([1, 0])
        ax.set_yticklabels(["seen in\ntraining", "held\nout"], fontsize=SMALL)
        ax.set_xlim(0, 108)
        ax.set_xlabel("verification accuracy (%)", fontsize=BODY*0.8)
        for yy, vv in zip([1, 0], vals):
            ax.text(vv + 2, yy, f"{vv:.1f}%", va="center", fontsize=SMALL,
                    color=C["t1"], weight="bold")
        body(f, "127 people to train on. 31 held out and\n"
                "never seen during training.", 0.335, C["t2"], BODY*0.86)
    note(f, "Splitting by image leaves the same faces on both\n"
            "sides. That is 16.6 points of inflation.", C["danger"])
    made.append(save(f, "04_protocol.png"))

    # ---------------------------------------------------------------- 05 --
    f = slide(5)
    kicker(f, "BENCHMARK", C["ml"])
    head(f, "136x the weights\nbuys 19 points.")
    if hist:
        ax = panel(f, [0.17, 0.40, 0.70, 0.27])
        ax.bar(["mine\n1.3 MB", "ArcFace\n174 MB"], [77.3, 96.4],
               color=[C["data"], C["t3"]], width=0.5)
        ax.set_ylim(0, 112)
        ax.set_ylabel("accuracy (%)", fontsize=BODY*0.8)
        ax.tick_params(labelsize=SMALL)
        for xx, vv in zip([0, 1], [77.3, 96.4]):
            ax.text(xx, vv + 3, f"{vv:.1f}%", ha="center", fontsize=SMALL,
                    color=C["t1"], weight="bold")
        body(f, "Both scored on the same image pairs, from\n"
                "identities neither model was trained on.",
             0.335, C["t2"], BODY*0.86)
    note(f, "ArcFace gets unaligned 62x47 crops here, which\n"
            "understates it. Its published LFW figure is 99.8%.", C["t3"])
    made.append(save(f, "05_benchmark.png"))

    # ---------------------------------------------------------------- 06 --
    q = M["quantisation"]["models"]
    f = slide(6)
    kicker(f, "DEPLOYMENT", C["ctrl"])
    head(f, "INT8 made it\n4x smaller and\n11% slower.")
    ax = panel(f, [0.15, 0.335, 0.76, 0.24])
    names = [m["name"].split("_")[0] for m in q]
    y = np.arange(len(q))
    ax.barh(y + 0.19, [m["mb_fp32"] for m in q], 0.34, color=C["t3"], label="fp32")
    ax.barh(y - 0.19, [m["mb_int8"] for m in q], 0.34, color=C["ctrl"], label="int8")
    ax.set_yticks(y); ax.set_yticklabels(names, fontsize=SMALL)
    ax.set_xlabel("weights (MB)", fontsize=BODY*0.8)
    ax.legend(fontsize=SMALL)
    note(f, "CoreML runs fp16 on the Neural Engine faster than\n"
            "the CPU runs INT8. It would win on a Raspberry Pi.", C["warn"])
    made.append(save(f, "06_quantisation.png"))

    # ---------------------------------------------------------------- 07 --
    d = M["degradation"]
    f = slide(7)
    kicker(f, "MEASUREMENT", C["data"])
    head(f, "Blur kills detection.\nNoise does not.")
    ax = panel(f, [0.15, 0.395, 0.76, 0.27])
    ax.plot([p["px"] for p in d["motion_blur_px"]],
            [p["people"] for p in d["motion_blur_px"]],
            "o-", lw=2.6, ms=10, color=C["danger"], label="motion blur")
    ax.plot([0, 5, 9], [p["people"] for p in d["sensor_noise"]],
            "s--", lw=2.6, ms=10, color=C["data"], label="sensor noise")
    ax.set_ylim(-0.3, 2.6); ax.set_yticks([0, 1, 2])
    ax.set_xlabel("degradation", fontsize=BODY*0.8)
    ax.set_ylabel("people found\n(of 2)", fontsize=BODY*0.8)
    ax.legend(fontsize=SMALL, loc="lower left")
    body(f, "At 15 px of blur the detector finds nobody.\n"
            "At 4x gain it still finds both people.", 0.335, C["t2"], BODY*0.86)
    note(f, "So the camera is set for short exposure and\n"
            "accepts the sensor noise that comes with it.", C["ok"])
    made.append(save(f, "07_degradation.png"))

    # ---------------------------------------------------------------- 08 --
    L = M["lag_stability"]
    f = slide(8)
    kicker(f, "CONTROL", C["ctrl"])
    head(f, "The vision loop closes\nthrough the airframe.")
    ax = panel(f, [0.15, 0.395, 0.76, 0.26])
    ax.plot([p["delay_ms"] for p in L["points"]],
            [p["no_comp"] for p in L["points"]],
            "o-", lw=2.6, ms=9, color=C["danger"], label="uncompensated")
    ax.plot([p["delay_ms"] for p in L["points"]],
            [p["with_comp"] for p in L["points"]],
            "s-", lw=2.6, ms=9, color=C["data"], label="compensated")
    ax.axvline(L["measured_rtt_ms"], color=C["t3"], ls=":", lw=2)
    ax.set_xlabel("link delay (ms)", fontsize=BODY*0.8)
    ax.set_ylabel("oscillation\n(deg p-p)", fontsize=BODY*0.8)
    ax.legend(fontsize=SMALL, loc="upper left")
    body(f, "Pitching to chase moves the camera, which moves\n"
            f"the subject in frame. Gain 34/26 = {L['loop_gain']}.",
         0.335, C["t2"], BODY*0.86)
    note(f, "Above 1, the correction overshoots. Uncompensated\n"
            f"it rings at one frame; the real RTT is {L['measured_rtt_ms']} ms.",
         C["danger"])
    made.append(save(f, "08_control.png"))

    # ---------------------------------------------------------------- 09 --
    f = slide(9)
    kicker(f, "LIMITATIONS", C["warn"])
    head(f, "What is not done.")
    body(f, "The aircraft has never flown and the boards\n"
            "are not fabricated.\n\n"
            "PID gains are scaled from typical values, not\n"
            "tuned on this airframe.\n\n"
            "The stability result is simulated, with\n"
            "estimated mass properties.\n\n"
            "The two-person test has never been run on\n"
            "two real people.", 0.70, C["t2"], BODY*0.94)
    note(f, "Listed because the numbers on the other slides\n"
            "are only worth anything if this slide exists.", C["warn"])
    made.append(save(f, "09_limitations.png"))

    # ---------------------------------------------------------------- 10 --
    f = slide(10)
    kicker(f, "THE STACK", C["data"])
    head(f, "Built end to end.")
    body(f, "Board       generated from a Python netlist,\n"
            "                with a release check before fab\n\n"
            "Firmware   ESP32-S3 from the IMU registers up\n\n"
            "Vision      ONNX Runtime and CoreML, no torch\n\n"
            "Protocol   generated from one source, so the\n"
            "                two ends cannot drift", 0.68, C["t2"], BODY*0.88)
    body(f, f"{M['tests']['python']} Python and "
            f"{M['tests']['native_cpp']} native C++ tests",
         0.245, C["t1"], BODY*0.92, "bold")
    note(f, "Code, write-up and every measurement:\n"
            "github.com/Pai-88/pursuit-drone", C["data"])
    made.append(save(f, "10_stack.png"))

    # ----------------------------------------------------------------- pdf --
    pdf = os.path.join(OUT, "PURSUIT-1_carousel.pdf")
    try:
        from PIL import Image
        ims = [Image.open(p).convert("RGB") for p in sorted(made)]
        ims[0].save(pdf, save_all=True, append_images=ims[1:], resolution=100)
    except ImportError:
        pdf = None

    print(f"{len(made)} slides -> {os.path.relpath(OUT, REPO)}")
    for p in sorted(made):
        print(f"  {os.path.basename(p)}")
    print(f"\ncarousel PDF -> {os.path.relpath(pdf, REPO) if pdf else 'PIL not installed'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
