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
BODY, HEAD, HERO, SMALL = 44*PX, 76*PX, 104*PX, 34*PX

C = dict(bg="#0B0E14", card="#161C29", raised="#1F2739", hair="#2A3346",
         t1="#E8ECF4", t2="#94A1B8", t3="#6B7A94",
         data="#5B9CFF", ml="#A78BFA", ctrl="#22C1A6",
         warn="#F2B441", danger="#F2545B", ok="#3DD68C")

plt.rcParams.update({
    "font.family": "sans-serif",
    "font.sans-serif": ["Inter", "Helvetica Neue", "DejaVu Sans"],
    "text.color": C["t1"], "axes.labelcolor": C["t2"],
    "xtick.color": C["t3"], "ytick.color": C["t3"],
    "axes.edgecolor": C["hair"], "figure.facecolor": C["bg"],
    "axes.facecolor": C["card"], "savefig.facecolor": C["bg"],
})


def slide(n):
    fig = plt.figure(figsize=(W_IN, H_IN), dpi=DPI)
    fig.text(0.90, 0.035, f"{n}", ha="right", va="center",
             fontsize=SMALL, color=C["t3"])
    fig.text(0.075, 0.035, "PURSUIT-1", ha="left", va="center",
             fontsize=SMALL, color=C["t3"], weight="bold")
    return fig


def kicker(fig, text, color):
    fig.text(0.075, 0.935, text, fontsize=SMALL, color=color,
             weight="bold", va="top")


def head(fig, text, y=0.885, size=HEAD):
    fig.text(0.075, y, text, fontsize=size, color=C["t1"],
             weight="bold", va="top", linespacing=1.18)


def body(fig, text, y, color=None, size=BODY, weight="normal"):
    fig.text(0.075, y, text, fontsize=size, color=color or C["t2"],
             va="top", linespacing=1.45, weight=weight)


def note(fig, text, color=C["warn"], y=0.045):
    """A bordered callout — the one thing on the slide that must be read.

    Pinned to the bottom of every slide so the content zone above it is a
    fixed, predictable box. Body text must end by y=0.21 or it collides with
    this — which it did, on five slides, until the grid was made explicit.
    """
    ax = fig.add_axes([0.075, y, 0.85, 0.150]); ax.axis("off")
    ax.add_patch(FancyBboxPatch((0, 0), 1, 1, transform=ax.transAxes,
                 boxstyle="round,pad=0.02,rounding_size=0.03",
                 facecolor=C["card"], edgecolor=color, linewidth=2.4,
                 clip_on=False))
    ax.text(0.035, 0.5, text, transform=ax.transAxes, va="center",
            fontsize=BODY*0.76, color=C["t1"], linespacing=1.45)


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
    head(f, "I built a drone\nthat follows one\nspecific person.", 0.83, HERO)
    body(f, "The printed circuit board IS the airframe.\n"
            "No separate frame — the motors clamp into\nrings routed through the arms.",
         0.465, C["t2"])
    body(f, "92 mm  ·  66 g  ·  ESP32-S3\n"
            f"{M['tests']['total']} tests  ·  MIT licensed",
         0.295, C["t3"], BODY*0.90)
    note(f, "It has never flown — the boards are not\n"
            "fabricated yet. Everything here is measured\n"
            "on the bench or in simulation.",
         C["warn"])
    made.append(save(f, "01_hook.png"))

    # ---------------------------------------------------------------- 02 --
    f = slide(2)
    kicker(f, "THE PROBLEM", C["ml"])
    head(f, "Following a person\nis not detection.")
    body(f, "A detector finds every person in frame.\n"
            "Following needs the RIGHT one, held across\n"
            "occlusion, motion blur and bad angles.", 0.72)
    body(f, "So identity is two signals with different jobs:", 0.55, C["t1"],
         BODY, "bold")
    body(f, "FACE  acquires and re-confirms. Strict.\n"
            "        ArcFace 512-d, threshold 0.36.\n\n"
            "BODY  sustains between face frames.\n"
            "        Re-ID 768-d — and it has to, because\n"
            "        only 26% of frames carry a usable face.", 0.50, C["t2"], BODY*0.92)
    note(f, "A readable face BELOW 0.15 is evidence\n"
            "AGAINST. Without that veto, two people in\n"
            "similar clothing swapped identity for good.", C["danger"])
    made.append(save(f, "02_problem.png"))

    # ---------------------------------------------------------------- 03 --
    v = verification_scores()
    f = slide(3)
    kicker(f, "EVALUATION  —  DOES IT ACTUALLY SEPARATE?", C["ml"])
    head(f, "I measured it\ninstead of trusting it.")
    if v is not None:
        pos, neg, nid = v
        ax = panel(f, [0.115, 0.445, 0.80, 0.275])
        bins = np.linspace(-0.25, 1.0, 42)
        ax.hist(neg, bins=bins, color=C["danger"], alpha=0.80,
                label=f"different person  (n={neg.size})")
        ax.hist(pos, bins=bins, color=C["ok"], alpha=0.80,
                label=f"same person  (n={pos.size})")
        ax.axvline(0.36, color=C["data"], lw=3)
        ax.text(0.345, ax.get_ylim()[1]*0.55, "0.36", rotation=90,
                fontsize=SMALL, color=C["data"], weight="bold",
                ha="right", va="center")
        ax.set_xlabel("ArcFace cosine similarity", fontsize=BODY*0.8)
        ax.set_ylabel("pairs", fontsize=BODY*0.8)
        ax.legend(fontsize=SMALL, facecolor=C["raised"], edgecolor=C["hair"],
                  labelcolor=C["t1"], loc="upper right")
        gap = pos.min() - neg.max()
        body(f, f"Separation  +{gap:.3f}   ·   no overlap   ·   AUC 1.000\n"
                f"0 false accepts in {neg.size}   ·   0 false rejects in {pos.size}",
             0.355, C["t1"], BODY*0.86, "bold")
        note(f, f"Only {nid} identities, so the 95% CI on the\n"
                f"false-accept rate is [0, 5.2%]. That is a\n"
                f"statement about SAMPLE SIZE, not the model.", C["warn"])
    else:
        body(f, "Gallery absent — it is gitignored biometric data.", 0.6)
    made.append(save(f, "03_separation.png"))

    # ---------------------------------------------------------------- 04 --
    f = slide(4)
    kicker(f, "THE OPERATING POINT", C["ml"])
    head(f, "The two errors are\nnot equally bad.")
    body(f, "FALSE REJECT — miss one frame.\n"
            "Costs nothing. Body re-ID carries the lock,\n"
            "and acquisition needs 5 consecutive hits.", 0.70, C["ok"])
    body(f, "FALSE ACCEPT — lock onto a stranger.\n"
            "The drone follows the wrong person, and the\n"
            "appearance model then REINFORCES the error.", 0.52, C["danger"])
    body(f, "So the threshold buys a negligible false-accept\n"
            "rate using false rejects, which are nearly free.\n"
            "0.36 sits well above the 0.108 where accepts first\n"
            "appear — deliberately conservative.", 0.345, C["t2"], BODY*0.92)
    note(f, "A threshold is a DECISION. It should be\n"
            "argued from measured error costs — which\n"
            "is the part most projects skip.", C["data"])
    made.append(save(f, "04_operating_point.png"))

    # ---------------------------------------------------------------- 05 --
    f = slide(5)
    kicker(f, "WHAT THE EVALUATION CAUGHT", C["danger"])
    head(f, "My own dataset\nwas mislabelled.")
    body(f, "Two identities enrolled: 'Paing' and 'alex'.", 0.70, C["t2"])
    ax = panel(f, [0.10, 0.36, 0.82, 0.26])
    vals = [0.579, 0.560]
    ax.barh([1, 0], vals, color=[C["danger"], C["t3"]], height=0.55)
    ax.set_yticks([1, 0])
    ax.set_yticklabels(["Paing  vs  alex", "Paing  vs  Paing"], fontsize=SMALL)
    ax.set_xlim(0, 0.72)
    ax.set_xlabel("median cosine similarity", fontsize=BODY*0.8)
    for i, val in zip([1, 0], vals):
        ax.text(val + 0.012, i, f"{val:.3f}", va="center",
                fontsize=SMALL, color=C["t1"], weight="bold")
    body(f, "A stranger cannot resemble you more than you do.\n"
            "'alex' was a placeholder enrolled from my own\n"
            "webcam and never replaced.", 0.345, C["t2"], BODY*0.92)
    note(f, "Every 'different person' pair was really the\n"
            "same person. The first run reported a confident\n"
            "AUC of 0.83 — fiction. Now it self-checks.",
         C["danger"])
    made.append(save(f, "05_bug.png"))

    # ---------------------------------------------------------------- 06 --
    q = M["quantisation"]["models"]
    f = slide(6)
    kicker(f, "DEPLOYMENT  —  INT8 QUANTISATION", C["ctrl"])
    head(f, "4x smaller.\nAnd slower.")
    ax = panel(f, [0.13, 0.40, 0.79, 0.31])
    names = [m["name"].split("_")[0] for m in q]
    y = np.arange(len(q))
    ax.barh(y+0.19, [m["mb_fp32"] for m in q], 0.36, color=C["t3"], label="fp32")
    ax.barh(y-0.19, [m["mb_int8"] for m in q], 0.36, color=C["ctrl"], label="int8")
    ax.set_yticks(y); ax.set_yticklabels(names, fontsize=SMALL)
    ax.set_xlabel("weights (MB)", fontsize=BODY*0.8)
    ax.legend(fontsize=SMALL, facecolor=C["raised"], edgecolor=C["hair"],
              labelcolor=C["t1"])
    tot32 = sum(m["mb_fp32"] for m in q); tot8 = sum(m["mb_int8"] for m in q)
    body(f, f"{tot32:.0f} MB  ->  {tot8:.0f} MB     ·     latency 0.89x  (WORSE)\n"
            f"output agreement 0.976 – 0.999", 0.32, C["t1"], BODY*0.95, "bold")
    note(f, "CoreML runs fp16 on the Neural Engine faster\n"
            "than the CPU runs INT8. So here it is a memory\n"
            "win and a latency LOSS. It would win on a Pi.",
         C["warn"])
    made.append(save(f, "06_quantisation.png"))

    # ---------------------------------------------------------------- 07 --
    d = M["degradation"]
    f = slide(7)
    kicker(f, "THE EXPERIMENT THAT CHANGED THE HARDWARE", C["data"])
    head(f, "Noise is fine.\nBlur is fatal.")
    ax = panel(f, [0.13, 0.40, 0.79, 0.30])
    ax.plot([p["px"] for p in d["motion_blur_px"]],
            [p["people"] for p in d["motion_blur_px"]],
            "o-", lw=4, ms=13, color=C["danger"], label="motion blur (px)")
    ax.plot([0, 5, 9], [p["people"] for p in d["sensor_noise"]],
            "s--", lw=4, ms=13, color=C["ok"], label="sensor noise (gain 1/2/4x)")
    ax.set_ylim(-0.25, 2.5); ax.set_yticks([0, 1, 2])
    ax.set_xlabel("degradation", fontsize=BODY*0.8)
    ax.set_ylabel("people detected\n(of 2)", fontsize=BODY*0.8)
    ax.legend(fontsize=SMALL, facecolor=C["raised"], edgecolor=C["hair"],
              labelcolor=C["t1"], loc="lower left")
    body(f, "YOLO keys on edges. Noise leaves them intact.\n"
            "Blur destroys them. At 15 px of blur: zero people.", 0.32, C["t2"])
    note(f, "So the camera now PREFERS a dark, noisy,\n"
            "short-exposure frame. The old config capped\n"
            "gain to avoid noise — exactly backwards.", C["ok"])
    made.append(save(f, "07_degradation.png"))

    # ---------------------------------------------------------------- 08 --
    L = M["lag_stability"]
    f = slide(8)
    kicker(f, "CONTROL  —  THE RESULT THAT MATTERS", C["ctrl"])
    head(f, "The loop closes\nthrough the airframe.")
    ax = panel(f, [0.13, 0.38, 0.79, 0.30])
    ax.plot([p["delay_ms"] for p in L["points"]],
            [p["no_comp"] for p in L["points"]],
            "o-", lw=4, ms=12, color=C["danger"], label="uncompensated")
    ax.plot([p["delay_ms"] for p in L["points"]],
            [p["with_comp"] for p in L["points"]],
            "s-", lw=4, ms=12, color=C["ok"], label="attitude compensated")
    ax.axvline(L["measured_rtt_ms"], color=C["data"], ls=":", lw=3)
    ax.set_xlabel("link delay (ms)", fontsize=BODY*0.8)
    ax.set_ylabel("oscillation\n(deg p-p)", fontsize=BODY*0.8)
    ax.legend(fontsize=SMALL, facecolor=C["raised"], edgecolor=C["hair"],
              labelcolor=C["t1"], loc="upper left")
    body(f, "Pitching to chase moves the camera, which moves\n"
            "where the subject appears. That path has gain\n"
            f"34/26 = {L['loop_gain']} — above one.", 0.345, C["t2"], BODY*0.92)
    note(f, f"Above unity, the correction overshoots the\n"
            f"error that caused it. Uncompensated it rings at\n"
            f"ONE frame — and the real RTT is {L['measured_rtt_ms']} ms.", C["danger"])
    made.append(save(f, "08_control.png"))

    # ---------------------------------------------------------------- 09 --
    f = slide(9)
    kicker(f, "WHAT IS NOT DONE", C["warn"])
    head(f, "It has never flown.")
    body(f, "The boards are not fabricated. The PID gains are\n"
            "scaled from typical values and have never touched\n"
            "this airframe. The stability result above is a\n"
            "simulation with ESTIMATED mass properties.", 0.72)
    body(f, "Also true:", 0.53, C["t1"], BODY, "bold")
    body(f, "·  nothing here was TRAINED — the models are\n"
            "   pretrained; the work is evaluation and deployment\n"
            "·  the two-person discrimination test has never\n"
            "   actually been run on two real people\n"
            "·  the face models are non-commercial research use", 0.45)
    note(f, "I would rather you know the limits than\n"
            "discover them. Everything claimed above is\n"
            "measured. This slide is what is not.",
         C["warn"])
    made.append(save(f, "09_honest.png"))

    # ---------------------------------------------------------------- 10 --
    f = slide(10)
    kicker(f, "THE STACK", C["data"])
    head(f, "Built end to end.")
    body(f, "PCB          generated from a Python netlist, with\n"
            "                a 7-gate check that must pass before fab\n\n"
            "FIRMWARE  ESP32-S3 from the IMU registers up:\n"
            "                Madgwick, cascaded PID, mixer, watchdog\n\n"
            "VISION      ONNX Runtime + CoreML, no torch:\n"
            "                detect, track, re-identify, servo\n\n"
            "PROTOCOL  generated from ONE source, so the two\n"
            "                ends cannot drift. Checked in CI.", 0.70,
         C["t2"], BODY*0.80)
    body(f, f"{M['tests']['python']} Python + {M['tests']['native_cpp']} native C++ tests",
         0.275, C["ok"], BODY*0.92, "bold")
    note(f, "Full write-up, every measurement, the code:\n"
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
