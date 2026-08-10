#!/usr/bin/env python3
"""Every plot worth having from the training run.

    ml/.venv/bin/python ml/plot_all.py

Three figures, all from recorded runs in ml/runs/:

  ml_training.png    loss, accuracy, learning-rate schedule, score separation
  ml_evaluation.png  ROC, DET, threshold sweep, calibration
  ml_ablations.png   data scaling and embedding width

A DET curve is included because this is a verification problem, and biometrics
reports DET rather than ROC: both error rates on log axes, where the operating
region actually has resolution. On an ROC the entire interesting part of a good
verifier is squashed into the top-left pixel.
"""
from __future__ import annotations

import json
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

ROOT = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(ROOT)
RUNS = os.path.join(ROOT, "runs")
OUT = os.path.join(REPO, "docs", "media")

BLUE, ORANGE, RED, GREY = "#1b6ca8", "#d1701a", "#b02a2a", "#7a7a7a"
plt.rcParams.update({
    "font.family": "sans-serif",
    "font.sans-serif": ["Helvetica Neue", "Helvetica", "Arial", "DejaVu Sans"],
    "font.size": 9, "axes.titlesize": 10, "axes.labelsize": 9,
    "axes.spines.top": False, "axes.spines.right": False,
    "axes.grid": True, "grid.color": "#d5d5d5", "grid.linewidth": 0.5,
    "legend.frameon": False, "legend.fontsize": 8,
    "figure.facecolor": "white", "axes.facecolor": "white",
    "savefig.facecolor": "white",
})


def main() -> int:
    H = json.load(open(os.path.join(RUNS, "history.json")))
    E = np.load(os.path.join(RUNS, "eval.npz"))
    A = json.load(open(os.path.join(RUNS, "ablations.json")))
    h = H["history"]; ep = [r["epoch"] for r in h]
    d, m, t = H["dataset"], H["model"], H["training"]
    v = H["verification"]

    s = E["te_scores"]; lab = E["te_labels"].astype(int)

    # ------------------------------------------------------ fig 1: training --
    fig, ax = plt.subplots(2, 2, figsize=(9.5, 6.4))

    ax[0, 0].plot(ep, [r["train_loss"] for r in h], lw=1.6, color=BLUE, label="train")
    ax[0, 0].plot(ep, [r["val_loss"] for r in h], lw=1.6, color=ORANGE, label="validation")
    ax[0, 0].set_xlabel("epoch"); ax[0, 0].set_ylabel("cross-entropy")
    ax[0, 0].set_title("Loss", loc="left"); ax[0, 0].legend()

    ax[0, 1].plot(ep, [r["train_acc"] for r in h], lw=1.6, color=BLUE, label="train")
    ax[0, 1].plot(ep, [r["val_acc"] for r in h], lw=1.6, color=ORANGE, label="validation")
    ax[0, 1].set_xlabel("epoch")
    ax[0, 1].set_ylabel(f"{d['train_identities']}-way accuracy")
    ax[0, 1].set_title("Identity classification", loc="left"); ax[0, 1].legend()

    ax[1, 0].plot(ep, [r["lr"] for r in h], lw=1.6, color=GREY)
    ax[1, 0].set_xlabel("epoch"); ax[1, 0].set_ylabel("learning rate")
    ax[1, 0].set_title("Cosine schedule", loc="left")
    # The schedule explains the shape of the loss curve, which is why it is here.
    vl = [r["val_loss"] for r in h]
    drop = int(np.argmax(np.array(vl[:-1]) - np.array(vl[1:]))) + 1
    ax[1, 0].axvline(ep[drop], color=ORANGE, ls=":", lw=1.2)
    ax[1, 0].annotate(f"validation collapses\nat epoch {ep[drop]}",
                      xy=(ep[drop], h[drop]["lr"]),
                      xytext=(ep[drop] - 21, max(r["lr"] for r in h) * 0.42),
                      arrowprops=dict(arrowstyle="->", color=GREY, lw=0.8),
                      fontsize=8, color="#555")

    bins = np.linspace(-0.6, 1.0, 45)
    ax[1, 1].hist(s[lab == 0], bins=bins, color=RED, alpha=0.65, label="different")
    ax[1, 1].hist(s[lab == 1], bins=bins, color=BLUE, alpha=0.65, label="same")
    ax[1, 1].axvline(v["held_out"]["thr"], color="black", lw=1.2,
                     label=f"threshold {v['held_out']['thr']:.2f}")
    ax[1, 1].set_xlabel("cosine similarity"); ax[1, 1].set_ylabel("pairs")
    ax[1, 1].set_title("Score separation, held-out identities", loc="left")
    ax[1, 1].legend()

    fig.suptitle(f"Face embedding on {d['images']:,} LFW photographs · "
                 f"{m['params']:,} params · {t['epochs']} epochs · "
                 f"{t['seconds']:.0f}s on {t['device'].upper()} · seed {t['seed']}",
                 fontsize=10, y=0.995)
    fig.tight_layout(rect=[0, 0, 1, 0.97])
    p1 = os.path.join(OUT, "ml_training.png")
    fig.savefig(p1, dpi=200, bbox_inches="tight"); plt.close(fig)

    # ---------------------------------------------------- fig 2: evaluation --
    fig, ax = plt.subplots(2, 2, figsize=(9.5, 6.6))

    ax[0, 0].plot(E["te_fpr"], E["te_tpr"], lw=1.8, color=BLUE,
                  label=f"held-out   AUC {v['held_out']['auc']:.3f}")
    ax[0, 0].plot(E["tr_fpr"], E["tr_tpr"], lw=1.4, ls="--", color=RED,
                  label=f"seen in training   AUC {v['seen_in_training']['auc']:.3f}")
    ax[0, 0].plot([0, 1], [0, 1], lw=0.7, ls=":", color="#999")
    ax[0, 0].set_xlabel("false accept rate"); ax[0, 0].set_ylabel("true accept rate")
    ax[0, 0].set_title("ROC", loc="left"); ax[0, 0].legend(loc="lower right")

    far = np.maximum(E["te_fpr"], 1e-4); frr = np.maximum(1 - E["te_tpr"], 1e-4)
    ax[0, 1].plot(far, frr, lw=1.8, color=BLUE)
    ax[0, 1].set_xscale("log"); ax[0, 1].set_yscale("log")
    ax[0, 1].set_xlabel("false accept rate"); ax[0, 1].set_ylabel("false reject rate")
    ax[0, 1].set_title("DET — the biometric view, log axes", loc="left")
    i = int(np.argmin(np.abs(far - frr)))
    ax[0, 1].plot(far[i], frr[i], "o", color=RED, ms=6)
    ax[0, 1].annotate(f"EER ≈ {far[i]*100:.0f}%", xy=(far[i], frr[i]),
                      xytext=(far[i]*1.6, frr[i]*2.4), fontsize=8, color=RED)

    ths = np.linspace(-0.4, 1.0, 200)
    FAR = [(s[lab == 0] >= th).mean() for th in ths]
    FRR = [(s[lab == 1] < th).mean() for th in ths]
    ACC = [((s >= th) == lab).mean() for th in ths]
    ax[1, 0].plot(ths, FAR, lw=1.6, color=RED, label="false accept")
    ax[1, 0].plot(ths, FRR, lw=1.6, color=ORANGE, label="false reject")
    ax[1, 0].plot(ths, ACC, lw=1.6, color=BLUE, label="accuracy")
    ax[1, 0].axvline(v["held_out"]["thr"], color="black", lw=1.0, ls=":")
    ax[1, 0].set_xlabel("threshold"); ax[1, 0].set_ylabel("rate")
    ax[1, 0].set_title("Threshold sweep", loc="left"); ax[1, 0].legend()

    # Calibration: does a similarity of x mean an x chance of being the same?
    qs = np.quantile(s, np.linspace(0, 1, 11))
    cx, cy = [], []
    for a, b in zip(qs[:-1], qs[1:]):
        sel = (s >= a) & (s < b)
        if sel.sum() >= 10:
            cx.append(float(s[sel].mean())); cy.append(float(lab[sel].mean()))
    ax[1, 1].plot([min(cx), max(cx)], [0, 1], lw=0.8, ls=":", color="#999")
    ax[1, 1].plot(cx, cy, "o-", lw=1.6, ms=5, color=BLUE)
    ax[1, 1].set_xlabel("mean cosine similarity in bin")
    ax[1, 1].set_ylabel("fraction actually same")
    ax[1, 1].set_title("Calibration — similarity is not a probability", loc="left")

    fig.suptitle("Verification on identities never seen in training "
                 f"({d['test_identities']} people, {len(lab):,} pairs)",
                 fontsize=10, y=0.995)
    fig.tight_layout(rect=[0, 0, 1, 0.97])
    p2 = os.path.join(OUT, "ml_evaluation.png")
    fig.savefig(p2, dpi=200, bbox_inches="tight"); plt.close(fig)

    # ----------------------------------------------------- fig 3: ablations --
    fig, ax = plt.subplots(1, 2, figsize=(9.5, 4.0))

    ds = A["data_scaling"]
    ax[0].plot([r["images"] for r in ds], [r["acc"] * 100 for r in ds],
               "o-", lw=1.8, ms=6, color=BLUE)
    ax[0].set_xlabel("training images"); ax[0].set_ylabel("accuracy (%)")
    ax[0].set_title("Data scaling — still climbing at 100%", loc="left")
    gain = (ds[-1]["acc"] - ds[-2]["acc"]) * 100
    ax[0].annotate(f"the last 25% of the data was\nstill worth {gain:+.1f} points",
                   xy=(ds[-1]["images"], ds[-1]["acc"] * 100),
                   xytext=(0.34, 0.28), textcoords="axes fraction",
                   arrowprops=dict(arrowstyle="->", color=GREY, lw=0.8),
                   fontsize=8, color="#555")

    es = A["embedding_width"]
    ax2 = ax[1]
    ax2.plot([r["emb"] for r in es], [r["acc"] * 100 for r in es],
             "o-", lw=1.8, ms=6, color=BLUE)
    ax2.set_xscale("log", base=2)
    ax2.set_xticks([r["emb"] for r in es])
    ax2.set_xticklabels([str(r["emb"]) for r in es])
    ax2.set_xlabel("embedding dimension"); ax2.set_ylabel("accuracy (%)")
    lo = min(r["acc"] for r in es) * 100; hi = max(r["acc"] for r in es) * 100
    ax2.axhspan(lo, hi, color=GREY, alpha=0.13)
    ax2.set_title(f"Embedding width — flat within {hi-lo:.1f} points", loc="left")
    ax2.text(0.30, 0.13, "no trend — width is not the\nbottleneck at this data scale",
             transform=ax2.transAxes, fontsize=8, color="#555")
    ax2.set_ylim(lo - (hi - lo) * 0.55, hi + (hi - lo) * 0.22)

    fig.suptitle(f"Each point is a training run from scratch · "
                 f"{A['epochs']} epochs · same identity-disjoint split · "
                 f"seed {A['seed']}", fontsize=9.5, y=1.02)
    fig.tight_layout()
    p3 = os.path.join(OUT, "ml_ablations.png")
    fig.savefig(p3, dpi=200, bbox_inches="tight"); plt.close(fig)

    for p in (p1, p2, p3):
        print(f"wrote {os.path.relpath(p, REPO)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
