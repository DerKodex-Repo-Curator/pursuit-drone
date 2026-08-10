#!/usr/bin/env python3
"""Every plot worth having from the training run.

    ml/.venv/bin/python ml/plot_all.py

Twelve plots, four to a figure, all from recorded runs in ml/runs/:

  ml_training.png    loss · identity accuracy · LR schedule · generalisation gap
  ml_evaluation.png  score separation · ROC · DET · calibration
  ml_decisions.png   data scaling · embedding width · size vs accuracy · embedding space

The split is by QUESTION, not by convenience. The first figure asks whether the
optimiser worked. The second asks how good the resulting verifier is. The third
asks what to do next, which is the only one that changes an action.

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


def leg(ax, ncol=2):
    """Put the legend on the title row, outside the data area.

    Legends 'inside, in the empty corner' is a bet that the corner stays empty.
    It does not: the loss curve, the histogram and the threshold sweep each
    ended up with a legend sitting on the lines it was labelling. Titles here
    are left-aligned, so the right end of the title row is always free.
    """
    ax.legend(loc="lower right", bbox_to_anchor=(1.0, 1.0), ncol=ncol,
              frameon=False, fontsize=8, borderaxespad=0.2,
              columnspacing=1.4, handlelength=1.6)


def main() -> int:
    H = json.load(open(os.path.join(RUNS, "history.json")))
    E = np.load(os.path.join(RUNS, "eval.npz"))
    A = json.load(open(os.path.join(RUNS, "ablations.json")))
    h = H["history"]; ep = [r["epoch"] for r in h]
    d, m, t = H["dataset"], H["model"], H["training"]
    v = H["verification"]

    s_ = E["te_scores"]; lab = E["te_labels"].astype(int)

    # ------------------------------------------------------ fig 1: training --
    fig, ax = plt.subplots(2, 2, figsize=(10, 6.8))

    ax[0, 0].plot(ep, [r["train_loss"] for r in h], lw=1.6, color=BLUE, label="train")
    ax[0, 0].plot(ep, [r["val_loss"] for r in h], lw=1.6, color=ORANGE, label="validation")
    ax[0, 0].set_xlabel("epoch"); ax[0, 0].set_ylabel("cross-entropy")
    ax[0, 0].set_title("Loss", loc="left"); leg(ax[0, 0])

    ax[0, 1].plot(ep, [r["train_acc"] for r in h], lw=1.6, color=BLUE, label="train")
    ax[0, 1].plot(ep, [r["val_acc"] for r in h], lw=1.6, color=ORANGE, label="validation")
    ax[0, 1].set_xlabel("epoch")
    ax[0, 1].set_ylabel(f"{d['train_identities']}-way accuracy")
    ax[0, 1].set_title("Identity classification", loc="left"); leg(ax[0, 1])

    ax[1, 0].plot(ep, [r["lr"] for r in h], lw=1.6, color=GREY)
    ax[1, 0].set_xlabel("epoch"); ax[1, 0].set_ylabel("learning rate")
    ax[1, 0].set_title("Cosine schedule", loc="left")
    # The schedule explains the shape of the loss curve, which is why it is here.
    vl = [r["val_loss"] for r in h]
    drop = int(np.argmax(np.array(vl[:-1]) - np.array(vl[1:]))) + 1
    ax[1, 0].axvline(ep[drop], color=ORANGE, ls=":", lw=1.4)
    # No leader line: the arrow had to cross the whole cosine curve to reach
    # its target, and the dotted rule already marks the epoch. Orange rather
    # than grey so it reads as belonging to the validation curve it describes,
    # not to the grey learning-rate curve it sits on top of.
    # Right edge held well clear of the dotted rule. At 0.84 the label was
    # touching it; the gap has to be explicit, because the rule's position is
    # set by the data and will move on the next run.
    ax[1, 0].text(0.76, 0.94, f"validation collapses\nat epoch {ep[drop]}",
                  transform=ax[1, 0].transAxes, ha="right", va="top",
                  fontsize=8, color=ORANGE, linespacing=1.4)

    gap = [r["train_acc"] - r["val_acc"] for r in h]
    ax[1, 1].plot(ep, gap, lw=1.6, color=RED)
    ax[1, 1].axhline(0, lw=0.8, color="#999", ls=":")
    ax[1, 1].fill_between(ep, 0, gap, color=RED, alpha=0.12)
    ax[1, 1].set_xlabel("epoch"); ax[1, 1].set_ylabel("train acc − val acc")
    ax[1, 1].set_title("Generalisation gap", loc="left")
    ax[1, 1].text(0.04, 0.94, f"peaks at {max(gap):.2f}, closes to {gap[-1]:.2f}\n"
                  "as the schedule decays",
                  transform=ax[1, 1].transAxes, va="top", fontsize=8, color="#555")

    fig.suptitle(f"Face embedding on {d['images']:,} LFW photographs · "
                 f"{m['params']:,} params · {t['epochs']} epochs · "
                 f"{t['seconds']:.0f}s on {t['device'].upper()} · seed {t['seed']}",
                 fontsize=10, y=0.995)
    fig.tight_layout(rect=[0, 0, 1, 0.97])
    p1 = os.path.join(OUT, "ml_training.png")
    fig.savefig(p1, dpi=200, bbox_inches="tight"); plt.close(fig)

    # ---------------------------------------------------- fig 2: evaluation --
    fig, ax = plt.subplots(2, 2, figsize=(10, 7.0))

    bins = np.linspace(-0.6, 1.0, 45)
    ax[0, 0].hist(s_[lab == 0], bins=bins, color=RED, alpha=0.65, label="different")
    ax[0, 0].hist(s_[lab == 1], bins=bins, color=BLUE, alpha=0.65, label="same")
    ax[0, 0].axvline(v["held_out"]["thr"], color="black", lw=1.2,
                     label=f"thr {v['held_out']['thr']:.2f}")
    ax[0, 0].set_xlabel("cosine similarity"); ax[0, 0].set_ylabel("pairs")
    ax[0, 0].set_title("Score separation", loc="left"); leg(ax[0, 0], ncol=3)

    ax[0, 1].plot(E["te_fpr"], E["te_tpr"], lw=1.8, color=BLUE,
                  label=f"held-out   AUC {v['held_out']['auc']:.3f}")
    ax[0, 1].plot(E["tr_fpr"], E["tr_tpr"], lw=1.4, ls="--", color=RED,
                  label=f"seen in training   AUC {v['seen_in_training']['auc']:.3f}")
    ax[0, 1].plot([0, 1], [0, 1], lw=0.7, ls=":", color="#999")
    ax[0, 1].set_xlabel("false accept rate"); ax[0, 1].set_ylabel("true accept rate")
    ax[0, 1].set_title("ROC", loc="left"); leg(ax[0, 1], ncol=1)

    far = np.maximum(E["te_fpr"], 1e-4); frr = np.maximum(1 - E["te_tpr"], 1e-4)
    ax[1, 0].plot(far, frr, lw=1.8, color=BLUE)
    ax[1, 0].set_xscale("log"); ax[1, 0].set_yscale("log")
    ax[1, 0].set_xlabel("false accept rate"); ax[1, 0].set_ylabel("false reject rate")
    ax[1, 0].set_title("DET — log axes, the biometric view", loc="left")
    i = int(np.argmin(np.abs(far - frr)))
    ax[1, 0].plot(far[i], frr[i], "o", color=RED, ms=6)
    ax[1, 0].text(0.05, 0.12, f"EER ≈ {far[i]*100:.0f}%", transform=ax[1, 0].transAxes,
                  fontsize=8, color=RED)

    ths = np.linspace(-0.4, 1.0, 200)
    FAR = [(s_[lab == 0] >= th).mean() for th in ths]
    FRR = [(s_[lab == 1] < th).mean() for th in ths]
    ACC = [((s_ >= th) == lab).mean() for th in ths]
    ax[1, 1].plot(ths, FAR, lw=1.6, color=RED, label="false accept")
    ax[1, 1].plot(ths, FRR, lw=1.6, color=ORANGE, label="false reject")
    ax[1, 1].plot(ths, ACC, lw=1.6, color=BLUE, label="accuracy")
    ax[1, 1].axvline(v["held_out"]["thr"], color="black", lw=1.0, ls=":")
    ax[1, 1].set_xlabel("threshold"); ax[1, 1].set_ylabel("rate")
    ax[1, 1].set_title("Threshold sweep", loc="left"); leg(ax[1, 1], ncol=3)

    fig.suptitle("Verification on identities never seen in training "
                 f"({d['test_identities']} people, {len(lab):,} pairs)",
                 fontsize=10, y=0.995)
    fig.tight_layout(rect=[0, 0, 1, 0.97])
    p2 = os.path.join(OUT, "ml_evaluation.png")
    fig.savefig(p2, dpi=200, bbox_inches="tight"); plt.close(fig)

    # ------------------------------------- fig 3: what to do next (4 panels) --
    fig, ax = plt.subplots(2, 2, figsize=(10, 7.4))

    ds = A["data_scaling"]
    ax[0, 0].plot([r["images"] for r in ds], [r["acc"] * 100 for r in ds],
                  "o-", lw=1.8, ms=6, color=BLUE)
    ax[0, 0].set_xlabel("training images"); ax[0, 0].set_ylabel("accuracy (%)")
    ax[0, 0].set_title("Data scaling — still climbing", loc="left")
    g = (ds[-1]["acc"] - ds[-2]["acc"]) * 100
    ax[0, 0].text(0.40, 0.20, f"the last 25% of the data\nwas still worth {g:+.1f} points",
                  transform=ax[0, 0].transAxes, fontsize=8, color="#555")

    es = A["embedding_width"]
    ax[0, 1].plot([r["emb"] for r in es], [r["acc"] * 100 for r in es],
                  "o-", lw=1.8, ms=6, color=BLUE)
    ax[0, 1].set_xscale("log", base=2)
    ax[0, 1].set_xticks([r["emb"] for r in es])
    ax[0, 1].set_xticklabels([str(r["emb"]) for r in es])
    ax[0, 1].set_xlabel("embedding dimension"); ax[0, 1].set_ylabel("accuracy (%)")
    lo = min(r["acc"] for r in es) * 100; hi = max(r["acc"] for r in es) * 100
    ax[0, 1].axhspan(lo, hi, color=GREY, alpha=0.13)
    ax[0, 1].set_ylim(lo - (hi - lo) * 0.6, hi + (hi - lo) * 0.3)
    ax[0, 1].set_title(f"Embedding width — flat within {hi-lo:.1f} pts", loc="left")
    ax[0, 1].text(0.05, 0.12, "no trend: width is not the\nbottleneck at this data scale",
                  transform=ax[0, 1].transAxes, fontsize=8, color="#555")

    # size vs accuracy, against the pretrained model
    afp = os.path.join(RUNS, "arcface.json")
    if os.path.exists(afp):
        af = json.load(open(afp))
        ax[1, 0].bar(["mine\n%.1f MB" % m["mb_fp32"], "ArcFace\n%.0f MB" % af["mb"]],
                     [v["held_out"]["acc"] * 100, af["acc"] * 100],
                     color=[BLUE, GREY], width=0.5)
        ax[1, 0].set_ylim(0, 112); ax[1, 0].set_ylabel("accuracy (%)")
        for xx, vv in zip([0, 1], [v["held_out"]["acc"]*100, af["acc"]*100]):
            ax[1, 0].text(xx, vv + 3, f"{vv:.1f}%", ha="center", fontsize=9)
        ax[1, 0].set_title(f"{af['mb']/m['mb_fp32']:.0f}x the weights, "
                           f"+{af['acc']*100 - v['held_out']['acc']*100:.0f} points",
                           loc="left")
        # Above the bars, not behind them: at y=0.10 this sat inside the blue
        # column and was unreadable.
        ax[1, 0].text(0.03, 0.97, "same pairs, same protocol,\nneither model "
                      "trained on these people",
                      transform=ax[1, 0].transAxes, va="top", fontsize=8,
                      color="#555")

    # calibration — is a similarity a probability?
    qs = np.quantile(s_, np.linspace(0, 1, 11))
    cx, cy = [], []
    for a, b in zip(qs[:-1], qs[1:]):
        sel = (s_ >= a) & (s_ < b)
        if sel.sum() >= 10:
            cx.append(float(s_[sel].mean())); cy.append(float(lab[sel].mean()))
    ax[1, 1].plot([min(cx), max(cx)], [0, 1], lw=0.8, ls=":", color="#999")
    ax[1, 1].plot(cx, cy, "o-", lw=1.6, ms=5, color=BLUE)
    ax[1, 1].set_xlabel("mean similarity in bin")
    ax[1, 1].set_ylabel("fraction actually same")
    ax[1, 1].set_title("Calibration", loc="left")
    ax[1, 1].text(0.05, 0.90, "a score of 0.6 is not a\n60% chance of a match",
                  transform=ax[1, 1].transAxes, va="top", fontsize=8, color="#555")

    fig.suptitle("What to do next — each ablation point is a training run from "
                 f"scratch · {A['epochs']} epochs · seed {A['seed']}",
                 fontsize=10, y=0.995)
    fig.tight_layout(rect=[0, 0, 1, 0.97])
    p3 = os.path.join(OUT, "ml_decisions.png")
    fig.savefig(p3, dpi=200, bbox_inches="tight"); plt.close(fig)

    for p in (p1, p2, p3):
        print(f"wrote {os.path.relpath(p, REPO)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
