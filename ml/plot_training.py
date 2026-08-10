#!/usr/bin/env python3
"""Plot the real training run, and benchmark it against the pretrained ArcFace.

    ml/.venv/bin/python ml/plot_training.py

Produces two figures from `ml/runs/history.json` and `ml/runs/eval.npz`:

  training_curves.png   loss and accuracy per epoch — the actual run
  verification_bench.png ROC of the trained model against ArcFace, on the
                        SAME pairs, plus the protocol comparison

Nothing here is illustrative. Every point is from the run recorded in
ml/runs/, and the ArcFace numbers are computed live on the same image pairs.
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

# Plain scientific styling, on purpose.
#
# An earlier version of this file drew these on a near-black background with
# saturated blue/purple/green accents, rounded cards and accent bars. It looked
# like every AI-generated dashboard of the last two years, because that palette
# IS the Tailwind default that those tools emit — and a reader pattern-matches
# it before reading a single number. For a figure whose entire job is to be
# believed, that is a self-inflicted wound.
#
# So: white ground, black text, muted colours, matplotlib defaults left alone.
# This is what a figure in a paper looks like.
C = dict(mine="#1b6ca8", pre="#7a7a7a", inflated="#c0392b",
         train="#1b6ca8", val="#d1701a", grid="#cccccc", note="#555555")

plt.rcParams.update({
    "font.family": "sans-serif",
    "font.sans-serif": ["Helvetica Neue", "Helvetica", "Arial", "DejaVu Sans"],
    "font.size": 10,
    "axes.titlesize": 11,
    "axes.labelsize": 10,
    "axes.spines.top": False,
    "axes.spines.right": False,
    "axes.grid": True,
    "grid.color": C["grid"],
    "grid.linewidth": 0.6,
    "grid.alpha": 0.7,
    "legend.frameon": False,
    "figure.facecolor": "white",
    "axes.facecolor": "white",
    "savefig.facecolor": "white",
})


def style(ax):
    ax.set_axisbelow(True)
    return ax


def arcface_on_same_pairs(pairs, labels):
    """Score the SAME image pairs with the pretrained ArcFace.

    Caveat stated plainly: ArcFace expects 112x112 faces aligned by five
    landmarks. These are 62x47 LFW crops upscaled, with no landmark
    alignment, so this UNDERSTATES ArcFace — its published LFW number is
    99.8% under the proper protocol. It is still the right comparison to
    show, because it is the same input both models get.
    """
    try:
        import cv2
        import onnxruntime as ort
        from sklearn.datasets import fetch_lfw_people
    except ImportError as e:
        print(f"  (ArcFace comparison skipped: {e})")
        return None

    path = os.path.join(REPO, "vision", "models", "buffalo_l", "w600k_r50.onnx")
    if not os.path.exists(path):
        print("  (ArcFace comparison skipped: weights not fetched)")
        return None

    d = fetch_lfw_people(min_faces_per_person=10, resize=0.5, color=True)
    imgs = (d.images * 255).astype(np.uint8)

    opts = ort.SessionOptions(); opts.log_severity_level = 3
    sess = ort.InferenceSession(path, opts, providers=["CPUExecutionProvider"])
    iname = sess.get_inputs()[0].name

    need = sorted({int(i) for p in pairs for i in p})
    emb = {}
    B = 64
    for s in range(0, len(need), B):
        chunk = need[s:s+B]
        batch = np.stack([
            cv2.resize(imgs[i], (112, 112), interpolation=cv2.INTER_CUBIC)
            for i in chunk]).astype(np.float32)
        batch = (batch - 127.5) / 127.5
        batch = np.transpose(batch, (0, 3, 1, 2))
        out = sess.run(None, {iname: np.ascontiguousarray(batch)})[0]
        out /= np.maximum(np.linalg.norm(out, axis=1, keepdims=True), 1e-12)
        for k, i in enumerate(chunk):
            emb[i] = out[k]

    s = np.array([float(emb[int(a)] @ emb[int(b)]) for a, b in pairs])
    order = np.argsort(-s); ls = labels[order]
    tpr = np.cumsum(ls) / max(labels.sum(), 1)
    fpr = np.cumsum(1 - ls) / max((1 - labels).sum(), 1)
    auc = float(np.trapezoid(tpr, fpr))
    ths = np.linspace(-1, 1, 401)
    acc = max(float(((s >= t) == labels).mean()) for t in ths)
    return dict(fpr=fpr, tpr=tpr, auc=auc, acc=acc)


def main() -> int:
    with open(os.path.join(RUNS, "history.json")) as f:
        H = json.load(f)
    E = np.load(os.path.join(RUNS, "eval.npz"))
    h = H["history"]
    ep = [r["epoch"] for r in h]

    # ------------------------------------------------------- training curves --
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(7.5, 6.2), sharex=True)
    style(a1); style(a2)

    a1.plot(ep, [r["train_loss"] for r in h], lw=1.8, color=C["train"], label="train")
    a1.plot(ep, [r["val_loss"] for r in h], lw=1.8, color=C["val"], label="validation")
    a1.set_ylabel("cross-entropy loss")
    a1.legend(loc="lower right", bbox_to_anchor=(1.0, 1.0), ncol=2,
              frameon=False, fontsize=8, borderaxespad=0.2)
    d = H["dataset"]; m = H["model"]; t = H["training"]
    a1.set_title(
        f"Face embedding trained from scratch on {d['images']:,} real LFW "
        f"photographs\n{m['params']:,} parameters ({m['mb_fp32']} MB) · "
        f"{t['epochs']} epochs · {t['seconds']}s on {t['device'].upper()} · "
        f"seed {t['seed']}", fontsize=11, loc="left")

    a2.plot(ep, [r["train_acc"] for r in h], lw=1.8, color=C["train"], label="train")
    a2.plot(ep, [r["val_acc"] for r in h], lw=1.8, color=C["val"], label="validation")
    a2.set_xlabel("epoch"); a2.set_ylabel(f"identity accuracy\n({d['train_identities']}-way)")
    a2.legend(loc="lower right", bbox_to_anchor=(1.0, 1.0), ncol=2,
              frameon=False, fontsize=8, borderaxespad=0.2)

    # The honest annotation: validation went WORSE before it got better.
    vl = [r["val_loss"] for r in h]
    worst = int(np.argmax(vl))
    if worst > 2:
        a1.annotate("validation loss rises for ~30 epochs\nbefore the "
                    "representation generalises",
                    xy=(ep[worst], vl[worst]),
                    xytext=(ep[worst] + 2, max(vl) * 0.93),
                    arrowprops=dict(arrowstyle="->", color=C["note"], lw=0.9),
                    fontsize=9, color=C["note"])

    fig.tight_layout()
    p1 = os.path.join(OUT, "training_curves.png")
    fig.savefig(p1, dpi=200, bbox_inches="tight"); plt.close(fig)

    # -------------------------------------------------- verification benchmark --
    fig, (b1, b2) = plt.subplots(1, 2, figsize=(10, 4.4))
    style(b1); style(b2)

    v = H["verification"]
    b1.plot(E["te_fpr"], E["te_tpr"], lw=2.0, color=C["mine"],
            label=f"held-out identities   AUC {v['held_out']['auc']:.3f}")
    b1.plot(E["tr_fpr"], E["tr_tpr"], lw=1.6, ls="--", color=C["inflated"],
            label=f"identities seen in training   AUC {v['seen_in_training']['auc']:.3f}")

    pairs = None
    af = None
    try:
        # Rebuild the exact held-out pairs to score ArcFace on them.
        import importlib.util
        spec = importlib.util.spec_from_file_location(
            "tfe", os.path.join(ROOT, "train_face_embed.py"))
        tfe = importlib.util.module_from_spec(spec); spec.loader.exec_module(tfe)
        rng = np.random.default_rng(tfe.SEED)
        X, y, names, tr, te, tr_ids, te_ids = tfe.load_identity_split(rng)
        # burn the same draws the training script made, so the pairs match
        _ = rng.permutation(len(tr))
        pairs, lab = tfe.make_pairs(y, te, rng)
        af = arcface_on_same_pairs(pairs, lab)
    except Exception as exc:
        print(f"  (ArcFace comparison skipped: {exc})")

    if af:
        b1.plot(af["fpr"], af["tpr"], lw=2.0, color=C["pre"],
                label=f"ArcFace, pretrained   AUC {af['auc']:.3f}")
    b1.plot([0, 1], [0, 1], lw=0.8, ls=":", color="#999999")
    b1.set_xlabel("false accept rate"); b1.set_ylabel("true accept rate")
    b1.set_title("Verification ROC, identical image pairs", loc="left")
    b1.legend(fontsize=8, loc="lower right", bbox_to_anchor=(1.0, 1.0),
              ncol=1, frameon=False, borderaxespad=0.2)

    # size vs accuracy
    labels = ["mine\n(trained here)", "ArcFace\n(pretrained)"]
    sizes = [m["mb_fp32"], 174.4]
    accs = [v["held_out"]["acc"] * 100, (af["acc"] * 100) if af else np.nan]
    cols = [C["mine"], C["pre"]]
    bars = b2.bar(labels, accs, color=cols, width=0.5)
    for bar, s, a in zip(bars, sizes, accs):
        if np.isfinite(a):
            b2.text(bar.get_x() + bar.get_width()/2, a + 1.5,
                    f"{a:.1f}%\n{s:.1f} MB", ha="center", fontsize=9.5,
                    color="black")
    b2.set_ylim(0, 108); b2.set_ylabel("verification accuracy (%)")
    b2.set_title(f"{sizes[1]/sizes[0]:.0f}x the weights, +{accs[1]-accs[0]:.0f} points",
                 loc="left")

    fig.text(0.5, 0.015, "Held-out identities only. ArcFace scored on the same "
             "unaligned 62x47 crops, which understates it; its published LFW "
             "figure is 99.8% with alignment.",
             ha="center", fontsize=8.5, color=C["note"])
    fig.tight_layout(rect=[0, 0.06, 1, 1])
    p2 = os.path.join(OUT, "verification_bench.png")
    fig.savefig(p2, dpi=200, bbox_inches="tight"); plt.close(fig)

    print(f"wrote {os.path.relpath(p1, REPO)}")
    print(f"wrote {os.path.relpath(p2, REPO)}")
    if af:
        print(f"\nArcFace on the same pairs: AUC {af['auc']:.4f}, "
              f"acc {af['acc']*100:.1f}%")
        print(f"Mine:                      AUC {v['held_out']['auc']:.4f}, "
              f"acc {v['held_out']['acc']*100:.1f}%  "
              f"({m['mb_fp32']} MB vs 174.4 MB)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
