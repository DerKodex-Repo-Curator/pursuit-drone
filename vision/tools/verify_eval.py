#!/usr/bin/env python3
"""Evaluate the face-verification stage as a decision system.

    vision/.venv/bin/python vision/tools/verify_eval.py

Why
---
`FaceConfig.match_thres = 0.36` and `veto_thres = 0.15` were chosen from a
handful of observed similarity values. That is a defensible way to pick a
starting point and a bad way to justify shipping one, because it says nothing
about the quantity that actually matters: how often the system locks onto the
WRONG person.

This is the missing measurement. It builds every same-person and
different-person pair available, sweeps the decision threshold across the whole
range, and reports the error rates at each — so the shipped operating point can
be defended by a number instead of by an anecdote.

The asymmetry is the whole design question
------------------------------------------
The two errors are not equally bad, and it is not close.

A **false reject** — failing to confirm the right person on one frame — costs
almost nothing. The face stage runs on roughly a quarter of frames, body ReID
sustains the lock in between, and the lock machine needs 5 consecutive
confirmations to acquire and 45 misses to release. One missed frame is
invisible.

A **false accept** — confirming a stranger — hands the lock to the wrong
person, and the aircraft then follows them. There is no recovery from inside
the system: the appearance model updates on face-confirmed frames, so a single
false accept is immediately reinforced by the body stage.

So the operating point should sit wherever the false-accept rate is negligible,
and should buy that with false rejects, which are nearly free. That is an
argument this script turns into a threshold.

Honest limits, stated up front
------------------------------
The gallery has TWO enrolled identities. That yields plenty of positive pairs
but only ONE cross-identity comparison, so the false-accept estimate rests on a
single pair of faces and its confidence interval is enormous. Faces detected in
the repository's test images are folded in as extra negative identities, which
helps and does not fix it.

**This is a small-sample result and is reported as one.** Every number below
carries its N. The correct next step is 10-20 enrolled people, which needs
consent and an afternoon, not a change of method — the harness does not change.

Nothing personal is written out. The gallery is gitignored biometric data; what
leaves this script is aggregate rates and a plot.
"""
from __future__ import annotations

import argparse
import glob
import itertools
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from pursuit.config import Config                       # noqa: E402


def unit(v: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(v, axis=-1, keepdims=True)
    return v / np.maximum(n, 1e-12)


def load_gallery(path: str) -> dict[str, np.ndarray]:
    out: dict[str, np.ndarray] = {}
    for f in sorted(glob.glob(os.path.join(path, "*.npz"))):
        d = np.load(f, allow_pickle=True)
        if "vectors" not in d:
            continue
        out[os.path.splitext(os.path.basename(f))[0]] = unit(d["vectors"])
    return out


def extra_identities(cfg: Config) -> dict[str, np.ndarray]:
    """Faces from the repository's fixture images, as extra NEGATIVE identities.

    They contribute no positive pairs — one image each, so there is nothing to
    pair them with — but every additional identity multiplies the number of
    genuinely different-person comparisons, which is the estimate most starved
    of data here.
    """
    out: dict[str, np.ndarray] = {}
    data = os.path.join(ROOT, "tests", "data")
    if not os.path.isdir(data):
        return out
    try:
        import cv2
        from pursuit.face import FaceAnalyzer
    except Exception as exc:                              # pragma: no cover
        print(f"  (skipping fixture faces: {exc})")
        return out

    try:
        fa = FaceAnalyzer(cfg.face, cfg.providers)
    except FileNotFoundError:
        print("  (skipping fixture faces: models not fetched)")
        return out

    for img_path in sorted(glob.glob(os.path.join(data, "*.jpg"))):
        img = cv2.imread(img_path)
        if img is None:
            continue
        faces = fa.detect_and_embed(img)
        stem = os.path.splitext(os.path.basename(img_path))[0]
        for i, f in enumerate(faces):
            out[f"{stem}#{i}"] = unit(f.embedding[None, :])
    return out


def detect_collisions(gallery: dict[str, np.ndarray]) -> list[tuple[str, str]]:
    """Find identity labels that are probably the same person.

    This check exists because the first run of this script found exactly that,
    and would otherwise have reported a meaningless AUC of 0.83 with total
    confidence. Two identities were enrolled, `Paing` and `alex`; the median
    similarity BETWEEN them was 0.579 while `Paing`'s median similarity to
    ITSELF was 0.560. A stranger cannot resemble you more than you do. The
    second enrolment had been made from the same webcam and the same face as
    the first — a placeholder that was never replaced.

    Every 'different person' pair drawn from those two labels was really a
    same-person pair, so the negative distribution was poisoned and every error
    rate computed from it was fiction. An evaluation that cannot notice this is
    worse than no evaluation, because it produces a number to be confident
    about.

    Criterion: two labels collide when their cross-similarity median is at
    least as high as the lower of their two self-similarity medians. That is
    deliberately blunt — it is a smoke alarm, not a classifier.
    """
    names = [n for n in sorted(gallery) if gallery[n].shape[0] >= 2]
    self_med = {}
    for n in names:
        v = gallery[n]
        s = [float(v[a] @ v[b]) for a, b in itertools.combinations(range(len(v)), 2)]
        self_med[n] = float(np.median(s))

    hits = []
    for n1, n2 in itertools.combinations(names, 2):
        cross = float(np.median(gallery[n1] @ gallery[n2].T))
        if cross >= min(self_med[n1], self_med[n2]):
            hits.append((n1, n2))
    return hits


def build_pairs(gallery: dict[str, np.ndarray]):
    """Every same-identity and different-identity cosine similarity."""
    pos, neg = [], []
    names = sorted(gallery)

    for n in names:
        v = gallery[n]
        for a, b in itertools.combinations(range(len(v)), 2):
            pos.append(float(v[a] @ v[b]))

    for n1, n2 in itertools.combinations(names, 2):
        v1, v2 = gallery[n1], gallery[n2]
        for a in range(len(v1)):
            for b in range(len(v2)):
                neg.append(float(v1[a] @ v2[b]))

    return np.array(pos), np.array(neg)


def roc(pos: np.ndarray, neg: np.ndarray, grid: np.ndarray):
    """FAR and FRR at every threshold. Decision is `similarity >= t`."""
    far = np.array([(neg >= t).mean() if neg.size else np.nan for t in grid])
    frr = np.array([(pos < t).mean() if pos.size else np.nan for t in grid])
    return far, frr


def auc_score(pos: np.ndarray, neg: np.ndarray) -> float:
    """Probability that a random positive outscores a random negative.

    Computed directly from the rank statistic (Mann-Whitney U) rather than by
    integrating the ROC, so it does not inherit the grid's resolution.
    """
    if not pos.size or not neg.size:
        return float("nan")
    allv = np.concatenate([pos, neg])
    order = allv.argsort()
    ranks = np.empty_like(order, dtype=float)
    ranks[order] = np.arange(1, allv.size + 1)
    # Average ranks over ties, or identical scores bias the estimate.
    _, inv, counts = np.unique(allv, return_inverse=True, return_counts=True)
    sums = np.zeros(counts.size)
    np.add.at(sums, inv, ranks)
    ranks = (sums / counts)[inv]
    r_pos = ranks[:pos.size].sum()
    u = r_pos - pos.size * (pos.size + 1) / 2.0
    return float(u / (pos.size * neg.size))


def wilson(k: int, n: int, z: float = 1.96) -> tuple[float, float]:
    """95% interval for a rate. Normal approximation is useless at k = 0, and
    k = 0 is exactly the case this script hopes to report."""
    if n == 0:
        return (float("nan"), float("nan"))
    p = k / n
    d = 1 + z * z / n
    c = (p + z * z / (2 * n)) / d
    h = z * np.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return (max(0.0, c - h), min(1.0, c + h))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--gallery", default=None)
    ap.add_argument("--no-fixtures", action="store_true")
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    cfg = Config()
    gallery_dir = args.gallery or os.path.join(ROOT, cfg.gallery)
    gallery = load_gallery(gallery_dir)
    if not gallery:
        print(f"no enrolled identities in {gallery_dir}\n"
              f"  run:  python vision/enroll.py --name you")
        return 1

    enrolled = {k: v.shape[0] for k, v in gallery.items()}
    print("Enrolled identities (biometric — never committed):")
    for k, n in enrolled.items():
        print(f"  {k:<12} {n} embeddings")

    # Before anything is measured, check the labels are believable.
    collisions = detect_collisions(gallery)
    if collisions:
        print("\n  *** IDENTITY COLLISION ***")
        for n1, n2 in collisions:
            c = float(np.median(gallery[n1] @ gallery[n2].T))
            v1, v2 = gallery[n1], gallery[n2]
            s1 = float(np.median([v1[a] @ v1[b] for a, b
                                  in itertools.combinations(range(len(v1)), 2)]))
            print(f"  '{n1}' and '{n2}' score {c:.3f} to each other, and "
                  f"'{n1}' scores {s1:.3f} to itself.")
        print("  A stranger cannot resemble you more than you do — these "
              "labels are the")
        print("  same person. They are MERGED into one identity below; "
              "treating them as")
        print("  different would poison every negative pair and produce a "
              "confident,")
        print("  meaningless error rate.")
        for n1, n2 in collisions:
            if n1 in gallery and n2 in gallery:
                gallery[n1] = np.vstack([gallery[n1], gallery.pop(n2)])

    if not args.no_fixtures:
        extra = extra_identities(cfg)
        if extra:
            print(f"\n  + {len(extra)} face(s) from tests/data as extra "
                  f"negative identities")
            gallery.update(extra)

    pos, neg = build_pairs(gallery)
    n_ident = len(gallery)
    print(f"\n{len(pos)} same-person pairs, {len(neg)} different-person pairs, "
          f"from {n_ident} identities")
    if len(enrolled) < 3:
        print("  WARNING: fewer than 3 multi-image identities. The "
              "false-accept\n"
              "  estimate below rests on very few distinct face pairs and its\n"
              "  interval is correspondingly wide. Treat it as indicative.")

    print(f"\nsame-person     min {pos.min():.3f}  median {np.median(pos):.3f}"
          f"  max {pos.max():.3f}")
    print(f"different       min {neg.min():.3f}  median {np.median(neg):.3f}"
          f"  max {neg.max():.3f}")
    gap = pos.min() - neg.max()
    print(f"separation      {gap:+.3f}   "
          f"({'clean — no overlap' if gap > 0 else 'OVERLAPPING'})")

    grid = np.linspace(-0.2, 1.0, 601)
    far, frr = roc(pos, neg, grid)

    print(f"\nAUC             {auc_score(pos, neg):.4f}")
    i_eer = int(np.nanargmin(np.abs(far - frr)))
    print(f"EER             {0.5*(far[i_eer]+frr[i_eer]):.4f} "
          f"at threshold {grid[i_eer]:.3f}")

    print(f"\n{'threshold':>10} {'FAR':>8} {'FRR':>8}   note")
    print("-" * 62)
    marks = {round(cfg.face.match_thres, 3): "SHIPPED match_thres",
             round(cfg.face.veto_thres, 3): "SHIPPED veto_thres",
             round(float(grid[i_eer]), 3): "equal error rate"}
    for t in sorted(set(list(np.arange(0.0, 0.95, 0.1)) + list(marks))):
        j = int(np.argmin(np.abs(grid - t)))
        note = marks.get(round(t, 3), "")
        print(f"{grid[j]:>10.3f} {far[j]:>8.4f} {frr[j]:>8.4f}   {note}")

    # The shipped point, with an interval.
    j = int(np.argmin(np.abs(grid - cfg.face.match_thres)))
    k_fa = int((neg >= grid[j]).sum())
    k_fr = int((pos < grid[j]).sum())
    lo, hi = wilson(k_fa, neg.size)
    print(f"\nAt the shipped match_thres = {cfg.face.match_thres}:")
    print(f"  false accepts  {k_fa}/{neg.size}   "
          f"95% CI [{lo:.4f}, {hi:.4f}]")
    print(f"  false rejects  {k_fr}/{pos.size}")
    print(f"\n  Read: with this data the false-accept rate is consistent with "
          f"anything\n  up to {hi*100:.1f}%. That is the honest statement, and "
          f"it is a statement about\n  the SAMPLE SIZE, not about the model. "
          f"Ten to twenty enrolled people\n  would tighten it by roughly an "
          f"order of magnitude.")

    # Where could the threshold go?
    safe = grid[(far == 0.0) & ~np.isnan(far)]
    if safe.size:
        print(f"\n  Lowest threshold with zero observed false accepts: "
              f"{safe.min():.3f}")
        j2 = int(np.argmin(np.abs(grid - safe.min())))
        print(f"  There, FRR = {frr[j2]:.4f} against {frr[j]:.4f} at the "
              f"shipped point.")
        if safe.min() < cfg.face.match_thres:
            print(f"  So {cfg.face.match_thres} is CONSERVATIVE on this data — "
                  f"it buys margin\n  against the small sample rather than "
                  f"against observed errors, which is\n  the right way round "
                  f"given the asymmetry.")

    _plot(pos, neg, grid, far, frr, cfg, n_ident, args.out)
    return 0


def _plot(pos, neg, grid, far, frr, cfg, n_ident, out) -> None:
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("\n(matplotlib not installed — skipping the plot)")
        return

    repo = os.path.dirname(ROOT)
    out = out or os.path.join(repo, "docs", "media", "verification_roc.png")
    os.makedirs(os.path.dirname(out), exist_ok=True)

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))

    bins = np.linspace(-0.2, 1.0, 45)
    ax1.hist(neg, bins=bins, alpha=0.65, color="#d62728",
             label=f"different person (n={neg.size})")
    ax1.hist(pos, bins=bins, alpha=0.65, color="#2ca02c",
             label=f"same person (n={pos.size})")
    ax1.axvline(cfg.face.match_thres, color="#1f77b4", lw=2,
                label=f"match_thres {cfg.face.match_thres}")
    ax1.axvline(cfg.face.veto_thres, color="#ff7f0e", lw=2, ls="--",
                label=f"veto_thres {cfg.face.veto_thres}")
    ax1.set_xlabel("ArcFace cosine similarity")
    ax1.set_ylabel("pairs")
    ax1.set_title(f"Score separation ({n_ident} identities)")
    ax1.legend(fontsize=8)
    ax1.grid(alpha=0.3)

    ax2.plot(grid, far, lw=2, color="#d62728", label="false accept (a stranger "
                                                     "gets the lock)")
    ax2.plot(grid, frr, lw=2, color="#2ca02c", label="false reject (one frame "
                                                     "missed)")
    ax2.axvline(cfg.face.match_thres, color="#1f77b4", lw=2,
                label=f"shipped {cfg.face.match_thres}")
    ax2.set_yscale("symlog", linthresh=1e-3)
    ax2.set_xlabel("decision threshold")
    ax2.set_ylabel("error rate")
    ax2.set_title("The two errors are not equally bad")
    ax2.legend(fontsize=8)
    ax2.grid(alpha=0.3)

    fig.suptitle("PURSUIT-1 face verification — operating point, and why\n"
                 "SMALL SAMPLE: see the confidence interval in the tool output",
                 fontsize=11)
    fig.tight_layout()
    fig.savefig(out, dpi=130)
    print(f"\nplot -> {os.path.relpath(out, repo)}")


if __name__ == "__main__":
    sys.exit(main())
