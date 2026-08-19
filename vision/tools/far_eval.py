#!/usr/bin/env python3
"""Estimate the face stage's FALSE-ACCEPT rate against a large impostor set.

The shipped `match_thres = 0.36` was defended by one stranger and 64 pairs,
which bounds the false-accept rate at nothing better than 4.7% (rule of three).
This runs thousands of genuine strangers through the SAME code path the drone
uses and turns that bound into a measurement.

Matches the pipeline exactly or the number describes nothing:
  * SCRFD detection, 5-point landmarks, umeyama alignment to 112x112
  * ArcFace w600k_r50, L2-normalised
  * scoring is Identity.similarity: MEAN OF THE TOP 3 against the 8 enrolled
    vectors -- not the max, which is what a naive pairwise sweep would use
  * faces under FaceConfig.min_face_px are skipped, as in flight

Writes only aggregate rates and a per-image score array. No impostor image, no
embedding and nothing from the gallery is persisted.

What was actually run (2026-08-17, reported as VIS-001)
-------------------------------------------------------
LFW (Labeled Faces in the Wild), every unique face that SCRFD found at or above
FaceConfig.min_face_px:

    vision/.venv/bin/python vision/tools/far_eval.py \
        --images '/path/to/lfw/**/*.jpg' --label-from-dir --out scores.json
    vision/.venv/bin/python vision/tools/far_breakdown.py scores.json

Result: ZERO false accepts in 24,110 impostor faces at the shipped
match_thres = 0.36. Zero events in n trials bounds the rate at 3/n by the rule
of three, so that is a 95% upper bound of 1.2e-4 -- not proof of zero.

Read far_breakdown.py before quoting the headline number. LFW is 2000s Western
news photography and is not distributed like the people who will actually stand
in front of this drone, so the average over it is the wrong statistic if the
subgroups differ. The breakdown is there to check that, not as decoration.

The dataset is deliberately NOT vendored: it is large, separately licensed, and
consists of photographs of real people who did not consent to being in this
repository. Point --images at your own copy.
"""
from __future__ import annotations

import argparse
import glob
import json
import os
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, "/Users/paing/Documents/pursuit_drone/vision")
from pursuit.config import Config              # noqa: E402
from pursuit.face import FaceAnalyzer          # noqa: E402

TARGET = "/Users/paing/Documents/pursuit_drone/vision/targets/Paing.npz"


def similarity(vectors: np.ndarray, emb: np.ndarray, top_k: int = 3) -> float:
    """Byte-for-byte the same statistic as gallery.Identity.similarity."""
    sims = vectors @ emb.astype(np.float32)
    k = min(top_k, len(sims))
    return float(np.sort(sims)[-k:].mean())


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--images", required=True,
                    help="glob for impostor images")
    ap.add_argument("--limit", type=int, default=0)
    ap.add_argument("--out", default="scores.json")
    ap.add_argument("--label-from-dir", action="store_true",
                    help="use the parent directory name as an identity label")
    args = ap.parse_args()

    cfg = Config()
    vectors = np.load(TARGET)["vectors"].astype(np.float32)
    # The enrolled vectors are stored normalised, but never assume it.
    vectors = vectors / np.linalg.norm(vectors, axis=1, keepdims=True)

    fa = FaceAnalyzer(cfg.face, cfg.providers)
    paths = sorted(glob.glob(args.images, recursive=True))
    if args.limit:
        paths = paths[:args.limit]
    print(f"impostor images found: {len(paths):,}")
    print(f"enrolled vectors: {vectors.shape}   min_face_px={cfg.face.min_face_px}")

    scores, labels, kept, no_face, too_small = [], [], 0, 0, 0
    t0 = time.time()
    for i, p in enumerate(paths):
        img = cv2.imread(p)
        if img is None:
            continue
        faces = fa.detect(img)
        if not faces:
            no_face += 1
            continue
        # LFW-style sets are one subject per image; take the biggest face.
        f = max(faces, key=lambda z: z.w * z.h)
        if min(f.w, f.h) < cfg.face.min_face_px:
            too_small += 1
            continue
        emb = fa.embed(img, f)
        scores.append(similarity(vectors, emb))
        labels.append(os.path.basename(os.path.dirname(p))
                      if args.label_from_dir else "")
        kept += 1
        if (i + 1) % 1000 == 0:
            el = time.time() - t0
            print(f"  {i+1:6,}/{len(paths):,}  kept {kept:,}  "
                  f"{1000*el/(i+1):.1f} ms/img  eta {el/(i+1)*(len(paths)-i-1)/60:.1f} min")

    s = np.asarray(scores, dtype=np.float64)
    print(f"\nembedded {len(s):,} impostor faces in {time.time()-t0:.0f}s"
          f"   (no face: {no_face:,}   too small: {too_small:,})")
    if len(s) == 0:
        print("nothing embedded -- check the glob")
        return 1

    # Leave-one-out on the enrolled shots: the honest genuine distribution
    # under the same top-3 statistic.
    gen = []
    for i in range(len(vectors)):
        rest = np.delete(vectors, i, axis=0)
        gen.append(similarity(rest, vectors[i]))
    gen = np.asarray(gen)

    print(f"\ngenuine (leave-one-out, n={len(gen)}): "
          f"min {gen.min():.3f}  mean {gen.mean():.3f}  max {gen.max():.3f}")
    print(f"impostor (n={len(s):,}):              "
          f"min {s.min():.3f}  mean {s.mean():.3f}  max {s.max():.3f}")
    print(f"impostor 99th pct {np.percentile(s,99):.3f}   "
          f"99.9th {np.percentile(s,99.9):.3f}")

    print("\n" + "="*74)
    print(f"{'thresh':>7} {'false accepts':>15} {'FAR':>11} {'95% upper':>11}"
          f" {'genuine kept':>13}")
    print("="*74)
    n = len(s)
    for t in (0.20, 0.25, 0.28, 0.30, 0.33, 0.36, 0.40, 0.45, 0.50, 0.55):
        fa_n = int((s >= t).sum())
        far = fa_n / n
        # Rule of three when nothing was observed; otherwise a normal-ish band.
        if fa_n == 0:
            hi = 3.0 / n
        else:
            hi = far + 1.96 * np.sqrt(far * (1 - far) / n)
        keep = (gen >= t).mean()
        note = "  <- SHIPPED" if abs(t - 0.36) < 1e-9 else ""
        print(f"{t:7.2f} {fa_n:15,} {far:11.2e} {hi:11.2e} {100*keep:12.0f}%{note}")

    print("\nprobability of >=1 false accept per 10-min flight, 3 bystanders")
    print("(face stage fires ~5 Hz -> ~9,000 impostor comparisons)")
    N = 9000
    for t in (0.28, 0.36, 0.45, 0.50):
        fa_n = int((s >= t).sum())
        far = fa_n / n if fa_n else 3.0 / n      # use the bound when zero
        tag = "measured" if fa_n else "upper bound"
        print(f"   thresh {t:.2f}   FAR {far:.2e} ({tag})   "
              f"P = {1-(1-far)**N:7.2%}")

    with open(args.out, "w") as fh:
        json.dump({"scores": [round(float(x), 4) for x in s],
                   "labels": labels,
                   "genuine_loo": [round(float(x), 4) for x in gen]}, fh)
    print(f"\nwrote {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
