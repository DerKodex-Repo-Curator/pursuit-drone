#!/usr/bin/env python3
"""Break a far_eval.py score dump down by label.

The headline false-accept rate is an average over whoever happened to be in
the impostor set. That average is the wrong number if the people who will
actually stand near the drone are not distributed like the dataset -- which
for a benchmark built from 2000s Western news photography, they are not.
"""
from __future__ import annotations

import collections
import json
import sys

import numpy as np


def wilson_upper(k: int, n: int, z: float = 1.96) -> float:
    """Upper 95% bound on a rate. Valid at k=0, unlike a normal interval."""
    if n == 0:
        return float("nan")
    p = k / n
    d = 1 + z * z / n
    centre = p + z * z / (2 * n)
    half = z * np.sqrt(p * (1 - p) / n + z * z / (4 * n * n))
    return (centre + half) / d


def main() -> int:
    path = sys.argv[1]
    thresholds = [float(x) for x in sys.argv[2:]] or [0.36, 0.45, 0.50]
    d = json.load(open(path))
    s = np.asarray(d["scores"], dtype=np.float64)
    labels = d["labels"]
    gen = np.asarray(d.get("genuine_loo", []), dtype=np.float64)

    print(f"{path}   n = {len(s):,} impostor faces")
    if gen.size:
        print(f"genuine leave-one-out: min {gen.min():.3f}  mean {gen.mean():.3f}")
    print(f"impostor: mean {s.mean():.3f}  max {s.max():.3f}  "
          f"99.9th pct {np.percentile(s, 99.9):.3f}")

    groups = collections.defaultdict(list)
    for lab, sc in zip(labels, s):
        groups[lab].append(sc)
    # A per-identity breakdown of LFW would be 5,749 groups of ~2; only report
    # the breakdown when the labels are actually few enough to mean something.
    if 1 < len(groups) <= 12:
        for t in thresholds:
            print(f"\n--- threshold {t:.2f} ---")
            print(f"{'group':>18} {'n':>7} {'accepts':>9} {'FAR':>10} "
                  f"{'95% upper':>11} {'max sim':>9}")
            for lab in sorted(groups, key=lambda g: -np.mean(groups[g])):
                a = np.asarray(groups[lab])
                k = int((a >= t).sum())
                print(f"{lab:>18} {len(a):7,} {k:9,} {k/len(a):10.2e} "
                      f"{wilson_upper(k, len(a)):11.2e} {a.max():9.3f}")
            k = int((s >= t).sum())
            print(f"{'ALL':>18} {len(s):7,} {k:9,} {k/len(s):10.2e} "
                  f"{wilson_upper(k, len(s)):11.2e} {s.max():9.3f}")
    else:
        top = sorted(zip(s, labels), reverse=True)[:12]
        print("\nclosest impostors observed:")
        for sc, lab in top:
            print(f"   {sc:.3f}  {lab}")

    print("\nhow much margin is left, worst impostor vs worst genuine:")
    if gen.size:
        print(f"   worst genuine {gen.min():.3f}  -  best impostor {s.max():.3f}"
              f"  =  {gen.min()-s.max():+.3f}")
        mid = (gen.min() + s.max()) / 2
        print(f"   midpoint between the two populations: {mid:.3f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
