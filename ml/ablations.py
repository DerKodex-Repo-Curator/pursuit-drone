#!/usr/bin/env python3
"""Ablations: the plots that actually change a decision.

    ml/.venv/bin/python ml/ablations.py

Loss curves tell you the optimiser worked. They do not tell you what to do
next. These two do:

  DATA SCALING     accuracy against how much training data was used. If the
                   curve is still climbing at 100%, collecting more data is
                   the highest-value thing available. If it has flattened, more
                   data is wasted effort and the model or the loss is the
                   limit. This is the single most decision-useful plot in
                   applied machine learning and it is usually missing.

  EMBEDDING WIDTH  accuracy against embedding dimension, with model size on
                   the same axis. PURSUIT-1 runs on a 66 g aircraft, so
                   "how small can this get before it stops working" is not an
                   academic question.

Each point is a real training run from scratch. Every run uses the same
identity-disjoint split and the same seed, so the only thing varying is the
axis being swept.

Runs at reduced epochs to keep the sweep to a few minutes; the absolute numbers
are therefore slightly below the full-length run in train_face_embed.py, and
the SHAPE is what these plots are for.
"""
from __future__ import annotations

import importlib.util
import json
import os
import time

import numpy as np
import torch

ROOT = os.path.dirname(os.path.abspath(__file__))
RUNS = os.path.join(ROOT, "runs")

spec = importlib.util.spec_from_file_location(
    "tfe", os.path.join(ROOT, "train_face_embed.py"))
tfe = importlib.util.module_from_spec(spec)
spec.loader.exec_module(tfe)

EPOCHS = 25          # reduced from 40 to keep the sweep short
DATA_FRACS = [0.15, 0.30, 0.50, 0.75, 1.0]
EMB_DIMS = [16, 32, 64, 128, 256]


def one_run(X, y, tr, te, train_ids, dev, seed, frac=1.0, emb=128):
    """Train once and return held-out verification accuracy."""
    rng = np.random.default_rng(seed)
    torch.manual_seed(seed)

    if frac < 1.0:
        # Subsample IMAGES, keeping every identity represented — otherwise the
        # sweep confounds 'less data' with 'fewer classes'.
        keep = []
        by = {}
        for i in tr:
            by.setdefault(int(y[i]), []).append(int(i))
        for k, v in by.items():
            n = max(2, int(round(len(v) * frac)))
            keep += list(rng.choice(v, size=min(n, len(v)), replace=False))
        tr_use = np.array(sorted(keep))
    else:
        tr_use = tr

    tfe.EPOCHS = EPOCHS
    tfe.EMB_DIM = emb
    log = []
    model = tfe.train(X, y, tr_use, train_ids, dev, rng, log)
    E = tfe.embed_all(model, X, dev)
    pairs, lab = tfe.make_pairs(y, te, np.random.default_rng(0))
    m = tfe.verify_metrics(E, pairs, lab)
    n_params = sum(p.numel() for p in model.parameters() if p.requires_grad)
    return dict(acc=m["acc"], auc=m["auc"], images=int(len(tr_use)),
                params=int(n_params), mb=n_params * 4 / 1e6)


def main() -> int:
    os.makedirs(RUNS, exist_ok=True)
    dev = tfe.device()
    rng = np.random.default_rng(tfe.SEED)
    X, y, names, tr, te, train_ids, test_ids = tfe.load_identity_split(rng)
    print(f"device {dev} · {len(tr)} train images / {len(train_ids)} identities "
          f"· {len(te)} test images / {len(test_ids)} identities\n")

    out = dict(epochs=EPOCHS, seed=tfe.SEED,
               train_images=int(len(tr)), test_identities=int(len(test_ids)))

    print("data scaling")
    ds = []
    for fr in DATA_FRACS:
        t0 = time.time()
        r = one_run(X, y, tr, te, train_ids, dev, tfe.SEED, frac=fr, emb=128)
        r["frac"] = fr
        ds.append(r)
        print(f"  {fr*100:5.0f}%  {r['images']:5d} images   "
              f"acc {r['acc']:.3f}  auc {r['auc']:.3f}   ({time.time()-t0:.0f}s)")
    out["data_scaling"] = ds

    print("\nembedding width")
    es = []
    for e in EMB_DIMS:
        t0 = time.time()
        r = one_run(X, y, tr, te, train_ids, dev, tfe.SEED, frac=1.0, emb=e)
        r["emb"] = e
        es.append(r)
        print(f"  {e:4d}-d  {r['mb']:5.2f} MB   acc {r['acc']:.3f}  "
              f"auc {r['auc']:.3f}   ({time.time()-t0:.0f}s)")
    out["embedding_width"] = es

    with open(os.path.join(RUNS, "ablations.json"), "w") as f:
        json.dump(out, f, indent=2)
    print(f"\nwrote ml/runs/ablations.json")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
