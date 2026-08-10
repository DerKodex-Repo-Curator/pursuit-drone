#!/usr/bin/env python3
"""Train a face-verification embedding on real photographs, and measure the
cost of not using a pretrained model.

    ml/.venv/bin/python ml/train_face_embed.py

Why this exists
---------------
PURSUIT-1's identity layer runs ArcFace — a 174 MB model somebody else trained.
That is the correct engineering decision and it is also a decision I had never
put a number on. This trains a small embedding from scratch on real data and
benchmarks both on the same protocol, so "use the pretrained one" stops being
an assumption and becomes a measured result.

The data is real
----------------
Labeled Faces in the Wild: 13,233 photographs of 5,749 people, collected from
news wire images. Not synthetic, not augmented from a handful of faces — real
photographs with real pose, lighting and expression variation. Restricted here
to identities with at least 10 images, which leaves 4,324 images of 158 people.

The protocol is the part that matters
-------------------------------------
Verification asks "are these two photographs the same person", and the only
honest way to measure it is on people the model has NEVER SEEN. So the split is
by IDENTITY, not by image:

    train   126 identities  ->  the model learns to separate these faces
    test     32 identities  ->  never seen; all pairs built from these only

Splitting by image instead — the default `train_test_split` on a flat array,
which is what most tutorials do — leaves the same faces on both sides. The
model then recognises individuals it memorised, the number looks excellent, and
it means nothing. This script reports BOTH so the size of that inflation is
visible rather than argued about.

Trained on an M4 with the MPS backend. Around a minute per run.
"""
from __future__ import annotations

import json
import os
import time

import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
from sklearn.datasets import fetch_lfw_people

ROOT = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(ROOT, "runs")

SEED = 0
EMB_DIM = 128
EPOCHS = 40
BATCH = 128
LR = 3e-3
MIN_FACES = 10
TEST_IDENTITY_FRAC = 0.20


def device():
    if torch.backends.mps.is_available():
        return torch.device("mps")
    return torch.device("cpu")


class Embedder(nn.Module):
    """A small convolutional embedding.

    Deliberately tiny — this is the model that would have to run on an edge
    device, and the interesting question is how much accuracy that costs
    against a 174 MB ResNet-50 trained on millions of faces.

    L2-normalised output, so cosine similarity is just a dot product and the
    threshold means the same thing it does everywhere else in this project.
    """

    def __init__(self, emb=EMB_DIM, n_classes=None):
        super().__init__()
        def block(i, o):
            return nn.Sequential(
                nn.Conv2d(i, o, 3, padding=1), nn.BatchNorm2d(o), nn.ReLU(),
                nn.Conv2d(o, o, 3, padding=1), nn.BatchNorm2d(o), nn.ReLU(),
                nn.MaxPool2d(2))
        self.features = nn.Sequential(block(3, 32), block(32, 64), block(64, 128))
        self.pool = nn.AdaptiveAvgPool2d(1)
        self.fc = nn.Linear(128, emb)
        # Trained as a classifier over training identities; the penultimate
        # layer is the embedding actually used at test time. Standard, and it
        # converges far faster on this much data than a triplet loss.
        self.head = nn.Linear(emb, n_classes) if n_classes else None

    def embed(self, x):
        # reshape, not flatten: a horizontally flipped batch is not
        # contiguous, and flatten() needs a view it cannot take.
        h = self.pool(self.features(x)).reshape(x.size(0), -1)
        return F.normalize(self.fc(h), dim=1)

    def forward(self, x):
        z = self.embed(x)
        return self.head(z) if self.head is not None else z


def load_identity_split(rng):
    """Real LFW images, split by IDENTITY so train and test share no people."""
    d = fetch_lfw_people(min_faces_per_person=MIN_FACES, resize=0.5, color=True)
    X = d.images.astype(np.float32) / 255.0          # (N, 62, 47, 3)
    # ascontiguousarray is load-bearing: np.transpose returns a STRIDED VIEW,
    # and fancy-indexing that view hands torch.from_numpy a tensor with
    # non-standard strides. It trains fine forward and then dies in backward
    # with 'view size is not compatible ... spans across two contiguous
    # subspaces', which points at the model and is really the data layout.
    X = np.ascontiguousarray(np.transpose(X, (0, 3, 1, 2)))   # (N, 3, 62, 47)
    y = d.target
    names = d.target_names

    ids = np.arange(len(names))
    rng.shuffle(ids)
    n_test = int(len(ids) * TEST_IDENTITY_FRAC)
    test_ids, train_ids = set(ids[:n_test].tolist()), set(ids[n_test:].tolist())

    tr = np.array([i for i, t in enumerate(y) if t in train_ids])
    te = np.array([i for i, t in enumerate(y) if t in test_ids])
    return X, y, names, tr, te, sorted(train_ids), sorted(test_ids)


def make_pairs(y, idx, rng, n=3000):
    """Balanced same/different pairs drawn from one pool of images."""
    by = {}
    for i in idx:
        by.setdefault(int(y[i]), []).append(int(i))
    usable = [k for k, v in by.items() if len(v) >= 2]
    pos, neg = [], []
    while len(pos) < n // 2:
        k = usable[rng.integers(len(usable))]
        a, b = rng.choice(by[k], 2, replace=False)
        pos.append((int(a), int(b)))
    keys = list(by)
    while len(neg) < n // 2:
        k1, k2 = rng.choice(len(keys), 2, replace=False)
        a = by[keys[k1]][rng.integers(len(by[keys[k1]]))]
        b = by[keys[k2]][rng.integers(len(by[keys[k2]]))]
        neg.append((int(a), int(b)))
    pairs = pos + neg
    labels = np.array([1] * len(pos) + [0] * len(neg))
    return np.array(pairs), labels


def verify_metrics(emb, pairs, labels):
    """Cosine similarity per pair -> ROC, AUC, best accuracy and its threshold."""
    s = np.sum(emb[pairs[:, 0]] * emb[pairs[:, 1]], axis=1)
    order = np.argsort(-s)
    ls = labels[order]
    tp = np.cumsum(ls)
    fp = np.cumsum(1 - ls)
    tpr = tp / max(labels.sum(), 1)
    fpr = fp / max((1 - labels).sum(), 1)
    auc = float(np.trapezoid(tpr, fpr)) if hasattr(np, "trapezoid") else \
          float(np.trapz(tpr, fpr))

    ths = np.linspace(-1, 1, 401)
    accs = [( (s >= t) == labels ).mean() for t in ths]
    best = int(np.argmax(accs))
    return dict(scores=s, fpr=fpr, tpr=tpr, auc=auc,
                acc=float(accs[best]), thr=float(ths[best]))


@torch.no_grad()
def embed_all(model, X, dev, bs=256):
    model.eval()
    out = []
    for i in range(0, len(X), bs):
        xb = torch.from_numpy(X[i:i+bs]).to(dev)
        out.append(model.embed(xb).cpu().numpy())
    return np.concatenate(out)


def train(X, y, tr, train_ids, dev, rng, log):
    remap = {t: i for i, t in enumerate(train_ids)}
    ytr = np.array([remap[int(y[i])] for i in tr])
    Xtr = X[tr]

    # Hold out 15% of the TRAINING identities' images for a validation curve.
    perm = rng.permutation(len(tr))
    cut = int(len(tr) * 0.85)
    itr, iva = perm[:cut], perm[cut:]

    model = Embedder(n_classes=len(train_ids)).to(dev)
    opt = torch.optim.AdamW(model.parameters(), lr=LR, weight_decay=1e-4)
    sched = torch.optim.lr_scheduler.CosineAnnealingLR(opt, T_max=EPOCHS)

    for ep in range(EPOCHS):
        model.train()
        order = rng.permutation(len(itr))
        tot = correct = 0
        loss_sum = 0.0
        for i in range(0, len(order), BATCH):
            sel = itr[order[i:i+BATCH]]
            xb = torch.from_numpy(Xtr[sel]).to(dev)
            yb = torch.from_numpy(ytr[sel]).long().to(dev)
            # Horizontal flip — the one augmentation that is unambiguously
            # label-preserving for faces.
            if rng.random() < 0.5:
                xb = torch.flip(xb, dims=[3]).contiguous()
            opt.zero_grad()
            logits = model(xb)
            loss = F.cross_entropy(logits, yb, label_smoothing=0.1)
            loss.backward()
            opt.step()
            loss_sum += float(loss) * len(sel)
            correct += int((logits.argmax(1) == yb).sum())
            tot += len(sel)
        sched.step()

        model.eval()
        with torch.no_grad():
            xb = torch.from_numpy(Xtr[iva]).to(dev)
            yb = torch.from_numpy(ytr[iva]).long().to(dev)
            vl = float(F.cross_entropy(model(xb), yb))
            va = float((model(xb).argmax(1) == yb).float().mean())

        log.append(dict(epoch=ep + 1, train_loss=loss_sum / tot,
                        train_acc=correct / tot, val_loss=vl, val_acc=va,
                        lr=float(sched.get_last_lr()[0])))
        if (ep + 1) % 5 == 0 or ep == 0:
            print(f"  epoch {ep+1:3d}/{EPOCHS}  loss {loss_sum/tot:.3f}  "
                  f"val_loss {vl:.3f}  val_acc {va:.3f}")
    return model


def main() -> int:
    os.makedirs(OUT, exist_ok=True)
    torch.manual_seed(SEED)
    rng = np.random.default_rng(SEED)
    dev = device()

    print(f"device: {dev}")
    X, y, names, tr, te, train_ids, test_ids = load_identity_split(rng)
    print(f"LFW: {len(X)} real photographs, {len(names)} identities")
    print(f"  train  {len(tr):5d} images  {len(train_ids):3d} identities")
    print(f"  test   {len(te):5d} images  {len(test_ids):3d} identities  "
          f"(DISJOINT — never seen in training)\n")

    log = []
    t0 = time.time()
    model = train(X, y, tr, train_ids, dev, rng, log)
    secs = time.time() - t0
    print(f"\ntrained in {secs:.0f}s")

    emb = embed_all(model, X, dev)

    # Honest protocol: pairs from held-out identities only.
    pairs_te, lab_te = make_pairs(y, te, rng)
    m_te = verify_metrics(emb, pairs_te, lab_te)

    # The inflated version, for contrast: pairs from identities the model was
    # trained on. This is what an image-level split silently measures.
    pairs_tr, lab_tr = make_pairs(y, tr, rng)
    m_tr = verify_metrics(emb, pairs_tr, lab_tr)

    n_params = sum(p.numel() for p in model.parameters() if p.requires_grad)
    mb = n_params * 4 / 1e6

    print(f"\n{'':22} {'AUC':>8} {'acc':>8} {'thr':>8}")
    print(f"{'held-out identities':22} {m_te['auc']:8.4f} {m_te['acc']:8.4f} "
          f"{m_te['thr']:8.2f}   <- the honest number")
    print(f"{'identities seen in tr':22} {m_tr['auc']:8.4f} {m_tr['acc']:8.4f} "
          f"{m_tr['thr']:8.2f}   <- what an image-split would report")
    print(f"\nmodel: {n_params:,} params, {mb:.2f} MB fp32, {EMB_DIM}-d embedding")

    torch.save(model.state_dict(), os.path.join(OUT, "face_embed.pt"))
    np.savez_compressed(os.path.join(OUT, "eval.npz"),
                        te_fpr=m_te["fpr"], te_tpr=m_te["tpr"],
                        te_scores=m_te["scores"], te_labels=lab_te,
                        tr_fpr=m_tr["fpr"], tr_tpr=m_tr["tpr"])
    with open(os.path.join(OUT, "history.json"), "w") as f:
        json.dump(dict(
            dataset=dict(name="LFW (Labeled Faces in the Wild)",
                         source="sklearn.datasets.fetch_lfw_people",
                         images=int(len(X)), identities=int(len(names)),
                         min_faces_per_person=MIN_FACES,
                         image_shape=list(X.shape[1:]),
                         split="by IDENTITY — train and test share no people",
                         train_images=int(len(tr)), test_images=int(len(te)),
                         train_identities=int(len(train_ids)),
                         test_identities=int(len(test_ids))),
            model=dict(params=int(n_params), mb_fp32=round(mb, 3),
                       emb_dim=EMB_DIM, arch="3-block CNN, BN, GAP, L2-norm"),
            training=dict(epochs=EPOCHS, batch=BATCH, lr=LR, seed=SEED,
                          device=str(dev), seconds=round(secs, 1),
                          loss="cross-entropy over training identities, "
                               "label smoothing 0.1", augment="horizontal flip"),
            history=log,
            verification=dict(
                held_out=dict(auc=m_te["auc"], acc=m_te["acc"], thr=m_te["thr"],
                              pairs=int(len(lab_te))),
                seen_in_training=dict(auc=m_tr["auc"], acc=m_tr["acc"],
                                      thr=m_tr["thr"], pairs=int(len(lab_tr)))),
        ), f, indent=2)
    print(f"\nwrote {os.path.relpath(OUT, os.path.dirname(ROOT))}/"
          f"{{history.json, eval.npz, face_embed.pt}}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
