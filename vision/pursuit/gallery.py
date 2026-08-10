"""Enrolled identities on disk.

One `.npz` per person under `targets/`, holding a stack of L2-normalised
ArcFace vectors. Several vectors per person, not one: a single enrolment frame
locks in one pose and one lighting condition, and the drone will see neither
again. Ten frames across a few angles roughly halves the miss rate.

Matching takes the mean of the top-k similarities rather than the single best.
A lone outlier vector in the gallery — one frame where the subject blinked or
someone walked behind them — can otherwise produce a spuriously high max
against a stranger.
"""
from __future__ import annotations

import os
import time

import numpy as np


class Identity:
    def __init__(self, name: str, vectors: np.ndarray,
                 created: float | None = None,
                 height_m: float | None = None):
        if vectors.ndim != 2 or vectors.shape[1] == 0:
            raise ValueError(f"{name}: expected (N, D) embeddings, "
                             f"got {vectors.shape}")
        self.name = name
        self.vectors = vectors.astype(np.float32)
        self.created = created if created is not None else time.time()
        # Optional. With it the follow distance is a real number of metres via
        # the pinhole relation; without it the controller falls back to holding
        # a fraction of frame height, which works but is not a physical range.
        self.height_m = height_m

    def __len__(self) -> int:
        return int(self.vectors.shape[0])

    def similarity(self, embedding: np.ndarray, top_k: int = 3) -> float:
        """Cosine similarity in [-1, 1]; both sides are already unit-norm."""
        if embedding is None or len(self) == 0:
            return -1.0
        sims = self.vectors @ embedding.astype(np.float32)
        k = min(top_k, sims.shape[0])
        return float(np.sort(sims)[-k:].mean())

    def __repr__(self) -> str:                            # pragma: no cover
        return f"<Identity {self.name!r} n={len(self)}>"


class Gallery:
    def __init__(self, directory: str):
        self.directory = directory
        self.identities: dict[str, Identity] = {}
        os.makedirs(directory, exist_ok=True)

    # ------------------------------------------------------------- storage --
    def path_for(self, name: str) -> str:
        safe = "".join(c for c in name if c.isalnum() or c in "-_").strip()
        if not safe:
            raise ValueError(f"unusable identity name: {name!r}")
        return os.path.join(self.directory, f"{safe}.npz")

    def save(self, name: str, vectors: np.ndarray,
             height_m: float | None = None) -> str:
        path = self.path_for(name)
        np.savez_compressed(path, name=name, vectors=vectors.astype(np.float32),
                            created=time.time(),
                            height_m=float(height_m) if height_m else 0.0)
        self.identities[name] = Identity(name, vectors, height_m=height_m)
        return path

    def load_all(self) -> int:
        self.identities.clear()
        if not os.path.isdir(self.directory):
            return 0
        for fn in sorted(os.listdir(self.directory)):
            if not fn.endswith(".npz"):
                continue
            try:
                data = np.load(os.path.join(self.directory, fn),
                               allow_pickle=False)
                name = str(data["name"])
                # `height_m` post-dates the first galleries; 0.0 and absent
                # both mean "not recorded", so old files keep loading.
                height = float(data["height_m"]) if "height_m" in data else 0.0
                ident = Identity(name, data["vectors"],
                                 float(data["created"]) if "created" in data
                                 else None,
                                 height_m=height if height > 0.0 else None)
                self.identities[name] = ident
            except Exception as exc:                      # pragma: no cover
                print(f"  warn: skipping {fn}: {exc}")
        return len(self.identities)

    # ------------------------------------------------------------- matching --
    def best_match(self, embedding: np.ndarray,
                   restrict: str | None = None) -> tuple[str | None, float]:
        """Return (name, similarity) for the closest enrolled identity.

        `restrict` pins matching to one person. In flight that is the normal
        case — you are following a specific friend, and comparing against every
        enrolled face just adds ways to be wrong.
        """
        if embedding is None:
            return None, -1.0

        pool = self.identities
        if restrict is not None:
            ident = pool.get(restrict)
            if ident is None:
                return None, -1.0
            return restrict, ident.similarity(embedding)

        best_name, best_score = None, -1.0
        for name, ident in pool.items():
            score = ident.similarity(embedding)
            if score > best_score:
                best_name, best_score = name, score
        return best_name, best_score

    def names(self) -> list[str]:
        return sorted(self.identities)

    def __len__(self) -> int:
        return len(self.identities)

    def __contains__(self, name: str) -> bool:
        return name in self.identities
