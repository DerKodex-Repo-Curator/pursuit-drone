"""Learned person re-identification — the appearance channel that replaces
a colour histogram.

The histogram in `body.py` answers "same coloured blob as before?". That is
enough to bridge a few frames while the face is turned away, and no more: two
people in dark jackets are indistinguishable to it. It is why the face veto
exists — to catch the failure after it has already happened.

This answers the real question. A 768-d embedding trained for person re-ID
encodes build, clothing structure and texture, not just the average colour of
someone's torso, and it survives the pose and lighting changes that a
histogram does not.

Measured on this machine: 1.1ms per crop on CoreML, 12.4ms on CPU — cheap
enough to run on every track at the face cadence, and it batches.
"""
from __future__ import annotations

import os

import cv2
import numpy as np

from .config import BodyConfig
from .onnxbase import Session, make_session

# ImageNet normalisation, which is what the OpenCV Zoo youtu-reid graph was
# trained with. Getting these wrong does not error — it quietly produces
# embeddings that cluster by lighting instead of by person.
_MEAN = np.array([0.485, 0.456, 0.406], dtype=np.float32) * 255.0
_STD = np.array([0.229, 0.224, 0.225], dtype=np.float32) * 255.0

INPUT_W, INPUT_H = 128, 256      # the aspect ratio person ReID is trained on


class PersonReID:
    def __init__(self, model_path: str, providers):
        self.session: Session = make_session(model_path, providers)
        self.session.warmup((1, 3, INPUT_H, INPUT_W))
        self.dim = 768

    @property
    def last_ms(self) -> float:
        return self.session.last_ms

    # ----------------------------------------------------------- preprocess --
    @staticmethod
    def _crop(frame: np.ndarray, xyxy) -> np.ndarray | None:
        h, w = frame.shape[:2]
        x1, y1, x2, y2 = (int(round(float(v))) for v in xyxy)
        x1, y1 = max(0, x1), max(0, y1)
        x2, y2 = min(w, x2), min(h, y2)
        if x2 - x1 < 8 or y2 - y1 < 16:
            return None
        return frame[y1:y2, x1:x2]

    def _blob(self, crops: list[np.ndarray]) -> np.ndarray:
        out = np.empty((len(crops), 3, INPUT_H, INPUT_W), dtype=np.float32)
        for i, crop in enumerate(crops):
            # Stretched to the fixed aspect rather than letterboxed: the model
            # was trained on stretched crops, so padding to preserve aspect
            # would put it off-distribution.
            img = cv2.resize(crop, (INPUT_W, INPUT_H),
                             interpolation=cv2.INTER_LINEAR)
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB).astype(np.float32)
            img = (img - _MEAN) / _STD
            out[i] = img.transpose(2, 0, 1)
        return out

    # --------------------------------------------------------------- embed --
    def embed_many(self, frame: np.ndarray, boxes) -> list[np.ndarray | None]:
        """Embed several person boxes in one inference.

        Batched because the per-call overhead dominates at this model size —
        four separate calls cost far more than one call with four crops.
        """
        crops, index = [], []
        for i, box in enumerate(boxes):
            crop = self._crop(frame, box)
            if crop is not None:
                crops.append(crop)
                index.append(i)

        results: list[np.ndarray | None] = [None] * len(boxes)
        if not crops:
            return results

        raw = self.session.run1(self._blob(crops))[0]
        vecs = np.asarray(raw, dtype=np.float32).reshape(len(crops), -1)
        norms = np.linalg.norm(vecs, axis=1, keepdims=True)
        vecs = vecs / np.maximum(norms, 1e-9)

        for slot, vec in zip(index, vecs):
            results[slot] = vec
        return results

    def embed(self, frame: np.ndarray, xyxy) -> np.ndarray | None:
        return self.embed_many(frame, [xyxy])[0]


def similarity(a: np.ndarray | None, b: np.ndarray | None) -> float:
    """Cosine similarity on L2-normalised embeddings."""
    if a is None or b is None:
        return 0.0
    return float(np.dot(a, b))


def load(cfg: BodyConfig, providers) -> PersonReID | None:
    """Build the model, or return None so the caller falls back.

    Absence is not an error: the colour histogram still works, just less well,
    and a missing 102MB download should not stop the drone from flying.
    """
    if not cfg.use_reid:
        return None
    if not os.path.exists(cfg.reid_model):
        print(f"  reid      not found at {os.path.basename(cfg.reid_model)} "
              f"— falling back to the colour histogram")
        print(f"            run: python fetch_models.py --reid")
        return None
    try:
        return PersonReID(cfg.reid_model, providers)
    except Exception as exc:                              # pragma: no cover
        print(f"  reid      failed to load ({exc}) — using colour histogram")
        return None
