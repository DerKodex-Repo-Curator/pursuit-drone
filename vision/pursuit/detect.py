"""Person detector — YOLO11n, ONNX, person class only.

YOLO11's exported graph emits `(1, 84, N)`: rows 0-3 are the box as
centre-x, centre-y, width, height in letterboxed pixels, rows 4-83 are the 80
COCO class scores (already sigmoid'd, no separate objectness — that is the v8+
anchor-free head). We slice the single row we care about instead of taking an
argmax over 80 classes, which removes most of the post-processing cost.
"""
from __future__ import annotations

import numpy as np

from .config import DetectorConfig
from .onnxbase import Session, letterbox, make_session, nms


class Detection:
    """One person box in original-image coordinates."""

    __slots__ = ("xyxy", "score")

    def __init__(self, xyxy: np.ndarray, score: float):
        self.xyxy = xyxy.astype(np.float32)
        self.score = float(score)

    # -- geometry helpers used by the tracker, control loop and HUD --------
    @property
    def cx(self) -> float:
        return float((self.xyxy[0] + self.xyxy[2]) * 0.5)

    @property
    def cy(self) -> float:
        return float((self.xyxy[1] + self.xyxy[3]) * 0.5)

    @property
    def w(self) -> float:
        return float(self.xyxy[2] - self.xyxy[0])

    @property
    def h(self) -> float:
        return float(self.xyxy[3] - self.xyxy[1])

    @property
    def area(self) -> float:
        return self.w * self.h

    def __repr__(self) -> str:                            # pragma: no cover
        return (f"<Detection {self.xyxy.round(0).astype(int).tolist()} "
                f"{self.score:.2f}>")


class PersonDetector:
    def __init__(self, cfg: DetectorConfig, providers):
        self.cfg = cfg
        self.session: Session = make_session(cfg.model, providers)
        self.imgsz = cfg.imgsz
        # Respect a fixed input size baked into the graph at export time.
        shape = self.session.input_shape
        if isinstance(shape, (list, tuple)) and len(shape) == 4:
            if isinstance(shape[2], int) and shape[2] > 0:
                self.imgsz = int(shape[2])
        self.session.warmup((1, 3, self.imgsz, self.imgsz))

    @property
    def last_ms(self) -> float:
        return self.session.last_ms

    # ------------------------------------------------------------------ run --
    def __call__(self, frame: np.ndarray) -> list[Detection]:
        cfg = self.cfg
        padded, scale, dx, dy = letterbox(frame, self.imgsz)

        # BGR->RGB, HWC->CHW, 0-255 -> 0-1.
        blob = padded[:, :, ::-1].transpose(2, 0, 1)[None]
        blob = np.ascontiguousarray(blob, dtype=np.float32) / 255.0

        raw = self.session.run1(blob)[0]

        pred = np.squeeze(raw, axis=0) if raw.ndim == 3 else raw
        # Accept either (84, N) or (N, 84) — exporters disagree, and a wrong
        # guess here silently produces garbage boxes rather than an error.
        if pred.shape[0] < pred.shape[1]:
            pred = pred.T                                  # -> (N, 84)

        scores = pred[:, 4 + cfg.class_id]
        keep = scores >= cfg.conf_thres
        if not np.any(keep):
            return []

        pred = pred[keep]
        scores = scores[keep]

        # cxcywh (letterboxed px) -> xyxy (original px)
        cx, cy, w, h = pred[:, 0], pred[:, 1], pred[:, 2], pred[:, 3]
        boxes = np.stack([cx - w * 0.5, cy - h * 0.5,
                          cx + w * 0.5, cy + h * 0.5], axis=1)
        boxes[:, [0, 2]] -= dx
        boxes[:, [1, 3]] -= dy
        boxes /= scale

        fh, fw = frame.shape[:2]
        np.clip(boxes[:, [0, 2]], 0, fw - 1, out=boxes[:, [0, 2]])
        np.clip(boxes[:, [1, 3]], 0, fh - 1, out=boxes[:, [1, 3]])

        idx = nms(boxes, scores, cfg.nms_iou, cfg.max_det)
        return [Detection(boxes[i], scores[i]) for i in idx]
