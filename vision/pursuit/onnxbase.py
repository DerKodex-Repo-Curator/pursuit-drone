"""ONNX Runtime session factory.

The whole pipeline is ONNX — no torch, no ultralytics. That is partly forced
(this machine runs Python 3.14, which torch has no wheels for) and partly the
better engineering: ORT's CoreML provider puts the detector on the Apple
Neural Engine, the install is ~40MB instead of ~2.5GB, and the same graph
files run unchanged on a Jetson or a Pi later.

CoreML is *requested*, never assumed. Some operators fall back to CPU and some
ORT builds reject the provider outright, so `make_session` degrades quietly and
reports what it actually got in `.provider`.
"""
from __future__ import annotations

import os
import time
from typing import Sequence

import numpy as np
import onnxruntime as ort


class Session:
    """A loaded ONNX graph plus the bookkeeping the pipeline wants."""

    def __init__(self, path: str, providers: Sequence[str]):
        if not os.path.exists(path):
            raise FileNotFoundError(
                f"model not found: {path}\n"
                f"  run:  python fetch_models.py")

        opts = ort.SessionOptions()
        opts.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
        # Camera work is latency-bound, not throughput-bound: one frame at a
        # time, as fast as possible. Sequential execution avoids thread
        # thrash against the capture and control threads.
        opts.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
        opts.log_severity_level = 3          # errors only

        available = set(ort.get_available_providers())
        wanted = [p for p in providers if p in available]
        if "CPUExecutionProvider" not in wanted:
            wanted.append("CPUExecutionProvider")

        self.sess: ort.InferenceSession | None = None
        last_err: Exception | None = None
        # Walk the preference list down to bare CPU, so an unusable accelerator
        # costs a warning rather than the whole pipeline.
        for i in range(len(wanted)):
            try:
                self.sess = ort.InferenceSession(path, sess_options=opts,
                                                 providers=wanted[i:])
                break
            except Exception as exc:                      # pragma: no cover
                last_err = exc
        if self.sess is None:                             # pragma: no cover
            raise RuntimeError(f"could not load {path}: {last_err}")

        self.path = path
        self.provider = self.sess.get_providers()[0]
        self.inputs = [i.name for i in self.sess.get_inputs()]
        self.outputs = [o.name for o in self.sess.get_outputs()]
        self.input_shape = self.sess.get_inputs()[0].shape
        self.last_ms = 0.0

    def run(self, feed: dict[str, np.ndarray]) -> list[np.ndarray]:
        t0 = time.perf_counter()
        out = self.sess.run(self.outputs, feed)
        self.last_ms = (time.perf_counter() - t0) * 1e3
        return out

    def run1(self, array: np.ndarray) -> list[np.ndarray]:
        """Single-input convenience."""
        return self.run({self.inputs[0]: array})

    def warmup(self, shape: Sequence[int], n: int = 2) -> float:
        """Force graph compilation before the first real frame.

        CoreML compiles on first inference and that can take a second or more.
        Paying it during startup keeps it out of the control loop.
        """
        dummy = np.zeros(tuple(shape), dtype=np.float32)
        for _ in range(n):
            self.run1(dummy)
        return self.last_ms

    def __repr__(self) -> str:                            # pragma: no cover
        return (f"<Session {os.path.basename(self.path)} "
                f"{self.provider.replace('ExecutionProvider','')} "
                f"{self.last_ms:.1f}ms>")


def make_session(path: str, providers: Sequence[str]) -> Session:
    return Session(path, providers)


# ------------------------------------------------------------------ helpers --

def letterbox(img: np.ndarray, size: int,
              pad_value: int = 114) -> tuple[np.ndarray, float, int, int]:
    """Resize preserving aspect ratio, pad to a square.

    Returns the padded image plus the scale and offsets needed to map boxes
    back to original-image coordinates. Getting this inverse wrong is the
    single most common source of "the boxes are shifted" bugs, so the mapping
    lives here next to the forward transform rather than in the detector.
    """
    import cv2

    h, w = img.shape[:2]
    scale = min(size / w, size / h)
    nw, nh = int(round(w * scale)), int(round(h * scale))
    resized = cv2.resize(img, (nw, nh), interpolation=cv2.INTER_LINEAR)

    out = np.full((size, size, 3), pad_value, dtype=img.dtype)
    dx, dy = (size - nw) // 2, (size - nh) // 2
    out[dy:dy + nh, dx:dx + nw] = resized
    return out, scale, dx, dy


def nms(boxes: np.ndarray, scores: np.ndarray, iou_thres: float,
        max_det: int = 300) -> np.ndarray:
    """Greedy NMS on xyxy boxes. Returns kept indices, best score first."""
    if len(boxes) == 0:
        return np.empty((0,), dtype=np.int64)

    x1, y1, x2, y2 = boxes[:, 0], boxes[:, 1], boxes[:, 2], boxes[:, 3]
    areas = np.maximum(0.0, x2 - x1) * np.maximum(0.0, y2 - y1)
    order = scores.argsort()[::-1]

    keep: list[int] = []
    while order.size > 0 and len(keep) < max_det:
        i = order[0]
        keep.append(int(i))
        if order.size == 1:
            break
        rest = order[1:]
        xx1 = np.maximum(x1[i], x1[rest])
        yy1 = np.maximum(y1[i], y1[rest])
        xx2 = np.minimum(x2[i], x2[rest])
        yy2 = np.minimum(y2[i], y2[rest])
        inter = np.maximum(0.0, xx2 - xx1) * np.maximum(0.0, yy2 - yy1)
        union = areas[i] + areas[rest] - inter
        iou = np.where(union > 0, inter / np.maximum(union, 1e-9), 0.0)
        order = rest[iou <= iou_thres]

    return np.asarray(keep, dtype=np.int64)


def iou_matrix(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Pairwise IoU between two sets of xyxy boxes -> (len(a), len(b))."""
    if len(a) == 0 or len(b) == 0:
        return np.zeros((len(a), len(b)), dtype=np.float32)

    area_a = np.maximum(0.0, a[:, 2] - a[:, 0]) * np.maximum(0.0, a[:, 3] - a[:, 1])
    area_b = np.maximum(0.0, b[:, 2] - b[:, 0]) * np.maximum(0.0, b[:, 3] - b[:, 1])

    lt = np.maximum(a[:, None, :2], b[None, :, :2])
    rb = np.minimum(a[:, None, 2:4], b[None, :, 2:4])
    wh = np.clip(rb - lt, 0.0, None)
    inter = wh[..., 0] * wh[..., 1]

    union = area_a[:, None] + area_b[None, :] - inter
    return (inter / np.maximum(union, 1e-9)).astype(np.float32)
