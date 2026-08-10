"""Face detection and identity embedding (InsightFace buffalo_l, ONNX).

Two graphs:

  det_10g.onnx    SCRFD — anchor-free face detector, 3 strides (8/16/32),
                  2 anchors per cell, emits box distances *and* 5 landmarks.
  w600k_r50.onnx  ArcFace R50 — 512-d embedding of a 112x112 aligned crop.

The alignment step between them is not optional. ArcFace is trained on faces
warped onto a fixed 5-point template; feeding it a raw crop costs roughly half
the recognition accuracy, and the failure is silent — you just get a gallery
that never quite matches.
"""
from __future__ import annotations

import cv2
import numpy as np

from .config import FaceConfig
from .onnxbase import Session, make_session, nms

# ArcFace canonical landmark template for a 112x112 crop:
# left eye, right eye, nose tip, left mouth corner, right mouth corner.
ARCFACE_TEMPLATE = np.array([
    [38.2946, 51.6963],
    [73.5318, 51.5014],
    [56.0252, 71.7366],
    [41.5493, 92.3655],
    [70.7299, 92.2041],
], dtype=np.float32)


class Face:
    __slots__ = ("xyxy", "score", "kps", "embedding")

    def __init__(self, xyxy: np.ndarray, score: float, kps: np.ndarray):
        self.xyxy = xyxy.astype(np.float32)
        self.score = float(score)
        self.kps = kps.astype(np.float32).reshape(5, 2)
        self.embedding: np.ndarray | None = None

    @property
    def w(self) -> float:
        return float(self.xyxy[2] - self.xyxy[0])

    @property
    def h(self) -> float:
        return float(self.xyxy[3] - self.xyxy[1])

    def __repr__(self) -> str:                            # pragma: no cover
        return f"<Face {self.w:.0f}x{self.h:.0f} {self.score:.2f}>"


# ------------------------------------------------------------------ geometry --

def umeyama(src: np.ndarray, dst: np.ndarray) -> np.ndarray:
    """Least-squares similarity transform src -> dst (Umeyama 1991).

    Closed form, deterministic, and exact for the 5-point case. Preferred over
    cv2.estimateAffinePartial2D, whose RANSAC/LMEDS path can return a slightly
    different matrix run to run — which would make the same face produce
    different embeddings on different frames.
    """
    n = src.shape[0]
    src_mean = src.mean(axis=0)
    dst_mean = dst.mean(axis=0)
    src_d = src - src_mean
    dst_d = dst - dst_mean

    cov = dst_d.T @ src_d / n
    d = np.ones(2, dtype=np.float64)
    if np.linalg.det(cov) < 0:
        d[1] = -1.0

    u, s, vt = np.linalg.svd(cov)
    if np.linalg.matrix_rank(cov) == 1:
        # Degenerate (collinear landmarks) — fall back to a pure rotation.
        if np.linalg.det(u) * np.linalg.det(vt) > 0:
            rot = u @ vt
        else:
            rot = u @ np.diag(d) @ vt
    else:
        rot = u @ np.diag(d) @ vt

    var_src = src_d.var(axis=0).sum()
    scale = 1.0 if var_src < 1e-12 else float((s * d).sum() / var_src)

    out = np.eye(3, dtype=np.float32)
    out[:2, :2] = scale * rot
    out[:2, 2] = dst_mean - scale * rot @ src_mean
    return out[:2]


def align_face(frame: np.ndarray, kps: np.ndarray, size: int = 112) -> np.ndarray:
    template = ARCFACE_TEMPLATE
    if size != 112:
        template = template * (size / 112.0)
    m = umeyama(kps.astype(np.float64), template.astype(np.float64))
    return cv2.warpAffine(frame, m, (size, size), borderValue=0.0)


def _distance2bbox(points: np.ndarray, dist: np.ndarray) -> np.ndarray:
    return np.stack([points[:, 0] - dist[:, 0], points[:, 1] - dist[:, 1],
                     points[:, 0] + dist[:, 2], points[:, 1] + dist[:, 3]],
                    axis=-1)


def _distance2kps(points: np.ndarray, dist: np.ndarray) -> np.ndarray:
    out = []
    for i in range(0, dist.shape[1], 2):
        out.append(points[:, 0] + dist[:, i])
        out.append(points[:, 1] + dist[:, i + 1])
    return np.stack(out, axis=-1)


# ------------------------------------------------------------------ detector --

class FaceAnalyzer:
    """SCRFD detect -> align -> ArcFace embed."""

    STRIDES = (8, 16, 32)
    NUM_ANCHORS = 2

    def __init__(self, cfg: FaceConfig, providers):
        self.cfg = cfg
        self.det: Session = make_session(cfg.det_model, providers)
        self.rec: Session = make_session(cfg.rec_model, providers)
        self.det_size = cfg.det_size
        self.roi_size = max(64, (cfg.roi_size // 32) * 32)
        self._anchor_cache: dict[tuple[int, int, int], np.ndarray] = {}
        self.det.warmup((1, 3, self.det_size, self.det_size))
        self.rec.warmup((1, 3, 112, 112))
        self.providers = providers

        # The ROI pass runs on CPU, deliberately. SCRFD's graph advertises a
        # dynamic input, but measured on this machine:
        #
        #     CPU     640px  36.2ms      CoreML  640px  31.1ms
        #     CPU     320px  11.0ms      CoreML  320px  FAIL
        #     CPU     192px   5.7ms      CoreML  192px  FAIL
        #
        # CoreML compiles the graph against 640-based constants and rejects
        # every other size ("static output shape ... different ranks"), no
        # matter how fresh the session. CPU handles all of them — and CPU at
        # 192px is 5.4x faster than CoreML at 640px, which is the whole point.
        # Built lazily, so use_roi=False costs nothing.
        self._det_roi: Session | None = None

    def _roi_session(self) -> Session:
        if self._det_roi is None:
            self._det_roi = make_session(self.cfg.det_model,
                                         ["CPUExecutionProvider"])
            self._det_roi.warmup((1, 3, self.roi_size, self.roi_size))
        return self._det_roi

    @property
    def last_ms(self) -> float:
        det_ms = self.det.last_ms if self._det_roi is None \
            else self._det_roi.last_ms
        return det_ms + self.rec.last_ms

    # ------------------------------------------------------------- anchors --
    def _anchors(self, h: int, w: int, stride: int) -> np.ndarray:
        key = (h, w, stride)
        cached = self._anchor_cache.get(key)
        if cached is not None:
            return cached
        ys, xs = np.mgrid[:h, :w].astype(np.float32)
        pts = np.stack([xs, ys], axis=-1) * stride
        pts = pts.reshape(-1, 2)
        if self.NUM_ANCHORS > 1:
            pts = np.repeat(pts, self.NUM_ANCHORS, axis=0)
        self._anchor_cache[key] = pts
        return pts

    # ---------------------------------------------------------- roi detect --
    def detect_in_rois(self, frame: np.ndarray, rois, size: int | None = None):
        """Run detection only inside given regions, at a reduced input size.

        Face detection on a full 1280x720 frame is the most expensive stage in
        the pipeline. But once tracks exist, the only places a relevant face
        can be are their heads — so scan those instead. SCRFD's graph has a
        dynamic input, so a 192px square costs roughly (192/640)^2, about a
        tenth of the full-frame pass.

        Every track's head is scanned, not just the locked one. Restricting it
        to the target would save a little more and quietly disable the
        identity-transfer check, which needs to see other people's faces to
        notice the lock is on the wrong person.

        Returns faces in FRAME coordinates, deduplicated across overlapping
        regions.
        """
        out: list[Face] = []
        for (rx1, ry1, rx2, ry2) in rois:
            rx1, ry1 = max(0, int(rx1)), max(0, int(ry1))
            rx2 = min(frame.shape[1], int(rx2))
            ry2 = min(frame.shape[0], int(ry2))
            if rx2 - rx1 < 16 or ry2 - ry1 < 16:
                continue

            crop = frame[ry1:ry2, rx1:rx2]
            for face in self._detect(crop, self._roi_session(),
                                     size or self.roi_size):
                face.xyxy = face.xyxy + np.array([rx1, ry1, rx1, ry1],
                                                 dtype=np.float32)
                face.kps = face.kps + np.array([rx1, ry1], dtype=np.float32)
                out.append(face)

        # Regions around neighbouring people can overlap, so the same face can
        # come back twice with slightly different boxes.
        if len(out) > 1:
            boxes = np.stack([f.xyxy for f in out])
            scores = np.array([f.score for f in out], dtype=np.float32)
            out = [out[i] for i in nms(boxes, scores, 0.4)]
        return out

    @staticmethod
    def head_roi(xyxy, frame_w: int, frame_h: int, pad: float = 0.35):
        """The region of a person box where their face can be.

        Generous on purpose: the box comes from a tracker that may be coasting
        on a prediction, and a tight crop that misses the head produces a
        confident "no face here" rather than an obvious error.
        """
        x1, y1, x2, y2 = (float(v) for v in xyxy)
        bw, bh = x2 - x1, y2 - y1
        # Head occupies roughly the top eighth of a standing person, but the
        # box may be of someone seated, crouching or partly occluded.
        hw = max(bw * (0.5 + pad), 40.0)
        cx = (x1 + x2) * 0.5
        return (cx - hw, y1 - bh * 0.08,
                cx + hw, y1 + max(bh * 0.42, 60.0))

    # -------------------------------------------------------------- detect --
    def detect(self, frame: np.ndarray) -> list[Face]:
        """Full-frame detection at the configured detector size."""
        return self._detect(frame, self.det, self.det_size)

    def _detect(self, frame: np.ndarray, session: Session,
                size: int) -> list[Face]:
        cfg = self.cfg
        # SCRFD's strides are 8/16/32, so the input must divide by 32 or the
        # anchor grid and the feature maps disagree.
        size = max(64, (int(size) // 32) * 32)
        fh, fw = frame.shape[:2]

        # SCRFD wants the image top-left anchored in a square canvas — not
        # centred like YOLO's letterbox — because its anchor grid is absolute.
        scale = min(size / fw, size / fh)
        nw, nh = int(round(fw * scale)), int(round(fh * scale))
        canvas = np.zeros((size, size, 3), dtype=np.uint8)
        canvas[:nh, :nw] = cv2.resize(frame, (nw, nh))

        blob = cv2.dnn.blobFromImage(canvas, 1.0 / 128.0, (size, size),
                                     (127.5, 127.5, 127.5), swapRB=True)
        out = session.run1(blob.astype(np.float32))

        fmc = len(self.STRIDES)
        boxes_all, scores_all, kps_all = [], [], []

        for i, stride in enumerate(self.STRIDES):
            scores = out[i].reshape(-1)
            bbox = out[i + fmc].reshape(-1, 4) * stride
            kps = out[i + fmc * 2].reshape(-1, 10) * stride

            hw = size // stride
            points = self._anchors(hw, hw, stride)
            if points.shape[0] != scores.shape[0]:
                # Graph disagrees with our anchor assumption — rebuild from
                # the actual row count rather than emitting shifted boxes.
                n = scores.shape[0] // self.NUM_ANCHORS
                side = int(round(n ** 0.5))
                points = self._anchors(side, side, stride)
                if points.shape[0] != scores.shape[0]:
                    continue

            keep = scores >= cfg.det_thres
            if not np.any(keep):
                continue

            pts = points[keep]
            boxes_all.append(_distance2bbox(pts, bbox[keep]))
            kps_all.append(_distance2kps(pts, kps[keep]))
            scores_all.append(scores[keep])

        if not boxes_all:
            return []

        boxes = np.concatenate(boxes_all, axis=0) / scale
        kpts = np.concatenate(kps_all, axis=0) / scale
        scores = np.concatenate(scores_all, axis=0)

        idx = nms(boxes, scores, cfg.nms_iou)
        faces = []
        for i in idx:
            b = boxes[i]
            if min(b[2] - b[0], b[3] - b[1]) < cfg.min_face_px:
                continue
            b = np.array([max(0.0, b[0]), max(0.0, b[1]),
                          min(fw - 1.0, b[2]), min(fh - 1.0, b[3])],
                         dtype=np.float32)
            faces.append(Face(b, float(scores[i]), kpts[i]))
        return faces

    # --------------------------------------------------------------- embed --
    def embed(self, frame: np.ndarray, face: Face) -> np.ndarray:
        crop = align_face(frame, face.kps, 112)
        blob = cv2.dnn.blobFromImage(crop, 1.0 / 127.5, (112, 112),
                                     (127.5, 127.5, 127.5), swapRB=True)
        vec = self.rec.run1(blob.astype(np.float32))[0].reshape(-1)
        norm = float(np.linalg.norm(vec))
        vec = vec / norm if norm > 1e-9 else vec
        face.embedding = vec.astype(np.float32)
        return face.embedding

    def detect_and_embed(self, frame: np.ndarray) -> list[Face]:
        faces = self.detect(frame)
        for f in faces:
            self.embed(frame, f)
        return faces


# --------------------------------------------------------------- association --

def assign_faces_to_people(people, faces) -> dict[int, int]:
    """Map person index -> face index.

    A face belongs to the person box that contains its centre. Where several
    boxes qualify (a crowd, or someone standing behind someone else) the
    smallest wins, since the tighter box is the nearer person.
    """
    out: dict[int, int] = {}
    used: set[int] = set()

    for fi, face in enumerate(faces):
        fcx = (face.xyxy[0] + face.xyxy[2]) * 0.5
        fcy = (face.xyxy[1] + face.xyxy[3]) * 0.5

        best, best_area = -1, float("inf")
        for pi, person in enumerate(people):
            if pi in out:
                continue
            x1, y1, x2, y2 = person.xyxy
            if x1 <= fcx <= x2 and y1 <= fcy <= y2:
                area = (x2 - x1) * (y2 - y1)
                if area < best_area:
                    best, best_area = pi, area

        if best >= 0 and fi not in used:
            out[best] = fi
            used.add(fi)

    return out
