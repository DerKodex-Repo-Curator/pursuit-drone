#!/usr/bin/env python3
"""How much does the real camera cost us?

Everything in the vision stack was validated on a clean 1280x720 laptop
webcam. The drone's OV2640 delivers something quite different: lower
resolution, JPEG artefacts at quality ~12, rolling-shutter shear while the
airframe rotates, motion blur, and exposure that hunts when sky enters frame.

This applies those degradations to known-good images and measures what
actually survives — so the resolution and threshold choices are set from
numbers rather than from my judgement about what "should" be fine.

    python tools/degradation_sweep.py
    python tools/degradation_sweep.py --image tests/data/zidane.jpg
"""
from __future__ import annotations

import argparse
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from pursuit.config import Config                                # noqa: E402
from pursuit.detect import PersonDetector                        # noqa: E402
from pursuit.face import FaceAnalyzer                            # noqa: E402

RESOLUTIONS = [("QQVGA", 160, 120), ("QVGA", 320, 240),
                ("VGA", 640, 480), ("SVGA", 800, 600),
                ("XGA", 1024, 768), ("HD", 1280, 720)]


def jpeg_cycle(img: np.ndarray, quality: int) -> np.ndarray:
    """Encode and decode, exactly as the frame will travel over WiFi."""
    ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, quality])
    return cv2.imdecode(buf, cv2.IMREAD_COLOR) if ok else img


def motion_blur(img: np.ndarray, px: int) -> np.ndarray:
    """Horizontal smear, as produced by yaw during the exposure."""
    if px < 2:
        return img
    k = np.zeros((px, px), np.float32)
    k[px // 2, :] = 1.0 / px
    return cv2.filter2D(img, -1, k)


def rolling_shutter(img: np.ndarray, shear_px: float) -> np.ndarray:
    """Rows are exposed at different times, so rotation shears the frame.

    A global-shutter webcam has none of this; a cheap CMOS sensor on a
    rotating airframe has a lot, and it distorts exactly the vertical edges a
    person detector keys on.
    """
    if abs(shear_px) < 0.5:
        return img
    h, w = img.shape[:2]
    out = np.empty_like(img)
    for y in range(h):
        dx = int(round(shear_px * (y / h - 0.5) * 2))
        out[y] = np.roll(img[y], dx, axis=0)
    return out


def exposure(img: np.ndarray, gain: float) -> np.ndarray:
    return np.clip(img.astype(np.float32) * gain, 0, 255).astype(np.uint8)


def as_stream(img, w, h, quality=12, blur=0, shear=0.0, gain=1.0):
    """The full path a frame takes from sensor to detector."""
    out = cv2.resize(img, (w, h), interpolation=cv2.INTER_AREA)
    if gain != 1.0:
        out = exposure(out, gain)
    if blur:
        out = motion_blur(out, blur)
    if shear:
        out = rolling_shutter(out, shear)
    return jpeg_cycle(out, quality)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--image", default="tests/data/zidane.jpg")
    args = ap.parse_args()

    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    path = args.image if os.path.isabs(args.image) \
        else os.path.join(root, args.image)
    img = cv2.imread(path)
    if img is None:
        raise SystemExit(f"cannot read {path}")

    cfg = Config()
    det = PersonDetector(cfg.detector, cfg.providers)
    fa = FaceAnalyzer(cfg.face, cfg.providers)

    # Ground truth from the pristine image.
    ref_people = det(img)
    ref_faces = fa.detect_and_embed(img)
    ref_faces.sort(key=lambda f: -f.w * f.h)
    print(f"reference ({img.shape[1]}x{img.shape[0]}): "
          f"{len(ref_people)} people, {len(ref_faces)} faces\n")
    if not ref_faces:
        raise SystemExit("no reference face — pick an image with a clear face")
    target = ref_faces[0].embedding

    def measure(frame):
        people = det(frame)
        faces = fa.detect_and_embed(frame)
        best = max((float(target @ f.embedding) for f in faces), default=-1.0)
        biggest = max((min(f.w, f.h) for f in faces), default=0.0)
        return len(people), len(faces), best, biggest

    print("=== resolution, clean JPEG q12 ===")
    print(f"{'stream':8} {'people':>7} {'faces':>6} {'facepx':>7} "
          f"{'match':>7}  verdict")
    for name, w, h in RESOLUTIONS:
        n, nf, sim, px = measure(as_stream(img, w, h))
        v = "ok" if sim >= cfg.face.match_thres else (
            "FACE LOST" if nf == 0 else "below threshold")
        print(f"{name:8} {n:7} {nf:6} {px:7.0f} {sim:7.3f}  {v}")

    print("\n=== SVGA, JPEG quality sweep ===")
    print(f"{'quality':8} {'people':>7} {'faces':>6} {'match':>7}")
    for q in (5, 8, 12, 20, 30):
        n, nf, sim, _ = measure(as_stream(img, 800, 600, quality=q))
        print(f"{q:8} {n:7} {nf:6} {sim:7.3f}")

    print("\n=== SVGA q12, motion blur (yaw during exposure) ===")
    print(f"{'blur px':8} {'people':>7} {'faces':>6} {'match':>7}")
    for b in (0, 3, 5, 9, 15):
        n, nf, sim, _ = measure(as_stream(img, 800, 600, blur=b))
        print(f"{b:8} {n:7} {nf:6} {sim:7.3f}")

    print("\n=== SVGA q12, rolling shutter shear ===")
    print(f"{'shear px':8} {'people':>7} {'faces':>6} {'match':>7}")
    for s in (0, 4, 8, 16, 32):
        n, nf, sim, _ = measure(as_stream(img, 800, 600, shear=s))
        print(f"{s:8} {n:7} {nf:6} {sim:7.3f}")

    print("\n=== SVGA q12, exposure error ===")
    print(f"{'gain':8} {'people':>7} {'faces':>6} {'match':>7}")
    for g in (0.35, 0.5, 0.75, 1.0, 1.5, 2.2):
        n, nf, sim, _ = measure(as_stream(img, 800, 600, gain=g))
        print(f"{g:<8.2f} {n:7} {nf:6} {sim:7.3f}")

    print("\n=== realistic combined drone frames ===")
    combos = [
        ("SVGA calm",     dict(w=800, h=600, quality=12, blur=2, shear=2)),
        ("SVGA moving",   dict(w=800, h=600, quality=12, blur=6, shear=10)),
        ("SVGA aggressive", dict(w=800, h=600, quality=10, blur=11, shear=22)),
        ("VGA moving",    dict(w=640, h=480, quality=12, blur=6, shear=10)),
        ("QVGA moving",   dict(w=320, h=240, quality=12, blur=4, shear=6)),
    ]
    print(f"{'case':18} {'people':>7} {'faces':>6} {'match':>7}  verdict")
    for label, kw in combos:
        w, h = kw.pop("w"), kw.pop("h")
        n, nf, sim, _ = measure(as_stream(img, w, h, **kw))
        v = "follows" if sim >= cfg.face.match_thres else (
            "body-ReID only" if n >= len(ref_people) else "DEGRADED")
        print(f"{label:18} {n:7} {nf:6} {sim:7.3f}  {v}")

    print(f"\nface match threshold {cfg.face.match_thres}, "
          f"veto floor {cfg.face.veto_thres}")
    print("Detection surviving with the face lost is fine — that is what the "
          "body ReID channel is for.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
