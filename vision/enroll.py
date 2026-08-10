#!/usr/bin/env python3
"""Enrol the person the drone should follow.

    python enroll.py --name alex                  # from the webcam, guided
    python enroll.py --name alex --photos a.jpg b.jpg
    python enroll.py --list
    python enroll.py --remove alex

Several samples per person, not one. A single frame captures one pose under one
light, and the drone will see neither again — it will see the subject in
profile, backlit, from above, at fifteen metres. Ten samples across a few
angles roughly halves the miss rate for a few seconds of extra work.

The webcam flow asks for specific poses rather than just grabbing ten frames,
because ten frames of someone sitting perfectly still is one sample recorded
ten times.
"""
from __future__ import annotations

import argparse
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from pursuit.config import Config                              # noqa: E402
from pursuit.face import FaceAnalyzer                           # noqa: E402
from pursuit.gallery import Gallery                             # noqa: E402

POSES = [
    "look straight at the camera",
    "turn your head slightly LEFT",
    "turn your head slightly RIGHT",
    "tilt your chin DOWN a little",
    "tilt your chin UP a little",
    "step BACK so you are smaller in frame",
    "straight on again, different expression",
    "step slightly to one side",
]

FONT = cv2.FONT_HERSHEY_SIMPLEX


def _largest(faces):
    return max(faces, key=lambda f: f.w * f.h) if faces else None


def _banner(img, lines, colour=(255, 255, 255)):
    h, w = img.shape[:2]
    box = 26 * len(lines) + 16
    overlay = img.copy()
    cv2.rectangle(overlay, (0, h - box), (w, h), (24, 22, 20), -1)
    cv2.addWeighted(overlay, 0.65, img, 0.35, 0, img)
    for i, line in enumerate(lines):
        y = h - box + 28 + i * 26
        cv2.putText(img, line, (16, y), FONT, 0.6, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(img, line, (16, y), FONT, 0.6, colour, 1, cv2.LINE_AA)


def enroll_webcam(analyzer: FaceAnalyzer, cfg: Config,
                  samples: int) -> np.ndarray | None:
    cap = cv2.VideoCapture(cfg.source.camera)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, cfg.source.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cfg.source.height)
    if not cap.isOpened():
        print("could not open the camera")
        return None

    vectors: list[np.ndarray] = []
    print("\n  SPACE captures   BACKSPACE undoes   q aborts\n")

    try:
        while len(vectors) < samples:
            ok, frame = cap.read()
            if not ok:
                break
            if cfg.source.flip:
                frame = cv2.flip(frame, 1)

            faces = analyzer.detect(frame)
            face = _largest(faces)
            view = frame.copy()

            if face is not None:
                x1, y1, x2, y2 = (int(v) for v in face.xyxy)
                big = min(face.w, face.h) >= cfg.face.min_face_px * 2
                colour = (90, 230, 120) if big else (60, 190, 250)
                cv2.rectangle(view, (x1, y1), (x2, y2), colour, 2)
                for (px, py) in face.kps.astype(int):
                    cv2.circle(view, (px, py), 2, colour, -1)

            pose = POSES[len(vectors) % len(POSES)]
            lines = [f"[{len(vectors)}/{samples}]  {pose}"]
            if face is None:
                lines.append("no face detected")
                colour = (70, 70, 245)
            elif min(face.w, face.h) < cfg.face.min_face_px * 2:
                lines.append("move closer — face is small")
                colour = (60, 190, 250)
            else:
                lines.append("SPACE to capture")
                colour = (90, 230, 120)
            _banner(view, lines, colour)

            cv2.imshow("enroll", view)
            key = cv2.waitKey(1) & 0xFF

            if key == ord("q") or key == 27:
                print("aborted")
                return None
            if key == 8 and vectors:                    # backspace
                vectors.pop()
                print(f"  removed — {len(vectors)} left")
            if key == 32 and face is not None:
                if min(face.w, face.h) < cfg.face.min_face_px * 2:
                    print("  too small, move closer")
                    continue
                vectors.append(analyzer.embed(frame, face).copy())
                print(f"  captured {len(vectors)}/{samples}  ({pose})")
    finally:
        cap.release()
        cv2.destroyAllWindows()

    return np.stack(vectors) if vectors else None


def enroll_photos(analyzer: FaceAnalyzer, cfg: Config,
                  paths: list[str]) -> np.ndarray | None:
    vectors: list[np.ndarray] = []
    for path in paths:
        img = cv2.imread(path)
        if img is None:
            print(f"  skip  {path}: unreadable")
            continue
        faces = analyzer.detect(img)
        face = _largest(faces)
        if face is None:
            print(f"  skip  {os.path.basename(path)}: no face found")
            continue
        if min(face.w, face.h) < cfg.face.min_face_px:
            print(f"  skip  {os.path.basename(path)}: face too small "
                  f"({min(face.w, face.h):.0f}px)")
            continue
        vectors.append(analyzer.embed(img, face).copy())
        print(f"  ok    {os.path.basename(path)}  "
              f"face {face.w:.0f}x{face.h:.0f}px")
    return np.stack(vectors) if vectors else None


def report_quality(vectors: np.ndarray) -> None:
    """Mutual similarity across the captured samples.

    All-identical vectors mean the same pose was captured repeatedly, which
    gives a gallery that only recognises the subject when they stand exactly as
    they did during enrolment. Spread is the point.
    """
    if len(vectors) < 2:
        return
    sims = vectors @ vectors.T
    off = sims[~np.eye(len(vectors), dtype=bool)]
    lo, mean = float(off.min()), float(off.mean())
    print(f"\n  sample spread: mean {mean:.3f}, min {lo:.3f}")
    if mean > 0.92:
        print("  NOTE: samples are nearly identical — recapture with more "
              "varied angles, or the lock will only work head-on.")
    elif lo < 0.25:
        print("  NOTE: one sample looks unlike the rest. If someone else "
              "was in frame, re-enrol — a wrong vector here follows the "
              "wrong person later.")
    else:
        print("  good spread")


def main() -> int:
    ap = argparse.ArgumentParser(description="Enrol a follow target")
    ap.add_argument("--name", help="identity to create")
    ap.add_argument("--photos", nargs="+", help="enrol from image files")
    ap.add_argument("--samples", type=int, default=8)
    ap.add_argument("--height", type=float, metavar="METRES",
                    help="subject's height, e.g. 1.75. Lets the drone hold a "
                         "real follow distance in metres instead of a "
                         "fraction of frame height.")
    ap.add_argument("--list", action="store_true", help="list enrolled people")
    ap.add_argument("--remove", metavar="NAME", help="delete an identity")
    ap.add_argument("--config", help="YAML config override")
    args = ap.parse_args()

    cfg = Config.load(args.config)
    gallery = Gallery(cfg.gallery)
    gallery.load_all()

    if args.list:
        if not len(gallery):
            print("no identities enrolled")
            return 0
        print(f"{len(gallery)} enrolled in {cfg.gallery}:")
        for name in gallery.names():
            print(f"  {name:20} {len(gallery.identities[name])} samples")
        return 0

    if args.remove:
        path = gallery.path_for(args.remove)
        if not os.path.exists(path):
            print(f"'{args.remove}' is not enrolled")
            return 1
        os.remove(path)
        print(f"removed {args.remove}")
        return 0

    if not args.name:
        ap.error("--name is required (or use --list / --remove)")

    if args.name in gallery:
        existing = len(gallery.identities[args.name])
        reply = input(f"'{args.name}' already has {existing} samples. "
                      f"Replace? [y/N] ").strip().lower()
        if reply != "y":
            print("cancelled")
            return 1

    print("loading face models...")
    analyzer = FaceAnalyzer(cfg.face, cfg.providers)

    if args.photos:
        vectors = enroll_photos(analyzer, cfg, args.photos)
    else:
        vectors = enroll_webcam(analyzer, cfg, args.samples)

    if vectors is None or len(vectors) == 0:
        print("\nnothing captured — not saving")
        return 1

    if args.height is not None and not (0.5 <= args.height <= 2.5):
        print(f"\n--height {args.height} is not a plausible human height "
              f"in metres")
        return 1

    report_quality(vectors)
    path = gallery.save(args.name, vectors, height_m=args.height)
    print(f"\nsaved {len(vectors)} samples -> {path}")
    if args.height:
        print(f"height {args.height:.2f}m recorded — follow distance will be "
              f"held in metres")
    else:
        print("no --height given: follow distance falls back to a fraction of "
              "frame height. Re-run with --height <metres> for a real range.")
    print(f"\nnow run:  python run.py --target {args.name}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
