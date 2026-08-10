#!/usr/bin/env python3
"""Trigger the macOS camera permission prompt, then confirm it worked.

macOS only lists an app under Privacy & Security -> Camera *after* that app
asks for access. A background process asking gets denied silently and never
appears in the list, which is why there is nothing to switch on there.

So run this from Terminal.app, in the foreground, and approve the prompt when
it appears. The permission is granted to the terminal, not to Python, so every
script you later run from that terminal inherits it.

    cd ~/Documents/pursuit_drone/vision
    .venv/bin/python check_camera.py
"""
from __future__ import annotations

import sys
import time


def main() -> int:
    try:
        import cv2
    except ImportError:
        print("opencv is missing — run:  .venv/bin/python -m pip install -r requirements.txt")
        return 1

    print("opening the camera...")
    print("If a permission dialog appears, click OK / Allow.\n")

    cap = cv2.VideoCapture(0)
    # The prompt is asynchronous: the first read can fail while the dialog is
    # still on screen, so give it a few seconds rather than declaring failure.
    frame = None
    deadline = time.monotonic() + 15.0
    while time.monotonic() < deadline:
        if cap.isOpened():
            ok, frame = cap.read()
            if ok and frame is not None:
                break
        time.sleep(0.4)
        cap.release()
        cap = cv2.VideoCapture(0)

    if frame is None:
        cap.release()
        print("\nSTILL BLOCKED.\n")
        print("Try, in this order:")
        print("  1. Make sure you ran this from Terminal.app, not from an editor")
        print("     or an AI assistant — the permission attaches to whichever app")
        print("     launched python.")
        print("  2. Reset the camera permission and run this again:")
        print("       tccutil reset Camera")
        print("  3. If Terminal now appears under System Settings ->")
        print("     Privacy & Security -> Camera, switch it on and rerun.")
        print("  4. Quit and reopen Terminal — the permission is read at launch.")
        return 1

    h, w = frame.shape[:2]
    fps = cap.get(cv2.CAP_PROP_FPS)
    print(f"CAMERA OK — {w}x{h} @ {fps:.0f}fps")

    out = "camera_test.jpg"
    cv2.imwrite(out, frame)
    cap.release()
    print(f"saved a frame to {out}\n")
    print("Now run:")
    print("  .venv/bin/python enroll.py --name <friend>")
    print("  .venv/bin/python run.py --target <friend>")
    return 0


if __name__ == "__main__":
    sys.exit(main())
