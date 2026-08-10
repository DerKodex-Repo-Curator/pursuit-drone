#!/usr/bin/env python3
"""Live gamepad readout. Run this in a Terminal you can see.

    .venv/bin/python test_gamepad.py

Move the sticks and watch the bars. If they move, the controller works and
`run.py` will fly with it. If they stay at zero, the readout below says which
of the two likely causes it is.

macOS note: since Monterey the system's Game Controller framework claims Xbox
and PlayStation pads exclusively, so raw HID (hidapi) reads nothing at all from
them. That is why this uses GameController via pyobjc instead — named controls,
no byte offsets, no per-pad calibration.
"""
from __future__ import annotations

import sys
import time


def bar(value: float, width: int = 21) -> str:
    """Signed bar, centre origin."""
    mid = width // 2
    cells = ["-"] * width
    cells[mid] = "|"
    pos = int(round(mid + value * mid))
    pos = max(0, min(width - 1, pos))
    cells[pos] = "#"
    return "".join(cells)


def trigger_bar(value: float, width: int = 21) -> str:
    n = int(round(value * width))
    return "#" * n + "-" * (width - n)


def main() -> int:
    try:
        import GameController as GC
        from Foundation import NSDate, NSRunLoop
    except ImportError:
        print("pyobjc GameController is missing. Install it with:")
        print("  .venv/bin/python -m pip install pyobjc-framework-GameController")
        return 1

    def pump(seconds: float) -> None:
        NSRunLoop.currentRunLoop().runUntilDate_(
            NSDate.dateWithTimeIntervalSinceNow_(seconds))

    print("looking for a controller...")
    pad = None
    for _ in range(50):
        pump(0.1)
        found = GC.GCController.controllers()
        if found:
            pad = found[0]
            break

    if pad is None:
        print("\nNo controller found.\n")
        print("  * press the Xbox button to wake it")
        print("  * check System Settings > Bluetooth, or plug in USB-C")
        print("  * an Xbox pad shows up on macOS with no driver needed")
        return 1

    gamepad = pad.extendedGamepad()
    if gamepad is None:
        print(f"\nFound '{pad.vendorName()}' but it has no extended gamepad "
              f"profile — unsupported layout.")
        return 1

    print(f"\nconnected: {pad.productCategory()}  ({pad.vendorName()})")

    # GameController is event-driven. Polling `.value()` alone can read zero
    # forever on some macOS versions unless a change handler is registered —
    # registering one is what makes the framework actually start delivering
    # state for this process.
    events = {"n": 0}

    def on_change(gp, element):
        events["n"] += 1

    try:
        gamepad.setValueChangedHandler_(on_change)
    except Exception as exc:
        print(f"  (could not register a change handler: {exc})")

    print("\nMove the sticks and squeeze the triggers. Ctrl-C to stop.\n")

    moved = False
    frames = 0
    try:
        while True:
            pump(0.03)
            frames += 1
            lx = gamepad.leftThumbstick().xAxis().value()
            ly = gamepad.leftThumbstick().yAxis().value()
            rx = gamepad.rightThumbstick().xAxis().value()
            ry = gamepad.rightThumbstick().yAxis().value()
            lt = gamepad.leftTrigger().value()
            rt = gamepad.rightTrigger().value()
            a = bool(gamepad.buttonA().isPressed())
            b = bool(gamepad.buttonB().isPressed())
            y = bool(gamepad.buttonY().isPressed())

            if max(abs(lx), abs(ly), abs(rx), abs(ry), lt, rt) > 0.15:
                moved = True

            arm = "ARMED " if rt >= 0.35 else "safe  "
            mode = "MANUAL" if a else "ASSIST" if b else "FOLLOW" if y else "      "

            sys.stdout.write(
                f"\r  roll {bar(lx)} {lx:+.2f}  pitch {bar(ly)} {ly:+.2f}  "
                f"yaw {bar(rx)} {rx:+.2f}  thr {bar(ry)} {ry:+.2f}  "
                f"RT {trigger_bar(rt, 8)} {arm}{mode} ev{events['n']:5d}")
            sys.stdout.flush()
    except KeyboardInterrupt:
        pass

    print("\n")
    if events["n"] and not moved:
        # Events arriving but every value zero means the framework is talking
        # to us and the *decode* is wrong — a different problem from silence.
        print(f"{events['n']} input events arrived but all values read zero.")
        print("That is a decode problem, not a permission one — tell Claude.")
        return 1
    if moved:
        print("WORKING — the controller is delivering input.")
        print("\n  right trigger = deadman (hold to arm)")
        print("  left stick    = roll / pitch")
        print("  right stick   = yaw / throttle")
        print("  A / B / Y     = MANUAL / ASSIST / FOLLOW")
        print("\nNow try:  .venv/bin/python run.py --target alex")
    else:
        print(f"NO INPUT SEEN in {frames} samples.")
        print("\nThe controller was found but never reported a value, which is")
        print("almost always one of:")
        print("  1. it is paired but asleep — press the Xbox button")
        print("  2. another app has focus and is grabbing it")
        print("  3. macOS Input Monitoring: System Settings > Privacy &")
        print("     Security > Input Monitoring, and enable Terminal")
    return 0


if __name__ == "__main__":
    sys.exit(main())
