#!/usr/bin/env python3
"""Work out your controller's HID layout.

The profiles in `pursuit/pilot.py` come from public HID descriptors, not from
testing against every pad, so treat them as hypotheses. This tool shows you the
live report and tells you which byte each control moves, which is the whole
job — HID gamepads are just fixed-layout byte arrays.

    python calibrate_gamepad.py              # watch the raw report
    python calibrate_gamepad.py --learn      # guided: move one control at a time

Run it with props off and nothing armed. Nothing here talks to the drone.
"""
from __future__ import annotations

import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))


def list_devices():
    import hid
    pads = [d for d in hid.enumerate()
            if d.get("usage_page") == 1 and d.get("usage") in (4, 5, 8)]
    if not pads:
        print("No gamepad found.\n")
        print("  * plug it in by USB, or pair it over Bluetooth")
        print("  * on macOS a PS5/Xbox pad shows up without any driver")
        print("  * some pads only appear as a gamepad once a button is pressed")
        return []
    print(f"{len(pads)} gamepad-like device(s):\n")
    for i, d in enumerate(pads):
        print(f"  [{i}] {d.get('product_string') or '?'}  "
              f"{d['vendor_id']:#06x}:{d['product_id']:#06x}  "
              f"({d.get('manufacturer_string') or '?'})")
    return pads


def open_device(pads, index):
    import hid
    d = pads[index]
    dev = hid.Device(d["vendor_id"], d["product_id"])
    dev.nonblocking = True
    return dev, d


def watch(dev, seconds: float):
    """Print the report whenever it changes, with changed bytes marked."""
    print("\nMove the sticks and press buttons. Ctrl-C to stop.\n")
    prev = None
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        data = dev.read(64, timeout=50)
        if not data:
            continue
        data = bytes(data)
        if prev is not None and data == prev:
            continue
        marks = "".join(
            "^" if prev is not None and i < len(prev) and data[i] != prev[i]
            else " " for i in range(len(data)))
        print("  " + " ".join(f"{b:02x}" for b in data))
        print("  " + " ".join(f"{m} " for m in marks))
        prev = data


def learn(dev):
    """Guided mapping: move one control at a time, see which byte responds."""
    steps = [
        ("LEFT stick LEFT and RIGHT", "axis_roll"),
        ("LEFT stick UP and DOWN", "axis_pitch"),
        ("RIGHT stick LEFT and RIGHT", "axis_yaw"),
        ("RIGHT stick UP and DOWN", "axis_throttle"),
    ]
    found = {}
    for prompt, field in steps:
        input(f"\n  Move ONLY the {prompt}, then press Enter... ")
        samples = []
        end = time.monotonic() + 2.5
        print("  sampling for 2.5s — keep moving it")
        while time.monotonic() < end:
            data = dev.read(64, timeout=20)
            if data:
                samples.append(bytes(data))
        if len(samples) < 5:
            print("  saw almost nothing — is the pad still connected?")
            continue

        width = min(len(s) for s in samples)

        # An Xbox stick is 16-bit little-endian, so its LOW byte sweeps 0-255
        # over and over while the HIGH byte moves smoothly. Ranking on raw
        # byte range alone therefore always picks the low byte — the noisiest
        # one — and produces an axis that jitters wildly. So score 16-bit
        # pairs too, and prefer them when the high byte is genuinely moving.
        best8 = max(((max(s[i] for s in samples) - min(s[i] for s in samples), i)
                     for i in range(width)), default=(0, 0))

        best16 = (0, 0)
        for i in range(width - 1):
            vals = [s[i] | (s[i + 1] << 8) for s in samples]
            hi_range = max(s[i + 1] for s in samples) - min(s[i + 1] for s in samples)
            # A real 16-bit axis moves its high byte over a decent sweep; a
            # coincidental pair of unrelated bytes does not.
            if hi_range >= 8:
                rng = max(vals) - min(vals)
                if rng > best16[0]:
                    best16 = (rng, i)

        if best16[0] > 4000:
            idx, bits, rng = best16[1], 16, best16[0]
        elif best8[0] >= 20:
            idx, bits, rng = best8[1], 8, best8[0]
        else:
            print(f"  nothing moved much (best byte range {best8[0]}) — skipped")
            continue

        found[field] = (idx, bits)
        print(f"  -> {field} = byte {idx}, {bits}-bit  (range {rng})")

    # --- deadman: squeeze the trigger you want to arm with -----------------
    input("\n  Squeeze and release the trigger you want as the DEADMAN "
          "(right trigger\n  is the natural choice), then press Enter... ")
    rest, moved = [], []
    end = time.monotonic() + 2.5
    print("  sampling for 2.5s — squeeze it fully, then let go")
    while time.monotonic() < end:
        data = dev.read(64, timeout=20)
        if data:
            moved.append(bytes(data))
    deadman = None
    if len(moved) >= 5:
        width = min(len(s) for s in moved)
        cand = (0, 0, 8)
        for i in range(width - 1):
            vals = [s[i] | (s[i + 1] << 8) for s in moved]
            hi = max(s[i + 1] for s in moved) - min(s[i + 1] for s in moved)
            if hi >= 2 and (max(vals) - min(vals)) > cand[0]:
                cand = (max(vals) - min(vals), i, 16)
        for i in range(width):
            rng = max(s[i] for s in moved) - min(s[i] for s in moved)
            # A trigger rests near zero, unlike a stick which rests centred.
            if rng > cand[0] and min(s[i] for s in moved) < 32:
                cand = (rng, i, 8)
        if cand[0] > 40:
            deadman = (cand[1], cand[2])
            print(f"  -> deadman = byte {cand[1]}, {cand[2]}-bit "
                  f"(range {cand[0]})")

    print("\n" + "=" * 62)
    if found:
        print("Add this to PROFILES in pursuit/pilot.py:\n")
        bits = {b for _, b in found.values()}
        axis_bits = bits.pop() if len(bits) == 1 else 8
        if len(bits):
            print("    # NOTE: axes disagreed on width — check them by hand")
        print("    PadProfile(\"my pad\", 0xVVVV, (0xPPPP,),")
        for field in ("axis_roll", "axis_pitch", "axis_yaw", "axis_throttle"):
            idx = found.get(field, (0, axis_bits))[0]
            print(f"               {field}={idx},")
        print(f"               axis_bits={axis_bits},")
        if deadman:
            print(f"               deadman_axis={deadman[0]}, "
                  f"deadman_axis_bits={deadman[1]}, deadman_thresh=0.35,")
        else:
            print("               deadman_byte=?, deadman_mask=0x??,")
        print("               mode_byte=?, mode_manual=0x01,")
        print("               mode_assist=0x02, mode_follow=0x10),")
        print("\nFor the mode buttons, rerun WITHOUT --learn, press one button")
        print("at a time, and read which byte and which bit flips.")
    else:
        print("Nothing mapped. Try --learn again, or watch the raw report.")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--device", type=int, default=0)
    ap.add_argument("--seconds", type=float, default=60.0)
    ap.add_argument("--learn", action="store_true")
    args = ap.parse_args()

    try:
        import hid  # noqa: F401
    except ImportError:
        print("hidapi is missing:  .venv/bin/python -m pip install hidapi")
        return 1

    pads = list_devices()
    if not pads:
        return 1
    if args.device >= len(pads):
        print(f"\nno device [{args.device}]")
        return 1

    try:
        dev, info = open_device(pads, args.device)
    except Exception as exc:
        print(f"\ncould not open the device: {exc}")
        print("On macOS, Bluetooth pads sometimes need re-pairing before the "
              "HID interface opens.")
        return 1

    print(f"\nopened {info.get('product_string') or '?'}  "
          f"{info['vendor_id']:#06x}:{info['product_id']:#06x}")
    try:
        if args.learn:
            learn(dev)
        else:
            watch(dev, args.seconds)
    except KeyboardInterrupt:
        print("\nstopped")
    finally:
        dev.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
