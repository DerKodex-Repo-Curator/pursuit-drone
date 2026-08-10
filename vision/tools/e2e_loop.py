#!/usr/bin/env python3
"""The whole chain, live, in one process.

Every layer has its own suite, and the interop vectors prove the two protocol
implementations agree byte-for-byte. What none of that exercises is the SYSTEM
running: frames flowing through detection, the lock feeding the controller,
the pump keeping the wire warm, the (simulated) drone validating and applying
packets, telemetry closing the loop back into attitude compensation and
measured-latency lead. This runs exactly that, end to end, and checks the
properties that only exist at the system level.

    .venv/bin/python tools/e2e_loop.py
"""
from __future__ import annotations

import os
import socket
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from pursuit.config import Config                                # noqa: E402
from pursuit.face import FaceAnalyzer                            # noqa: E402
from pursuit.gallery import Gallery                              # noqa: E402
from pursuit.link import CommandPump, DroneLink                  # noqa: E402
from pursuit.lock import LockState                               # noqa: E402
from pursuit.pipeline import Pipeline                            # noqa: E402
from sim_drone import SimDrone                                   # noqa: E402


def free_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def pan_frames(img, n=150, amplitude=320):
    h, w = img.shape[:2]
    for i in range(n):
        dx = int(amplitude * np.sin(2 * np.pi * i / n))
        m = np.float32([[1, 0, dx], [0, 1, 0]])
        yield cv2.warpAffine(img, m, (w, h), borderMode=cv2.BORDER_REPLICATE)


def main() -> int:
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    img = cv2.imread(os.path.join(root, "tests", "data", "zidane.jpg"))
    if img is None:
        raise SystemExit("tests/data/zidane.jpg missing")

    import tempfile
    tmp = tempfile.mkdtemp(prefix="e2e_gallery_")

    cfg = Config()
    cfg.gallery = tmp
    cfg.link.host = "127.0.0.1"
    cfg.link.port = free_port()
    cfg.link.telem_port = free_port()
    cfg.link.enabled = True

    print("enrolling the target from the reference image...")
    fa = FaceAnalyzer(cfg.face, cfg.providers)
    faces = fa.detect_and_embed(img)
    faces.sort(key=lambda f: -f.w * f.h)
    Gallery(tmp).save("target", faces[0].embedding[None, :], height_m=1.85)

    drone = SimDrone(cfg.link.port, cfg.link.telem_port, "127.0.0.1",
                     lossy=0.10, corrupt=0.05)   # a deliberately bad WiFi day
    link = DroneLink(cfg.link)
    pump = CommandPump(link, cfg.link.send_hz).start()
    pipe = Pipeline(cfg, target_name="target", quiet=True)

    locked = 0
    frames = 0
    lead_before = pipe.controller.lead_time_s
    t0 = time.perf_counter()

    for frame in pan_frames(img):
        attitude = None
        if link.telemetry is not None:
            attitude = (link.telemetry.roll, link.telemetry.pitch)
        if link.stats.rtt_ms > 0.0:
            pipe.controller.set_measured_latency(link.stats.rtt_ms)

        result = pipe.process(frame, armed=False, attitude=attitude)
        pump.submit(result.command)

        deadline = time.perf_counter() + 0.01
        while time.perf_counter() < deadline:
            drone.poll()
        drone.watchdog()
        if frames % 2 == 0:
            drone.send_telemetry()

        frames += 1
        if result.lock.state is LockState.LOCKED:
            locked += 1

    elapsed = time.perf_counter() - t0
    time.sleep(0.1)
    pump.stop()
    link.close()
    drone.close()

    s = link.stats
    print(f"\n{frames} frames in {elapsed:.1f}s ({frames/elapsed:.1f} fps)")
    print(f"lock      LOCKED on {locked}/{frames} frames "
          f"({100*locked/frames:.0f}%)")
    print(f"link tx   {s.sent} sent (x{cfg.link.redundancy} redundancy)")
    print(f"drone rx  {drone.accepted} accepted, {drone.stale} stale copies, "
          f"{drone.bad_crc} corrupt rejected, {drone.dropped} lost in transit")
    print(f"telemetry {s.received} received, rtt {s.rtt_ms:.1f}ms")
    print(f"lead      {lead_before*1000:.0f}ms guess -> "
          f"{pipe.controller.lead_time_s*1000:.0f}ms measured")
    print(f"drone     state {drone.state} (unarmed commands, must be 1=IDLE)")

    failures = []
    if locked < frames * 0.5:
        failures.append("lock held under 50%")
    if drone.accepted < 20:
        failures.append("too few commands accepted")
    if drone.stale < drone.accepted:
        failures.append("redundant copies not reaching the drone")
    if drone.bad_crc == 0:
        failures.append("corruption injected but nothing rejected")
    if not (0.0 < s.rtt_ms < 500.0):
        failures.append(f"rtt implausible ({s.rtt_ms:.1f}ms)")
    if abs(pipe.controller.lead_time_s - lead_before) < 1e-6:
        failures.append("measured latency never adopted")
    if drone.state != 1:
        failures.append(f"drone left IDLE on unarmed commands (state "
                        f"{drone.state})")

    if failures:
        print("\nE2E FAIL: " + "; ".join(failures))
        return 1
    print("\nE2E PASS — full chain healthy under 10% loss + 5% corruption")
    return 0


if __name__ == "__main__":
    sys.exit(main())
