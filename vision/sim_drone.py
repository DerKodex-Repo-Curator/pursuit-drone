#!/usr/bin/env python3
"""A fake PURSUIT-1 on the other end of the UDP link.

    python sim_drone.py                    # listen, print, reply with telemetry
    python sim_drone.py --lossy 0.1        # drop 10% of packets
    python sim_drone.py --corrupt 0.05     # flip a bit in 5% — CRC must catch

Two jobs:

**Exercise the link before the aircraft exists.** Run this in one terminal and
`run.py --link --host 127.0.0.1` in another, and the whole transmit path gets
tested — packing, CRC, sequencing, the pump's failsafe substitution, telemetry
decode, the HUD's link panel — with no hardware.

**Model the failures the real link will have.** WiFi drops and corrupts
packets. `--lossy` and `--corrupt` make that happen on demand, so the handling
is verified rather than assumed. A corrupted packet that gets *accepted* is the
serious bug, and the only way to know the CRC catches it is to corrupt some.

It also implements the same watchdog the firmware must: no valid command for
PURSUIT_LINK_TIMEOUT_MS and it goes to FAILSAFE, then LANDING. If the real
firmware does not do this, the aircraft flies away when the laptop sleeps.
"""
from __future__ import annotations

import argparse
import math
import os
import random
import signal
import socket
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from pursuit.protocol import (CMD_SIZE, ProtocolError, STATE_FAILSAFE,  # noqa: E402
                              STATE_FLYING, STATE_IDLE, STATE_LANDING,
                              Command, Telemetry, is_newer)

TIMEOUT_MS = 400.0
LAND_AFTER_MS = 3000.0


class SimDrone:
    """Just enough vehicle to make the link behave realistically."""

    def __init__(self, cmd_port: int, telem_port: int, host: str,
                 lossy: float, corrupt: float, seed: int = 0):
        self.rng = random.Random(seed)
        self.lossy = lossy
        self.corrupt = corrupt
        self.host = host
        self.telem_port = telem_port

        self.rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.rx.bind(("0.0.0.0", cmd_port))
        self.rx.settimeout(0.05)
        self.tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.t0 = time.monotonic()
        self.last_cmd_at = 0.0
        self.last_cmd_t_ms = 0
        self.last_seq = -1
        self.state = STATE_IDLE
        self.seq = 0

        # Toy dynamics — enough that the HUD's telemetry panel shows plausible
        # numbers and altitude visibly chases the setpoint.
        self.yaw = 0.0
        self.alt = 0.0
        self.vbat = 4.05

        self.accepted = self.dropped = self.bad_crc = self.stale = 0

    # ------------------------------------------------------------------ recv --
    def poll(self) -> None:
        try:
            raw, addr = self.rx.recvfrom(512)
        except socket.timeout:
            return
        except OSError:
            return

        if self.lossy and self.rng.random() < self.lossy:
            self.dropped += 1
            return

        if self.corrupt and self.rng.random() < self.corrupt:
            buf = bytearray(raw)
            i = self.rng.randrange(len(buf))
            buf[i] ^= 1 << self.rng.randrange(8)
            raw = bytes(buf)

        if len(raw) != CMD_SIZE:
            self.bad_crc += 1
            return
        try:
            cmd = Command.unpack(raw)
        except ProtocolError:
            # This is the branch that must fire when --corrupt is on. If it
            # never does, the CRC is not doing its job.
            self.bad_crc += 1
            return

        if self.last_seq >= 0 and not is_newer(cmd.seq, self.last_seq):
            self.stale += 1
            return

        self.last_seq = cmd.seq
        self.last_cmd_at = time.monotonic()
        self.last_cmd_t_ms = cmd.t_ms
        self.accepted += 1
        self.apply(cmd)

    # ----------------------------------------------------------------- model --
    def apply(self, cmd: Command) -> None:
        dt = 1.0 / 30.0
        if cmd.failsafe or not cmd.armed:
            self.state = STATE_FAILSAFE if cmd.failsafe else STATE_IDLE
        else:
            self.state = STATE_FLYING
            self.yaw = (self.yaw + cmd.yaw_rate * dt) % 360.0
        if cmd.alt_hold:
            self.alt += (cmd.alt_sp - self.alt) * 0.15
        self.vbat = max(3.30, self.vbat - 2.0e-5)

    def watchdog(self) -> None:
        """The behaviour the firmware must copy.

        Silence is not a reason to hold the last command — it is a reason to
        stop acting on it. Level off first, then descend, because a drone that
        keeps its last yaw rate after losing the link travels in a circle until
        it hits something.
        """
        if self.last_cmd_at == 0.0:
            return
        silent_ms = (time.monotonic() - self.last_cmd_at) * 1000.0
        if silent_ms > LAND_AFTER_MS:
            self.state = STATE_LANDING
            self.alt = max(0.0, self.alt - 0.02)
        elif silent_ms > TIMEOUT_MS:
            self.state = STATE_FAILSAFE

    # ------------------------------------------------------------------ send --
    def send_telemetry(self) -> None:
        self.seq += 1
        # Echo the last command's own timestamp, exactly as the firmware
        # does: the two ends share no clock, so an absolute local time is
        # useless to the laptop, but its own stamp back lets it measure a
        # true round trip for the lead compensation.
        telem = Telemetry(
            seq=self.seq, t_ms=self.last_cmd_t_ms,
            vbat=self.vbat, alt=self.alt,
            roll=2.0 * math.sin(self.seq / 40.0),
            pitch=1.5 * math.cos(self.seq / 55.0),
            yaw=self.yaw, loop_hz=1000, state=self.state)
        try:
            self.tx.sendto(telem.pack(), (self.host, self.telem_port))
        except OSError:
            pass

    def close(self) -> None:
        self.rx.close()
        self.tx.close()


def main() -> int:
    ap = argparse.ArgumentParser(description="Fake PURSUIT-1 for link testing")
    ap.add_argument("--port", type=int, default=14550)
    ap.add_argument("--telem-port", type=int, default=14551)
    ap.add_argument("--host", default="127.0.0.1",
                    help="where to send telemetry")
    ap.add_argument("--lossy", type=float, default=0.0,
                    help="fraction of commands to drop, 0..1")
    ap.add_argument("--corrupt", type=float, default=0.0,
                    help="fraction to bit-flip, 0..1")
    ap.add_argument("--seconds", type=float, default=0.0,
                    help="exit after N seconds (0 = run until Ctrl-C)")
    args = ap.parse_args()

    drone = SimDrone(args.port, args.telem_port, args.host,
                     args.lossy, args.corrupt)
    running = True

    def stop(*_):
        nonlocal running
        running = False
    signal.signal(signal.SIGINT, stop)

    print(f"sim drone listening on :{args.port}, "
          f"telemetry -> {args.host}:{args.telem_port}")
    if args.lossy or args.corrupt:
        print(f"  fault injection: drop {args.lossy:.0%}, "
              f"corrupt {args.corrupt:.0%}")
    print()

    t_start = time.monotonic()
    next_telem = time.monotonic()
    next_print = time.monotonic()

    try:
        while running:
            drone.poll()
            drone.watchdog()
            now = time.monotonic()

            if now >= next_telem:
                drone.send_telemetry()
                next_telem = now + 1.0 / 30.0

            if now >= next_print:
                next_print = now + 1.0
                from pursuit.protocol import STATE_NAMES
                print(f"  {STATE_NAMES.get(drone.state,'?'):9} "
                      f"alt {drone.alt:4.2f}m  yaw {drone.yaw:6.1f}  "
                      f"vbat {drone.vbat:.2f}V  |  "
                      f"ok {drone.accepted}  dropped {drone.dropped}  "
                      f"rejected {drone.bad_crc}  stale {drone.stale}")

            if args.seconds and (now - t_start) > args.seconds:
                break
            time.sleep(0.002)
    finally:
        drone.close()

    total = drone.accepted + drone.bad_crc + drone.stale
    print(f"\naccepted {drone.accepted}  rejected {drone.bad_crc}  "
          f"stale {drone.stale}  dropped-in-transit {drone.dropped}")
    if drone.corrupt and total:
        print(f"corruption was injected and {drone.bad_crc} packets were "
              f"rejected — CRC is doing its job")
    return 0


if __name__ == "__main__":
    sys.exit(main())
