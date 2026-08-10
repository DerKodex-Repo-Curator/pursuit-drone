#!/usr/bin/env python3
"""Decode and plot a PURSUIT-1 flight log.

    # after landing, with the laptop still on the PURSUIT-1 network
    curl -o flight.pbb http://192.168.4.1:82/
    python3 tools/decode_blackbox.py flight.pbb

    python3 tools/decode_blackbox.py flight.pbb --csv flight.csv
    python3 tools/decode_blackbox.py flight.pbb --plot flight.png

What to look for when tuning, in the order worth checking:

  gyro vs setpoint   the rate loop's actual job. Lag means P too low;
                     overshoot then ringing means P too high or D too low;
                     a fast fuzzy oscillation on top means D too high and it
                     is amplifying gyro noise.
  motor traces       if any motor sits pinned at 0 or 255 the mixer is
                     saturated and NOTHING above it is being tuned — you are
                     looking at the limits, not the gains.
  loop_us            should sit near 1000. Spikes mean something (usually
                     WiFi or the camera) is stealing the control loop.
  vbat               sag on throttle steps. If it dips under ~3.5V the
                     brownout limiter is the thing to fix, not the PIDs.

The record layout comes from firmware/src/blackbox.h and the header carries
`record_size`, so a log written by a different firmware build is rejected
rather than silently mis-parsed.
"""
from __future__ import annotations

import argparse
import struct
import sys

HEADER_FMT = "<4sHHIII"
HEADER_SIZE = struct.calcsize(HEADER_FMT)
RECORD_FMT = "<I3h3h3h3h4BBBHBBH"
RECORD_SIZE = struct.calcsize(RECORD_FMT)

# Must match blackbox.cpp exactly — see the note in blackbox.h.
GYRO_SCALE = 2000.0 / 32767.0
ANGLE_SCALE = 180.0 / 32767.0
UNIT_SCALE = 1.0 / 32767.0
DUTY_SCALE = 1.0 / 255.0
VOLT_SCALE = 1.0 / 50.0

STATE_NAMES = {0: "BOOT", 1: "IDLE", 2: "ARMED", 3: "FLYING",
               4: "FAILSAFE", 5: "LANDING"}


class Flight:
    def __init__(self, path: str):
        with open(path, "rb") as fh:
            raw = fh.read()

        if len(raw) < HEADER_SIZE:
            raise SystemExit(f"{path}: too short to be a flight log")

        magic, ver, rec_size, hz, n, dropped = struct.unpack(
            HEADER_FMT, raw[:HEADER_SIZE])
        if magic != b"PBB1":
            raise SystemExit(f"{path}: not a PURSUIT blackbox file "
                             f"(magic {magic!r})")
        if rec_size != RECORD_SIZE:
            raise SystemExit(
                f"{path}: record size {rec_size} but this decoder expects "
                f"{RECORD_SIZE}.\nThe log came from a different firmware "
                f"build — decoding it would silently misread every field.")

        self.sample_hz = hz
        self.dropped = dropped
        body = raw[HEADER_SIZE:]
        avail = len(body) // RECORD_SIZE
        if avail < n:
            print(f"  warning: header claims {n} records, file holds {avail} "
                  f"— download may have been truncated")
            n = avail

        self.t, self.gyro, self.att, self.sp = [], [], [], []
        self.pid, self.motor, self.throttle = [], [], []
        self.vbat, self.alt, self.state, self.flags, self.loop_us = \
            [], [], [], [], []

        for i in range(n):
            f = struct.unpack_from(RECORD_FMT, body, i * RECORD_SIZE)
            self.t.append(f[0] * 1e-6)
            self.gyro.append(tuple(v * GYRO_SCALE for v in f[1:4]))
            self.att.append(tuple(v * ANGLE_SCALE for v in f[4:7]))
            # sp[2] is a yaw RATE, so it carries the gyro scale, not the angle
            self.sp.append((f[7] * ANGLE_SCALE, f[8] * ANGLE_SCALE,
                            f[9] * GYRO_SCALE))
            self.pid.append(tuple(v * UNIT_SCALE for v in f[10:13]))
            self.motor.append(tuple(v * DUTY_SCALE for v in f[13:17]))
            self.throttle.append(f[17] * DUTY_SCALE)
            self.vbat.append(f[18] * VOLT_SCALE)
            self.alt.append(f[19] * 0.001)
            self.state.append(f[20])
            self.flags.append(f[21])
            self.loop_us.append(f[22])

    def __len__(self) -> int:
        return len(self.t)

    def duration(self) -> float:
        return (self.t[-1] - self.t[0]) if len(self.t) > 1 else 0.0


def summarise(f: Flight) -> None:
    if not len(f):
        print("empty log")
        return

    print(f"{len(f)} records, {f.duration():.1f}s at {f.sample_hz}Hz")
    if f.dropped:
        print(f"  {f.dropped} records overwritten — the flight outlasted the "
              f"buffer, so the EARLY part is missing")

    states = {}
    for s in f.state:
        states[s] = states.get(s, 0) + 1
    print("  states: " + "  ".join(
        f"{STATE_NAMES.get(s, s)} {c*100//len(f)}%"
        for s, c in sorted(states.items(), key=lambda kv: -kv[1])))

    lo, hi = min(f.vbat), max(f.vbat)
    print(f"  vbat {lo:.2f}-{hi:.2f}V", end="")
    if lo < 3.5:
        print("   <- SAGGED BELOW BUCK DROPOUT: fix power before tuning")
    else:
        print()

    mean_loop = sum(f.loop_us) / len(f)
    worst = max(f.loop_us)
    print(f"  loop {mean_loop:.0f}us mean, {worst}us worst", end="")
    print("   <- control loop was stalled" if worst > 3000 else "")

    # Saturation is the thing that silently invalidates a tuning session.
    pinned = sum(1 for m in f.motor if max(m) >= 0.99 or min(m) <= 0.01)
    if pinned > len(f) * 0.05:
        print(f"  motors saturated in {pinned*100//len(f)}% of samples "
              f"— the mixer is at its limits, so the PID gains are NOT what "
              f"you are observing")

    for i, name in enumerate(("roll", "pitch")):
        err = [abs(a[i] - s[i]) for a, s in zip(f.att, f.sp)]
        print(f"  {name:5} tracking error  mean {sum(err)/len(err):5.1f} deg"
              f"   worst {max(err):5.1f} deg")


def write_csv(f: Flight, path: str) -> None:
    import csv
    with open(path, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["t_s", "gx", "gy", "gz", "roll", "pitch", "yaw",
                    "sp_roll", "sp_pitch", "sp_yawrate",
                    "pid_roll", "pid_pitch", "pid_yaw",
                    "m1", "m2", "m3", "m4", "throttle", "vbat", "alt_m",
                    "state", "flags", "loop_us"])
        for i in range(len(f)):
            w.writerow([f"{f.t[i]:.4f}", *[f"{v:.2f}" for v in f.gyro[i]],
                        *[f"{v:.2f}" for v in f.att[i]],
                        *[f"{v:.2f}" for v in f.sp[i]],
                        *[f"{v:.4f}" for v in f.pid[i]],
                        *[f"{v:.3f}" for v in f.motor[i]],
                        f"{f.throttle[i]:.3f}", f"{f.vbat[i]:.2f}",
                        f"{f.alt[i]:.3f}", f.state[i], f.flags[i],
                        f.loop_us[i]])
    print(f"wrote {path}")


def plot(f: Flight, path: str) -> None:
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        raise SystemExit("plotting needs matplotlib:  pip install matplotlib")

    t0 = f.t[0]
    t = [x - t0 for x in f.t]
    fig, ax = plt.subplots(5, 1, figsize=(14, 14), sharex=True)

    for i, (name, col) in enumerate((("roll", "tab:blue"),
                                     ("pitch", "tab:orange"))):
        ax[i].plot(t, [a[i] for a in f.att], col, lw=0.8, label=f"{name} actual")
        ax[i].plot(t, [s[i] for s in f.sp], "k--", lw=0.8, label="setpoint")
        ax[i].set_ylabel(f"{name} (deg)")
        ax[i].legend(loc="upper right", fontsize=8)
        ax[i].grid(alpha=0.3)

    ax[2].plot(t, [g[2] for g in f.gyro], "tab:green", lw=0.6, label="yaw rate")
    ax[2].plot(t, [s[2] for s in f.sp], "k--", lw=0.8, label="setpoint")
    ax[2].set_ylabel("yaw (deg/s)")
    ax[2].legend(loc="upper right", fontsize=8)
    ax[2].grid(alpha=0.3)

    for i in range(4):
        ax[3].plot(t, [m[i] for m in f.motor], lw=0.6, label=f"M{i+1}")
    ax[3].axhline(1.0, color="r", ls=":", lw=0.8)
    ax[3].axhline(0.0, color="r", ls=":", lw=0.8)
    ax[3].set_ylabel("motor duty")
    ax[3].legend(loc="upper right", fontsize=8, ncol=4)
    ax[3].grid(alpha=0.3)

    ax[4].plot(t, f.vbat, "tab:red", lw=0.8, label="vbat")
    ax[4].axhline(3.5, color="k", ls=":", lw=0.8, label="buck dropout")
    ax4b = ax[4].twinx()
    ax4b.plot(t, f.alt, "tab:purple", lw=0.8, label="alt")
    ax4b.set_ylabel("alt (m)")
    ax[4].set_ylabel("vbat (V)")
    ax[4].set_xlabel("time (s)")
    ax[4].legend(loc="upper left", fontsize=8)
    ax[4].grid(alpha=0.3)

    fig.suptitle(f"PURSUIT-1  {len(f)} records  {f.duration():.1f}s")
    fig.tight_layout()
    fig.savefig(path, dpi=120)
    print(f"wrote {path}")


def main() -> int:
    ap = argparse.ArgumentParser(description="Decode a PURSUIT-1 flight log")
    ap.add_argument("log")
    ap.add_argument("--csv")
    ap.add_argument("--plot")
    args = ap.parse_args()

    f = Flight(args.log)
    summarise(f)
    if args.csv:
        write_csv(f, args.csv)
    if args.plot:
        plot(f, args.plot)
    return 0


if __name__ == "__main__":
    sys.exit(main())
