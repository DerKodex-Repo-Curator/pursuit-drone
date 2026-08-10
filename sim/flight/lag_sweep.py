#!/usr/bin/env python3
"""How much transport delay can the pitch cascade tolerate before it oscillates?

    python sim/flight/lag_sweep.py            # sweep, table, and a plot
    python sim/flight/lag_sweep.py --trace 60 # one run at 60 ms, traced

The question
------------
PURSUIT-1 closes an attitude loop across a WiFi link. The outer loop lives on a
laptop, sees the world through a camera and a detector, and its command arrives
at the aircraft tens of milliseconds after the world it describes. Delay inside
a feedback loop is phase lag, and enough phase lag turns negative feedback into
positive feedback at some frequency. The only question is which frequency, and
whether the loop has gain there.

`vision/pursuit/config.py` sets `lead_time_s = 0.15` — a guess, later measured
by `vision/tools/e2e_loop.py` at about 20 ms on loopback. Nothing established
what the aircraft could actually tolerate, so there was no way to know whether
20 ms was comfortably inside the margin or most of the way to the edge.

Method
------
Close the full three-loop cascade around the rigid-body model:

    vision PD (30 Hz, delayed) -> angle P (1 kHz) -> rate PID (1 kHz)
      -> mixer -> motor lag -> rigid body -> back to vision

Disturb it — a 40 deg/s impulse in pitch rate, roughly a gust or a knock — and
watch what the pitch angle does over the next few seconds.

Stability is read from the ENVELOPE, not from a single peak: the RMS of the
last second against the RMS of the first second after the disturbance. Below 1
the oscillation is decaying, above 1 it is growing. This is a ratio of the same
quantity at two times, so it does not depend on the disturbance size and is not
sensitive to the inertia estimate in the way an absolute overshoot would be.

What the result means, and does not
-----------------------------------
The mass properties and the motor time constant in quadrotor.py are estimates.
So the ABSOLUTE boundary this reports is provisional and will move when the
hardware exists and those are measured. What is robust is the shape: that a
margin exists, roughly where it sits, and which term dominates. Treat the
number as an order of magnitude with a named sensitivity, not a specification.
"""
from __future__ import annotations

import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__)))))

from sim.flight.cascade import (AnglePid, DelayLine, RatePid, VisionPD,  # noqa: E402
                                MAX_ANGLE_DEG, RATE_D_LPF_HZ, RATE_I_LIMIT,
                                RATE_KD, RATE_KI, RATE_KP, VIS_PITCH_DEADBAND,
                                VIS_PITCH_KD, VIS_PITCH_KP, mix)
from sim.flight.quadrotor import Airframe, Quadrotor  # noqa: E402

INNER_HZ = 1000.0        # config.h LOOP_HZ
VISION_HZ = 30.0         # config.py LinkConfig.send_hz
SIM_SECONDS = 9.0
VFOV_HALF_DEG = 26.0     # OV2640 at SVGA, ~52 degrees vertical
STEP_AT_S = 0.5
TARGET_STEP_DEG = 12.0
"""The subject moves 12 degrees off the camera axis at t = 0.5 s — someone
taking a couple of steps at 3 m. Large enough to clear the 3% vision deadband
by a wide margin, small enough to stay inside the 25 degree attitude clamp so
the test measures the loop and not the limiter."""


TELEM_HZ = 20.0
"""config.h TELEM_HZ. The attitude used for compensation arrives on telemetry
at 20 Hz while frames arrive at 30 Hz, so the two are never quite paired —
which is what stops the compensation from being perfect."""


def run(delay_ms: float, frame: Airframe | None = None,
        attitude_comp: bool = True) -> dict:
    """One closed-loop tracking run at a given transport delay.

    The task is the real one: the subject moves off the camera axis and the
    aircraft must pitch to re-centre them. That closes the loop that actually
    carries the delay —

        pitch attitude -> where the subject appears in frame -> vision PD
          -> commanded pitch -> attitude

    — which is the loop the earlier version of this script failed to excite. A
    rate impulse alone is crushed by the 1 kHz inner loop within a few tenths
    of a degree, never clears the vision deadband, and produces a flat sweep
    that looks like a large stability margin and is really just an outer loop
    that was never switched on.
    """
    frame = frame or Airframe()
    dt = 1.0 / INNER_HZ
    steps = int(SIM_SECONDS * INNER_HZ)
    vision_every = int(round(INNER_HZ / VISION_HZ))

    drone = Quadrotor(frame=frame)
    rate_pitch = RatePid(RATE_KP, RATE_KI, RATE_KD, RATE_I_LIMIT, RATE_D_LPF_HZ)
    angle_pitch = AnglePid()
    vision = VisionPD(VIS_PITCH_KP, VIS_PITCH_KD, VIS_PITCH_DEADBAND,
                      MAX_ANGLE_DEG)
    link = DelayLine(delay_ms / 1000.0, 1.0 / VISION_HZ)

    hover = frame.hover_duty()
    pitch_sp_deg = 0.0
    target_deg = 0.0
    telem_pitch_deg = 0.0        # last attitude the laptop was told about
    telem_every = int(round(INNER_HZ / TELEM_HZ))
    t_hist, err_hist, pitch_hist = [], [], []

    for i in range(steps):
        t = i * dt
        if t >= STEP_AT_S:
            target_deg = TARGET_STEP_DEG

        # Telemetry is its own, slower stream. The attitude the laptop uses for
        # compensation is therefore up to one telemetry period stale relative
        # to the frame it is applied to, and that residual is what stops the
        # compensation being perfect.
        if i % telem_every == 0:
            telem_pitch_deg = drone.pitch_deg

        if i % vision_every == 0:
            # Where the subject appears, in half-frames. The firmware's
            # convention is pitch-positive = nose DOWN (see madgwick.h), so
            # pitching down raises the subject in frame: apparent bearing is
            # target + pitch. This coupling IS the feedback path under test.
            apparent_deg = target_deg + drone.pitch_deg

            # The frame and the attitude that goes with it travel together.
            delayed = link.push((apparent_deg, telem_pitch_deg))
            if delayed is not None:
                app_deg, att_deg = delayed
                if attitude_comp:
                    # vision/pursuit/geometry.py: subtract the camera's own
                    # attitude before computing any error, so that pitching to
                    # chase the subject is not itself read as the subject
                    # moving. Without this the loop is closed through the
                    # airframe with a DC gain of VIS_PITCH_KP/VFOV_HALF_DEG =
                    # 34/26 = 1.31 — above unity, so any delay oscillates.
                    app_deg = app_deg - att_deg
                # Positive error means the subject sits high, which is
                # corrected by pitching nose UP, i.e. a negative setpoint.
                pitch_sp_deg = -vision.update(app_deg / VFOV_HALF_DEG,
                                              1.0 / VISION_HZ)

        rate_sp = angle_pitch.update(pitch_sp_deg, drone.pitch_deg)
        u_pitch = rate_pitch.update(rate_sp, drone.q_dps, dt)
        drone.step(mix(hover, 0.0, u_pitch, 0.0), dt)

        t_hist.append(t)
        err_hist.append(target_deg + drone.pitch_deg)
        pitch_hist.append(drone.pitch_deg)

        if drone.diverged:
            return dict(delay_ms=delay_ms, growth=float("inf"),
                        peak_deg=float("inf"), settle_s=float("inf"),
                        diverged=True, t=np.array(t_hist),
                        err=np.array(err_hist), pitch=np.array(pitch_hist))

    return _metrics(delay_ms, np.array(t_hist), np.array(err_hist),
                    np.array(pitch_hist))


def _metrics(delay_ms: float, t: np.ndarray, err: np.ndarray,
             pitch: np.ndarray) -> dict:
    """Envelope growth of the tracking error after the step.

    A ratio of the same quantity at two times, so it is independent of the step
    size and far less sensitive to the inertia estimate than an absolute
    overshoot would be. Below 1 the oscillation is decaying; above 1 it is
    growing and the loop is unstable at this delay.
    """
    after = t >= STEP_AT_S
    t_a, e_a = t[after], err[after]
    if t_a.size == 0:
        return dict(delay_ms=delay_ms, growth=0.0, peak_deg=0.0,
                    settle_s=0.0, diverged=False, t=t, err=err, pitch=pitch)

    # Two things are measured, because one of them alone misleads.
    #
    # Oscillation is the deviation ABOUT the settled value, never the value
    # itself. The outer loop is proportional-only by deliberate design — an
    # integrator would wind up against the permanent error of chasing a moving
    # person — so it settles to a non-zero offset and stays there. At zero
    # delay this model settles to 5.20 degrees, which is exactly the
    # 12/(1 + 34/26) the gains predict on paper, and comparing raw RMS between
    # two windows would score that perfect result as 1.0 and call it marginal.
    #
    # Window length matters too: the unstable mode here has a period near two
    # seconds, so one-second windows straddle it and understate the amplitude.
    t0, t1 = t_a[0], t_a[-1]
    early = e_a[(t_a >= t0 + 1.0) & (t_a < t0 + 2.5)]
    late = e_a[t_a >= t1 - 1.5]

    osc_early = float(np.std(early)) if early.size else 0.0
    osc_late = float(np.std(late)) if late.size else 0.0
    growth = osc_late / osc_early if osc_early > 1e-9 else 0.0

    # The number a pilot would actually care about: how far the aircraft is
    # still swinging once the commanded transient is long finished.
    tail = e_a[t_a >= t1 - 3.0]
    residual = float(np.max(tail) - np.min(tail)) if tail.size else 0.0

    # Settling: last time the error left a +/-2 degree band around its own
    # steady-state offset.
    offset = float(np.median(tail)) if tail.size else 0.0
    outside = np.where(np.abs(e_a - offset) > 2.0)[0]
    settle = float(t_a[outside[-1]] - STEP_AT_S) if outside.size else 0.0

    return dict(delay_ms=delay_ms, growth=growth, residual_deg=residual,
                peak_deg=float(np.max(np.abs(e_a))), settle_s=settle,
                diverged=False, t=t, err=err, pitch=pitch)


def verdict(residual_deg: float, growth: float, diverged: bool) -> str:
    """Judged on residual swing, not on the growth ratio alone.

    Growth is a ratio of two windows and is noisy when both are near zero — a
    perfectly settled loop can score anything. The residual peak-to-peak is the
    physically meaningful quantity: how far the aircraft is still swinging
    about its tracking point seconds after the subject stopped moving. A degree
    or two is invisible; ten degrees is an aircraft visibly hunting.
    """
    if diverged:
        return "DIVERGED"
    if residual_deg < 0.5:
        return "settled"
    if residual_deg < 2.0:
        return "small ripple"
    if residual_deg < 6.0:
        return "OSCILLATING"
    if growth > 1.0:
        return "UNSTABLE (growing)"
    return "UNSTABLE"


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--trace", type=float, default=None, metavar="MS",
                    help="trace a single run at this delay instead of sweeping")
    ap.add_argument("--max-ms", type=float, default=200.0)
    ap.add_argument("--step-ms", type=float, default=17.0,
                    help="default is one 30 Hz frame period")
    ap.add_argument("--tau-motor", type=float, default=None, metavar="MS",
                    help="override the motor time constant, to see how much "
                         "the answer depends on the least certain number")
    ap.add_argument("--out", default=None,
                    help="where to write the plot (default docs/media/)")
    args = ap.parse_args()

    frame = Airframe()
    if args.tau_motor is not None:
        frame.tau_motor_s = args.tau_motor / 1000.0

    print(f"airframe   {frame.mass_kg*1000:.0f} g, T:W "
          f"{frame.thrust_to_weight():.2f}, hover duty "
          f"{frame.hover_duty()*100:.0f}%")
    print(f"motor tau  {frame.tau_motor_s*1000:.0f} ms  (the least certain "
          f"number here — try --tau-motor 25 and 45)")
    print(f"loops      vision {VISION_HZ:.0f} Hz delayed -> angle P + rate PID "
          f"{INNER_HZ:.0f} Hz on-board\n")

    if args.trace is not None:
        for comp in (False, True):
            r = run(args.trace, frame=frame, attitude_comp=comp)
            tag = "with" if comp else "without"
            print(f"{tag:>7} attitude compensation, {args.trace:.0f} ms: "
                  f"residual {r['residual_deg']:.1f} deg p-p, peak "
                  f"{r['peak_deg']:.1f} deg, settle {r['settle_s']:.2f} s "
                  f"-> {verdict(r['residual_deg'], r['growth'], r['diverged'])}")
        return 0

    delays = np.arange(0.0, args.max_ms + 1e-9, args.step_ms)
    off = [run(float(d), frame=frame, attitude_comp=False) for d in delays]
    on = [run(float(d), frame=frame, attitude_comp=True) for d in delays]

    print("Residual oscillation after the subject stops moving, in degrees")
    print("peak-to-peak. The two columns are the same aircraft and the same")
    print("gains; the only difference is whether vision/pursuit/geometry.py")
    print("subtracts the camera's own attitude before computing image error.\n")
    print(f"{'delay':>7} {'frames':>7} | {'NO comp':>9} {'verdict':<20} | "
          f"{'WITH comp':>9} {'verdict':<14}")
    print("-" * 84)

    b_off = b_on = None
    for a, b in zip(off, on):
        fr = int(round(a["delay_ms"] / (1000.0 / VISION_HZ)))
        va = verdict(a["residual_deg"], a["growth"], a["diverged"])
        vb = verdict(b["residual_deg"], b["growth"], b["diverged"])
        print(f"{a['delay_ms']:>7.0f} {fr:>7} | {a['residual_deg']:>9.1f} "
              f"{va:<20} | {b['residual_deg']:>9.2f} {vb:<14}")
        if b_off is None and a["residual_deg"] >= 2.0:
            b_off = a["delay_ms"]
        if b_on is None and b["residual_deg"] >= 2.0:
            b_on = b["delay_ms"]

    measured = 20.0    # tools/e2e_loop.py, loopback
    print()
    print(f"Boundary WITHOUT compensation  "
          f"{('~%.0f ms' % b_off) if b_off else '> %.0f ms' % args.max_ms}")
    print(f"Boundary WITH compensation     "
          f"{('~%.0f ms' % b_on) if b_on else '> %.0f ms' % args.max_ms}")
    print(f"Measured round trip            {measured:.0f} ms "
          f"(loopback, tools/e2e_loop.py)")
    print()
    if b_off and not b_on:
        print(f"Reading: uncompensated, the image-feedback path has a DC gain of")
        print(f"VIS_PITCH_KP / VFOV_HALF_DEG = {VIS_PITCH_KP:.0f}/{VFOV_HALF_DEG:.0f}"
              f" = {VIS_PITCH_KP/VFOV_HALF_DEG:.2f}. Above unity, so pitching to")
        print(f"chase the subject moves them further in frame than the error that")
        print(f"prompted it, and the loop oscillates at the FIRST frame of delay.")
        print(f"Attitude compensation opens that path and buys back an order of")
        print(f"magnitude of delay tolerance. This quantifies a change that was")
        print(f"previously justified only by argument.")
        print()
        print(f"Caveat: the subject here is stationary once stepped, so perfect")
        print(f"compensation fully opens the loop and the remaining margin is")
        print(f"large. Against a continuously MOVING subject the loop closes")
        print(f"through the tracker and the true margin will be smaller.")

    _plot(delays, off, on, b_off, frame, args.out)
    return 0


def _plot(delays, off, on, boundary, frame, out) -> None:
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("\n(matplotlib not installed — skipping the plot)")
        return

    root = os.path.dirname(os.path.dirname(os.path.dirname(
        os.path.abspath(__file__))))
    out = out or os.path.join(root, "docs", "media", "lag_stability.png")
    os.makedirs(os.path.dirname(out), exist_ok=True)

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9.5, 8.5))

    ax1.plot(delays, [r["residual_deg"] for r in off], "o-", lw=2,
             color="#d62728", label="no attitude compensation")
    ax1.plot(delays, [r["residual_deg"] for r in on], "s-", lw=2,
             color="#2ca02c", label="with attitude compensation (geometry.py)")
    ax1.axhline(2.0, color="#666", ls="--", lw=1.2,
                label="2 deg p-p — visible hunting")
    ax1.axvline(20, color="#1f77b4", ls=":", lw=1.5,
                label="20 ms measured round trip (e2e_loop.py)")
    if boundary:
        ax1.annotate("unstable from the first\nframe of delay",
                     xy=(boundary, 7.6), xytext=(boundary + 25, 16),
                     arrowprops=dict(arrowstyle="->", color="#d62728"),
                     color="#d62728", fontsize=9)
    ax1.set_xlabel("transport delay in the vision loop (ms)")
    ax1.set_ylabel("residual oscillation (deg peak-to-peak)")
    ax1.set_title("PURSUIT-1 pitch cascade: does the outer loop survive the "
                  "link delay?\n"
                  f"rigid-body model, ESTIMATED mass properties, motor tau "
                  f"{frame.tau_motor_s*1000:.0f} ms — see quadrotor.py",
                  fontsize=11)
    ax1.grid(alpha=0.3)
    ax1.legend(fontsize=8, loc="upper left")

    for r, c, lab in ((off[2], "#d62728", "no comp"),
                      (on[2], "#2ca02c", "with comp")):
        ax2.plot(r["t"], r["err"], lw=1.5, color=c,
                 label=f"{lab}, {r['delay_ms']:.0f} ms delay")
    ax2.axvline(STEP_AT_S, color="#666", ls=":", lw=1,
                label=f"{TARGET_STEP_DEG:.0f} deg target step")
    ax2.axhline(0, color="#999", lw=0.8)
    ax2.set_xlabel("time (s)")
    ax2.set_ylabel("tracking error (deg off camera axis)")
    ax2.set_title("Step response — the subject moves, the aircraft re-centres "
                  "them", fontsize=11)
    ax2.grid(alpha=0.3)
    ax2.legend(fontsize=8)

    fig.tight_layout()
    fig.savefig(out, dpi=130)
    print(f"plot -> {os.path.relpath(out, root)}")


if __name__ == "__main__":
    sys.exit(main())
