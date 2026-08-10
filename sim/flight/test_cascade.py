"""Tests for the flight simulation.

Two jobs. The first is ordinary correctness. The second matters more: this
package DUPLICATES the firmware's gains in Python, and a duplicate drifts. The
wire protocol avoids that problem by generating one side from the other; there
is no such guard here, so these tests pin the numbers and the structure loudly
enough that a retune of pid.cpp shows up as a red test rather than as a
simulation that quietly stops describing the aircraft.

    vision/.venv/bin/python -m pytest sim/flight/test_cascade.py -q
"""
from __future__ import annotations

import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__)))))

from sim.flight import cascade as C          # noqa: E402
from sim.flight.lag_sweep import run         # noqa: E402
from sim.flight.quadrotor import Airframe, Quadrotor  # noqa: E402


# --------------------------------------------------------------- drift guard
def test_ported_gains_still_match_the_firmware():
    """If this fails, someone retuned the firmware and this model is stale.

    Re-read the named files and update BOTH, or the sweep is describing an
    aircraft that no longer exists.
    """
    # firmware/src/main.cpp
    assert C.ANGLE_KP == 6.0
    # firmware/include/config.h
    assert C.MAX_RATE_DPS == 180.0
    assert C.MAX_ANGLE_DEG == 25.0
    # firmware/src/pid.cpp, default_roll_rate_gains()
    assert (C.RATE_KP, C.RATE_KI, C.RATE_KD) == (0.0016, 0.012, 0.000045)
    assert C.RATE_I_LIMIT == 0.25
    assert C.RATE_D_LPF_HZ == 80.0
    # vision/pursuit/config.py, ControlConfig
    assert (C.VIS_PITCH_KP, C.VIS_PITCH_KD) == (34.0, 4.5)
    assert (C.VIS_YAW_KP, C.VIS_YAW_KD) == (95.0, 9.0)


# ------------------------------------------------------------------- airframe
def test_hover_duty_agrees_with_the_mass_budget():
    """Thrust is quadratic in duty, so hover is not weight/max-thrust.

    The README predicts hover at 60-65%. Getting ~69% from an independent
    route — mass, thrust constant and a square law — is a real consistency
    check on both numbers. Treating thrust as linear would give 47% and
    silently halve the loop gain at the operating point.
    """
    assert 0.60 <= Airframe().hover_duty() <= 0.75


def test_hovering_holds_altitude_authority_in_reserve():
    assert Airframe().thrust_to_weight() > 1.5


def test_a_level_airframe_at_hover_duty_does_not_rotate():
    d = Quadrotor()
    hover = d.frame.hover_duty()
    for _ in range(1000):
        d.step(np.full(4, hover), 0.001)
    assert abs(d.roll_deg) < 1e-6
    assert abs(d.pitch_deg) < 1e-6
    assert not d.diverged


def test_motor_lag_is_step_size_independent():
    """Exact discretisation, not forward Euler — because the step size is one
    of the things being swept, and a lag that changes with dt would make the
    sweep measure the integrator instead of the aircraft."""
    tau = Airframe().tau_motor_s
    ends = []
    # Step sizes chosen to divide tau exactly. Using one that does not (0.002
    # into 0.035 truncates to 17 steps, i.e. 34 ms) makes this test fail on
    # its own rounding rather than on anything the model did.
    for dt in (0.0001, 0.0005, 0.001):
        n = round(tau / dt)
        assert abs(n * dt - tau) < 1e-12, "pick a dt that divides tau"
        d = Quadrotor()
        for _ in range(n):                   # exactly one time constant
            d.step(np.ones(4), dt)
        ends.append(d.duty[0])
    assert max(ends) - min(ends) < 1e-6
    assert ends[0] == pytest.approx(1.0 - np.exp(-1.0), abs=1e-6)


# -------------------------------------------------------------------- mixer
def test_mixer_preserves_the_differential_when_it_desaturates():
    """The whole point of common-mode desaturation. Clipping one motor breaks
    the roll/pitch/yaw the controller asked for, so a pure throttle command
    starts producing roll — the aircraft tips exactly when it is working
    hardest."""
    m = C.mix(0.95, 0.30, 0.0, 0.0)
    assert np.all(m >= 0.0) and np.all(m <= 1.0)
    # right pair minus left pair should still be the commanded 2*roll
    got = ((m[1] + m[3]) - (m[0] + m[2])) / 2.0
    assert got == pytest.approx(0.60, abs=1e-6)


def test_pure_throttle_produces_no_moment():
    m = C.mix(0.5, 0.0, 0.0, 0.0)
    assert np.allclose(m, 0.5)


# --------------------------------------------------------------------- PID
def test_derivative_acts_on_measurement_so_a_setpoint_step_does_not_kick():
    """Differentiating the error makes a setpoint step produce an unbounded
    spike. The aircraft's real rotation rate cannot step, so differentiating
    that instead gives the same damping with no kick."""
    pid = C.RatePid(C.RATE_KP, C.RATE_KI, C.RATE_KD, C.RATE_I_LIMIT,
                    C.RATE_D_LPF_HZ)
    pid.update(0.0, 0.0, 0.001)
    kick = pid.update(500.0, 0.0, 0.001)     # huge setpoint step, still tracking
    assert abs(kick) < 1.0                   # bounded, no division by dt


def test_integrator_does_not_wind_up_while_saturated():
    pid = C.RatePid(C.RATE_KP, C.RATE_KI, C.RATE_KD, C.RATE_I_LIMIT,
                    C.RATE_D_LPF_HZ)
    for _ in range(5000):
        pid.update(2000.0, 0.0, 0.001)       # far beyond any real authority
    assert abs(pid.integral) <= C.RATE_I_LIMIT + 1e-9


def test_angle_loop_output_is_clamped_to_the_rate_limit():
    a = C.AnglePid()
    assert a.update(1000.0, 0.0) == pytest.approx(C.MAX_RATE_DPS)
    assert a.update(-1000.0, 0.0) == pytest.approx(-C.MAX_RATE_DPS)


# ------------------------------------------------------------- delay line
def test_delay_line_quantises_to_whole_frames():
    """The vision loop can only act at 30 Hz, so delay is quantised to frame
    periods. This is why the sweep shows steps rather than a smooth curve, and
    it is physical rather than an artefact."""
    dl = C.DelayLine(0.033, 1.0 / 30.0)
    assert dl.push("a") is None
    assert dl.push("b") == "a"

    zero = C.DelayLine(0.0, 1.0 / 30.0)
    assert zero.push("a") == "a"


# ------------------------------------------------- the headline result
def test_zero_delay_settles_to_the_analytic_steady_state():
    """A P-only outer loop leaves a permanent offset, and the value is
    predictable on paper:

        e_ss = step / (1 + KP/VFOV_HALF) = 12 / (1 + 34/26) = 5.20 deg

    Matching it is a genuine end-to-end check that the plant, the mixer, both
    inner loops and the sign conventions all agree with each other. It is also
    the reason the first version of the stability metric was wrong: it scored
    this perfect result as 'marginal', because RMS cannot tell a constant
    offset from a constant-amplitude oscillation.
    """
    r = run(0.0, attitude_comp=False)
    settled = r["err"][-1]
    expected = 12.0 / (1.0 + C.VIS_PITCH_KP / 26.0)
    assert settled == pytest.approx(expected, abs=0.05)


def test_uncompensated_loop_oscillates_at_one_frame_of_delay():
    """The finding. Uncompensated image feedback has DC gain 34/26 = 1.31, so
    the loop is unstable at the very first frame of transport delay — well
    inside the 20 ms round trip actually measured by tools/e2e_loop.py."""
    assert C.VIS_PITCH_KP / 26.0 > 1.0        # the cause
    assert run(0.0, attitude_comp=False)["residual_deg"] < 0.5
    assert run(34.0, attitude_comp=False)["residual_deg"] > 5.0


def test_attitude_compensation_is_what_makes_the_design_work():
    """Same airframe, same gains; the only difference is whether
    vision/pursuit/geometry.py subtracts the camera's own attitude before
    computing image error."""
    for delay_ms in (34.0, 102.0, 187.0):
        assert run(delay_ms, attitude_comp=True)["residual_deg"] < 2.0
        assert run(delay_ms, attitude_comp=False)["residual_deg"] > 5.0
