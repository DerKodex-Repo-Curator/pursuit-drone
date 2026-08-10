"""Authority arbitration.

This is the file that decides whether a spinning propeller is the operator's
decision or the software's, so the four rules in arbiter.py each get tested
directly, plus the combinations where an interlock could be bypassed.
"""
from __future__ import annotations

import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from pursuit.arbiter import Arbiter, FlightMode                  # noqa: E402
from pursuit.config import ControlConfig                          # noqa: E402
from pursuit.pilot import KeyboardPilot, PilotInput, deadzone     # noqa: E402
from pursuit.protocol import (FLAG_ALT_HOLD, FLAG_ARMED,          # noqa: E402
                              FLAG_FAILSAFE, FLAG_LOCKED, Command)

DT = 1 / 30


def held(**kw):
    kw.setdefault("deadman", True)
    return PilotInput(connected=True, **kw)


def vision_cmd(**kw):
    kw.setdefault("flags", FLAG_ALT_HOLD | FLAG_LOCKED)
    kw.setdefault("alt_sp", 1.4)
    return Command(seq=99, **kw)


@pytest.fixture
def arb():
    return Arbiter(ControlConfig())


# ------------------------------------------------- rule 1: no deadman, no motors

def test_disarmed_without_the_deadman(arb):
    for mode in FlightMode:
        arb.request_mode(mode)
        cmd = arb.update(PilotInput(roll=1.0, pitch=1.0, throttle=1.0,
                                    deadman=False, connected=True),
                         vision_cmd(yaw_rate=90.0), DT)
        assert not (cmd.flags & FLAG_ARMED), f"armed with no deadman in {mode}"
        assert cmd.roll == 0.0 and cmd.pitch == 0.0 and cmd.yaw_rate == 0.0
        assert cmd.flags & FLAG_FAILSAFE


def test_deadman_is_held_not_latched(arb):
    arb.request_mode(FlightMode.MANUAL)
    assert arb.update(held(throttle=0.5), None, DT).flags & FLAG_ARMED
    # Releasing it disarms on the very next frame — no timeout, no grace.
    assert not (arb.update(PilotInput(throttle=0.5, deadman=False,
                                      connected=True), None, DT).flags
                & FLAG_ARMED)


def test_releasing_the_deadman_resets_altitude(arb):
    """Re-arming must not resume a setpoint from a previous flight, which
    could be metres from where the aircraft now sits."""
    arb.request_mode(FlightMode.ASSIST)
    for _ in range(60):
        arb.update(held(throttle=1.0), None, DT)
    assert arb.alt_sp > ControlConfig().alt_default

    arb.update(PilotInput(deadman=False, connected=True), None, DT)
    assert arb.alt_sp == pytest.approx(ControlConfig().alt_default)


# --------------------------------------------- rule 2: touching sticks wins

@pytest.mark.parametrize("axis", ["roll", "pitch", "yaw", "throttle"])
def test_any_stick_takes_control_back_from_follow(arb, axis):
    arb.request_mode(FlightMode.FOLLOW)
    arb.update(held(), vision_cmd(yaw_rate=40.0), DT)
    assert arb.mode is FlightMode.FOLLOW

    cmd = arb.update(held(**{axis: 0.9}), vision_cmd(yaw_rate=40.0), DT)
    assert arb.mode is FlightMode.ASSIST, f"{axis} did not take control"
    assert cmd.yaw_rate != pytest.approx(40.0), "vision still had the controls"


def test_override_latches(arb):
    """Letting go must NOT hand control back to the autonomy by itself."""
    arb.request_mode(FlightMode.FOLLOW)
    arb.update(held(roll=0.9), vision_cmd(), DT)
    assert arb.mode is FlightMode.ASSIST

    for _ in range(120):
        arb.update(held(), vision_cmd(yaw_rate=50.0), DT)
    assert arb.mode is FlightMode.ASSIST, "autonomy resumed on its own"


def test_small_stick_movement_does_not_override(arb):
    """Noise and drift must not knock it out of FOLLOW."""
    arb.request_mode(FlightMode.FOLLOW)
    for _ in range(30):
        arb.update(held(roll=0.05, yaw=-0.04), vision_cmd(yaw_rate=30.0), DT)
    assert arb.mode is FlightMode.FOLLOW


def test_override_inherits_the_altitude(arb):
    """Taking over must not command a step change in height."""
    arb.request_mode(FlightMode.FOLLOW)
    arb.update(held(), vision_cmd(alt_sp=1.9), DT)
    arb.update(held(pitch=0.8), vision_cmd(alt_sp=1.9), DT)
    assert arb.mode is FlightMode.ASSIST
    assert arb.alt_sp == pytest.approx(1.9, abs=1e-6)


# ------------------------------------------ rule 3: vision only in FOLLOW

@pytest.mark.parametrize("mode", [FlightMode.MANUAL, FlightMode.ASSIST])
def test_vision_is_ignored_outside_follow(arb, mode):
    arb.request_mode(mode)
    cmd = arb.update(held(), vision_cmd(roll=9.0, pitch=9.0, yaw_rate=99.0), DT)
    assert cmd.yaw_rate == pytest.approx(0.0)
    assert cmd.pitch == pytest.approx(0.0)
    assert not (cmd.flags & FLAG_LOCKED)


def test_vision_commands_pass_through_in_follow(arb):
    arb.request_mode(FlightMode.FOLLOW)
    cmd = arb.update(held(), vision_cmd(roll=1.0, pitch=-3.0, yaw_rate=45.0), DT)
    assert cmd.yaw_rate == pytest.approx(45.0)
    assert cmd.pitch == pytest.approx(-3.0)
    assert cmd.flags & FLAG_ARMED and cmd.flags & FLAG_LOCKED


def test_follow_without_a_vision_command_holds_level(arb):
    arb.request_mode(FlightMode.FOLLOW)
    cmd = arb.update(held(), None, DT)
    assert cmd.roll == 0.0 and cmd.pitch == 0.0 and cmd.yaw_rate == 0.0
    assert cmd.flags & FLAG_FAILSAFE


# ----------------------------------------------------------------- modes --

def test_starts_in_manual():
    """Booting into autonomy means it can fly away before anyone agreed."""
    assert Arbiter(ControlConfig()).mode is FlightMode.MANUAL


def test_manual_maps_sticks_to_limits(arb):
    cfg = ControlConfig()
    arb.request_mode(FlightMode.MANUAL)
    cmd = arb.update(held(roll=1.0, pitch=1.0, yaw=1.0, throttle=1.0), None, DT)
    assert cmd.roll == pytest.approx(cfg.roll_max)
    assert cmd.pitch == pytest.approx(cfg.pitch_max)
    assert cmd.yaw_rate == pytest.approx(cfg.yaw_max)
    # In MANUAL the field is throttle 0..1, so no altitude-hold flag.
    assert not (cmd.flags & FLAG_ALT_HOLD)
    assert cmd.alt_sp == pytest.approx(1.0)   # full stick = full throttle


def test_manual_stick_is_over_driven_safely(arb):
    arb.request_mode(FlightMode.MANUAL)
    cmd = arb.update(held(roll=50.0, pitch=-50.0, yaw=99.0), None, DT)
    cfg = ControlConfig()
    assert abs(cmd.roll) <= cfg.roll_max + 1e-6
    assert abs(cmd.pitch) <= cfg.pitch_max + 1e-6
    assert abs(cmd.yaw_rate) <= cfg.yaw_max + 1e-6


def test_manual_neutral_stick_is_zero_throttle(arb):
    """A spring-centred stick rests at centre. Arming with it there must
    command a STOPPED aircraft, not a half-throttle one."""
    arb.request_mode(FlightMode.MANUAL)
    cmd = arb.update(held(throttle=0.0), None, DT)
    assert cmd.flags & FLAG_ARMED
    assert cmd.alt_sp == pytest.approx(0.0, abs=1e-6)
    # Pulling DOWN from centre must also stay at zero, not go negative.
    cmd = arb.update(held(throttle=-0.8), None, DT)
    assert cmd.alt_sp == pytest.approx(0.0, abs=1e-6)
    # And pushing up still reaches full throttle.
    cmd = arb.update(held(throttle=1.0), None, DT)
    assert cmd.alt_sp == pytest.approx(1.0, abs=1e-6)


def test_assist_throttle_is_a_climb_rate(arb):
    cfg = ControlConfig()
    arb.request_mode(FlightMode.ASSIST)
    start = arb.alt_sp

    for _ in range(30):
        cmd = arb.update(held(throttle=1.0), None, DT)
    assert cmd.alt_sp > start
    assert cmd.flags & FLAG_ALT_HOLD

    # Centre stick holds, which is what makes assisted flight easy.
    held_at = cmd.alt_sp
    for _ in range(60):
        cmd = arb.update(held(throttle=0.0), None, DT)
    assert cmd.alt_sp == pytest.approx(held_at, abs=1e-6)

    for _ in range(200):
        cmd = arb.update(held(throttle=-1.0), None, DT)
    assert cmd.alt_sp == pytest.approx(cfg.alt_min, abs=1e-6)


def test_assist_altitude_is_bounded(arb):
    cfg = ControlConfig()
    arb.request_mode(FlightMode.ASSIST)
    for _ in range(600):
        cmd = arb.update(held(throttle=1.0), None, DT)
    assert cmd.alt_sp <= cfg.alt_max + 1e-6


def test_mode_request_from_the_pilot(arb):
    arb.update(held(mode_request=FlightMode.FOLLOW), vision_cmd(), DT)
    assert arb.mode is FlightMode.FOLLOW
    arb.update(held(mode_request=FlightMode.MANUAL), vision_cmd(), DT)
    assert arb.mode is FlightMode.MANUAL


def test_every_command_is_transmittable(arb):
    """Whatever the arbiter emits must survive packing — no NaN, no infinity.

    NaN is the one that matters. `x < lo` and `x > hi` are both false for NaN,
    so a plain two-sided clamp passes it through untouched; it would then reach
    the packer, be refused, and silently stop the entire command stream rather
    than moving one axis wrongly.
    """
    import math
    for mode in FlightMode:
        arb.request_mode(mode)
        for pilot in (held(), held(roll=1, pitch=-1, yaw=1, throttle=1),
                      PilotInput(deadman=False, connected=True),
                      held(roll=math.inf), held(pitch=-math.inf),
                      held(yaw=math.nan),
                      held(roll=math.nan, pitch=math.nan,
                           yaw=math.nan, throttle=math.nan)):
            for vision in (None, vision_cmd(yaw_rate=120.0)):
                cmd = arb.update(pilot, vision, DT)
                cmd.pack()


def test_a_stuck_nan_axis_reads_as_neutral(arb):
    """An unknowable axis must be treated as centred, not as a command."""
    arb.request_mode(FlightMode.MANUAL)
    cmd = arb.update(held(roll=float("nan"), yaw=float("nan")), None, DT)
    assert cmd.roll == 0.0 and cmd.yaw_rate == 0.0
    assert cmd.flags & FLAG_ARMED, "a bad axis should not disarm the aircraft"


def test_nan_does_not_trigger_a_false_override(arb):
    """`abs(nan) > threshold` is false, so NaN must not silently *prevent* an
    override either — it is simply not an input."""
    arb.request_mode(FlightMode.FOLLOW)
    arb.update(held(roll=float("nan")), vision_cmd(), DT)
    assert arb.mode is FlightMode.FOLLOW
    arb.update(held(roll=0.9), vision_cmd(), DT)
    assert arb.mode is FlightMode.ASSIST


# ----------------------------------------------------------------- pilot --

def test_deadzone_is_continuous():
    assert deadzone(0.05, 0.08) == 0.0
    assert deadzone(1.0, 0.08) == pytest.approx(1.0)
    assert deadzone(-1.0, 0.08) == pytest.approx(-1.0)
    assert deadzone(0.0800001, 0.08) == pytest.approx(0.0, abs=1e-4)


def test_keyboard_deadman_expires():
    """A keyboard gives events, not held state, so the deadman must decay —
    otherwise one press would arm the aircraft indefinitely."""
    kp = KeyboardPilot(hold_s=0.3)
    kp.poll(now=0.0, dt=DT)
    kp.key(ord(" "))
    assert kp.poll(now=0.0, dt=DT).deadman
    assert kp.poll(now=0.2, dt=DT).deadman
    assert not kp.poll(now=0.5, dt=DT).deadman


def test_keyboard_axes_return_to_centre():
    kp = KeyboardPilot()
    for _ in range(6):
        kp.key(ord("d"))
    assert kp.poll(now=0.0, dt=DT).roll > 0.1
    for _ in range(40):
        out = kp.poll(now=1.0, dt=DT)
    assert out.roll == pytest.approx(0.0, abs=1e-6)


class _FakePad:
    """HidGamepad with the HID layer replaced, so report decoding is testable
    without a controller plugged in."""

    def __init__(self, profile):
        from pursuit.pilot import HidGamepad
        self.g = HidGamepad.__new__(HidGamepad)
        self.g.profile = profile
        self.g.dz = 0.08
        self.g.dev = None
        self.g.last_report = b""

    def decode(self, report: bytes):
        from pursuit.pilot import PilotInput
        from pursuit.arbiter import FlightMode as FM
        g, p = self.g, self.g.profile
        mode = None
        if p.mode_byte < len(report):
            bits = report[p.mode_byte]
            if bits & p.mode_manual:
                mode = FM.MANUAL
            elif bits & p.mode_assist:
                mode = FM.ASSIST
            elif bits & p.mode_follow:
                mode = FM.FOLLOW
        deadman = False
        if p.deadman_axis is not None:
            deadman = g._trigger(report, p.deadman_axis, p.deadman_axis_bits,
                                 p.deadman_full_scale) >= p.deadman_thresh
        return PilotInput(
            roll=g._axis(report, p.axis_roll, p.axis_bits, False),
            pitch=g._axis(report, p.axis_pitch, p.axis_bits, p.invert_pitch),
            yaw=g._axis(report, p.axis_yaw, p.axis_bits, False),
            throttle=g._axis(report, p.axis_throttle, p.axis_bits,
                             p.invert_throttle),
            deadman=deadman, mode_request=mode, connected=True)


def xbox_report(lx=0x8000, ly=0x8000, rx=0x8000, ry=0x8000, lt=0, rt=0,
                buttons=0):
    """An Xbox Wireless input report: id, four 16-bit sticks, two triggers,
    hat, buttons."""
    out = bytearray(16)
    out[0] = 0x01
    for off, val in ((1, lx), (3, ly), (5, rx), (7, ry), (9, lt), (11, rt)):
        out[off] = val & 0xFF
        out[off + 1] = (val >> 8) & 0xFF
    out[14] = buttons
    return bytes(out)


def xbox_profile():
    from pursuit.pilot import PROFILES
    return next(p for p in PROFILES if p.vendor == 0x045E)


def test_xbox_sticks_are_decoded_as_16_bit():
    """The bug this guards against: reading a 16-bit stick one byte at a time
    gets the LOW byte, which sweeps 0-255 repeatedly across the stick's travel.
    The drone would twitch violently while the code looked perfectly happy."""
    pad = _FakePad(xbox_profile())

    centred = pad.decode(xbox_report())
    assert abs(centred.roll) < 1e-6 and abs(centred.pitch) < 1e-6

    full_right = pad.decode(xbox_report(lx=0xFFFF))
    assert full_right.roll > 0.9

    full_left = pad.decode(xbox_report(lx=0x0000))
    assert full_left.roll < -0.9

    # Halfway: the low byte here is 0x00, so a byte-wise reader would call
    # this hard-left instead of quarter-right.
    quarter = pad.decode(xbox_report(lx=0xC000))
    assert 0.3 < quarter.roll < 0.7, \
        f"16-bit decode wrong: got {quarter.roll:.3f}"


def test_xbox_low_byte_sweep_does_not_move_the_axis():
    """Walk the low byte across its whole range with the high byte fixed at
    centre. A correct 16-bit reader barely moves; a byte-wise one saturates."""
    pad = _FakePad(xbox_profile())
    values = [pad.decode(xbox_report(lx=0x8000 | lo)).roll
              for lo in range(0, 256, 16)]
    assert max(abs(v) for v in values) < 0.02, \
        f"low byte alone moved the axis to {max(abs(v) for v in values):.3f}"


def test_xbox_pitch_is_inverted():
    """Stick forward must be positive pitch, and HID reports 'up' as low."""
    pad = _FakePad(xbox_profile())
    assert pad.decode(xbox_report(ly=0x0000)).pitch > 0.9
    assert pad.decode(xbox_report(ly=0xFFFF)).pitch < -0.9


def test_xbox_trigger_deadman():
    """Triggers are 10-bit in a 16-bit field: normalising by 65535 would make
    a fully squeezed trigger read 0.016 and the deadman would never engage."""
    pad = _FakePad(xbox_profile())
    assert not pad.decode(xbox_report(rt=0)).deadman
    assert not pad.decode(xbox_report(rt=100)).deadman
    assert pad.decode(xbox_report(rt=1023)).deadman
    assert pad.decode(xbox_report(rt=600)).deadman


def test_short_report_does_not_crash():
    """A truncated or unexpected report must read as neutral, not raise."""
    pad = _FakePad(xbox_profile())
    for n in (0, 1, 4, 9, 13):
        out = pad.decode(xbox_report()[:n])
        assert out.roll == 0.0 and out.pitch == 0.0
        assert not out.deadman


def test_keyboard_mode_keys():
    kp = KeyboardPilot()
    kp.key(ord("3"))
    assert kp.poll(0.0, DT).mode_request is FlightMode.FOLLOW
    # Consumed once, not repeated every frame.
    assert kp.poll(0.0, DT).mode_request is None
