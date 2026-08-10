"""Who is flying the aircraft, and when.

Three modes over one link:

    MANUAL   you fly it, entirely. Throttle is throttle.
    ASSIST   you fly it, the drone holds altitude off the ToF.
    FOLLOW   vision flies it, while you hold the deadman.

Four rules, in priority order. They are short because a safety interlock you
cannot recite from memory is one you cannot trust:

1. **No deadman, no motors.** The arm bit is set only while the pilot is
   actively holding the control. Not a toggle — a toggle means a laptop left
   running in another room is still authorising flight.

2. **Touching the sticks takes control.** Any deflection past a threshold
   during FOLLOW drops immediately to ASSIST and *latches* there. You do not
   have to find a mode button while the drone is doing something wrong; the
   instinctive reaction is the correct one.

3. **Vision only ever commands in FOLLOW.** In the other modes its output is
   computed (so the HUD still shows what it would do) and discarded.

4. **Mode changes are explicit** — a button press, or rule 2. Nothing else
   moves the aircraft between modes on its own.

Rule 2 is deliberately all-or-nothing rather than per-axis. Blending pilot yaw
with autonomous pitch sounds clever and produces an aircraft where neither
party knows who is responsible for what. "Touch the sticks and you are flying"
is one sentence, and it is testable.
"""
from __future__ import annotations

import math
import time
from enum import Enum

from .config import ControlConfig
from .protocol import (FLAG_ALT_HOLD, FLAG_ARMED, FLAG_FAILSAFE, FLAG_LOCKED,
                       Command)


class FlightMode(Enum):
    MANUAL = "MANUAL"
    ASSIST = "ASSIST"
    FOLLOW = "FOLLOW"


def _clamp(x: float, lo: float, hi: float) -> float:
    """Bound a value, treating a non-finite input as zero.

    The NaN case is not theoretical padding. `x < lo` and `x > hi` are both
    false for NaN, so a plain two-sided clamp returns it unchanged — a stuck
    HID axis or a bad read would carry NaN all the way to the packer, which
    then refuses the packet and stops the whole command stream. Zero is the
    right answer for an axis whose value is unknowable: neutral.
    """
    if not math.isfinite(x):
        return 0.0
    return lo if x < lo else hi if x > hi else x


class Arbiter:
    """Combines pilot input and vision output into the command actually sent."""

    def __init__(self, cfg: ControlConfig,
                 override_threshold: float = 0.18,
                 start_mode: FlightMode = FlightMode.MANUAL):
        self.cfg = cfg
        # How far a stick must move during FOLLOW to count as "the pilot has
        # taken over". Comfortably outside the deadzone and any drift, well
        # inside a deliberate input.
        self.override_threshold = override_threshold

        # MANUAL by default, deliberately. A system that boots into autonomy
        # is one that can fly away before anyone has agreed that it should.
        self.mode = start_mode
        self.seq = 0
        self.alt_sp = cfg.alt_default
        self._t0 = time.monotonic()
        self.last_override = 0.0
        self.overrides = 0

    def reset(self) -> None:
        self.mode = FlightMode.MANUAL
        self.alt_sp = self.cfg.alt_default

    def _t_ms(self) -> int:
        return int((time.monotonic() - self._t0) * 1000.0) & 0xFFFFFFFF

    # ----------------------------------------------------------------- mode --
    def request_mode(self, mode: FlightMode) -> None:
        self.mode = mode

    def _pilot_is_flying(self, pilot) -> bool:
        # Sanitised rather than compared raw: `nan > threshold` is false, so a
        # raw comparison happens to fail safe, but only by accident. Making it
        # explicit means the behaviour survives someone rewriting this line.
        return max(abs(_clamp(pilot.roll, -1.0, 1.0)),
                   abs(_clamp(pilot.pitch, -1.0, 1.0)),
                   abs(_clamp(pilot.yaw, -1.0, 1.0)),
                   abs(_clamp(pilot.throttle, -1.0, 1.0))
                   ) > self.override_threshold

    # --------------------------------------------------------------- update --
    def update(self, pilot, vision: Command | None, dt: float) -> Command:
        cfg = self.cfg
        self.seq += 1
        dt = _clamp(dt, 1e-3, 0.25)

        if pilot.mode_request is not None:
            self.mode = pilot.mode_request

        # --- rule 1: deadman ---------------------------------------------
        # Checked before anything else and independently of mode, so no
        # combination of state can produce an armed command without it.
        if not pilot.deadman:
            # Altitude is reset too: re-arming should not resume a setpoint
            # from a previous flight, metres away from where it now sits.
            self.alt_sp = cfg.alt_default
            return Command(seq=self.seq, t_ms=self._t_ms(),
                           roll=0.0, pitch=0.0, yaw_rate=0.0,
                           alt_sp=cfg.alt_default,
                           flags=FLAG_ALT_HOLD | FLAG_FAILSAFE)

        # --- rule 2: the pilot touching the sticks wins -------------------
        if self.mode is FlightMode.FOLLOW and self._pilot_is_flying(pilot):
            self.mode = FlightMode.ASSIST
            self.last_override = time.monotonic()
            self.overrides += 1
            # Inherit whatever altitude the follow loop had reached, so taking
            # over does not command a step change in height.
            if vision is not None:
                self.alt_sp = vision.alt_sp

        # --- rule 3: vision only in FOLLOW --------------------------------
        if self.mode is FlightMode.FOLLOW:
            if vision is None:
                # FOLLOW selected but the pipeline produced nothing this frame.
                # Hold level rather than guessing.
                return Command(seq=self.seq, t_ms=self._t_ms(),
                               roll=0.0, pitch=0.0, yaw_rate=0.0,
                               alt_sp=self.alt_sp,
                               flags=FLAG_ARMED | FLAG_ALT_HOLD | FLAG_FAILSAFE)
            self.alt_sp = vision.alt_sp
            # Re-stamp seq/arm so the arbiter remains the single author of
            # what goes on the wire; the vision loop keeps its own counter.
            return Command(seq=self.seq, t_ms=self._t_ms(),
                           roll=vision.roll, pitch=vision.pitch,
                           yaw_rate=vision.yaw_rate, alt_sp=vision.alt_sp,
                           flags=vision.flags | FLAG_ARMED)

        # --- manual and assist --------------------------------------------
        roll = _clamp(pilot.roll, -1.0, 1.0) * cfg.roll_max
        pitch = _clamp(pilot.pitch, -1.0, 1.0) * cfg.pitch_max
        yaw = _clamp(pilot.yaw, -1.0, 1.0) * cfg.yaw_max
        flags = FLAG_ARMED

        if self.mode is FlightMode.ASSIST:
            # Throttle stick becomes a climb RATE, centre means hold. That is
            # what makes assisted flight easy: let go and it stays put.
            rate = _clamp(pilot.throttle, -1.0, 1.0) * cfg.alt_max_rate
            self.alt_sp = _clamp(self.alt_sp + rate * dt,
                                 cfg.alt_min, cfg.alt_max)
            alt_sp = self.alt_sp
            flags |= FLAG_ALT_HOLD
        else:
            # MANUAL: the stick is throttle and the field carries throttle
            # 0..1 rather than metres — hence no ALT_HOLD flag.
            #
            # Only the UPPER half of the stick throttles. A gamepad stick is
            # spring-centred, so mapping the full travel ((t+1)/2) makes the
            # resting position 50% throttle — squeeze the deadman and the
            # motors leap to half power on a bench. Bottom-half-dead costs
            # resolution and buys the property that a neutral stick is a
            # stopped aircraft. The firmware's ARMED idle gate is the second
            # layer of the same defence.
            alt_sp = _clamp(max(0.0, pilot.throttle), 0.0, 1.0)
            self.alt_sp = cfg.alt_default

        return Command(seq=self.seq, t_ms=self._t_ms(),
                       roll=roll, pitch=pitch, yaw_rate=yaw,
                       alt_sp=alt_sp, flags=flags)

    # ----------------------------------------------------------------- info --
    @property
    def status(self) -> str:                              # pragma: no cover
        return self.mode.value
