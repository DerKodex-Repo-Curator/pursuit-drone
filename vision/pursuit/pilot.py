"""Pilot input — what the human is asking for.

Two backends behind one interface:

  KeyboardPilot   works today, no hardware. Digital, so the sticks ramp rather
                  than jump; fine for bench testing and mode logic, not for
                  actually flying.
  HidGamepad      analogue, via hidapi. What you would fly on.

pygame would be the obvious choice and has no wheels for Python 3.14, so this
reads HID reports directly. That means a per-controller byte layout, and the
layouts below are from public HID descriptors rather than from testing against
each pad — so `calibrate_gamepad.py` exists to dump raw reports and tell you
the offsets for whatever you actually own. Treat an untested profile as a
hypothesis, and check it on the bench with props off.

Every axis is normalised to [-1, 1] with centre at 0, except throttle in
MANUAL where the arbiter remaps it. Deadzones are applied here so neither the
arbiter nor the control loop has to care which physical device produced the
number.
"""
from __future__ import annotations

from dataclasses import dataclass

from .arbiter import FlightMode


@dataclass
class PilotInput:
    """One frame of human intent."""
    roll: float = 0.0          # -1 left .. +1 right
    pitch: float = 0.0         # -1 back .. +1 forward
    yaw: float = 0.0           # -1 left .. +1 right
    throttle: float = 0.0      # -1 down .. +1 up (centre = hold in ASSIST)
    deadman: bool = False      # HELD, never latched
    mode_request: FlightMode | None = None
    connected: bool = False

    @property
    def any_stick(self) -> float:
        return max(abs(self.roll), abs(self.pitch),
                   abs(self.yaw), abs(self.throttle))


def deadzone(value: float, dz: float = 0.08) -> float:
    """Remove centre slop without creating a step at the edge of it."""
    if abs(value) <= dz:
        return 0.0
    span = max(1.0 - dz, 1e-6)
    return (abs(value) - dz) / span * (1.0 if value > 0 else -1.0)


# ------------------------------------------------------------------ keyboard --

class KeyboardPilot:
    """Digital stand-in driven by the preview window's key codes.

    Real flight needs analogue sticks, but this exercises every path — modes,
    the deadman, the FOLLOW override — with no hardware at all, which is what
    makes the arbiter testable before a controller exists.

    The deadman is the awkward part: a keyboard gives key *events*, not held
    state, so it is held for `hold_s` after each press and the key must be
    repeated to sustain it. Autorepeat does that for you if you lean on it.
    """

    RAMP = 2.5          # units per second while a direction key is held
    DECAY = 4.0         # return to centre when nothing is pressed

    def __init__(self, hold_s: float = 0.35):
        self.hold_s = hold_s
        self._axes = {"roll": 0.0, "pitch": 0.0, "yaw": 0.0, "throttle": 0.0}
        self._pressed_at = -1e9
        self._mode_request: FlightMode | None = None
        self._now = 0.0

    def key(self, code: int) -> None:
        """Feed a key code from cv2.waitKey (-1/255 when nothing was pressed)."""
        if code in (-1, 255):
            return
        ch = chr(code) if 0 <= code < 256 else ""

        step = self.RAMP * 0.06
        if ch == "a":
            self._axes["roll"] -= step
        elif ch == "d":
            self._axes["roll"] += step
        elif ch == "w":
            self._axes["pitch"] += step
        elif ch == "s":
            self._axes["pitch"] -= step
        elif ch == "q":
            self._axes["yaw"] -= step
        elif ch == "e":
            self._axes["yaw"] += step
        elif ch == "i":
            self._axes["throttle"] += step
        elif ch == "k":
            self._axes["throttle"] -= step
        elif ch == "1":
            self._mode_request = FlightMode.MANUAL
        elif ch == "2":
            self._mode_request = FlightMode.ASSIST
        elif ch == "3":
            self._mode_request = FlightMode.FOLLOW
        elif ch == " ":
            self._pressed_at = self._now

    def poll(self, now: float, dt: float) -> PilotInput:
        self._now = now
        for name, value in self._axes.items():
            decay = self.DECAY * dt
            if value > 0:
                self._axes[name] = max(0.0, value - decay)
            elif value < 0:
                self._axes[name] = min(0.0, value + decay)
            self._axes[name] = max(-1.0, min(1.0, self._axes[name]))

        request, self._mode_request = self._mode_request, None
        return PilotInput(
            roll=self._axes["roll"], pitch=self._axes["pitch"],
            yaw=self._axes["yaw"], throttle=self._axes["throttle"],
            deadman=(now - self._pressed_at) < self.hold_s,
            mode_request=request, connected=True)


# ------------------------------------------------------------------ gamepad --

class _HidHandle:
    """Adapter over the two rival `hid` modules on PyPI.

    `hidapi` (Cython) exposes `hid.device()` with `.open()/.set_nonblocking()`.
    `hid` (ctypes) exposes `hid.Device(vid, pid)` with a `.nonblocking`
    attribute. They are different packages that install the same import name,
    so which one you get depends on what was pip-installed — and calling the
    wrong one fails with a bare AttributeError that looks nothing like the
    real problem.
    """

    def __init__(self, mod, vid: int, pid: int):
        self.raw = None
        self.style = None
        if hasattr(mod, "Device"):
            dev = mod.Device(vid, pid)
            try:
                dev.nonblocking = True
            except Exception:
                pass
            self.raw, self.style = dev, "new"
        else:
            dev = mod.device()
            dev.open(vid, pid)
            dev.set_nonblocking(1)
            self.raw, self.style = dev, "old"

    def read(self, size: int = 64, timeout_ms: int = 0) -> bytes:
        if self.style == "new":
            return bytes(self.raw.read(size, timeout=timeout_ms) or b"")
        # The Cython build's read() takes no timeout in non-blocking mode.
        return bytes(self.raw.read(size) or b"")

    def close(self) -> None:
        try:
            self.raw.close()
        except Exception:
            pass

    def describe(self) -> str:
        try:
            if self.style == "new":
                return f"{self.raw.manufacturer} / {self.raw.product}"
            return (f"{self.raw.get_manufacturer_string()} / "
                    f"{self.raw.get_product_string()}")
        except Exception:                                 # pragma: no cover
            return "?"

@dataclass
class PadProfile:
    """Byte layout of one controller's HID input report.

    Two things here are easy to get wrong and produce garbage rather than an
    error:

    **Axis width.** Xbox sticks are 16-bit little-endian centred on 0x8000;
    PlayStation sticks are 8-bit centred on 0x80. Reading an Xbox stick as one
    byte gets you the *low* byte of a 16-bit value — which sweeps 0-255
    repeatedly as the stick moves, so the drone would twitch wildly and
    look, from the code's side, perfectly well behaved.

    **Report id.** hidapi includes the leading report-id byte for devices that
    use numbered reports, and omits it for those that do not. `report_id`
    records which to expect; offsets below are always into the report as
    hidapi delivers it.
    """
    name: str
    vendor: int
    products: tuple[int, ...]

    axis_roll: int
    axis_pitch: int
    axis_yaw: int
    axis_throttle: int
    axis_bits: int = 8

    # Deadman is either an analogue trigger past a threshold, or a button bit.
    # A trigger is the better control: it is proportional, it is where your
    # finger already is, and it springs back on its own.
    deadman_axis: int | None = None
    deadman_axis_bits: int = 16
    # Declared, never learned — see HidGamepad._trigger. Xbox triggers are
    # 10-bit inside a 16-bit field, so this cannot come from the field width.
    deadman_full_scale: int = 1023
    deadman_thresh: float = 0.35
    deadman_byte: int | None = None
    deadman_mask: int = 0

    mode_byte: int = 0
    mode_manual: int = 0x01
    mode_assist: int = 0x02
    mode_follow: int = 0x08

    invert_pitch: bool = True      # sticks report "up" as a low value
    invert_throttle: bool = True


# Layouts from published HID descriptors, NOT verified against hardware.
# `calibrate_gamepad.py --learn` maps whatever you actually own; treat these
# as hypotheses until you have checked one.
PROFILES = [
    # Xbox Wireless Controller over Bluetooth. Report id 0x01, then four
    # 16-bit sticks, then two 10-bit triggers, hat, then the button bits.
    # Deadman on the RIGHT TRIGGER; modes on A / B / Y.
    PadProfile("Xbox Wireless", 0x045E,
               (0x0B13, 0x0B20, 0x0B21, 0x0B22, 0x02FD, 0x02E0, 0x0B12,
                0x02EA),
               axis_roll=1, axis_pitch=3, axis_yaw=5, axis_throttle=7,
               axis_bits=16,
               deadman_axis=11, deadman_axis_bits=16,
               deadman_full_scale=1023, deadman_thresh=0.35,
               mode_byte=14, mode_manual=0x01, mode_assist=0x02,
               mode_follow=0x10),
    PadProfile("DualSense (PS5)", 0x054C, (0x0CE6, 0x0DF2),
               axis_roll=1, axis_pitch=2, axis_yaw=3, axis_throttle=4,
               axis_bits=8,
               deadman_axis=6, deadman_axis_bits=8, deadman_full_scale=255,
               deadman_thresh=0.35,
               mode_byte=9, mode_manual=0x20, mode_assist=0x40,
               mode_follow=0x80),
    PadProfile("DualShock 4", 0x054C, (0x09CC, 0x05C4),
               axis_roll=1, axis_pitch=2, axis_yaw=3, axis_throttle=4,
               axis_bits=8,
               deadman_axis=9, deadman_axis_bits=8, deadman_full_scale=255,
               deadman_thresh=0.35,
               mode_byte=5, mode_manual=0x20, mode_assist=0x40,
               mode_follow=0x80),
]


class HidGamepad:
    """Analogue pilot input over raw HID."""

    def __init__(self, profile: PadProfile | None = None, dz: float = 0.08):
        import hid
        self._hid = hid
        self.dz = dz
        self.profile = profile
        self.dev = None
        self.last_report: bytes = b""
        self._open()

    # ------------------------------------------------------------- discovery --
    @staticmethod
    def find() -> PadProfile | None:
        """First connected controller we have a layout for."""
        try:
            import hid
        except ImportError:
            return None
        present = {(d.get("vendor_id"), d.get("product_id"))
                   for d in hid.enumerate()}
        for prof in PROFILES:
            for pid in prof.products:
                if (prof.vendor, pid) in present:
                    return prof
        # Same vendor, unknown product id: controllers get new ids with
        # firmware revisions, and the report layout almost always survives.
        # Better to try the family layout and let calibration correct it than
        # to fall back to a keyboard.
        vendors = {d.get("vendor_id") for d in hid.enumerate()}
        for prof in PROFILES:
            if prof.vendor in vendors:
                return prof
        return None

    @staticmethod
    def any_gamepad() -> list[dict]:
        """Anything that *looks* like a gamepad, profile or not.

        Used to tell "no controller plugged in" apart from "controller plugged
        in but unrecognised" — different problems with different fixes.
        """
        try:
            import hid
        except ImportError:
            return []
        return [d for d in hid.enumerate()
                if d.get("usage_page") == 1 and d.get("usage") in (4, 5, 8)]

    def _open(self) -> None:
        if self.profile is None:
            self.profile = self.find()
        if self.profile is None:
            return
        present = {(d.get("vendor_id"), d.get("product_id"))
                   for d in self._hid.enumerate()}
        candidates = [(self.profile.vendor, pid)
                      for pid in self.profile.products
                      if (self.profile.vendor, pid) in present]
        candidates += [(v, p) for (v, p) in present
                       if v == self.profile.vendor and
                       (v, p) not in candidates]
        for vendor, product in candidates:
            try:
                self.dev = _HidHandle(self._hid, vendor, product)
                self.product_id = product
                return
            except Exception:                             # pragma: no cover
                continue
        print(f"  gamepad: found {self.profile.name} but could not open it "
              f"(another app may have it, or macOS needs input permission)")
        self.dev = None

    @property
    def connected(self) -> bool:
        return self.dev is not None

    # ------------------------------------------------------------------ poll --
    @staticmethod
    def _raw(report: bytes, index: int, bits: int) -> int | None:
        """Unsigned axis value at `index`, 8- or 16-bit little-endian."""
        if index < 0 or index + (bits // 8) > len(report):
            return None
        if bits == 16:
            return report[index] | (report[index + 1] << 8)
        return report[index]

    def _axis(self, report: bytes, index: int, bits: int,
              invert: bool) -> float:
        """Centred stick axis, normalised to [-1, 1]."""
        raw = self._raw(report, index, bits)
        if raw is None:
            return 0.0
        full = (1 << bits) - 1
        centre = (full + 1) // 2
        value = (raw - centre) / float(centre)
        if invert:
            value = -value
        return deadzone(max(-1.0, min(1.0, value)), self.dz)

    def _trigger(self, report: bytes, index: int, bits: int,
                 full_scale: int) -> float:
        """Trigger travel in [0, 1]. Rests at zero, not at centre.

        `full_scale` is declared by the profile rather than learned from what
        has been seen so far. An adaptive maximum sounds convenient and gives
        you an arming threshold that moves: before the trigger has ever been
        fully squeezed the assumed range is too small, so a light touch reads
        as a large fraction and arms the aircraft earlier than the operator
        expects. A fixed scale is predictable, and being wrong in the other
        direction — assuming a wider range than the hardware uses — fails
        safe, because the deadman simply never engages.

        Xbox triggers are 10-bit (0-1023) inside a 16-bit field, which is why
        this cannot be derived from the field width.
        """
        raw = self._raw(report, index, bits)
        if raw is None:
            return 0.0
        return max(0.0, min(1.0, raw / float(max(full_scale, 1))))

    def poll(self, now: float, dt: float) -> PilotInput:
        if self.dev is None:
            return PilotInput(connected=False)

        report = self.last_report
        try:
            # Drain the queue: HID buffers reports, and acting on a stale one
            # means flying on a stick position from several frames ago.
            while True:
                chunk = self.dev.read(64)
                if not chunk:
                    break
                report = bytes(chunk)
        except Exception:                                 # pragma: no cover
            self.dev.close()
            self.dev = None
            return PilotInput(connected=False)

        if not report:
            return PilotInput(connected=True)
        self.last_report = report

        p = self.profile
        mode = None
        if p.mode_byte < len(report):
            bits = report[p.mode_byte]
            if bits & p.mode_manual:
                mode = FlightMode.MANUAL
            elif bits & p.mode_assist:
                mode = FlightMode.ASSIST
            elif bits & p.mode_follow:
                mode = FlightMode.FOLLOW

        deadman = False
        if p.deadman_axis is not None:
            self.deadman_travel = self._trigger(report, p.deadman_axis,
                                                p.deadman_axis_bits,
                                                p.deadman_full_scale)
            deadman = self.deadman_travel >= p.deadman_thresh
        elif p.deadman_byte is not None and p.deadman_byte < len(report):
            deadman = bool(report[p.deadman_byte] & p.deadman_mask)

        return PilotInput(
            roll=self._axis(report, p.axis_roll, p.axis_bits, False),
            pitch=self._axis(report, p.axis_pitch, p.axis_bits,
                             p.invert_pitch),
            yaw=self._axis(report, p.axis_yaw, p.axis_bits, False),
            throttle=self._axis(report, p.axis_throttle, p.axis_bits,
                                p.invert_throttle),
            deadman=deadman, mode_request=mode, connected=True)

    def close(self) -> None:
        if self.dev is not None:
            try:
                self.dev.close()
            except Exception:
                pass
            self.dev = None


def make_pilot(prefer_gamepad: bool = True, quiet: bool = False):
    """Pick the best available input, reporting honestly which one it is."""
    if prefer_gamepad:
        profile = HidGamepad.find()
        if profile is not None:
            pad = HidGamepad(profile)
            if pad.connected:
                if not quiet:
                    print(f"  pilot     {profile.name} (analogue)")
                return pad
        else:
            others = HidGamepad.any_gamepad()
            if others and not quiet:
                names = ", ".join((d.get("product_string") or "?")
                                  for d in others[:3])
                print(f"  pilot     a gamepad is connected ({names}) but has "
                      f"no profile")
                print(f"            run: python calibrate_gamepad.py")
    if not quiet:
        print("  pilot     keyboard (digital — bench testing only)")
    return KeyboardPilot()
