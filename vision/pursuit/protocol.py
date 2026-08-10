"""Wire format between the laptop and the flight controller.

Raw floats over UDP would be the obvious thing and it is a genuinely bad idea
on an aircraft. UDP delivers packets corrupted, duplicated, reordered and late,
and every one of those failure modes has a specific consequence here:

  corrupted   a flipped exponent bit turns a 3 deg/s yaw into 3e38 deg/s
  reordered   a stale packet overwrites a newer command
  duplicated  harmless, but indistinguishable from a stuck sender
  late        the subject has moved; the command is for where they were

So every packet carries magic + version + sequence + CRC16, and the receiver
drops anything that fails a check. Sequence numbers make staleness detectable;
the timestamp makes latency measurable; the CRC makes corruption loud.

Byte order is little-endian to match the ESP32, so the firmware can memcpy the
struct instead of byte-swapping in the hot path. `gen_protocol_header.py`
emits the matching C header from these same definitions — the two sides cannot
drift, because there is only one description of the format.
"""
from __future__ import annotations

import math
import struct
from dataclasses import dataclass

PROTOCOL_VERSION = 1

CMD_MAGIC = 0x5031          # 'P1'
TELEM_MAGIC = 0x5054        # 'PT'

CMD_FORMAT = "<HBBIIffffH"
TELEM_FORMAT = "<HBBIIfffffHH"

CMD_SIZE = struct.calcsize(CMD_FORMAT)          # 30
TELEM_SIZE = struct.calcsize(TELEM_FORMAT)      # 36

# --- command flags ---------------------------------------------------------
FLAG_ARMED = 1 << 0         # operator is holding the arm control
FLAG_LOCKED = 1 << 1        # vision has a confirmed target
FLAG_ALT_HOLD = 1 << 2      # alt_sp is metres; otherwise it is throttle 0..1
FLAG_FAILSAFE = 1 << 3      # vision has given up — hold level, do not steer

# --- vehicle states --------------------------------------------------------
STATE_BOOT, STATE_IDLE, STATE_ARMED = 0, 1, 2
STATE_FLYING, STATE_FAILSAFE, STATE_LANDING = 3, 4, 5

STATE_NAMES = {
    STATE_BOOT: "BOOT", STATE_IDLE: "IDLE", STATE_ARMED: "ARMED",
    STATE_FLYING: "FLYING", STATE_FAILSAFE: "FAILSAFE",
    STATE_LANDING: "LANDING",
}


def crc16_ccitt(data: bytes, seed: int = 0xFFFF) -> int:
    """CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflection.

    Chosen over a checksum because it catches all single- and double-bit
    errors and all bursts up to 16 bits, and over CRC-32 because 2 bytes of a
    30-byte packet is enough at this rate and the ESP32 gets a smaller table.
    """
    crc = seed
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 \
                else (crc << 1) & 0xFFFF
    return crc


class ProtocolError(ValueError):
    pass


def _finite(value: float, name: str) -> float:
    """Reject NaN and infinity at the boundary.

    NaN is the dangerous one: it propagates silently through every arithmetic
    operation downstream, compares false against every bound check written as
    `if x > limit`, and would sail through a naive clamp untouched.
    """
    v = float(value)
    if not math.isfinite(v):
        raise ProtocolError(f"{name} is not finite: {value!r}")
    return v


@dataclass
class Command:
    """Laptop -> drone."""
    seq: int = 0
    t_ms: int = 0
    roll: float = 0.0           # deg
    pitch: float = 0.0          # deg
    yaw_rate: float = 0.0       # deg/s
    alt_sp: float = 0.0         # m if FLAG_ALT_HOLD, else throttle 0..1
    flags: int = 0

    def pack(self) -> bytes:
        body = struct.pack(
            CMD_FORMAT[:-1],
            CMD_MAGIC, PROTOCOL_VERSION, self.flags & 0xFF,
            self.seq & 0xFFFFFFFF, self.t_ms & 0xFFFFFFFF,
            _finite(self.roll, "roll"), _finite(self.pitch, "pitch"),
            _finite(self.yaw_rate, "yaw_rate"), _finite(self.alt_sp, "alt_sp"),
        )
        return body + struct.pack("<H", crc16_ccitt(body))

    @classmethod
    def unpack(cls, raw: bytes) -> "Command":
        if len(raw) != CMD_SIZE:
            raise ProtocolError(f"expected {CMD_SIZE} bytes, got {len(raw)}")
        magic, ver, flags, seq, t_ms, roll, pitch, yaw, alt, crc = \
            struct.unpack(CMD_FORMAT, raw)
        if magic != CMD_MAGIC:
            raise ProtocolError(f"bad magic 0x{magic:04X}")
        if ver != PROTOCOL_VERSION:
            raise ProtocolError(f"version {ver} != {PROTOCOL_VERSION}")
        if crc != crc16_ccitt(raw[:-2]):
            raise ProtocolError("CRC mismatch")
        for name, value in (("roll", roll), ("pitch", pitch),
                            ("yaw_rate", yaw), ("alt_sp", alt)):
            _finite(value, name)
        return cls(seq=seq, t_ms=t_ms, roll=roll, pitch=pitch,
                   yaw_rate=yaw, alt_sp=alt, flags=flags)

    # -- flag helpers --------------------------------------------------------
    @property
    def armed(self) -> bool:
        return bool(self.flags & FLAG_ARMED)

    @property
    def locked(self) -> bool:
        return bool(self.flags & FLAG_LOCKED)

    @property
    def alt_hold(self) -> bool:
        return bool(self.flags & FLAG_ALT_HOLD)

    @property
    def failsafe(self) -> bool:
        return bool(self.flags & FLAG_FAILSAFE)


@dataclass
class Telemetry:
    """Drone -> laptop."""
    seq: int = 0
    t_ms: int = 0
    vbat: float = 0.0           # V
    alt: float = 0.0            # m
    roll: float = 0.0           # deg
    pitch: float = 0.0          # deg
    yaw: float = 0.0            # deg
    loop_hz: int = 0
    state: int = STATE_BOOT

    def pack(self) -> bytes:
        body = struct.pack(
            TELEM_FORMAT[:-1],
            TELEM_MAGIC, PROTOCOL_VERSION, self.state & 0xFF,
            self.seq & 0xFFFFFFFF, self.t_ms & 0xFFFFFFFF,
            _finite(self.vbat, "vbat"), _finite(self.alt, "alt"),
            _finite(self.roll, "roll"), _finite(self.pitch, "pitch"),
            _finite(self.yaw, "yaw"), self.loop_hz & 0xFFFF,
        )
        return body + struct.pack("<H", crc16_ccitt(body))

    @classmethod
    def unpack(cls, raw: bytes) -> "Telemetry":
        if len(raw) != TELEM_SIZE:
            raise ProtocolError(f"expected {TELEM_SIZE} bytes, got {len(raw)}")
        (magic, ver, state, seq, t_ms, vbat, alt, roll, pitch, yaw,
         loop_hz, crc) = struct.unpack(TELEM_FORMAT, raw)
        if magic != TELEM_MAGIC:
            raise ProtocolError(f"bad magic 0x{magic:04X}")
        if ver != PROTOCOL_VERSION:
            raise ProtocolError(f"version {ver} != {PROTOCOL_VERSION}")
        if crc != crc16_ccitt(raw[:-2]):
            raise ProtocolError("CRC mismatch")
        return cls(seq=seq, t_ms=t_ms, vbat=vbat, alt=alt, roll=roll,
                   pitch=pitch, yaw=yaw, loop_hz=loop_hz, state=state)

    @property
    def state_name(self) -> str:
        return STATE_NAMES.get(self.state, f"?{self.state}")


def is_newer(seq: int, last_seq: int, window: int = 1 << 31) -> bool:
    """Strictly newer, surviving uint32 wraparound.

    Strict matters. An earlier version returned True for `seq == last_seq`,
    which meant an exact duplicate was accepted and re-applied — harmless
    while every command was sent once, and a live bug the moment the link
    started transmitting each command several times for redundancy.

    The wraparound handling is close to theatre at 30Hz (the counter lasts
    about 4.5 years) but it costs one comparison, and the alternative is a
    fault that appears once and can never be reproduced.
    """
    delta = (seq - last_seq) & 0xFFFFFFFF
    return 0 < delta < window
