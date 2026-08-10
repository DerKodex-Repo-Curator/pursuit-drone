"""The wire format is the one place a bug reaches the aircraft directly."""
from __future__ import annotations

import os
import random
import struct
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from pursuit.protocol import (CMD_MAGIC, CMD_SIZE, FLAG_ALT_HOLD, FLAG_ARMED,
                              FLAG_FAILSAFE, FLAG_LOCKED, PROTOCOL_VERSION,
                              TELEM_SIZE, Command, ProtocolError, Telemetry,
                              crc16_ccitt, is_newer)


def test_sizes_are_fixed():
    # The C header asserts these exact numbers; drifting silently would shift
    # every field past the change on the firmware side.
    assert CMD_SIZE == 30
    assert TELEM_SIZE == 36


def test_command_round_trip():
    cmd = Command(seq=1234, t_ms=99, roll=1.5, pitch=-2.25, yaw_rate=45.0,
                  alt_sp=1.25,
                  flags=FLAG_ARMED | FLAG_LOCKED | FLAG_ALT_HOLD)
    out = Command.unpack(cmd.pack())
    assert out.seq == cmd.seq
    # float32 on the wire, so exact equality is the wrong assertion.
    assert out.roll == pytest.approx(cmd.roll, abs=1e-6)
    assert out.yaw_rate == pytest.approx(cmd.yaw_rate, abs=1e-6)
    assert out.armed and out.locked and out.alt_hold
    assert not out.failsafe


def test_telemetry_round_trip():
    tel = Telemetry(seq=9, vbat=3.72, alt=1.4, yaw=270.0, loop_hz=980, state=3)
    out = Telemetry.unpack(tel.pack())
    assert out.vbat == pytest.approx(3.72, abs=1e-6)
    assert out.loop_hz == 980
    assert out.state_name == "FLYING"


def test_every_single_bit_flip_is_caught():
    """A CRC that catches most corruption is not good enough.

    Exhaustive rather than sampled: flip each of the 240 bits of a command in
    turn and require every one to be rejected. A single accepted corruption is
    a setpoint the drone acts on.
    """
    raw = Command(seq=7, roll=3.0, pitch=-4.0, yaw_rate=90.0,
                  alt_sp=1.5, flags=FLAG_ARMED).pack()
    escaped = []
    for byte in range(len(raw)):
        for bit in range(8):
            bad = bytearray(raw)
            bad[byte] ^= 1 << bit
            try:
                Command.unpack(bytes(bad))
            except ProtocolError:
                continue
            escaped.append((byte, bit))
    assert not escaped, f"corruption accepted at {escaped}"


def test_random_multi_bit_corruption():
    rng = random.Random(20260805)
    base = Command(seq=11, yaw_rate=33.0, alt_sp=1.1).pack()
    escaped = 0
    for _ in range(4000):
        bad = bytearray(base)
        for _ in range(rng.randint(1, 4)):
            bad[rng.randrange(len(bad))] ^= 1 << rng.randrange(8)
        if bytes(bad) == base:
            continue
        try:
            Command.unpack(bytes(bad))
            escaped += 1
        except ProtocolError:
            pass
    # CRC-16 admits a ~1/65536 residual for multi-bit bursts; anything much
    # above that means the check is not covering the whole packet.
    assert escaped <= 1, f"{escaped} corrupted packets accepted"


def test_wrong_magic_and_version_rejected():
    raw = bytearray(Command(seq=1).pack())
    raw[0:2] = struct.pack("<H", 0x1234)
    raw[-2:] = struct.pack("<H", crc16_ccitt(bytes(raw[:-2])))
    with pytest.raises(ProtocolError, match="magic"):
        Command.unpack(bytes(raw))

    raw = bytearray(Command(seq=1).pack())
    raw[2] = PROTOCOL_VERSION + 1
    raw[-2:] = struct.pack("<H", crc16_ccitt(bytes(raw[:-2])))
    with pytest.raises(ProtocolError, match="version"):
        Command.unpack(bytes(raw))


def test_truncated_packet_rejected():
    raw = Command(seq=1).pack()
    for n in (0, 1, CMD_SIZE - 1, CMD_SIZE + 1):
        with pytest.raises(ProtocolError):
            Command.unpack(raw[:n] if n <= CMD_SIZE else raw + b"\x00")


@pytest.mark.parametrize("bad", [float("nan"), float("inf"), float("-inf")])
def test_non_finite_setpoints_refused(bad):
    """NaN is the dangerous one — it survives naive clamps and range checks."""
    with pytest.raises(ProtocolError):
        Command(roll=bad).pack()
    with pytest.raises(ProtocolError):
        Command(yaw_rate=bad).pack()


def test_nan_on_the_wire_is_rejected_on_receipt():
    # A sender that skipped validation, or memory corruption in flight.
    body = struct.pack("<HBBIIffff", CMD_MAGIC, PROTOCOL_VERSION, 0, 1, 0,
                       float("nan"), 0.0, 0.0, 0.0)
    raw = body + struct.pack("<H", crc16_ccitt(body))
    with pytest.raises(ProtocolError):
        Command.unpack(raw)


def test_crc_known_answer():
    # Pins the polynomial/init/reflection choice so a "harmless" refactor of
    # crc16_ccitt cannot silently desync from the C implementation.
    assert crc16_ccitt(b"123456789") == 0x29B1


def test_sequence_wraparound():
    assert is_newer(5, 4)
    assert not is_newer(4, 5)
    assert is_newer(0, 0xFFFFFFFF)          # wrapped, still newer
    assert not is_newer(0xFFFFFFFF, 0)


def test_an_exact_duplicate_is_not_newer():
    """Strictness matters: the link sends each command several times so a
    burst of packet loss cannot gap it, and a non-strict comparison would
    treat every copy as fresh and re-apply the same setpoint."""
    assert not is_newer(5, 5)
    assert not is_newer(0, 0)
    assert not is_newer(0xFFFFFFFF, 0xFFFFFFFF)


def test_failsafe_flag_survives():
    cmd = Command(flags=FLAG_ALT_HOLD | FLAG_FAILSAFE, alt_sp=1.0)
    out = Command.unpack(cmd.pack())
    assert out.failsafe and out.alt_hold and not out.armed
