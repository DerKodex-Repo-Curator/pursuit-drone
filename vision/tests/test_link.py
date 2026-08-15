"""Link integration — the real sockets, against the drone simulator.

These run over loopback rather than with mocks, because the things that break a
UDP control link are exactly the things a mock papers over: ports already in
use, packets arriving out of order, a receive thread that never starts, a CRC
check that is never actually exercised because nothing ever corrupts anything.
"""
from __future__ import annotations

import os
import socket
import sys
import time

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from pursuit.config import LinkConfig                            # noqa: E402
from pursuit.link import CommandPump, DroneLink                  # noqa: E402
from pursuit.protocol import (FLAG_ALT_HOLD, FLAG_ARMED, Command,  # noqa: E402
                              STATE_FLYING)
from sim_drone import SimDrone                                    # noqa: E402


def free_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


@pytest.fixture
def rig():
    cmd_port, telem_port = free_port(), free_port()
    made = []

    def build(lossy=0.0, corrupt=0.0):
        drone = SimDrone(cmd_port, telem_port, "127.0.0.1", lossy, corrupt)
        cfg = LinkConfig(host="127.0.0.1", port=cmd_port,
                         telem_port=telem_port, enabled=True)
        link = DroneLink(cfg)
        made.extend([drone, link])
        return drone, link

    yield build
    for obj in made:
        obj.close()


def pump_once(drone, link, cmd, settle=0.05):
    link.send(cmd)
    deadline = time.monotonic() + settle
    while time.monotonic() < deadline:
        drone.poll()
        time.sleep(0.002)


def test_command_reaches_the_drone(rig):
    drone, link = rig()
    pump_once(drone, link, Command(seq=1, yaw_rate=30.0, alt_sp=1.5,
                                   flags=FLAG_ARMED | FLAG_ALT_HOLD))
    # One command, accepted once — even though the link transmits several
    # copies of it. The duplicates are discarded by the sequence check, which
    # is what makes redundancy free rather than a source of repeated setpoints.
    assert drone.accepted == 1
    assert drone.stale == LinkConfig().redundancy - 1
    assert drone.state == STATE_FLYING


def test_redundancy_survives_packet_loss(rig):
    """The reason for sending copies: a burst of loss must not gap the link,
    because a gap past the watchdog drops the aircraft into failsafe."""
    drone, link = rig(lossy=0.5)
    delivered = 0
    for seq in range(1, 41):
        before = drone.accepted
        pump_once(drone, link, Command(seq=seq, flags=FLAG_ARMED | FLAG_ALT_HOLD),
                  settle=0.01)
        if drone.accepted > before:
            delivered += 1
    # At 50% loss per packet, 3 copies give ~87% per command versus 50% for a
    # single send. Assert comfortably above the single-send rate.
    assert delivered >= 28, f"only {delivered}/40 commands got through"


def test_telemetry_comes_back(rig):
    drone, link = rig()
    pump_once(drone, link, Command(seq=1, flags=FLAG_ARMED | FLAG_ALT_HOLD))
    for _ in range(30):
        drone.send_telemetry()
        time.sleep(0.01)
        if link.telemetry is not None:
            break
    assert link.telemetry is not None, "no telemetry decoded"
    assert link.link_up
    assert link.stats.received > 0
    assert 3.0 < link.telemetry.vbat < 4.3


def test_link_goes_down_when_telemetry_stops(rig):
    drone, link = rig()
    drone.send_telemetry()
    for _ in range(50):
        time.sleep(0.01)
        if link.telemetry is not None:
            break
    assert link.link_up
    time.sleep(link.cfg.timeout_ms / 1000.0 + 0.15)
    assert not link.link_up, "stale telemetry still reported as a live link"


def test_stale_and_replayed_packets_are_rejected(rig):
    drone, link = rig()
    for seq in (10, 11, 12):
        pump_once(drone, link, Command(seq=seq, flags=FLAG_ALT_HOLD))
    assert drone.accepted == 3

    before_stale = drone.stale
    pump_once(drone, link, Command(seq=11, flags=FLAG_ALT_HOLD))  # replay
    assert drone.accepted == 3, "an out-of-order packet was acted on"
    assert drone.stale > before_stale


def test_corruption_is_always_caught(rig):
    """Every injected bit-flip must be rejected, not merely most of them."""
    drone, link = rig(corrupt=1.0)
    for seq in range(1, 41):
        pump_once(drone, link, Command(seq=seq, yaw_rate=40.0,
                                       flags=FLAG_ARMED | FLAG_ALT_HOLD),
                  settle=0.01)
    assert drone.accepted == 0, \
        f"{drone.accepted} corrupted commands were accepted as valid"
    assert drone.bad_crc >= 35


def test_packet_loss_does_not_break_the_stream(rig):
    drone, link = rig(lossy=0.4)
    for seq in range(1, 61):
        pump_once(drone, link, Command(seq=seq, flags=FLAG_ALT_HOLD),
                  settle=0.008)
    assert drone.accepted > 20, "far more loss than injected"
    assert drone.bad_crc == 0, "loss was misreported as corruption"


def test_watchdog_trips_when_commands_stop(rig):
    from pursuit.protocol import STATE_FAILSAFE
    drone, link = rig()
    pump_once(drone, link, Command(seq=1, yaw_rate=90.0,
                                   flags=FLAG_ARMED | FLAG_ALT_HOLD))
    assert drone.state == STATE_FLYING

    time.sleep(0.5)          # longer than TIMEOUT_MS
    drone.watchdog()
    assert drone.state == STATE_FAILSAFE, \
        "vehicle kept flying on a command that stopped arriving"


def test_pump_substitutes_failsafe_for_a_stale_command(rig):
    """The pump must keep the link alive without repeating an old setpoint.

    Re-sending a stale hard-left yaw at 30Hz would look like a perfectly
    healthy link from the firmware's side, right up until it hit something.
    """
    drone, link = rig()
    pump = CommandPump(link, hz=60.0, stale_ms=40.0).start()
    try:
        pump.submit(Command(seq=1, yaw_rate=95.0,
                            flags=FLAG_ARMED | FLAG_ALT_HOLD, alt_sp=1.4))
        deadline = time.monotonic() + 0.6
        while time.monotonic() < deadline:
            drone.poll()
            time.sleep(0.002)
    finally:
        pump.stop()

    assert pump.substituted > 0, "pump kept resending a stale command"
    assert drone.accepted > 1
    # The vehicle must have been told to stop steering, and must not have
    # been left holding the original 95 deg/s.
    assert drone.state != STATE_FLYING


def test_pump_forwards_fresh_commands_unchanged(rig):
    drone, link = rig()
    pump = CommandPump(link, hz=60.0, stale_ms=200.0).start()
    try:
        for seq in range(1, 16):
            pump.submit(Command(seq=seq, yaw_rate=25.0, alt_sp=1.2,
                                flags=FLAG_ARMED | FLAG_ALT_HOLD))
            deadline = time.monotonic() + 0.03
            while time.monotonic() < deadline:
                drone.poll()
                time.sleep(0.002)
    finally:
        pump.stop()

    assert pump.substituted == 0, "fresh commands were replaced by failsafes"
    assert drone.accepted > 5
    assert drone.state == STATE_FLYING


def test_malformed_setpoints_are_never_transmitted(rig):
    drone, link = rig()
    bad = Command(seq=1, yaw_rate=float("nan"), flags=FLAG_ARMED)
    assert link.send(bad) is False
    assert link.stats.sent == 0
    assert link.stats.send_errors == 1
    time.sleep(0.05)
    drone.poll()
    assert drone.accepted == 0


def test_rtt_is_not_poisoned_by_a_gap_in_commands(rig):
    """A silent stretch must not be averaged in as latency.

    The firmware echoes the timestamp of the last command it ACCEPTED, and
    telemetry keeps flowing at 20Hz whether or not new commands arrive. So a
    repeated echo does not mean a slow round trip — it means no new command
    has been accepted, and the age of that stamp is the length of the gap.

    Measured on hardware 2026-08-11: six seconds of commands followed by five
    of silence reported rtt 1641ms on a link that pings in 3.2ms. That number
    feeds the controller's lead compensation, where the stability boundary
    without attitude compensation is ~17ms.
    """
    drone, link = rig()

    for seq in range(1, 15):
        link.send(Command(seq=seq, alt_sp=1.0,
                          flags=FLAG_ARMED | FLAG_ALT_HOLD))
        deadline = time.monotonic() + 0.04
        while time.monotonic() < deadline:
            drone.poll()
            drone.send_telemetry()
            time.sleep(0.005)
    settled = link.stats.rtt_ms
    assert 0.0 < settled < 250.0, f"loopback rtt implausible: {settled}"

    # Now go quiet, but keep the drone talking — which is exactly what real
    # hardware does, telemetry at 20Hz regardless. Every packet from here on
    # echoes the SAME stamp, growing stale by the length of the silence.
    deadline = time.monotonic() + 1.2
    while time.monotonic() < deadline:
        drone.send_telemetry()
        time.sleep(0.01)

    assert link.stats.received > 0
    assert link.stats.rtt_ms == pytest.approx(settled, abs=1.0), (
        f"a {1.2:.1f}s command gap moved the rtt estimate from {settled:.1f}ms "
        f"to {link.stats.rtt_ms:.1f}ms — stale echoes are being averaged in")
