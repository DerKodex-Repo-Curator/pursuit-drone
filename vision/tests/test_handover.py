"""Manual/autonomous handover, end to end over real sockets.

The arbiter unit tests prove the decision logic. These prove the decision
actually reaches the aircraft: real UDP, the command pump, and the drone
simulator applying the packets and running its own watchdog.

The scenario is the one you would fly: take off manually, hand over to the
autonomy, then grab control back mid-flight because it did something you did
not like.
"""
from __future__ import annotations

import os
import socket
import sys
import time

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from pursuit.arbiter import Arbiter, FlightMode                  # noqa: E402
from pursuit.config import ControlConfig, LinkConfig             # noqa: E402
from pursuit.link import DroneLink                                # noqa: E402
from pursuit.pilot import PilotInput                              # noqa: E402
from pursuit.protocol import (FLAG_ALT_HOLD, FLAG_ARMED,          # noqa: E402
                              FLAG_LOCKED, STATE_FLYING, STATE_IDLE,
                              Command)
from sim_drone import SimDrone                                    # noqa: E402

DT = 1 / 30


def free_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


@pytest.fixture
def rig():
    cmd_port, telem_port = free_port(), free_port()
    drone = SimDrone(cmd_port, telem_port, "127.0.0.1", 0.0, 0.0)
    link = DroneLink(LinkConfig(host="127.0.0.1", port=cmd_port,
                                telem_port=telem_port, enabled=True))
    arb = Arbiter(ControlConfig())
    yield drone, link, arb
    link.close()
    drone.close()


def fly(drone, link, arb, pilot, vision, frames=6):
    """Push a few frames through the whole chain and settle the receiver."""
    last = None
    for _ in range(frames):
        last = arb.update(pilot, vision, DT)
        link.send(last)
        deadline = time.monotonic() + 0.02
        while time.monotonic() < deadline:
            drone.poll()
            time.sleep(0.002)
    return last


def held(**kw):
    kw.setdefault("deadman", True)
    return PilotInput(connected=True, **kw)


def vision_cmd(**kw):
    kw.setdefault("flags", FLAG_ALT_HOLD | FLAG_LOCKED)
    kw.setdefault("alt_sp", 1.4)
    return Command(seq=1, **kw)


def test_full_handover_sequence(rig):
    drone, link, arb = rig

    # 1. Manual take-off. The pilot has the controls and the drone knows it.
    arb.request_mode(FlightMode.MANUAL)
    fly(drone, link, arb, held(throttle=0.6), None)
    assert drone.state == STATE_FLYING
    assert drone.accepted > 0

    # 2. Hand over to the autonomy.
    arb.request_mode(FlightMode.FOLLOW)
    cmd = fly(drone, link, arb, held(), vision_cmd(yaw_rate=35.0))
    assert arb.mode is FlightMode.FOLLOW
    assert cmd.yaw_rate == pytest.approx(35.0)
    assert cmd.flags & FLAG_LOCKED

    # 3. The autonomy does something alarming. The pilot grabs the sticks —
    #    without touching a mode button — and control returns instantly.
    cmd = fly(drone, link, arb, held(pitch=-0.9), vision_cmd(yaw_rate=35.0))
    assert arb.mode is FlightMode.ASSIST
    assert cmd.yaw_rate != pytest.approx(35.0)
    assert cmd.pitch < 0

    # 4. Letting go does NOT give it back.
    cmd = fly(drone, link, arb, held(), vision_cmd(yaw_rate=35.0), frames=20)
    assert arb.mode is FlightMode.ASSIST
    assert cmd.yaw_rate != pytest.approx(35.0)


def test_releasing_the_deadman_disarms_the_aircraft(rig):
    drone, link, arb = rig
    arb.request_mode(FlightMode.FOLLOW)
    fly(drone, link, arb, held(), vision_cmd(yaw_rate=60.0))
    assert drone.state == STATE_FLYING

    fly(drone, link, arb, PilotInput(connected=True, deadman=False),
        vision_cmd(yaw_rate=60.0))
    assert drone.state != STATE_FLYING, \
        "aircraft still flying after the deadman was released"


def test_autonomy_cannot_arm_on_its_own(rig):
    """Vision commanding hard, nobody holding the deadman: nothing may move.

    Asserts the aircraft is not flying rather than pinning one specific state
    constant — the arbiter sets FAILSAFE alongside clearing ARMED, so the
    simulator lands in FAILSAFE, not IDLE, and both are correct answers to
    "the operator is not authorising this".
    """
    drone, link, arb = rig
    arb.request_mode(FlightMode.FOLLOW)
    for _ in range(30):
        cmd = arb.update(PilotInput(connected=True, deadman=False),
                         vision_cmd(roll=8.0, pitch=8.0, yaw_rate=110.0), DT)
        link.send(cmd)
        drone.poll()
        assert not (cmd.flags & FLAG_ARMED)
        assert cmd.roll == 0.0 and cmd.pitch == 0.0 and cmd.yaw_rate == 0.0
    time.sleep(0.05)
    drone.poll()
    assert drone.state != STATE_FLYING
    assert drone.accepted > 0, "packets never arrived, so this proved nothing"


def test_a_dead_pilot_input_does_not_arm(rig):
    """Unplugged controller: PilotInput() defaults must be safe, not neutral-
    but-armed."""
    drone, link, arb = rig
    arb.request_mode(FlightMode.FOLLOW)
    blank = PilotInput()                 # connected=False, deadman=False
    cmd = fly(drone, link, arb, blank, vision_cmd(yaw_rate=80.0))
    assert not (cmd.flags & FLAG_ARMED)
    assert drone.state != STATE_FLYING


def test_watchdog_still_covers_a_crashed_laptop(rig):
    """The arbiter is not a substitute for the firmware watchdog: if the whole
    process dies mid-command, the drone must still stand down on its own."""
    from pursuit.protocol import STATE_FAILSAFE
    drone, link, arb = rig
    arb.request_mode(FlightMode.MANUAL)
    fly(drone, link, arb, held(throttle=0.8), None)
    assert drone.state == STATE_FLYING

    time.sleep(0.5)          # nothing sent — as if the process was killed
    drone.watchdog()
    assert drone.state == STATE_FAILSAFE
