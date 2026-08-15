#!/usr/bin/env python3
"""Bench-test the radio link against real hardware, then give the WiFi back.

    python3 tools/link_check.py

WHY THIS OWNS THE NETWORK SWITCH
--------------------------------
The aircraft is a SoftAP with no route to the internet, so testing the link
means taking the laptop off its normal network. Doing that by hand, in steps,
is how a bench session ends with a machine parked on `PURSUIT-1` wondering
why nothing loads — and if anything is driving this remotely, it has just cut
its own connection and cannot undo it.

So the whole excursion is one process with the restore in a `finally`. It
comes back if the test passes, if it fails, if it raises, or if you Ctrl-C it.
The only way to stay disconnected is to kill -9 this script.

WHAT IT PROVES that no unit test can
------------------------------------
  1. The protocol agrees ACROSS A NETWORK, not just across a byte vector.
     Matching struct sizes prove nothing: field order can differ while sizes
     agree, endianness can be right in one direction only, and a CRC computed
     over a different byte range is self-consistent within each language and
     rejects everything across the link.
  2. Round-trip latency, measured. The controller's lead compensation is
     derived from it, and the stability boundary without attitude
     compensation is ~17 ms — so this number decides whether the outer loop
     settles or oscillates.
  3. THE ARMING INTERLOCK, on hardware. It holds FLAG_ARMED on every packet
     and the aircraft must refuse to arm whenever its sensors have not
     passed. A firmware that armed here would be one bad sensor away from
     spinning motors on a bench.

It never commands a climb: alt_sp stays below the ARMED->FLYING threshold, so
even a fully working aircraft stays on the ground. Props off anyway.
"""
from __future__ import annotations

import argparse
import os
import re
import subprocess
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "vision"))

SECRETS = os.path.join(ROOT, "firmware", "include", "secrets.h")


# ------------------------------------------------------------------- network

def _sh(*args: str) -> str:
    return subprocess.run(args, capture_output=True, text=True).stdout.strip()


def wifi_device() -> str:
    out = _sh("networksetup", "-listallhardwareports")
    lines = out.splitlines()
    for i, line in enumerate(lines):
        if "Wi-Fi" in line or "AirPort" in line:
            if i + 1 < len(lines):
                return lines[i + 1].split()[-1]
    return "en0"


def current_ssid(dev: str) -> str | None:
    """The SSID actually associated, or None.

    Not `networksetup -getairportnetwork`: on recent macOS that reports "not
    associated" while the interface plainly holds a DHCP lease from the very
    AP it says it is not on. `ipconfig getsummary` reads the association.
    """
    out = _sh("ipconfig", "getsummary", dev)
    m = re.search(r"^\s*SSID\s*:\s*(.+)$", out, re.MULTILINE)
    return m.group(1).strip() if m else None


def ipv4(dev: str) -> str | None:
    out = _sh("ifconfig", dev)
    m = re.search(r"inet (\d+\.\d+\.\d+\.\d+)", out)
    return m.group(1) if m else None


def read_secrets() -> tuple[str, str]:
    if not os.path.exists(SECRETS):
        raise SystemExit(
            f"{SECRETS} not found.\n"
            "cp firmware/include/secrets.h.example firmware/include/secrets.h")
    text = open(SECRETS).read()

    def grab(name: str) -> str:
        m = re.search(rf'define\s+{name}\s+"([^"]*)"', text)
        if not m:
            raise SystemExit(f"could not find {name} in secrets.h")
        return m.group(1)

    return grab("PURSUIT_AP_SSID"), grab("PURSUIT_AP_PASS")


def on_drone_net(dev: str) -> bool:
    ip = ipv4(dev)
    return bool(ip and ip.startswith("192.168.4."))


def join(dev: str, ssid: str, password: str, attempts: int = 3) -> bool:
    """Join, and keep asking.

    Retries the request rather than only polling after one. Two real reasons
    it fails first time: the aircraft has just been reset by a flash and its
    SoftAP is not up yet, and macOS quietly declines to settle on an AP with
    no route to the internet.
    """
    for attempt in range(1, attempts + 1):
        subprocess.run(
            ["networksetup", "-setairportnetwork", dev, ssid, password],
            capture_output=True, text=True)
        for _ in range(8):
            if on_drone_net(dev):
                return True
            time.sleep(1.0)
        if attempt < attempts:
            print(f"  not associated yet, retrying ({attempt}/{attempts})")
    return False


def restore(dev: str, ssid: str | None) -> None:
    print("\n--- restoring your network ---")
    # Drop the drone from the preferred list first. Otherwise macOS will
    # cheerfully auto-join a bare SoftAP with no internet again later, which
    # is a baffling thing to debug an hour after a bench session.
    subprocess.run(["networksetup", "-removepreferredwirelessnetwork",
                    dev, "PURSUIT-1"], capture_output=True, text=True)
    if ssid:
        subprocess.run(["networksetup", "-setairportnetwork", dev, ssid],
                       capture_output=True, text=True)
    # Power-cycle regardless: it makes macOS re-pick the best known network,
    # which is the right answer whether or not the name above was captured.
    subprocess.run(["networksetup", "-setairportpower", dev, "off"],
                   capture_output=True)
    time.sleep(2)
    subprocess.run(["networksetup", "-setairportpower", dev, "on"],
                   capture_output=True)
    for _ in range(15):
        ip = ipv4(dev)
        if ip and not ip.startswith("192.168.4."):
            print(f"back on {current_ssid(dev) or '?'} ({ip})")
            return
        time.sleep(2)
    print("WARNING: could not confirm the network came back — check WiFi.")


# ---------------------------------------------------------------- the test

def run_test(seconds: float, dev: str | None = None) -> int:
    from pursuit.config import LinkConfig
    from pursuit.link import DroneLink
    from pursuit.protocol import (FLAG_ALT_HOLD, FLAG_ARMED, FLAG_LOCKED,
                                  Command)

    cfg = LinkConfig()
    link = DroneLink(cfg)
    print(f"link -> {cfg.host}:{cfg.port}, telemetry back on {cfg.telem_port}")

    states: dict[str, int] = {}
    seq = 0
    t0 = time.monotonic()
    print(f"\nsending {seconds:.0f}s of valid commands at 30Hz, ARM HELD")
    while time.monotonic() - t0 < seconds:
        seq += 1
        link.send(Command(
            seq=seq, roll=0.0, pitch=2.5, yaw_rate=-12.0,
            # Below the ARMED->FLYING climb threshold on purpose: this must
            # not command an aircraft upward, even a working one.
            alt_sp=0.10,
            flags=FLAG_ARMED | FLAG_LOCKED | FLAG_ALT_HOLD))
        t = link.telemetry
        if t is not None:
            states[t.state_name] = states.get(t.state_name, 0) + 1
        time.sleep(1.0 / 30.0)

    s = link.stats
    t = link.telemetry
    print(f"\n  sent {s.sent}   received {s.received}   "
          f"rejected {s.rejected}   send_errors {s.send_errors}")
    print(f"  rtt {s.rtt_ms:.1f} ms      link_up {link.link_up}")
    if t is not None:
        print(f"  state {t.state_name}   loop {t.loop_hz} Hz   "
              f"vbat {t.vbat:.2f} V   alt {t.alt:.2f} m")
        print(f"  attitude  roll {t.roll:+.1f}  pitch {t.pitch:+.1f}  "
              f"yaw {t.yaw:+.1f}")
    print(f"  states seen: {states}")

    ok = True
    if s.received == 0:
        # Distinguish "the aircraft is silent" from "we are not on its network
        # any more", because they look identical from here and only one of
        # them is the aircraft's fault. UDP sendto to an unreachable host does
        # not raise, so send_errors stays at 0 either way.
        #
        # This is not hypothetical: macOS drops an AP with no route to the
        # internet, and a run that joined in one step and tested in the next
        # reported "the aircraft never answered" while sitting on the house
        # WiFi. A test that can lie about which end failed is worse than no
        # test.
        if dev is not None and not on_drone_net(dev):
            print(f"\nINCONCLUSIVE  the laptop left the drone's network "
                  f"mid-test (now {ipv4(dev)}). macOS drops an access point "
                  f"with no internet. Nothing was proved about the aircraft.")
            return 3
        print("\nFAIL  no telemetry — the aircraft never answered")
        ok = False
    if s.rejected:
        print(f"\nFAIL  {s.rejected} telemetry packets rejected — the two ends "
              f"disagree about the wire format")
        ok = False
    armed_states = {"ARMED", "FLYING"}
    if armed_states & set(states):
        print("\n*** The aircraft ARMED. That is correct only if its sensors "
              "passed.\n    On a bench with no IMU it would mean the interlock "
              "is broken.")
    else:
        print("\nOK    held ARM on every packet and the aircraft did not arm")

    link.close()
    return 0 if ok else 1


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--seconds", type=float, default=6.0)
    args = ap.parse_args()

    if sys.platform != "darwin":
        raise SystemExit("the WiFi handling here is macOS-only; join "
                         "PURSUIT-1 yourself and run the test body directly")

    dev = wifi_device()
    previous = current_ssid(dev)
    ssid, password = read_secrets()
    print(f"wifi {dev}, currently on {previous or '(nothing)'}")
    print(f"joining {ssid} — you will be OFFLINE until this finishes")

    try:
        if not join(dev, ssid, password):
            print(f"could not join {ssid}. Is the aircraft powered?")
            return 2
        print(f"joined, {ipv4(dev)}")
        return run_test(args.seconds, dev)
    except KeyboardInterrupt:
        print("\ninterrupted")
        return 130
    finally:
        # The whole reason this script exists.
        restore(dev, previous)


if __name__ == "__main__":
    sys.exit(main())
