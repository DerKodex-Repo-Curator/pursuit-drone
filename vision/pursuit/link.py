"""UDP transport to the drone, with a watchdog in both directions.

Two independent failure modes, two independent guards:

  laptop stops sending   the *firmware* must notice and level off. That guard
                         lives on the ESP32; nothing here can help once the
                         laptop is wedged. What this module guarantees is that
                         packets keep flowing at a steady rate whether or not
                         vision is producing anything, so silence genuinely
                         means "the sender died" and not "no target visible".

  drone stops replying   this module notices (`link_up` goes false) and the
                         HUD says so. The drone is still flying on its own
                         watchdog; the operator needs to know they are blind.

Sending never blocks and never raises. A vision pipeline that stalls because
the drone was switched off is worse than useless — errors are counted and
surfaced, not propagated.
"""
from __future__ import annotations

import socket
import threading
import time

from .config import LinkConfig
from .protocol import (FLAG_ALT_HOLD, FLAG_FAILSAFE, Command, ProtocolError,
                       TELEM_SIZE, Telemetry, is_newer)


class LinkStats:
    __slots__ = ("sent", "send_errors", "received", "rejected", "stale",
                 "last_rx_ms", "last_tx_ms", "rtt_ms")

    def __init__(self):
        self.sent = 0
        self.send_errors = 0
        self.received = 0
        self.rejected = 0        # bad magic / version / CRC
        self.stale = 0           # arrived out of order
        self.last_rx_ms = 0.0
        self.last_tx_ms = 0.0
        self.rtt_ms = 0.0


class DroneLink:
    def __init__(self, cfg: LinkConfig):
        self.cfg = cfg
        self.stats = LinkStats()
        self.telemetry: Telemetry | None = None

        self._sock: socket.socket | None = None
        self._rx: socket.socket | None = None
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._last_seq = -1
        self._t0 = time.monotonic()
        self._cmd_times: dict[int, float] = {}

        if cfg.enabled:
            self._open()

    # ----------------------------------------------------------------- setup --
    def _open(self) -> None:
        try:
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._sock.setblocking(False)

            self._rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self._rx.bind(("0.0.0.0", self.cfg.telem_port))
            self._rx.settimeout(0.2)

            self._thread = threading.Thread(target=self._rx_loop,
                                            name="telemetry", daemon=True)
            self._thread.start()
        except OSError as exc:                            # pragma: no cover
            print(f"  link: could not open sockets ({exc}) — running offline")
            self.close()

    # ------------------------------------------------------------------ send --
    def send(self, cmd: Command) -> bool:
        if self._sock is None:
            return False
        # Stamp t_ms HERE, with this link's clock, overwriting whatever the
        # controller wrote. The drone echoes this stamp back in telemetry and
        # the round-trip is computed against the same clock that wrote it —
        # any other arrangement compares two unrelated epochs and produces a
        # number that is silently filtered as implausible, which is exactly
        # what the e2e loop caught: rtt stuck at 0.0 and the lead
        # compensation never calibrating.
        cmd.t_ms = int(self._now_ms()) & 0xFFFFFFFF
        try:
            raw = cmd.pack()
        except ProtocolError as exc:
            # A non-finite setpoint reached the packer. Dropping the packet is
            # correct: the firmware's watchdog handles the gap, whereas a
            # corrupt command would be acted on.
            self.stats.send_errors += 1
            print(f"  link: refusing to send malformed command: {exc}")
            return False

        # Send several copies. The firmware discards any sequence it has
        # already accepted, so duplicates cost nothing but make a short burst
        # of packet loss survivable — and a burst that crosses the watchdog
        # timeout drops the aircraft into failsafe, which ends the follow.
        sent = 0
        for _ in range(max(1, self.cfg.redundancy)):
            try:
                self._sock.sendto(raw, (self.cfg.host, self.cfg.port))
                sent += 1
            except OSError:
                # Host unreachable is the normal state when the drone is off.
                break
        if sent == 0:
            self.stats.send_errors += 1
            return False

        self.stats.sent += 1
        self.stats.last_tx_ms = self._now_ms()
        self._cmd_times[cmd.seq] = time.monotonic()
        if len(self._cmd_times) > 256:
            for key in sorted(self._cmd_times)[:128]:
                self._cmd_times.pop(key, None)
        return True

    # -------------------------------------------------------------- receive --
    def _rx_loop(self) -> None:
        while not self._stop.is_set():
            try:
                raw, _addr = self._rx.recvfrom(512)
            except socket.timeout:
                continue
            except OSError:
                break

            if len(raw) != TELEM_SIZE:
                self.stats.rejected += 1
                continue
            try:
                telem = Telemetry.unpack(raw)
            except ProtocolError:
                self.stats.rejected += 1
                continue

            with self._lock:
                if self._last_seq >= 0 and not is_newer(telem.seq,
                                                        self._last_seq):
                    self.stats.stale += 1
                    continue
                self._last_seq = telem.seq
                self.telemetry = telem
                self.stats.received += 1
                self.stats.last_rx_ms = self._now_ms()

                # The firmware echoes back the t_ms of the last command it
                # accepted, so this subtraction is a true round-trip without
                # the two ends needing a shared clock. It is the number the
                # controller's lead compensation actually wants; before this
                # it was a hard-coded guess.
                if telem.t_ms:
                    rtt = self._now_ms() - float(telem.t_ms)
                    if 0.0 <= rtt < 2000.0:
                        self.stats.rtt_ms = (rtt if self.stats.rtt_ms == 0.0
                                             else 0.85 * self.stats.rtt_ms
                                             + 0.15 * rtt)

    # ------------------------------------------------------------------ state --
    def _now_ms(self) -> float:
        return (time.monotonic() - self._t0) * 1000.0

    @property
    def link_up(self) -> bool:
        if self.stats.received == 0:
            return False
        return (self._now_ms() - self.stats.last_rx_ms) < self.cfg.timeout_ms

    @property
    def enabled(self) -> bool:
        return self._sock is not None

    def close(self) -> None:
        self._stop.set()
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=0.5)
        for sock in (self._sock, self._rx):
            if sock is not None:
                try:
                    sock.close()
                except OSError:
                    pass
        self._sock = self._rx = self._thread = None

    def __enter__(self) -> "DroneLink":
        return self

    def __exit__(self, *exc) -> None:
        self.close()


class RateLimiter:
    """Fixed-rate gate for the send loop.

    Vision runs at whatever the detector manages; the control link must run at
    a constant rate regardless, because the firmware infers link health from
    packet arrival timing.
    """

    def __init__(self, hz: float):
        self.period = 1.0 / max(hz, 1e-3)
        self._next = time.monotonic()

    def ready(self) -> bool:
        now = time.monotonic()
        if now < self._next:
            return False
        # Skip missed slots rather than bursting to catch up after a stall.
        self._next = max(now, self._next + self.period)
        return True


class CommandPump:
    """Send at a constant rate, independent of how fast vision runs.

    This exists to make the drone able to tell three situations apart, because
    each one wants a different response:

      fresh commands arriving   follow them
      link alive, FAILSAFE set  laptop is fine, it just cannot see the target
                                — hold position and wait
      nothing arriving at all   the laptop or the WiFi is gone — land

    Without a pump the second and third cases look identical from the ESP32,
    and the only safe reading of "silence" is the pessimistic one, so the drone
    would land every time the subject stepped behind a sofa.

    Critically, a command older than `stale_ms` is NOT repeated. Re-sending a
    stale hard-left yaw at 30Hz would fly the aircraft into a wall at constant
    speed while looking, from the firmware's side, like a perfectly healthy
    link. Past that age the pump substitutes a neutral failsafe command.
    """

    def __init__(self, link: "DroneLink", hz: float, stale_ms: float = 150.0):
        self.link = link
        self.period = 1.0 / max(hz, 1e-3)
        self.stale_s = stale_ms / 1000.0
        self.substituted = 0

        self._cmd: Command | None = None
        self._cmd_at = 0.0
        self._seq = 0
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    def submit(self, cmd: Command) -> None:
        with self._lock:
            self._cmd = cmd
            self._cmd_at = time.monotonic()
            self._seq = max(self._seq, cmd.seq)

    def _next_packet(self) -> Command | None:
        with self._lock:
            cmd, age_at = self._cmd, self._cmd_at
            if cmd is None:
                return None
            if (time.monotonic() - age_at) <= self.stale_s:
                return cmd
            self._seq += 1
            self.substituted += 1
            # Neutral attitude, hold the last altitude, failsafe flagged.
            return Command(seq=self._seq, t_ms=cmd.t_ms,
                           roll=0.0, pitch=0.0, yaw_rate=0.0,
                           alt_sp=cmd.alt_sp,
                           flags=FLAG_ALT_HOLD | FLAG_FAILSAFE)

    def _loop(self) -> None:
        limiter = RateLimiter(1.0 / self.period)
        while not self._stop.is_set():
            if limiter.ready():
                packet = self._next_packet()
                if packet is not None:
                    self.link.send(packet)
            self._stop.wait(self.period * 0.25)

    def start(self) -> "CommandPump":
        if self._thread is None and self.link.enabled:
            self._thread = threading.Thread(target=self._loop,
                                            name="cmd-pump", daemon=True)
            self._thread.start()
        return self

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=0.5)
            self._thread = None
