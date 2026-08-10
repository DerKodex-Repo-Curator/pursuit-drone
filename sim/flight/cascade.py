"""The PURSUIT-1 control cascade, ported to numpy.

This is a PORT, and a port is a duplicate, and a duplicate drifts. That is the
exact failure this project already guards against on the wire — the protocol
header is generated from one source so the two ends cannot disagree — and the
honest thing is to say plainly that no such guard exists here. If you retune
firmware/src/pid.cpp, these numbers do not follow. `test_cascade.py` pins the
values that were current when this was written so the drift is at least loud.

Three nested loops, outermost first:

    vision PD      30 Hz, off-board, over WiFi, sees the world LATE
      -> attitude setpoint, degrees
    angle P        1 kHz, on-board          (main.cpp: ANGLE_KP)
      -> body rate setpoint, deg/s
    rate PID       1 kHz, on-board          (pid.cpp: default_*_rate_gains)
      -> normalised torque, -1..1
    mixer          (mixer.cpp)
      -> four motor duties

Only the outermost loop carries transport delay. The inner two live on the
aircraft and see the IMU directly. That is the whole reason the cascade is
arranged this way, and it is also why the interesting stability question is
about the OUTER loop: the inner loops are fast and local, the outer one is
slow and remote.
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass

import numpy as np

# ---------------------------------------------------------------------------
# Gains, mirrored from the firmware. Sources named so a reader can check them.
# ---------------------------------------------------------------------------

# firmware/src/main.cpp: static constexpr float ANGLE_KP = 6.0f;
ANGLE_KP = 6.0
# firmware/include/config.h: MAX_YAW_RATE_DPS 180.0f — also the clamp on the
# angle loop's rate output.
MAX_RATE_DPS = 180.0
# firmware/include/config.h: MAX_ANGLE_DEG 25.0f
MAX_ANGLE_DEG = 25.0

# firmware/src/pid.cpp: default_roll_rate_gains() / default_pitch_rate_gains()
RATE_KP = 0.0016
RATE_KI = 0.012
RATE_KD = 0.000045
RATE_I_LIMIT = 0.25
RATE_D_LPF_HZ = 80.0

# firmware/src/pid.cpp: default_yaw_rate_gains()
YAW_KP = 0.0045
YAW_KI = 0.030
YAW_KD = 0.0
YAW_I_LIMIT = 0.30

# vision/pursuit/config.py: ControlConfig
VIS_PITCH_KP = 34.0     # degrees of pitch per unit normalised height error
VIS_PITCH_KD = 4.5
VIS_PITCH_DEADBAND = 0.03
VIS_YAW_KP = 95.0       # deg/s per unit normalised horizontal error
VIS_YAW_KD = 9.0
VIS_YAW_DEADBAND = 0.04


class RatePid:
    """Mirror of PidRate in firmware/src/pid.cpp.

    Two details are load-bearing and are reproduced rather than simplified:

    * **D acts on the measurement, not the error.** Differentiating the error
      makes a step in setpoint produce an unbounded spike — "derivative kick".
      The aircraft's actual rotation rate cannot step, so differentiating it
      gives the same damping with no kick. Sign flips accordingly.
    * **Conditional integration.** A plain clamp on the integrator is not
      enough: while saturated it still stores authority it cannot spend, and
      then spends it as overshoot on the way back. Integration is skipped
      entirely when the output is already saturated and the error would push
      it further out.
    """

    def __init__(self, kp: float, ki: float, kd: float,
                 i_limit: float, d_lpf_hz: float):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.i_limit = i_limit
        self.d_lpf_hz = d_lpf_hz
        self.reset()

    def reset(self) -> None:
        self.integral = 0.0
        self.prev_meas = 0.0
        self.d_filtered = 0.0
        self.primed = False

    def update(self, setpoint: float, measurement: float, dt: float) -> float:
        error = setpoint - measurement

        if not self.primed:
            # The first call has no previous measurement, so the derivative
            # would be the whole rate divided by dt — a large kick at exactly
            # the moment of arming.
            self.prev_meas = measurement
            self.primed = True

        d_raw = -(measurement - self.prev_meas) / dt if dt > 0 else 0.0
        self.prev_meas = measurement

        if self.d_lpf_hz > 0.0 and dt > 0.0:
            rc = 1.0 / (2.0 * np.pi * self.d_lpf_hz)
            a = dt / (rc + dt)
            self.d_filtered += a * (d_raw - self.d_filtered)
        else:
            self.d_filtered = d_raw

        p_term = self.kp * error
        d_term = self.kd * self.d_filtered
        unsat = p_term + self.integral + d_term

        candidate = self.integral + self.ki * error * dt
        saturated_high = unsat >= 1.0 and error > 0.0
        saturated_low = unsat <= -1.0 and error < 0.0
        if not (saturated_high or saturated_low):
            self.integral = float(np.clip(candidate, -self.i_limit, self.i_limit))

        return float(np.clip(p_term + self.integral + d_term, -1.0, 1.0))


class AnglePid:
    """Mirror of AnglePid in firmware/src/pid.cpp — proportional only.

    An integrator here would fight the rate loop's integrator for the same
    authority: two integrators in series on one physical axis, each convinced
    the other's contribution is a disturbance to be cancelled.
    """

    def __init__(self, kp: float = ANGLE_KP, max_rate: float = MAX_RATE_DPS):
        self.kp, self.max_rate = kp, max_rate

    def update(self, angle_sp_deg: float, angle_deg: float) -> float:
        return float(np.clip(self.kp * (angle_sp_deg - angle_deg),
                             -self.max_rate, self.max_rate))


def mix(throttle: float, roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Mirror of mixer.cpp, including the desaturation behaviour.

    The mixer does NOT clip individual motors. Clipping one breaks the
    differential the controller asked for, so a pure throttle command starts
    producing roll — the aircraft tips exactly when it is working hardest.
    Instead the whole set is shifted in common mode, which preserves every
    differential and sacrifices only total thrust, which is the one thing that
    can be given up safely.

    Motor order M1 front-left, M2 front-right, M3 rear-left, M4 rear-right.
    Yaw right speeds up the CCW pair (M2, M3): reaction torque opposes the
    rotation the prop is producing.
    """
    m = np.array([
        throttle - roll - pitch - yaw,   # M1 front-left,  CW
        throttle + roll - pitch + yaw,   # M2 front-right, CCW
        throttle - roll + pitch + yaw,   # M3 rear-left,   CCW
        throttle + roll + pitch - yaw,   # M4 rear-right,  CW
    ])

    low, high = m.min(), m.max()
    if high > 1.0:
        m -= (high - 1.0)
    if low < 0.0:
        m -= min(m.min(), 0.0)
    return np.clip(m, 0.0, 1.0)


@dataclass
class DelayLine:
    """Transport delay, quantised to whole control periods.

    The vision loop is not merely slow, it is LATE: a frame is captured,
    encoded, sent over WiFi, run through detection and tracking, turned into a
    setpoint, sent back, and only then acted on. Every one of those stages adds
    phase lag to a loop that is already closed around an integrator.

    The measured round trip on the loopback harness is about 20 ms; over real
    WiFi with a busy channel it will be more, and the point of the sweep is to
    find out how much more matters.
    """

    delay_s: float
    dt: float

    def __post_init__(self) -> None:
        n = max(0, int(round(self.delay_s / self.dt)))
        self._q: deque = deque([None] * n, maxlen=n) if n else deque()

    def push(self, value):
        if self._q.maxlen is None or self._q.maxlen == 0:
            return value
        out = self._q[0] if self._q[0] is not None else None
        self._q.append(value)
        return out


class VisionPD:
    """Mirror of the outer loop in vision/pursuit/control.py.

    PD with no integral term anywhere, deliberately. Following a person means a
    permanent non-zero error — you are always slightly behind them — and an
    integrator would read that as an offset to be eliminated and wind up
    against it until it saturated.
    """

    def __init__(self, kp: float, kd: float, deadband: float, out_limit: float):
        self.kp, self.kd = kp, kd
        self.deadband = deadband
        self.out_limit = out_limit
        self.prev_err = 0.0
        self.primed = False

    def reset(self) -> None:
        self.prev_err = 0.0
        self.primed = False

    def update(self, error: float, dt: float) -> float:
        if abs(error) < self.deadband:
            error = 0.0
        if not self.primed:
            self.prev_err = error
            self.primed = True
        d = (error - self.prev_err) / dt if dt > 0 else 0.0
        self.prev_err = error
        return float(np.clip(self.kp * error + self.kd * d,
                             -self.out_limit, self.out_limit))
