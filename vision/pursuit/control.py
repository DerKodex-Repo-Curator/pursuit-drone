"""Image-space error -> flight setpoints.

Three independent PD loops:

  yaw    horizontal offset of the target from the aim point
  pitch  bounding-box height vs the height we want (standoff distance)
  alt    vertical offset, integrated into the ToF altitude setpoint

No integral term anywhere. A drone following a walking person has a permanent
non-zero error — that is what following *is* — and an I term would treat it as
a bias to be eliminated, wind up, and then overshoot every time the subject
stopped. The steady-state offset a P controller leaves behind is exactly the
lag needed to keep up.

Sign conventions, stated once because getting one backwards flies the drone
into a wall:

  yaw_rate  + = rotate right (clockwise seen from above)
  pitch     + = nose down / translate forward
  roll      + = right wing down / translate right
  alt_sp    metres above ground, from the belly ToF

The vertical one is the counter-intuitive one: raising a forward-pointing
camera makes a fixed object appear *lower* in frame, so a target sitting too
high in the image means the drone must climb.
"""
from __future__ import annotations

import math
import time

import numpy as np

from .config import CameraConfig, ControlConfig
from .geometry import height_px_for_distance, level_box
from .lock import LockResult, LockState
from .protocol import (FLAG_ALT_HOLD, FLAG_ARMED, FLAG_FAILSAFE, FLAG_LOCKED,
                       Command)


def soft_deadband(x: float, width: float) -> float:
    """Deadband that stays continuous at the edges.

    A hard deadband (`0 if |x|<w else x`) steps discontinuously from 0 to w the
    instant the error crosses the boundary, and the drone twitches. Rescaling
    the remainder over [w, 1] means the output leaves zero smoothly.
    """
    if width <= 0.0:
        return x
    ax = abs(x)
    if ax <= width:
        return 0.0
    span = max(1.0 - width, 1e-6)
    return math.copysign((ax - width) / span, x)


def clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


class _Axis:
    """One PD channel with deadband, clamp and slew limit."""

    def __init__(self, kp: float, kd: float, limit: float,
                 deadband: float, slew: float):
        self.kp, self.kd = kp, kd
        self.limit, self.deadband, self.slew = limit, deadband, slew
        self.prev_error = 0.0
        self.output = 0.0
        self.primed = False

    def reset(self) -> None:
        self.prev_error = 0.0
        self.output = 0.0
        self.primed = False

    def step(self, error: float, dt: float, scale: float = 1.0) -> float:
        e = soft_deadband(error, self.deadband)

        # The first frame after a reset has no previous error, so the derivative
        # would be the entire error divided by dt — a huge spike right at the
        # moment the drone acquires a target. Skip D until we have two samples.
        if self.primed and dt > 1e-4:
            derivative = (e - self.prev_error) / dt
        else:
            derivative = 0.0
            self.primed = True
        self.prev_error = e

        target = (self.kp * e + self.kd * derivative) * scale
        target = clamp(target, -self.limit, self.limit)

        max_step = self.slew * max(dt, 1e-4)
        delta = clamp(target - self.output, -max_step, max_step)
        self.output += delta
        return self.output

    def relax(self, dt: float) -> float:
        """Slew toward neutral — used when there is nothing to follow."""
        max_step = self.slew * max(dt, 1e-4)
        self.output -= clamp(self.output, -max_step, max_step)
        self.prev_error = 0.0
        self.primed = False
        return self.output

    def drive(self, target: float, dt: float) -> float:
        """Slew toward an explicit output, bypassing the PD term.

        Used by the search sweep, which is an open-loop command — there is no
        target to measure an error against, which is the entire problem.
        """
        max_step = self.slew * max(dt, 1e-4)
        self.output += clamp(target - self.output, -max_step, max_step)
        self.prev_error = 0.0
        self.primed = False
        return self.output


class FollowController:
    def __init__(self, cfg: ControlConfig,
                 camera: CameraConfig | None = None,
                 subject_height_m: float | None = None):
        self.cfg = cfg
        self.camera = camera or CameraConfig()
        # Set at enrolment. Without it the standoff falls back to a box-height
        # fraction, which works but is not a physical quantity.
        self.subject_height_m = subject_height_m
        self.yaw = _Axis(cfg.yaw_kp, cfg.yaw_kd, cfg.yaw_max,
                         cfg.yaw_deadband, cfg.slew_deg_per_s)
        self.pitch = _Axis(cfg.pitch_kp, cfg.pitch_kd, cfg.pitch_max,
                           cfg.pitch_deadband, cfg.slew_deg_per_s)
        self.alt_sp = cfg.alt_default
        self.seq = 0
        self._t0 = time.monotonic()
        self._last_error = (0.0, 0.0, 0.0)
        # False while the target box is clipped by a horizontal frame edge, so
        # the HUD and logs can show why altitude has stopped responding.
        self.vertical_valid = True
        # Diagnostics the HUD reads, so a corrected loop can be told apart from
        # a broken one at a glance.
        self.attitude_applied = (0.0, 0.0)
        self.lead_px = (0.0, 0.0)
        self.range_m: float | None = None
        self._searching = False
        self._search_started = 0.0
        self._search_dir = 0.0
        self._ever_locked = False
        # Overrides cfg.lead_time_s once the link has measured a real
        # round-trip. The configured value is only ever a starting guess.
        self._measured_lead_s: float | None = None

    def reset(self) -> None:
        self.yaw.reset()
        self.pitch.reset()
        self.alt_sp = self.cfg.alt_default
        self._last_error = (0.0, 0.0, 0.0)
        self.attitude_applied = (0.0, 0.0)
        self.lead_px = (0.0, 0.0)
        self.range_m = None
        self._searching = False
        self._ever_locked = False

    def set_measured_latency(self, rtt_ms: float) -> None:
        """Adopt a measured round-trip instead of the configured guess.

        Half the round trip is the one-way age of the frame the controller is
        acting on; the vision pipeline's own processing is already inside it,
        since the timestamp is stamped when the command is built. Clamped
        because a single bad sample must not throw the aim point across the
        frame.
        """
        if not math.isfinite(rtt_ms) or rtt_ms <= 0.0:
            return
        lead = clamp(rtt_ms * 0.5e-3, 0.02, 0.40)
        self._measured_lead_s = lead

    @property
    def lead_time_s(self) -> float:
        return (self._measured_lead_s if self._measured_lead_s is not None
                else self.cfg.lead_time_s)

    @property
    def errors(self) -> tuple[float, float, float]:
        """(yaw, height, vertical) normalised errors — for the HUD."""
        return self._last_error

    def _t_ms(self) -> int:
        return int((time.monotonic() - self._t0) * 1000.0) & 0xFFFFFFFF

    def _search_or_relax(self, lock: LockResult, dt: float) -> float:
        """Sweep toward where the subject was last heading, or stand down.

        Three deliberate restrictions, because a drone that yaws on its own is
        alarming and, done wrong, dangerous:

          * only after a real lock — a drone that has never seen anyone sits
            still rather than spinning to look for a stranger;
          * only while genuinely lost, not while coasting, since coasting is
            already steering at the predicted position;
          * only for `search_timeout_s`, then it holds. An unbounded sweep is
            just a slow spin, and it will find a wall before it finds anyone.
        """
        cfg = self.cfg
        if not (cfg.search_enabled and self._ever_locked
                and lock.state in (LockState.LOST, LockState.SEARCHING)):
            self._searching = False
            return self.yaw.relax(dt)

        now = time.monotonic()
        if not self._searching:
            self._searching = True
            self._search_started = now

        if (now - self._search_started) > cfg.search_timeout_s:
            return self.yaw.relax(dt)

        direction = self._search_dir if self._search_dir else 1.0
        return self.yaw.drive(direction * cfg.search_yaw_rate, dt)

    # ---------------------------------------------------------------- update --
    def update(self, lock: LockResult, frame_w: int, frame_h: int,
               dt: float, armed: bool = False,
               attitude: tuple[float, float] | None = None) -> Command:
        """`attitude` is the drone's measured (roll, pitch) in degrees.

        Pass it whenever telemetry is live. Without it the loop still works but
        couples: pitching forward to chase makes the subject rise in frame and
        the altitude axis reads that as a real climb command.
        """
        cfg = self.cfg
        self.seq += 1
        dt = clamp(dt, 1e-3, 0.25)

        flags = FLAG_ALT_HOLD
        if armed:
            flags |= FLAG_ARMED
        if lock.state is LockState.LOCKED:
            flags |= FLAG_LOCKED
            self._ever_locked = True
            self._searching = False

        if not lock.commanding or lock.xyxy is None:
            if lock.state in (LockState.LOST, LockState.SEARCHING):
                flags |= FLAG_FAILSAFE
            self._last_error = (0.0, 0.0, 0.0)
            self.range_m = None
            return Command(
                seq=self.seq, t_ms=self._t_ms(),
                roll=0.0, pitch=self.pitch.relax(dt),
                yaw_rate=self._search_or_relax(lock, dt),
                alt_sp=self.alt_sp, flags=flags)

        # --- undo the airframe's own attitude --------------------------------
        focal = self.camera.focal_px(frame_w)
        box = lock.xyxy
        if attitude is not None:
            roll_deg, pitch_deg = attitude
            self.attitude_applied = (roll_deg, pitch_deg)
            box = level_box(box, roll_deg, pitch_deg, focal, frame_w, frame_h)
        else:
            self.attitude_applied = (0.0, 0.0)

        x1, y1, x2, y2 = (float(v) for v in box)
        cx, cy = (x1 + x2) * 0.5, (y1 + y2) * 0.5
        box_h = max(y2 - y1, 1.0)

        # --- aim where they will be, not where they were ---------------------
        # Velocity comes from the tracker's Kalman filter in px/frame; scaling
        # by dt converts to px/s without needing to know the frame rate.
        lead_x = lead_y = 0.0
        lead_s = self.lead_time_s
        if lock.velocity is not None and lead_s > 0.0:
            vx, vy = lock.velocity
            lead_x = clamp(vx / dt * lead_s, -cfg.lead_max_px, cfg.lead_max_px)
            lead_y = clamp(vy / dt * lead_s, -cfg.lead_max_px, cfg.lead_max_px)
        self.lead_px = (lead_x, lead_y)
        cx += lead_x
        cy += lead_y

        half_w, half_h = frame_w * 0.5, frame_h * 0.5
        aim_y = half_h + cfg.aim_y_bias * half_h

        # --- normalised errors, all in roughly [-1, 1] ----------------------
        e_yaw = (cx - half_w) / max(half_w, 1.0)
        e_vert = (cy - aim_y) / max(half_h, 1.0)

        # Height is the distance cue — not width or area, because height barely
        # changes when the subject turns sideways while width nearly halves.
        # With a known subject height it becomes a real range in metres.
        if cfg.use_metric_standoff and self.subject_height_m:
            self.range_m = (self.subject_height_m * focal) / box_h
            want_h = height_px_for_distance(cfg.standoff_m,
                                            self.subject_height_m, focal)
            e_dist = (want_h - box_h) / max(want_h, 1.0)
        else:
            self.range_m = None
            h_frac = box_h / max(float(frame_h), 1.0)
            e_dist = (cfg.target_height_frac - h_frac) / cfg.target_height_frac

        # A distant subject can otherwise produce an unbounded error: at 20m
        # the wanted height is many times the measured one.
        e_dist = clamp(e_dist, -2.0, 2.0)
        self._last_error = (e_yaw, e_dist, e_vert)
        # Remember which way they were going, so a search sweeps the right way.
        if abs(e_yaw) > 0.05:
            self._search_dir = 1.0 if e_yaw > 0 else -1.0

        # While coasting on a Kalman prediction, fade the command out. The
        # prediction is trustworthy for a few frames after an occlusion and
        # increasingly fictional after that.
        scale = 1.0
        if lock.source == "predicted" and lock.miss_frames > 0:
            scale = cfg.coast_decay ** lock.miss_frames

        yaw_rate = self.yaw.step(e_yaw, dt, scale)
        pitch = self.pitch.step(e_dist, dt, scale)

        # --- altitude: rate-limited integration into the ToF setpoint -------
        # Only if the box is vertically complete. A box clipped by the top or
        # bottom edge has a centroid pulled toward the visible end, and acting
        # on it drives the aircraft further in the direction that clips more of
        # the subject — the failure runs away rather than settling.
        #
        # Tested against the ORIGINAL box, not the attitude-corrected one:
        # clipping is a fact about what the detector saw in the real image. The
        # levelled box is *expected* to fall outside the frame — that is the
        # whole point of the correction — and testing it here would freeze
        # altitude at exactly the moment a pitch correction was working.
        ox1, oy1, ox2, oy2 = (float(v) for v in lock.xyxy)
        self.vertical_valid = (oy1 > cfg.edge_margin_px and
                               oy2 < frame_h - cfg.edge_margin_px)
        if self.vertical_valid:
            e_alt = soft_deadband(e_vert, cfg.alt_deadband)
            alt_rate = clamp(-cfg.alt_kp * e_alt * scale,
                             -cfg.alt_max_rate, cfg.alt_max_rate)
            step = clamp(alt_rate * dt, -cfg.slew_alt_per_s * dt,
                         cfg.slew_alt_per_s * dt)
            self.alt_sp = clamp(self.alt_sp + step, cfg.alt_min, cfg.alt_max)

        return Command(
            seq=self.seq, t_ms=self._t_ms(),
            roll=0.0, pitch=pitch, yaw_rate=yaw_rate,
            alt_sp=self.alt_sp, flags=flags)
