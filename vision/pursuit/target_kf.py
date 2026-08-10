"""A constant-velocity Kalman filter on the target's state in the image.

What this replaces
------------------
Lead compensation previously used the tracker's own velocity, differenced
per frame and scaled by dt. That works, and it has three problems that only
show up on an aircraft:

1. **It is noisy.** A per-frame difference of two detector outputs is
   dominated by box jitter. Multiplying it by a lead time and adding it to the
   aim point injects that jitter straight into an attitude setpoint.
2. **It vanishes when the detection does.** The tracker's velocity is only
   available while there is a track. A missed frame leaves the controller
   aiming at a stale point with no notion of how stale.
3. **It does not distinguish "not moving" from "not measured".** Both come
   back as zero, and those want opposite responses.

A constant-velocity Kalman filter fixes all three from one model: it fuses the
noisy measurements into a smoothed estimate, it keeps predicting through gaps
and reports how long it has been coasting, and its covariance grows while it
coasts so the controller can tell a confident estimate from a guess.

State and model
---------------
Six states: the box centre and height, and their rates.

    x = [cx, cy, h, vx, vy, vh]

Constant velocity between frames, so

    cx' = cx + vx*dt      vx' = vx

Height is carried in the same filter rather than a separate one because it is
the range cue — distance comes from box HEIGHT, never width, since a person
gets much wider when they gesture and no taller — and smoothing it matters for
the same reason smoothing the centre does.

The process noise is a proper piecewise-white-acceleration model rather than
an ad-hoc diagonal: an unknown acceleration acting over dt perturbs position by
0.5*a*dt^2 and velocity by a*dt, and those two are strongly correlated. Using a
diagonal Q pretends they are independent, which makes the filter trust its
position estimate more than it should exactly when the subject changes speed.

Deliberately NOT a tracker
--------------------------
This does not do association, birth or death — `bytetrack.py` owns all of
that. This filter sees one target, the one the lock machine has already
decided to follow, and its only job is to hand the controller a smooth state
and an honest confidence.
"""
from __future__ import annotations

import numpy as np


class TargetKF:
    """Constant-velocity filter over (cx, cy, h) in pixels.

    Units are pixels and seconds throughout — never pixels-per-frame, which is
    what the previous lead computation used and which silently changes meaning
    whenever the frame rate does.
    """

    NDIM = 6

    def __init__(self,
                 pos_std: float = 6.0,
                 h_std: float = 10.0,
                 accel_std: float = 260.0,
                 h_accel_std: float = 90.0,
                 max_coast_s: float = 1.0):
        """
        pos_std      measurement noise on the centre, px. Detector box jitter
                     on a stationary subject is a few px; 6 is deliberately
                     slightly pessimistic, which costs a little lag and buys a
                     lot of smoothness.
        h_std        measurement noise on box height, px. Larger than the
                     centre because the box top and bottom are both noisy and
                     legs get clipped by the frame edge.
        accel_std    process noise, px/s^2. How hard the subject can change
                     velocity between frames. A person accelerating at ~2 m/s^2
                     at 3 m through a 640 px frame is a few hundred px/s^2.
        h_accel_std  process noise on height. Smaller: range changes slowly
                     compared to lateral position.
        max_coast_s  after this long with no measurement the estimate is no
                     longer offered. Coasting is for occlusion, not for
                     inventing a target that has gone.
        """
        self.pos_std = pos_std
        self.h_std = h_std
        self.accel_std = accel_std
        self.h_accel_std = h_accel_std
        self.max_coast_s = max_coast_s

        self.x = np.zeros(self.NDIM)
        self.P = np.eye(self.NDIM) * 1e4
        self.initialised = False
        self.coast_s = 0.0
        self.updates = 0

    # ------------------------------------------------------------------ core
    def reset(self) -> None:
        self.x = np.zeros(self.NDIM)
        self.P = np.eye(self.NDIM) * 1e4
        self.initialised = False
        self.coast_s = 0.0
        self.updates = 0

    def _F(self, dt: float) -> np.ndarray:
        F = np.eye(self.NDIM)
        F[0, 3] = F[1, 4] = F[2, 5] = dt
        return F

    def _Q(self, dt: float) -> np.ndarray:
        """Piecewise white acceleration.

        For each axis the noise enters as G a with G = [dt^2/2, dt]^T, giving
        the 2x2 block sigma^2 * [[dt^4/4, dt^3/2], [dt^3/2, dt^2]]. The
        off-diagonal terms are the point: position and velocity error from an
        unknown acceleration are correlated, and a diagonal Q denies it.
        """
        Q = np.zeros((self.NDIM, self.NDIM))
        d4, d3, d2 = dt ** 4 / 4.0, dt ** 3 / 2.0, dt ** 2
        for i, s in ((0, self.accel_std), (1, self.accel_std),
                     (2, self.h_accel_std)):
            v = i + 3
            s2 = s * s
            Q[i, i] = d4 * s2
            Q[i, v] = Q[v, i] = d3 * s2
            Q[v, v] = d2 * s2
        return Q

    def predict(self, dt: float) -> None:
        if dt <= 0.0:
            return
        if not self.initialised:
            return
        F = self._F(dt)
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + self._Q(dt)
        self.coast_s += dt

    def update(self, cx: float, cy: float, h: float) -> None:
        z = np.array([float(cx), float(cy), float(h)])
        if not np.all(np.isfinite(z)):
            return

        if not self.initialised:
            # Seed position from the measurement and velocity from nothing —
            # with a large velocity covariance, so the first few updates move
            # it freely rather than fighting a fabricated zero.
            self.x[:3] = z
            self.x[3:] = 0.0
            self.P = np.diag([self.pos_std ** 2, self.pos_std ** 2,
                              self.h_std ** 2,
                              (self.accel_std * 0.5) ** 2,
                              (self.accel_std * 0.5) ** 2,
                              (self.h_accel_std * 0.5) ** 2])
            self.initialised = True
            self.coast_s = 0.0
            self.updates = 1
            return

        H = np.zeros((3, self.NDIM))
        H[0, 0] = H[1, 1] = H[2, 2] = 1.0
        R = np.diag([self.pos_std ** 2, self.pos_std ** 2, self.h_std ** 2])

        y = z - H @ self.x
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)

        self.x = self.x + K @ y
        # Joseph form: stays symmetric positive-definite under floating point
        # in a way the textbook (I - KH)P does not, and this filter runs for
        # the whole of a flight.
        I_KH = np.eye(self.NDIM) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T

        self.coast_s = 0.0
        self.updates += 1

    # ------------------------------------------------------------- accessors
    @property
    def valid(self) -> bool:
        """Usable estimate: seeded, and not coasting past its shelf life."""
        return self.initialised and self.coast_s <= self.max_coast_s

    @property
    def centre(self) -> tuple[float, float]:
        return float(self.x[0]), float(self.x[1])

    @property
    def box_h(self) -> float:
        return float(self.x[2])

    @property
    def velocity(self) -> tuple[float, float]:
        """Centre velocity in PIXELS PER SECOND.

        The unit matters. The previous lead computation worked in px/frame and
        converted by dividing by dt at the point of use, which meant a dropped
        frame changed the meaning of the number rather than its value.
        """
        return float(self.x[3]), float(self.x[4])

    @property
    def pos_std_est(self) -> float:
        """Current 1-sigma positional uncertainty, px. Grows while coasting,
        which is what lets a caller distinguish a confident estimate from an
        extrapolation."""
        return float(np.sqrt(max(self.P[0, 0], 0.0) + max(self.P[1, 1], 0.0)))

    def predict_ahead(self, t_s: float) -> tuple[float, float]:
        """Where the centre will be in t_s seconds, on the current estimate.

        This is what lead compensation wants: not a differenced measurement
        scaled by a guess, but the filter's own forward projection.
        """
        cx, cy = self.centre
        vx, vy = self.velocity
        return cx + vx * t_s, cy + vy * t_s
