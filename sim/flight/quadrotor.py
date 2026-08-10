"""A 6-DOF rigid-body model of the PURSUIT-1 airframe.

Why this exists
---------------
Every gain in this project is a guess. `ANGLE_KP = 6.0f` in main.cpp and the
rate gains in pid.cpp are scaled from typical brushed-whoop values and have
never touched this airframe. `vision/sim_drone.py` does not help: it is a link
simulator, and says so — its altitude "dynamics" are a first-order lag toward
the setpoint, which is a plant that is stable by construction and therefore
cannot tell you anything about whether yours is.

So there was no way to answer the first question any interviewer asks about a
control system: how do you know it is stable? The honest answer was "I don't."

This is the smallest model that can answer it. It is not a flight simulator —
no aerodynamics, no ground effect, no wind, no battery sag, no prop wash. It is
rigid-body rotational dynamics plus the two things that actually set the
stability margin of a cascaded attitude controller: **motor lag** and
**transport delay**.

What is modelled, and why each one matters
------------------------------------------
* **Rotational dynamics** about all three body axes, including the gyroscopic
  cross-coupling term ``w x (I w)``. Small at these rates, kept because
  dropping it is the kind of simplification that makes a model agree with you.
* **First-order motor lag.** A coreless motor and prop cannot change speed
  instantly; the rotor has inertia and the driver is a low-side FET with no
  active braking. This lag is the dominant phase loss in the inner loop and is
  the single most important term here. Set it to zero and the rate loop looks
  stable at gains that will oscillate on the bench.
* **Quadratic thrust.** Thrust goes as the square of rotor speed, and for a
  brushed motor at roughly constant load, speed goes as duty. So thrust goes
  as duty squared. Modelling it as linear makes the loop gain wrong by a factor
  of two at hover, in the optimistic direction.
* **Thrust saturation.** Duty clamps to [0, 1]. A controller that only works
  because it was allowed to command 130% throttle is not a working controller.

Assumptions, and how wrong they might be
----------------------------------------
The mass properties are ESTIMATES, from the budget in the top-level README:
66 g all-up, 102 g usable static thrust after the pack sag this board's own
SPICE model predicts. The inertias are computed from a plate plus four point
masses, not measured. The motor time constant is a literature value for
8520-class coreless motors, not measured on these ones.

That makes every absolute number here provisional. What survives being wrong
is the SHAPE of the result — the trend of stability against transport delay,
and the ordering of which loop gives out first. Inertia enters the inner loop
gain linearly, so a 30% inertia error moves the stability boundary by roughly
30%, and the sweep in lag_sweep.py reports the margin so that a 30% error is
visible rather than fatal.

Measure ``TAU_MOTOR`` first when the hardware exists: step one motor with the
bench console and watch the blackbox. It is both the least certain number and
the one the answer is most sensitive to.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

G = 9.80665


@dataclass
class Airframe:
    """Physical constants. Defaults are the PURSUIT-1 estimates."""

    mass_kg: float = 0.066
    """66 g all-up: frame 13, XIAO+camera 6, motors 21, props 2, IMU and
    passives 4, 650 mAh pack 18, wire 2."""

    arm_m: float = 0.0354
    """Motors sit on the diagonals of a 100 mm wheelbase, so each is
    100/2 * cos(45) = 35.4 mm off both the roll and the pitch axis. Using the
    50 mm half-wheelbase here instead is a common slip and inflates control
    authority by 41%."""

    thrust_max_n: float = 0.343
    """Per motor at 100% duty, STATIC, before pack sag: the README budget's
    ~140 g nominal, so 0.140 * 9.81 / 4 = 0.343 N.

    Deliberately the nominal figure and not the ~102 g sag-derated one. Sag is
    a function of current draw over time and is not modelled here; folding it
    into a static constant would double-count it the moment it ever is. With
    this value hover lands at 69% duty, which matches the 60-65% the mass
    budget predicts — a useful check that the quadratic thrust curve and the
    mass estimate agree with each other."""

    tau_motor_s: float = 0.035
    """Rotor + prop spin-up time constant. THE most influential number in this
    file and the least certain. Coreless 8520s are typically 25-45 ms."""

    k_drag: float = 0.020
    """Yaw torque per unit thrust, N.m/N. Reaction torque of the prop against
    the air. Small props on a light airframe: 0.015-0.025 is the usual band."""

    Ixx: float = 3.1e-5
    Iyy: float = 3.1e-5
    Izz: float = 6.0e-5
    """Plate of 92 mm square at 25 g, plus four 2.7 g motors at 35.4 mm.
    Roll and pitch are equal because the airframe is symmetric in X and the
    pack sits on the centreline. Estimated, not measured."""

    def hover_duty(self) -> float:
        """Duty at which the four motors exactly carry the weight.

        Thrust is quadratic in duty, so this is not 'weight over max thrust'.
        Getting it wrong shifts the operating point at which the loop gain is
        evaluated, and loop gain varies with it.
        """
        need_per_motor = self.mass_kg * G / 4.0
        return float(np.sqrt(np.clip(need_per_motor / self.thrust_max_n, 0.0, 1.0)))

    def thrust_to_weight(self) -> float:
        return 4.0 * self.thrust_max_n / (self.mass_kg * G)


@dataclass
class Quadrotor:
    """Rotational state of the airframe, integrated at the loop rate.

    Attitude is carried as roll/pitch/yaw in radians. Euler angles are the
    wrong representation near +/-90 degrees of pitch, which is fine here: the
    firmware clamps commanded attitude to 25 degrees and anything past 60 is
    already a crash, not a control problem. The integrator flags it rather
    than pretending.

    Frame is FLU, matching firmware/src/frame.h: +X forward, +Y LEFT, +Z up,
    right-handed. That means a clockwise-from-above yaw is NEGATIVE r. This is
    the convention that was inverted in the firmware and would have produced
    yaw runaway on the first hover; it is repeated here deliberately so that
    the simulation and the aircraft share one definition rather than two.
    """

    frame: Airframe = field(default_factory=Airframe)

    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    p: float = 0.0
    q: float = 0.0
    r: float = 0.0

    duty: np.ndarray = field(default_factory=lambda: np.zeros(4))
    """Actual motor duty, lagging the commanded duty. Order matches
    scripts/design.py and the firmware mixer: M1 front-left, M2 front-right,
    M3 rear-left, M4 rear-right."""

    diverged: bool = False

    def reset(self) -> None:
        self.roll = self.pitch = self.yaw = 0.0
        self.p = self.q = self.r = 0.0
        self.duty = np.zeros(4)
        self.diverged = False

    # ------------------------------------------------------------------ step
    def step(self, duty_cmd: np.ndarray, dt: float) -> None:
        """Advance one control period under a commanded motor duty vector."""
        f = self.frame

        duty_cmd = np.clip(np.asarray(duty_cmd, dtype=float), 0.0, 1.0)

        # Motor lag. Exact discretisation of a first-order lag rather than a
        # forward-Euler approximation, so the result does not depend on the
        # step size — which matters because this is swept.
        alpha = 1.0 - np.exp(-dt / f.tau_motor_s)
        self.duty += alpha * (duty_cmd - self.duty)

        thrust = f.thrust_max_n * self.duty ** 2          # N, per motor

        # Mixer geometry, mirroring firmware/src/mixer.cpp.
        #   M1 front-left   M2 front-right
        #   M3 rear-left    M4 rear-right
        # +Y is LEFT, so the left pair (M1, M3) produces a NEGATIVE roll
        # moment about +X when it pushes harder than the right pair.
        t1, t2, t3, t4 = thrust
        tau_x = f.arm_m * ((t2 + t4) - (t1 + t3))         # roll,  right - left
        tau_y = f.arm_m * ((t3 + t4) - (t1 + t2))         # pitch, rear  - front
        # Reaction torque. M2 and M3 spin CCW seen from above, M1 and M4 CW;
        # a prop's drag torque opposes its own rotation.
        tau_z = f.k_drag * ((t2 + t3) - (t1 + t4))

        tau = np.array([tau_x, tau_y, tau_z])
        w = np.array([self.p, self.q, self.r])
        I = np.array([f.Ixx, f.Iyy, f.Izz])

        # Euler's equation. The cross term is small at these rates but free.
        wdot = (tau - np.cross(w, I * w)) / I

        self.p += wdot[0] * dt
        self.q += wdot[1] * dt
        self.r += wdot[2] * dt

        # Body rates to Euler rates. At the 25 degree limit the small-angle
        # approximation is already 10% wrong on the yaw coupling, so it is
        # done properly.
        cr, sr = np.cos(self.roll), np.sin(self.roll)
        cp_, tp = np.cos(self.pitch), np.tan(self.pitch)
        if abs(cp_) < 1e-6:
            self.diverged = True
            return
        self.roll += (self.p + self.q * sr * tp + self.r * cr * tp) * dt
        self.pitch += (self.q * cr - self.r * sr) * dt
        self.yaw += (self.q * sr / cp_ + self.r * cr / cp_) * dt

        if (abs(self.roll) > np.pi / 2 or abs(self.pitch) > np.pi / 2
                or not np.isfinite(self.roll) or not np.isfinite(self.pitch)):
            self.diverged = True

    # --------------------------------------------------------------- helpers
    @property
    def roll_deg(self) -> float:
        return float(np.degrees(self.roll))

    @property
    def pitch_deg(self) -> float:
        return float(np.degrees(self.pitch))

    @property
    def yaw_deg(self) -> float:
        return float(np.degrees(self.yaw))

    @property
    def p_dps(self) -> float:
        return float(np.degrees(self.p))

    @property
    def q_dps(self) -> float:
        return float(np.degrees(self.q))

    @property
    def r_dps(self) -> float:
        return float(np.degrees(self.r))
