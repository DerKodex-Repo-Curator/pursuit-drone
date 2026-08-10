#include "pid.h"

#include <cmath>

namespace {

float clampf(float v, float lo, float hi)
{
    if (!std::isfinite(v)) return 0.0f;
    return v < lo ? lo : (v > hi ? hi : v);
}

/* One-pole low-pass expressed by cutoff frequency rather than a raw
 * coefficient, so the filtering does not silently change if the loop rate
 * does. */
float lpf(float prev, float sample, float dt_s, float cutoff_hz)
{
    if (cutoff_hz <= 0.0f || dt_s <= 0.0f) return sample;
    const float rc = 1.0f / (6.2831853f * cutoff_hz);
    const float a = dt_s / (rc + dt_s);
    return prev + a * (sample - prev);
}

}  /* namespace */

/* ------------------------------------------------------------------- rate */

RatePid::RatePid()
    : g_{0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
      integral_(0.0f), prev_measured_(0.0f), d_filtered_(0.0f),
      output_(0.0f), primed_(false)
{
}

void RatePid::configure(const PidGains &g)
{
    g_ = g;
    reset();
}

void RatePid::reset()
{
    integral_ = 0.0f;
    prev_measured_ = 0.0f;
    d_filtered_ = 0.0f;
    output_ = 0.0f;
    primed_ = false;
}

float RatePid::update(float setpoint_dps, float measured_dps, float dt_s,
                      float limit)
{
    if (!(dt_s > 0.0f) || dt_s > 0.5f) return output_;
    if (!std::isfinite(setpoint_dps)) setpoint_dps = 0.0f;
    if (!std::isfinite(measured_dps)) return output_;

    const float error = setpoint_dps - measured_dps;

    /* --- derivative, on measurement --------------------------------------
     * Differentiating the error would spike every time the setpoint moved.
     * The sign flips because d(error)/dt = -d(measurement)/dt when the
     * setpoint is constant, and we want the damping to act the same way.
     */
    float derivative = 0.0f;
    if (primed_) {
        const float raw = -(measured_dps - prev_measured_) / dt_s;
        d_filtered_ = lpf(d_filtered_, raw, dt_s, g_.d_lpf_hz);
        derivative = d_filtered_;
    } else {
        /* First sample after a reset has no previous measurement, so the
         * derivative would be the entire rate divided by dt — a huge kick at
         * exactly the moment of arming. */
        primed_ = true;
    }
    prev_measured_ = measured_dps;

    const float p_term = g_.kp * error;
    const float d_term = g_.kd * derivative;

    /* --- integral, with conditional integration --------------------------
     * Accumulate first, then decide whether to keep it. If the unsaturated
     * output is already beyond the limit and the error would push it further
     * that way, the integrator is storing authority that does not exist —
     * and will spend it as overshoot once the aircraft catches up.
     */
    const float candidate = integral_ + g_.ki * error * dt_s;
    const float unsat = p_term + candidate + d_term;

    const bool pushing_further =
        (unsat > limit && error > 0.0f) || (unsat < -limit && error < 0.0f);

    if (!pushing_further) {
        integral_ = clampf(candidate, -g_.i_limit, g_.i_limit);
    }

    output_ = clampf(p_term + integral_ + d_term, -limit, limit);
    return output_;
}

/* ------------------------------------------------------------------ angle */

AnglePid::AnglePid() : kp_(0.0f), max_rate_dps_(0.0f) {}

void AnglePid::configure(float kp, float max_rate_dps)
{
    kp_ = kp;
    max_rate_dps_ = max_rate_dps;
}

float AnglePid::update(float setpoint_deg, float measured_deg) const
{
    if (!std::isfinite(setpoint_deg)) setpoint_deg = 0.0f;
    if (!std::isfinite(measured_deg)) return 0.0f;
    const float error = setpoint_deg - measured_deg;
    return clampf(kp_ * error, -max_rate_dps_, max_rate_dps_);
}

/* ---------------------------------------------------------------- presets */

PidGains default_roll_rate_gains()
{
    /* d_lpf at 80Hz: below the motor/prop noise that dominates above ~150Hz,
     * above the ~20Hz airframe response we actually want to damp. */
    return PidGains{0.0016f, 0.012f, 0.000045f, 0.25f, 80.0f};
}

PidGains default_pitch_rate_gains()
{
    /* Same as roll — the airframe is symmetric in X, and the battery sits on
     * the centreline, so there is no reason for the axes to differ. */
    return default_roll_rate_gains();
}

PidGains default_yaw_rate_gains()
{
    /* Yaw authority on a brushed quad comes only from motor torque
     * differences, which is weak, so P is higher and D is zero — there is
     * nothing fast enough on this axis for D to damp, and it would only
     * amplify noise. */
    return PidGains{0.0045f, 0.030f, 0.0f, 0.30f, 0.0f};
}
