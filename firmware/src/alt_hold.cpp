#include "alt_hold.h"

#include <cmath>

namespace {

float clampf(float v, float lo, float hi)
{
    if (!std::isfinite(v)) return lo;
    return v < lo ? lo : (v > hi ? hi : v);
}

/* Only learn hover while genuinely holding station. Learning during a climb
 * teaches the aircraft that climb throttle is hover throttle, and it then
 * climbs forever. */
constexpr float LEARN_ERR_M = 0.12f;
constexpr float LEARN_RATE_MS = 0.15f;

}  /* namespace */

AltHoldConfig default_alt_hold_config()
{
    AltHoldConfig c;
    /* Modest gains because the baseline does the heavy lifting — the PID only
     * ever supplies the difference from hover. */
    c.kp = 0.35f;
    c.ki = 0.08f;
    c.kd = 0.22f;
    c.i_limit = 0.20f;
    c.max_climb_ms = 0.6f;
    c.hover_learn_hz = 0.05f;   /* ~20s time constant */
    c.min_throttle = 0.15f;
    c.max_throttle = 0.90f;
    /* Hover throttle, MEASURED on the airframe 2026-08-20.
     *
     * Props on, board restrained, all four motors ramped in steps: the frame
     * was straining and nearly free of the surface at 29% per motor. So hover
     * is near 0.30.
     *
     * The previous 0.79 was never measured. It came from a T:W estimate of
     * ~1.6 -- hover near sqrt(1/1.6) of full throttle -- and that 1.6 came in
     * turn from the SPICE power model whose assumptions have now been
     * contradicted twice on hardware. The real airframe is far better than
     * that: hovering at ~30% throttle implies a thrust-to-weight nearer 3
     * than 1.6.
     *
     * Getting this wrong upward is the dangerous direction. hover_ is the
     * FEEDFORWARD -- what the loop commands before any correction -- so 0.79
     * against a true 0.30 would open at more than double the throttle needed
     * and take ~20 s (hover_learn_hz = 0.05) to learn back down. That is a
     * launch, not a lift.
     *
     * Caveat on the number: it was measured on a pack sagging to 3.02 V under
     * that load. Less voltage means less thrust per unit duty, so a CHARGED
     * pack should hover somewhat below 0.30. 0.30 is therefore a slight
     * overestimate -- re-measure once the pack is healthy. The learner trims
     * from here either way; it only has to start in the right neighbourhood,
     * and 0.30 is the right neighbourhood where 0.79 was not. */
    return c;
}

float pursuit_clamp_alt_setpoint(float requested_m, bool baro_present,
                                 float tof_ceiling_m, float baro_ceiling_m)
{
    const float ceiling = baro_present ? baro_ceiling_m : tof_ceiling_m;
    /* Floor at zero rather than at the ceiling: a garbled setpoint should make
     * the aircraft descend to the ground, not climb to the limit. */
    return clampf(requested_m, 0.0f, ceiling);
}

AltitudeHold::AltitudeHold()
    : c_(default_alt_hold_config()),
      hover_(0.30f), integral_(0.0f), sp_m_(0.0f),
      prev_height_(0.0f), climb_ms_(0.0f), primed_(false), holding_(false)
{
}

void AltitudeHold::configure(const AltHoldConfig &c)
{
    c_ = c;
}

void AltitudeHold::arm(float current_height_m)
{
    /* Start the setpoint where the aircraft already is, so engaging altitude
     * hold does not command a step. */
    sp_m_ = std::isfinite(current_height_m) ? current_height_m : 0.0f;
    integral_ = 0.0f;
    climb_ms_ = 0.0f;
    prev_height_ = sp_m_;
    primed_ = false;
    holding_ = false;
    /* hover_ deliberately survives: it is a property of this airframe and
     * this battery, not of this particular arming. */
}

void AltitudeHold::reset_learning()
{
    hover_ = 0.30f;
}

float AltitudeHold::update(float target_m, float height_m, bool have_height,
                           float fallback_throttle, float dt_s)
{
    if (!(dt_s > 0.0f) || dt_s > 0.5f) dt_s = 0.03f;

    if (!have_height || !std::isfinite(height_m)) {
        /* No usable height. Hand control back rather than holding an
         * altitude the aircraft cannot measure — a loop flying on a stale
         * number drifts silently into the floor or the ceiling. */
        holding_ = false;
        integral_ = 0.0f;
        primed_ = false;
        return clampf(fallback_throttle, 0.0f, c_.max_throttle);
    }

    /* --- rate-limited setpoint ------------------------------------------
     * The commanded target can jump (the vision loop retargets, the pilot
     * flicks a stick). Slewing it means the aircraft climbs at a chosen rate
     * instead of demanding full throttle at the instant of the change. */
    const float want = std::isfinite(target_m) ? target_m : sp_m_;
    const float max_step = c_.max_climb_ms * dt_s;
    sp_m_ += clampf(want - sp_m_, -max_step, max_step);

    /* --- climb rate from the height signal -------------------------------
     * Differentiating a noisy rangefinder is unpleasant, so it is heavily
     * smoothed. This is the D term's input, and it is on the MEASUREMENT for
     * the same reason as the rate loop: a setpoint step must not produce a
     * derivative spike. */
    if (primed_) {
        const float raw = (height_m - prev_height_) / dt_s;
        climb_ms_ += 0.25f * (raw - climb_ms_);
    } else {
        primed_ = true;
    }
    prev_height_ = height_m;

    const float error = sp_m_ - height_m;

    const float p = c_.kp * error;
    const float d = -c_.kd * climb_ms_;   /* oppose motion, not error change */

    const float candidate = integral_ + c_.ki * error * dt_s;
    const float unsat = hover_ + p + candidate + d;
    const bool pushing_further =
        (unsat > c_.max_throttle && error > 0.0f) ||
        (unsat < c_.min_throttle && error < 0.0f);
    if (!pushing_further) {
        integral_ = clampf(candidate, -c_.i_limit, c_.i_limit);
    }

    const float out = clampf(hover_ + p + integral_ + d,
                             c_.min_throttle, c_.max_throttle);

    /* --- learn hover -----------------------------------------------------
     * Only while actually holding station: small error AND small vertical
     * speed. Then drift the baseline toward what is currently being commanded,
     * and bleed the integrator into it so the two do not both hold the same
     * offset.
     */
    holding_ = fabsf(error) < LEARN_ERR_M && fabsf(climb_ms_) < LEARN_RATE_MS;
    if (holding_) {
        const float a = clampf(c_.hover_learn_hz * dt_s, 0.0f, 0.2f);
        hover_ += a * (out - hover_);
        hover_ = clampf(hover_, c_.min_throttle, c_.max_throttle);
        integral_ *= (1.0f - a);
    }

    return out;
}
