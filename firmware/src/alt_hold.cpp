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
    /* Hover throttle. DERIVED, NOT MEASURED -- and this constant has now been
     * wrong three times, so read the whole comment before changing it again.
     *
     * History: 0.62, then 0.79, then 0.30. None of the three was measured.
     *
     * The 0.30 came from a props-on ramp on 2026-08-20 where the operator saw
     * the restrained frame straining and "nearly free of the surface" at 29%
     * per motor. Three things make that unsound as a hover measurement:
     *
     *   1. The bench ramp CANNOT COMMAND MORE THAN 35% -- benchtest.cpp clamps
     *      g_duty to 0.35. The observation was taken a hair under the tool's
     *      own ceiling, so it bounds hover from BELOW and nothing more. "It
     *      almost lifted at the highest duty available" is not "hover is 29%".
     *   2. Straining against a restraint is a subjective judgement of restraint
     *      force, not a weight measurement. Nothing was on a scale.
     *   3. The pack voltage quoted alongside it (3.02 V) came through the
     *      100k/100k divider that commit 2676ce2 proved was corrupting this
     *      board's ADC, on a board whose VBAT_SENSE joint was independently
     *      found dead the same week.
     *
     * What the physics says. Thrust on a brushed motor goes as duty^2, so
     * hover duty = sqrt(W / T_max). At the README's 66 g AUW:
     *
     *      T_max 140 g (nominal)  ->  hover 0.69
     *      T_max 102 g (derated)  ->  hover 0.80
     *
     * and inverting, hover_ = 0.30 would require 183 g of thrust PER 8520
     * motor, 733 g total. An 8520 coreless on a 65 mm prop makes ~35 g. The
     * 0.30 is not off by a little; it is off by a factor of five, and the
     * README's own "hover sits near 60-65% throttle" agrees with the physics.
     *
     * So: 0.69, the LOW end of the physically defensible band. Low is the safe
     * direction to be wrong. Too little feedforward and the aircraft simply
     * does not leave the ground while the integrator makes up the difference;
     * too much and the loop opens above hover and climbs -- and the learner
     * cannot climb back down quickly (hover_learn_hz = 0.05, ~20 s). 0.69 also
     * sits within learner range of BOTH ends of the band, which 0.30 did not.
     *
     * THIS NUMBER STILL NEEDS MEASURING, and it is an afternoon's work:
     * restrain the airframe inverted over a kitchen scale, run the four-motor
     * ramp, and record grams against duty against pack voltage. Until someone
     * does that, treat altitude hold as unproven. */
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
      hover_(0.69f), integral_(0.0f), sp_m_(0.0f),
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
    hover_ = 0.69f;
}

void AltitudeHold::set_pack_voltage(float volts)
{
    /* 3.0..4.35 V is the plausible 1S window; anything else (0 at boot, a
     * broken divider, NaN) leaves compensation OFF rather than trusting it. */
    if (std::isfinite(volts) && volts > 3.0f && volts < 4.35f) {
        vscale_ = clampf(3.70f / volts, 0.85f, 1.25f);
    } else {
        vscale_ = 1.0f;
    }
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

    /* hover_ is the duty at V_REF; hover_eff is what THIS pack voltage
     * needs for the same thrust. */
    const float hover_eff = clampf(hover_ * vscale_,
                                   c_.min_throttle, c_.max_throttle);

    const float candidate = integral_ + c_.ki * error * dt_s;
    const float unsat = hover_eff + p + candidate + d;
    const bool pushing_further =
        (unsat > c_.max_throttle && error > 0.0f) ||
        (unsat < c_.min_throttle && error < 0.0f);
    if (!pushing_further) {
        integral_ = clampf(candidate, -c_.i_limit, c_.i_limit);
    }

    const float out = clampf(hover_eff + p + integral_ + d,
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
        /* Normalise the observed steady output back to V_REF before learning,
         * or a hover learned on a full pack is wrong on a sagged one -- the
         * exact drift this compensation exists to remove. */
        hover_ += a * (out / vscale_ - hover_);
        hover_ = clampf(hover_, c_.min_throttle, c_.max_throttle);
        integral_ *= (1.0f - a);
    }

    return out;
}
