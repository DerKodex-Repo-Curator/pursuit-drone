#include "battery.h"

#include <cmath>

#include "config.h"

namespace {

float clampf(float v, float lo, float hi)
{
    if (!std::isfinite(v)) return lo;
    return v < lo ? lo : (v > hi ? hi : v);
}

/* First-order low-pass with an explicit time constant, so the filtering does
 * not silently change when the sample rate does. */
float lowpass(float prev, float sample, float dt_s, float tau_s)
{
    if (dt_s <= 0.0f) return prev;
    const float a = dt_s / (tau_s + dt_s);
    return prev + a * (sample - prev);
}

}  /* namespace */

Battery::Battery()
    : fast_v_(VBAT_NOMINAL_V),
      slow_v_(VBAT_NOMINAL_V),
      scale_(1.0f),
      critical_(false),
      primed_(false)
{
}

void Battery::reset(float volts)
{
    if (!std::isfinite(volts)) volts = VBAT_NOMINAL_V;
    fast_v_ = slow_v_ = volts;
    scale_ = 1.0f;
    critical_ = false;
    primed_ = true;
}

void Battery::update(float volts, float dt_s)
{
    if (!std::isfinite(volts)) { glitches_++; return; }

    /* An impossible reading is the instrument, not the aircraft. Dropping it
     * is the whole point: 0.0f is perfectly finite, so before this check a
     * glitched sample went straight into the filters and dragged them down.
     * That is how a healthy pack came to report 1.47 V over telemetry and
     * trip emergency_land(). Counted, not silently swallowed — a rising
     * glitch rate is itself the symptom worth seeing. */
    if (volts < VBAT_SANE_MIN_V) { glitches_++; return; }

    if (!primed_) {
        reset(volts);
        return;
    }

    /* 20ms tracks a throttle burst; 2s ignores one entirely. */
    fast_v_ = lowpass(fast_v_, volts, dt_s, 0.02f);
    slow_v_ = lowpass(slow_v_, volts, dt_s, 2.0f);

    /* --- sag limiter ----------------------------------------------------
     * Full authority above VBAT_NOMINAL_V, tapering to zero extra throttle
     * at VBAT_CRITICAL_V. Reacting to the fast voltage is the point: by the
     * time a 2s filter notices the sag, the buck has already dropped out.
     */
    if (fast_v_ >= VBAT_NOMINAL_V) {
        scale_ = 1.0f;
    } else if (fast_v_ <= VBAT_CRITICAL_V) {
        /* Never zero. Cutting throttle entirely at low voltage turns a
         * controlled descent into a fall, and a falling aircraft draws no
         * current — so the voltage recovers, throttle returns, and it
         * oscillates its way to the ground. A floor keeps it flyable. */
        scale_ = 0.35f;
    } else {
        const float span = VBAT_NOMINAL_V - VBAT_CRITICAL_V;
        const float t = (fast_v_ - VBAT_CRITICAL_V) / span;
        scale_ = 0.35f + 0.65f * t;
    }
    scale_ = clampf(scale_, 0.0f, 1.0f);

    /* --- state of charge -------------------------------------------------
     * Latching, and driven by the SLOW voltage: a pack that dips to 3.4V
     * under a burst is not flat, but one that sits there is. Latching stops
     * it flickering as load varies.
     */
    if (slow_v_ <= VBAT_CRITICAL_V) critical_ = true;
}

bool Battery::low() const
{
    return slow_v_ <= VBAT_LOW_V;
}

float adc_to_pack_volts(uint32_t raw, uint32_t adc_max, float vref,
                        float divider)
{
    if (adc_max == 0) return 0.0f;
    const float at_pin = (float)raw / (float)adc_max * vref;
    return at_pin * divider;
}
