/* Pack monitoring and throttle limiting.
 *
 * Not a nice-to-have. SPICE on this board's own drive stage (docs/
 * VERIFICATION.md §6) puts the pack at 3.61V during a four-motor full-throttle
 * burst with an optimistic 60mR source model — a real 1S cell is worse. The
 * XIAO's 3V3 comes from an SGM6029C buck that drops out around 3.5V in, so
 * without a limiter the flight controller browns out *in the air*, mid-burst,
 * with the motors still spinning.
 *
 * Two mechanisms, because they solve different problems:
 *
 *   sag limiter   reacts to the INSTANTANEOUS voltage. Fast, so it catches
 *                 the throttle burst that causes the sag in the first place.
 *   state of      reacts to the FILTERED voltage. Slow, so a normal burst
 *   charge        does not read as an empty pack and trigger a landing.
 *
 * Using one filter for both is the obvious simplification and gets you either
 * a limiter too slow to prevent brownout, or an aircraft that lands itself
 * every time you accelerate.
 */
#ifndef PURSUIT_BATTERY_H
#define PURSUIT_BATTERY_H

#include <stdint.h>

class Battery {
public:
    Battery();

    /* Feed a raw pack voltage measurement (already scaled through the
     * divider). `dt_s` is the interval since the previous sample. */
    void update(float volts, float dt_s);

    /* Multiply the throttle command by this. 1.0 = full authority. */
    float throttle_scale() const { return scale_; }

    /* Slow, filtered voltage — for telemetry and the land decision. */
    float volts() const { return slow_v_; }
    /* Fast voltage — what the limiter reacts to. */
    float instant_volts() const { return fast_v_; }

    /* True once the pack is genuinely flat, not merely sagging under load. */
    bool critical() const { return critical_; }
    bool low() const;

    void reset(float volts);

    /* How many samples were thrown away as physically impossible. Rises only
     * while motor PWM is running; a non-zero value at rest would mean
     * something else is wrong. */
    uint32_t glitches() const { return glitches_; }

private:
    float fast_v_;
    float slow_v_;
    float scale_;
    bool  critical_;
    bool  primed_;
    uint32_t glitches_ = 0;
};

/* Convert a raw ADC reading to pack volts through the 100k/100k divider.
 * Separate so the tests can exercise the scaling without an ADC. */
float adc_to_pack_volts(uint32_t raw, uint32_t adc_max, float vref,
                        float divider);

#endif /* PURSUIT_BATTERY_H */
