#include "mixer.h"

#include <cmath>

namespace {

float clampf(float v, float lo, float hi)
{
    /* Non-finite first: NaN fails both comparisons, so a plain two-sided
     * clamp passes it straight through to the PWM register. */
    if (!std::isfinite(v)) return lo;
    return v < lo ? lo : (v > hi ? hi : v);
}

}  /* namespace */

MotorOutputs mix_quad_x(float roll, float pitch, float yaw, float throttle)
{
    if (!std::isfinite(roll))     roll = 0.0f;
    if (!std::isfinite(pitch))    pitch = 0.0f;
    if (!std::isfinite(yaw))      yaw = 0.0f;
    if (!std::isfinite(throttle)) throttle = 0.0f;

    /* Signs per the table in mixer.h. */
    float m1 = throttle + roll + pitch - yaw;   /* front-left,  CW  */
    float m2 = throttle - roll + pitch + yaw;   /* front-right, CCW */
    float m3 = throttle + roll - pitch + yaw;   /* rear-left,   CCW */
    float m4 = throttle - roll - pitch - yaw;   /* rear-right,  CW  */

    /* --- desaturation ----------------------------------------------------
     * Shift the whole mix in common mode rather than clipping individual
     * motors. Clipping one motor breaks the differential the controller
     * asked for, so a pure throttle command starts producing roll — the
     * aircraft tips at exactly the moment it is working hardest.
     */
    const float hi = fmaxf(fmaxf(m1, m2), fmaxf(m3, m4));
    const float lo = fminf(fminf(m1, m2), fminf(m3, m4));

    if (hi > 1.0f) {
        const float shift = hi - 1.0f;
        m1 -= shift; m2 -= shift; m3 -= shift; m4 -= shift;
    }
    const float lo2 = fminf(fminf(m1, m2), fminf(m3, m4));
    if (lo2 < 0.0f) {
        const float shift = -lo2;
        m1 += shift; m2 += shift; m3 += shift; m4 += shift;
    }

    /* If the demand was so large that shifting both ways cannot satisfy it,
     * attitude wins and throttle is sacrificed — an aircraft at the wrong
     * height is recoverable, one at the wrong attitude is not. The clamp
     * below is what actually gives up, and only after both shifts. */
    MotorOutputs out;
    out.m1 = clampf(m1, 0.0f, 1.0f);
    out.m2 = clampf(m2, 0.0f, 1.0f);
    out.m3 = clampf(m3, 0.0f, 1.0f);
    out.m4 = clampf(m4, 0.0f, 1.0f);
    (void)lo;
    return out;
}

MotorOutputs apply_idle_floor(MotorOutputs m, float floor_duty, bool armed)
{
    if (!armed) {
        /* Disarmed means stopped. Not "nearly stopped". */
        return MotorOutputs{0.0f, 0.0f, 0.0f, 0.0f};
    }
    floor_duty = clampf(floor_duty, 0.0f, 0.5f);
    m.m1 = fmaxf(m.m1, floor_duty);
    m.m2 = fmaxf(m.m2, floor_duty);
    m.m3 = fmaxf(m.m3, floor_duty);
    m.m4 = fmaxf(m.m4, floor_duty);
    return m;
}

uint32_t duty_to_pwm(float duty, uint32_t pwm_max)
{
    duty = clampf(duty, 0.0f, 1.0f);
    return (uint32_t)lroundf(duty * (float)pwm_max);
}
