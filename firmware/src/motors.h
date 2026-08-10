/* Motor PWM output.
 *
 * The only interesting part is the phase stagger, and it is not optional.
 *
 * By default every LEDC channel on a timer rises on the same edge. All four
 * gates then switch on simultaneously, so the four ~2.75A motor currents
 * become one synchronised ~11A step on a 1S pack with maybe 60mR of source
 * impedance. That is most of the rail sag this board's SPICE model predicts,
 * and it is entirely self-inflicted.
 *
 * Staggering the channels by 90 degrees (hpoint = 0, T/4, T/2, 3T/4) spreads
 * the four edges across the period. Peak pack current drops to roughly one
 * motor's worth, and C1's ripple heating roughly halves. It costs nothing but
 * setting a register field the simple Arduino ledcWrite() API does not expose
 * — which is why this uses the IDF driver directly.
 */
#ifndef PURSUIT_MOTORS_H
#define PURSUIT_MOTORS_H

#include <stdint.h>

#include "mixer.h"

/* Configure the timer and all four channels. Safe to call once at boot;
 * leaves every motor stopped. */
void motors_begin(void);

/* Write four 0..1 duties, preserving the per-channel phase offsets. */
void motors_write(const MotorOutputs &m);

/* Immediately stop all four. Used by disarm, failsafe and the panic paths;
 * does not depend on the mixer or on any state being consistent. */
void motors_stop(void);

#endif /* PURSUIT_MOTORS_H */
