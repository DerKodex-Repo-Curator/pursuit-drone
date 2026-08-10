/* Cascaded attitude control: angle loop wrapped around a rate loop.
 *
 *   angle setpoint --> [ P ] --> rate setpoint --> [ PID ] --> motor demand
 *                       ^                            ^
 *                  attitude                        gyro
 *
 * Two loops rather than one because they answer different questions. The
 * inner loop asks "am I rotating at the rate I was told?" and can run hard
 * and fast against a clean gyro signal. The outer loop asks "am I at the
 * angle I was told?" and must be gentle, because its input is a filtered,
 * slightly lagged estimate. A single angle->motor loop has to be tuned for
 * the worse of the two and ends up sluggish and prone to oscillation.
 *
 * Three details that matter more than the gains:
 *
 * **D on measurement, not error.** A step change in setpoint makes the error
 * jump, and differentiating that jump produces an enormous one-frame spike —
 * "derivative kick". Differentiating the *measurement* gives the same
 * damping with no kick, because the aircraft's actual rotation rate cannot
 * step.
 *
 * **D low-passed.** The gyro is the noisiest signal on the aircraft and
 * differentiation amplifies noise by frequency. Unfiltered D at 1kHz turns
 * motor hiss into control output and heats the motors for nothing.
 *
 * **Conditional integration.** When the output is already saturated the
 * integrator keeps accumulating an error it cannot fix, so when the aircraft
 * finally responds there is a large stored term driving it past the target.
 * Freezing the integrator while saturated in the direction of saturation is
 * the fix; a plain clamp alone is not enough.
 *
 * Pure maths, no hardware — fully host-tested.
 */
#ifndef PURSUIT_PID_H
#define PURSUIT_PID_H

#include <stdint.h>

struct PidGains {
    float kp;
    float ki;
    float kd;
    float i_limit;      /* absolute cap on the integral term's contribution */
    float d_lpf_hz;     /* derivative low-pass cutoff; 0 disables */
};

class RatePid {
public:
    RatePid();
    void configure(const PidGains &g);

    /* setpoint and measurement in deg/s; returns a normalised demand which
     * the caller clamps and hands to the mixer. `limit` is the output
     * magnitude beyond which the integrator stops accumulating. */
    float update(float setpoint_dps, float measured_dps, float dt_s,
                 float limit);

    void reset();

    float integral() const { return integral_; }
    float last_output() const { return output_; }

private:
    PidGains g_;
    float integral_;
    float prev_measured_;
    float d_filtered_;
    float output_;
    bool  primed_;
};

/* Outer loop. Proportional only, deliberately: an integrator here fights the
 * inner loop's integrator for the same authority, and the pair oscillate
 * against each other in a way that is very hard to tune out. */
class AnglePid {
public:
    AnglePid();
    void configure(float kp, float max_rate_dps);

    float update(float setpoint_deg, float measured_deg) const;

private:
    float kp_;
    float max_rate_dps_;
};

/* Default gains for a ~66g whoop-class quad on 8520 brushed motors.
 *
 * STARTING POINTS, NOT TUNED VALUES. They are scaled from typical brushed
 * whoop setups and have never flown this airframe. Expect to halve them if it
 * oscillates and raise them if it feels mushy — and do that with the aircraft
 * held down, props on, one axis at a time.
 */
PidGains default_roll_rate_gains();
PidGains default_pitch_rate_gains();
PidGains default_yaw_rate_gains();

#endif /* PURSUIT_PID_H */
