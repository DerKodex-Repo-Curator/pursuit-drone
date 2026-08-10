/* X-quad motor mixing.
 *
 * Turns a desired (roll, pitch, yaw, throttle) into four motor demands. Pure
 * arithmetic, no Arduino headers, so the host test build compiles it directly
 * — a sign error here flips the aircraft on take-off and is not something to
 * find with props fitted.
 *
 * Geometry, seen from above with the USB-C edge forward:
 *
 *      M1 (FL, CW)      M2 (FR, CCW)
 *              \        /
 *               +------+        front
 *              /        \
 *      M3 (RL, CCW)     M4 (RR, CW)
 *
 * Roll right  -> lift the left side  -> M1, M3 up   / M2, M4 down
 * Pitch fwd   -> lift the back       -> M3, M4 up   / M1, M2 down
 * Yaw right   -> more torque from the CCW pair, which pushes the airframe the
 *                other way -> M2, M3 up / M1, M4 down
 *
 * The yaw row is the one people get backwards: a motor's reaction torque
 * opposes its own rotation, so speeding up the CCW pair yaws the aircraft
 * clockwise.
 */
#ifndef PURSUIT_MIXER_H
#define PURSUIT_MIXER_H

#include <stdint.h>

struct MotorOutputs {
    float m1, m2, m3, m4;   /* 0..1 */
};

/* Desaturation strategy when the mix asks for more than a motor can give.
 *
 * Naive clipping silently changes the attitude command: clip one motor and
 * the differential between that pair no longer matches what the controller
 * asked for, so the aircraft rolls when it was told to climb. Instead the
 * whole mix is shifted in common mode — trading throttle, which the pilot
 * barely notices, for attitude authority, which keeps it upright.
 */
MotorOutputs mix_quad_x(float roll, float pitch, float yaw, float throttle);

/* Motors below a small floor stop turning entirely, and a stopped motor
 * produces no control authority at all — the aircraft becomes uncontrollable
 * on that axis until it spins up again, which takes tens of milliseconds.
 * Holding an idle floor keeps every axis live.
 */
MotorOutputs apply_idle_floor(MotorOutputs m, float floor_duty, bool armed);

/* Scale 0..1 to the PWM range, clamping and rejecting non-finite input. */
uint32_t duty_to_pwm(float duty, uint32_t pwm_max);

#endif /* PURSUIT_MIXER_H */
