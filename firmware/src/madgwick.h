/* Madgwick attitude filter, 6-axis (gyro + accelerometer).
 *
 * The gyro alone gives beautifully smooth attitude that drifts away over
 * tens of seconds. The accelerometer alone gives a drift-free reference that
 * is useless while the aircraft accelerates, which on a quadcopter is always.
 * Madgwick fuses them: integrate the gyro, then nudge the result down the
 * gradient of "does measured gravity agree with where I think down is".
 *
 * `beta` is the whole trade-off. It sets how hard the accelerometer pulls.
 * Too low and gyro bias wins and the horizon slowly tips; too high and every
 * bump in the airframe is read as a change in attitude and the aircraft
 * chases its own vibration. 0.04-0.1 is the usual band for a small quad.
 *
 * No magnetometer, so yaw is gyro-only and WILL drift — about a degree every
 * few seconds. That is fine here: the controller commands a yaw *rate*, and
 * the human (or the vision loop) closes the heading loop by looking at where
 * the aircraft is actually pointing. Absolute heading is never used.
 *
 * Pure maths, no hardware, so the host tests exercise it directly.
 */
#ifndef PURSUIT_MADGWICK_H
#define PURSUIT_MADGWICK_H

#include <stdint.h>

struct EulerAngles {
    /* FLU body frame — see src/frame.h. The pitch sign is the one everyone
     * assumes wrong: in this implementation + is nose DOWN, which happens to
     * match the protocol's command convention exactly, so no conversion sits
     * between setpoint and estimate. The tests in test_attitude.cpp pin all
     * three signs against a physical accelerometer model. */
    float roll;    /* deg, + = right wing down       */
    float pitch;   /* deg, + = nose DOWN             */
    float yaw;     /* deg, + = nose LEFT (CCW from above); wraps, drifts */
};

class Madgwick {
public:
    explicit Madgwick(float beta = 0.06f);

    /* gx/gy/gz in deg/s, ax/ay/az in any consistent unit (only the direction
     * is used). dt_s is the interval since the previous call. */
    void update(float gx, float gy, float gz,
                float ax, float ay, float az, float dt_s);

    EulerAngles euler() const;

    /* Snap straight to whatever the accelerometer says. Used at boot: the
     * filter would otherwise take several seconds to converge from its
     * identity quaternion, and an aircraft that thinks it is level when it is
     * sitting at 15 degrees will lurch on take-off. */
    void align_to_accel(float ax, float ay, float az);

    void reset();

    /* Raised temporarily after reset so the filter converges in a fraction of
     * a second, then decays to the flight value. High gain is right when you
     * know nothing and wrong once you do. */
    void set_beta(float b) { beta_ = b; }
    float beta() const { return beta_; }

    /* True once the estimate has settled enough to fly on. */
    bool converged() const { return converged_; }

    void quaternion(float q[4]) const;

private:
    float q0_, q1_, q2_, q3_;
    float beta_;
    float flight_beta_;
    float settle_s_;
    bool  converged_;
};

#endif /* PURSUIT_MADGWICK_H */
