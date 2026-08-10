/* Attitude estimation and control, tested on the host.
 *
 * These are the behaviours that decide whether the aircraft holds itself
 * upright, and none of them need hardware to check: whether the filter
 * converges to the right angle, whether the derivative kicks on a setpoint
 * step, whether the integrator winds up while saturated.
 *
 *   pio test -e native
 */
#include <unity.h>

#include <cmath>

#include "frame.h"
#include "madgwick.h"
#include "pid.h"

static const float DT = 1.0f / 1000.0f;   /* the real loop rate */
static const float D2R = 3.14159265f / 180.0f;

/* Feed a static orientation for `seconds`, gyro reading zero. */
static void settle(Madgwick &f, float ax, float ay, float az, float seconds)
{
    const int n = (int)(seconds / DT);
    for (int i = 0; i < n; i++) f.update(0, 0, 0, ax, ay, az, DT);
}

/* What a real accelerometer reads at a given attitude.
 *
 * Spelled out as a helper because the sign convention is the single easiest
 * thing to get backwards here, and getting it backwards gives an estimate
 * that is exactly wrong rather than obviously broken — the aircraft would
 * correct a 30-degree bank by rolling to 60.
 *
 * An accelerometer measures SPECIFIC FORCE. Level and upright it reads +1g
 * on +Z, because the table pushes up. Roll phi puts +sin(phi) on Y; pitch
 * theta (nose up) puts -sin(theta) on X.
 */
struct AccelReading { float x, y, z; };

static AccelReading accel_at(float roll_deg, float pitch_deg)
{
    const float r = roll_deg * D2R, p = pitch_deg * D2R;
    AccelReading a;
    a.x = -sinf(p);
    a.y =  sinf(r) * cosf(p);
    a.z =  cosf(r) * cosf(p);
    return a;
}

/* ------------------------------------------------------------- madgwick */

void test_level_converges_to_zero(void)
{
    Madgwick f;
    settle(f, 0.0f, 0.0f, 1.0f, 3.0f);
    EulerAngles e = f.euler();
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 0.0f, e.roll);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 0.0f, e.pitch);
    TEST_ASSERT_TRUE(f.converged());
}

void test_static_roll_is_recovered(void)
{
    Madgwick f;
    AccelReading a = accel_at(30.0f, 0.0f);
    settle(f, a.x, a.y, a.z, 4.0f);
    TEST_ASSERT_FLOAT_WITHIN(2.0f, 30.0f, f.euler().roll);
}

void test_negative_roll_is_recovered(void)
{
    /* Both directions, because a sign error shows up as a mirror image and
     * a single-sided test cannot see it. */
    Madgwick f;
    AccelReading a = accel_at(-30.0f, 0.0f);
    settle(f, a.x, a.y, a.z, 4.0f);
    TEST_ASSERT_FLOAT_WITHIN(2.0f, -30.0f, f.euler().roll);
}

void test_static_pitch_is_recovered(void)
{
    Madgwick f;
    AccelReading a = accel_at(0.0f, 20.0f);
    settle(f, a.x, a.y, a.z, 4.0f);
    TEST_ASSERT_FLOAT_WITHIN(2.0f, 20.0f, f.euler().pitch);
}

void test_combined_roll_and_pitch(void)
{
    Madgwick f;
    AccelReading a = accel_at(15.0f, -25.0f);
    settle(f, a.x, a.y, a.z, 5.0f);
    TEST_ASSERT_FLOAT_WITHIN(2.5f, 15.0f, f.euler().roll);
    TEST_ASSERT_FLOAT_WITHIN(2.5f, -25.0f, f.euler().pitch);
}

void test_align_to_accel_is_instant(void)
{
    /* Boot alignment matters: without it the filter starts at identity and
     * takes seconds to find the horizon, so an aircraft powered up on a
     * sloped surface believes it is level and lurches on take-off. */
    Madgwick f;
    AccelReading a = accel_at(25.0f, 0.0f);
    f.align_to_accel(a.x, a.y, a.z);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 25.0f, f.euler().roll);

    Madgwick g;
    AccelReading b = accel_at(-12.0f, 18.0f);
    g.align_to_accel(b.x, b.y, b.z);
    TEST_ASSERT_FLOAT_WITHIN(1.5f, -12.0f, g.euler().roll);
    TEST_ASSERT_FLOAT_WITHIN(1.5f, 18.0f, g.euler().pitch);
}

void test_gyro_rotation_is_integrated(void)
{
    /* Spin about Z with no useful accel information; yaw should track. */
    Madgwick f;
    settle(f, 0, 0, 1.0f, 2.5f);
    const float start = f.euler().yaw;
    for (int i = 0; i < 1000; i++) f.update(0, 0, 90.0f, 0, 0, 1.0f, DT);
    float moved = f.euler().yaw - start;
    if (moved < -180.0f) moved += 360.0f;
    if (moved > 180.0f) moved -= 360.0f;
    TEST_ASSERT_FLOAT_WITHIN(8.0f, 90.0f, moved);
}

void test_bad_samples_do_not_corrupt_the_estimate(void)
{
    Madgwick f;
    settle(f, 0, 0, 1.0f, 2.5f);
    const float before = f.euler().roll;

    const float nan_v = std::nan("");
    f.update(nan_v, nan_v, nan_v, 0, 0, 1.0f, DT);   /* bad gyro   */
    f.update(0, 0, 0, 0, 0, 0, DT);                   /* zero accel */
    f.update(0, 0, 0, 0, 0, 1.0f, 0.0f);              /* zero dt    */
    f.update(0, 0, 0, 0, 0, 1.0f, 10.0f);             /* absurd dt  */

    EulerAngles e = f.euler();
    TEST_ASSERT_FALSE(std::isnan(e.roll));
    TEST_ASSERT_FALSE(std::isnan(e.pitch));
    TEST_ASSERT_FALSE(std::isnan(e.yaw));
    TEST_ASSERT_FLOAT_WITHIN(2.0f, before, e.roll);
}

void test_no_nan_at_ninety_degrees_pitch(void)
{
    /* asin(1.0000001) is NaN, and accumulated rounding gets there. A NaN
     * attitude would reach the mixer. */
    Madgwick f;
    f.align_to_accel(1.0f, 0.0f, 0.0f);
    for (int i = 0; i < 200; i++) f.update(0, 0, 0, 1.0f, 0.0f, 0.0f, DT);
    EulerAngles e = f.euler();
    TEST_ASSERT_FALSE(std::isnan(e.pitch));
    TEST_ASSERT_TRUE(fabsf(e.pitch) <= 90.5f);
}

/* ------------------------------------------------------------------ pid */

void test_rate_pid_drives_toward_setpoint(void)
{
    RatePid pid;
    pid.configure(default_roll_rate_gains());
    /* Told to rotate right, currently still: output must be positive. */
    const float out = pid.update(100.0f, 0.0f, DT, 1.0f);
    TEST_ASSERT_TRUE(out > 0.0f);
    TEST_ASSERT_TRUE(pid.update(-100.0f, 0.0f, DT, 1.0f) < 0.0f);
}

void test_no_derivative_kick_on_a_setpoint_step(void)
{
    /* The reason D acts on measurement, not error. With D-on-error this
     * output would be orders of magnitude larger for one frame. */
    RatePid pid;
    PidGains g = default_roll_rate_gains();
    g.ki = 0.0f;
    pid.configure(g);

    for (int i = 0; i < 50; i++) pid.update(0.0f, 0.0f, DT, 1.0f);
    const float step = pid.update(200.0f, 0.0f, DT, 1.0f);

    /* Should be essentially the P term alone. */
    TEST_ASSERT_FLOAT_WITHIN(0.02f, g.kp * 200.0f, step);
}

void test_integral_is_bounded(void)
{
    RatePid pid;
    PidGains g = default_roll_rate_gains();
    pid.configure(g);
    for (int i = 0; i < 20000; i++) pid.update(500.0f, 0.0f, DT, 1.0f);
    TEST_ASSERT_TRUE(fabsf(pid.integral()) <= g.i_limit + 1e-6f);
}

void test_saturation_stops_windup(void)
{
    /* A plain clamp is not enough: while the output is pinned the integrator
     * keeps storing authority it cannot use, and spends it as overshoot the
     * moment the aircraft responds. */
    RatePid pid;
    pid.configure(default_roll_rate_gains());

    /* Hard against the limit for a long time. */
    for (int i = 0; i < 5000; i++) pid.update(2000.0f, 0.0f, DT, 0.05f);
    const float wound = fabsf(pid.integral());

    RatePid fresh;
    fresh.configure(default_roll_rate_gains());
    for (int i = 0; i < 5000; i++) fresh.update(20.0f, 0.0f, DT, 1.0f);

    TEST_ASSERT_TRUE_MESSAGE(wound <= fabsf(fresh.integral()) + 1e-6f,
        "integrator wound up further while saturated than while in range");
}

void test_output_respects_the_limit(void)
{
    RatePid pid;
    pid.configure(default_roll_rate_gains());
    for (int i = 0; i < 2000; i++) {
        const float out = pid.update(5000.0f, -5000.0f, DT, 0.3f);
        TEST_ASSERT_TRUE(fabsf(out) <= 0.3f + 1e-6f);
    }
}

void test_pid_survives_bad_input(void)
{
    RatePid pid;
    pid.configure(default_roll_rate_gains());
    const float nan_v = std::nan("");
    TEST_ASSERT_FALSE(std::isnan(pid.update(nan_v, 0.0f, DT, 1.0f)));
    TEST_ASSERT_FALSE(std::isnan(pid.update(0.0f, nan_v, DT, 1.0f)));
    TEST_ASSERT_FALSE(std::isnan(pid.update(0.0f, 0.0f, 0.0f, 1.0f)));
    TEST_ASSERT_FALSE(std::isnan(pid.update(0.0f, 0.0f, -1.0f, 1.0f)));
}

void test_reset_clears_state(void)
{
    RatePid pid;
    pid.configure(default_roll_rate_gains());
    for (int i = 0; i < 1000; i++) pid.update(300.0f, 0.0f, DT, 1.0f);
    TEST_ASSERT_TRUE(fabsf(pid.integral()) > 0.0f);
    pid.reset();
    TEST_ASSERT_EQUAL_FLOAT(0.0f, pid.integral());
    /* And the first sample after reset must not produce a derivative kick. */
    PidGains g = default_roll_rate_gains();
    const float first = pid.update(100.0f, 50.0f, DT, 1.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, g.kp * 50.0f, first);
}

void test_angle_loop_produces_a_rate_setpoint(void)
{
    AnglePid ang;
    ang.configure(6.0f, 180.0f);
    /* 10 deg of error at kp 6 = 60 deg/s of demand. */
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 60.0f, ang.update(10.0f, 0.0f));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, -60.0f, ang.update(-10.0f, 0.0f));
    /* Level and told to be level: nothing. */
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, ang.update(15.0f, 15.0f));
    /* And it cannot demand more than the airframe is allowed. */
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 180.0f, ang.update(90.0f, 0.0f));
}

/* --------------------------------------------------- closed-loop sanity */

void test_cascade_stabilises_a_toy_airframe(void)
{
    /* A crude first-order model: motor demand produces angular acceleration,
     * with some damping. Not a substitute for tuning on the real aircraft,
     * but it does catch a cascade wired up with the wrong sign — which would
     * accelerate away from level instead of toward it.
     */
    Madgwick filt;
    AnglePid ang;
    RatePid rate;
    ang.configure(6.0f, 180.0f);
    rate.configure(default_roll_rate_gains());

    float angle_deg = 20.0f;      /* released from a 20 degree bank */
    float rate_dps = 0.0f;

    for (int i = 0; i < 4000; i++) {          /* 4 seconds at 1kHz */
        AccelReading a = accel_at(angle_deg, 0.0f);
        filt.update(rate_dps, 0.0f, 0.0f, a.x, a.y, a.z, DT);

        const float est = filt.euler().roll;
        const float rate_sp = ang.update(0.0f, est);
        const float demand = rate.update(rate_sp, rate_dps, DT, 1.0f);

        /* demand -> angular acceleration, plus aerodynamic damping */
        const float accel = demand * 9000.0f - rate_dps * 1.5f;
        rate_dps += accel * DT;
        angle_deg += rate_dps * DT;
    }

    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(3.0f, 0.0f, angle_deg,
        "cascade failed to bring the airframe back to level");
    TEST_ASSERT_TRUE(fabsf(rate_dps) < 30.0f);
}

/* -------------------------------------------------------------- yaw sign */

void test_yaw_loop_converges_with_physical_gyro_sign(void)
{
    /* THE test that guards the yaw feedback sign.
     *
     * Physical model, FLU frame (see frame.h): protocol yaw+ is CLOCKWISE
     * from above, and a clockwise yaw is rotation about -Z, so the gyro
     * reads gz = -rate_cw. Close the loop through the real conversion and
     * the aircraft must settle at the commanded rate; feed raw gz instead
     * and this diverges (checked below), which is exactly what the aircraft
     * would have done on its first yaw input.
     */
    RatePid pid;
    pid.configure(default_yaw_rate_gains());

    float rate_cw = 0.0f;                 /* physical clockwise rate */
    for (int i = 0; i < 4000; i++) {
        const float gz = -rate_cw;        /* what the FLU gyro reports */
        const float demand = pid.update(90.0f, pursuit_yaw_rate_cw(gz),
                                        DT, 0.3f);
        /* demand+ speeds the CCW prop pair -> clockwise reaction torque */
        rate_cw += (demand * 2500.0f - rate_cw * 2.0f) * DT;
    }
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(10.0f, 90.0f, rate_cw,
        "yaw loop did not settle at the commanded clockwise rate");

    /* And the uncorrected wiring must NOT converge — if it does, the model
     * above is meaningless and so is the test. */
    RatePid wrong;
    wrong.configure(default_yaw_rate_gains());
    float r2 = 0.0f;
    for (int i = 0; i < 4000; i++) {
        const float gz = -r2;
        const float demand = wrong.update(90.0f, gz, DT, 0.3f);  /* raw gz */
        r2 += (demand * 2500.0f - r2 * 2.0f) * DT;
    }
    TEST_ASSERT_TRUE_MESSAGE(fabsf(r2 - 90.0f) > 45.0f,
        "raw-gz wiring converged — the physical model here is broken");
}

void test_yaw_rate_cw_is_a_pure_negation(void)
{
    TEST_ASSERT_EQUAL_FLOAT(-120.0f, pursuit_yaw_rate_cw(120.0f));
    TEST_ASSERT_EQUAL_FLOAT(35.0f, pursuit_yaw_rate_cw(-35.0f));
}

/* ----------------------------------------------------------------- main */

int main(int, char **)
{
    UNITY_BEGIN();

    RUN_TEST(test_level_converges_to_zero);
    RUN_TEST(test_static_roll_is_recovered);
    RUN_TEST(test_negative_roll_is_recovered);
    RUN_TEST(test_static_pitch_is_recovered);
    RUN_TEST(test_combined_roll_and_pitch);
    RUN_TEST(test_align_to_accel_is_instant);
    RUN_TEST(test_gyro_rotation_is_integrated);
    RUN_TEST(test_bad_samples_do_not_corrupt_the_estimate);
    RUN_TEST(test_no_nan_at_ninety_degrees_pitch);

    RUN_TEST(test_rate_pid_drives_toward_setpoint);
    RUN_TEST(test_no_derivative_kick_on_a_setpoint_step);
    RUN_TEST(test_integral_is_bounded);
    RUN_TEST(test_saturation_stops_windup);
    RUN_TEST(test_output_respects_the_limit);
    RUN_TEST(test_pid_survives_bad_input);
    RUN_TEST(test_reset_clears_state);
    RUN_TEST(test_angle_loop_produces_a_rate_setpoint);

    RUN_TEST(test_cascade_stabilises_a_toy_airframe);

    RUN_TEST(test_yaw_loop_converges_with_physical_gyro_sign);
    RUN_TEST(test_yaw_rate_cw_is_a_pure_negation);

    return UNITY_END();
}
