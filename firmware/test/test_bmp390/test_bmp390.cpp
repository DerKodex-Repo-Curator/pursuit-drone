/* BMP390 compensation arithmetic.
 *
 * The I2C is not testable on a laptop; the maths is, and the maths is where
 * this goes wrong. Each trim coefficient has its OWN power-of-two scaling, and
 * using one coefficient's exponent for its neighbour produces a pressure that
 * looks entirely plausible and is wrong by a constant factor — which becomes a
 * constant factor in altitude, which becomes an aircraft that thinks it is
 * somewhere it is not.
 */
#include <unity.h>

#include <cmath>

#include "bmp390.h"

/* A representative trim set, already in the float form read_calibration()
 * produces. Values are typical of a real part rather than round numbers, so a
 * scaling slip does not accidentally cancel. */
static Bmp390Calib nominal_calib(void)
{
    Bmp390Calib c{};
    c.t1 = 27427840.0f;
    c.t2 = 1.8626e-05f;
    c.t3 = -1.4211e-14f;
    c.p1 = -0.00043869f;
    c.p2 = -4.2841e-05f;
    c.p3 = 1.3970e-09f;
    c.p4 = 0.0f;
    c.p5 = 26214400.0f;
    c.p6 = 405.0f;
    c.p7 = -0.02734f;
    c.p8 = -0.00021362f;
    c.p9 = 1.1013e-11f;
    c.p10 = 3.5527e-14f;
    c.p11 = -8.1315e-20f;
    return c;
}

void test_altitude_is_zero_at_the_reference(void)
{
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f,
                             bmp390_altitude_m(101325.0f, 101325.0f));
}

void test_altitude_increases_as_pressure_falls(void)
{
    const float low = bmp390_altitude_m(101325.0f);
    const float high = bmp390_altitude_m(100000.0f);
    TEST_ASSERT_TRUE(high > low);
}

void test_altitude_matches_the_standard_atmosphere(void)
{
    /* Textbook checkpoints. 500 m is about 95461 Pa, 1000 m about 89876 Pa
     * in the ISA. Loose tolerance — the formula is an approximation — but
     * tight enough to catch an exponent typo, which is the realistic error. */
    TEST_ASSERT_FLOAT_WITHIN(15.0f, 500.0f, bmp390_altitude_m(95461.0f));
    TEST_ASSERT_FLOAT_WITHIN(25.0f, 1000.0f, bmp390_altitude_m(89876.0f));
}

void test_thirty_feet_is_about_nine_metres_of_pressure_change(void)
{
    /* The whole point of fitting this part. Near sea level roughly 12 Pa per
     * metre, so 30 ft should be a bit over 100 Pa — well inside the BMP390's
     * resolution, which is why it can hold that altitude at all. */
    const float ground = bmp390_altitude_m(101325.0f);
    const float up = bmp390_altitude_m(101325.0f - 108.0f);
    TEST_ASSERT_FLOAT_WITHIN(1.5f, 9.0f, up - ground);
}

void test_nonsense_pressure_does_not_produce_nonsense_altitude(void)
{
    TEST_ASSERT_EQUAL_FLOAT(0.0f, bmp390_altitude_m(0.0f));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, bmp390_altitude_m(-5.0f));
    TEST_ASSERT_TRUE(std::isfinite(bmp390_altitude_m(101325.0f, 0.0f)));
}

void test_temperature_compensation_is_monotonic_in_the_raw_word(void)
{
    const Bmp390Calib c = nominal_calib();
    float prev = -1e9f;
    for (uint32_t r = 7000000; r < 9000000; r += 100000) {
        Bmp390Raw raw{0, r};
        const float t = bmp390_compensate_temperature(raw, c);
        TEST_ASSERT_TRUE(std::isfinite(t));
        TEST_ASSERT_TRUE(t > prev);
        prev = t;
    }
}

void test_pressure_compensation_stays_finite_across_the_range(void)
{
    const Bmp390Calib c = nominal_calib();
    for (uint32_t p = 4000000; p < 9000000; p += 250000) {
        Bmp390Raw raw{p, 8000000};
        const float t = bmp390_compensate_temperature(raw, c);
        const float pa = bmp390_compensate_pressure(raw, c, t);
        TEST_ASSERT_TRUE(std::isfinite(pa));
    }
}

void test_pressure_responds_to_the_raw_word(void)
{
    /* A compensation that ignores its input — easy to write by dropping a
     * term — would produce a constant, and a constant pressure reads as an
     * aircraft that never changes height. */
    const Bmp390Calib c = nominal_calib();
    const float t = bmp390_compensate_temperature(Bmp390Raw{0, 8000000}, c);
    const float a = bmp390_compensate_pressure(Bmp390Raw{5000000, 8000000}, c, t);
    const float b = bmp390_compensate_pressure(Bmp390Raw{6000000, 8000000}, c, t);
    TEST_ASSERT_TRUE(std::fabs(b - a) > 1.0f);
}

void test_a_missing_part_is_not_fatal(void)
{
    /* On the host there is no I2C, which is the same code path as a board with
     * no barometer fitted. It must report absent and hand back an invalid
     * sample rather than pretending. */
    Bmp390 baro;
    TEST_ASSERT_FALSE(baro.begin());
    TEST_ASSERT_FALSE(baro.present());
    TEST_ASSERT_FALSE(baro.read().valid);
}

int main(int, char **)
{
    UNITY_BEGIN();
    RUN_TEST(test_altitude_is_zero_at_the_reference);
    RUN_TEST(test_altitude_increases_as_pressure_falls);
    RUN_TEST(test_altitude_matches_the_standard_atmosphere);
    RUN_TEST(test_thirty_feet_is_about_nine_metres_of_pressure_change);
    RUN_TEST(test_nonsense_pressure_does_not_produce_nonsense_altitude);
    RUN_TEST(test_temperature_compensation_is_monotonic_in_the_raw_word);
    RUN_TEST(test_pressure_compensation_stays_finite_across_the_range);
    RUN_TEST(test_pressure_responds_to_the_raw_word);
    RUN_TEST(test_a_missing_part_is_not_fatal);
    return UNITY_END();
}
