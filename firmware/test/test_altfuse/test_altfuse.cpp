/* Rangefinder + barometer fusion.
 *
 * The load-bearing test here is test_no_step_at_the_handover. Everything else
 * is bookkeeping; that one is the reason this module exists as its own tested
 * file rather than a few lines inside alt_hold.
 */
#include <unity.h>

#include <cmath>

#include "altitude_fuse.h"

static const float DT = 0.02f;   /* 50 Hz, roughly the barometer's rate */

/* Feed both sensors for a while at a fixed true height. `baro_datum` is how
 * far the barometer's zero sits from the ground — the whole problem. */
static void settle(AltitudeFuse &f, float true_h, float baro_datum,
                   int n = 400, bool tof_valid = true)
{
    for (int i = 0; i < n; i++) {
        f.push_baro(baro_datum + true_h, DT);
        f.push_tof(true_h, tof_valid, DT);
    }
}

void test_nothing_is_offered_before_any_data(void)
{
    AltitudeFuse f;
    TEST_ASSERT_FALSE(f.valid());
    TEST_ASSERT_EQUAL(AltSource::None, f.source());
}

void test_low_down_it_is_the_rangefinder(void)
{
    AltitudeFuse f;
    f.arm(100.0f);                 /* barometer thinks it is at 100 m */
    settle(f, 1.20f, 100.0f);
    TEST_ASSERT_EQUAL(AltSource::Tof, f.source());
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 1.20f, f.height());
}

void test_the_baro_datum_is_learned_not_assumed(void)
{
    /* Barometer reads 100 m at ground level. The fused height must still be
     * the rangefinder's 1.2 m, and the learned offset must be ~0 once armed
     * at that datum. */
    AltitudeFuse f;
    f.arm(100.0f);
    settle(f, 1.20f, 100.0f);
    TEST_ASSERT_TRUE(f.offset_learned());
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 0.0f, f.offset());
}

void test_no_step_at_the_handover(void)
{
    /* THE test. Climb through the 3.0-3.5 m band one centimetre at a time and
     * assert the fused height never jumps.
     *
     * Without the learned offset this is where altitude leaps by whatever the
     * two datums differ by — mid-climb, under power, into a loop that will
     * dutifully correct for it. */
    AltitudeFuse f;
    f.arm(250.0f);                       /* an awkward datum on purpose */
    settle(f, 0.80f, 250.0f);            /* learn the offset down low   */

    float prev = f.height();
    for (int i = 0; i <= 400; i++) {
        const float h = 0.80f + i * 0.01f;      /* 0.8 m -> 4.8 m */
        f.push_baro(250.0f + h, DT);
        /* The rangefinder stops reporting past its range, exactly as the
         * driver behaves. */
        f.push_tof(h, h <= 3.5f, DT);

        TEST_ASSERT_TRUE(f.valid());
        const float step = std::fabs(f.height() - prev);
        TEST_ASSERT_TRUE_MESSAGE(step < 0.08f,
                                 "fused height stepped at the handover");
        prev = f.height();
    }
    /* And it ended up on the barometer, tracking the real height. */
    TEST_ASSERT_EQUAL(AltSource::Baro, f.source());
    TEST_ASSERT_FLOAT_WITHIN(0.30f, 4.80f, f.height());
}

void test_it_blends_rather_than_switching(void)
{
    AltitudeFuse f;
    f.arm(0.0f);
    settle(f, 1.0f, 0.0f);
    f.push_baro(3.2f, DT);
    f.push_tof(3.2f, true, DT);
    TEST_ASSERT_EQUAL(AltSource::Blend, f.source());
}

void test_above_range_it_is_the_barometer(void)
{
    AltitudeFuse f;
    f.arm(0.0f);
    settle(f, 1.0f, 0.0f);
    for (int i = 0; i < 100; i++) {
        f.push_baro(6.0f, DT);
        f.push_tof(0.0f, false, DT);      /* out of range */
    }
    TEST_ASSERT_EQUAL(AltSource::Baro, f.source());
    TEST_ASSERT_FLOAT_WITHIN(0.20f, 6.0f, f.height());
}

void test_offset_is_only_learned_where_the_tof_is_trustworthy(void)
{
    /* Feeding it a bad reading at 3.4 m — inside the band where the VL53L1X
     * reads long — must not corrupt the offset the barometer depends on. */
    AltitudeFuse f;
    f.arm(0.0f);
    settle(f, 1.0f, 0.0f);
    const float good = f.offset();
    for (int i = 0; i < 200; i++) {
        f.push_baro(3.4f, DT);
        f.push_tof(3.4f + 1.5f, true, DT);   /* a wildly long reading */
    }
    TEST_ASSERT_FLOAT_WITHIN(0.05f, good, f.offset());
}

void test_a_dead_barometer_falls_back_to_the_rangefinder(void)
{
    /* No barometer fitted at all: the aircraft must still fly on the ToF,
     * exactly as it does today. */
    AltitudeFuse f;
    for (int i = 0; i < 100; i++) f.push_tof(1.10f, true, DT);
    TEST_ASSERT_EQUAL(AltSource::Tof, f.source());
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 1.10f, f.height());
}

void test_stale_readings_stop_being_offered(void)
{
    AltitudeFuse f;
    f.arm(0.0f);
    settle(f, 1.0f, 0.0f);
    TEST_ASSERT_TRUE(f.valid());
    for (int i = 0; i < 100; i++) f.push_tof(0.0f, false, DT);  /* 2 s */
    TEST_ASSERT_EQUAL(AltSource::None, f.source());
}

void test_non_finite_input_is_refused(void)
{
    AltitudeFuse f;
    f.arm(0.0f);
    settle(f, 1.0f, 0.0f);
    const float before = f.height();
    f.push_baro(NAN, DT);
    f.push_tof(INFINITY, true, DT);
    TEST_ASSERT_TRUE(std::isfinite(f.height()));
    TEST_ASSERT_FLOAT_WITHIN(0.05f, before, f.height());
}

void test_arming_rezeroes_so_height_starts_near_zero(void)
{
    AltitudeFuse f;
    f.arm(812.5f);                     /* somewhere up a hill */
    for (int i = 0; i < 100; i++) f.push_baro(812.5f, DT);
    f.push_tof(0.0f, false, DT);
    TEST_ASSERT_FLOAT_WITHIN(0.10f, 0.0f, f.height());
}

int main(int, char **)
{
    UNITY_BEGIN();
    RUN_TEST(test_nothing_is_offered_before_any_data);
    RUN_TEST(test_low_down_it_is_the_rangefinder);
    RUN_TEST(test_the_baro_datum_is_learned_not_assumed);
    RUN_TEST(test_no_step_at_the_handover);
    RUN_TEST(test_it_blends_rather_than_switching);
    RUN_TEST(test_above_range_it_is_the_barometer);
    RUN_TEST(test_offset_is_only_learned_where_the_tof_is_trustworthy);
    RUN_TEST(test_a_dead_barometer_falls_back_to_the_rangefinder);
    RUN_TEST(test_stale_readings_stop_being_offered);
    RUN_TEST(test_non_finite_input_is_refused);
    RUN_TEST(test_arming_rezeroes_so_height_starts_near_zero);
    return UNITY_END();
}
