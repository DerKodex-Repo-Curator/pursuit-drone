/* Battery filter and ADC glitch rejection.
 *
 * The glitch tests exist because of a real two-day misdiagnosis. On
 * 2026-08-19 the aircraft reported 1.47 V over telemetry during its first
 * armed run and tripped emergency_land(). That was taken as evidence of a
 * collapsing pack, and an impedance argument was built on top of it.
 *
 * It was the ADC. The sense divider is 100k/100k -- a 50 kOhm source into a
 * sample-and-hold that wants under 10 kOhm -- and with motor PWM running it
 * returned 1.70 V at 0% duty and 0.00 V mid-run, on a pack a multimeter put
 * at 3.64 V. update() rejected non-finite values, but 0.0f is finite, so
 * every glitch went straight into the filters.
 *
 * These tests pin the rejection so that never silently returns.
 */
#include <unity.h>

#include "battery.h"
#include "config.h"

static void feed(Battery &b, float v, int n, float dt = 0.02f)
{
    for (int i = 0; i < n; i++) b.update(v, dt);
}

void setUp(void) {}
void tearDown(void) {}

static void test_a_zero_volt_sample_is_rejected_not_filtered(void)
{
    Battery b;
    b.reset(3.90f);
    feed(b, 0.0f, 50);
    /* 0.0f is finite, so nothing but the sanity floor stops it. One second of
     * these used to drag the fast filter to the floor and latch critical. */
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 3.90f, b.instant_volts());
    TEST_ASSERT_FALSE(b.critical());
    TEST_ASSERT_EQUAL_UINT32(50, b.glitches());
}

static void test_the_impossible_reading_that_started_it_all(void)
{
    Battery b;
    b.reset(3.64f);
    feed(b, 1.70f, 100);   /* what the ADC actually reported at 0% duty */
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 3.64f, b.instant_volts());
    TEST_ASSERT_EQUAL_UINT32(100, b.glitches());
}

static void test_a_genuinely_collapsing_pack_is_still_believed(void)
{
    /* The floor must reject the impossible without hiding the alarming.
     * 2.80 V is above VBAT_SANE_MIN_V and must pass straight through. */
    Battery b;
    b.reset(3.90f);
    feed(b, 2.80f, 400, 0.02f);
    TEST_ASSERT_TRUE(b.instant_volts() < 3.0f);
    TEST_ASSERT_TRUE(b.critical());
    TEST_ASSERT_EQUAL_UINT32(0, b.glitches());
}

static void test_glitches_do_not_disturb_a_good_signal(void)
{
    /* Alternating good and impossible samples, which is the observed
     * pattern. The good ones must land and the bad ones must vanish. */
    Battery b;
    b.reset(3.80f);
    for (int i = 0; i < 200; i++) {
        b.update(3.70f, 0.02f);
        b.update(0.0f, 0.02f);
    }
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 3.70f, b.instant_volts());
    TEST_ASSERT_EQUAL_UINT32(200, b.glitches());
}

static void test_a_nan_is_counted_as_a_glitch_too(void)
{
    Battery b;
    b.reset(3.90f);
    b.update(0.0f / 0.0f, 0.02f);
    TEST_ASSERT_EQUAL_UINT32(1, b.glitches());
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 3.90f, b.instant_volts());
}

static void test_the_floor_is_below_the_critical_threshold(void)
{
    /* If these ever cross, a pack heading for a genuine brownout would be
     * discarded as noise on its way down. */
    TEST_ASSERT_TRUE(VBAT_SANE_MIN_V < VBAT_CRITICAL_V);
}

int main(int, char **)
{
    UNITY_BEGIN();
    RUN_TEST(test_a_zero_volt_sample_is_rejected_not_filtered);
    RUN_TEST(test_the_impossible_reading_that_started_it_all);
    RUN_TEST(test_a_genuinely_collapsing_pack_is_still_believed);
    RUN_TEST(test_glitches_do_not_disturb_a_good_signal);
    RUN_TEST(test_a_nan_is_counted_as_a_glitch_too);
    RUN_TEST(test_the_floor_is_below_the_critical_threshold);
    return UNITY_END();
}
