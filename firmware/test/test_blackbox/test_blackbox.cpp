/* Flight recorder.
 *
 * Two things must hold. The logger must never be able to affect flight — it
 * runs inside the 1kHz loop, so a blocking write or an allocation would be a
 * control-loop stall. And the scaling must round-trip, because a mismatch
 * between the writer and tools/decode_blackbox.py produces a plot that looks
 * entirely plausible and is wrong by a constant factor, which is the hardest
 * kind of wrong to notice while tuning.
 */
#include <unity.h>

#include <cmath>
#include <cstring>

#include "blackbox.h"

/* ------------------------------------------------------------- scaling */

void test_record_is_exactly_forty_bytes(void)
{
    /* The decoder checks this against the file header and refuses to parse a
     * log from a different build rather than misreading every field. */
    TEST_ASSERT_EQUAL_UINT32(40, sizeof(BlackboxRecord));
    TEST_ASSERT_EQUAL_UINT32(20, sizeof(BlackboxHeader));
}

void test_gyro_scaling_round_trips(void)
{
    const float vals[] = {0.0f, 1.0f, -1.0f, 250.0f, -250.0f, 1999.0f,
                          -1999.0f};
    for (float v : vals) {
        const float back = bb_scale_gyro(v) * (2000.0f / 32767.0f);
        TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.1f, v, back, "gyro scale");
    }
}

void test_angle_scaling_round_trips(void)
{
    const float vals[] = {0.0f, 0.5f, -0.5f, 25.0f, -25.0f, 179.0f, -179.0f};
    for (float v : vals) {
        const float back = bb_scale_angle(v) * (180.0f / 32767.0f);
        TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.02f, v, back, "angle scale");
    }
}

void test_unit_and_duty_scaling(void)
{
    TEST_ASSERT_EQUAL_INT16(0, bb_scale_unit(0.0f));
    TEST_ASSERT_EQUAL_INT16(32767, bb_scale_unit(1.0f));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, -0.5f,
                             bb_scale_unit(-0.5f) / 32767.0f);
    TEST_ASSERT_EQUAL_UINT8(0, bb_scale_duty(0.0f));
    TEST_ASSERT_EQUAL_UINT8(255, bb_scale_duty(1.0f));
    TEST_ASSERT_FLOAT_WITHIN(0.005f, 0.5f, bb_scale_duty(0.5f) / 255.0f);
}

void test_volt_scaling_covers_the_pack_range(void)
{
    for (float v = 3.0f; v <= 4.3f; v += 0.05f) {
        const float back = bb_scale_volts(v) / 50.0f;
        TEST_ASSERT_FLOAT_WITHIN(0.02f, v, back);
    }
}

void test_scaling_clamps_instead_of_wrapping(void)
{
    /* An out-of-range value must saturate, not wrap: a wrapped int16 turns a
     * hard-right spin into a hard-LEFT one on the plot, and you would tune
     * the wrong sign. */
    TEST_ASSERT_EQUAL_INT16(32767, bb_scale_gyro(99999.0f));
    TEST_ASSERT_TRUE(bb_scale_gyro(-99999.0f) <= -32767);
    TEST_ASSERT_EQUAL_INT16(32767, bb_scale_angle(9999.0f));
    TEST_ASSERT_EQUAL_UINT8(255, bb_scale_duty(50.0f));
    TEST_ASSERT_EQUAL_UINT8(255, bb_scale_volts(99.0f));
}

void test_nan_logs_as_zero_not_garbage(void)
{
    const float n = std::nan("");
    TEST_ASSERT_EQUAL_INT16(0, bb_scale_gyro(n));
    TEST_ASSERT_EQUAL_INT16(0, bb_scale_angle(n));
    TEST_ASSERT_EQUAL_INT16(0, bb_scale_unit(n));
    TEST_ASSERT_EQUAL_UINT8(0, bb_scale_duty(n));
    TEST_ASSERT_EQUAL_UINT8(0, bb_scale_volts(n));
}

/* --------------------------------------------------------------- buffer */

static BlackboxRecord mk(uint32_t t)
{
    BlackboxRecord r;
    std::memset(&r, 0, sizeof(r));
    r.t_us = t;
    return r;
}

void test_push_does_nothing_until_started(void)
{
    /* Otherwise a log is an afternoon of bench idling with a flight buried
     * somewhere in it. */
    TEST_ASSERT_TRUE(blackbox_begin(500, 1));
    blackbox_stop();
    for (int i = 0; i < 100; i++) blackbox_push(mk(i));
    TEST_ASSERT_EQUAL_UINT32(0, blackbox_count());
}

void test_records_come_back_in_order(void)
{
    TEST_ASSERT_TRUE(blackbox_begin(500, 1));
    blackbox_start();
    for (uint32_t i = 0; i < 50; i++) blackbox_push(mk(i * 2000));
    blackbox_stop();

    TEST_ASSERT_EQUAL_UINT32(50, blackbox_count());
    BlackboxRecord r;
    for (uint32_t i = 0; i < 50; i++) {
        TEST_ASSERT_TRUE(blackbox_peek(i, &r));
        TEST_ASSERT_EQUAL_UINT32(i * 2000, r.t_us);
    }
}

void test_a_full_buffer_keeps_the_most_recent_flight(void)
{
    /* Overflowing must drop the OLDEST samples. When a flight outlasts the
     * buffer, the interesting part is the crash at the end, not the taxi at
     * the start. */
    TEST_ASSERT_TRUE(blackbox_begin(10, 1));      /* capacity 10 */
    TEST_ASSERT_EQUAL_UINT32(10, blackbox_capacity());
    blackbox_start();
    for (uint32_t i = 0; i < 25; i++) blackbox_push(mk(i));

    TEST_ASSERT_EQUAL_UINT32(10, blackbox_count());
    TEST_ASSERT_EQUAL_UINT32(15, blackbox_dropped());

    BlackboxRecord r;
    TEST_ASSERT_TRUE(blackbox_peek(0, &r));
    TEST_ASSERT_EQUAL_UINT32_MESSAGE(15, r.t_us,
        "oldest surviving record is wrong — the ring dropped the wrong end");
    TEST_ASSERT_TRUE(blackbox_peek(9, &r));
    TEST_ASSERT_EQUAL_UINT32(24, r.t_us);
}

void test_starting_clears_the_previous_flight(void)
{
    TEST_ASSERT_TRUE(blackbox_begin(500, 1));
    blackbox_start();
    for (uint32_t i = 0; i < 30; i++) blackbox_push(mk(i));
    blackbox_stop();

    blackbox_start();
    TEST_ASSERT_EQUAL_UINT32(0, blackbox_count());
    TEST_ASSERT_EQUAL_UINT32(0, blackbox_dropped());
}

void test_overflow_never_reports_more_than_capacity(void)
{
    TEST_ASSERT_TRUE(blackbox_begin(100, 1));
    blackbox_start();
    for (uint32_t i = 0; i < 10000; i++) blackbox_push(mk(i));
    TEST_ASSERT_TRUE(blackbox_count() <= blackbox_capacity());
    BlackboxRecord r;
    TEST_ASSERT_TRUE(blackbox_peek(blackbox_count() - 1, &r));
    TEST_ASSERT_EQUAL_UINT32(9999, r.t_us);
}

int main(int, char **)
{
    UNITY_BEGIN();
    RUN_TEST(test_record_is_exactly_forty_bytes);
    RUN_TEST(test_gyro_scaling_round_trips);
    RUN_TEST(test_angle_scaling_round_trips);
    RUN_TEST(test_unit_and_duty_scaling);
    RUN_TEST(test_volt_scaling_covers_the_pack_range);
    RUN_TEST(test_scaling_clamps_instead_of_wrapping);
    RUN_TEST(test_nan_logs_as_zero_not_garbage);

    RUN_TEST(test_push_does_nothing_until_started);
    RUN_TEST(test_records_come_back_in_order);
    RUN_TEST(test_a_full_buffer_keeps_the_most_recent_flight);
    RUN_TEST(test_starting_clears_the_previous_flight);
    RUN_TEST(test_overflow_never_reports_more_than_capacity);
    return UNITY_END();
}
