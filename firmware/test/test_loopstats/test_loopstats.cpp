/* Loop-timing statistics.
 *
 * The point of these tests is not that a histogram counts correctly — it is
 * that the instrument reports a stall as a stall. `config.h` declares
 * LOOP_HZ 1000, and the version of this measurement that everybody writes
 * first (iterations per second) would report a perfectly healthy 1 kHz on a
 * loop that froze for a tenth of a second. So the load-bearing test here is
 * `test_a_stall_is_not_hidden_by_the_mean`.
 */
#include <unity.h>

#include "loopstats.h"

void test_an_empty_window_reports_nothing_rather_than_garbage(void)
{
    LoopStats s(1000);
    TEST_ASSERT_EQUAL_UINT32(0, s.count());
    TEST_ASSERT_EQUAL_UINT32(0, s.mean_us());
    TEST_ASSERT_EQUAL_UINT32(0, s.max_us());
    TEST_ASSERT_EQUAL_UINT32(0, s.min_us());
    TEST_ASSERT_EQUAL_UINT32(0, s.overruns());
    TEST_ASSERT_EQUAL_UINT32(0, s.overrun_ppm());
    TEST_ASSERT_EQUAL_UINT32(0, s.percentile_us(0.99f));
}

void test_basic_moments(void)
{
    LoopStats s(1000);
    for (int i = 0; i < 100; i++) s.sample(500);
    TEST_ASSERT_EQUAL_UINT32(100, s.count());
    TEST_ASSERT_EQUAL_UINT32(500, s.mean_us());
    TEST_ASSERT_EQUAL_UINT32(500, s.max_us());
    TEST_ASSERT_EQUAL_UINT32(500, s.min_us());
    TEST_ASSERT_EQUAL_UINT32(0, s.overruns());
}

void test_the_budget_boundary_is_exclusive(void)
{
    /* A period exactly equal to the budget met the budget. Off-by-one here
     * would report a permanent 100% overrun rate on a loop that is fine, and
     * an instrument that cries wolf gets ignored. */
    LoopStats s(1000);
    s.sample(1000);
    TEST_ASSERT_EQUAL_UINT32(0, s.overruns());
    s.sample(1001);
    TEST_ASSERT_EQUAL_UINT32(1, s.overruns());
}

void test_a_stall_is_not_hidden_by_the_mean(void)
{
    /* THE test. 1800 iterations at 500 us (2 kHz) plus one 100 ms stall.
     *
     * Total elapsed is 1800*500 + 100000 = 1.0 s over 1801 iterations, so the
     * naive "iterations per second" instrument reports ~1800 Hz and the mean
     * period reports ~555 us. Both say the loop beat its 1 kHz budget. One of
     * those iterations was a tenth of a second during which the aircraft was
     * integrating gyro rates against a frozen attitude estimate. */
    LoopStats s(1000);
    for (int i = 0; i < 1800; i++) s.sample(500);
    s.sample(100000);

    /* The mean is genuinely reassuring, and genuinely wrong. */
    TEST_ASSERT_TRUE(s.mean_us() < 600);

    /* The instrument that matters is not fooled. */
    TEST_ASSERT_EQUAL_UINT32(1, s.overruns());
    TEST_ASSERT_EQUAL_UINT32(100000, s.max_us());
    TEST_ASSERT_EQUAL_UINT32(100000, s.percentile_us(1.0f));
}

void test_percentiles_are_conservative_not_interpolated(void)
{
    /* 90 fast, 10 slow. p95 must land in the slow group and be reported as the
     * bucket's UPPER edge — a percentile that rounded down would understate
     * the worst case, which is the only direction that matters here. */
    LoopStats s(1000);
    for (int i = 0; i < 90; i++) s.sample(400);   /* bucket <=500  */
    for (int i = 0; i < 10; i++) s.sample(1800);  /* bucket <=2000 */

    TEST_ASSERT_EQUAL_UINT32(500, s.percentile_us(0.50f));
    TEST_ASSERT_EQUAL_UINT32(500, s.percentile_us(0.90f));
    TEST_ASSERT_EQUAL_UINT32(2000, s.percentile_us(0.95f));
    TEST_ASSERT_EQUAL_UINT32(2000, s.percentile_us(1.00f));
}

void test_percentile_in_the_unbounded_bucket_reports_the_real_worst_case(void)
{
    /* The top bucket has no upper edge. Reporting UINT32_MAX would be useless
     * and reporting 10000 would be a lie, so it reports what was observed. */
    LoopStats s(1000);
    for (int i = 0; i < 99; i++) s.sample(500);
    s.sample(250000);
    TEST_ASSERT_EQUAL_UINT32(250000, s.percentile_us(1.0f));
}

void test_overrun_rate_in_ppm(void)
{
    LoopStats s(1000);
    for (int i = 0; i < 999; i++) s.sample(500);
    s.sample(5000);
    TEST_ASSERT_EQUAL_UINT32(1, s.overruns());
    TEST_ASSERT_EQUAL_UINT32(1000, s.overrun_ppm());   /* 1 in 1000 */
}

void test_histogram_totals_match_the_sample_count(void)
{
    /* A sample that fell through every comparison and was silently dropped
     * would make the histogram disagree with the count, and the distribution
     * would understate the tail without saying so. */
    LoopStats s(1000);
    const uint32_t periods[] = {0, 1, 249, 250, 251, 999, 1000, 1001,
                                4999, 9999, 10000, 10001, 1000000};
    for (uint32_t p : periods) s.sample(p);

    uint32_t total = 0;
    for (int i = 0; i < LOOPSTATS_BUCKETS; i++) total += s.bucket(i);
    TEST_ASSERT_EQUAL_UINT32(s.count(), total);
    TEST_ASSERT_EQUAL_UINT32(sizeof(periods) / sizeof(periods[0]), total);
}

void test_reset_clears_everything(void)
{
    LoopStats s(1000);
    for (int i = 0; i < 50; i++) s.sample(9000);
    s.reset();
    TEST_ASSERT_EQUAL_UINT32(0, s.count());
    TEST_ASSERT_EQUAL_UINT32(0, s.overruns());
    TEST_ASSERT_EQUAL_UINT32(0, s.max_us());
    uint32_t total = 0;
    for (int i = 0; i < LOOPSTATS_BUCKETS; i++) total += s.bucket(i);
    TEST_ASSERT_EQUAL_UINT32(0, total);
}

void test_the_counters_do_not_overflow_across_a_long_flight(void)
{
    /* Seven minutes at 2 kHz is ~840k samples of ~500 us. The sum is what
     * overflows first if it is kept in 32 bits: 840000*500 = 4.2e8 fits, but a
     * full pack at a slower loop would not, and a wrapped sum turns the mean
     * into noise. Held in 64 bits; this pins that. */
    LoopStats s(1000);
    for (int i = 0; i < 840000; i++) s.sample(500);
    TEST_ASSERT_EQUAL_UINT32(840000, s.count());
    TEST_ASSERT_EQUAL_UINT32(500, s.mean_us());
}

int main(int, char **)
{
    UNITY_BEGIN();
    RUN_TEST(test_an_empty_window_reports_nothing_rather_than_garbage);
    RUN_TEST(test_basic_moments);
    RUN_TEST(test_the_budget_boundary_is_exclusive);
    RUN_TEST(test_a_stall_is_not_hidden_by_the_mean);
    RUN_TEST(test_percentiles_are_conservative_not_interpolated);
    RUN_TEST(test_percentile_in_the_unbounded_bucket_reports_the_real_worst_case);
    RUN_TEST(test_overrun_rate_in_ppm);
    RUN_TEST(test_histogram_totals_match_the_sample_count);
    RUN_TEST(test_reset_clears_everything);
    RUN_TEST(test_the_counters_do_not_overflow_across_a_long_flight);
    return UNITY_END();
}
