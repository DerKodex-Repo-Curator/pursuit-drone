/* Loop-timing statistics.
 *
 * config.h declares LOOP_HZ 1000. Until this file existed that was an
 * intention, not a measurement: nothing sampled the real period and nothing
 * would have noticed if it were missed.
 *
 * The obvious instrument — count iterations, divide by elapsed seconds — is
 * the one that hides the failure you care about. A loop that runs at 2 kHz for
 * 900 ms and then stalls for 100 ms reports a perfectly healthy 1 kHz mean.
 * That stall is 100 ms of an aircraft integrating gyro rates against a frozen
 * estimate, which is the difference between a control loop and a suggestion.
 *
 * So this keeps the distribution instead: a histogram, the worst case ever
 * seen, and a count of periods that exceeded the declared budget. The mean is
 * still available, but it is the least interesting number here.
 *
 * No Arduino headers, so it compiles and is tested on the host.
 */
#ifndef PURSUIT_LOOPSTATS_H
#define PURSUIT_LOOPSTATS_H

#include <stdint.h>

/* Bucket upper edges in microseconds. Chosen around the 1000 us budget rather
 * than as a uniform sweep — resolution matters most either side of the line
 * that defines a miss, and everything past 10 ms is equally bad. */
#define LOOPSTATS_BUCKETS 10

class LoopStats {
public:
    explicit LoopStats(uint32_t budget_us = 1000);

    void reset(void);

    /* One measured loop period. Cheap by construction: a compare chain and two
     * increments, no division, no floating point. This runs at loop rate. */
    void sample(uint32_t period_us);

    uint32_t count(void)     const { return count_; }
    uint32_t overruns(void)  const { return overruns_; }
    uint32_t max_us(void)    const { return max_us_; }
    uint32_t min_us(void)    const { return count_ ? min_us_ : 0; }
    uint32_t budget_us(void) const { return budget_us_; }

    uint32_t mean_us(void) const;

    /* Upper edge of the bucket containing the requested percentile, so the
     * answer is always conservative: a reported p99 of 2000 us means 99% of
     * periods were at or below 2000, never "about 2000". `frac` is 0..1.
     *
     * The top bucket is unbounded, so a percentile that lands there reports
     * max_us() instead of a fictional edge. */
    uint32_t percentile_us(float frac) const;

    /* Fraction of periods that exceeded the budget, in parts per million —
     * integer, because a jitter counter that allocates a float on an
     * ESP32 loop is its own small irony. */
    uint32_t overrun_ppm(void) const;

    uint32_t bucket(int i) const;
    static uint32_t bucket_edge_us(int i);   /* UINT32_MAX for the last */

private:
    uint32_t budget_us_;
    uint32_t count_;
    uint32_t overruns_;
    uint32_t max_us_;
    uint32_t min_us_;
    uint64_t sum_us_;
    uint32_t bucket_[LOOPSTATS_BUCKETS];
};

/* The flight loop's own instance, defined in main.cpp. Declared here so the
 * bench console can report it. Only referenced from target builds — main.cpp
 * is excluded from the host build, where this stays an unused declaration. */
extern LoopStats g_loop_stats;

#endif /* PURSUIT_LOOPSTATS_H */
