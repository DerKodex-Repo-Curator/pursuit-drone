#include "loopstats.h"

namespace {

/* Upper edges, microseconds. The last bucket is unbounded. */
const uint32_t EDGE[LOOPSTATS_BUCKETS] = {
    250, 500, 750, 1000, 1500, 2000, 3000, 5000, 10000, UINT32_MAX,
};

}  /* namespace */

LoopStats::LoopStats(uint32_t budget_us)
    : budget_us_(budget_us ? budget_us : 1)
{
    reset();
}

void LoopStats::reset(void)
{
    count_ = 0;
    overruns_ = 0;
    max_us_ = 0;
    min_us_ = UINT32_MAX;
    sum_us_ = 0;
    for (int i = 0; i < LOOPSTATS_BUCKETS; i++) bucket_[i] = 0;
}

void LoopStats::sample(uint32_t period_us)
{
    count_++;
    sum_us_ += period_us;
    if (period_us > max_us_) max_us_ = period_us;
    if (period_us < min_us_) min_us_ = period_us;
    if (period_us > budget_us_) overruns_++;

    for (int i = 0; i < LOOPSTATS_BUCKETS; i++) {
        if (period_us <= EDGE[i]) { bucket_[i]++; return; }
    }
    bucket_[LOOPSTATS_BUCKETS - 1]++;   /* unreachable: last edge is UINT32_MAX */
}

uint32_t LoopStats::mean_us(void) const
{
    if (count_ == 0) return 0;
    return (uint32_t)(sum_us_ / count_);
}

uint32_t LoopStats::percentile_us(float frac) const
{
    if (count_ == 0) return 0;
    if (frac <= 0.0f) return min_us_;
    if (frac > 1.0f) frac = 1.0f;

    /* Ceiling, so p50 of an even count lands on the upper of the two middle
     * samples rather than silently rounding a miss away. */
    uint32_t target = (uint32_t)(frac * (float)count_);
    if ((float)target < frac * (float)count_) target++;
    if (target == 0) target = 1;

    uint32_t seen = 0;
    for (int i = 0; i < LOOPSTATS_BUCKETS; i++) {
        seen += bucket_[i];
        if (seen >= target) {
            /* The top bucket has no meaningful edge; the worst case actually
             * observed is the only honest number available. */
            return (EDGE[i] == UINT32_MAX) ? max_us_ : EDGE[i];
        }
    }
    return max_us_;
}

uint32_t LoopStats::overrun_ppm(void) const
{
    if (count_ == 0) return 0;
    return (uint32_t)(((uint64_t)overruns_ * 1000000ULL) / count_);
}

uint32_t LoopStats::bucket(int i) const
{
    if (i < 0 || i >= LOOPSTATS_BUCKETS) return 0;
    return bucket_[i];
}

uint32_t LoopStats::bucket_edge_us(int i)
{
    if (i < 0 || i >= LOOPSTATS_BUCKETS) return 0;
    return EDGE[i];
}
