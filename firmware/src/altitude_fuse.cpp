#include "altitude_fuse.h"

#include <cmath>

namespace {

/* First-order low-pass, exact rather than forward-Euler, so the response does
 * not change with loop rate. The rangefinder runs at 30 Hz and the barometer
 * near 50; the loop calling this runs at 1 kHz. */
inline float lp(float prev, float x, float dt, float tau)
{
    if (tau <= 0.0f) return x;
    const float a = 1.0f - std::exp(-dt / tau);
    return prev + a * (x - prev);
}

inline float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

}  /* namespace */

AltitudeFuse::AltitudeFuse(const AltFuseConfig &cfg) : cfg_(cfg)
{
    reset();
}

void AltitudeFuse::configure(const AltFuseConfig &cfg) { cfg_ = cfg; }

void AltitudeFuse::reset()
{
    height_m_ = 0.0f;
    baro_raw_m_ = 0.0f;
    baro_zero_m_ = 0.0f;
    offset_m_ = 0.0f;
    offset_valid_ = false;
    baro_primed_ = false;
    armed_ = false;
    tof_last_m_ = 0.0f;
    tof_age_s_ = 1e9f;
    baro_age_s_ = 1e9f;
    baro_settled_s_ = 0.0f;
    source_ = AltSource::None;
}

void AltitudeFuse::arm(float baro_alt_m)
{
    /* Height above the launch point, not above sea level. Pressure altitude
     * moves tens of metres with the weather between one session and the next,
     * so an absolute datum would make every flight start at a different
     * number. */
    if (std::isfinite(baro_alt_m)) {
        baro_zero_m_ = baro_alt_m;
        baro_raw_m_ = baro_alt_m;
        baro_primed_ = true;
    }
    armed_ = true;
    /* The offset is re-learned each flight. The aircraft may not be sitting
     * where it sat last time. */
    offset_valid_ = false;
    offset_m_ = 0.0f;
    baro_settled_s_ = 0.0f;
}

void AltitudeFuse::push_baro(float baro_alt_m, float dt_s)
{
    if (!std::isfinite(baro_alt_m) || dt_s <= 0.0f) return;

    if (!baro_primed_) {
        baro_raw_m_ = baro_alt_m;
        baro_zero_m_ = baro_alt_m;
        baro_primed_ = true;
    } else {
        baro_raw_m_ = lp(baro_raw_m_, baro_alt_m, dt_s, cfg_.baro_tau_s);
    }
    baro_settled_s_ += dt_s;
    baro_age_s_ = 0.0f;
    recompute();
}

void AltitudeFuse::push_tof(float height_m, bool valid, float dt_s)
{
    if (dt_s > 0.0f) {
        tof_age_s_ += dt_s;
        baro_age_s_ += dt_s;
    }

    if (valid && std::isfinite(height_m)) {
        tof_last_m_ = height_m;
        tof_age_s_ = 0.0f;

        /* Learn the datum difference, but ONLY where the rangefinder is
         * genuinely trustworthy. Learning it at 3.4 m — where the VL53L1X is
         * already returning long, noisy readings — poisons the very number
         * the barometer depends on above the handover. */
        if (baro_primed_ && baro_settled_s_ >= cfg_.baro_settle_s
            && height_m <= cfg_.tof_trust_m) {
            const float rel = baro_raw_m_ - baro_zero_m_;
            const float measured = rel - height_m;
            if (!offset_valid_) {
                offset_m_ = measured;      /* seed, do not ramp from zero */
                offset_valid_ = true;
            } else {
                offset_m_ = lp(offset_m_, measured, dt_s, cfg_.offset_tau_s);
            }
        }
    }
    recompute();
}

void AltitudeFuse::recompute()
{
    const bool tof_fresh = tof_age_s_ <= cfg_.max_age_s;
    const bool baro_fresh = baro_primed_ && baro_age_s_ <= cfg_.max_age_s;

    /* Barometer, expressed as height above the launch point. */
    const float baro_h = (baro_raw_m_ - baro_zero_m_)
                       - (offset_valid_ ? offset_m_ : 0.0f);

    if (!tof_fresh && !baro_fresh) {
        source_ = AltSource::None;
        return;
    }
    if (!baro_fresh) {                       /* no barometer fitted, or dead */
        height_m_ = tof_last_m_;
        source_ = AltSource::Tof;
        return;
    }
    if (!tof_fresh) {
        height_m_ = baro_h;
        source_ = AltSource::Baro;
        return;
    }

    /* Both present. Weight by where the RANGEFINDER says we are, since that is
     * the sensor whose validity is in question. */
    const float lo = cfg_.tof_trust_m, hi = cfg_.tof_max_m;
    if (tof_last_m_ <= lo) {
        height_m_ = tof_last_m_;
        source_ = AltSource::Tof;
    } else if (tof_last_m_ >= hi) {
        height_m_ = baro_h;
        source_ = AltSource::Baro;
    } else {
        /* Linear crossfade across the handover band. Switching on one sample
         * puts a step into the altitude loop at the exact moment the aircraft
         * is climbing through it. */
        const float w = clampf((tof_last_m_ - lo) / (hi - lo), 0.0f, 1.0f);
        height_m_ = (1.0f - w) * tof_last_m_ + w * baro_h;
        source_ = AltSource::Blend;
    }
}
