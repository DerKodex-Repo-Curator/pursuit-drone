#include "rangefinder.h"

#include <cmath>

#include "config.h"

namespace {

constexpr float DEG2RAD = 0.017453292519943295f;

}  /* namespace */

float project_range_to_height(float range_m, float roll_deg, float pitch_deg,
                              float max_tilt_deg)
{
    if (!std::isfinite(range_m) || range_m <= 0.0f) return -1.0f;
    if (!std::isfinite(roll_deg) || !std::isfinite(pitch_deg)) return -1.0f;

    /* Total tilt from vertical. cos of the combined tilt is the product of
     * the two cosines — this is the projection of the body -Z axis onto the
     * world vertical, not an approximation. */
    const float cr = cosf(roll_deg * DEG2RAD);
    const float cp = cosf(pitch_deg * DEG2RAD);
    const float cos_tilt = cr * cp;

    const float limit = cosf(max_tilt_deg * DEG2RAD);
    if (cos_tilt <= limit || cos_tilt <= 0.0f) return -1.0f;

    return range_m * cos_tilt;
}

#ifndef PURSUIT_HOST_BUILD

#include <VL53L1X.h>
#include <Wire.h>

/* The VL53L1X in long-distance mode is specified to 4m in the dark and much
 * less in sunlight. Anything beyond this is "no floor found" rather than a
 * real height — over a stairwell it genuinely is. Below MIN_RANGE_M the
 * sensor is inside its own minimum and the number is noise. Past
 * MAX_TILT_DEG the beam points at furniture, not the floor. */
static constexpr float MAX_RANGE_M = 3.5f;
static constexpr float MIN_RANGE_M = 0.03f;
static constexpr float MAX_TILT_DEG = 35.0f;

static VL53L1X g_sensor;

Rangefinder::Rangefinder()
    : ready_(false), last_height_m_(0.0f), filtered_m_(0.0f), bad_(0),
      err_("")
{
}

bool Rangefinder::begin()
{
    g_sensor.setBus(&Wire);
    g_sensor.setTimeout(100);
    if (!g_sensor.init()) {
        err_ = "VL53L1X not responding";
        ready_ = false;
        return false;
    }

    /* Long mode reaches 4m; Short is more robust to ambient light but caps
     * near 1.3m, which is below the altitude ceiling this aircraft uses. */
    g_sensor.setDistanceMode(VL53L1X::Long);
    /* 33ms budget matches the 30Hz altitude loop. A longer budget is quieter
     * but adds lag to a loop that is already the slowest thing on the
     * aircraft. */
    g_sensor.setMeasurementTimingBudget(33000);
    g_sensor.startContinuous(33);

    ready_ = true;
    err_ = "";
    return true;
}

RangeSample Rangefinder::update(float roll_deg, float pitch_deg)
{
    RangeSample s{0.0f, 0.0f, false};
    if (!ready_) return s;

    if (!g_sensor.dataReady()) {
        /* Not an error — the sensor runs at 30Hz and the loop at 1kHz, so
         * most calls legitimately have nothing new. */
        s.height_m = last_height_m_;
        return s;
    }

    const uint16_t mm = g_sensor.read(false);
    if (g_sensor.timeoutOccurred()) {
        bad_++;
        err_ = "timeout";
        return s;
    }

    /* The status byte matters: the sensor reports a number even when it did
     * not really see anything, and acting on a phantom floor is how a drone
     * climbs into a ceiling. */
    const auto status = g_sensor.ranging_data.range_status;
    if (status != VL53L1X::RangeValid) {
        bad_++;
        return s;
    }

    const float raw_m = (float)mm * 0.001f;
    if (raw_m < MIN_RANGE_M || raw_m > MAX_RANGE_M) {
        bad_++;
        return s;
    }

    const float h = project_range_to_height(raw_m, roll_deg, pitch_deg,
                                            MAX_TILT_DEG);
    if (h < 0.0f) {
        bad_++;          /* banked too far to trust it */
        return s;
    }

    /* Light smoothing. The VL53L1X is noisy at a few centimetres RMS, and an
     * altitude loop chasing that noise makes the aircraft bob. */
    if (bad_ > 5 || filtered_m_ <= 0.0f) filtered_m_ = h;   /* re-acquire */
    else filtered_m_ += 0.35f * (h - filtered_m_);

    bad_ = 0;
    last_height_m_ = filtered_m_;
    s.height_m = filtered_m_;
    s.raw_m = raw_m;
    s.valid = true;
    return s;
}

#else  /* PURSUIT_HOST_BUILD */

Rangefinder::Rangefinder()
    : ready_(false), last_height_m_(0.0f), filtered_m_(0.0f), bad_(0),
      err_("host") {}
bool Rangefinder::begin() { return false; }
RangeSample Rangefinder::update(float, float)
{
    RangeSample s{};
    s.height_m = filtered_m_;   /* referenced so the host build stays quiet */
    return s;
}

#endif
