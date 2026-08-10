/* VL53L1X belly rangefinder — height above whatever is directly below.
 *
 * The sensor points along the airframe's -Z axis, not at the ground. Those
 * are the same direction only while level, and the difference is not small:
 * banked at 30 degrees the beam travels 1/cos(30) = 1.15x further to reach
 * the same floor, so the aircraft reads 15% high and descends to correct an
 * error that does not exist. Every bank becomes a small unintended descent.
 *
 * So the raw range is projected back onto the vertical:
 *
 *      height = range * cos(roll) * cos(pitch)
 *
 * Past `max_tilt_deg` the correction stops being trustworthy — the beam is
 * pointing at a wall or a chair leg rather than the floor, and no amount of
 * trigonometry recovers a measurement of the wrong surface. Those samples are
 * rejected outright and the estimate coasts.
 *
 * The maths and the validity logic are hardware-free so the host tests can
 * drive them; only `begin()` and `read_raw_mm()` touch I2C.
 */
#ifndef PURSUIT_RANGEFINDER_H
#define PURSUIT_RANGEFINDER_H

#include <stdint.h>

struct RangeSample {
    float height_m;     /* tilt-corrected height above the surface */
    float raw_m;        /* what the sensor actually measured       */
    bool  valid;
};

/* Pure geometry, exposed for testing. `range_m` is the sensor's own reading,
 * roll and pitch in degrees. Returns a negative value when the attitude makes
 * the reading meaningless. */
float project_range_to_height(float range_m, float roll_deg, float pitch_deg,
                              float max_tilt_deg);

class Rangefinder {
public:
    Rangefinder();

    bool begin();
    bool available() const { return ready_; }

    /* Poll. Returns a sample whose `valid` is false when the sensor had
     * nothing useful — out of range, too much ambient light, or the aircraft
     * banked past the usable angle. */
    RangeSample update(float roll_deg, float pitch_deg);

    /* Last good height, and how long ago it was. The controller uses the age
     * to decide when to stop trusting altitude hold at all. */
    float last_height_m() const { return last_height_m_; }
    uint32_t consecutive_bad() const { return bad_; }

    const char *last_error() const { return err_; }

private:
    bool  ready_;
    float last_height_m_;
    float filtered_m_;
    uint32_t bad_;
    const char *err_;
};

#endif /* PURSUIT_RANGEFINDER_H */
