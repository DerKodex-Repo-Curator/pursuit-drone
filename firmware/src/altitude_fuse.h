/* Fusing a rangefinder with a barometer into one height.
 *
 * Two sensors with opposite failure modes:
 *
 *   VL53L1X   ~1 cm precise, TRUE height above the ground under it, and it
 *             stops existing past 3.5 m. Sunlight shortens that further.
 *   BMP390    works at any altitude, but reports PRESSURE altitude against an
 *             arbitrary datum, drifts with weather, and is noisy.
 *
 * Neither flies the aircraft alone. Below 3 m the rangefinder is simply
 * better. Above 3.5 m it does not exist. The whole problem is the handover.
 *
 * THE TRAP, and the reason this is its own tested module: the barometer's zero
 * is not the ground. If you switch sources at 3.5 m you get a step of however
 * far apart the two datums happen to be — metres, mid-climb, under power, into
 * an altitude loop that will faithfully try to correct it. So the offset
 * between them is learned continuously WHILE the rangefinder is valid, and the
 * barometer is only ever used with that offset already subtracted.
 *
 * Blending, not switching. Between `tof_trust_m` and `tof_max_m` the two are
 * weighted so the source changes gradually rather than on one sample.
 *
 * No Arduino headers: this is the part that has to be right, so it is tested
 * on the host.
 */
#ifndef PURSUIT_ALTITUDE_FUSE_H
#define PURSUIT_ALTITUDE_FUSE_H

#include <stdint.h>

enum class AltSource : uint8_t {
    None = 0,    /* nothing usable yet                        */
    Tof  = 1,    /* rangefinder alone                         */
    Blend = 2,   /* handover region, both weighted            */
    Baro = 3,    /* barometer alone, offset-corrected         */
};

struct AltFuseConfig {
    /* Below this the rangefinder is the only thing used. */
    float tof_trust_m = 3.0f;
    /* Above this the rangefinder is ignored entirely — it is past its spec
     * and returning noise or nothing. */
    float tof_max_m = 3.5f;
    /* Time constant for learning the baro-to-ground offset. Slow: the offset
     * is a property of the weather, not of this second. */
    float offset_tau_s = 3.0f;
    /* Barometer low-pass. It is genuinely noisy on a small airframe even with
     * foam over the port. */
    float baro_tau_s = 0.35f;
    /* A reading older than this is not a reading. */
    float max_age_s = 0.5f;
    /* Do not learn the datum offset until the barometer's own filter has
     * settled. Seeding from a sample taken while it is still converging bakes
     * that transient into the offset, and it then decays with the SLOW offset
     * time constant — visible as a metre of drift over the first ten seconds
     * of a flight. Three baro time constants is 95% converged. */
    float baro_settle_s = 1.2f;
};

class AltitudeFuse {
public:
    explicit AltitudeFuse(const AltFuseConfig &cfg = AltFuseConfig());

    void configure(const AltFuseConfig &cfg);

    /* Call on arm. Captures the current barometric altitude as the datum, so
     * height is measured from where the aircraft took off rather than from
     * sea level — which is the only definition the pilot cares about, and the
     * only one that survives the weather changing between flights. */
    void arm(float baro_alt_m);
    void reset();

    /* Raw barometric altitude in metres, from the pressure formula. Any
     * datum: the offset machinery below makes it relative. */
    void push_baro(float baro_alt_m, float dt_s);

    /* Tilt-compensated height from the rangefinder. `valid` false when the
     * sensor reported nothing usable — do NOT pass a stale value. */
    void push_tof(float height_m, bool valid, float dt_s);

    /* Metres above the arming point. Only meaningful when valid(). */
    float height() const { return height_m_; }
    bool valid() const { return source_ != AltSource::None; }
    AltSource source() const { return source_; }

    /* How far apart the two datums are, metres. Exposed because a large or
     * rapidly moving offset means the barometer is being disturbed — prop
     * wash reaching the port is the usual cause, and it is worth seeing in
     * the blackbox rather than guessing at it later. */
    float offset() const { return offset_m_; }
    bool offset_learned() const { return offset_valid_; }

    /* Seconds since each source last produced something usable. */
    float tof_age_s() const { return tof_age_s_; }
    float baro_age_s() const { return baro_age_s_; }

private:
    AltFuseConfig cfg_;

    float height_m_;
    float baro_raw_m_;      /* filtered, still on the sensor's own datum */
    float baro_zero_m_;     /* datum captured at arm                     */
    float offset_m_;        /* baro-relative minus tof, learned          */
    bool  offset_valid_;
    bool  baro_primed_;
    bool  armed_;
    float tof_last_m_;
    float tof_age_s_;
    float baro_age_s_;
    float baro_settled_s_;
    AltSource source_;

    void recompute();
};

#endif /* PURSUIT_ALTITUDE_FUSE_H */
