/* Altitude hold.
 *
 * Structure matters more than the gains here. The naive version is a PID on
 * height error driving throttle directly, and it behaves badly for a reason
 * that is easy to miss: hovering needs a large, roughly constant throttle
 * (~60% on this airframe), and a pure PID has to manufacture that entirely
 * from its integrator. So every time the loop engages, the aircraft sinks
 * while the integrator winds up to find hover again.
 *
 * Instead the output is
 *
 *      throttle = hover_throttle + pid(height_error)
 *
 * where `hover_throttle` is *learned* — it drifts slowly toward whatever
 * throttle is actually holding station. The PID then only ever has to supply
 * the difference, which is small, so it can be gentle and still responsive.
 * The learned value also survives setpoint changes, so climbing to a new
 * height does not re-derive hover from scratch.
 *
 * Learning is deliberately slow (tens of seconds) and only happens while the
 * altitude error is small. Learning during a climb would teach it that the
 * climb throttle is hover, and it would then climb forever.
 *
 * Degrades to passing pilot throttle straight through when the rangefinder is
 * unavailable — an aircraft that cannot measure height must still be flyable,
 * just not hands-off.
 */
#ifndef PURSUIT_ALT_HOLD_H
#define PURSUIT_ALT_HOLD_H

#include <stdint.h>

struct AltHoldConfig {
    float kp;              /* throttle per metre of error   */
    float ki;              /* per metre-second              */
    float kd;              /* per m/s of climb rate         */
    float i_limit;
    float max_climb_ms;    /* setpoint slew, m/s            */
    float hover_learn_hz;  /* how fast the baseline adapts  */
    float min_throttle;
    float max_throttle;
};

AltHoldConfig default_alt_hold_config();

/* Clamp a commanded height to what the fitted sensors can actually observe.
 *
 * This is deliberately on the aircraft rather than in the vision stack. The
 * laptop has no way to know whether a barometer is soldered to this particular
 * board — and the consequence of getting it wrong is not an error message, it
 * is a climb past the rangefinder's range into open-loop throttle. A limit
 * that depends on hardware belongs where the hardware is.
 *
 * `requested_m` is whatever arrived over the link, already CRC-checked but not
 * otherwise trusted: a non-finite or negative value returns the floor rather
 * than propagating into the altitude loop.
 */
float pursuit_clamp_alt_setpoint(float requested_m, bool baro_present,
                                 float tof_ceiling_m, float baro_ceiling_m);

class AltitudeHold {
public:
    AltitudeHold();
    void configure(const AltHoldConfig &c);

    /* Call at the rangefinder's rate. `height_m` must be tilt-corrected.
     * `have_height` false means fly on `fallback_throttle` instead. */
    float update(float target_m, float height_m, bool have_height,
                 float fallback_throttle, float dt_s);

    /* Call on arm. Keeps the learned hover estimate — it is a property of the
     * airframe and battery, not of this particular flight — but clears the
     * integrator and the rate estimate, which are not. */
    void arm(float current_height_m);
    void reset_learning();

    float hover_throttle() const { return hover_; }
    float climb_rate() const { return climb_ms_; }
    bool  holding() const { return holding_; }

private:
    AltHoldConfig c_;
    float hover_;
    float integral_;
    float sp_m_;          /* rate-limited, follows the commanded target */
    float prev_height_;
    float climb_ms_;
    bool  primed_;
    bool  holding_;
};

#endif /* PURSUIT_ALT_HOLD_H */
