/* Flight state machine and link watchdog.
 *
 * Owns the only question that matters on a small aircraft: are the motors
 * allowed to turn right now, and if so, on whose authority?
 *
 *            power on
 *               |
 *             BOOT ---- sensors ok ----> IDLE <--------------+
 *                                         |                   |
 *                       valid ARMED cmd   |                   | disarm,
 *                                         v                   | or landed
 *                                       ARMED                 |
 *                                         |                   |
 *                        throttle raised  |                   |
 *                                         v                   |
 *                                      FLYING ----------------+
 *                                         |
 *                     no valid command    |  (LINK_TIMEOUT_MS)
 *                                         v
 *                                     FAILSAFE
 *                                         |
 *                         still silent    |  (LAND_TIMEOUT_MS)
 *                                         v
 *                                      LANDING --> IDLE
 *                                                (LANDING_TIMEOUT_MS)
 *
 * The rules that make this safe rather than merely tidy:
 *
 * 1. FLAG_ARMED is a HELD authority, not a latch. Every accepted packet must
 *    carry it. One armed packet does not licence the next second of flight.
 *
 * 2. A stale command is never re-applied. On timeout the aircraft levels and
 *    holds, then descends — it does not keep the last yaw rate, which would
 *    fly it in a circle at constant speed into whatever is nearby.
 *
 * 3. Packets failing validation are DISCARDED, not clamped. A command you
 *    cannot trust is worse than no command, because absence is handled
 *    safely and corruption is acted upon.
 *
 * Free of Arduino headers so the host tests can drive it with a fake clock.
 */
#ifndef PURSUIT_FLIGHT_STATE_H
#define PURSUIT_FLIGHT_STATE_H

#include <stdint.h>

#include "pursuit_link.h"

struct Setpoint {
    float roll_deg;
    float pitch_deg;
    float yaw_rate_dps;
    float alt_sp_m;      /* metres when alt_hold, else throttle 0..1 */
    bool  alt_hold;
    bool  locked;        /* vision has a confirmed target */
};

class FlightState {
public:
    FlightState();

    /* Feed a raw UDP datagram. Returns true if it was accepted.
     * `now_ms` is a monotonic millisecond clock. */
    bool on_packet(const uint8_t *buf, uint32_t len, uint32_t now_ms);

    /* Advance time. Call every loop, packet or no packet. */
    void tick(uint32_t now_ms);

    /* Sensors have initialised and self-tested. */
    void sensors_ready();

    /* Force a safe state — used by a low battery or a failed sensor. */
    void emergency_land();
    void disarm();

    uint8_t state() const { return state_; }
    bool motors_enabled() const;
    const Setpoint &setpoint() const { return setpoint_; }

    uint32_t accepted() const { return accepted_; }
    uint32_t rejected() const { return rejected_; }
    uint32_t stale() const { return stale_; }
    uint32_t silence_ms(uint32_t now_ms) const;

    /* The sender's own timestamp from the last accepted command. Echoed back
     * in telemetry so the laptop can measure a true round-trip without the
     * two ends needing a shared clock. */
    uint32_t last_cmd_t_ms() const { return last_cmd_t_ms_; }

private:
    void enter(uint8_t s);

    uint8_t  state_;
    uint32_t last_cmd_ms_;
    uint32_t last_cmd_t_ms_;
    uint32_t last_seq_;
    bool     have_seq_;
    bool     sensors_ok_;

    Setpoint setpoint_;

    /* When the current LANDING began, and whether that clock is running.
     * Started by tick() rather than enter() because enter() has no clock;
     * cleared on any state that is not LANDING so a second landing gets a
     * second full descent. */
    uint32_t landing_since_ms_;
    bool     landing_timing_;

    uint32_t accepted_;
    uint32_t rejected_;
    uint32_t stale_;
};

#endif /* PURSUIT_FLIGHT_STATE_H */
