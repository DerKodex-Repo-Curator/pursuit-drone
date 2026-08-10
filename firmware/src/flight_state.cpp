#include "flight_state.h"

#include <cmath>
#include <cstring>

#include "config.h"

namespace {

/* Silence long enough that the command stream is genuinely gone rather than
 * one dropped packet. At the pump's 30Hz this is ~12 missed packets. */
constexpr uint32_t LINK_TIMEOUT_MS = PURSUIT_LINK_TIMEOUT_MS;
/* Still silent this long and the aircraft puts itself down. */
constexpr uint32_t LAND_TIMEOUT_MS = 3000;

float clampf(float v, float lo, float hi)
{
    if (!std::isfinite(v)) return 0.0f;
    return v < lo ? lo : (v > hi ? hi : v);
}

}  /* namespace */

FlightState::FlightState()
    : state_(PURSUIT_STATE_BOOT),
      last_cmd_ms_(0),
      last_cmd_t_ms_(0),
      last_seq_(0),
      have_seq_(false),
      sensors_ok_(false),
      setpoint_{},
      accepted_(0),
      rejected_(0),
      stale_(0)
{
}

void FlightState::enter(uint8_t s)
{
    if (state_ == s) return;
    state_ = s;

    /* Leaving any powered state neutralises the setpoint. Whatever the
     * aircraft was told to do a moment ago is not a safe default for what
     * comes next. */
    if (s == PURSUIT_STATE_IDLE || s == PURSUIT_STATE_FAILSAFE ||
        s == PURSUIT_STATE_LANDING) {
        setpoint_.roll_deg = 0.0f;
        setpoint_.pitch_deg = 0.0f;
        setpoint_.yaw_rate_dps = 0.0f;
        setpoint_.locked = false;
    }
}

void FlightState::sensors_ready()
{
    sensors_ok_ = true;
    if (state_ == PURSUIT_STATE_BOOT) enter(PURSUIT_STATE_IDLE);
}

void FlightState::disarm()
{
    have_seq_ = false;
    enter(PURSUIT_STATE_IDLE);
}

void FlightState::emergency_land()
{
    if (state_ == PURSUIT_STATE_ARMED || state_ == PURSUIT_STATE_FLYING ||
        state_ == PURSUIT_STATE_FAILSAFE) {
        enter(PURSUIT_STATE_LANDING);
    } else {
        enter(PURSUIT_STATE_IDLE);
    }
}

bool FlightState::on_packet(const uint8_t *buf, uint32_t len, uint32_t now_ms)
{
    /* Validation before anything else touches the payload: length, magic,
     * version, CRC and finiteness are all checked by pursuit_cmd_valid(). */
    if (!pursuit_cmd_valid(buf, len)) {
        rejected_++;
        return false;
    }

    pursuit_cmd_t cmd;
    std::memcpy(&cmd, buf, sizeof(cmd));

    /* Reordered or replayed. UDP does both, and acting on an old setpoint is
     * indistinguishable from acting on a wrong one. */
    if (have_seq_ && !pursuit_seq_newer(cmd.seq, last_seq_)) {
        stale_++;
        return false;
    }

    last_seq_ = cmd.seq;
    have_seq_ = true;
    last_cmd_ms_ = now_ms;
    last_cmd_t_ms_ = cmd.t_ms;
    accepted_++;

    const bool armed_request = (cmd.flags & PURSUIT_FLAG_ARMED) != 0;
    const bool failsafe = (cmd.flags & PURSUIT_FLAG_FAILSAFE) != 0;

    setpoint_.roll_deg = clampf(cmd.roll, -MAX_ANGLE_DEG, MAX_ANGLE_DEG);
    setpoint_.pitch_deg = clampf(cmd.pitch, -MAX_ANGLE_DEG, MAX_ANGLE_DEG);
    setpoint_.yaw_rate_dps =
        clampf(cmd.yaw_rate, -MAX_YAW_RATE_DPS, MAX_YAW_RATE_DPS);
    setpoint_.alt_hold = (cmd.flags & PURSUIT_FLAG_ALT_HOLD) != 0;
    setpoint_.alt_sp_m = cmd.alt_sp;
    setpoint_.locked = (cmd.flags & PURSUIT_FLAG_LOCKED) != 0;

    if (!sensors_ok_) return true;   /* stay in BOOT until sensors pass */

    /* --- authority ------------------------------------------------------
     * The arm bit must be present on THIS packet. It is a held control on
     * the operator's side, so it must be a held condition here too; treating
     * it as a latch would mean one packet licences unlimited flight.
     */
    if (!armed_request) {
        if (state_ == PURSUIT_STATE_FLYING) {
            /* Disarming in the air is a deliberate abort, so descend under
             * control rather than cutting motors and dropping it. */
            enter(PURSUIT_STATE_LANDING);
        } else if (state_ != PURSUIT_STATE_LANDING) {
            enter(PURSUIT_STATE_IDLE);
        }
        return true;
    }

    if (failsafe) {
        /* The laptop is alive but cannot see the target. Hold, do not steer.
         * Distinct from silence, which eventually lands. */
        setpoint_.roll_deg = 0.0f;
        setpoint_.pitch_deg = 0.0f;
        setpoint_.yaw_rate_dps = 0.0f;
        if (state_ == PURSUIT_STATE_FLYING || state_ == PURSUIT_STATE_ARMED) {
            enter(PURSUIT_STATE_FAILSAFE);
        }
        return true;
    }

    switch (state_) {
    case PURSUIT_STATE_IDLE:
        enter(PURSUIT_STATE_ARMED);
        break;
    case PURSUIT_STATE_ARMED: {
        /* Only leave ARMED once actually commanded upward, so that arming
         * on the bench does not read as flight. */
        const bool climbing = setpoint_.alt_hold
            ? (setpoint_.alt_sp_m > 0.15f)
            : (setpoint_.alt_sp_m > MIN_SPIN_THROTTLE);
        if (climbing) enter(PURSUIT_STATE_FLYING);
        break;
    }
    case PURSUIT_STATE_FAILSAFE:
        enter(PURSUIT_STATE_FLYING);   /* recovered */
        break;
    default:
        break;
    }
    return true;
}

void FlightState::tick(uint32_t now_ms)
{
    if (state_ == PURSUIT_STATE_BOOT || state_ == PURSUIT_STATE_IDLE) return;
    if (last_cmd_ms_ == 0) return;

    const uint32_t silent = now_ms - last_cmd_ms_;

    if (silent > LAND_TIMEOUT_MS) {
        enter(PURSUIT_STATE_LANDING);
        return;
    }
    if (silent > LINK_TIMEOUT_MS) {
        /* Level off and hold. Explicitly NOT holding the last command: a
         * stale yaw rate re-applied at loop rate flies a constant-radius
         * circle until something stops it. */
        setpoint_.roll_deg = 0.0f;
        setpoint_.pitch_deg = 0.0f;
        setpoint_.yaw_rate_dps = 0.0f;
        setpoint_.locked = false;
        if (state_ != PURSUIT_STATE_LANDING) enter(PURSUIT_STATE_FAILSAFE);
    }
}

bool FlightState::motors_enabled() const
{
    return state_ == PURSUIT_STATE_ARMED || state_ == PURSUIT_STATE_FLYING ||
           state_ == PURSUIT_STATE_FAILSAFE || state_ == PURSUIT_STATE_LANDING;
}

uint32_t FlightState::silence_ms(uint32_t now_ms) const
{
    if (last_cmd_ms_ == 0) return 0;
    return now_ms - last_cmd_ms_;
}
