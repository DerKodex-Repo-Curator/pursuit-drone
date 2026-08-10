/* Body frame conventions — read this before touching any sign.
 *
 * The body frame this firmware's maths actually implements is FLU:
 *
 *      +X forward (USB-C edge), +Y LEFT, +Z up   (right-handed)
 *
 * Not "Y right". X-forward/Y-right/Z-up is LEFT-handed (fwd x right = down),
 * and every quaternion identity silently assumes a right-handed frame. The
 * Madgwick filter, the accel model in the tests (level => az = +1g) and the
 * Euler extraction are all mutually consistent in FLU:
 *
 *      roll  + = right wing down     (rotation +about +X)
 *      pitch + = nose DOWN           (rotation +about +Y_left)
 *      gyro  gx,gy follow the same signs; gz + = nose swinging LEFT (CCW
 *            from above), because +Z is up.
 *
 * Happy accidents and one trap:
 *
 *   roll   protocol roll+  = right wing down  = FLU roll+   -> no conversion
 *   pitch  protocol pitch+ = nose down        = FLU pitch+  -> no conversion
 *   yaw    protocol yaw+   = CLOCKWISE from above = -gz     -> MUST NEGATE
 *
 * Feeding raw gz to the yaw rate PID is positive feedback: command a right
 * turn, the aircraft turns right, gz goes NEGATIVE, the error grows, and it
 * spins up until yaw authority saturates. The conversion lives here, once,
 * with a closed-loop test in test_attitude.cpp that fails if anyone "fixes"
 * the sign back.
 *
 * Whether the PHYSICAL sensor delivers FLU depends on how the ICM-42688-P is
 * mounted on the PCB — see the IMU_ORIENT_* macros in config.h and the bench
 * 'o' test, which is the ground truth before any flight.
 */
#ifndef PURSUIT_FRAME_H
#define PURSUIT_FRAME_H

#include <stdint.h>

#include "pursuit_link.h"

/* Measured yaw rate in the protocol's clockwise-positive convention. */
static inline float pursuit_yaw_rate_cw(float gz_flu)
{
    return -gz_flu;
}

/* Throttle gate for the ARMED state.
 *
 * ARMED means "authorised, not yet flying". Passing the commanded throttle
 * straight through in ARMED lets a spring-centred stick (which rests at half
 * travel) spin the motors to 50% the instant the deadman is squeezed — on a
 * bench, at half throttle, with the aircraft unrestrained. Until the state
 * machine declares FLYING, the motors hold the idle floor and nothing more.
 */
static inline float pursuit_armed_idle_gate(uint8_t state, float throttle,
                                            float idle_floor)
{
    if (state == PURSUIT_STATE_ARMED)
        return throttle < idle_floor ? throttle : idle_floor;
    return throttle;
}

#endif /* PURSUIT_FRAME_H */
