/* PURSUIT-1 board configuration.
 *
 * Pin numbers are the ESP32-S3 GPIO numbers, taken from scripts/design.py —
 * which is the canonical netlist the PCB was generated from. The XIAO silk
 * labels (D0, D1, ...) are given alongside because that is what is printed on
 * the module, and they are NOT the same numbers.
 *
 * If design.py changes, change this file. There is no automatic link between
 * them, so a mismatch here means the firmware drives the wrong motor and the
 * aircraft flips on take-off.
 */
#ifndef PURSUIT_CONFIG_H
#define PURSUIT_CONFIG_H

#include <stdint.h>

/* ---- motors -------------------------------------------------------------
 * X-quad layout seen from above, USB-C edge forward:
 *
 *      M1 (FL, CW)      M2 (FR, CCW)
 *              \        /
 *               \      /
 *                +----+          front = USB-C
 *               /      \
 *              /        \
 *      M3 (RL, CCW)     M4 (RR, CW)
 *
 * Diagonal pairs spin the same way, which is the standard X pattern: it makes
 * the net torque cancel in hover and lets a torque imbalance produce yaw.
 * The belly silkscreen marks each position AND its required rotation — the
 * firmware mix and the fitted motors must agree or yaw runs away.
 */
#define PIN_M1_PWM   1   /* D0  */
#define PIN_M2_PWM   2   /* D1  */
#define PIN_M3_PWM   4   /* D3  */
#define PIN_M4_PWM   7   /* D8  */

/* ---- sensors ------------------------------------------------------------ */
#define PIN_SDA      5   /* D4  — XIAO default SDA */
#define PIN_SCL      6   /* D5  — XIAO default SCL */
#define PIN_IMU_INT  8   /* D9  — ICM-42688-P INT1, open-drain */

#define I2C_ADDR_IMU   0x68  /* AD0 tied low on the board */
#define I2C_ADDR_TOF   0x29  /* VL53L1X default */
#define I2C_FREQ_HZ    400000

/* ---- battery ------------------------------------------------------------
 * 100k/100k divider, so the ADC sees half the pack voltage. GPIO3 is ADC1_CH2
 * and also a strap pin — high-Z at reset is fine, which is why it was chosen.
 */
#define PIN_VBAT_SENSE 3   /* D2 */
#define VBAT_DIVIDER   2.0f
#define ADC_VREF       3.3f

/* ---- indicators --------------------------------------------------------- */
#define PIN_LED_CTRL 9   /* D10 — BOTH LEDs share this pin */

/* ---- PWM ----------------------------------------------------------------
 * 32kHz is above audio and comfortably inside the AO3400A's switching
 * ability, while staying low enough that gate-charge losses do not dominate
 * at 1S. 8-bit resolution is all a brushed motor can use.
 */
#define MOTOR_PWM_FREQ_HZ  32000
#define MOTOR_PWM_BITS     8
#define MOTOR_PWM_MAX      ((1 << MOTOR_PWM_BITS) - 1)

/* ---- loop rates --------------------------------------------------------- */
#define LOOP_HZ        1000   /* attitude control */
#define TOF_HZ         30     /* VL53L1X long-range mode caps out near here */
#define TELEM_HZ       20
#define BATT_HZ        50

/* ---- flight limits ------------------------------------------------------
 * Deliberately conservative: this airframe has T:W around 1.6 once pack sag
 * is accounted for, so there is little authority to spare.
 */
#define MAX_ANGLE_DEG      25.0f
#define MAX_YAW_RATE_DPS   180.0f
#define MIN_SPIN_THROTTLE  0.06f  /* keeps motors turning so control is live */
#define MAX_THROTTLE       0.95f  /* leave headroom for attitude correction */

/* ---- battery protection -------------------------------------------------
 * The XIAO's 3V3 comes from an SGM6029C buck that drops out around 3.5V in.
 * SPICE puts the pack at 3.61V during a four-motor full-throttle burst with
 * an optimistic 60mR source model, so a real cell is worse. Without a limiter
 * the flight controller browns out in the air.
 */
#define VBAT_CRITICAL_V   3.40f  /* land now */
#define VBAT_LOW_V        3.60f  /* start clamping throttle */
#define VBAT_NOMINAL_V    3.85f  /* full authority above this */
#define VBAT_FULL_V       4.20f

/* ---- IMU mounting orientation -------------------------------------------
 * The flight maths runs in the FLU body frame: +X forward (USB-C edge),
 * +Y LEFT, +Z up, right-handed — see src/frame.h. Whether the ICM-42688-P's
 * own axes deliver that depends on how the chip sits on the PCB, which no
 * amount of desk reasoning settles. These macros remap sensor axes into the
 * body frame in one place; the bench 'o' test is the ground truth that
 * validates them BEFORE any flight:
 *
 *     tilt the RIGHT edge down  -> roll must read POSITIVE
 *     tilt the NOSE down        -> pitch must read POSITIVE
 *
 * If either sign is wrong, fix it HERE (swap/negate), never in the filter.
 * The same mapping applies to gyro and accel — any rigid mounting is a
 * permutation with signs, identical for both.
 */
#define IMU_ORIENT_X(x, y, z) (x)
#define IMU_ORIENT_Y(x, y, z) (y)
#define IMU_ORIENT_Z(x, y, z) (z)

#endif /* PURSUIT_CONFIG_H */
