/* ICM-42688-P 6-axis IMU, I2C.
 *
 * TDK's low-noise successor to the MPU-6000 family. On this board it sits at
 * 0x68 (AD0 tied low on the PCB — never floating, which is why the address is
 * not configurable at runtime) on the same 400kHz bus as the VL53L1X.
 *
 * Configured for 1kHz output on both gyro and accel in Low Noise mode, which
 * is what the 1kHz control loop wants. Full scale is deliberately wide:
 * +/-2000 deg/s and +/-16g. A crash or a hard flick easily exceeds narrower
 * ranges, and a saturated gyro reads a plausible-but-wrong number rather than
 * failing — the attitude estimate then rotates to something arbitrary while
 * the aircraft is already in trouble.
 *
 * Axis convention delivered by this driver is the FLU body frame:
 *   +X forward (USB-C edge), +Y LEFT, +Z up (right-handed — "Y right, Z up"
 *   would be left-handed, and every quaternion identity assumes right-handed;
 *   see src/frame.h). Gyro in deg/s, accel in g.
 *
 * The raw sensor axes pass through the IMU_ORIENT_* macros in config.h on
 * the way out, because the mapping from chip axes to body axes depends on
 * how the package is mounted on the PCB. The bench 'o' test validates the
 * mapping on real hardware before any flight.
 */
#ifndef PURSUIT_ICM42688_H
#define PURSUIT_ICM42688_H

#include <stdint.h>

struct ImuSample {
    float gx, gy, gz;   /* deg/s, bias-corrected */
    float ax, ay, az;   /* g                     */
    float temp_c;
    bool  valid;
};

class Icm42688 {
public:
    Icm42688();

    /* Probe and configure. Returns false if WHO_AM_I is wrong or the bus is
     * dead — the caller must NOT arm in that case. */
    bool begin();

    /* One burst read of the 12-byte accel+gyro block. */
    ImuSample read();

    /* Average the gyro at rest to find its zero offset. Every MEMS gyro has
     * one, it is a few deg/s, and left uncorrected the attitude estimate
     * yaws steadily and the aircraft slowly rotates in hover.
     *
     * Refuses to complete if the board is moving: calibrating while someone
     * is holding it bakes that motion in as the new definition of "still".
     * Returns false if it could not get a quiet window.
     */
    bool calibrate_gyro(uint32_t samples = 1000);

    bool calibrated() const { return calibrated_; }
    void gyro_bias(float out[3]) const;

    uint8_t who_am_i() const { return whoami_; }
    const char *last_error() const { return err_; }

private:
    bool write_reg(uint8_t reg, uint8_t val);
    bool read_regs(uint8_t reg, uint8_t *buf, uint8_t len);

    float gyro_scale_;   /* LSB -> deg/s */
    float accel_scale_;  /* LSB -> g     */
    float bias_[3];
    bool  calibrated_;
    uint8_t whoami_;
    const char *err_;
};

#endif /* PURSUIT_ICM42688_H */
