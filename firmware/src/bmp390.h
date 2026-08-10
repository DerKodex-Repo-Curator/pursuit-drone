/* BMP390 barometric pressure sensor.
 *
 * Fitted to lift the altitude-hold ceiling off the VL53L1X's 3.5 m. It shares
 * the existing I2C bus via J2 — the rangefinder header already brings out
 * +3V3, GND, SDA and SCL, so nothing on the board changes. Addresses do not
 * collide: IMU 0x68, ToF 0x29, BMP390 0x77 (0x76 if SDO is pulled low).
 *
 * PUT FOAM OVER THE PORT. Not tape — open-cell foam. A barometer on a small
 * quadcopter is ruined by two things, and both are mechanical rather than
 * electrical: prop wash across the port reads as tens of metres of noise, and
 * light falling on the MEMS die shifts the reading outright. This is the
 * single most common reason people conclude barometers do not work at this
 * scale.
 *
 * The chip does not report altitude. It reports raw ADC counts for pressure
 * and temperature, plus a block of factory trim coefficients in its NVM, and
 * the compensation arithmetic in the datasheet (rev 1.4, section 8.5) has to
 * be done in the right order and the right integer widths. Temperature is
 * compensated FIRST because the pressure compensation consumes its result.
 */
#ifndef PURSUIT_BMP390_H
#define PURSUIT_BMP390_H

#include <stdint.h>

/* Factory trim, read once from NVM at boot and converted to the float form the
 * compensation uses. Kept separate from the driver so the arithmetic can be
 * exercised on the host with known-good coefficients. */
struct Bmp390Calib {
    float t1, t2, t3;
    float p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11;
};

/* Raw 24-bit ADC words as they come off the wire. */
struct Bmp390Raw {
    uint32_t pressure;
    uint32_t temperature;
};

/* Compensation, per the datasheet. Pure arithmetic, no I2C — so it is host
 * testable, which matters because getting the coefficient scaling wrong gives
 * a plausible-looking pressure that is wrong by a constant factor, and a
 * constant factor in pressure is a constant factor in altitude. */
float bmp390_compensate_temperature(const Bmp390Raw &raw,
                                    const Bmp390Calib &c);
float bmp390_compensate_pressure(const Bmp390Raw &raw, const Bmp390Calib &c,
                                 float temperature_c);

/* Pressure (Pa) -> altitude (m) by the international barometric formula.
 *
 * `sea_level_pa` is only a datum. The fusion in altitude_fuse.h subtracts its
 * own reference anyway, so this being wrong shifts every reading equally and
 * costs nothing — which is exactly why height is measured from the arming
 * point rather than from this number.
 */
float bmp390_altitude_m(float pressure_pa, float sea_level_pa = 101325.0f);

struct Bmp390Sample {
    float pressure_pa;
    float temperature_c;
    float altitude_m;
    bool valid;
};

class Bmp390 {
public:
    Bmp390();

    /* Probes both addresses, checks CHIP_ID, soft-resets, reads the trim and
     * configures oversampling. False if the part is absent — which is not
     * fatal anywhere: without it the aircraft flies on the rangefinder exactly
     * as it does now. */
    bool begin();

    /* Non-blocking. Returns valid=false when no new conversion is ready; the
     * sensor runs near 50 Hz and the control loop at 1 kHz, so most calls
     * legitimately have nothing. */
    Bmp390Sample read();

    bool present() const { return present_; }
    uint8_t chip_id() const { return chip_id_; }
    uint8_t address() const { return addr_; }
    const char *last_error() const { return err_; }
    const Bmp390Calib &calibration() const { return calib_; }

private:
    Bmp390Calib calib_;
    uint8_t addr_;
    uint8_t chip_id_;
    bool present_;
    const char *err_;

    bool write_reg(uint8_t reg, uint8_t val);
    bool read_regs(uint8_t reg, uint8_t *buf, uint8_t len);
    bool probe(uint8_t addr);
    bool read_calibration();
};

#endif /* PURSUIT_BMP390_H */
