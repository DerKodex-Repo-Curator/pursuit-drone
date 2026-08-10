#include "bmp390.h"

#include <cmath>

/* ---------------------------------------------------------------------------
 * Compensation. Datasheet rev 1.4 section 8.5, in float rather than the
 * fixed-point reference: the ESP32-S3 has an FPU, and the integer version's
 * intermediate scaling is the easiest thing in this file to get subtly wrong.
 *
 * No I2C in this section on purpose — it compiles and is tested on the host.
 * ------------------------------------------------------------------------ */

float bmp390_compensate_temperature(const Bmp390Raw &raw,
                                    const Bmp390Calib &c)
{
    const float d1 = (float)raw.temperature - c.t1;
    const float d2 = d1 * c.t2;
    return d2 + (d1 * d1) * c.t3;
}

float bmp390_compensate_pressure(const Bmp390Raw &raw, const Bmp390Calib &c,
                                 float t)
{
    const float t2 = t * t;
    const float t3 = t2 * t;

    /* Order matters: each partial consumes the previous one's temperature
     * powers, and the datasheet's grouping is not arbitrary. */
    float po1 = c.p5 + c.p6 * t + c.p7 * t2 + c.p8 * t3;

    float po2 = (float)raw.pressure *
                (c.p1 + c.p2 * t + c.p3 * t2 + c.p4 * t3);

    const float p = (float)raw.pressure;
    float po3 = (p * p) * (c.p9 + c.p10 * t);
    po3 += (p * p * p) * c.p11;

    return po1 + po2 + po3;
}

float bmp390_altitude_m(float pressure_pa, float sea_level_pa)
{
    if (!(pressure_pa > 0.0f) || !(sea_level_pa > 0.0f)) return 0.0f;
    /* International barometric formula. The exponent is R*L/(g*M); the
     * constants are the ISA troposphere values. */
    return 44330.0f * (1.0f - powf(pressure_pa / sea_level_pa, 0.1902949f));
}

#ifndef PURSUIT_HOST_BUILD

#include <Arduino.h>
#include <Wire.h>

#include "config.h"

namespace {

constexpr uint8_t REG_CHIP_ID   = 0x00;
constexpr uint8_t REG_ERR       = 0x02;
constexpr uint8_t REG_STATUS    = 0x03;
constexpr uint8_t REG_DATA      = 0x04;   /* p[0..2] then t[0..2], LE      */
constexpr uint8_t REG_PWR_CTRL  = 0x1B;
constexpr uint8_t REG_OSR       = 0x1C;
constexpr uint8_t REG_ODR       = 0x1D;
constexpr uint8_t REG_CONFIG    = 0x1F;
constexpr uint8_t REG_CALIB     = 0x31;   /* 21 bytes of NVM trim          */
constexpr uint8_t REG_CMD       = 0x7E;

constexpr uint8_t CHIP_ID_BMP390  = 0x60;
constexpr uint8_t CHIP_ID_BMP390B = 0x50;   /* some batches report this */
constexpr uint8_t CMD_SOFT_RESET  = 0xB6;

/* STATUS bit 5: pressure data ready. Bit 6: temperature. */
constexpr uint8_t STATUS_DRDY_PRESS = 0x20;

constexpr uint8_t ADDR_PRIMARY  = 0x77;
constexpr uint8_t ADDR_ALT      = 0x76;

inline uint32_t le24(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16);
}

}  /* namespace */

Bmp390::Bmp390()
    : calib_{}, addr_(0), chip_id_(0), present_(false), err_("")
{
}

bool Bmp390::write_reg(uint8_t reg, uint8_t val)
{
    Wire.beginTransmission(addr_);
    Wire.write(reg);
    Wire.write(val);
    return Wire.endTransmission() == 0;
}

bool Bmp390::read_regs(uint8_t reg, uint8_t *buf, uint8_t len)
{
    Wire.beginTransmission(addr_);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;    /* repeated start */
    if (Wire.requestFrom((int)addr_, (int)len) != len) return false;
    for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
    return true;
}

bool Bmp390::probe(uint8_t addr)
{
    addr_ = addr;
    uint8_t id = 0;
    if (!read_regs(REG_CHIP_ID, &id, 1)) return false;
    chip_id_ = id;
    return id == CHIP_ID_BMP390 || id == CHIP_ID_BMP390B;
}

bool Bmp390::read_calibration()
{
    uint8_t b[21];
    if (!read_regs(REG_CALIB, b, sizeof(b))) return false;

    /* NVM words, then the datasheet's fixed scaling into float. The powers of
     * two are not decoration: each coefficient has its own exponent, and using
     * one of them for its neighbour yields a pressure that looks entirely
     * plausible and is wrong by orders of magnitude. */
    const uint16_t nvm_t1 = (uint16_t)(b[1] << 8 | b[0]);
    const uint16_t nvm_t2 = (uint16_t)(b[3] << 8 | b[2]);
    const int8_t   nvm_t3 = (int8_t)b[4];
    const int16_t  nvm_p1 = (int16_t)(b[6] << 8 | b[5]);
    const int16_t  nvm_p2 = (int16_t)(b[8] << 8 | b[7]);
    const int8_t   nvm_p3 = (int8_t)b[9];
    const int8_t   nvm_p4 = (int8_t)b[10];
    const uint16_t nvm_p5 = (uint16_t)(b[12] << 8 | b[11]);
    const uint16_t nvm_p6 = (uint16_t)(b[14] << 8 | b[13]);
    const int8_t   nvm_p7 = (int8_t)b[15];
    const int8_t   nvm_p8 = (int8_t)b[16];
    const int16_t  nvm_p9 = (int16_t)(b[18] << 8 | b[17]);
    const int8_t   nvm_p10 = (int8_t)b[19];
    const int8_t   nvm_p11 = (int8_t)b[20];

    calib_.t1 = (float)nvm_t1 / 0.00390625f;          /* 2^-8   */
    calib_.t2 = (float)nvm_t2 / 1073741824.0f;        /* 2^30   */
    calib_.t3 = (float)nvm_t3 / 281474976710656.0f;   /* 2^48   */
    calib_.p1 = ((float)nvm_p1 - 16384.0f) / 1048576.0f;
    calib_.p2 = ((float)nvm_p2 - 16384.0f) / 536870912.0f;
    calib_.p3 = (float)nvm_p3 / 4294967296.0f;
    calib_.p4 = (float)nvm_p4 / 137438953472.0f;
    calib_.p5 = (float)nvm_p5 / 0.125f;               /* 2^3    */
    calib_.p6 = (float)nvm_p6 / 64.0f;
    calib_.p7 = (float)nvm_p7 / 256.0f;
    calib_.p8 = (float)nvm_p8 / 32768.0f;
    calib_.p9 = (float)nvm_p9 / 281474976710656.0f;
    calib_.p10 = (float)nvm_p10 / 281474976710656.0f;
    calib_.p11 = (float)nvm_p11 / 36893488147419103232.0f;   /* 2^65 */
    return true;
}

bool Bmp390::begin()
{
    /* The bus is already up — the IMU brought it up in its own begin(). This
     * is harmless if called twice. */
    Wire.begin(PIN_SDA, PIN_SCL, I2C_FREQ_HZ);

    if (!probe(ADDR_PRIMARY) && !probe(ADDR_ALT)) {
        err_ = "no BMP390 (checked 0x77 and 0x76)";
        present_ = false;
        return false;
    }

    write_reg(REG_CMD, CMD_SOFT_RESET);
    delay(10);                       /* datasheet: 2 ms; 10 is free */

    if (!read_calibration()) {
        err_ = "calibration read failed";
        present_ = false;
        return false;
    }

    /* x8 pressure / x1 temperature. More oversampling is quieter and slower,
     * and on an airframe that is already fighting prop wash the extra latency
     * costs more than the noise it removes. */
    if (!write_reg(REG_OSR, 0x03)) { err_ = "osr"; return false; }
    /* ODR 50 Hz — comfortably above the 30 Hz altitude loop. */
    if (!write_reg(REG_ODR, 0x02)) { err_ = "odr"; return false; }
    /* IIR coefficient 3. Light: the real filtering is in altitude_fuse, where
     * its time constant is explicit rather than buried in a register. */
    if (!write_reg(REG_CONFIG, 0x02)) { err_ = "config"; return false; }
    /* Enable both sensors, normal mode. */
    if (!write_reg(REG_PWR_CTRL, 0x33)) { err_ = "pwr"; return false; }
    delay(10);

    present_ = true;
    err_ = "";
    return true;
}

Bmp390Sample Bmp390::read()
{
    Bmp390Sample s{0.0f, 0.0f, 0.0f, false};
    if (!present_) return s;

    uint8_t st = 0;
    if (!read_regs(REG_STATUS, &st, 1)) { err_ = "status"; return s; }
    if (!(st & STATUS_DRDY_PRESS)) return s;    /* nothing new — not an error */

    uint8_t b[6];
    if (!read_regs(REG_DATA, b, sizeof(b))) { err_ = "data"; return s; }

    Bmp390Raw raw;
    raw.pressure = le24(&b[0]);
    raw.temperature = le24(&b[3]);

    /* Temperature first: the pressure compensation consumes its result. */
    s.temperature_c = bmp390_compensate_temperature(raw, calib_);
    s.pressure_pa = bmp390_compensate_pressure(raw, calib_, s.temperature_c);

    /* A sanity band rather than blind trust. 30-110 kPa spans sea level to
     * roughly 9 km; anything outside it is a bus glitch or a half-read
     * register, and acting on it would command a climb. */
    if (!(s.pressure_pa > 30000.0f && s.pressure_pa < 110000.0f)) {
        err_ = "implausible pressure";
        return s;
    }

    s.altitude_m = bmp390_altitude_m(s.pressure_pa);
    s.valid = true;
    err_ = "";
    return s;
}

#else  /* PURSUIT_HOST_BUILD — no I2C on the host */

Bmp390::Bmp390()
    : calib_{}, addr_(0), chip_id_(0), present_(false), err_("host") {}
bool Bmp390::begin() { return false; }
Bmp390Sample Bmp390::read() { return Bmp390Sample{0.0f, 0.0f, 0.0f, false}; }
bool Bmp390::write_reg(uint8_t, uint8_t) { return false; }
bool Bmp390::read_regs(uint8_t, uint8_t *, uint8_t) { return false; }
bool Bmp390::probe(uint8_t) { return false; }
bool Bmp390::read_calibration() { return false; }

#endif
