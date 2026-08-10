#include "icm42688.h"

#ifndef PURSUIT_HOST_BUILD

#include <Arduino.h>
#include <Wire.h>
#include <cmath>

#include "config.h"

namespace {

/* Bank 0 registers, from the ICM-42688-P datasheet rev 1.7 section 14. */
constexpr uint8_t REG_DEVICE_CONFIG   = 0x11;
constexpr uint8_t REG_INT_CONFIG      = 0x14;
constexpr uint8_t REG_TEMP_DATA1      = 0x1D;
constexpr uint8_t REG_ACCEL_DATA_X1   = 0x1F;
constexpr uint8_t REG_PWR_MGMT0       = 0x4E;
constexpr uint8_t REG_GYRO_CONFIG0    = 0x4F;
constexpr uint8_t REG_ACCEL_CONFIG0   = 0x50;
constexpr uint8_t REG_GYRO_ACCEL_CFG0 = 0x52;
constexpr uint8_t REG_WHO_AM_I        = 0x75;
constexpr uint8_t REG_BANK_SEL        = 0x76;

constexpr uint8_t WHOAMI_EXPECTED = 0x47;

/* GYRO_CONFIG0: FS_SEL in [7:5], ODR in [3:0].
 * FS 0 = +/-2000 dps, ODR 0x06 = 1kHz. */
constexpr uint8_t GYRO_CFG  = (0u << 5) | 0x06;
/* ACCEL_CONFIG0: FS 0 = +/-16g, ODR 0x06 = 1kHz. */
constexpr uint8_t ACCEL_CFG = (0u << 5) | 0x06;

/* PWR_MGMT0: gyro [3:2] = 11 (low noise), accel [1:0] = 11 (low noise). */
constexpr uint8_t PWR_LOW_NOISE = 0x0F;

/* GYRO_ACCEL_CONFIG0: anti-alias/notch bandwidth selectors, accel [7:4],
 * gyro [3:0]. 0 = ODR/2, the widest available — the control loop wants the
 * least phase lag it can get, and filtering belongs in the D term where its
 * cutoff is explicit rather than buried in a sensor register. */
constexpr uint8_t FILTER_CFG = 0x00;

int16_t be16(const uint8_t *p)
{
    /* The ICM-42688-P is BIG-endian on the wire, unlike almost everything
     * else on this board. Reading it little-endian gives values that look
     * like plausible noise, which is worse than garbage. */
    return (int16_t)(((uint16_t)p[0] << 8) | p[1]);
}

}  /* namespace */

Icm42688::Icm42688()
    : gyro_scale_(2000.0f / 32768.0f),
      accel_scale_(16.0f / 32768.0f),
      bias_{0.0f, 0.0f, 0.0f},
      calibrated_(false), whoami_(0), err_("")
{
}

bool Icm42688::write_reg(uint8_t reg, uint8_t val)
{
    Wire.beginTransmission(I2C_ADDR_IMU);
    Wire.write(reg);
    Wire.write(val);
    return Wire.endTransmission() == 0;
}

bool Icm42688::read_regs(uint8_t reg, uint8_t *buf, uint8_t len)
{
    Wire.beginTransmission(I2C_ADDR_IMU);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;   /* repeated start */
    if (Wire.requestFrom((int)I2C_ADDR_IMU, (int)len) != len) return false;
    for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
    return true;
}

bool Icm42688::begin()
{
    Wire.begin(PIN_SDA, PIN_SCL, I2C_FREQ_HZ);

    if (!write_reg(REG_BANK_SEL, 0)) { err_ = "bus dead"; return false; }

    /* Soft reset, then wait. The datasheet requires 1ms before any further
     * register access; anything sooner is silently ignored and leaves the
     * device half-configured. */
    write_reg(REG_DEVICE_CONFIG, 0x01);
    delay(5);
    write_reg(REG_BANK_SEL, 0);

    if (!read_regs(REG_WHO_AM_I, &whoami_, 1)) {
        err_ = "no response";
        return false;
    }
    if (whoami_ != WHOAMI_EXPECTED) {
        /* Wrong part, wrong address, or a bus with no pull-ups. Do not
         * proceed: an aircraft armed on a sensor that is not there is worse
         * than one that refuses to arm. */
        err_ = "bad WHO_AM_I";
        return false;
    }

    if (!write_reg(REG_GYRO_CONFIG0, GYRO_CFG))   { err_ = "gyro cfg";  return false; }
    if (!write_reg(REG_ACCEL_CONFIG0, ACCEL_CFG)) { err_ = "accel cfg"; return false; }
    if (!write_reg(REG_GYRO_ACCEL_CFG0, FILTER_CFG)) { err_ = "filt cfg"; return false; }

    /* Power up last, so the sensors start already configured. */
    if (!write_reg(REG_PWR_MGMT0, PWR_LOW_NOISE)) { err_ = "pwr"; return false; }
    /* Datasheet: no register writes for 200us after changing PWR_MGMT0, and
     * the gyro needs ~45ms to become stable after leaving OFF. Reading before
     * then returns data that looks valid and is not. */
    delay(60);

    /* INT1 push-pull, active high, pulsed — the pin is wired to GPIO8 but the
     * loop currently polls. Configured anyway so the line is driven rather
     * than floating next to a 400kHz bus. */
    write_reg(REG_INT_CONFIG, 0x03);

    gyro_scale_ = 2000.0f / 32768.0f;
    accel_scale_ = 16.0f / 32768.0f;
    err_ = "";
    return true;
}

ImuSample Icm42688::read()
{
    ImuSample s{};
    uint8_t buf[14];

    /* One burst from TEMP through gyro Z: 14 bytes in a single transaction,
     * so every axis is from the same sample instant. Reading axes in separate
     * transactions lets the device update between them and shears the
     * attitude estimate. */
    if (!read_regs(REG_TEMP_DATA1, buf, sizeof(buf))) {
        s.valid = false;
        return s;
    }

    const int16_t traw = be16(&buf[0]);
    const int16_t ax = be16(&buf[2]);
    const int16_t ay = be16(&buf[4]);
    const int16_t az = be16(&buf[6]);
    const int16_t gx = be16(&buf[8]);
    const int16_t gy = be16(&buf[10]);
    const int16_t gz = be16(&buf[12]);

    s.temp_c = (float)traw / 132.48f + 25.0f;

    /* Chip axes -> FLU body axes, via the config.h mapping. Bias is applied
     * AFTER the remap so it is a body-frame quantity — recalibrating after a
     * mapping change therefore starts clean instead of subtracting an offset
     * measured in a different frame. */
    const float rax = (float)ax * accel_scale_;
    const float ray = (float)ay * accel_scale_;
    const float raz = (float)az * accel_scale_;
    const float rgx = (float)gx * gyro_scale_;
    const float rgy = (float)gy * gyro_scale_;
    const float rgz = (float)gz * gyro_scale_;

    s.ax = IMU_ORIENT_X(rax, ray, raz);
    s.ay = IMU_ORIENT_Y(rax, ray, raz);
    s.az = IMU_ORIENT_Z(rax, ray, raz);
    s.gx = IMU_ORIENT_X(rgx, rgy, rgz) - bias_[0];
    s.gy = IMU_ORIENT_Y(rgx, rgy, rgz) - bias_[1];
    s.gz = IMU_ORIENT_Z(rgx, rgy, rgz) - bias_[2];
    s.valid = true;
    return s;
}

bool Icm42688::calibrate_gyro(uint32_t samples)
{
    double sum[3] = {0.0, 0.0, 0.0};
    float peak[3] = {0.0f, 0.0f, 0.0f};
    uint32_t got = 0;

    bias_[0] = bias_[1] = bias_[2] = 0.0f;   /* measure raw */

    for (uint32_t i = 0; i < samples; i++) {
        ImuSample s = read();
        if (!s.valid) { delay(1); continue; }
        sum[0] += s.gx; sum[1] += s.gy; sum[2] += s.gz;
        peak[0] = fmaxf(peak[0], fabsf(s.gx));
        peak[1] = fmaxf(peak[1], fabsf(s.gy));
        peak[2] = fmaxf(peak[2], fabsf(s.gz));
        got++;
        delay(1);
    }

    if (got < samples / 2) { err_ = "calib: too few samples"; return false; }

    /* Reject the window if the board moved. A gyro bias is a fraction of a
     * degree per second; anything above a few deg/s peak is someone holding
     * the aircraft, and averaging that in teaches it that "level" is
     * whatever angle it was being held at. */
    const float MOVE_LIMIT_DPS = 5.0f;
    if (peak[0] > MOVE_LIMIT_DPS || peak[1] > MOVE_LIMIT_DPS ||
        peak[2] > MOVE_LIMIT_DPS) {
        err_ = "calib: board moved — put it down and reset";
        return false;
    }

    bias_[0] = (float)(sum[0] / got);
    bias_[1] = (float)(sum[1] / got);
    bias_[2] = (float)(sum[2] / got);
    calibrated_ = true;
    err_ = "";
    return true;
}

void Icm42688::gyro_bias(float out[3]) const
{
    out[0] = bias_[0]; out[1] = bias_[1]; out[2] = bias_[2];
}

#else  /* PURSUIT_HOST_BUILD — no I2C on the host */

Icm42688::Icm42688()
    : gyro_scale_(2000.0f / 32768.0f), accel_scale_(16.0f / 32768.0f),
      bias_{0.0f, 0.0f, 0.0f}, calibrated_(false), whoami_(0), err_("host") {}
bool Icm42688::begin() { return false; }
ImuSample Icm42688::read()
{
    /* Scales referenced so the host build does not warn about members that
     * only the target branch uses. */
    ImuSample s{};
    s.gx = s.gy = s.gz = 0.0f * gyro_scale_;
    s.ax = s.ay = s.az = 0.0f * accel_scale_;
    return s;
}
bool Icm42688::calibrate_gyro(uint32_t) { return false; }
void Icm42688::gyro_bias(float out[3]) const
{
    out[0] = bias_[0]; out[1] = bias_[1]; out[2] = bias_[2];
}
bool Icm42688::write_reg(uint8_t, uint8_t) { return false; }
bool Icm42688::read_regs(uint8_t, uint8_t *, uint8_t) { return false; }

#endif
