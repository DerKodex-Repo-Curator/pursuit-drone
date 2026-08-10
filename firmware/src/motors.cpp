#include "motors.h"

#ifndef PURSUIT_HOST_BUILD

#include <Arduino.h>
#include <driver/ledc.h>

#include "config.h"

namespace {

constexpr ledc_mode_t   MODE  = LEDC_LOW_SPEED_MODE;
constexpr ledc_timer_t  TIMER = LEDC_TIMER_0;

struct Chan {
    int gpio;
    ledc_channel_t ch;
    uint32_t hpoint;
};

/* hpoint is where in the PWM period this channel's pulse STARTS, in timer
 * ticks. Spacing the four starts a quarter-period apart is the whole trick:
 * the gates no longer all switch on the same edge, so the pack sees four
 * separate ~2.75A steps instead of one ~11A step. */
Chan g_chan[4] = {
    {PIN_M1_PWM, LEDC_CHANNEL_0, 0},
    {PIN_M2_PWM, LEDC_CHANNEL_1, (MOTOR_PWM_MAX + 1) / 4},
    {PIN_M3_PWM, LEDC_CHANNEL_2, (MOTOR_PWM_MAX + 1) / 2},
    {PIN_M4_PWM, LEDC_CHANNEL_3, 3 * (MOTOR_PWM_MAX + 1) / 4},
};

bool g_ready = false;

void write_one(const Chan &c, float duty)
{
    const uint32_t d = duty_to_pwm(duty, MOTOR_PWM_MAX);
    /* set_duty_with_hpoint, not set_duty: the plain call resets hpoint to 0
     * and silently un-staggers the channel on the first write. */
    ledc_set_duty_with_hpoint(MODE, c.ch, d, c.hpoint);
    ledc_update_duty(MODE, c.ch);
}

}  /* namespace */

void motors_begin(void)
{
    /* Drive the gates low before the peripheral is configured. The 1k
     * pulldowns (R5-R8) already hold them there through reset, but the pins
     * must not float during the window where LEDC is half-initialised. */
    for (const Chan &c : g_chan) {
        pinMode(c.gpio, OUTPUT);
        digitalWrite(c.gpio, LOW);
    }

    ledc_timer_config_t timer = {};
    timer.speed_mode = MODE;
    timer.duty_resolution = (ledc_timer_bit_t)MOTOR_PWM_BITS;
    timer.timer_num = TIMER;
    timer.freq_hz = MOTOR_PWM_FREQ_HZ;
    timer.clk_cfg = LEDC_AUTO_CLK;
    ledc_timer_config(&timer);

    for (const Chan &c : g_chan) {
        ledc_channel_config_t ch = {};
        ch.gpio_num = c.gpio;
        ch.speed_mode = MODE;
        ch.channel = c.ch;
        ch.intr_type = LEDC_INTR_DISABLE;
        ch.timer_sel = TIMER;
        ch.duty = 0;
        ch.hpoint = c.hpoint;
        ledc_channel_config(&ch);
    }

    g_ready = true;
    motors_stop();
}

void motors_write(const MotorOutputs &m)
{
    if (!g_ready) return;
    write_one(g_chan[0], m.m1);
    write_one(g_chan[1], m.m2);
    write_one(g_chan[2], m.m3);
    write_one(g_chan[3], m.m4);
}

void motors_stop(void)
{
    if (!g_ready) {
        for (const Chan &c : g_chan) {
            pinMode(c.gpio, OUTPUT);
            digitalWrite(c.gpio, LOW);
        }
        return;
    }
    for (const Chan &c : g_chan) write_one(c, 0.0f);
}

#else  /* PURSUIT_HOST_BUILD — no LEDC on the host */

void motors_begin(void) {}
void motors_write(const MotorOutputs &) {}
void motors_stop(void) {}

#endif
