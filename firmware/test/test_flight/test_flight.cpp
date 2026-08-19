/* Host-side tests for the flight-critical logic.
 *
 * Everything here runs on the laptop with no board attached, because these
 * are the behaviours you cannot safely discover with props fitted: which way
 * a motor spins, whether a stale command gets re-applied, whether the
 * autonomy can arm itself.
 *
 *   pio test -e native
 */
#include <unity.h>

#include <cmath>
#include <cstring>

#include "battery.h"
#include "config.h"
#include "flight_state.h"
#include "frame.h"
#include "mixer.h"
#include "pursuit_link.h"

/* ------------------------------------------------------------------ utils */

static void fill_cmd(pursuit_cmd_t *c, uint32_t seq, float roll, float pitch,
                     float yaw, float alt, uint8_t flags)
{
    std::memset(c, 0, sizeof(*c));
    c->magic = PURSUIT_CMD_MAGIC;
    c->version = PURSUIT_PROTOCOL_VERSION;
    c->flags = flags;
    c->seq = seq;
    c->t_ms = seq * 33;
    c->roll = roll;
    c->pitch = pitch;
    c->yaw_rate = yaw;
    c->alt_sp = alt;
    c->crc = pursuit_crc16((const uint8_t *)c, sizeof(*c) - 2);
}

static const uint8_t ARMED_HOLD = PURSUIT_FLAG_ARMED | PURSUIT_FLAG_ALT_HOLD;

/* ------------------------------------------------------------------ mixer */

void test_hover_is_symmetric(void)
{
    MotorOutputs m = mix_quad_x(0, 0, 0, 0.5f);
    TEST_ASSERT_FLOAT_WITHIN(1e-5, 0.5f, m.m1);
    TEST_ASSERT_FLOAT_WITHIN(1e-5, 0.5f, m.m2);
    TEST_ASSERT_FLOAT_WITHIN(1e-5, 0.5f, m.m3);
    TEST_ASSERT_FLOAT_WITHIN(1e-5, 0.5f, m.m4);
}

void test_roll_right_lifts_the_left_side(void)
{
    MotorOutputs m = mix_quad_x(0.2f, 0, 0, 0.5f);
    TEST_ASSERT_TRUE(m.m1 > m.m2);   /* front-left  > front-right */
    TEST_ASSERT_TRUE(m.m3 > m.m4);   /* rear-left   > rear-right  */
}

void test_pitch_forward_lifts_the_back(void)
{
    MotorOutputs m = mix_quad_x(0, 0.2f, 0, 0.5f);
    TEST_ASSERT_TRUE(m.m1 > m.m3);   /* front > rear ... */
    TEST_ASSERT_TRUE(m.m2 > m.m4);
}

void test_yaw_right_speeds_up_the_ccw_pair(void)
{
    /* A motor's reaction torque opposes its own rotation, so to yaw the
     * airframe clockwise you speed up the COUNTER-clockwise motors. This is
     * the row everyone gets backwards. */
    MotorOutputs m = mix_quad_x(0, 0, 0.2f, 0.5f);
    TEST_ASSERT_TRUE(m.m2 > m.m1);   /* M2 CCW up, M1 CW down */
    TEST_ASSERT_TRUE(m.m3 > m.m4);   /* M3 CCW up, M4 CW down */
}

void test_saturation_preserves_attitude_authority(void)
{
    /* Full throttle AND a roll demand. Naive clipping would flatten the
     * differential and the aircraft would stop rolling exactly when it is
     * working hardest; the common-mode shift must keep it. */
    MotorOutputs m = mix_quad_x(0.3f, 0, 0, 1.0f);
    TEST_ASSERT_TRUE(m.m1 > m.m2);
    TEST_ASSERT_FLOAT_WITHIN(1e-4, 0.6f, m.m1 - m.m2);
}

void test_outputs_stay_in_range(void)
{
    const float vals[] = {-5.0f, -1.0f, 0.0f, 0.5f, 1.0f, 5.0f};
    for (float r : vals)
        for (float p : vals)
            for (float y : vals)
                for (float t : vals) {
                    MotorOutputs m = mix_quad_x(r, p, y, t);
                    TEST_ASSERT_TRUE(m.m1 >= 0.0f && m.m1 <= 1.0f);
                    TEST_ASSERT_TRUE(m.m2 >= 0.0f && m.m2 <= 1.0f);
                    TEST_ASSERT_TRUE(m.m3 >= 0.0f && m.m3 <= 1.0f);
                    TEST_ASSERT_TRUE(m.m4 >= 0.0f && m.m4 <= 1.0f);
                }
}

void test_nan_never_reaches_a_motor(void)
{
    const float nan_v = std::nan("");
    MotorOutputs m = mix_quad_x(nan_v, nan_v, nan_v, nan_v);
    TEST_ASSERT_FALSE(std::isnan(m.m1));
    TEST_ASSERT_FALSE(std::isnan(m.m2));
    TEST_ASSERT_FALSE(std::isnan(m.m3));
    TEST_ASSERT_FALSE(std::isnan(m.m4));
    TEST_ASSERT_EQUAL_UINT32(0, duty_to_pwm(nan_v, MOTOR_PWM_MAX));
}

void test_disarmed_means_stopped(void)
{
    MotorOutputs m{0.8f, 0.8f, 0.8f, 0.8f};
    MotorOutputs off = apply_idle_floor(m, 0.06f, false);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, off.m1);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, off.m2);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, off.m3);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, off.m4);
}

void test_idle_floor_keeps_control_alive(void)
{
    MotorOutputs m{0.0f, 0.0f, 0.0f, 0.0f};
    MotorOutputs on = apply_idle_floor(m, 0.06f, true);
    TEST_ASSERT_FLOAT_WITHIN(1e-5, 0.06f, on.m1);
    TEST_ASSERT_FLOAT_WITHIN(1e-5, 0.06f, on.m4);
}

/* ------------------------------------------------------------ state machine */

void test_stays_in_boot_until_sensors_pass(void)
{
    FlightState fs;
    pursuit_cmd_t c;
    fill_cmd(&c, 1, 0, 0, 0, 0.5f, ARMED_HOLD);
    TEST_ASSERT_TRUE(fs.on_packet((uint8_t *)&c, sizeof(c), 100));
    TEST_ASSERT_EQUAL_UINT8(PURSUIT_STATE_BOOT, fs.state());
    TEST_ASSERT_FALSE(fs.motors_enabled());
}

void test_a_fault_cannot_promote_boot_to_idle(void)
{
    /* Found on hardware. A bare XIAO has no battery divider, so VBAT reads
     * 0.00V, the limiter latches critical, and main.cpp calls
     * emergency_land() at 50Hz. That used to fall through to enter(IDLE) and
     * the aircraft reported IDLE over telemetry while its own boot banner
     * insisted it was held in BOOT.
     *
     * Arming stayed correctly refused — that gate is sensors_ok_, which is
     * independent of the state — but IDLE reads to every downstream consumer
     * as "the sensors passed". A FAULT must never move the aircraft forward
     * through the state machine. */
    FlightState fs;
    TEST_ASSERT_EQUAL_UINT8(PURSUIT_STATE_BOOT, fs.state());
    for (int i = 0; i < 50; i++) fs.emergency_land();
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(PURSUIT_STATE_BOOT, fs.state(),
        "a fault promoted the aircraft out of its sensor gate");
    TEST_ASSERT_FALSE(fs.motors_enabled());

    /* And it must still be unarmable afterwards. */
    pursuit_cmd_t c;
    fill_cmd(&c, 1, 0, 0, 0, 0.5f, ARMED_HOLD);
    fs.on_packet((uint8_t *)&c, sizeof(c), 100);
    TEST_ASSERT_EQUAL_UINT8(PURSUIT_STATE_BOOT, fs.state());
}

void test_emergency_land_still_works_once_sensors_have_passed(void)
{
    /* The BOOT guard must not have disarmed the actual failsafe. */
    FlightState fs;
    fs.sensors_ready();
    TEST_ASSERT_EQUAL_UINT8(PURSUIT_STATE_IDLE, fs.state());

    pursuit_cmd_t c;
    fill_cmd(&c, 1, 0, 0, 0, 0.5f, ARMED_HOLD);
    fs.on_packet((uint8_t *)&c, sizeof(c), 100);
    fill_cmd(&c, 2, 0, 0, 0, 0.5f, ARMED_HOLD);
    fs.on_packet((uint8_t *)&c, sizeof(c), 130);
    TEST_ASSERT_EQUAL_UINT8(PURSUIT_STATE_FLYING, fs.state());

    fs.emergency_land();
    TEST_ASSERT_EQUAL_UINT8(PURSUIT_STATE_LANDING, fs.state());
}

void test_arming_requires_the_flag_on_every_packet(void)
{
    FlightState fs;
    fs.sensors_ready();

    pursuit_cmd_t c;
    fill_cmd(&c, 1, 0, 0, 0, 0.0f, ARMED_HOLD);
    fs.on_packet((uint8_t *)&c, sizeof(c), 100);
    TEST_ASSERT_EQUAL_UINT8(PURSUIT_STATE_ARMED, fs.state());

    /* Flag dropped on the very next packet: authority is withdrawn at once.
     * A latch here would mean one packet licences unlimited flight. */
    fill_cmd(&c, 2, 0, 0, 0, 0.0f, PURSUIT_FLAG_ALT_HOLD);
    fs.on_packet((uint8_t *)&c, sizeof(c), 133);
    TEST_ASSERT_EQUAL_UINT8(PURSUIT_STATE_IDLE, fs.state());
    TEST_ASSERT_FALSE(fs.motors_enabled());
}

void test_corrupt_packets_are_discarded(void)
{
    FlightState fs;
    fs.sensors_ready();

    pursuit_cmd_t c;
    fill_cmd(&c, 1, 5.0f, 5.0f, 90.0f, 1.0f, ARMED_HOLD);
    uint8_t buf[sizeof(c)];
    std::memcpy(buf, &c, sizeof(c));
    buf[10] ^= 0x08;                       /* flip a bit in the payload */

    TEST_ASSERT_FALSE(fs.on_packet(buf, sizeof(buf), 100));
    TEST_ASSERT_EQUAL_UINT32(0, fs.accepted());
    TEST_ASSERT_EQUAL_UINT32(1, fs.rejected());
    TEST_ASSERT_EQUAL_UINT8(PURSUIT_STATE_IDLE, fs.state());
}

void test_replayed_packets_are_ignored(void)
{
    FlightState fs;
    fs.sensors_ready();

    pursuit_cmd_t c;
    fill_cmd(&c, 10, 0, 0, 0, 0.5f, ARMED_HOLD);
    TEST_ASSERT_TRUE(fs.on_packet((uint8_t *)&c, sizeof(c), 100));
    fill_cmd(&c, 11, 0, 0, 0, 0.5f, ARMED_HOLD);
    TEST_ASSERT_TRUE(fs.on_packet((uint8_t *)&c, sizeof(c), 133));

    fill_cmd(&c, 10, 0, 0, 20.0f, 0.5f, ARMED_HOLD);   /* replay */
    TEST_ASSERT_FALSE(fs.on_packet((uint8_t *)&c, sizeof(c), 166));
    TEST_ASSERT_EQUAL_UINT32(1, fs.stale());
    TEST_ASSERT_EQUAL_FLOAT(0.0f, fs.setpoint().yaw_rate_dps);
}

void test_watchdog_levels_off_then_lands(void)
{
    FlightState fs;
    fs.sensors_ready();

    pursuit_cmd_t c;
    fill_cmd(&c, 1, 0, 0, 0, 0.5f, ARMED_HOLD);
    fs.on_packet((uint8_t *)&c, sizeof(c), 1000);
    fill_cmd(&c, 2, 10.0f, 0, 90.0f, 1.0f, ARMED_HOLD);
    fs.on_packet((uint8_t *)&c, sizeof(c), 1033);
    TEST_ASSERT_EQUAL_UINT8(PURSUIT_STATE_FLYING, fs.state());

    fs.tick(1200);                                  /* still inside timeout */
    TEST_ASSERT_EQUAL_UINT8(PURSUIT_STATE_FLYING, fs.state());

    fs.tick(1600);                                  /* past LINK_TIMEOUT_MS */
    TEST_ASSERT_EQUAL_UINT8(PURSUIT_STATE_FAILSAFE, fs.state());

    /* The critical property: the last command is NOT held. A stale 90 deg/s
     * yaw re-applied at loop rate flies a constant circle into a wall. */
    TEST_ASSERT_EQUAL_FLOAT(0.0f, fs.setpoint().yaw_rate_dps);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, fs.setpoint().roll_deg);

    fs.tick(5000);                                  /* long silence */
    TEST_ASSERT_EQUAL_UINT8(PURSUIT_STATE_LANDING, fs.state());
}

void test_failsafe_flag_holds_but_does_not_land(void)
{
    /* Laptop alive, target lost. Different from silence, and must behave
     * differently — otherwise the drone lands whenever someone steps behind
     * a sofa. */
    FlightState fs;
    fs.sensors_ready();

    pursuit_cmd_t c;
    fill_cmd(&c, 1, 0, 0, 0, 1.0f, ARMED_HOLD);
    fs.on_packet((uint8_t *)&c, sizeof(c), 1000);
    fill_cmd(&c, 2, 5.0f, 5.0f, 45.0f, 1.0f, ARMED_HOLD);
    fs.on_packet((uint8_t *)&c, sizeof(c), 1033);
    TEST_ASSERT_EQUAL_UINT8(PURSUIT_STATE_FLYING, fs.state());

    fill_cmd(&c, 3, 0, 0, 0, 1.0f, ARMED_HOLD | PURSUIT_FLAG_FAILSAFE);
    fs.on_packet((uint8_t *)&c, sizeof(c), 1066);
    TEST_ASSERT_EQUAL_UINT8(PURSUIT_STATE_FAILSAFE, fs.state());
    TEST_ASSERT_TRUE(fs.motors_enabled());          /* still flying */
    TEST_ASSERT_EQUAL_FLOAT(0.0f, fs.setpoint().roll_deg);

    fill_cmd(&c, 4, 3.0f, 0, 0, 1.0f, ARMED_HOLD);  /* target reacquired */
    fs.on_packet((uint8_t *)&c, sizeof(c), 1099);
    TEST_ASSERT_EQUAL_UINT8(PURSUIT_STATE_FLYING, fs.state());
}

void test_setpoints_are_clamped_to_flight_limits(void)
{
    FlightState fs;
    fs.sensors_ready();
    pursuit_cmd_t c;
    fill_cmd(&c, 1, 900.0f, -900.0f, 9000.0f, 0.5f, ARMED_HOLD);
    fs.on_packet((uint8_t *)&c, sizeof(c), 100);
    TEST_ASSERT_FLOAT_WITHIN(1e-4, MAX_ANGLE_DEG, fs.setpoint().roll_deg);
    TEST_ASSERT_FLOAT_WITHIN(1e-4, -MAX_ANGLE_DEG, fs.setpoint().pitch_deg);
    TEST_ASSERT_FLOAT_WITHIN(1e-4, MAX_YAW_RATE_DPS,
                             fs.setpoint().yaw_rate_dps);
}

void test_disarming_in_flight_lands_rather_than_dropping(void)
{
    FlightState fs;
    fs.sensors_ready();
    pursuit_cmd_t c;
    fill_cmd(&c, 1, 0, 0, 0, 1.0f, ARMED_HOLD);
    fs.on_packet((uint8_t *)&c, sizeof(c), 1000);
    fill_cmd(&c, 2, 0, 0, 0, 1.0f, ARMED_HOLD);
    fs.on_packet((uint8_t *)&c, sizeof(c), 1033);
    TEST_ASSERT_EQUAL_UINT8(PURSUIT_STATE_FLYING, fs.state());

    fill_cmd(&c, 3, 0, 0, 0, 1.0f, PURSUIT_FLAG_ALT_HOLD);
    fs.on_packet((uint8_t *)&c, sizeof(c), 1066);
    TEST_ASSERT_EQUAL_UINT8(PURSUIT_STATE_LANDING, fs.state());
}

/* --- LANDING is sticky, and LANDING ends ---------------------------------
 *
 * These three pin the 2026-08-19 fixes. Both failures were in the same state
 * and pulled in opposite directions: a repeated fault cut the descent short
 * after one tick, and nothing ever ended the descent at all. */

void test_repeated_emergency_land_stays_in_landing(void)
{
    /* A flat pack latches critical, so main.cpp calls emergency_land() on
     * every battery tick — 50Hz, forever. Before the fix the second call fell
     * through to the else branch and drove LANDING -> IDLE, cutting the
     * motors after a single 20 ms tick of controlled descent. */
    FlightState fs;
    fs.sensors_ready();
    pursuit_cmd_t c;
    fill_cmd(&c, 1, 0, 0, 0, 1.0f, ARMED_HOLD);
    fs.on_packet((uint8_t *)&c, sizeof(c), 1000);
    fill_cmd(&c, 2, 0, 0, 0, 1.0f, ARMED_HOLD);
    fs.on_packet((uint8_t *)&c, sizeof(c), 1033);
    TEST_ASSERT_EQUAL_UINT8(PURSUIT_STATE_FLYING, fs.state());

    fs.emergency_land();
    TEST_ASSERT_EQUAL_UINT8(PURSUIT_STATE_LANDING, fs.state());

    /* 50 more calls — one second of a latched fault at the battery tick
     * rate. The descent must survive all of them. */
    for (int i = 0; i < 50; i++) {
        fs.emergency_land();
        TEST_ASSERT_EQUAL_UINT8(PURSUIT_STATE_LANDING, fs.state());
        TEST_ASSERT_TRUE(fs.motors_enabled());
    }
}

void test_landing_times_out_to_disarmed_but_not_early(void)
{
    /* LANDING was terminal: disarm() had no callers, and there was no
     * timeout, so the aircraft descended and then sat on the ground with the
     * props turning until the pack was flat. */
    FlightState fs;
    fs.sensors_ready();
    pursuit_cmd_t c;
    fill_cmd(&c, 1, 0, 0, 0, 1.0f, ARMED_HOLD);
    fs.on_packet((uint8_t *)&c, sizeof(c), 1000);
    fill_cmd(&c, 2, 0, 0, 0, 1.0f, ARMED_HOLD);
    fs.on_packet((uint8_t *)&c, sizeof(c), 1033);
    TEST_ASSERT_EQUAL_UINT8(PURSUIT_STATE_FLYING, fs.state());

    fs.emergency_land();
    TEST_ASSERT_EQUAL_UINT8(PURSUIT_STATE_LANDING, fs.state());

    /* Descent must actually get to run. Cutting early is the fall this whole
     * state exists to avoid, so the interesting half of this test is the
     * half that asserts NOTHING happens. */
    for (uint32_t t = 1066; t < 1066 + 1900; t += 10) {
        fs.tick(t);
        TEST_ASSERT_EQUAL_UINT8(PURSUIT_STATE_LANDING, fs.state());
        TEST_ASSERT_TRUE(fs.motors_enabled());
    }

    /* ...and then it must end. */
    for (uint32_t t = 1066 + 1900; t <= 1066 + 2600; t += 10) fs.tick(t);
    TEST_ASSERT_EQUAL_UINT8(PURSUIT_STATE_IDLE, fs.state());
    TEST_ASSERT_FALSE(fs.motors_enabled());
}

void test_landing_times_out_even_if_the_link_never_worked(void)
{
    /* IMU loss or a flat pack can put the aircraft into LANDING on a flight
     * where no command was ever accepted, leaving last_cmd_ms_ at 0. tick()
     * returns early on that, so a timer placed after the guard would never
     * run — the one case where a stuck LANDING has no operator to notice. */
    FlightState bench;
    bench.sensors_ready();
    pursuit_cmd_t c;
    fill_cmd(&c, 1, 0, 0, 0, 1.0f, ARMED_HOLD);
    bench.on_packet((uint8_t *)&c, sizeof(c), 0);   /* clock still at zero */
    bench.emergency_land();
    TEST_ASSERT_EQUAL_UINT8(PURSUIT_STATE_LANDING, bench.state());

    for (uint32_t t = 0; t <= 2600; t += 10) bench.tick(t);
    TEST_ASSERT_EQUAL_UINT8(PURSUIT_STATE_IDLE, bench.state());
    TEST_ASSERT_FALSE(bench.motors_enabled());
}

void test_a_disarm_packet_cannot_cut_a_landing_short(void)
{
    /* The mirror image of the timeout: the arm bit is a HELD control, so the
     * operator releasing it is exactly how a landing is commanded — and if
     * the released bit also exited LANDING, letting go would cut the motors
     * mid-air. Only the timeout may end this state. */
    FlightState fs;
    fs.sensors_ready();
    pursuit_cmd_t c;
    fill_cmd(&c, 1, 0, 0, 0, 1.0f, ARMED_HOLD);
    fs.on_packet((uint8_t *)&c, sizeof(c), 1000);
    fill_cmd(&c, 2, 0, 0, 0, 1.0f, ARMED_HOLD);
    fs.on_packet((uint8_t *)&c, sizeof(c), 1033);
    TEST_ASSERT_EQUAL_UINT8(PURSUIT_STATE_FLYING, fs.state());

    /* Button released: descend. */
    fill_cmd(&c, 3, 0, 0, 0, 1.0f, PURSUIT_FLAG_ALT_HOLD);
    fs.on_packet((uint8_t *)&c, sizeof(c), 1066);
    TEST_ASSERT_EQUAL_UINT8(PURSUIT_STATE_LANDING, fs.state());

    /* It stays released — 30Hz of disarmed packets for the next half second,
     * every one of which must leave the descent running. */
    uint32_t seq = 4;
    for (uint32_t t = 1099; t < 1600; t += 33, seq++) {
        fill_cmd(&c, seq, 0, 0, 0, 1.0f, PURSUIT_FLAG_ALT_HOLD);
        fs.on_packet((uint8_t *)&c, sizeof(c), t);
        fs.tick(t);
        TEST_ASSERT_EQUAL_UINT8(PURSUIT_STATE_LANDING, fs.state());
        TEST_ASSERT_TRUE(fs.motors_enabled());
    }
}

void test_armed_state_holds_idle_throttle(void)
{
    /* A spring-centred stick rests at half travel, so without this gate the
     * motors jump to 50% the instant the deadman is squeezed — before the
     * state machine has declared FLYING. */
    TEST_ASSERT_EQUAL_FLOAT(MIN_SPIN_THROTTLE,
        pursuit_armed_idle_gate(PURSUIT_STATE_ARMED, 0.5f,
                                MIN_SPIN_THROTTLE));
    /* Below the floor passes through — zero commanded stays zero. */
    TEST_ASSERT_EQUAL_FLOAT(0.0f,
        pursuit_armed_idle_gate(PURSUIT_STATE_ARMED, 0.0f,
                                MIN_SPIN_THROTTLE));
    /* FLYING and LANDING are untouched. */
    TEST_ASSERT_EQUAL_FLOAT(0.8f,
        pursuit_armed_idle_gate(PURSUIT_STATE_FLYING, 0.8f,
                                MIN_SPIN_THROTTLE));
    TEST_ASSERT_EQUAL_FLOAT(0.4f,
        pursuit_armed_idle_gate(PURSUIT_STATE_LANDING, 0.4f,
                                MIN_SPIN_THROTTLE));
}

/* ---------------------------------------------------------------- battery */

void test_full_pack_gets_full_authority(void)
{
    Battery b;
    b.reset(4.10f);
    for (int i = 0; i < 100; i++) b.update(4.10f, 0.02f);
    TEST_ASSERT_FLOAT_WITHIN(1e-3, 1.0f, b.throttle_scale());
    TEST_ASSERT_FALSE(b.critical());
}

void test_sag_reduces_throttle_quickly(void)
{
    /* The whole point: the limiter must react within a burst, not after it. */
    Battery b;
    b.reset(3.90f);
    for (int i = 0; i < 5; i++) b.update(3.55f, 0.02f);   /* 100ms of sag */
    TEST_ASSERT_TRUE(b.throttle_scale() < 0.95f);
    /* ...while the slow filter has barely noticed, so it does not land. */
    TEST_ASSERT_TRUE(b.volts() > VBAT_LOW_V);
    TEST_ASSERT_FALSE(b.critical());
}

void test_throttle_never_cut_entirely(void)
{
    /* Zero throttle at low voltage turns a descent into a fall — and a
     * falling aircraft draws no current, so the voltage recovers, throttle
     * returns, and it oscillates its way down. */
    Battery b;
    b.reset(3.20f);
    for (int i = 0; i < 200; i++) b.update(3.20f, 0.02f);
    TEST_ASSERT_TRUE(b.throttle_scale() > 0.2f);
}

void test_sustained_low_voltage_latches_critical(void)
{
    Battery b;
    b.reset(3.85f);
    for (int i = 0; i < 1000; i++) b.update(3.35f, 0.02f);  /* 20s */
    TEST_ASSERT_TRUE(b.critical());

    /* Latched: a recovering voltage does not un-declare a flat pack. */
    for (int i = 0; i < 500; i++) b.update(3.95f, 0.02f);
    TEST_ASSERT_TRUE(b.critical());
}

void test_adc_scaling(void)
{
    /* Half scale on a 12-bit ADC at 3.3V through a 2:1 divider = 3.3V pack. */
    float v = adc_to_pack_volts(2048, 4095, 3.3f, 2.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 3.3f, v);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, adc_to_pack_volts(100, 0, 3.3f, 2.0f));
}

/* ------------------------------------------------------------------- main */

/* ------------------------------------------------- battery arming interlock */
/* Added 2026-08-20, once the VBAT sense chain was proved trustworthy. Before
 * the ADC glitch fix a phantom reading could have latched critical() and
 * grounded a healthy aircraft, which is why this gate was deliberately held
 * back until then. */
void test_a_flat_pack_refuses_to_arm(void)
{
    FlightState fs;
    fs.sensors_ready();
    fs.set_battery_critical(true);

    pursuit_cmd_t c;
    fill_cmd(&c, 1, 0, 0, 0, 0.0f, ARMED_HOLD);
    fs.on_packet((uint8_t *)&c, sizeof(c), 100);
    TEST_ASSERT_EQUAL_UINT8(PURSUIT_STATE_IDLE, fs.state());
    TEST_ASSERT_FALSE(fs.motors_enabled());
}

void test_the_interlock_does_not_block_a_healthy_pack(void)
{
    /* The gate must only ever refuse. If this regresses, nothing flies. */
    FlightState fs;
    fs.sensors_ready();
    fs.set_battery_critical(false);

    pursuit_cmd_t c;
    fill_cmd(&c, 1, 0, 0, 0, 0.0f, ARMED_HOLD);
    fs.on_packet((uint8_t *)&c, sizeof(c), 100);
    TEST_ASSERT_EQUAL_UINT8(PURSUIT_STATE_ARMED, fs.state());
}

void test_a_flat_pack_does_not_oscillate_arm_to_land(void)
{
    /* The failure this exists to stop: arm packet -> ARMED, battery tick ->
     * LANDING, timeout -> IDLE, still-held arm button -> ARMED, forever. */
    FlightState fs;
    fs.sensors_ready();
    fs.set_battery_critical(true);

    pursuit_cmd_t c;
    uint32_t t = 100;
    for (int i = 0; i < 200; i++) {        /* ~6.6s of held arm at 30Hz */
        fill_cmd(&c, (uint8_t)(i + 1), 0, 0, 0, 0.0f, ARMED_HOLD);
        fs.on_packet((uint8_t *)&c, sizeof(c), t);
        fs.emergency_land();
        fs.tick(t);
        TEST_ASSERT_EQUAL_UINT8(PURSUIT_STATE_IDLE, fs.state());
        t += 33;
    }
}

int main(int, char **)
{
    UNITY_BEGIN();

    RUN_TEST(test_hover_is_symmetric);
    RUN_TEST(test_roll_right_lifts_the_left_side);
    RUN_TEST(test_pitch_forward_lifts_the_back);
    RUN_TEST(test_yaw_right_speeds_up_the_ccw_pair);
    RUN_TEST(test_saturation_preserves_attitude_authority);
    RUN_TEST(test_outputs_stay_in_range);
    RUN_TEST(test_nan_never_reaches_a_motor);
    RUN_TEST(test_disarmed_means_stopped);
    RUN_TEST(test_idle_floor_keeps_control_alive);

    RUN_TEST(test_stays_in_boot_until_sensors_pass);
    RUN_TEST(test_a_fault_cannot_promote_boot_to_idle);
    RUN_TEST(test_emergency_land_still_works_once_sensors_have_passed);
    RUN_TEST(test_arming_requires_the_flag_on_every_packet);
    RUN_TEST(test_a_flat_pack_refuses_to_arm);
    RUN_TEST(test_the_interlock_does_not_block_a_healthy_pack);
    RUN_TEST(test_a_flat_pack_does_not_oscillate_arm_to_land);
    RUN_TEST(test_corrupt_packets_are_discarded);
    RUN_TEST(test_replayed_packets_are_ignored);
    RUN_TEST(test_watchdog_levels_off_then_lands);
    RUN_TEST(test_failsafe_flag_holds_but_does_not_land);
    RUN_TEST(test_setpoints_are_clamped_to_flight_limits);
    RUN_TEST(test_disarming_in_flight_lands_rather_than_dropping);
    RUN_TEST(test_repeated_emergency_land_stays_in_landing);
    RUN_TEST(test_landing_times_out_to_disarmed_but_not_early);
    RUN_TEST(test_landing_times_out_even_if_the_link_never_worked);
    RUN_TEST(test_a_disarm_packet_cannot_cut_a_landing_short);
    RUN_TEST(test_armed_state_holds_idle_throttle);

    RUN_TEST(test_full_pack_gets_full_authority);
    RUN_TEST(test_sag_reduces_throttle_quickly);
    RUN_TEST(test_throttle_never_cut_entirely);
    RUN_TEST(test_sustained_low_voltage_latches_critical);
    RUN_TEST(test_adc_scaling);

    return UNITY_END();
}
