/* PURSUIT-1 flight firmware — top level.
 *
 * Division of labour with the laptop:
 *
 *   laptop  decides WHO to follow and WHERE they are, and sends attitude
 *           setpoints at 30Hz.
 *   here    keeps the aircraft in the air at 1kHz, and refuses to act on
 *           anything it cannot verify.
 *
 * Everything genuinely safety-critical — the mixer, the watchdog, the arming
 * interlocks, the battery limiter, the attitude filter and the PID cascade —
 * lives in files with no Arduino headers and is tested on the host
 * (`pio test -e native`, 40 tests). This file is the hardware wiring around
 * that logic, and is deliberately thin.
 *
 * STATUS: complete stabilisation path — IMU -> Madgwick -> angle PID -> rate
 * PID -> mixer -> staggered PWM. NEVER FLOWN. The PID gains are scaled from
 * typical brushed-whoop values and have not touched this airframe, so the
 * first power-up is props-off, on the bench, watching telemetry.
 */
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

#include "alt_hold.h"
#include "battery.h"
#include "benchtest.h"
#include "blackbox.h"
#include "camera.h"
#include "config.h"
#include "flight_state.h"
#include "frame.h"
#include "icm42688.h"
#include "loopstats.h"
#include "madgwick.h"
#include "mixer.h"
#include "motors.h"
#include "pid.h"
#include "pursuit_link.h"
#include "rangefinder.h"

/* SoftAP so the drone does not depend on a router being present in a field.
 * The laptop joins this network; the link config in the vision stack defaults
 * to 192.168.4.1, which is the ESP32's own AP address.
 *
 * The passphrase lives in secrets.h, which is gitignored — see
 * include/secrets.h.example. Packet validation (magic, version, CRC, strict
 * sequence, held ARM authority) proves a command is well-formed, not that it
 * came from the operator; WPA2 is what makes that distinction, so the key is
 * treated as a credential rather than a constant. */
#if __has_include("secrets.h")
#include "secrets.h"
#endif

#ifndef PURSUIT_AP_SSID
#define PURSUIT_AP_SSID "PURSUIT-1"
#endif
#ifndef PURSUIT_AP_PASS
#error "No secrets.h — run: cp firmware/include/secrets.h.example firmware/include/secrets.h, then set your own PURSUIT_AP_PASS. Refusing to build with a published default."
#endif

static const char *AP_SSID = PURSUIT_AP_SSID;
static const char *AP_PASS = PURSUIT_AP_PASS;

static WiFiUDP g_udp;
static IPAddress g_peer;
static uint16_t g_peer_port = 0;

static FlightState g_state;
static Battery g_batt;

static uint32_t g_telem_seq = 0;
static uint32_t g_loop_count = 0;
static uint32_t g_loop_hz = 0;

/* Loop-period distribution against the LOOP_HZ budget. The mean rate above is
 * kept because telemetry carries it, but it is the number that hides a stall;
 * this is the one to read after a flight. Exposed so the bench console can
 * print it. */
LoopStats g_loop_stats(1000000UL / LOOP_HZ);

static Icm42688 g_imu;
static Madgwick g_ahrs(0.06f);
static AnglePid g_ang_roll, g_ang_pitch;
static RatePid  g_rate_roll, g_rate_pitch, g_rate_yaw;
static Rangefinder g_tof;
static AltitudeHold g_alt;

static EulerAngles g_att = {0.0f, 0.0f, 0.0f};
static ImuSample g_imu_sample{};
static float g_height_m = 0.0f;
static bool g_height_valid = false;
static uint32_t g_last_height_ms = 0;

/* Outer-loop gain: 6 deg/s of rotation demanded per degree of attitude
 * error, capped at the airframe's rate limit. Gentle on purpose — the angle
 * loop's input is a filtered estimate with real lag, and pushing it hard is
 * how a cascade starts fighting itself. */
static constexpr float ANGLE_KP = 6.0f;

static uint8_t g_imu_bad = 0;

static void attitude_update(float dt_s)
{
    g_imu_sample = g_imu.read();
    if (!g_imu_sample.valid) {
        /* Flying on a frozen attitude estimate is unrecoverable: the PID
         * holds whatever correction it last computed while the aircraft
         * drifts. ~50ms of consecutive failures on a 1kHz bus means the
         * sensor or the bus is gone, not glitching. */
        if (g_imu_bad < 255) g_imu_bad++;
        if (g_imu_bad == 50 && g_state.motors_enabled()) {
            Serial.println("IMU LOST IN FLIGHT — emergency landing");
            g_state.emergency_land();
        }
        return;
    }
    g_imu_bad = 0;

    g_ahrs.update(g_imu_sample.gx, g_imu_sample.gy, g_imu_sample.gz,
                  g_imu_sample.ax, g_imu_sample.ay, g_imu_sample.az, dt_s);
    g_att = g_ahrs.euler();
}

/* ------------------------------------------------------------------ link */

static void link_begin(void)
{
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS);
    g_udp.begin(PURSUIT_CMD_PORT);
    Serial.printf("AP %s  ip %s  cmd:%u telem:%u\n", AP_SSID,
                  WiFi.softAPIP().toString().c_str(),
                  PURSUIT_CMD_PORT, PURSUIT_TELEM_PORT);
}

static void link_poll(uint32_t now_ms)
{
    uint8_t buf[64];
    int n;
    /* Drain the whole queue each pass. Acting on the oldest datagram when a
     * newer one is already buffered means steering on stale information. */
    while ((n = g_udp.parsePacket()) > 0) {
        const int got = g_udp.read(buf, sizeof(buf));
        if (got <= 0) continue;
        if (g_state.on_packet(buf, (uint32_t)got, now_ms)) {
            g_peer = g_udp.remoteIP();
            g_peer_port = PURSUIT_TELEM_PORT;
        }
    }
}

static void telemetry_send(uint32_t now_ms)
{
    if (g_peer_port == 0) return;

    pursuit_telem_t t = {};
    t.magic = PURSUIT_TELEM_MAGIC;
    t.version = PURSUIT_PROTOCOL_VERSION;
    t.state = g_state.state();
    t.seq = ++g_telem_seq;
    /* Echo the timestamp from the last accepted COMMAND rather than the local
     * clock. The two ends have no shared time base, so an absolute reading
     * from here is meaningless to the laptop — but echoing its own stamp back
     * lets it subtract and get a true round-trip. That number is what the
     * controller's lead compensation needs, and it is currently a guess. */
    t.t_ms = g_state.last_cmd_t_ms();
    t.vbat = g_batt.volts();
    t.alt = g_height_valid ? g_height_m : 0.0f;
    t.roll = g_att.roll;
    t.pitch = g_att.pitch;
    t.yaw = g_att.yaw;
    /* The vision loop subtracts roll and pitch from the target's apparent
     * position, so these must be the real attitude — not the commanded one.
     * Sending the setpoint here would make the correction a no-op and the
     * altitude axis would couple to pitch exactly as before. */
    t.loop_hz = (uint16_t)g_loop_hz;
    t.crc = pursuit_crc16((const uint8_t *)&t, sizeof(t) - 2);

    g_udp.beginPacket(g_peer, g_peer_port);
    g_udp.write((const uint8_t *)&t, sizeof(t));
    g_udp.endPacket();
}

/* ------------------------------------------------------------------ setup */

void setup(void)
{
    Serial.begin(115200);

    /* Motors first, before anything that can block or fail. A half-configured
     * peripheral must never leave a gate floating near threshold. */
    motors_begin();
    motors_stop();

    pinMode(PIN_LED_CTRL, OUTPUT);
    digitalWrite(PIN_LED_CTRL, LOW);

    analogReadResolution(12);
    /* analogReadMilliVolts applies the factory eFuse calibration. The raw
     * count path (adc_to_pack_volts) assumes a linear 3.3V ADC, and the
     * S3's ADC is 5-10% nonlinear — 0.2-0.4V of error against a sag-limiter
     * window that is only 0.45V wide. */
    g_batt.reset((float)analogReadMilliVolts(PIN_VBAT_SENSE) * 1e-3f *
                 VBAT_DIVIDER);

    link_begin();

    g_ang_roll.configure(ANGLE_KP, MAX_YAW_RATE_DPS);
    g_ang_pitch.configure(ANGLE_KP, MAX_YAW_RATE_DPS);
    g_rate_roll.configure(default_roll_rate_gains());
    g_rate_pitch.configure(default_pitch_rate_gains());
    g_rate_yaw.configure(default_yaw_rate_gains());

    /* --- sensors, and the gate that depends on them ---------------------
     * The aircraft stays in BOOT — motors physically disabled — unless the
     * IMU answers, self-tests and calibrates. Arming on a sensor that is not
     * there is worse than refusing to arm.
     */
    if (!g_imu.begin()) {
        Serial.printf("IMU FAILED: %s (who_am_i 0x%02X) — will not arm\n",
                      g_imu.last_error(), g_imu.who_am_i());
        return;
    }
    Serial.println("IMU ok (ICM-42688-P)");

    Serial.println("calibrating gyro — keep still...");
    if (!g_imu.calibrate_gyro()) {
        Serial.printf("CALIBRATION FAILED: %s — will not arm\n",
                      g_imu.last_error());
        return;
    }
    float bias[3];
    g_imu.gyro_bias(bias);
    Serial.printf("gyro bias  %.2f %.2f %.2f deg/s\n",
                  bias[0], bias[1], bias[2]);

    /* Snap the filter to the measured horizon rather than letting it converge
     * from identity: an aircraft powered up on a slope would otherwise
     * believe it is level for the first couple of seconds. */
    ImuSample s = g_imu.read();
    if (s.valid) g_ahrs.align_to_accel(s.ax, s.ay, s.az);

    /* --- optional sensors ------------------------------------------------
     * Neither of these gates arming. Without the rangefinder the aircraft
     * flies on direct throttle; without the camera it flies but cannot
     * follow. Refusing to arm over a missing nice-to-have would ground a
     * perfectly airworthy machine.
     */
    if (g_tof.begin()) {
        Serial.println("rangefinder ok (VL53L1X) — altitude hold available");
    } else {
        Serial.printf("rangefinder ABSENT (%s) — direct throttle only\n",
                      g_tof.last_error());
    }

    if (camera_begin()) {
        Serial.printf("camera ok — stream on http://%s:81/\n",
                      WiFi.softAPIP().toString().c_str());
    } else {
        Serial.printf("camera ABSENT (%s) — no vision\n", camera_error());
    }

    g_alt.configure(default_alt_hold_config());

    if (blackbox_begin(500, 420)) {
        blackbox_serve();
        Serial.printf("blackbox ok — %u records (~%us at 500Hz), "
                      "download from http://%s:82/ after landing\n",
                      (unsigned)blackbox_capacity(),
                      (unsigned)(blackbox_capacity() / 500),
                      WiFi.softAPIP().toString().c_str());
    } else {
        Serial.println("blackbox ABSENT (no PSRAM) — tuning will be blind");
    }

    g_state.sensors_ready();
    Serial.println("PURSUIT-1 ready — NEVER FLOWN, GAINS UNTUNED, PROPS OFF");
    bench_print_help();
}

/* ------------------------------------------------------------------- loop */

void loop(void)
{
    static uint32_t last_loop_us = micros();
    static uint32_t last_batt_ms = 0;
    static uint32_t last_tof_ms = 0;
    static uint32_t last_telem_ms = 0;
    static uint32_t last_hz_ms = 0;

    const uint32_t now_ms = millis();
    const uint32_t now_us = micros();
    const uint32_t period_us = now_us - last_loop_us;
    const float dt_s = (float)period_us * 1e-6f;
    last_loop_us = now_us;

    /* Characterise the loop rather than assert it. The very first iteration
     * measures the gap since last_loop_us was zero-initialised — an artefact
     * of boot, not a stall — so it is skipped. */
    static bool first_loop = true;
    if (first_loop) first_loop = false;
    else            g_loop_stats.sample(period_us);

    /* Bench console. Only ever reachable while IDLE — bench_key() and
     * bench_update() both re-check the state, so a command arriving mid-test
     * cancels it rather than racing it. */
    static BenchDeps bench{&g_state, &g_imu, &g_ahrs, &g_batt, &g_tof};
    while (Serial.available()) {
        const char c = (char)Serial.read();
        if (c != '\n' && c != '\r') bench_key(c, bench);
    }

    const uint32_t before = g_state.accepted();
    link_poll(now_ms);
    if (g_state.accepted() != before) bench_abort("command received");
    g_state.tick(now_ms);
    bench_update(now_ms, bench);

    attitude_update(dt_s);

    /* Rangefinder at its own rate — it only produces a new sample at 30Hz,
     * and the attitude is passed in so the reading can be projected back onto
     * the vertical before anything acts on it. */
    if (now_ms - last_tof_ms >= 1000 / TOF_HZ) {
        RangeSample r = g_tof.update(g_att.roll, g_att.pitch);
        if (r.valid) {
            g_height_m = r.height_m;
            g_height_valid = true;
            g_last_height_ms = now_ms;
        }
        last_tof_ms = now_ms;
    }

    if (now_ms - last_batt_ms >= 1000 / BATT_HZ) {
        const float v = (float)analogReadMilliVolts(PIN_VBAT_SENSE) * 1e-3f *
                        VBAT_DIVIDER;
        g_batt.update(v, (float)(now_ms - last_batt_ms) * 1e-3f);
        last_batt_ms = now_ms;

        /* A flat pack is not a reason to keep flying and hope. */
        if (g_batt.critical()) g_state.emergency_land();
    }

    /* --- mix and drive ---------------------------------------------------
     * Everything above decides WHAT; this decides whether it reaches a motor.
     */
    if (bench_active()) {
        /* A bench test owns the motors. Return early so the flight path
         * cannot write over it, and so the flight path's own disarm branch
         * cannot stop a test that is deliberately running. */
        return;
    }

    if (g_state.motors_enabled()) {
        const Setpoint &sp = g_state.setpoint();
        if (!blackbox_recording()) blackbox_start();

        /* --- cascade: angle -> rate -> motor demand ----------------------
         * The outer loop turns "be at 10 degrees" into "rotate at N deg/s";
         * the inner loop makes that rotation actually happen. Yaw skips the
         * outer loop entirely because the command is already a rate — there
         * is no absolute heading to hold without a magnetometer.
         */
        const float roll_rate_sp = g_ang_roll.update(sp.roll_deg, g_att.roll);
        const float pitch_rate_sp = g_ang_pitch.update(sp.pitch_deg,
                                                       g_att.pitch);

        const float roll_cmd = g_rate_roll.update(roll_rate_sp,
                                                  g_imu_sample.gx, dt_s, 0.5f);
        const float pitch_cmd = g_rate_pitch.update(pitch_rate_sp,
                                                    g_imu_sample.gy, dt_s, 0.5f);
        /* Yaw is measured in the protocol's clockwise-positive convention.
         * Raw gz here is positive-feedback — see frame.h and the closed-loop
         * test that pins this. */
        const float yaw_cmd = g_rate_yaw.update(
            sp.yaw_rate_dps, pursuit_yaw_rate_cw(g_imu_sample.gz), dt_s, 0.3f);

        /* --- throttle: altitude hold, or straight through -----------------
         * A height reading older than ~200ms is not a height. The altitude
         * loop degrades to passing the commanded throttle through rather than
         * holding a number it can no longer measure — silently drifting into
         * the floor on a stale reading is the failure to avoid.
         */
        const bool height_fresh =
            g_height_valid && (now_ms - g_last_height_ms) < 200;

        float throttle;
        if (sp.alt_hold) {
            throttle = g_alt.update(sp.alt_sp_m, g_height_m, height_fresh,
                                    g_alt.hover_throttle(), dt_s);
        } else {
            throttle = sp.alt_sp_m;   /* the field carries throttle directly */
        }

        if (g_state.state() == PURSUIT_STATE_LANDING) {
            /* Descend under control rather than cutting: a dropped aircraft
             * breaks props and, on this airframe, the camera flex. Below the
             * hover baseline it sinks; not so far below that it falls. */
            throttle = fminf(throttle, g_alt.hover_throttle() * 0.80f);
        }

        /* ARMED is authorised-not-flying: hold idle until FLYING, so a
         * spring-centred stick cannot spin motors to 50% on arm. */
        throttle = pursuit_armed_idle_gate(g_state.state(), throttle,
                                           MIN_SPIN_THROTTLE);

        /* Battery limiting last, so it can override everything above. */
        throttle *= g_batt.throttle_scale();
        if (throttle > MAX_THROTTLE) throttle = MAX_THROTTLE;

        MotorOutputs m = mix_quad_x(roll_cmd, pitch_cmd, yaw_cmd, throttle);
        m = apply_idle_floor(m, MIN_SPIN_THROTTLE, true);
        motors_write(m);

        /* Record AFTER the motors are driven, so a slow logger can never sit
         * between the control decision and the aircraft acting on it. */
        static uint32_t last_bb_us = 0;
        if (blackbox_recording() && (now_us - last_bb_us) >= 2000) {
            /* The loop runs faster than 500Hz; pushing every pass silently
             * shrank the 7-minute buffer to under two. */
            last_bb_us = now_us;
            BlackboxRecord r;
            r.t_us = now_us;
            r.gyro[0] = bb_scale_gyro(g_imu_sample.gx);
            r.gyro[1] = bb_scale_gyro(g_imu_sample.gy);
            r.gyro[2] = bb_scale_gyro(pursuit_yaw_rate_cw(g_imu_sample.gz));
            r.att[0] = bb_scale_angle(g_att.roll);
            r.att[1] = bb_scale_angle(g_att.pitch);
            r.att[2] = bb_scale_angle(g_att.yaw);
            r.sp[0] = bb_scale_angle(sp.roll_deg);
            r.sp[1] = bb_scale_angle(sp.pitch_deg);
            r.sp[2] = bb_scale_gyro(sp.yaw_rate_dps);
            r.pid[0] = bb_scale_unit(roll_cmd);
            r.pid[1] = bb_scale_unit(pitch_cmd);
            r.pid[2] = bb_scale_unit(yaw_cmd);
            r.motor[0] = bb_scale_duty(m.m1);
            r.motor[1] = bb_scale_duty(m.m2);
            r.motor[2] = bb_scale_duty(m.m3);
            r.motor[3] = bb_scale_duty(m.m4);
            r.throttle = bb_scale_duty(throttle);
            r.vbat = bb_scale_volts(g_batt.instant_volts());
            r.alt_mm = (uint16_t)(g_height_valid
                                  ? fminf(g_height_m * 1000.0f, 65535.0f) : 0);
            r.state = g_state.state();
            r.flags = (uint8_t)((sp.locked ? 1 : 0) |
                                (sp.alt_hold ? 2 : 0) |
                                (g_height_valid ? 4 : 0) |
                                (g_alt.holding() ? 8 : 0));
            /* The loop PERIOD — spikes here are stalls (WiFi, camera). */
            r.loop_us = (uint16_t)fminf(dt_s * 1e6f, 65535.0f);
            blackbox_push(r);
        }
    } else {
        motors_stop();
        /* Reset the integrators whenever the motors are off. Otherwise the
         * error accumulated while sitting disarmed on a bench is waiting to
         * be spent the instant it arms. */
        g_rate_roll.reset();
        g_rate_pitch.reset();
        g_rate_yaw.reset();
        /* Re-seat the altitude setpoint at wherever the aircraft actually is,
         * so arming never commands a step change in height. The LEARNED hover
         * throttle deliberately survives — it is a property of the airframe
         * and battery, not of this arming. */
        g_alt.arm(g_height_valid ? g_height_m : 0.0f);
        /* One log per flight, not an afternoon of bench idling. */
        if (blackbox_recording()) {
            blackbox_stop();
            Serial.printf("blackbox: %u records (%u overwritten) — "
                          "http://%s:82/\n",
                          (unsigned)blackbox_count(),
                          (unsigned)blackbox_dropped(),
                          WiFi.softAPIP().toString().c_str());
        }
    }

    digitalWrite(PIN_LED_CTRL,
                 g_state.motors_enabled() ? ((now_ms / 100) & 1) : LOW);

    if (now_ms - last_telem_ms >= 1000 / TELEM_HZ) {
        telemetry_send(now_ms);
        last_telem_ms = now_ms;
    }

    g_loop_count++;
    if (now_ms - last_hz_ms >= 1000) {
        g_loop_hz = g_loop_count;
        g_loop_count = 0;
        last_hz_ms = now_ms;
    }
}
