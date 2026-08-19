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
#include "altitude_fuse.h"
#include "battery.h"
#include "benchtest.h"
#include "blackbox.h"
#include "bmp390.h"
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
static Bmp390 g_baro;
static AltitudeFuse g_altfuse;
static AltitudeHold g_alt;

static EulerAngles g_att = {0.0f, 0.0f, 0.0f};
static ImuSample g_imu_sample{};

/* The FUSED height above the arming point — rangefinder low down, barometer
 * above it, blended across the handover. Everything downstream (altitude hold,
 * telemetry, the blackbox) reads this rather than either sensor directly, so
 * there is exactly one definition of "how high is it" in the firmware. */
static float g_height_m = 0.0f;
static bool g_height_valid = false;
static uint32_t g_last_height_ms = 0;

/* Raw barometric altitude on the sensor's own datum, kept only so the fusion
 * filter can be re-zeroed at the instant of arming. Never used as a height. */
static float g_baro_alt_m = 0.0f;

/* Outer-loop gain: 6 deg/s of rotation demanded per degree of attitude
 * error, capped at the airframe's rate limit. Gentle on purpose — the angle
 * loop's input is a filtered estimate with real lag, and pushing it hard is
 * how a cascade starts fighting itself. */
static constexpr float ANGLE_KP = 6.0f;

/* How long the estimate has been stale, in seconds. Accumulated rather than
 * counted, because the loop rate is not a constant when the IMU is failing —
 * see IMU_LOSS_TIMEOUT_S in config.h. */
static float g_imu_bad_s = 0.0f;
static bool g_imu_loss_declared = false;

/* Median of five, because a single sample is not trustworthy while the
 * motors switch.
 *
 * The divider is 100k/100k, so the ADC sees a 50 kOhm source; Espressif want
 * well under 10 kOhm for the sample-and-hold to settle. It is marginal even
 * in silence, and 32 kHz motor PWM pushes it over. Measured 2026-08-20: the
 * raw channel returned 1.70 V at 0% duty and 0.00 V mid-run, on a pack a
 * meter put at 3.64 V.
 *
 * A median rather than a mean on purpose. The corruption is impulsive — a few
 * wildly wrong samples among good ones — and a mean folds those outliers into
 * the answer, while a median of five discards up to two of them outright.
 * Averaging noise like this is how you get a number that is smooth, stable,
 * and wrong.
 *
 * Costs five conversions at BATT_HZ (50Hz), not per loop. */
static float read_pack_volts(void)
{
    float s[5];
    for (int i = 0; i < 5; i++) {
        s[i] = (float)analogReadMilliVolts(PIN_VBAT_SENSE) * 1e-3f *
               VBAT_DIVIDER;
    }
    for (int i = 1; i < 5; i++) {          /* insertion sort, n=5 */
        const float k = s[i];
        int j = i - 1;
        while (j >= 0 && s[j] > k) { s[j + 1] = s[j]; j--; }
        s[j + 1] = k;
    }
    return s[2];
}

static void attitude_update(float dt_s)
{
    /* Read into a LOCAL and publish it only once it is known good.
     *
     * Found by inspection 2026-08-19: this used to assign g_imu.read()
     * straight into g_imu_sample and test .valid afterwards. A failed burst
     * read returns a value-initialised ImuSample (icm42688.cpp builds
     * `ImuSample s{}` over a plain aggregate with no default member
     * initialisers), so every axis is 0.0 — and that zeroed sample was
     * already in the global by the time anyone asked whether it was real.
     * Madgwick was correctly spared by the early return below, but NOTHING
     * between here and mix_quad_x() re-checks validity, so the rate PIDs were
     * handed 0.0 deg/s as a measurement of the airframe's actual rotation.
     *
     * That is not a benign zero. pid.cpp rejects only non-finite values, so
     * 0.0f sails through, and it differentiates on the MEASUREMENT — a
     * real-rate -> 0 -> real-rate round trip is two large steps in
     * d(measurement)/dt. The kick is kd*|rate|/(rc+dt) = 0.0164 per deg/s
     * against the 0.5 limit passed below: saturated above ~30 deg/s, and
     * still 16-66% of full roll authority at a mundane 5-20 deg/s, on a
     * single dropped I2C transaction. Coasting on the previous sample is
     * ~1 ms stale at loop rate; that is a far smaller lie than zero.
     *
     * The validity test MUST be on the local `s`, not on the global. Writing
     * `if (s.valid) g_imu_sample = s;` and then testing g_imu_sample.valid
     * would leave the global's flag stuck true from the last good read: the
     * accumulator below would never advance, the emergency_land() would be
     * unreachable, and a totally dead IMU would fly on a frozen sample
     * forever — worse than the bug this fixes.
     *
     * Deliberately NOT resetting the rate PIDs on the recovery edge:
     * RatePid::reset() also zeroes integral_, which would dump the
     * accumulated hover trim every time a single read was dropped. The
     * recovery step is the genuine rate change the D term should see.
     */
    const ImuSample s = g_imu.read();
    if (!s.valid) {
        /* Flying on a frozen attitude estimate is unrecoverable: the PID
         * holds whatever correction it last computed while the aircraft
         * drifts out from under it. */
        if (dt_s > 0.0f) g_imu_bad_s += dt_s;
        if (!g_imu_loss_declared && g_imu_bad_s >= IMU_LOSS_TIMEOUT_S
            && g_state.motors_enabled()) {
            g_imu_loss_declared = true;
            Serial.printf("IMU LOST IN FLIGHT (%.0f ms) — emergency landing\n",
                          g_imu_bad_s * 1000.0f);
            g_state.emergency_land();
        }
        return;
    }
    g_imu_sample = s;
    g_imu_bad_s = 0.0f;
    g_imu_loss_declared = false;

    g_ahrs.update(g_imu_sample.gx, g_imu_sample.gy, g_imu_sample.gz,
                  g_imu_sample.ax, g_imu_sample.ay, g_imu_sample.az, dt_s);
    g_att = g_ahrs.euler();
}

/* ------------------------------------------------------------------ link */

static void link_begin(void)
{
    WiFi.mode(WIFI_AP);
    /* Channel is chosen, not defaulted — see WIFI_AP_CHANNEL in config.h.
     * Arguments after the passphrase are (channel, hidden, max_connections):
     * one client, because a second station on this AP is either a mistake or
     * someone else, and both share the airtime the control link needs. */
    WiFi.softAP(AP_SSID, AP_PASS, WIFI_AP_CHANNEL, 0, 1);
    WiFi.setTxPower(WIFI_TX_POWER);
    g_udp.begin(PURSUIT_CMD_PORT);
    Serial.printf("AP %s  ip %s  ch%u  txpwr %.1f dBm  cmd:%u telem:%u\n",
                  AP_SSID, WiFi.softAPIP().toString().c_str(),
                  (unsigned)WIFI_AP_CHANNEL, WiFi.getTxPower() * 0.25f,
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
     *
     * A failure here used to `return` immediately. That kept the aircraft
     * safe and made it undiagnosable: without an IMU the firmware never
     * reached the camera, the blackbox or the link, so a bare XIAO on a desk
     * — which is exactly the state hardware arrives in, before the PCB
     * exists — could not be used to verify any of them.
     *
     * Now it degrades into bring-up mode instead. Every optional subsystem is
     * still brought up and can be exercised from the bench console; the
     * aircraft simply never leaves BOOT, so `sensors_ready()` is not called
     * and the motors cannot be enabled by any path. The safety property is
     * unchanged and now explicit rather than implied by control flow.
     */
    bool imu_ok = g_imu.begin();
    if (!imu_ok) {
        Serial.printf("IMU FAILED: %s (who_am_i 0x%02X, expected 0x47)\n",
                      g_imu.last_error(), g_imu.who_am_i());
    } else {
        Serial.println("IMU ok (ICM-42688-P)");
        Serial.println("calibrating gyro — keep still...");
        if (!g_imu.calibrate_gyro()) {
            Serial.printf("CALIBRATION FAILED: %s\n", g_imu.last_error());
            imu_ok = false;
        } else {
            float bias[3];
            g_imu.gyro_bias(bias);
            Serial.printf("gyro bias  %.2f %.2f %.2f deg/s\n",
                          bias[0], bias[1], bias[2]);
            /* Snap the filter to the measured horizon rather than letting it
             * converge from identity: an aircraft powered up on a slope would
             * otherwise believe it is level for the first couple of seconds. */
            ImuSample s = g_imu.read();
            if (s.valid) g_ahrs.align_to_accel(s.ax, s.ay, s.az);
        }
    }

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

    /* The barometer is what decides the ceiling. Without it the aircraft is
     * limited to the rangefinder's honest range; with it, roughly 30 ft. Both
     * numbers are printed because the difference is the entire reason the part
     * is fitted, and because a barometer that silently failed to probe would
     * otherwise look identical to one that is working. */
    if (g_baro.begin()) {
        Serial.printf("barometer ok (BMP390 @0x%02X) — ceiling %.1fm\n",
                      g_baro.address(), ALT_CEILING_BARO_M);
        Serial.println("  port must be covered with open-cell FOAM, not tape:"
                       " prop wash across");
        Serial.println("  it reads as tens of metres, and light on the die "
                       "shifts the reading");
    } else {
        Serial.printf("barometer ABSENT (%s) — ceiling %.1fm on the "
                      "rangefinder alone\n",
                      g_baro.last_error(), ALT_CEILING_TOF_M);
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

    if (imu_ok) {
        g_state.sensors_ready();
        Serial.println("PURSUIT-1 ready — NEVER FLOWN, GAINS UNTUNED, "
                       "PROPS OFF");
    } else {
        /* Deliberately loud. The dangerous version of this state is the one
         * where someone assumes the aircraft is merely quiet. */
        Serial.println("");
        Serial.println("=============== BRING-UP MODE — NO IMU ============"
                       "===========");
        Serial.println(" The aircraft is held in BOOT. sensors_ready() was "
                       "NOT called, so");
        Serial.println(" the motors cannot be enabled by any path, including "
                       "a valid armed");
        Serial.println(" command over the link. This is not a fault state to "
                       "clear — it is");
        Serial.println(" what a board with no IMU is supposed to do.");
        Serial.println("");
        Serial.println(" Still fully testable from here:");
        Serial.println("   camera + MJPEG stream      (on the XIAO module "
                       "itself)");
        Serial.println("   WiFi SoftAP + the UDP link (point the vision stack "
                       "at it)");
        Serial.println("   PSRAM blackbox allocation");
        Serial.println("   battery sense on D2");
        Serial.println("   motor PWM pins             ('1'-'4', drives GPIO "
                       "with no FETs)");
        Serial.println("");
        Serial.println(" Expected on a BARE XIAO with no PURSUIT-1 board: the "
                       "IMU and the");
        Serial.println(" rangefinder are on the PCB, so both SHOULD be "
                       "absent here.");
        Serial.println("==================================================="
                       "===========");
    }
    bench_print_help();
}

/* ------------------------------------------------------------------- loop */

void loop(void)
{
    static uint32_t last_loop_us = micros();
    static uint32_t last_batt_ms = 0;
    static uint32_t last_tof_ms = 0;
    static uint32_t last_baro_ms = 0;
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
    static BenchDeps bench{&g_state, &g_imu, &g_ahrs, &g_batt, &g_tof, &g_baro};
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

    /* Barometer at its own rate, pushed in RAW on its own datum — the fusion
     * filter subtracts a reference captured at arming, so an absolute
     * pressure altitude that is tens of metres out with the weather costs
     * nothing. read() returns invalid when no conversion is ready, which at
     * 50Hz against a 1kHz loop is most of the time. */
    if (now_ms - last_baro_ms >= 1000 / BARO_HZ) {
        /* A gap longer than this is a stall, not a sample interval. Feeding
         * the real number into a first-order filter would collapse it to a
         * pass-through for that step. */
        const float baro_dt =
            fminf((float)(now_ms - last_baro_ms) * 1e-3f, 0.2f);
        last_baro_ms = now_ms;

        Bmp390Sample b = g_baro.read();
        if (b.valid) {
            g_baro_alt_m = b.altitude_m;
            g_altfuse.push_baro(b.altitude_m, baro_dt);
        }
    }

    /* Rangefinder at its own rate — it only produces a new sample at 30Hz,
     * and the attitude is passed in so the reading can be projected back onto
     * the vertical before anything acts on it. */
    if (now_ms - last_tof_ms >= 1000 / TOF_HZ) {
        const float tof_dt =
            fminf((float)(now_ms - last_tof_ms) * 1e-3f, 0.2f);
        last_tof_ms = now_ms;

        RangeSample r = g_tof.update(g_att.roll, g_att.pitch);
        /* Pushed on EVERY cycle, valid or not. This call is also what ages
         * both sources inside the filter, so skipping it after a bad reading
         * would freeze the staleness check that decides when to stop trusting
         * altitude at all — the reading would simply never grow old. */
        g_altfuse.push_tof(r.height_m, r.valid, tof_dt);

        if (g_altfuse.valid()) {
            g_height_m = g_altfuse.height();
            g_height_valid = true;
            g_last_height_ms = now_ms;
        }
    }

    if (now_ms - last_batt_ms >= 1000 / BATT_HZ) {
        const float v = read_pack_volts();
        g_batt.update(v, (float)(now_ms - last_batt_ms) * 1e-3f);
        last_batt_ms = now_ms;

        /* A flat pack is not a reason to keep flying and hope. */
        g_state.set_battery_critical(g_batt.critical());
        if (g_batt.critical()) g_state.emergency_land();
    }

    /* Capture the barometric datum at the INSTANT of arming, on the rising
     * edge only. arm() deliberately discards the learned baro-to-ground
     * offset — the aircraft may not be standing where it stood last flight —
     * so calling it every pass while disarmed, the way the altitude setpoint
     * is re-seated below, would mean the offset is never learned at all.
     *
     * The datum being one noisy sample rather than a filtered one does not
     * matter: whatever error it carries lands in the offset and is subtracted
     * again. That is the whole point of learning an offset instead of
     * trusting a datum. */
    static bool was_enabled = false;
    const bool motors_on = g_state.motors_enabled();
    if (motors_on && !was_enabled) g_altfuse.arm(g_baro_alt_m);
    was_enabled = motors_on;

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
            /* The laptop asks for a height; the aircraft decides whether that
             * height is one it can actually measure. Without the barometer
             * this clamps to the rangefinder's honest range, so a vision
             * stack configured for a barometer it does not have cannot climb
             * the aircraft out of its own sensing. */
            const float target_m = pursuit_clamp_alt_setpoint(
                sp.alt_sp_m, g_baro.present(),
                ALT_CEILING_TOF_M, ALT_CEILING_BARO_M);
            throttle = g_alt.update(target_m, g_height_m, height_fresh,
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
            /* Clamped at BOTH ends. The rangefinder could never report a
             * negative height, so the old one-sided clamp was safe; the
             * barometer can — a slightly imperfect offset, or the aircraft
             * genuinely below where it armed. Converting a negative float to
             * unsigned is undefined, and in practice logs a 65-metre spike. */
            r.alt_mm = (uint16_t)(g_height_valid
                                  ? fminf(fmaxf(g_height_m * 1000.0f, 0.0f),
                                          65535.0f)
                                  : 0);
            r.state = g_state.state();
            /* Bits 4-5 carry WHICH sensor produced the height above. Without
             * it a log shows one altitude trace and no way to tell whether a
             * step in it was the aircraft moving or the fusion changing its
             * mind — which is exactly the failure this whole path exists to
             * prevent, and therefore the one worth being able to see. */
            r.flags = (uint8_t)((sp.locked ? 1 : 0) |
                                (sp.alt_hold ? 2 : 0) |
                                (g_height_valid ? 4 : 0) |
                                (g_alt.holding() ? 8 : 0) |
                                ((uint8_t)g_altfuse.source() << 4));
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
            /* How far apart the two datums ended up. A steady value near zero
             * is a healthy barometer; a large or visibly moving one means
             * prop wash is reaching the port, and the fix is mechanical —
             * more foam — not a gain change. */
            if (g_altfuse.offset_learned()) {
                Serial.printf("baro datum offset %.2fm at landing\n",
                              g_altfuse.offset());
            } else if (g_baro.present()) {
                Serial.println("baro datum NEVER LEARNED — the rangefinder "
                               "gave nothing usable below 3m, so the aircraft "
                               "flew on pressure alone above the handover");
            }
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
