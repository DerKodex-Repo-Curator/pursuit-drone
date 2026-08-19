#include "benchtest.h"

#ifndef PURSUIT_HOST_BUILD

#include <Arduino.h>
#include <cmath>

#include "battery.h"
#include "blackbox.h"

void prof_report(void);
void prof_reset(void);
#include "bmp390.h"
#include "camera.h"
#include "config.h"
#include "flight_state.h"
#include "icm42688.h"
#include "loopstats.h"
#include "madgwick.h"
#include "mixer.h"
#include "motors.h"
#include "pursuit_link.h"
#include "rangefinder.h"

namespace {

enum class Mode { Off, Motor, MotorAll, Orientation, Sensors };

Mode g_mode = Mode::Off;
int  g_motor = -1;              /* which motor is spinning, 0-3   */
float g_duty = 0.12f;           /* low enough not to walk the board */
uint32_t g_until_ms = 0;
uint32_t g_last_print = 0;

/* Deliberately brief. A motor left spinning while you look away is how a
 * bare shaft finds a finger. */
constexpr uint32_t MOTOR_RUN_MS = 2500;

/* --- all four at once ----------------------------------------------------
 * Added 2026-08-19. Until now there was NO way to run four motors together:
 * 'a' called run_motor(0) and spun M1 alone, while the help text claimed it
 * swept all four. So the largest current this airframe can draw had never
 * once been demanded of the pack, and every four-motor figure in the docs
 * was a single-motor measurement multiplied by four.
 *
 * RAMPED, not stepped. Four coreless motors going stopped -> duty inside one
 * PWM period is the worst transient the board can produce, and the 3V3 rail
 * has only ~8uF behind it (C6/C7 are deliberately DNF), so it falls 3.3->2.8V
 * in ~9.5us. Battery protection samples at 50Hz and cannot see that; the ramp
 * is what keeps the step inside what the rail can follow.
 *
 * The run aborts itself if the pack collapses past ALL_ABORT_V. That is below
 * VBAT_CRITICAL_V on purpose — the point of this test is to find out how far
 * it sags, so the abort is a backstop against a total collapse, not the
 * measurement threshold. */
constexpr uint32_t ALL_RAMP_MS = 1500;
constexpr uint32_t ALL_RUN_MS  = 4000;
constexpr float    ALL_ABORT_V = 3.00f;

uint32_t g_all_start_ms = 0;
float    g_vmin = 99.0f;        /* lowest instantaneous pack volts seen */
float    g_vmin_duty = 0.0f;    /* ...and the per-motor duty it happened at */

const char *MOTOR_NAME[4] = {
    "M1 FRONT-LEFT  (should spin CW,  seen from above)",
    "M2 FRONT-RIGHT (should spin CCW, seen from above)",
    "M3 REAR-LEFT   (should spin CCW, seen from above)",
    "M4 REAR-RIGHT  (should spin CW,  seen from above)",
};

void stop_all(void)
{
    motors_stop();
    g_motor = -1;
}

void run_motor(int i)
{
    g_mode = Mode::Motor;
    g_motor = i;
    g_until_ms = millis() + MOTOR_RUN_MS;
    Serial.printf("\n>>> %s\n", MOTOR_NAME[i]);
    Serial.printf("    spinning at %.0f%% for %.1fs — PROPS OFF\n",
                  g_duty * 100.0f, MOTOR_RUN_MS / 1000.0f);
    Serial.println("    watch: is it the motor you expected, turning the way "
                   "the belly silk says?");
}

void run_all(void)
{
    g_mode = Mode::MotorAll;
    g_motor = -1;
    g_all_start_ms = millis();
    g_vmin = 99.0f;
    g_vmin_duty = 0.0f;
    Serial.printf("\n>>> ALL FOUR MOTORS — ramping to %.0f%% over %.1fs, "
                  "holding to %.1fs\n", g_duty * 100.0f,
                  ALL_RAMP_MS / 1000.0f, ALL_RUN_MS / 1000.0f);
    Serial.println("    PROPS OFF. This is the first time this board has ever "
                   "run four at once.");
    Serial.println("    watch: does the pack hold up, and does the board "
                   "reset?\n");
}

void orientation_report(const BenchDeps &d)
{
    ImuSample s = d.imu->read();
    if (!s.valid) { Serial.println("IMU read failed"); return; }
    EulerAngles e = d.ahrs->euler();

    Serial.printf("  accel % 6.2f % 6.2f % 6.2f g   "
                  "gyro % 7.1f % 7.1f % 7.1f dps   "
                  "att roll % 6.1f pitch % 6.1f\n",
                  s.ax, s.ay, s.az, s.gx, s.gy, s.gz, e.roll, e.pitch);
}

}  /* namespace */

void bench_print_loop_timing(void)
{
    /* config.h declares LOOP_HZ. This prints whether the aircraft actually
     * achieved it, as a distribution rather than an average — an average is
     * exactly the statistic a stall hides inside. */
    const LoopStats &s = g_loop_stats;
    if (s.count() == 0) { Serial.println("  loop        (no samples yet)"); return; }

    Serial.printf("  loop        budget %luus  mean %luus  min %luus  "
                  "WORST %luus\n",
                  (unsigned long)s.budget_us(), (unsigned long)s.mean_us(),
                  (unsigned long)s.min_us(), (unsigned long)s.max_us());
    Serial.printf("              p50 <=%luus  p99 <=%luus  p99.9 <=%luus\n",
                  (unsigned long)s.percentile_us(0.50f),
                  (unsigned long)s.percentile_us(0.99f),
                  (unsigned long)s.percentile_us(0.999f));
    Serial.printf("              overruns %lu / %lu  (%lu ppm)  %s\n",
                  (unsigned long)s.overruns(), (unsigned long)s.count(),
                  (unsigned long)s.overrun_ppm(),
                  s.overruns() == 0 ? "(budget met)"
                                    : "(budget MISSED — see histogram)");

    Serial.println("              period histogram:");
    for (int i = 0; i < LOOPSTATS_BUCKETS; i++) {
        const uint32_t n = s.bucket(i);
        if (n == 0) continue;
        const uint32_t edge = LoopStats::bucket_edge_us(i);
        char label[16];
        if (edge == UINT32_MAX) snprintf(label, sizeof(label), "  >10000");
        else                    snprintf(label, sizeof(label), "  <=%5lu",
                                         (unsigned long)edge);
        /* One bar character per 2% of samples, so the shape is readable on a
         * 80-column serial console without any host-side tooling. */
        const uint32_t pct = (uint32_t)(((uint64_t)n * 100ULL) / s.count());
        Serial.printf("            %sus %7lu  %2lu%% ", label,
                      (unsigned long)n, (unsigned long)pct);
        for (uint32_t b = 0; b < pct / 2; b++) Serial.print('#');
        Serial.println("");
    }
}

/* Dump the flight log as CSV over USB, oldest record first.
 *
 * The blackbox already serves over WiFi at 192.168.4.1:82, but reading it that
 * way means leaving your own network and joining the aircraft's AP after every
 * single hop. That friction is why flight logs go unread. On the bench the USB
 * cable is already attached, so this is the path that will actually get used.
 *
 * Scales are the inverse of bb_scale_* in blackbox.h. Emitted as text rather
 * than the packed 40-byte record because the point is to paste it into
 * something that plots, not to save four bytes. */
void blackbox_dump_csv(void)
{
    const uint32_t n = blackbox_count();
    if (n == 0) {
        Serial.println("blackbox empty — nothing recorded yet. Recording "
                       "starts on arming and stops on landing, so a bench "
                       "session that never armed has no log.");
        return;
    }

    Serial.printf("\n#PURSUIT-1 blackbox, %lu records, %lu overwritten\n",
                  (unsigned long)n, (unsigned long)blackbox_dropped());
    Serial.println("t_s,gx,gy,gz,roll,pitch,yaw,sp_roll,sp_pitch,sp_yawrate,"
                   "pid_r,pid_p,pid_y,m1,m2,m3,m4,thr,vbat,alt_mm,state,"
                   "flags,loop_us");

    BlackboxRecord r;
    for (uint32_t i = 0; i < n; i++) {
        if (!blackbox_peek(i, &r)) break;
        Serial.printf(
            "%.4f,%.2f,%.2f,%.2f,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f,"
            "%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f,%u,%u,%u,%u\n",
            r.t_us * 1e-6,
            r.gyro[0] * 2000.0f / 32767.0f, r.gyro[1] * 2000.0f / 32767.0f,
            r.gyro[2] * 2000.0f / 32767.0f,
            r.att[0] * 180.0f / 32767.0f, r.att[1] * 180.0f / 32767.0f,
            r.att[2] * 180.0f / 32767.0f,
            r.sp[0] * 180.0f / 32767.0f, r.sp[1] * 180.0f / 32767.0f,
            r.sp[2] * 2000.0f / 32767.0f,
            r.pid[0] / 32767.0f, r.pid[1] / 32767.0f, r.pid[2] / 32767.0f,
            r.motor[0] / 255.0f, r.motor[1] / 255.0f, r.motor[2] / 255.0f,
            r.motor[3] / 255.0f, r.throttle / 255.0f, r.vbat / 50.0f,
            (unsigned)r.alt_mm, (unsigned)r.state, (unsigned)r.flags,
            (unsigned)r.loop_us);
        /* The USB CDC buffer is not deep enough for 500Hz of text arriving in
         * a tight loop. Yielding periodically lets it drain rather than drop
         * lines — a log with silent holes is worse than no log. */
        if ((i & 0x3F) == 0) delay(1);
    }
    Serial.printf("#end (%lu records)\n\n", (unsigned long)n);
}

/* Drive each motor gate pin as a plain GPIO, high, one at a time.
 *
 * Added 2026-08-20, when all four motors went silent at once and a meter read
 * 0V across M1 with VBAT healthy at 3.6V on J1. That narrows it to gate drive,
 * but not to a cause: a dead MCU pin, a broken header joint and a
 * misconfigured LEDC peripheral all look identical from the motor's end.
 *
 * This bypasses LEDC entirely. digitalWrite through the Arduino GPIO path is
 * about as simple as the ESP32 gets, so if a pin still will not go high the
 * fault is the pin or the joint, not the PWM setup. And a steady 3.3V DC is
 * far easier to read on a multimeter than the ~1.15V average of a 35% duty
 * cycle, which is easy to mistake for noise.
 *
 * Restores LEDC afterwards -- motors_begin() reattaches the channels, and the
 * pins are left low. */
void gate_pin_test(void)
{
    const int pins[4] = {PIN_M1_PWM, PIN_M2_PWM, PIN_M3_PWM, PIN_M4_PWM};
    const char *names[4] = {"M1 (D0)", "M2 (D1)", "M3 (D3)", "M4 (D8)"};

    Serial.println("\n>>> GATE PIN TEST — PROPS OFF");
    Serial.println("    Each pin is driven HIGH for 4s. Meter it against GND:");
    Serial.println("      ~3.3V  the MCU is driving — fault is downstream");
    Serial.println("      ~0V    nothing coming out of this pin\n");

    for (int i = 0; i < 4; i++) {
        pinMode(pins[i], OUTPUT);
        digitalWrite(pins[i], HIGH);
        Serial.printf("    %s  HIGH  (gpio %d) — measure now\n",
                      names[i], pins[i]);
        delay(4000);
        digitalWrite(pins[i], LOW);
    }

    /* Hand the pins back to the PWM peripheral. */
    motors_begin();
    motors_stop();
    Serial.println("\n    done — LEDC restored, motors stopped\n");
}

void bench_print_help(void)
{
    Serial.println("\n=================== BENCH TEST — PROPS OFF ==========="
                   "=======");
    Serial.println(" 1 2 3 4   spin one motor for 2.5s");
    Serial.println(" a         spin M1 only (2.5s)");
    Serial.println(" A         spin ALL FOUR together, ramped — the real "
                   "power-path test");
    Serial.println(" o         orientation check (tilt the board, watch the "
                   "numbers)");
    Serial.println(" s         sensor + battery + link report");
    Serial.println(" e         camera exposure: flight (dark, blur-proof) "
                   "<-> bench (bright)");
    Serial.println(" v         video: close (SVGA, face ID) <-> range (VGA, "
                   "35% less bitrate)");
    Serial.println(" +  -      motor test duty up / down");
    Serial.println(" d         dump the flight log as CSV over USB");
    Serial.println(" p         loop section profile (which part is slow)");
    Serial.println(" g         drive each motor gate pin HIGH (meter it)");
    Serial.println(" x         stop everything");
    Serial.println(" h         this help");
    Serial.println("");
    Serial.println(" What you are checking:");
    Serial.println("  * the motor that spins is the one named");
    Serial.println("  * it turns the direction the belly silk says");
    Serial.println("    (to reverse a brushed motor, swap its two wires)");
    Serial.println("  * tilting the RIGHT edge DOWN gives POSITIVE roll");
    Serial.println("  * tilting the NOSE DOWN gives POSITIVE pitch");
    Serial.println("    (yes, nose DOWN — the protocol and the filter both");
    Serial.println("     use nose-down-positive; see src/frame.h)");
    Serial.println("  * a wrong sign here means every correction in flight "
                   "is backwards");
    Serial.println("======================================================"
                   "=======\n");
}

bool bench_key(char c, const BenchDeps &d)
{
    /* Hard gate. Anything other than IDLE means the aircraft is armed,
     * flying, or in failsafe, and none of those are a bench. */
    if (d.state->state() != PURSUIT_STATE_IDLE &&
        d.state->state() != PURSUIT_STATE_BOOT) {
        if (c == 'x') { stop_all(); g_mode = Mode::Off; return true; }
        Serial.println("bench test refused: aircraft is not IDLE");
        return true;
    }

    switch (c) {
    case '1': case '2': case '3': case '4':
        run_motor(c - '1');
        return true;
    case 'a':
        run_motor(0);
        g_mode = Mode::Motor;
        return true;
    case 'A':
        run_all();
        return true;
    case 'o':
        g_mode = Mode::Orientation;
        Serial.println("\n>>> ORIENTATION — tilt the board slowly and watch:");
        Serial.println("    RIGHT edge down -> roll POSITIVE");
        Serial.println("    NOSE down       -> pitch POSITIVE");
        Serial.println("    wrong sign? fix IMU_ORIENT_* in config.h, "
                       "NOT the filter");
        Serial.println("    press x to stop\n");
        return true;
    case 's':
        g_mode = Mode::Sensors;
        return true;
    case 'v': {
        /* Video is a couple of Mbps against ~7 kbps of command traffic, so it
         * is what collapses first as range opens up. RANGE trades the
         * resolution that face recognition needs -- and which stops working
         * past 3.4 m anyway -- for a measured 35% CUT in bitrate. Not
         * persisted: a reboot comes back at SVGA. */
        const bool to_range = !camera_range_mode();
        if (!camera_set_range_mode(to_range)) {
            Serial.println("video: no camera");
        } else {
            if (to_range)
                Serial.printf("video: RANGE  VGA 640x480 q%d  ~35%% less "
                              "bitrate  <- NO face ID\n", CAM_QUALITY_RANGE);
            else
                Serial.printf("video: CLOSE  SVGA 800x600 q%d  face ID to "
                              "3.4m\n", CAM_QUALITY_CLOSE);
        }
        return true;
    }
    case 'e': {
        /* Flight biases hard toward a SHORT exposure because motion blur is
         * what kills the detector, and pays for it with gain. On a bench
         * nothing is moving, so that same setting only makes a backlit
         * subject unreadable. This flips between the two; it is not
         * persisted, so a reboot always comes back flight-ready. */
        const bool to_bench = camera_exposure_bias() != CAM_AE_LEVEL_BENCH;
        const int want = to_bench ? CAM_AE_LEVEL_BENCH : CAM_AE_LEVEL_FLIGHT;
        if (!camera_set_exposure_bias(want)) {
            Serial.println("exposure: no camera");
        } else {
            Serial.printf("exposure: %s (ae_level %+d)%s\n",
                          to_bench ? "BENCH — brighter" : "FLIGHT — short",
                          want,
                          to_bench ? "   <- do NOT enrol a face like this;"
                                     " match what the aircraft will see"
                                   : "");
        }
        return true;
    }
    case '+':
        g_duty = fminf(g_duty + 0.03f, 0.35f);
        Serial.printf("motor test duty %.0f%%\n", g_duty * 100.0f);
        return true;
    case '-':
        g_duty = fmaxf(g_duty - 0.03f, 0.05f);
        Serial.printf("motor test duty %.0f%%\n", g_duty * 100.0f);
        return true;
    case 'x':
        stop_all();
        g_mode = Mode::Off;
        Serial.println("bench test stopped");
        return true;
    case 'd':
        blackbox_dump_csv();
        return true;
    case 'p':
        prof_report();
        prof_reset();
        return true;
    case 'g':
        gate_pin_test();
        return true;
    case 'h':
        bench_print_help();
        return true;
    default:
        return false;
    }
}

void bench_update(uint32_t now_ms, const BenchDeps &d)
{
    if (g_mode == Mode::Off) return;

    /* Any real command means someone is flying this thing. Bench mode loses
     * immediately rather than arguing about it. */
    if (d.state->state() != PURSUIT_STATE_IDLE &&
        d.state->state() != PURSUIT_STATE_BOOT) {
        bench_abort("aircraft left IDLE");
        return;
    }

    switch (g_mode) {
    case Mode::Motor: {
        if (g_motor < 0) { g_mode = Mode::Off; return; }
        if ((int32_t)(now_ms - g_until_ms) >= 0) {
            stop_all();
            g_mode = Mode::Off;
            Serial.println("    done. next: 1/2/3/4, or x\n");
            return;
        }
        MotorOutputs m{0, 0, 0, 0};
        float *out[4] = {&m.m1, &m.m2, &m.m3, &m.m4};
        *out[g_motor] = g_duty;
        motors_write(m);
        /* Print the battery WHILE the motor is actually turning. Polling it
         * with 's' cannot work: that key switches the bench out of motor mode
         * and stops the motor, so the sample is always taken with the load
         * already removed. Cost a wasted bring-up session on 2026-08-19 —
         * the reading looked reassuringly flat because nothing was loading
         * the pack at the instant it was sampled. */
        if (now_ms - g_last_print >= 200) {
            g_last_print = now_ms;
            Serial.printf("    MOTORLOAD vbat slow %.2f  fast %.2f  duty %.0f%%\n",
                          d.battery->volts(), d.battery->instant_volts(),
                          g_duty * 100.0f);
        }
        break;
    }

    case Mode::MotorAll: {
        const uint32_t el = now_ms - g_all_start_ms;
        if (el >= ALL_RUN_MS) {
            stop_all();
            g_mode = Mode::Off;
            Serial.printf("\n    done. lowest pack voltage %.2f V at %.0f%% "
                          "per motor\n", g_vmin, g_vmin_duty * 100.0f);
            Serial.printf("    critical threshold is %.2f V — %s\n\n",
                          VBAT_CRITICAL_V,
                          g_vmin < VBAT_CRITICAL_V
                              ? "TRIPPED. four-motor flight is not possible "
                                "on this pack."
                              : "held. the pack survives four motors here.");
            return;
        }

        /* Linear ramp, then hold. */
        const float frac = (el < ALL_RAMP_MS)
            ? (float)el / (float)ALL_RAMP_MS : 1.0f;
        const float duty = g_duty * frac;

        MotorOutputs m{duty, duty, duty, duty};
        motors_write(m);

        const float v = d.battery->instant_volts();
        if (v < g_vmin) { g_vmin = v; g_vmin_duty = duty; }

        if (v < ALL_ABORT_V) {
            stop_all();
            g_mode = Mode::Off;
            Serial.printf("\n    ABORTED at %.0f%% — pack fell to %.2f V "
                          "(floor %.2f V)\n", duty * 100.0f, v, ALL_ABORT_V);
            Serial.println("    that IS the result: this pack cannot carry "
                           "four motors this far.\n");
            return;
        }

        if (now_ms - g_last_print >= 200) {
            g_last_print = now_ms;
            Serial.printf("    ALLMOTOR vbat slow %.2f  fast %.2f  duty %.0f%% "
                          "each\n", d.battery->volts(), v, duty * 100.0f);
        }
        break;
    }

    case Mode::Orientation:
        if (now_ms - g_last_print >= 250) {
            g_last_print = now_ms;
            orientation_report(d);
        }
        break;

    case Mode::Sensors: {
        ImuSample s = d.imu->read();
        Serial.println("\n--- sensor report ---");
        Serial.printf("  IMU        %s   who_am_i 0x%02X\n",
                      s.valid ? "OK" : "FAILED", d.imu->who_am_i());
        /* Bus health. Read this WITH A MOTOR SPINNING (press A, then s) —
         * idle numbers are always clean and tell you nothing. */
        {
            const uint32_t r = d.imu->reads();
            const uint32_t f = d.imu->read_failures();
            Serial.printf("  I2C        %lu reads, %lu failed (%.2f%%), "
                          "worst run %lu\n",
                          (unsigned long)r, (unsigned long)f,
                          r ? (100.0f * (float)f / (float)r) : 0.0f,
                          (unsigned long)d.imu->worst_consecutive_failures());
        }
        if (s.valid) {
            const float g = sqrtf(s.ax * s.ax + s.ay * s.ay + s.az * s.az);
            Serial.printf("  gravity    %.2f g  %s\n", g,
                          (g > 0.85f && g < 1.15f) ? "(plausible)"
                          : "(WRONG — check accel scaling or mounting)");
            float bias[3];
            d.imu->gyro_bias(bias);
            Serial.printf("  gyro bias  %.2f %.2f %.2f dps  %s\n",
                          bias[0], bias[1], bias[2],
                          d.imu->calibrated() ? "(calibrated)"
                                              : "(NOT CALIBRATED)");
        }
        Serial.printf("  battery    %.2f V  %s\n", d.battery->volts(),
                      d.battery->volts() < 3.5f ? "(LOW — charge before "
                                                  "flying)" : "");
        /* Rejected ADC samples. Zero at rest and climbing under motor PWM is
         * the expected shape — see VBAT_SANE_MIN_V. Non-zero at REST would
         * mean something new and worse. */
        Serial.printf("  vbat ADC   %lu glitches rejected\n",
                      (unsigned long)d.battery->glitches());
        Serial.printf("  rangefinder %s\n",
                      d.tof->available() ? "OK" : "absent (no altitude hold)");
        if (d.baro && d.baro->present()) {
            /* Report the SPREAD over a second, not one reading. The absolute
             * pressure altitude is meaningless — it moves tens of metres with
             * the weather and the fusion filter subtracts it anyway. What
             * matters on a bench is how much the number wanders, because
             * that noise is what the altitude loop will be chasing. Run this
             * once with the props still, then again with them spinning: if
             * the spread grows, the foam is not doing its job. */
            float lo = 1e9f, hi = -1e9f, sum = 0.0f;
            int n = 0;
            const uint32_t until = millis() + 1000;
            while ((int32_t)(millis() - until) < 0) {
                Bmp390Sample b = d.baro->read();
                if (b.valid) {
                    if (b.altitude_m < lo) lo = b.altitude_m;
                    if (b.altitude_m > hi) hi = b.altitude_m;
                    sum += b.altitude_m;
                    n++;
                }
                delay(5);
            }
            if (n > 0) {
                Serial.printf("  barometer  OK @0x%02X   %d samples/s, "
                              "%.2fm spread\n",
                              d.baro->address(), n, hi - lo);
                Serial.printf("             mean %.1fm (pressure altitude — "
                              "absolute value is meaningless)\n", sum / n);
                if (hi - lo > 1.0f) {
                    Serial.println("             SPREAD TOO WIDE: cover the "
                                   "port with open-cell foam, and keep light "
                                   "off it");
                }
            } else {
                Serial.println("  barometer  PRESENT BUT SILENT — probed OK "
                               "and returns no conversions");
            }
        } else {
            Serial.printf("  barometer  absent (%s) — ceiling %.1fm\n",
                          d.baro ? d.baro->last_error() : "not wired",
                          ALT_CEILING_TOF_M);
        }
        Serial.printf("  camera     %s   exposure %+d (%s)   video %s\n",
                      camera_ready() ? "OK" : "absent",
                      camera_exposure_bias(),
                      camera_exposure_bias() == CAM_AE_LEVEL_BENCH
                          ? "bench" : "flight",
                      camera_range_mode() ? "RANGE (VGA)" : "CLOSE (SVGA)");
        bench_print_loop_timing();
        Serial.println("---------------------\n");
        g_mode = Mode::Off;
        break;
    }

    default:
        break;
    }
}

bool bench_active(void)
{
    return g_mode == Mode::Motor;   /* only motor tests own the outputs */
}

void bench_abort(const char *why)
{
    if (g_mode == Mode::Off) return;
    stop_all();
    g_mode = Mode::Off;
    Serial.printf("bench test aborted: %s\n", why);
}

#else  /* PURSUIT_HOST_BUILD */

bool bench_key(char, const BenchDeps &) { return false; }
void bench_update(uint32_t, const BenchDeps &) {}
bool bench_active(void) { return false; }
void bench_abort(const char *) {}
void bench_print_help(void) {}

#endif
