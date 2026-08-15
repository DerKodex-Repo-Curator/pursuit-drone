#include "benchtest.h"

#ifndef PURSUIT_HOST_BUILD

#include <Arduino.h>
#include <cmath>

#include "battery.h"
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

enum class Mode { Off, Motor, Orientation, Sensors };

Mode g_mode = Mode::Off;
int  g_motor = -1;              /* which motor is spinning, 0-3   */
float g_duty = 0.12f;           /* low enough not to walk the board */
uint32_t g_until_ms = 0;
uint32_t g_last_print = 0;

/* Deliberately brief. A motor left spinning while you look away is how a
 * bare shaft finds a finger. */
constexpr uint32_t MOTOR_RUN_MS = 2500;

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

void bench_print_help(void)
{
    Serial.println("\n=================== BENCH TEST — PROPS OFF ==========="
                   "=======");
    Serial.println(" 1 2 3 4   spin one motor for 2.5s");
    Serial.println(" a         spin all four in sequence");
    Serial.println(" o         orientation check (tilt the board, watch the "
                   "numbers)");
    Serial.println(" s         sensor + battery + link report");
    Serial.println(" e         camera exposure: flight (dark, blur-proof) "
                   "<-> bench (bright)");
    Serial.println(" +  -      motor test duty up / down");
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
        Serial.printf("  camera     %s   exposure %+d (%s)\n",
                      camera_ready() ? "OK" : "absent",
                      camera_exposure_bias(),
                      camera_exposure_bias() == CAM_AE_LEVEL_BENCH
                          ? "bench" : "flight");
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
