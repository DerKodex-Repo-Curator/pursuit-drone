/* Bench self-test — the checks that stop a first flight ending in ten seconds.
 *
 * On a hand-built quad the usual causes of an immediate flip are not subtle
 * bugs. They are:
 *
 *   * a motor soldered to the wrong pad, so M2's command drives M3
 *   * a motor spinning the wrong way, so its torque adds instead of cancelling
 *   * a CW prop fitted to a CCW motor
 *   * the IMU reading an axis inverted, so every correction is backwards
 *   * one dead FET, so three motors lift and the fourth does nothing
 *
 * Every one of them is obvious on a bench and invisible until the moment the
 * aircraft leaves the ground. This mode makes them visible.
 *
 * Driven over the serial console with props OFF. It is hard-gated: it refuses
 * to run unless the aircraft is IDLE, and any command arriving over the link
 * cancels it immediately. A test mode that could spin a motor mid-flight
 * would be a worse hazard than the bugs it finds.
 */
#ifndef PURSUIT_BENCHTEST_H
#define PURSUIT_BENCHTEST_H

#include <stdint.h>

class FlightState;
class Icm42688;
class Madgwick;
class Battery;
class Rangefinder;
class Bmp390;

struct BenchDeps {
    FlightState *state;
    Icm42688    *imu;
    Madgwick    *ahrs;
    Battery     *battery;
    Rangefinder *tof;
    Bmp390      *baro;
};

/* Feed one serial character. Returns true if it was consumed as a command. */
bool bench_key(char c, const BenchDeps &d);

/* Call every loop. Owns the motors only while a test is running. */
void bench_update(uint32_t now_ms, const BenchDeps &d);

/* True while a test has the motors — the main loop must not drive them. */
bool bench_active(void);

/* Abort immediately. Called whenever a link command arrives. */
void bench_abort(const char *why);

void bench_print_help(void);

/* Loop-period distribution against the LOOP_HZ budget: mean, worst case,
 * percentiles, overrun count and an ASCII histogram. Included in the 's'
 * report and printable on its own. */
void bench_print_loop_timing(void);

#endif /* PURSUIT_BENCHTEST_H */
