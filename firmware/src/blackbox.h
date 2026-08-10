/* Flight data recorder.
 *
 * Tuning a PID cascade without a log is guessing. The aircraft oscillates,
 * you change a number, it oscillates differently, and you have no way to tell
 * whether the problem is P, D, a noisy gyro, motor saturation or the airframe
 * flexing. With gyro, setpoint, PID output and motor traces at 500Hz it stops
 * being guesswork.
 *
 * **Not the microSD.** The Sense expansion board wires its card to D8/D9/D10
 * for SCK/MISO/MOSI — which on THIS board are M4_PWM, IMU_INT and LED_CTRL.
 * Logging to SD would clock motor four's gate at SPI rate. So records go to a
 * PSRAM ring buffer and are downloaded over WiFi after landing; 8MB holds
 * roughly a full pack at 500Hz, which is exactly the window that matters.
 *
 * The logger is built so it CANNOT affect flight:
 *
 *   * `push()` never blocks, never allocates and never takes a lock the
 *     control loop could wait on.
 *   * A full buffer drops the sample and increments a counter. A logger that
 *     stalls the attitude loop to preserve a record has inverted its own
 *     priorities.
 *   * All memory is claimed once at boot. If PSRAM is absent, logging is
 *     simply off — the aircraft still flies.
 */
#ifndef PURSUIT_BLACKBOX_H
#define PURSUIT_BLACKBOX_H

#include <stdint.h>

/* Fixed 40-byte record. Values are scaled into integers rather than stored as
 * floats: it halves the size, and at these ranges the quantisation (0.06 deg/s
 * of gyro, 0.006 deg of attitude) is far below the sensor noise anyway. */
struct __attribute__((packed)) BlackboxRecord {
    uint32_t t_us;          /* since boot                                  */
    int16_t  gyro[3];       /* deg/s   * 32767/2000                        */
    int16_t  att[3];        /* deg     * 32767/180                         */
    int16_t  sp[3];         /* roll/pitch deg, yaw deg/s, same scales      */
    int16_t  pid[3];        /* normalised -1..1 * 32767                    */
    uint8_t  motor[4];      /* 0..255                                      */
    uint8_t  throttle;      /* 0..255                                      */
    uint8_t  vbat;          /* volts * 50                                  */
    uint16_t alt_mm;
    uint8_t  state;
    uint8_t  flags;
    uint16_t loop_us;       /* how long the last loop took                 */
};

static_assert(sizeof(BlackboxRecord) == 40, "record must stay 40 bytes");

/* File header, so a decoder can never silently mis-parse a log written by a
 * different firmware build. */
struct __attribute__((packed)) BlackboxHeader {
    char     magic[4];      /* "PBB1"                                      */
    uint16_t version;
    uint16_t record_size;
    uint32_t sample_hz;
    uint32_t records;
    uint32_t dropped;
};

/* Claim the PSRAM buffer. `seconds` sizes it; returns false when there is no
 * PSRAM, which is not fatal. */
bool blackbox_begin(uint32_t sample_hz = 500, uint32_t seconds = 420);

/* Called from the control loop. Must stay cheap — this is the one function
 * that runs at loop rate. */
void blackbox_push(const BlackboxRecord &r);

/* Start/stop recording. Arming starts it; disarming stops it, so a log
 * contains one flight rather than an afternoon of bench idling. */
void blackbox_start(void);
void blackbox_stop(void);
bool blackbox_recording(void);

/* Serve the buffer over HTTP so it can be pulled after landing. */
void blackbox_serve(uint16_t port = 82);

uint32_t blackbox_count(void);
uint32_t blackbox_dropped(void);
uint32_t blackbox_capacity(void);

/* Scaling, shared with tools/decode_blackbox.py. Exposed so the host tests
 * can round-trip them: a mismatch between writer and decoder yields a plot
 * that looks entirely plausible and is wrong by a constant factor, which is
 * the hardest kind of wrong to spot. */
int16_t bb_scale_gyro(float dps);
int16_t bb_scale_angle(float deg);
int16_t bb_scale_unit(float u);
uint8_t bb_scale_duty(float d);
uint8_t bb_scale_volts(float v);

#ifdef PURSUIT_HOST_BUILD
bool blackbox_peek(uint32_t i, BlackboxRecord *out);
#endif

#endif /* PURSUIT_BLACKBOX_H */
