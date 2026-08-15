#include "blackbox.h"

#include <cmath>

/* --------------------------------------------------------------- scaling --
 * Shared with the Python decoder (tools/decode_blackbox.py). Kept here as
 * plain functions so the host tests can round-trip them — a scale mismatch
 * between writer and decoder produces a plot that looks plausible and is
 * wrong by a constant factor, which is the hardest kind of wrong to notice.
 */
int16_t bb_scale_gyro(float dps)
{
    if (!std::isfinite(dps)) return 0;
    float v = dps * (32767.0f / 2000.0f);
    if (v > 32767.0f) v = 32767.0f;
    if (v < -32768.0f) v = -32768.0f;
    return (int16_t)lrintf(v);
}

int16_t bb_scale_angle(float deg)
{
    if (!std::isfinite(deg)) return 0;
    float v = deg * (32767.0f / 180.0f);
    if (v > 32767.0f) v = 32767.0f;
    if (v < -32768.0f) v = -32768.0f;
    return (int16_t)lrintf(v);
}

int16_t bb_scale_unit(float u)
{
    if (!std::isfinite(u)) return 0;
    float v = u * 32767.0f;
    if (v > 32767.0f) v = 32767.0f;
    if (v < -32768.0f) v = -32768.0f;
    return (int16_t)lrintf(v);
}

uint8_t bb_scale_duty(float d)
{
    if (!std::isfinite(d) || d <= 0.0f) return 0;
    if (d >= 1.0f) return 255;
    return (uint8_t)lrintf(d * 255.0f);
}

uint8_t bb_scale_volts(float v)
{
    if (!std::isfinite(v) || v <= 0.0f) return 0;
    float s = v * 50.0f;              /* 4.20V -> 210, fits to 5.1V */
    if (s > 255.0f) s = 255.0f;
    return (uint8_t)lrintf(s);
}

#ifndef PURSUIT_HOST_BUILD

#include <Arduino.h>
#include <WiFi.h>

namespace {

BlackboxRecord *g_buf = nullptr;
uint32_t g_cap = 0;                  /* records the buffer holds */
volatile uint32_t g_head = 0;        /* next write index         */
volatile uint32_t g_count = 0;       /* records held (capped)    */
volatile uint32_t g_dropped = 0;
volatile bool g_recording = false;
uint32_t g_sample_hz = 500;
WiFiServer *g_srv = nullptr;

void serve_task(void *)
{
    for (;;) {
        if (!g_srv) { vTaskDelay(pdMS_TO_TICKS(200)); continue; }
        WiFiClient c = g_srv->available();
        if (!c) { vTaskDelay(pdMS_TO_TICKS(20)); continue; }

        uint32_t t0 = millis();
        while (c.connected() && !c.available() && millis() - t0 < 1000)
            vTaskDelay(pdMS_TO_TICKS(2));
        while (c.available()) c.read();

        const uint32_t n = g_count;
        BlackboxHeader h{};
        h.magic[0] = 'P'; h.magic[1] = 'B'; h.magic[2] = 'B'; h.magic[3] = '1';
        h.version = 1;
        h.record_size = sizeof(BlackboxRecord);
        h.sample_hz = g_sample_hz;
        h.records = n;
        h.dropped = g_dropped;

        const uint32_t bytes = sizeof(h) + n * sizeof(BlackboxRecord);
        c.printf("HTTP/1.1 200 OK\r\n"
                 "Content-Type: application/octet-stream\r\n"
                 "Content-Disposition: attachment; filename=\"flight.pbb\"\r\n"
                 "Content-Length: %u\r\n\r\n", (unsigned)bytes);
        c.write((const uint8_t *)&h, sizeof(h));

        if (g_buf && n) {
            /* Oldest first. The ring may have wrapped, in which case the
             * oldest record sits just ahead of the write head. */
            const uint32_t start = (g_count >= g_cap) ? g_head : 0;
            for (uint32_t i = 0; i < n; i += 64) {
                const uint32_t chunk = (n - i) < 64 ? (n - i) : 64;
                for (uint32_t j = 0; j < chunk; j++) {
                    const uint32_t idx = (start + i + j) % g_cap;
                    c.write((const uint8_t *)&g_buf[idx],
                            sizeof(BlackboxRecord));
                }
                vTaskDelay(1);   /* let WiFi breathe between chunks */
            }
        }
        c.stop();
    }
}

}  /* namespace */

bool blackbox_begin(uint32_t sample_hz, uint32_t seconds)
{
    g_sample_hz = sample_hz ? sample_hz : 500;

    if (!psramFound()) {
        /* No PSRAM: no logging. Deliberately NOT falling back to internal
         * RAM — a few hundred records is not a flight log, and stealing heap
         * from the WiFi stack to get them is a poor trade. */
        g_buf = nullptr;
        g_cap = 0;
        return false;
    }

    /* Ask for what was requested, but never for more than exists.
     *
     * The default request is 500Hz for 420s, which is 8,400,000 bytes against
     * 8MiB of PSRAM = 8,388,608. It does not fit, and it did not fit before
     * the camera took its framebuffers either. A single ps_malloc of that
     * size returns null, and the old code treated that exactly like having no
     * PSRAM at all: no log, one line of boot text, and a tuning session spent
     * wondering why nothing is being recorded.
     *
     * So the buffer is SIZED TO WHAT IS THERE. A shorter log is a log; a
     * failed allocation is not. Callers read blackbox_capacity() to find out
     * what they actually got, and main.cpp prints the real duration rather
     * than the one that was asked for.
     */
    const size_t rec = sizeof(BlackboxRecord);
    uint64_t want = (uint64_t)g_sample_hz * seconds;

    /* Headroom. The camera has already claimed its framebuffers by the time
     * this runs, but WiFi still grows under load, and a PSRAM allocator with
     * nothing left is a brownout waiting for a busy moment. */
    const size_t RESERVE = 512u * 1024u;
    const size_t free_ps = ESP.getFreePsram();
    const size_t usable = free_ps > RESERVE ? free_ps - RESERVE : 0;
    const uint64_t fits = usable / rec;
    if (want > fits) want = fits;

    /* Back off rather than give up: fragmentation can defeat an allocation
     * that the free-byte count says should succeed. */
    while (want >= 2000) {
        g_buf = (BlackboxRecord *)ps_malloc((size_t)want * rec);
        if (g_buf) {
            g_cap = (uint32_t)want;
            g_head = g_count = g_dropped = 0;
            return true;
        }
        want = want * 3 / 4;
    }

    g_buf = nullptr;
    g_cap = 0;
    return false;
}

void blackbox_push(const BlackboxRecord &r)
{
    /* The one function that runs at loop rate. No locks, no allocation, no
     * branches that can block — a full buffer overwrites the oldest record
     * rather than stalling the caller. */
    if (!g_recording || !g_buf || !g_cap) return;

    const uint32_t idx = g_head;
    g_buf[idx] = r;
    g_head = (idx + 1) % g_cap;
    if (g_count < g_cap) g_count++;
    else g_dropped++;          /* wrapped: oldest history is being lost */
}

void blackbox_start(void)
{
    if (!g_buf) return;
    g_head = g_count = g_dropped = 0;
    g_recording = true;
}

void blackbox_stop(void) { g_recording = false; }
bool blackbox_recording(void) { return g_recording; }

void blackbox_serve(uint16_t port)
{
    if (!g_buf) return;
    g_srv = new WiFiServer(port);
    g_srv->begin();
    /* Core 0 with the rest of the networking, so a download in progress can
     * never delay the attitude loop on core 1. */
    xTaskCreatePinnedToCore(serve_task, "bbserve", 4096, nullptr, 1, nullptr,
                            0);
}

uint32_t blackbox_count(void) { return g_count; }
uint32_t blackbox_dropped(void) { return g_dropped; }
uint32_t blackbox_capacity(void) { return g_cap; }

#else  /* PURSUIT_HOST_BUILD — a plain heap ring, so the tests can drive it */

#include <cstdlib>
#include <cstring>

static BlackboxRecord *g_buf = nullptr;
static uint32_t g_cap = 0, g_head = 0, g_count = 0, g_dropped = 0;
static bool g_recording = false;
static uint32_t g_sample_hz = 500;

bool blackbox_begin(uint32_t sample_hz, uint32_t seconds)
{
    g_sample_hz = sample_hz ? sample_hz : 500;
    free(g_buf);
    g_cap = g_sample_hz * seconds;
    g_buf = (BlackboxRecord *)calloc(g_cap, sizeof(BlackboxRecord));
    g_head = g_count = g_dropped = 0;
    return g_buf != nullptr;
}

void blackbox_push(const BlackboxRecord &r)
{
    if (!g_recording || !g_buf || !g_cap) return;
    g_buf[g_head] = r;
    g_head = (g_head + 1) % g_cap;
    if (g_count < g_cap) g_count++;
    else g_dropped++;
}

void blackbox_start(void) { if (g_buf) { g_head = g_count = g_dropped = 0; g_recording = true; } }
void blackbox_stop(void) { g_recording = false; }
bool blackbox_recording(void) { return g_recording; }
void blackbox_serve(uint16_t) {}
uint32_t blackbox_count(void) { return g_count; }
uint32_t blackbox_dropped(void) { return g_dropped; }
uint32_t blackbox_capacity(void) { return g_cap; }

/* Test-only peek at a stored record, oldest-first. */
bool blackbox_peek(uint32_t i, BlackboxRecord *out)
{
    if (!g_buf || i >= g_count) return false;
    const uint32_t start = (g_count >= g_cap) ? g_head : 0;
    *out = g_buf[(start + i) % g_cap];
    return true;
}

#endif
