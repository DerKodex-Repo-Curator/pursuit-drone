#include "camera.h"

#ifndef PURSUIT_HOST_BUILD

#include <Arduino.h>
#include <WiFi.h>
#include <esp_camera.h>

#include "config.h"

namespace {

/* XIAO ESP32S3 Sense camera pins. These are on the module's own internal
 * connector, which is why none of them appear in scripts/design.py — the
 * board never routes them. */
constexpr int PIN_XCLK  = 10;
constexpr int PIN_SIOD  = 40;
constexpr int PIN_SIOC  = 39;
constexpr int PIN_VSYNC = 38;
constexpr int PIN_HREF  = 47;
constexpr int PIN_PCLK  = 13;
constexpr int PIN_D7 = 48, PIN_D6 = 11, PIN_D5 = 12, PIN_D4 = 14;
constexpr int PIN_D3 = 16, PIN_D2 = 18, PIN_D1 = 17, PIN_D0 = 15;

bool g_ready = false;
volatile uint32_t g_frames = 0;
volatile uint32_t g_clients = 0;
const char *g_err = "";
int g_ae_level = CAM_AE_LEVEL_FLIGHT;
WiFiServer *g_server = nullptr;

const char *BOUNDARY = "pursuitframe";

void stream_client(WiFiClient &client)
{
    client.print(
        "HTTP/1.1 200 OK\r\n"
        "Access-Control-Allow-Origin: *\r\n"
        "Content-Type: multipart/x-mixed-replace;boundary=pursuitframe\r\n"
        "\r\n");

    while (client.connected()) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) {
            g_err = "frame grab failed";
            break;
        }

        char hdr[96];
        const int n = snprintf(hdr, sizeof(hdr),
                               "--%s\r\nContent-Type: image/jpeg\r\n"
                               "Content-Length: %u\r\n\r\n",
                               BOUNDARY, (unsigned)fb->len);
        client.write((const uint8_t *)hdr, n);
        client.write(fb->buf, fb->len);
        client.print("\r\n");

        esp_camera_fb_return(fb);
        g_frames++;

        /* Yield rather than spin. Even on its own core, starving the WiFi
         * driver's own tasks stalls the very socket we are writing to. */
        vTaskDelay(1);
    }
}

void camera_task(void *)
{
    for (;;) {
        if (!g_server) { vTaskDelay(pdMS_TO_TICKS(100)); continue; }
        WiFiClient client = g_server->available();
        if (client) {
            g_clients++;
            /* Read and discard the request line — the server serves exactly
             * one thing, so parsing the path would only add ways to fail. */
            uint32_t start = millis();
            while (client.connected() && client.available() == 0 &&
                   millis() - start < 1000) {
                vTaskDelay(pdMS_TO_TICKS(2));
            }
            while (client.available()) client.read();

            stream_client(client);
            client.stop();
            if (g_clients) g_clients--;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

}  /* namespace */

bool camera_begin(uint16_t port)
{
    camera_config_t cfg = {};
    cfg.ledc_channel = LEDC_CHANNEL_4;   /* 0-3 belong to the motors */
    cfg.ledc_timer = LEDC_TIMER_1;       /* motors own LEDC_TIMER_0  */
    cfg.pin_d0 = PIN_D0; cfg.pin_d1 = PIN_D1;
    cfg.pin_d2 = PIN_D2; cfg.pin_d3 = PIN_D3;
    cfg.pin_d4 = PIN_D4; cfg.pin_d5 = PIN_D5;
    cfg.pin_d6 = PIN_D6; cfg.pin_d7 = PIN_D7;
    cfg.pin_xclk = PIN_XCLK;
    cfg.pin_pclk = PIN_PCLK;
    cfg.pin_vsync = PIN_VSYNC;
    cfg.pin_href = PIN_HREF;
    cfg.pin_sccb_sda = PIN_SIOD;
    cfg.pin_sccb_scl = PIN_SIOC;
    cfg.pin_pwdn = -1;
    cfg.pin_reset = -1;
    cfg.xclk_freq_hz = 20000000;
    cfg.pixel_format = PIXFORMAT_JPEG;

    /* SVGA (800x600). The *detector* would be happy with QVGA — it letterboxes
     * to 640 anyway — but the FACE needs roughly ten times more pixels on
     * target, and it is the face that acquires the lock.
     *
     * Face width is ~0.15m. At a focal length of (width/2)/tan(hfov/2) the
     * face spans 0.15*f/distance pixels, and ArcFace needs ~28px:
     *
     *     QVGA  320x240   face is 12.6px at 3m   usable only to 1.35m
     *     VGA   640x480   face is 25.1px at 3m   usable to 2.69m
     *     SVGA  800x600   face is 31.4px at 3m   usable to 3.36m  <-- this
     *
     * At QVGA the aircraft could never re-acquire its target at the distance
     * it is trying to hold — it would follow until the first occlusion and
     * then be unable to find them again. SVGA costs bandwidth and a little
     * latency; losing the lock costs the demo.
     *
     * If the link cannot sustain it at range, drop to VGA and lower
     * `face.min_face_px` to 24 — that keeps face acquisition usable to ~3.1m.
     */
    cfg.frame_size = FRAMESIZE_SVGA;
    cfg.jpeg_quality = 12;
    cfg.fb_count = 2;                       /* double-buffer, no tearing */
    cfg.fb_location = CAMERA_FB_IN_PSRAM;
    cfg.grab_mode = CAMERA_GRAB_LATEST;     /* newest frame, never a queue */

    if (!psramFound()) {
        /* Without PSRAM there is not enough RAM for two QVGA buffers, and the
         * allocation failure shows up as a cryptic init error rather than a
         * clear one. */
        cfg.frame_size = FRAMESIZE_QQVGA;
        cfg.fb_count = 1;
        cfg.fb_location = CAMERA_FB_IN_DRAM;
    }

    const esp_err_t e = esp_camera_init(&cfg);
    if (e != ESP_OK) {
        g_err = "esp_camera_init failed";
        g_ready = false;
        return false;
    }

    sensor_t *s = esp_camera_sensor_get();
    if (s) {
        /* The camera hangs under the airframe looking forward; the FPC is
         * bent through 90 degrees, which puts the image upside down. */
        s->set_vflip(s, 1);
        s->set_hmirror(s, 0);
        /* Bias hard toward SHORT exposure, and let gain go high to pay for it.
         *
         * Measured (vision/tools/degradation_sweep.py) on the person detector,
         * reference 2 people:
         *
         *     blur  5px  -> 2      gain 2x, noise sigma 8   -> 2
         *     blur  9px  -> 2      gain 2x, noise sigma 16  -> 2
         *     blur 15px  -> 0      gain 4x, noise sigma 28  -> 2
         *
         * YOLO tolerates a very noisy frame and falls off a cliff with motion
         * blur — it keys on edges, and blur destroys them while noise leaves
         * them intact. A quadcopter yawing to track someone produces blur
         * constantly, so the right trade is a dark noisy frame every time.
         *
         * An earlier version of this capped gain at 4x to avoid noise, which
         * had the priority exactly backwards. */
        s->set_gainceiling(s, GAINCEILING_16X);
        s->set_ae_level(s, CAM_AE_LEVEL_FLIGHT);
        g_ae_level = CAM_AE_LEVEL_FLIGHT;
        s->set_aec2(s, 1);             /* DSP-assisted AEC converges faster */

        /* Auto white balance, explicitly.
         *
         * This was simply never configured, and under tungsten the frames
         * came back with a heavy yellow-green cast. That is not cosmetic
         * here: a face embedding is not colour-invariant, so a cast present
         * at enrolment and absent at match time (or the reverse) costs
         * similarity on every comparison — and the whole identity layer is a
         * threshold on that similarity.
         *
         * The gain control matters as much as the mode: with awb_gain off the
         * sensor computes a correction and then does not apply it. */
        s->set_whitebal(s, 1);
        s->set_awb_gain(s, 1);
        s->set_wb_mode(s, 0);          /* 0 = auto */
    }

    g_server = new WiFiServer(port);
    g_server->begin();

    /* Core 0. loop() runs on core 1, and this must never be able to delay it
     * — see the header. Priority 1 keeps it below the WiFi driver's own
     * tasks so it cannot starve the stack it depends on. */
    xTaskCreatePinnedToCore(camera_task, "camstream", 4096, nullptr, 1,
                            nullptr, 0);

    g_ready = true;
    g_err = "";
    return true;
}

bool camera_set_exposure_bias(int level)
{
    if (level < -2) level = -2;
    if (level > 2) level = 2;
    sensor_t *s = esp_camera_sensor_get();
    if (!s) return false;
    if (s->set_ae_level(s, level) != 0) return false;
    g_ae_level = level;
    return true;
}

int camera_exposure_bias(void) { return g_ae_level; }

bool camera_ready(void) { return g_ready; }
uint32_t camera_frames(void) { return g_frames; }
uint32_t camera_clients(void) { return g_clients; }
const char *camera_error(void) { return g_err; }

#else  /* PURSUIT_HOST_BUILD */

bool camera_begin(uint16_t) { return false; }
bool camera_ready(void) { return false; }
bool camera_set_exposure_bias(int) { return false; }
int camera_exposure_bias(void) { return 0; }
uint32_t camera_frames(void) { return 0; }
uint32_t camera_clients(void) { return 0; }
const char *camera_error(void) { return "host"; }

#endif
