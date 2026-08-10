/* OV2640 capture and MJPEG streaming for the off-board vision pipeline.
 *
 * Runs as a FreeRTOS task pinned to **core 0**, while the flight control loop
 * owns core 1. That split is the whole design:
 *
 *   core 0   WiFi stack, camera driver, JPEG encode, TCP writes
 *   core 1   Arduino loop() — IMU, attitude, PID, mixer, PWM
 *
 * A single JPEG encode is a few milliseconds and a blocked TCP write can be
 * tens. On a shared core either would stall the 1kHz attitude loop, and an
 * attitude loop that misses ten milliseconds on a 66g quadcopter is an
 * attitude loop that has already lost the aircraft. Pinning is not tidiness;
 * it is what makes streaming survivable at all.
 *
 * The camera sits on the XIAO Sense's own internal connector, not on any
 * header pin — which is why none of these GPIOs appear in design.py.
 */
#ifndef PURSUIT_CAMERA_H
#define PURSUIT_CAMERA_H

#include <stdint.h>

/* Initialise the sensor and start the streaming server on core 0.
 * Returns false if the camera is absent or fails to initialise — which is
 * NOT fatal. The aircraft flies without vision; it just cannot follow. */
bool camera_begin(uint16_t port = 81);

bool camera_ready(void);
uint32_t camera_frames(void);
uint32_t camera_clients(void);
const char *camera_error(void);

#endif /* PURSUIT_CAMERA_H */
