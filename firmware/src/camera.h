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

/* Exposure bias, -2..2, straight through to the sensor's AE target.
 *
 * Flight and bench want opposite things and the difference is about two
 * stops. In the air the enemy is MOTION BLUR — a yawing quadcopter smears
 * every edge, and the detector keys on edges — so the right answer is a short
 * exposure paid for with gain, accepting a dark noisy frame. On a desk
 * nothing is moving, there is no blur to avoid, and the same setting just
 * makes a backlit subject unreadable.
 *
 * Deliberately not persisted and not reachable over the link: it is a bench
 * convenience, and the flight value is the one that must survive a reboot.
 */
bool camera_set_exposure_bias(int level);
int camera_exposure_bias(void);

/* Trade resolution for link margin.
 *
 * The video stream is a couple of Mbps against ~7 kbps of command traffic, so
 * video is what collapses first as range opens up. RANGE mode (VGA, coarser
 * quantiser) measured 35% less: 2.61 -> 1.70 Mbps at an unchanged ~25 fps.
 *
 * The cost is face recognition, which needs ~28 px across a face and
 * therefore stops working past 3.4 m regardless — so at the distance where
 * you would reach for this, the resolution was buying nothing. Body re-ID and
 * person detection are unaffected: a person is still ~88 px tall at 10 m.
 *
 * NOT persisted. A reboot comes back at SVGA, because the close-in work the
 * aircraft exists for is the work that needs it.
 */
bool camera_set_range_mode(bool on);
bool camera_range_mode(void);

#endif /* PURSUIT_CAMERA_H */
