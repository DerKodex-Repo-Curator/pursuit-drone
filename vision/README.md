# PURSUIT-1 — off-board vision

The half of the drone that does not fit on the drone. The aircraft streams
camera frames to a laptop over WiFi; this turns them into flight setpoints and
sends them back over UDP. The ESP32-S3 runs stabilisation and nothing else.

```
frame ─► YOLO11n person boxes ─► ByteTrack ─► SCRFD face ─► ArcFace 512-d
      ─► target lock ─► PD control ─► CRC-checked UDP command ─► drone
```

**It runs with no drone.** Omit `--link` and the pipeline runs on your webcam,
the HUD shows exactly the setpoints it *would* transmit, and nothing is sent.
That is the intended way to build and tune it while the PCB is at the fab.

## Quick start

```bash
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt
.venv/bin/python fetch_models.py          # ~300MB, once

.venv/bin/python enroll.py --name alex     # guided webcam capture
.venv/bin/python run.py --target alex      # follow, link off
```

`run.py` keys: `SPACE` arm/disarm · `r` reset lock · `t` toggle other tracks ·
`p` pause · `q` quit.

macOS will ask for camera permission the first time. If it never asks, grant it
under System Settings → Privacy & Security → Camera.

## Why this stack

Everything is ONNX Runtime on CoreML — no torch, no ultralytics. That was
initially forced (Python 3.14 has no torch wheels) and turned out to be the
better call: 40MB of dependencies instead of 2.5GB, the detector runs on the
Apple Neural Engine, and the same three `.onnx` files will run unchanged on a
Jetson or a Pi if the pipeline ever moves onto a companion board.

| stage | model | size | ~ms |
|---|---|---|---|
| person detection | YOLO11n | 10.9 MB | 28 |
| face detection | SCRFD 10G | 16.9 MB | 30 |
| face embedding | ArcFace R50 (w600k) | 174 MB | 8 |

~22 fps with faces every frame, ~30 fps locked (faces run on a cadence).

## Identity: two signals, different jobs

Face recognition alone does not survive flight. The subject turns around,
walks off, looks down — and a face-only lock breaks every few seconds, so the
drone stops dead each time. So identity is split:

- **Face (ArcFace)** — *acquires* and *re-confirms*. Authoritative, strict
  threshold, the only thing allowed to start a lock.
- **Torso colour histogram** — *sustains* only. It answers "is this the same
  body I confirmed 0.4s ago?", which is a much easier question than "who is
  this?", and it costs nothing.

The appearance model updates **only on face-confirmed frames**. An
unconditioned EMA slowly walks onto whoever the tracker happens to be
following, so within a minute it would confidently confirm a stranger.

Measured on `tests/data/zidane.jpg` (two people, one enrolled):

| | cosine similarity |
|---|---|
| different people | **0.037** |
| same person, 0.6× scale | 0.995 |
| same person, 0.4× + blur | 0.932 |
| same person, underexposed | 0.991 |
| match threshold | 0.36 |

## The lock state machine

```
SEARCHING ──face confirmed 5 frames──► LOCKED
    ▲                                    │ no confirmation
    └────── unlock timeout (45f) ─── COASTING ──(re-confirm)──► LOCKED
                                         │
                                         ▼  LOST → failsafe → SEARCHING
```

Acquiring is deliberately slower than holding. Symmetric thresholds make the
lock oscillate between two people standing together and the drone yaws back and
forth between them.

Coasting is graded, not binary. For 20 frames after losing the subject the
drone keeps chasing the Kalman prediction — that is almost always a brief
occlusion. After that it holds station but *keeps the track identity*, so
walking back into frame re-locks instantly.

The body channel is a **768-d learned person-ReID embedding** (OpenCV Zoo
youtu-reid, 1.1ms per crop on CoreML, batched across tracks). It falls back to
an HSV colour histogram if the model is missing. The upgrade is not cosmetic —
measured on four similarly-dressed people in `bus.jpg`:

| descriptor | worst score between *different* people | its threshold | verdict |
|---|---|---|---|
| colour histogram | **0.655** | 0.55 | would confirm the wrong person |
| learned re-ID | **0.385** | 0.55 | no confusion |

Three guards stop a crossing from leaving the drone following the wrong person:

- **Identity transfer** — if another track matches the enrolled face clearly
  better than the current one, the lock moves to it.
- **Face veto** — a face we could read but that scores below 0.15 is treated
  as evidence *against* the lock, and overrides a body match. Without this, an
  ID swap between two people in similar clothing is sustained by colour alone,
  forever: the body keeps agreeing and the face is never consulted. The veto
  threshold sits in the empty band between measured strangers (≤0.08) and the
  worst same-person score (≥0.5 under blur, scale and low light), so it cannot
  fire on a bad angle of the right person.

## Control

Three PD loops, no integral anywhere. A drone following a walking person has a
permanent non-zero error — that is what following *is* — and an I term would
treat it as a bias to eliminate, wind up, and overshoot every time the subject
stopped.

| axis | error signal | output |
|---|---|---|
| yaw | horizontal offset from aim point | deg/s, ±120 |
| pitch | box **height** vs 0.45 of frame | deg, ±14 |
| altitude | vertical offset, integrated | m, 0.4–2.0 |

Distance comes from box **height**, never width or area. `zidane.jpg` is the
reason: one subject has his arms outstretched and his box is 993px wide by
511px tall. Height barely changes when someone turns sideways; width nearly
halves.

Every axis has a soft deadband (continuous, so it leaves zero smoothly), an
output clamp and a slew limit. The slew limit is the last line of defence —
even a wild vision glitch cannot command more than 110°/s of change per second.

**The altitude loop ignores a vertically clipped box.** Found on the first
real webcam run, where the setpoint walked 1.20 → 0.40 m and pinned at the
floor. When the subject is close their legs fall outside the frame, so the box
centroid sits higher than their true centre; the loop reads "target too low",
descends, and that clips *more* of them. It runs away rather than settling. If
a box touches the top or bottom edge the vertical axis holds — yaw and pitch
carry on, since the horizontal centroid and the box height are still good.

### Attitude compensation (do not remove this)

The camera is bolted to the airframe with no gimbal, so every degree of tilt
moves the whole image. Uncorrected, that is a feedback loop rather than an
offset:

```
chase forward → pitch nose-down → subject appears HIGHER in frame
             → altitude loop reads "too high" → climb → ...
```

At the 14° pitch limit through a 65° lens that apparent shift is ~200px —
larger than any real movement the subject makes. Roll couples into yaw the same
way. `geometry.py` undoes both from the roll/pitch already in the telemetry
packet, before any error is computed. It costs nothing and it never shows up on
a webcam, which is exactly why it is easy to miss.

Set `camera.hfov_deg` to your actual lens or the correction will be scaled
wrong. Measure it: put a metre rule at a known distance, count pixels,
`f = pixels × distance / width_in_metres`.

### Lead compensation

The tracker's Kalman filter estimates target velocity every frame. The
controller aims `lead_time_s` ahead of the measured position, because at 1.5 m/s
and 3 m range the subject is ~75px ahead of where they were last seen — a
systematic lag the drone would otherwise never close.

### Standoff in metres

Enrol a subject height (`enroll.py --height 1.75`) and the follow distance
becomes a real range via the pinhole relation, instead of a fraction of frame
height. Physical, so it survives a change of camera, lens or resolution.
Without a height it falls back to the fraction.

### Tuning at a desk

Flight defaults assume the subject is metres away, filling ~45% of frame
height. At a laptop you fill 80–90%, so pitch sits pinned at its −14° limit
and you cannot see that axis respond:

```bash
.venv/bin/python run.py --target alex --config configs/desk.yaml
```

Retargets the standoff to arm's length so leaning in and out moves the pitch
bar. A tuning aid — do not fly with it.

## The link

Raw floats over UDP is the obvious design and a genuinely bad one. UDP
delivers packets corrupted, reordered, duplicated and late, and each has a
specific consequence: a flipped exponent bit turns 3°/s into 3e38°/s.

Every packet carries magic + version + sequence + CRC-16/CCITT. The receiver
drops anything failing a check rather than clamping it — a command you cannot
trust is worse than no command, because the watchdog handles absence safely.

`gen_protocol_header.py` emits `firmware/include/pursuit_link.h` from
`pursuit/protocol.py`, so the two ends cannot drift. The generated C and the
Python pack byte-identical frames, CRC included — verified, not assumed.

**`CommandPump`** sends at a constant 30Hz regardless of vision frame rate, so
the drone can distinguish three situations that want different responses:

| | drone should |
|---|---|
| fresh commands arriving | follow them |
| link alive, `FAILSAFE` set | laptop is fine, just can't see the target — hold |
| nothing arriving at all | laptop or WiFi is gone — land |

A command older than 150ms is **not** repeated; the pump substitutes a neutral
failsafe. Re-sending a stale hard-left yaw at 30Hz looks like a perfectly
healthy link from the firmware's side, right up until it hits something.

## Flying it: hybrid manual + autonomous

Three modes over one link. `pursuit/arbiter.py` is the only thing that authors
what goes on the wire, so no code path can arm the aircraft without passing its
checks.

| mode | who flies | throttle stick |
|---|---|---|
| `MANUAL` | you, entirely | throttle |
| `ASSIST` | you, altitude held off the ToF | climb rate — centre holds |
| `FOLLOW` | vision, while you hold the deadman | (vision) |

Four rules, in priority order:

1. **No deadman, no motors.** The arm bit is set only while you are actively
   holding the control — never latched. A toggle means a laptop left running
   in another room is still authorising flight.
2. **Touching the sticks takes control.** Any deflection past a threshold
   during `FOLLOW` drops to `ASSIST` and *latches* there. You do not have to
   find a mode button while the drone is doing something wrong; the
   instinctive reaction is the correct one. Letting go does not hand it back.
3. **Vision only ever commands in `FOLLOW`.** Elsewhere its output is computed
   for the HUD and discarded.
4. **It boots in `MANUAL`.** A system that starts in autonomy can fly away
   before anyone has agreed that it should.

Rule 2 is all-or-nothing rather than per-axis on purpose. Blending pilot yaw
with autonomous pitch sounds clever and produces an aircraft where neither
party knows who is responsible for what.

```bash
.venv/bin/python run.py --target alex             # gamepad if present
.venv/bin/python run.py --target alex --keyboard
.venv/bin/python run.py --target alex --no-pilot  # autonomous only
```

Keyboard: `wasd` roll/pitch, `qe` yaw, `ik` throttle, `1`/`2`/`3` mode,
**hold SPACE** as the deadman, `x` to quit. Digital, so it ramps — fine for
bench-testing the mode logic, not for actually flying.

`pygame` has no Python 3.14 wheels, so gamepads are read as raw HID through
`hidapi`. The profiles in `pilot.py` come from published HID descriptors rather
than from testing every pad, so **treat an untested profile as a hypothesis**:

```bash
.venv/bin/python calibrate_gamepad.py --learn
```

It moves you through one control at a time and reports which byte responds.

An RC receiver would be better than any of this. `D7/GPIO44` is a free hardware
UART RX pin — exactly what SBUS or CRSF needs — giving a control link that does
not depend on WiFi, a laptop, or Python not crashing. Hand-wired to the
castellated pad; no board change.

## Testing without an aircraft

```bash
.venv/bin/python sim_drone.py --corrupt 0.05 --lossy 0.1   # terminal 1
.venv/bin/python run.py --target alex --link --host 127.0.0.1   # terminal 2
```

The simulator implements the same watchdog the firmware must, and injects the
failures the real link will have. A corrupted packet that gets *accepted* is
the serious bug, and the only way to know the CRC catches it is to corrupt some.

```bash
.venv/bin/python -m pytest tests/ -q          # 70 tests
.venv/bin/python -m pytest tests/ -q -m "not slow"   # skip model loading
```

The suite covers, among others: every one of the 240 single-bit flips in a
command packet being rejected; the yaw/pitch/altitude sign conventions; that a
coasting track cannot steal a neighbour's identity; that a clipped box cannot
run the altitude setpoint into the floor; and a full pipeline run that enrols a
real face and holds lock across a 45-frame pan while steering the right way.

### Measured on a MacBook Pro webcam, one enrolled subject

| | |
|---|---|
| lock held | 97% of frames (194/200) |
| face confirmation | 0.78–0.85 |
| body sustain between face checks | 0.98–1.00 |
| frames with face stage | 33ms → **without**, 78ms → with |
| overall | 13–15 fps at 1280×720 |

The detector dominates at ~33ms; raising `face.interval_locked` or dropping
`detector.imgsz` to 480 buys frame rate if the follow loop needs it.

## Layout

```
pursuit/
  config.py      every tunable, YAML-overridable
  onnxbase.py    ORT session factory (CoreML), letterbox, NMS, IoU
  detect.py      YOLO11n person detection
  face.py        SCRFD detect + ArcFace embed + 5-point alignment
  body.py        torso colour descriptor, drift-guarded memory
  bytetrack.py   Kalman + Hungarian, 3-pass association
  gallery.py     enrolled identities on disk
  lock.py        the target lock state machine
  control.py     PD loops -> setpoints
  protocol.py    packet format, CRC16
  link.py        UDP transport, telemetry, CommandPump
  hud.py         annotated preview
  pipeline.py    orchestration + per-stage timing
enroll.py  run.py  sim_drone.py  fetch_models.py  gen_protocol_header.py
```

## Not done yet

- The drone does not exist — the PCB is at the fab. Frames come from the
  webcam; swap `--source` for the aircraft's MJPEG URL when it flies.
- Firmware must honour `FLAG_ARMED`, implement the watchdog described above,
  and never act on a packet that fails `pursuit_cmd_valid()`.
- Gains in `ControlConfig` are reasoned, not flight-tuned. Expect to retune
  `yaw_kp` and `target_height_frac` on the first real flight, outdoors, with
  props off first.
