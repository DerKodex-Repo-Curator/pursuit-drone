# PURSUIT-1 bring-up

`VERIFICATION.md` is what was checked **before** the gerbers went out. This is
what gets checked **after** the boards come back, on the physical article, in
an order chosen so that no step can destroy the one after it.

Nothing in this project may be described as working until the sign-off table
at the bottom is filled in from real hardware. A passing test suite is not a
flying aircraft: 109 host tests and 211 vision tests say the logic is right,
and say nothing at all about whether M3 is soldered to the correct pad.

**Status: NOT STARTED. The aircraft has never flown. PID gains are scaled
from typical brushed-whoop values and have never touched this airframe.**

---

## Rules for the whole procedure

1. **Props off** until step 9 says otherwise. Every check up to that point is
   designed to be safe with bare shafts, and a bare shaft is survivable.
2. **One change at a time.** If two things change between a working and a
   broken state, you have learned nothing.
3. **A step that fails is a STOP, not a warning.** Each one below says what
   failing means. None of them mean "probably fine".
4. Keep a hand on the battery lead for anything involving motors. The fastest
   disarm is physical.

## What you need before starting

- Multimeter with continuity/diode mode
- USB-C **data** cable — a charge-only cable is the most common reason a XIAO
  never enumerates, and it fails silently
- 1S LiPo, charged, plus a charger
- Somewhere to clamp the board that is not your hand

## Unresolved before the PCBA is approved

**Q1–Q4 SOT-23 rotation.** The community rotation databases disagree, JLCPCB
publishes no official table, and the CPL carries KiCad's rotation with no
correction applied. D1–D4 have been checked (cathode inboard, toward VBAT).
The MOSFETs have **not** been confirmed. Hold `gerbers/ORIENTATION_CHECK.csv`
against JLCPCB's Confirm-Parts-Placement render and get their answer
numerically before approving.

Getting this wrong gives four transistors with drain and source swapped: the
body diode conducts, the gate does nothing, and no motor ever turns.

---

## 1. Bare board, no components energised

Before the XIAO goes anywhere near the headers.

- [ ] **Visual**: no solder bridges on U2 (0.5 mm pitch LGA-14), no tombstoned
      passives, no missing flyback diodes.
- [ ] **VBAT to GND**: not a short. Any reading under ~1 kΩ means stop.
- [ ] **3V3 to GND**: not a short.
- [ ] **JST-PH polarity.** JST-PH polarity is *not* standardised across
      suppliers, which is why the silkscreen carries a warning. Meter the
      battery lead against the pads. **Do not trust the wire colours.**
      Backwards means the pack vents.
- [ ] **Diode orientation** on D1–D4: banded end (cathode) inboard, toward
      board centre along the arm.

**STOP if**: any short, or the JST polarity does not match the silk.

## 2. XIAO alone, no board

Proves the module and your toolchain before either can be blamed on the PCB.

```bash
cp firmware/include/secrets.h.example firmware/include/secrets.h
# then edit it and set your own PURSUIT_AP_PASS — the build #errors rather
# than compile with a published default
```

- [ ] XIAO enumerates over USB-C. If not: **different cable first**, then
      double-tap RESET to force the bootloader.
- [ ] `pio run -e xiao` builds.
- [ ] `pio run -e xiao -t upload` flashes.
- [ ] Serial at 115200 shows the banner and then:

      =============== BRING-UP MODE — NO IMU ===========

      This is **correct** on a bare XIAO. The IMU and rangefinder are on the
      PCB, so both *should* be absent here. The aircraft holds in BOOT and
      the motors cannot be enabled by any path, including a valid armed
      command over the link.

- [ ] `h` prints the bench help.
- [ ] SoftAP `PURSUIT-1` appears; laptop can join; `192.168.4.1` responds.
- [ ] Camera stream at `http://192.168.4.1:81/` shows a picture.
- [ ] Blackbox reports its PSRAM allocation at boot (needs the Sense module).

**STOP if**: it does not enumerate with a known-good data cable. Everything
downstream needs this.

## 3. Board powered from USB, XIAO fitted, no battery

- [ ] 3V3 rail measures 3.25–3.35 V at a bypass cap.
- [ ] Serial reports `IMU ok (ICM-42688-P)` — WHO_AM_I must read **0x47**.
- [ ] Serial reports `rangefinder ok (VL53L1X)`.
- [ ] If a BMP390 is fitted: `barometer ok (BMP390 @0x77)` — or `@0x76` if
      SDO is pulled low.
- [ ] Press `s`. Sensor report shows:
      - gravity magnitude **0.85–1.15 g** (printed as "plausible")
      - gyro bias printed and marked *calibrated*
      - loop timing histogram centred near 1000 µs

**STOP if**: gravity is implausible — that is accel scaling or mounting, and
every attitude estimate downstream is built on it.

**Note**: gyro calibration *refuses* if peak rate exceeds 5 °/s. Hold the
board still. Calibrating while moving bakes that motion in as "level".

## 4. Orientation — the signs

The IMU's own axes versus the FLU body frame cannot be settled from a desk.
This is the ground truth, and it must pass before anything spins.

Press `o`, then tilt slowly and read the numbers:

| Physical motion | Required reading |
|---|---|
| **RIGHT edge down** | roll **POSITIVE** |
| **NOSE DOWN** | pitch **POSITIVE** |

Yes, nose *down* is positive — the protocol and the filter both use
nose-down-positive. See `src/frame.h`.

- [ ] Roll sign correct
- [ ] Pitch sign correct

**If either is wrong**: fix `IMU_ORIENT_*` in `include/config.h`, never the
filter. A wrong sign here means every correction in flight is backwards, and
the aircraft "corrects" a 30° bank by rolling to 60°.

## 5. Motors, props OFF, USB power

Bench test duty starts at 12% and is capped at 35%. Each run is 2.5 s — a
motor left spinning while you look away is how a bare shaft finds a finger.

Press `1`, `2`, `3`, `4` in turn. For each:

- [ ] The motor that spins is the one named
- [ ] It turns the direction the belly silkscreen says

| | Position | Required rotation (seen from above) |
|---|---|---|
| M1 | front-left | CW |
| M2 | front-right | CCW |
| M3 | rear-left | CCW |
| M4 | rear-right | CW |

To reverse a brushed motor, **swap its two wires**. Never change the mix.

**STOP if**: the wrong motor spins (a pad is miswired) or none spins (check
the Q1–Q4 rotation question above first — that is the likeliest cause).

## 6. Battery, first plug-in

Use a fused or current-limited lead if you have one.

- [ ] Pack voltage on the divider matches a meter within ~0.05 V. The
      firmware reads it through `analogReadMilliVolts` (eFuse-calibrated)
      because the S3's raw ADC is 5–10 % nonlinear, against a sag window only
      0.45 V wide.
- [ ] `s` reports battery within 0.05 V of the meter.
- [ ] Repeat step 5 on battery power. Watch the rail: the XIAO's buck drops
      out around **3.5 V in**, and SPICE puts a real pack at ~3.6 V during a
      four-motor burst.

**STOP if**: the board browns out or resets during a motor run. That is the
power path, and no amount of PID tuning survives a flight controller that
reboots in the air.

## 7. Barometer, if fitted

Mechanical, not electrical, and it decides whether altitude hold works at all
above 3.5 m.

- [ ] **Open-cell foam over the port. Not tape.** Tape seals it, and a sealed
      barometer measures a fixed trapped volume — it stops working entirely.
      Foam passes pressure and blocks wind and light.
- [ ] Foam is **dark**. Light on the MEMS die shifts the reading outright.
- [ ] Press `s` with props **still**: note the metres of spread over 1 s.
- [ ] Press `s` with props **spinning** (still no props fitted — motors bare):
      spread must not grow materially.

**If the spread grows**: prop wash is reaching the port. The fix is more
foam. It is not a gain change and not a filter setting.

Ceiling is `ALT_CEILING_TOF_M` 2.5 m without a barometer, `ALT_CEILING_BARO_M`
9.5 m with one. The firmware clamps the commanded height to whichever applies,
so the laptop cannot fly the aircraft out of its own sensing.

## 8. Link

- [ ] Laptop joins `PURSUIT-1`, `python vision/run.py --target <you> --link`
- [ ] HUD link panel shows **UP**, telemetry arriving, `tx`/`rx` climbing
- [ ] `bad` and `err` stay at zero
- [ ] RTT is sane (~20 ms). This is not a diagnostic: the controller's lead
      compensation is derived from it, and the stability boundary without
      attitude compensation is ~17 ms.
- [ ] Attitude in the HUD's horizon matches the board when you tilt it
- [ ] Kill the WiFi: aircraft levels off and enters FAILSAFE at 400 ms, then
      LANDING at 3 s. It must **never** hold the last command.

## 9. Props ON — first spin, aircraft restrained

Only now. Everything above must have passed.

- [ ] Aircraft tied down so it cannot leave the bench
- [ ] Correct props on the correct motors (CW props on CW motors)
- [ ] Arm and hold idle. ARMED must **hold idle throttle**, not spin to the
      commanded value — a spring-centred stick must not produce 50 % on arm.
- [ ] Raise throttle gently. Nothing oscillates, nothing screams.

## 10. Tuning, restrained, one axis at a time

- [ ] Roll axis: step the setpoint, pull the blackbox, look at gyro vs
      setpoint. Lag = P too low. Overshoot then ringing = P too high or D too
      low. Fast fuzzy oscillation = D too high, amplifying gyro noise.
- [ ] Pitch axis
- [ ] Yaw axis (kd = 0 by design; nothing is fast enough to damp)

```bash
curl -o flight.pbb http://192.168.4.1:82/
python3 tools/decode_blackbox.py flight.pbb --plot flight.png
```

Check `loop_us` sits near 1000 and that **no motor is pinned at 0 or 255**.
A saturated mixer means you are looking at the limits, not the gains, and
nothing above it is being tuned.

## 11. First hover

Outdoors is not the place to find out. Large room, nobody within the fall
radius, VLOS, and a finger on the disarm.

- [ ] Hover at ~1 m under the rangefinder alone
- [ ] Altitude hold engages without a step (the learned hover baseline exists
      precisely so it does not sink on engage)
- [ ] Land, pull the blackbox, check the `alt source` breakdown — it should
      read 100 % `tof` at this height
- [ ] Only then climb through 3–3.5 m and confirm the `tof → blend → baro`
      handover leaves **no step** in the altitude trace. A step there means
      the datum offset was learned while the port was disturbed.

---

## Sign-off

No entry may be filled in from a simulation, a test suite, or an expectation.

| # | Check | Date | Result | Notes |
|---|---|---|---|---|
| 1 | Bare board, no shorts, JST polarity | | | |
| 2 | XIAO enumerates, flashes, bring-up mode | | | |
| 3 | Rails, IMU 0x47, ToF, baro present | | | |
| 4 | Orientation signs (roll +, pitch +) | | | |
| 5 | Motor identity and rotation ×4 | | | |
| 6 | Battery sense, no brownout under load | | | |
| 7 | Baro foam, spread stable under wash | | | |
| 8 | Link up, failsafe on WiFi loss | | | |
| 9 | First spin, props on, restrained | | | |
| 10 | Roll / pitch / yaw tuned | | | |
| 11 | First hover, handover clean | | | |
