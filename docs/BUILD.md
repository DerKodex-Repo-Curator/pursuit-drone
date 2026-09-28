# Building PURSUIT-1

Ordering, assembly, the two requirements the board places on the firmware, first
power-up, deliberate design compromises, and the mass budget. The step-by-step
bring-up with its sign-off table is [`BRINGUP.md`](BRINGUP.md).

## Ordering (JLCPCB)

Run `scripts/preflight.py` first. It is a hard gate over the release
artefacts and exits non-zero if anything is stale or inconsistent.

Upload three files: `gerbers/PURSUIT-1_JLCPCB.zip` (fabrication),
`gerbers/JLCPCB_BOM.csv` and `gerbers/JLCPCB_CPL.csv` (assembly).
**4 layers** (this board is rev C: F.Cu / In1 GND plane / In2 VBAT plane /
B.Cu), 92 x 92mm, 1.6mm FR4, 1oz, any colour, quantity 5. Selecting 2 layers
here drops both internal planes and leaves the aircraft with no ground and no
power distribution; `preflight.py` checks the layer count and the order form
must agree with it.

**Surface finish: ENIG, not HASL.** Everything on the board is 0603/0805/
SMA/SOT-23 and would be happy on HASL, except U2, a bottom-terminated
LGA-14 on 0.5mm pitch with no visible fillets. HASL leaves a domed,
non-coplanar surface, which on that one part is the classic cause of opens
you cannot see without an X-ray. It is roughly +$20-25 on this order and
changes nothing in the files.

For assembly: **Top side only**, **PCBA quantity 2**, Economic, and set
**Confirm Parts Placement = Yes**: the SOT-23 rotation convention is not
settled between the published community databases, so the placement render
is the final check. The CPL carries KiCad's rotation plus 180 degrees on
Q1 to Q4 only: JLCPCB's own footprint for C20917 (AO3400A) sits 180 degrees
from KiCad's, which `gen_jlcpcb.py` corrects and `preflight.py` checks. Work
through `gerbers/ORIENTATION_CHECK.csv` one designator at a time against the
render before approving it; every FET's single drain lead should point
outboard along its arm.

**CORRECTED 2026-08-22.** This section used to say the board could not meet
JLCPCB's 2.5mm component-to-edge rule, and named the SMA flybacks at ~0.7mm
and the LEDs at ~0.5mm. Both numbers were wrong, and the conclusion drawn from
them (accept a DFM query, fall back to bare boards) was therefore drawn on
folklore. `check_place.py` now measures every machine-placed body against the
real Edge.Cuts polyline and the four prop rings:

| part | claimed | measured |
| --- | --- | --- |
| D1-D4 (SMA flybacks) | ~0.7mm | **4.65mm**, compliant by a factor of 6.6 |
| LED1 | ~0.5mm | **2.57mm**, compliant |
| LED2 | ~0.5mm | **0.87mm**, the one real offender |
| C8, R11, R12, R14 | ~1.2mm | 1.60mm |
| C10-C13 | not listed | 1.80mm |

So nine parts are inside 2.5mm, none is inside the 0.3mm depanel-risk line,
and every one of them is a plate or arm part that moves a few millimetres. The
X-frame was never the constraint. Fixing them is a rev C item, and once they
clear, JLCPCB's assembly yield commitment applies to this board for the first
time. The gate is machine-readable now, so this claim cannot rot again.

Cost is dominated by the IMU: C1850418 is **~$17 each**, Extended (so a
per-line feeder fee), and it forces mandatory X-ray inspection because the
LGA-14 pads are hidden. C4, and possibly LED1, are the other Extended lines
Check the **Extended components fee** line in your cart and divide by the
per-line rate to see how many they actually counted. The BOM is now **13
lines / 29 placements** (C6 and C7 are pads-only), so re-quote rather than
reusing an older cart total. Budget
**~GBP 75-90** all in for 5 boards with 2 assembled, once ENIG, the X-ray and
two extended-part fees are counted; the bare PCB itself is a few pounds of
that. Set the **order-number placement** option explicitly rather than
letting JLCPCB drop their code anywhere on the silk.

## Build order

Reference designators are on the Fab layer, not silkscreen (they would
collide at 0603 pitch), so **open `docs/assembly_top.svg` while placing
parts**; it shows every refdes in position.

1. **Top-side SMD arrives already assembled** if you ordered PCBA (Q1-Q4,
   D1-D4, all top R/C, both LEDs, and U2 the IMU). Only hand-solder these
   if you bought bare boards, and note U2 is an LGA-14 with hidden pads
   that needs paste + hotplate/hot-air, 260°C max, never ultrasonic-clean.
2. **Belly SMD** (yours either way): C1 and C10-C13. All iron-solderable.
3. **THT**: XIAO headers (module removable), JST J1, ToF right-angle header.
   Solder two short wires from the XIAO's underside BAT+/BAT- pads to
   the J4 pads just behind the module (enables USB-C charging of the pack).
4. **Motors**: the belly silk marks each position (M1-M4) *and its
   required rotation* (M1/M4 CW, M2/M3 CCW; diagonal pairs match, which is
   the standard X-quad pattern). Fit motors to match, and make your firmware
   mix agree with those labels. To reverse a brushed motor, swap its two
   wires. The 8.70mm rings are a deliberate CLEARANCE fit, not a press
   fit: at JLCPCB's +/-0.2mm routing tolerance a true interference fit
   either cracks the ring or disappears entirely. Seat each motor square
   to the board (the clearance fit gives no alignment datum of its own),
   then zip-tie through the flanking holes over the can; add a dab of CA
   if it still shifts. M1 FL / M2 FR / M3 RL / M4 RR, wires to the MT pads
   (+ marked). Buy 2x CW + 2x CCW motors and fit counter-rotating pairs
   diagonally.
5. **Camera**: bend the FPC ~90° so the lens faces forward over the front
   edge; secure with a foam wedge or a printed clip that grips the board's
   front edge. (There are deliberately no screw holes; see BOM.)
   Don't re-bend the flex repeatedly. Attach the U.FL antenna and tape the
   whip along a rear arm (away from the marked ANT zone under the module).
6. **Battery**: velcro pad on the belly front + zip-tie/strap through the
   two 3.2mm holes. Balance so the CG sits at the geometric centre.

## Two hard requirements the board places on the firmware

Both are **implemented**. This section records why they exist, because both
are the kind of thing that is invisible until an aircraft falls out of the air.

SPICE simulation of the drive stage (see VERIFICATION.md §6) shows the pack
rail sagging to **3.61 V at four-motor full throttle** with an optimistic
60 mΩ source model; a real 450 mAh 1S cell is worse. The XIAO's 3V3 buck
drops out around 3.5 V input, which would brown out the flight controller
mid-air. The board carries the sensor needed to prevent it: the VBAT_SENSE
divider on D2.

→ `firmware/src/battery.cpp`. It keeps **two** filters deliberately: a 20 ms
fast one for the sag limiter, which has to react *during* a burst or the buck
browns out anyway, and a 2 s slow one for the land decision, because otherwise
a normal throttle burst reads as a flat pack and the aircraft lands itself. The
throttle floor is 0.35: cutting to zero turns a descent into a fall, and a
falling aircraft draws no current, so the voltage recovers and it oscillates
its way down.

Second requirement: **stagger the four LEDC PWM channels by 90°**
(hpoint = 0, T/4, T/2, 3T/4). By default all four channels on a timer rise on
the same edge, turning four 2.75 A steps into one synchronised 11 A step on the
pack. Staggering cuts peak rail current to ~2.75 A and roughly halves C1's
ripple heating.

→ `firmware/src/motors.cpp`, and it must use `ledc_set_duty_with_hpoint()` from
the IDF driver rather than Arduino's `ledcWrite()`: plain `ledc_set_duty()`
resets hpoint to 0 and silently un-staggers everything on the first write.

The same simulation confirms the safety property the design depends on:
the gates cannot reach the AO3400A's 0.65 V threshold at reset.

R5-R8 are **1 kΩ, not 10 kΩ**, and the reason matters because an earlier
revision of this file got it wrong. It claimed the XIAO Sense expansion board
pulls D8 up through 4.7 kΩ; that came from a forum post. Seeed's actual
schematic (`XIAO_ESP32S3_ExpBoard_v1.0_SCH.pdf`, read at 900 dpi to resolve
the junction dots) puts the three 4.7 kΩ pull-ups on **D9/MISO**, **D10/MOSI**
and **SD_CS**, never on D8/SCK, which carries only a 22 Ω series link and an
ESD clamp. So M4_PWM floats at reset exactly like M1-M3.

1 kΩ is kept anyway, for a reason that survives checking: it bounds the gate
against an *unexpected* pull-up. The ESP32-S3's own internal pull-up is
~45 kΩ, and at 10 kΩ that fault would put a gate at 0.60 V, within 50 mV of
the AO3400A's 0.65 V minimum threshold. At 1 kΩ the same fault gives 72 mV.
It also discharges the gate about ten times faster when a pin goes high-Z. The
cost is 3.00 V of gate drive instead of 3.28 V, still above the 2.5 V at which
Rds(on) is specified.

The two pull-ups that *do* reach this board land on D9 (IMU_INT: harmless,
and it usefully serves the ICM-42688-P's open-drain INT1) and D10 (LED_CTRL:
0.29 mA of LED glow at reset).

## First power-up

The summary is below. The tickable version, ordered so no step can destroy
the one after it, is [**`docs/BRINGUP.md`**](BRINGUP.md), including the
sign-off table that has to be filled in from real hardware before any part of
this project is described as working.

1. **Verify battery polarity with a multimeter against the `+` on the top
   silk beside J1's left pad before ever plugging in.** JST-PH polarity is
   NOT standardised; roughly half of cheap packs are wired the other way.
   Reversed = every FET plus the XIAO die instantly.
   The same applies to C1 when you fit it: the belly silk reads `+ C1`
   beside its positive pad, and the footprint's silk bar is at that same
   end. C1 is a polymer/tantalum bulk cap straight across an unfused pack;
   backwards, it vents.
2. Optional: cut J3 on the Sense expansion board to drop the SD pull-ups on
   D9/D10. Not required: neither pin drives a motor. Leave it alone unless
   you want the IMU interrupt line clean, and note you lose the microSD slot.
3. Props OFF. Power via USB-C first: flash a firmware stub, check 3V3,
   scan I2C (expect 0x68 IMU, 0x29 ToF, and 0x77 if a BMP390 is fitted),
   read the battery ADC.
4. First LiPo plug-in through a fused/current-limited lead if you have one.
5. Motor test one at a time at low duty; verify spin directions.

## Design compromises (known, deliberate)

- Debug UART header dropped; use the XIAO's USB-C serial; D6/D7 remain
  free and reachable on the module's castellated pads.
- One 0.15mm trace runs under the IMU body (AP_CS→VDDIO tie), against
  TDK's strictest layout note but standard practice at hobby scale.
- One I2C bus shared by IMU (1kHz reads) + ToF (30Hz) ≈ 40-50% bus
  utilisation at 400kHz, which is fine; move the IMU to SPI in a respin if you ever
  need 8kHz gyro ODR. A BMP390 on the same bus adds ~50Hz of 7-byte reads,
  which is under 2% more and does not change that conclusion.
- JST-PH is rated 2A continuous vs ~11A full-throttle bursts, accepted
  whoop-class practice; solder a 20AWG pigtail to the J5 pads for hard use.
- The 1.6mm board is ~3g heavier than 1.2mm, chosen for first-spin crash
  survival. Arms are 12mm wide for the same reason (and to give the VBAT
  pour a clear channel past the motor traces).
- Motors are a **clearance** fit in 8.70mm holes, not a press fit: at
  JLCPCB's ±0.2mm routing tolerance a true interference fit either cracks
  the ring or vanishes entirely. Retention is the zip ties (+ a dab of CA).
- No expansion header: its 3V3 trunk had to wrap the whole right side and
  blocked the motor fan-out. J2 (ToF) already brings out 3V3 and GND.
- Both orientation LEDs hang off one GPIO (LED_CTRL), so they are always on
  or off together; nose vs tail is told by colour (yellow / red), never by
  blink pattern. D6/D7 are free if a respin wants them separated.
- Nine placed parts sit inside JLCPCB's 2.5mm PCBA guideline. Worst is
  **LED2 at 0.87mm**, then C8/R11/R12/R14 at 1.60mm and C10-C13 at 1.80mm.
  Nothing is inside the 0.3mm depanel-risk line. NOT unavoidable: D1-D4
  measure 4.65mm and LED1 2.57mm, so the old claim that 12mm arms made this
  impossible was simply wrong. `python3 scripts/check_place.py` prints the
  live list; see the ordering section.
- **C6 and C7 are pads only, deliberately not fitted.** The XIAO's 3V3 comes
  from U3, an SGM6029C 4 MHz buck whose datasheet gives an *effective* output
  capacitance range of **5-10 µF**, and the module already carries 9.4 µF
  nominal of its own. Fitting C6 (22 µF) and C7 (100 µF) put roughly 50-64 µF
  effective on that node, five to six times the specified window, on the rail
  that powers the flight controller. C2/C3/C4 stay fitted; they are IMU-local
  and add only ~1.9 µF. If bring-up shows the 3V3 rail actually sagging, hand-
  solder C6 and/or C7 then: adding to a bare pad takes five minutes, removing
  a reflowed 1206 sitting 7.3 mm from a 0.5 mm-pitch LGA does not.

## Mass and thrust budget (estimates, weigh as you build)

frame ~13g + XIAO+camera ~6g + motors 21g + props 2g + IMU/passives/
connectors ~4g + battery 18g (650mAh) + wires ~2g ≈ **66g AUW**.

Thrust is ~140g nominal, but derate it for the pack sag this board's own
SPICE model predicts and the usable figure is ~102g, so **T:W ≈ 1.6, not
2.3**. Hover sits near 60-65% throttle. That is flyable but not sprightly:
gust rejection is limited and there is no payload margin worth speaking of.

## UK legal

Camera drone: CAA **operator ID** (£11/yr, label the airframe) + free flyer
ID test, even sub-250g. Fly line-of-sight; person-following demos with a
consenting friend, indoors or private open space at walking pace.
