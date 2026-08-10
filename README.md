# PURSUIT-1 — person-following PCB-frame micro quadcopter

The PCB **is** the airframe. A 2-layer 1.6mm FR4 X-frame (100mm wheelbase,
91.91 x 91.91mm bounding box) carries the complete flight electronics:

- **Seeed XIAO ESP32S3 Sense** (flight controller + OV2640/OV3660 camera +
  WiFi) on 2.54mm headers, USB-C facing the front edge
- **ICM-42688-P** 6-axis IMU (I2C @400kHz, addr 0x68, INT1 on D9)
- **4x AO3400A** low-side brushed motor drivers (32kHz PWM) with SS34
  flybacks, driving 8520 coreless motors clamped into the frame rings
- **VL53L1X** (Pololu 3415) time-of-flight altitude sensor on the rear belly
- 4x 100nF VBAT ceramic bypass on the belly, one at each flyback
- 1S LiPo, JST-PH + solderable pigtail pads, battery telemetry divider

Person detection/re-ID runs **off-board**: the drone streams camera frames
over WiFi to a laptop (YOLO/MediaPipe + face-embedding lock), which returns
attitude/throttle setpoints over UDP. The on-board loop runs stabilisation
(Madgwick + cascaded PID at 1kHz) and altitude hold (30Hz ToF fusion).

## Repository layout

```
hardware/          KiCad 10 project (pursuit.kicad_pcb, pursuit.kicad_sch,
                   pursuit.pretty/ custom footprints)
gerbers/fab/       JLCPCB-ready gerbers + Excellon drill (Protel names)
gerbers/           BOM.csv (hand-build list), PURSUIT-1_schematic.pdf,
                   board renders, PURSUIT-1_JLCPCB.zip, and the three
                   JLCPCB upload files (BOM / CPL / ORIENTATION_CHECK)
scripts/           design.py    - canonical netlist (single source of truth)
                   gen_pcb.py   - board generator (pcbnew API)
                   gen_sch.py   - schematic generator
                   routes.py    - explicit routing tables
                   footprints.py- custom footprint emitter
                   check_place.py - fast placement-collision checker
                   audit_islands.py - pour-fragmentation release gate
                   preflight.py   - order gate over the release artefacts
                   check_sch.py   - schematic netlist vs design.py gate
                   gen_jlcpcb.py  - JLCPCB BOM/CPL/orientation sheet
                   release.py     - gerbers/drill/renders/PDF/zip + MANIFEST
docs/              DESIGN.md, VERIFICATION.md,
                   assembly_top.svg / assembly_bottom.svg (refdes map —
                   open these while soldering)
sim/               ngspice netlists for the motor drive + boot-safety proof
```

The board is **generated, not drawn**: `design.py` holds the netlist,
`gen_pcb.py`/`gen_sch.py` emit the KiCad files, and a 3-way netlist diff
(design ↔ schematic ↔ PCB) plus KiCad ERC/DRC gate every change. The
schematic is a real drawing — functional blocks, wired nets, power symbols,
signal flow left to right — and `check_sch.py` re-derives its netlist with
KiCad's own netlister and diffs it against `design.py` by both partition and
net name, so a wire that stops short of a pin fails the build rather than
merely looking plausible. To rebuild:

```
python3 scripts/footprints.py
<KiCad python3> scripts/gen_pcb.py
<KiCad python3> scripts/gen_sch.py
kicad-cli pcb drc --refill-zones --save-board hardware/pursuit.kicad_pcb
python3 scripts/release.py
<KiCad python3> scripts/gen_jlcpcb.py
<KiCad python3> scripts/preflight.py
```

`release.py` holds the export flags (`--subtract-soldermask`, absolute drill
origin) in code rather than in a shell history — a re-export without them
silently produces a different, un-reviewed silkscreen.

## Ordering (JLCPCB)

Run `scripts/preflight.py` first — it is a hard gate over the release
artefacts and exits non-zero if anything is stale or inconsistent.

Upload three files: `gerbers/PURSUIT-1_JLCPCB.zip` (fabrication),
`gerbers/JLCPCB_BOM.csv` and `gerbers/JLCPCB_CPL.csv` (assembly).
2 layers, 1.6mm FR4, 1oz, any colour, quantity 5.

**Surface finish: ENIG, not HASL.** Everything on the board is 0603/0805/
SMA/SOT-23 and would be happy on HASL, except U2 — a bottom-terminated
LGA-14 on 0.5mm pitch with no visible fillets. HASL leaves a domed,
non-coplanar surface, which on that one part is the classic cause of opens
you cannot see without an X-ray. It is roughly +$20-25 on this order and
changes nothing in the files.

For assembly: **Top side only**, **PCBA quantity 2**, Economic, and set
**Confirm Parts Placement = Yes** — the SOT-23 rotation convention is not
settled between the published community databases, so the placement render
is the only reliable check. The CPL carries KiCad's own rotation with no
correction applied; do not pre-apply one from a community table, read it
off their render. Work through `gerbers/ORIENTATION_CHECK.csv` one
designator at a time against that render before paying.

Expect a DFM note about **component-to-board-edge distance**. JLCPCB ask for
2.5mm and this board cannot give it: it is an X-frame with 12mm arms, so the
SMA flybacks sit ~0.7mm from an arm edge and LED1/LED2 ~0.5mm. That is
inherent to a PCB-as-airframe, not a mistake — but it may draw an engineering
query or an added rail/tab charge. If they decline to assemble it, the
fallback is bare boards plus the hand-build list in `gerbers/BOM.csv`.

Cost is dominated by the IMU: C1850418 is **~$17 each**, Extended (so a
per-line feeder fee), and it forces mandatory X-ray inspection because the
LGA-14 pads are hidden. C4, and possibly LED1, are the other Extended lines
— check the **Extended components fee** line in your cart and divide by the
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
parts** — it shows every refdes in position.

1. **Top-side SMD arrives already assembled** if you ordered PCBA (Q1-Q4,
   D1-D4, all top R/C, both LEDs, and U2 the IMU). Only hand-solder these
   if you bought bare boards — and note U2 is an LGA-14 with hidden pads
   that needs paste + hotplate/hot-air, 260°C max, never ultrasonic-clean.
2. **Belly SMD** (yours either way): C1 and C10-C13. All iron-solderable.
3. **THT**: XIAO headers (module removable), JST J1, ToF right-angle header.
   Solder two short wires from the XIAO's underside BAT+/BAT- pads to
   the J4 pads just behind the module (enables USB-C charging of the pack).
4. **Motors**: the belly silk marks each position (M1-M4) *and its
   required rotation* (M1/M4 CW, M2/M3 CCW — diagonal pairs match, which is
   the standard X-quad pattern). Fit motors to match, and make your firmware
   mix agree with those labels. To reverse a brushed motor, swap its two
   wires. The 8.70mm rings are a deliberate CLEARANCE fit, not a press
   fit — at JLCPCB's +/-0.2mm routing tolerance a true interference fit
   either cracks the ring or disappears entirely. Seat each motor square
   to the board (the clearance fit gives no alignment datum of its own),
   then zip-tie through the flanking holes over the can; add a dab of CA
   if it still shifts. M1 FL / M2 FR / M3 RL / M4 RR, wires to the MT pads
   (+ marked). Buy 2x CW + 2x CCW motors and fit counter-rotating pairs
   diagonally.
5. **Camera**: bend the FPC ~90° so the lens faces forward over the front
   edge; secure with a foam wedge or a printed clip that grips the board's
   front edge. (There are deliberately no screw holes — see BOM.)
   Don't re-bend the flex repeatedly. Attach the U.FL antenna and tape the
   whip along a rear arm (away from the marked ANT zone under the module).
6. **Battery**: velcro pad on the belly front + zip-tie/strap through the
   two 3.2mm holes. Balance so the CG sits at the geometric centre.

## Firmware requirement (not optional)

SPICE simulation of the drive stage (see VERIFICATION.md §6) shows the pack
rail sagging to **3.61 V at four-motor full throttle** with an optimistic
60 mΩ source model — a real 450 mAh 1S cell is worse. The XIAO's 3V3 buck
drops out around 3.5 V input, which would brown out the flight controller
mid-air. The board already carries the sensor needed to prevent this: the
VBAT_SENSE divider on D2. **Your firmware must read it and limit throttle /
warn on a sagging pack.** Treat that as part of the first flyable build, not
a later feature.

Second firmware requirement, free to implement: **stagger the four LEDC PWM
channels by 90°** (`ledc_set_duty_with_hpoint()`, hpoint = 0, T/4, T/2,
3T/4). By default all four channels on a timer rise on the same edge, which
turns four 2.75A steps into one synchronised 11A step on the pack. Staggering
cuts peak rail current to ~2.75A and roughly halves C1's ripple heating.

The same simulation confirms the safety property the design depends on:
the gates cannot reach the AO3400A's 0.65 V threshold at reset.

R5-R8 are **1 kΩ, not 10 kΩ**, and the reason matters because an earlier
revision of this file got it wrong. It claimed the XIAO Sense expansion board
pulls D8 up through 4.7 kΩ; that came from a forum post. Seeed's actual
schematic (`XIAO_ESP32S3_ExpBoard_v1.0_SCH.pdf`, read at 900 dpi to resolve
the junction dots) puts the three 4.7 kΩ pull-ups on **D9/MISO**, **D10/MOSI**
and **SD_CS** — never on D8/SCK, which carries only a 22 Ω series link and an
ESD clamp. So M4_PWM floats at reset exactly like M1-M3.

1 kΩ is kept anyway, for a reason that survives checking: it bounds the gate
against an *unexpected* pull-up. The ESP32-S3's own internal pull-up is
~45 kΩ, and at 10 kΩ that fault would put a gate at 0.60 V — within 50 mV of
the AO3400A's 0.65 V minimum threshold. At 1 kΩ the same fault gives 72 mV.
It also discharges the gate about ten times faster when a pin goes high-Z. The
cost is 3.00 V of gate drive instead of 3.28 V, still above the 2.5 V at which
Rds(on) is specified.

The two pull-ups that *do* reach this board land on D9 (IMU_INT — harmless,
and it usefully serves the ICM-42688-P's open-drain INT1) and D10 (LED_CTRL —
0.29 mA of LED glow at reset).

## First power-up — read this or release the smoke

1. **Verify battery polarity with a multimeter against the `+` on the top
   silk beside J1's left pad before ever plugging in.** JST-PH polarity is
   NOT standardised; roughly half of cheap packs are wired the other way.
   Reversed = every FET plus the XIAO die instantly.
   The same applies to C1 when you fit it: the belly silk reads `+ C1`
   beside its positive pad, and the footprint's silk bar is at that same
   end. C1 is a polymer/tantalum bulk cap straight across an unfused pack —
   backwards, it vents.
2. Optional: cut J3 on the Sense expansion board to drop the SD pull-ups on
   D9/D10. Not required — neither pin drives a motor. Leave it alone unless
   you want the IMU interrupt line clean, and note you lose the microSD slot.
3. Props OFF. Power via USB-C first: flash a firmware stub, check 3V3,
   scan I2C (expect 0x68 IMU, 0x29 ToF), read the battery ADC.
4. First LiPo plug-in through a fused/current-limited lead if you have one.
5. Motor test one at a time at low duty; verify spin directions.

## Design compromises (known, deliberate)

- Debug UART header dropped — use the XIAO's USB-C serial; D6/D7 remain
  free and reachable on the module's castellated pads.
- One 0.15mm trace runs under the IMU body (AP_CS→VDDIO tie) — against
  TDK's strictest layout note but standard practice at hobby scale.
- One I2C bus shared by IMU (1kHz reads) + ToF (30Hz) ≈ 40-50% bus
  utilisation at 400kHz — fine; move the IMU to SPI in a respin if you ever
  need 8kHz gyro ODR.
- JST-PH is rated 2A continuous vs ~11A full-throttle bursts — accepted
  whoop-class practice; solder a 20AWG pigtail to the J5 pads for hard use.
- The 1.6mm board is ~3g heavier than 1.2mm — chosen for first-spin crash
  survival. Arms are 12mm wide for the same reason (and to give the VBAT
  pour a clear channel past the motor traces).
- Motors are a **clearance** fit in 8.70mm holes, not a press fit: at
  JLCPCB's ±0.2mm routing tolerance a true interference fit either cracks
  the ring or vanishes entirely. Retention is the zip ties (+ a dab of CA).
- No expansion header: its 3V3 trunk had to wrap the whole right side and
  blocked the motor fan-out. J2 (ToF) already brings out 3V3 and GND.
- Both orientation LEDs hang off one GPIO (LED_CTRL), so they are always on
  or off together — nose vs tail is told by colour (yellow / red), never by
  blink pattern. D6/D7 are free if a respin wants them separated.
- Ten placed parts sit closer to a routed edge than JLCPCB's 2.5mm PCBA
  guideline (worst: LED1/LED2 ~0.5mm, D1-D4 ~0.7mm). Unavoidable on 12mm
  arms; see the ordering section.
- **C6 and C7 are pads only — deliberately not fitted.** The XIAO's 3V3 comes
  from U3, an SGM6029C 4 MHz buck whose datasheet gives an *effective* output
  capacitance range of **5-10 µF**, and the module already carries 9.4 µF
  nominal of its own. Fitting C6 (22 µF) and C7 (100 µF) put roughly 50-64 µF
  effective on that node — five to six times the specified window, on the rail
  that powers the flight controller. C2/C3/C4 stay fitted; they are IMU-local
  and add only ~1.9 µF. If bring-up shows the 3V3 rail actually sagging, hand-
  solder C6 and/or C7 then: adding to a bare pad takes five minutes, removing
  a reflowed 1206 sitting 7.3 mm from a 0.5 mm-pitch LGA does not.

## Mass / thrust budget (estimates — weigh as you build)

frame ~13g + XIAO+camera ~6g + motors 21g + props 2g + IMU/passives/
connectors ~4g + battery 18g (650mAh) + wires ~2g ≈ **66g AUW**.

Thrust is ~140g nominal, but derate it for the pack sag this board's own
SPICE model predicts and the usable figure is ~102g — so **T:W ≈ 1.6, not
2.3**. Hover sits near 60-65% throttle. That is flyable but not sprightly:
gust rejection is limited and there is no payload margin worth speaking of.

## UK legal

Camera drone: CAA **operator ID** (£11/yr, label the airframe) + free flyer
ID test, even sub-250g. Fly line-of-sight; person-following demos with a
consenting friend, indoors or private open space at walking pace.
