# Putting a bare VL53L1X on the board — researched, and NOT DONE

Researched 2026-08-22 against **ST DocID031281 Rev 3** (29-Nov-2018), read from
the PDF rather than from search summaries. The conclusion is **do not do this**,
but the pin map below is datasheet-confirmed and is here so nobody researches it
twice.

## Why not

**The part is "Standard Only" at JLCPCB.** One "Standard Only" part removes the
*entire order* from Economic PCBA — whichever side it sits on. And the optical
windows are on the **top of the package, opposite the LGA pads**, so a
downward-looking rangefinder must be **belly**-mounted, which makes it the only
bottom-side machine-placed part and forces Standard **double-sided**:

| | setup + stencil |
| --- | --- |
| Economic single-side (where rev C is now) | $9.71 |
| Standard single-side (chip on top — but then it looks UP) | $33.77 |
| Standard double-side (chip on the belly — the only useful orientation) | $67.54 |

That is **+$57.83** to save ~$50 of Pololu carriers across five boards. Net
roughly break-even, in exchange for:

- **MSL 3** (the ICM-42688-P is MSL 1 — the "no bake" note in BOM.csv does not
  extend to this part), 168 h floor life, and the standard 125 °C rebake
  **exceeds ST's 85 °C storage maximum**.
- **245 °C absolute maximum reflow**, measured at the top of the package. House
  SAC305 profiles commonly peak 245–250 °C. Convection only — vapour phase is
  explicitly unsuitable because the package is not sealed.
- **No wash, no ultrasonic, no solvent.**
- A **protective liner over the optics that survives 260 °C reflow and must be
  peeled off afterwards.** Nobody has confirmed whether JLCPCB does this. If
  they do not, five boards come back with a blind rangefinder and no error.
- **LGA-12 cannot be hand-soldered as a fallback** — twelve 0.5 mm pads entirely
  under the body, no castellations. If JLCPCB will not place it, there is no
  plan B without a stencil and hotplate.
- ST's shock clause: a module that has been dropped "must be rejected, even if
  no damage is visible". On a crash-prone airframe that is not a footnote.

Keep the Pololu 3415 carrier. It costs 7 hand-soldered joints, which is real,
but it is 7 joints against a blind-sensor risk and a tier change.

## Pin map — datasheet-confirmed, for if this is ever revisited

Package: Optical LGA12, 4.90 × 2.50 × 1.56 mm. Order code VL53L1CXV0FY/1
(= LCSC C190004). Land pattern: twelve 0.50 mm square pads, 0.80 mm pitch,
5 + 2 + 5, field 3.70 × 2.10 mm, pin-1 chamfer 0.25 × 0.08 mm.

| pin | name | connect to | note |
| --- | --- | --- | --- |
| 1 | AVDDVCSEL | +3V3 | VCSEL supply. Put the 100 nF next to **this** pin. |
| 2 | AVSSVCSEL | GND | |
| 3 | GND | GND | |
| 4 | GND2 | GND | ST forces these to ground "to avoid possible instabilities" |
| 5 | XSHUT | +3V3 via **10 k** | Active low. **Must never float** — leakage. |
| 6 | GND3 | GND | |
| 7 | GPIO1 | **leave unconnected** | Open-drain interrupt; the driver polls 0x0031 instead |
| 8 | DNC | **must be left floating** | The only pin with a hard do-not-connect |
| 9 | SDA | SDA | |
| 10 | SCL | SCL | |
| 11 | AVDD | +3V3 | Core/IO supply, separate pin from pin 1 — *not* internally bonded |
| 12 | GND4 | GND | |

There is **no IOVDD pin**. The "IOVDD" in ST's Figure 3 is the *host's* rail
feeding the I²C pull-ups. Decoupling is 100 nF + 4.7 µF on the common AVDD net.

## Optical geometry, which matters wherever the sensor ends up

Exclusion cones are **mechanical**, not performance cones, and their virtual
apexes sit essentially at the PCB plane, opening away from the board. So:

- **No copper, soldermask or silkscreen keep-out is needed** on any layer.
- Clear radius required at height *h* above the board: emitter `0.330·h`,
  receiver `0.360·h`. At an 8 mm battery pack: 2.64 / 2.88 mm.
- **Design for ≥5 mm lateral clearance from both optical axes**, and keep the
  battery and its strap out of both cones entirely — velcro, tape and heatshrink
  are all IR-reflective at 940 nm and read as a permanent ~5 mm target, below
  the 4 cm minimum range. The sensor would return a constant, not a height.
- **Nothing may bear on the cap**: 25 N maximum compressive load. A tensioned
  battery strap over the package exceeds that and cracks the optics.
- Fit **no cover glass** — open aperture is ST's own reference condition and it
  deletes two of three calibrations. Offset calibration is still mandatory after
  reflow.

## Firmware, if it is ever done

Essentially unchanged. `rangefinder.cpp` calls `init()` with `io_2v8` defaulting
true, which is correct for AVDD = 3.3 V, and R9/R10 at 4.7 k are already the
right pull-ups for 2V8 mode. The Pololu carrier's 2.8 V regulator and level
shifters are not needed at 3.3 V. Do **not** "fix" the driver for their absence.
