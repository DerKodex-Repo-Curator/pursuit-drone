# PURSUIT-1 verification log

Every gate below was run against the final `hardware/pursuit.kicad_pcb`
(KiCad 10.0.4) before the gerbers in `gerbers/fab/` were exported.

## 1. Datasheet research (6 parallel research agents, ~200 sources)

All fab-critical numbers were locked from primary sources before layout:

| Fact | Source | Status |
|---|---|---|
| XIAO row spacing 15.24mm, body 21x17.8, D0-D10→GPIO map, camera/mic use **no** header pins | Seeed official KiCad PCB v1.5 + wiki | confirmed |
| GPIO3 (D2) strapping pin → carries only the high-Z ADC divider | Seeed wiki + ESP32-S3 TRM | confirmed |
| ICM-42688-P LGA-14 pad coordinates, 1:1 NSMD land, I2C pin map (CS→VDDIO, AD0→GND, RESV7→GND, INT2/FSYNC→GND), decoupling 100n+2u2/10n | TDK DS-000347 v1.2 | confirmed |
| AO3400A SOT-23 G/S/D=1/2/3, Vgs(th) max 1.45V, 48mΩ worst-case | AOS datasheet | confirmed |
| 8520: ~2.75A full throttle, ~35g thrust, stall >5A; 32kHz PWM convention | bench measurements (Bitcraze/Oscar Liang) | confirmed |
| SS34 SMA flyback, cathode→VBAT | onsemi datasheet | confirmed |
| JST-PH polarity NOT standardised → silk warning + meter check step | multiple | confirmed |
| Motor press-fit 8.45mm unreliable alone (fab tol ±0.2) → zip-tie retention holes | PCBWay/community builds | confirmed |
| Pololu VL53L1X 3415: VDD out/VIN in, onboard 10k pullups + 47k XSHUT pullup, 0x29 | Pololu docs | confirmed |
| JLCPCB: 1.0mm min internal mill, ≥0.3 via drill at base price, 0.4mm edge clearance, Protel gerber names | jlcpcb.com capabilities | confirmed |

## 2. Netlist integrity (three-way diff)

`design.py` is the single source of truth. Automated diffs:

- design.py internal audit: 22 nets / 45 components / 113 connections, no
  single-ended nets — **PASS**
- design.py ↔ `pursuit.kicad_pcb`: **0 mismatches over 123 pads**
- design.py ↔ `pursuit.kicad_sch` (kicad-cli netlist export): **0 errors
  over 123 pins**

## 3. KiCad rule checks

- **ERC** (schematic): 0 violations
- **DRC** (board, errors severity, zones refilled): **0 violations,
  0 unconnected items** — includes copper clearance ≥0.2mm, hole-to-copper,
  edge clearance 0.4mm, courtyard overlap, solder-mask bridges
- Zone-island audit is an **executable release gate**,
  `scripts/audit_islands.py`, not a prose claim. It fails the build if any
  pour fragment holding a pad escapes through fewer than two vias. Current
  result: VBAT one polygon; GND two, the second being the reviewed waist
  pocket holding J4.2, which reaches the main pour via an 0.8mm F.Cu bridge
  and two vias at each end.

  This bullet previously asserted "single VBAT outline on F.Cu" for a board
  that had three VBAT polygons. That false claim is precisely how the D2
  flyback island survived into a third review round — which is why the
  check is now code with an exit status rather than a sentence here.

## 4. Gerber re-import audit

Parsed the exported Excellon + gerbers back from `gerbers/fab/`:

- Drill table, re-parsed from the released Excellon: 59x0.40mm, 2x0.75mm, 29x1.00mm, 2x1.20mm, 2x1.50mm, 10x3.20mm
  (104 holes). Regenerated from `gerbers/fab/pursuit.drl` rather than
  transcribed, because the previous transcription described a board that no
  longer existed (it still listed J3 and the camera-bracket holes).
- Edge.Cuts present with the closed X-outline + four 8.70mm motor circles
  (clearance fit; the 8.45mm interference fit was removed in round 2)
- All 9 layers plotted with Protel extensions + job file (JLCPCB spec)

## 5. Placement collision model

`scripts/check_place.py` — an independent SAT-polygon model of every
courtyard, hole, board edge and the corner-cut lines — reports **0
violations** (used for fast iteration; final authority is KiCad DRC).

## 6. SPICE simulation of the motor drive (ngspice)

The drive stage was simulated, not just calculated — netlists in `sim/`.

**Boot safety (`sim/worst_case.cir`)** — the claim "props cannot spin at
boot" is the life-and-death one, so it was modelled explicitly: a fifth
stage is driven by a floating pin (1 MΩ leak to 3.3 V, standing in for an
ESP32-S3 GPIO in its high-Z reset state). The 10 kΩ pulldown holds the gate
at **32.7 mV** — versus the AO3400A's 0.65 V *minimum* threshold — and motor
current is **8 nA**. Verified with a 20× margin.

| Quantity | Simulated | Rating / target |
|---|---|---|
| Gate voltage, driver floating | 32.7 mV | must be < 0.65 V → **pass, 20×** |
| Gate voltage, driven | 3.27 V | > 1.45 V worst-case Vgs(th) → pass |
| Switch-node flyback peak | 4.28 V | AO3400A 30 V, SS34 40 V → pass, 7× |
| Rds(on) implied at 3.3 V drive | ~18.7 mΩ | datasheet typ ~20-25 mΩ → matches |
| FET dissipation at 2.46 A | 0.12 W | SOT-23 → pass |
| Per-motor current, 100% duty | 2.46 A | research bench figure 2.75 A → agrees |
| Rail at 4-motor full throttle | **3.61 V** | see finding below |

**Finding — full-throttle brownout risk.** With four motors at 100% duty
(9.8 A) and a 60 mΩ source model, the pack rail sags to 3.61 V. A real
450 mAh 1S cell has higher internal resistance than modelled, so expect
worse. The XIAO's 3V3 buck enters dropout below roughly 3.5 V input, which
would brown out the ESP32 mid-flight. **Mitigation is firmware, and the
hardware for it already exists**: the VBAT_SENSE divider on D2 must drive a
throttle limiter and a low-voltage warning. This is recorded in the README
as a firmware requirement, not an optional feature.

## 7. Independent DRC (kicad-mcp)

Cross-checked with a second, independent KiCad tool path (`kicad-mcp`),
which reports *warnings* the `kicad-cli --severity-error` gate suppresses.
It initially found 43 silkscreen warnings (refdes colliding with each other
and sitting over pads, two sub-minimum text heights, one label overhanging
the board edge). All were fixed by moving reference designators to the Fab
layer — the assembly-drawing convention — and relocating the board texts.

Final state, both tool paths: **0 errors and 0 warnings.**

## 8. Adversarial design review

Seven independent review agents were launched (power/drive, IMU subsystem,
XIAO pin map, mechanical, fab rules, firmware bring-up, systems
integration). **One completed before the session usage limit stopped the
other six** — an honest gap in coverage, recorded here rather than papered
over. The completed reviewer (XIAO pin map) returned PASS_WITH_NOTES and
found two real documentation defects, both fixed:

1. **DESIGN.md's pin table contradicted the canonical `design.py`** on
   D0/D1/D2. Firmware written from the stale table would have driven M1 on
   M2's gate and PWM'd the battery-sense divider — a wrong motor mix, i.e.
   a flip on first takeoff. No fabrication impact; the board was always
   built from `design.py`. Table and diagram corrected.
2. **The Sense expansion's microSD slot is hard-wired to GPIO7/8/9** =
   M4_PWM / IMU_INT / LED_CTRL. Harmless with no card fitted (lines open),
   but a card would fight the IMU interrupt. Now a documented standing
   constraint in DESIGN.md.

It also confirmed clean: the full D→GPIO map against the Seeed wiki, that
GPIO1/2/4/7/9 are not strapping pins, that the 10 kΩ pulldowns prevent
boot-time motor glitches, that GPIO3's strap is inert unless an eFuse is
burned, ADC1 usability with WiFi active, and carrier-footprint pad
numbering consistency across `footprints.py`/`design.py`/`routes.py`.

**Not yet covered by adversarial review:** power/drive, IMU subsystem,
mechanical, fab rules, firmware bring-up, systems integration. The
simulation in §6 independently covers much of the power/drive dimension.

## 9. Adversarial review, round 2 — and the rework it forced

Four independent reviewers (power/drive, IMU+I2C, mechanical/aero,
fab/assembly) attacked the finished board. **All four returned FAIL.** The
findings were specific, numeric and correct; several would have destroyed
hardware. Every one below was fixed and re-verified.

| Severity | Finding | Fix |
|---|---|---|
| BLOCKER | FET **source** — the motor return — was a 0.3mm stub and ONE 0.4mm via per motor, 7x less copper than the drain carrying the identical 2.75A, and a single point of failure on a flexing airframe | 1.2mm trace into a **4-via cluster** per motor, placed hard against the pad |
| BLOCKER | Motor ring webs were 1.075mm nominal with **no tolerance window**: at JLCPCB's -0.2mm the 8.45mm "press fit" becomes 0.25mm interference and the hoop stress at the tie holes reaches ~4.5x FR4's tensile strength — the ring cracks as the motor goes in | Hole -> **8.70mm clearance fit** (retention is the zip ties), ring 9.6 -> **10.6mm**, ties 6.9 -> **7.5mm**: webs 1.55/1.50mm |
| BLOCKER | The module footprint was perfectly symmetric with **no orientation marking** — a 180-deg insertion puts the XIAO's 3V3 output on SDA and kills it | USB edge marked **three ways**: notched outline, "USB" text, and a pin-1 tick outboard of the row that survives the fitted header |
| BLOCKER | Gerbers and assembly SVGs were **one revision behind** the board | Everything re-exported from the current board and re-audited |
| MAJOR | Both pours ran **0.41mm from the motor holes** on both faces — the metal can spans the full 1.6mm and bridges VBAT to GND | Circular copper keepout, r=5.7mm, at every motor |
| MAJOR | Metal M2 camera-bracket hardware shorts VBAT (head) to GND (nut) | **Screw holes removed**; camera mounts on a foam/printed edge clip |
| MAJOR | IMU decoupling **cross-connected**: the 100nF sat 45mm of 0.3mm trace from VDD, the 10nF sat on VDD instead of VDDIO | Values swapped between the two sites — zero-layout fix, VDD now has 100nF at 3.9mm |
| MAJOR | **No ceramic anywhere on VBAT**; C1 was 30-45mm from three of four FETs and cannot supply a 2.75A edge in tens of ns | 4x 100nF X7R added on the belly at each flyback, tied straight to D's VBAT pad |
| MAJOR | Both pours **solid-connected every hand-soldered pad** — cold joints guaranteed on the 10A pads | Thermal relief everywhere, with solid override kept only for the true battery-current terminals |
| MAJOR | C1's footprint was a 6.3mm aluminium can but the BOM named a 7343 polymer; fitting an aluminium part gives 150-600mOhm ESR against 23mOhm of reactance — the bulk cap would do nothing while cooking | Land changed to 7343/D; BOM now specifies **ESR <= 30mOhm** as a hard constraint |
| MINOR | IMU paste apertures 1:1 on a 1:1 land — area ratio 0.55, below the IPC-7525 floor; paste would not release on any of the 14 pads | Lands grown to 0.625 x 0.35 (centres unchanged, matches KiCad's own LGA-14 pattern), AR 0.94 |
| MINOR | The IMU keepout was **inverted** — it permitted tracks (the harmful case) and blocked the B.Cu ground pour (the useful one) | Restricted to F.Cu, so the ground plane under the sensor is back |
| MINOR | Silk strokes 0.12mm and some text 0.9mm, both under JLCPCB's floors | All silk >= 1.0mm text / 0.15mm stroke, enforced in the emitter |

Two changes fell out of the rework rather than a finding: the arms went from
10mm to **12mm** (the VBAT pour was being pinched into an island between the
M_SW jog and the LED lane, stranding a motor's supply), and each arm now
carries an **explicit 1.2mm VBAT spine** from the plate to its flyback and
motor pad, so motor power never depends on pour topology. The J3 expansion
header was deleted — its bottom-layer 3V3 trunk had to wrap the entire right
side and was blocking the motor fan-out.

## Known accepted deviations

- 0.15mm AP_CS→VDDIO tie runs under the IMU body (TDK prefers nothing
  under the package; standard at hobby scale).
- One 0.3mm bottom trace transits the antenna keepout's x-band south of
  the U.FL area (outside the marked keepout rectangle itself).
- All silkscreen text is now ≥1.0 mm (JLC's legibility floor); reference
  designators live on the Fab layer, so the physical board carries only
  functional markings. Use `docs/assembly_top.svg` / `assembly_bottom.svg`
  while soldering.


## 10. Adversarial review, round 3 — the rework reviewed

Four reviewers attacked the *rework itself*, on the principle that fixes
introduce their own defects. Two returned FAIL. Findings and fixes:

| Severity | Finding | Fix |
|---|---|---|
| BLOCKER | **The flyback island came back.** Arm 2's VBAT pour was pinched into a 33mm² fragment by the LED1_A lane crossing the FET source cluster. D2's cathode — carrying 100% of motor 2's freewheel current — reached the rest of the net through exactly ONE 0.4mm via. Same defect class as the round-2 source blocker, on a board that had passed DRC with 0 errors. | LED1_A lane moved outboard to arm-perp +5.0, and **two redundant VBAT vias added at every flyback cathode**. Plus `audit_islands.py` as an executable release gate. |
| BLOCKER | **C1 had no polarity marking, and the silk that was there read backwards.** A hand-soldered polarised tantalum across an unfused 1S LiPo, whose only printed cue was a silk box wrapping the ANODE — which reads exactly like the electrolytic "band = negative" convention. | Explicit mirrored `+` and `C1+` on the belly silk beside pad 1. |
| MAJOR | All four gate-pulldown ground vias sat **0.025mm** from their own pad — below any fab's etch resolution, so pad and barrel merge and solder wicks into the via. These are the pulldowns whose only job is "props cannot spin at boot". DRC was silent because it is the same net. | Offset 0.9 → 1.35mm; measured edge gap now 0.475mm on all four. |
| MAJOR | A **stale pre-rework gerber set** was still present under the canonical release name, alongside two duplicate copies. It contained the single-via FET source, the ring-cracking press fit, and the VBAT/GND-shorting camera holes. | All stale folders deleted; exactly one release folder, carrying a `MANIFEST.txt` stamped with the board's SHA-256. |
| MAJOR | This document certified audit results the board no longer had — including the island audit that would have caught the blocker above. | Sections 3 and 4 regenerated from measured output. |
| MINOR | U1 and J4 were in the solid zone-connect override on a false justification ("battery-current terminals"). | Kept solid, but for the true reason: their annulus is too narrow for a spoke to form. Comment corrected. |
| ELECTRICAL | The only white 0805 LED with real stock has Vf 3.3V — driven from a 3.3V GPIO through 470R it would be **dark**. | LED1 → C84258, R13/R14 470R → 150R. **This fix was wrong — see round 4.** |

Board hash at round-3 release: `27a351f6ca1e4093`


## 11. Adversarial review, round 4 — the order artefacts reviewed

Rounds 1-3 attacked the board. Round 4 attacked the *files that get uploaded*
and the *claims this document makes*, because both errors that had actually
reached the customer by then (a CPL origin 150mm off the board, an LCSC code
for a 12k 0402 on a 150R line) were in that layer, not in the copper.

Three reviewers. One returned DO_NOT_ORDER, two ORDER_WITH_NOTES.

| Severity | Finding | Disposition |
|---|---|---|
| BLOCKER | **The round-3 C1 polarity fix was not on the board it certifies.** The only `+` legend near C1 was a `PIN1=+` on **F.SilkS** — the top face — for a part soldered on the belly, positioned at x≈141.5, i.e. on the **pad-2 (GND)** side of C1's centreline. The claimed `C1+` text did not exist anywhere. | Confirmed by measurement. Both markers rebuilt as glyphs placed at the + pad of the part they belong to and on that part's own face: `+` on F.SilkS beside J1 pad 1, `+ C1` on B.SilkS (mirrored) beside C1 pad 1. Now gated by preflight §7. |
| MAJOR | **C84258 is Vf 2.6-3.6 V, not "Vf 2.6 V max".** 2.6V is the bottom of the bin. Verified against the live JLCPCB listing: `"2.6V~3.6V"`, Extended. Every white/blue 0805 is InGaN and behaves the same, so no resistor value rescues it — the upper bins never conduct from a 3.3V GPIO. The round-3 fix replaced a dark LED with a coin-flip LED. | LED1 → **C84261** (NationStar NCD0805Y1, yellow 0805, Vf 1.6-2.6V — same family and bin range as the already-verified red C84256). Worst-case 3.7mA, typical ~7.7mA. Front/rear still distinguishable by colour. R13/R14 stay at 150R. |
| MAJOR | Five to ten placed parts violate JLCPCB's 2.5mm component-to-board-edge PCBA guideline (measured: LED1/LED2 ~0.5mm, D1-D4 ~0.7mm, C8/R11/R12/R14 ~1.2mm). | ~~**Accepted, not fixed.** Inherent to a 12mm-arm X-frame.~~ **THE MEASUREMENTS WERE WRONG — see §15.** Re-measured 2026-08-22 against the real Edge.Cuts polyline: D1-D4 are at **4.65mm** and LED1 at **2.57mm**, both compliant. The "inherent to the airframe" conclusion, and the bare-board fallback it justified, rested on numbers off by up to 6.6x. |
| MAJOR | Order specified HASL, but U2 is a bottom-terminated LGA-14 on 0.5mm pitch with hidden fillets. | **ENIG** now specified in README. No file change. |
| MINOR | A stale `PURSUIT-1_placement.csv` (mtime older than the board, still carrying `470R` for R13/R14 and listing bottom-side + THT parts) sat in `gerbers/` under a name that reads like the pick-and-place file. | Deleted. preflight now fails on *any* file in `gerbers/` older than the board, and on any unexpected `.csv` in that directory. |
| MINOR | **The origin gate did not work.** `re.findall(r"X(-?\d+)", edge)` also matched the format-spec line `%FSLAX46Y46*%`, yielding a phantom X=0.000046 that made the lower bound vacuous — and there was no Y check at all, though the bug it guards against was a two-axis offset. | Rewritten to parse only `X..Y..D0[123]` draw operations and to bound **both** axes against the Edge.Cuts bbox. |
| MINOR | `gerbers/BOM.csv` listed all 31 JLCPCB-supplied parts with no marking, told the builder to hand-solder U2, and priced the IMU at "~GBP 2-4" when LCSC's live price is **$17 with a 100pc MOQ**. Used as a shopping list it buys parts twice and budgets the most expensive line 5-8x low. | Rewritten with a `Supplied by` column (JLCPCB / YOU), corrected IMU price and MOQ, corrected LED and DC-bias-capacitance claims. |
| NOTE | Both orientation LEDs share one GPIO; C7's 100uF retains only ~25-35uF at 3.3V bias. | Neither is a defect. Both recorded in README's compromises list so the numbers are not re-derived later. |
| NOTE | One reviewer argued the CPL should carry a -90° correction on Q1-Q4 because the footprint matches the community database's `^SOT-23` rule. | **Not applied.** Those databases disagree with each other on this package and there is no official JLCPCB table; a blind correction can only turn a maybe-right rotation into a definitely-wrong one. The mitigation is Confirm Parts Placement, already mandatory, with `ORIENTATION_CHECK.csv` as the checklist. |

Two things fell out of the rework rather than a finding. The XIAO's `USB`
silk label sat 0.5mm from the front edge and was being printed as half-height
letters (`silk_edge_clearance`); it now sits inside a deepened port cut-out,
and DRC is at **0 violations including warnings**. And `scripts/release.py`
now owns the export flags: the shipped round-3 package had been exported with
`--subtract-soldermask`, a fact recorded nowhere, so a plain re-export
silently produced a different silkscreen. Regenerating without it changed
1,900 lines across both silk layers and nothing flagged it.

**Verification that the round-4 rework touched only what it claimed.** The
board was rebuilt from the generators and every exported layer compared
against the round-3 package after resolving D-codes to aperture definitions
(so renumbering cancels): `F_Cu`, `B_Cu`, `F_Mask`, `B_Mask`, `F_Paste`,
`B_Paste`, `Edge_Cuts`, the drill map and `pursuit.drl` are all **geometrically
identical**. `F_Silkscreen` and `B_Silkscreen` are the only files that changed.
Netlist re-diffed against `design.py`: 113/113 connections, 0 missing, 0 extra,
0 value mismatches.

Board hash at round-4 release: `067b16c6e2bff608`


## 12. Adversarial review, round 5 — the pre-payment audit

Seven reviewers, one per dimension, each with two adversarial skeptics tasked
with refuting whatever they raised. The session's usage cap killed 44 of the
55 agents, so most findings arrived **unverified** and were re-measured by
hand against the board before anything was changed. Two verdicts came back
DO_NOT_ORDER.

| Severity | Finding | Disposition |
|---|---|---|
| BLOCKER | **Motor 4 spins at power-on.** The XIAO ESP32S3 *Sense* expansion board carries 4.7 kΩ pull-ups (R4-R6, live by default through the J3 solder pad) on the microSD SPI lines. SCK is **D8/GPIO7** = U1 pad 13 = **M4_PWM**. So D8 is not floating at reset, and with a 10 kΩ pulldown M4's gate sits at `3.3 x 10k/(4.7k+100R+10k)` = **2.23 V** against an AO3400A Vgs(th) of 0.65 V minimum. Motor 4 drives hard from the moment the pack is connected until firmware runs (~200 ms+), and again on every reset or brownout. The board's headline safety claim — "the props cannot spin at boot" — was true of the bare PCB and false of the machine as specified. | **Confirmed by measurement and by Seeed's own documentation.** R5-R8 changed 10 kΩ → **1 kΩ** (C21190, Basic, same Uniroyal family as every other resistor here): boot gate 0.569 V, sub-threshold current ~30 µA at 25 °C and ~190 µA at 60 °C against the ~50 mA an 8520 needs to twitch; driven gate still 3.00 V, above the 2.5 V at which Rds(on) is specified. All four changed, not just R8, so gate drive stays equal across the four motors. Cutting J3 added as a belt-and-braces bring-up step. |
| MAJOR | **The assembly drawings were a revision behind.** `docs/assembly_top.svg` and `assembly_bottom.svg` still carried the deleted `PIN1=+` marking — i.e. the round-4 polarity defect — and README tells the builder to solder from them. Found independently by two dimensions. | Confirmed (mtime and a literal `PIN1=+` in the SVG). `release.py` now regenerates both with the board, and preflight fails if either predates it. |
| MINOR→REAL | **127 of the 141 silk graphics were 0.12 mm stroke**, below JLCPCB's 0.153 mm floor — including C1's polarity bar, all four diode cathode bars, the LED cathode marks and U2's pin-1 dot. These come from the *stock KiCad footprints*, not our emitter, so §9's claim that "all silk >= 0.15 mm stroke, enforced in the emitter" was true only of the text. | Confirmed by measurement. `enforce_silk_minimums()` now raises every silk stroke to 0.15 mm after placement; the rebuild reports 127 raised. |
| MINOR | The bare `+` added in round 4 measures 2.952 mm to C4 pad 1 and 3.200 mm to J1 pad 1 — marginally nearer the wrong part, and preflight §7 structurally could not detect that. | Changed to **`J1+`**, which is self-identifying regardless of distance. (C4 is an unpolarised 0603, so the misread was never dangerous, but the ambiguity is gone.) |
| MINOR | README claimed a `114 x 114 mm` bounding box; measured `91.91 x 91.91 mm`. | Corrected. |
| MAJOR | No fiducials, no tooling holes, no process edge, and no straight outer edge to clamp — JLCPCB's stated default is to add rails and fiducials themselves, after payment, to an outline the customer never approved. | **Accepted, not fixed.** This is how every JLCPCB PCBA order of an irregular outline works: they panelise onto their own rail frame carrying their own fiducials, and the board comes back with tab stubs on the arm tips to file off. Adding three fiducial dots would punch three voids in the VBAT pour — the exact defect class that produced the round-2 and round-3 blockers — and the benefit over their panel fiducials is marginal. Documented as an expected engineering note instead. |
| MAJOR | `+3V3` bulk (C7 100 µF + C6 22 µF) may exceed the XIAO's regulator maximum output capacitance. | **Not actionable before ordering** and not resolvable from the module's published data. Recorded as a bring-up check: if 3V3 fails to come up, lift C7 and retest. |
| MINOR | ICM-42688-P INT1 resets to open-drain and IMU_INT has no pull-up on this board. | Now moot in the shipping configuration — the Sense board's 4.7 kΩ pull-up on D9/GPIO8 is exactly that pull-up. If J3 is cut, configure INT1 as push-pull in firmware instead. |

**Verification that the round-5 rework touched only what it claims.** Rebuilt
from the generators and compared against the round-4 package with D-codes
resolved to aperture definitions: `F_Cu`, `B_Cu`, `F_Mask`, `B_Mask`,
`F_Paste`, `B_Paste`, `Edge_Cuts`, the drill map and `pursuit.drl` are all
**geometrically identical**. Only the two silkscreen layers changed. The
resistor change is a value/BOM change with no geometry. Netlist re-diffed
against `design.py`: 113/113 connections, 0 missing, 0 extra, 0 value
mismatches. DRC 0 violations including warnings. Zone-island audit passes.

Board hash at round-5 release: `6b86f2c7b846d719`


## 13. Adversarial review, round 6 — and a retraction

Round 6 re-ran five reviewers against the round-5 board. Its most important
result is that **the round-5 blocker was wrong**, and the correction matters
more than the original finding did.

### RETRACTED: the D8 pull-up

§12 recorded, as a confirmed BLOCKER, that the XIAO Sense expansion board
pulls **D8/SCK** up through 4.7 kΩ and that motor 4 would therefore spin at
power-on. That came from a Seeed *forum post* which loosely said the pull-ups
sit on "SCK, MISO and MOSI". It is false.

Seeed's own schematic — `XIAO_ESP32S3_ExpBoard_v1.0_SCH.pdf`, sha256
`8224252d259a7ab09d19748a461970627f8b28029857a224c747e0de25040c0c`, rendered
at 900 dpi so the junction dots resolve — shows the three 4.7 kΩ resistors off
JP3 landing as:

| | |
|---|---|
| R4 | dot on **D9/MISO** |
| R5 | crosses D9 and D8 with no dot, dots on **D10/MOSI** |
| R6 | crosses three nets with no dot, dots on **SD_CS** (→ IO21/USER_LED via R12 = 0 Ω; R11 → D2 is DNP) |
| D8/SCK | **no pull-up** — only R2 = 22 Ω in series to the socket CLK pin, plus one ESDPSA0402V05 clamp |

D8/GPIO7 is M4_PWM on this board. Motor 4 was never on a divider; all four
gates float at reset and sit at ~0 V, as originally designed.

**R5-R8 stay at 1 kΩ.** The value is fine — better than 10 kΩ, in fact — but
for a different reason, now recorded in `design.py`: at 10 kΩ an *unexpected*
pull-up as weak as the ESP32-S3's own ~45 kΩ internal one would put a gate at
0.60 V, within 50 mV of the AO3400A's 0.65 V minimum threshold. At 1 kΩ the
same fault gives 72 mV. Nothing that gets uploaded was wrong; four documents
asserting a false fact were.

Lesson, and it is the same one as rounds 3 and 4 in a new costume: **a forum
post is not a schematic.** The primary document existed, was free, and took
four minutes to read.

### NEW BLOCKER: 3V3 output capacitance exceeds the regulator's spec

| Severity | Finding | Disposition |
|---|---|---|
| BLOCKER | The XIAO's 3V3 rail is **U3 = SGM6029CYG/TR**, a 4 MHz synchronous buck (L4 = 470 nH, its own C5 + C6 = 4.7 µF each, Seeed annotate "Imax=600mA"). SGMicro's datasheet, Recommended Operating Conditions: `Effective Output Capacitance, COUT — SGM6029A/SGM6029B/SGM6029C ... 5µF to 10µF`. That is a range with an **upper** bound, and the XIAO uses the C variant. Anything on the 3V3 castellation lands on that node. With DC-bias derating C6 keeps ~18-20 µF and C7 ~25-35 µF, so fitting both put **~50-64 µF effective on a node specified for 5-10 µF** — 5× to 6× over, on the rail that powers the flight controller. The part's VOS pin feeds the modulator ramp from output ripple, and ripple scales as 1/COUT, so over-capacitance attacks the control loop itself; separately, charging ~50 µF inside a few hundred µs of soft-start needs ~360 mA on a 600 mA rail before the ESP32-S3 has booted. | **Confirmed independently** — regulator identified from Seeed's core schematic at 900 dpi, limit read from SGMicro's own datasheet text. C6 and C7 are now marked `dnf=True` in `design.py`: footprints and routing stay on the board, but both are excluded from the JLCPCB BOM and CPL. C2/C3/C4 remain (IMU-local, ~1.9 µF effective), leaving the node at ~7-9 µF — inside the window. Assembly drops from 31 placements to **29**, and from 15 BOM lines to **13**. |

The disposition is least-regret rather than certain: the datasheet gives a
*recommendation*, not an absolute maximum, and plenty of designs run bucks
outside it. But adding capacitance to a bare pad takes five minutes, while
removing a reflowed 1206 sitting 7.3 mm from a 0.5 mm-pitch LGA does not — so
the asymmetry decides it. If bring-up shows the 3V3 rail sagging, fit them.

`preflight.py` now understands `dnf`, so a do-not-fit part is excluded from
the assembled set rather than tripping the BOM/CPL coverage check.

### Coverage caveat

The session's usage cap killed 15 of 23 agents, including every skeptic on the
`module-integration` and `carryover` dimensions and the whole `upload-files`
reviewer. Both findings above were therefore re-verified **by hand** against
primary documents before anything was changed — but the unverified carryover
list in §12 is still unverified, and `upload-files` did not run at all this
round. The board, gerbers and drill are unchanged from §12, which `upload-files`
did clear in the previous round.

Board hash at release: `6b86f2c7b846d719` (unchanged — this round changed the
BOM/CPL and the documentation only)

---

## 14. Rev B — the bring-up defects designed out

Rev A flew nothing. Every hardware fault found during bring-up was a hand-made
solder joint, never a part and never the netlist: five cold joints (two motor
pins, two header pins, the 3V3 socket) and then two lifted annular rings when
those joints were reworked, which is what finally ended that board. This round
changes the design so the same builder gets a different outcome.

### The root cause, and the one number that fixes it

A 1.8mm pad on a 1.0mm drill leaves a **0.40mm annulus**. The zone's thermal
spoke was also **0.40mm**. A spoke has to fit inside the annular ring, so none
could be placed, and KiCad quietly fell back to a **solid** pour connection on
every through-hole pad touching a plane. A solid tie into 2200mm² of 1oz copper
carries heat away from the joint faster than a hand iron delivers it — so the
pads that were hardest to solder were exactly the ones carrying the plane.

The spoke is now **0.30mm** (`gen_pcb.add_zone`). Relief forms in the annulus
that already exists. Two 0.3mm spokes in 1oz carry ~0.9A each (IPC-2221, 10°C
rise) against a module whose worst case is a WiFi TX burst under 0.6A. The four
motor terminals, J1 and J5 — the pads that really do carry the pack — are still
forced solid in `build()`.

### What was tried first and abandoned

Widening the pads until a 0.4mm spoke fits. It does not work on this board and
the DRC said so twice:

| Attempt | Result |
| --- | --- |
| XIAO pads 1.8 → 3.0mm, symmetric | 16 tracks in violation. The waist between the two rows carries every signal on the board. |
| Same, grown outboard only via a drill offset | Still 16. SCL, M2_PWM, LED_CTRL and VBAT_SENSE run in the channel **outboard** of the pad column too. |
| Grown along the row (2.3mm) | 12 more. The 0.74mm gaps **between** pads in a column are themselves routing lanes. |
| J4 pads, any direction | 12. SDA and SCL thread the gap between its two pads; the +3V3 corridor and C6 sit immediately outboard. |

The XIAO carrier cannot grow in any direction, and that is now written into
`footprints.py` so the next person does not spend the afternoon rediscovering
it. Growing pads was the intuitive fix; changing the spoke was the correct one.

### A latent defect this round created and caught

The outboard-offset attempt wrote the offset into the pad's `(at ...)` and the
negated offset into `(drill ... (offset ...))`. That is backwards: `at` is the
pad **anchor**, which for a through-hole pad is the hole, and the drill offset
moves the **copper**. The result was fourteen XIAO holes 0.6mm off the module's
pin grid — a board no XIAO would seat in — while every render still looked
correct. DRC surfaced it only indirectly, as a 0.14mm pad-to-pad clearance
error inside the motor footprint.

`footprints._check_holes` now reads the holes back out of each generated
footprint and asserts the pitch of the part that mates with it. Re-introducing
the inversion fails the build with `hole spacing across the rows is 16.44,
expected 15.24`.

### A blind spot in the island audit

`audit_islands.py` attributed a pad to the fill polygon containing its centre.
A thermally relieved pad sits in a **hole** in that polygon, so the audit saw
no pad there — for every relieved pad on the board. Before this round almost
every pour pad was solid and the distinction never showed; with relief it would
have gone blind exactly when it started to matter. It now tests a ring of
points just outside the relief gap, which works for solid and relieved alike.

Running the corrected audit against the rev A board reproduces rev A's result
exactly, so the fix changed the gate's sensitivity, not its verdict.

### J4 keeps its solid tie

Relieving J4.1 splits the F.Cu VBAT pour into 2281mm² plus a stranded 91mm²
pocket: that pad is the only place the pour crosses the waist, and the +3V3
corridor at y5.7 and the J2 pad row fence the gap completely. J4 was not one of
the five failures. It stays solid and the plane stays whole — VBAT is now a
**single polygon**, where rev A had it fragmented around R11.

### Also in rev B

| Change | Why |
| --- | --- |
| Divider R11/R12 100k → 10k (`C25803` → `C25804`) | 100k/100k presents ~50kΩ to an ESP32-S3 ADC that needs <10kΩ to charge its sample-and-hold. Bring-up read a textbook battery sag under throttle, r²≈1.0 against PWM duty, that was entirely an artefact of the divider. 10k/10k presents 5kΩ and draws 210µA. |
| TP1/TP2/TP3 — VBAT, 3V3, GND | Rev A had no test point. Every rail measurement was taken on a pad that was also a joint, and a probe on a marginal joint measures the probe. Two of those pads eventually lifted. The row sits on the north plate, the only large area **not** under the module (the XIAO body covers x ±8.9, y −18.5..+2.5), with VBAT / 3V3 / GND on the top silk beside it. |
| Motor pads 2.0 → 2.4 × 2.0 oval, copper 0.30mm outboard | Annulus 0.50 → 1.00mm on the pads most likely to be reworked — a motor swap is the repair this airframe needs repeatedly. Offsetting outboard also opens pad-to-pad from 0.54 to 0.74mm, where growing symmetrically would have closed it to 0.14mm. |
| J5 pads 2.8 → 3.9 × 3.3 oval, copper 0.6mm outboard | Annulus 0.65 → 1.80mm, pad gap 0.50 → 1.70mm. J5 is where rev A lost an annular ring outright. Moved to (−18.6, −8.5) to clear Q1. |
| `board_probe_points.png` retired | A hand-annotated bring-up screenshot that no generator produced, so it could only go stale, and did. The board now carries the same information on its own silk. |

### Gate results

| Gate | Rev A | Rev B |
| --- | --- | --- |
| `check_place.py` | 0 | **0** |
| `check_sch.py` | 22 nets / 116 pins | **22 nets / 116 pins** |
| DRC (errors + warnings, zones filled) | 7 violations | **0 violations, 0 unconnected** |
| `audit_islands.py` | FAIL — VBAT fragmented | **PASS — VBAT one polygon** |
| `preflight.py` | — | **0 failures, 0 warnings → CLEAR TO ORDER** |

Board hash at release: `36ffab05e1d2544b`

### Not done

The BOM still splits at the same place: headers, the JST and C1 remain "YOU"
rather than JLCPCB assembly. Moving them needs LCSC codes verified the way
`C25804` was verified here, and an unverified code silently fits the wrong
part. The snapshot database also reported 0 stock for `C25804`, which for the
most common resistor value in existence reads as a stale snapshot rather than a
real shortage — re-check at order time.

---

## 15. The component-to-edge claim was folklore

Rounds 4 and 5 recorded that this board cannot meet JLCPCB's 2.5mm
component-to-board-edge rule, that the cause was "inherent to a 12mm-arm
X-frame", and that the fallback was to order bare boards and hand-build them.
That conclusion was accepted twice and written into README's ordering section.

It was measured against nothing. `check_place.py` had no body-to-edge rule at
all — its only edge test compared *courtyard* corners against `PLATE` and
`DIAG`, hand-maintained constants that model the outline rather than being it.

Measuring package **bodies** against the actual `Edge.Cuts` polyline and the
four prop rings:

| part | claim in the docs | measured |
| --- | --- | --- |
| D1-D4 | ~0.7 mm | **4.65 mm** |
| LED1 | ~0.5 mm | **2.57 mm** |
| LED2 | ~0.5 mm | **0.87 mm** |
| C8, R11, R12, R14 | ~1.2 mm | 1.60 mm |
| C10-C13 | not mentioned | 1.80 mm |

Nine parts are inside 2.5 mm; none is inside the 0.3 mm depanelling-risk line.
The two the docs singled out as worst are the two that were already fine, and
the actual worst offender — LED2 — was never named. Every one of the nine is a
plate or arm part that moves by millimetres.

The cost of the error was not a bad board; it was a bad *decision*. "The
airframe cannot meet this" ended the enquiry, and the fallback it justified
was giving up on machine assembly — on a project whose every hardware failure
has been a hand-soldered joint.

Now gated: `check_place.py` FAILs below 0.30 mm and WARNs below 2.50 mm,
measured from rotated body corners to the real outline. It also gained a
zero-back-side-placements gate, because JLCPCB's Economic PCBA has no
double-sided option at any price — one belly part moves the whole order to
Standard, roughly $58 of fixed fees, about ten times what two extra copper
layers cost.

---

## 16. Rev C, part 1 — four layers, and the gate that would have shipped them blank

The board is now four copper layers: **L1 signal / L2 solid GND / L3 solid VBAT
/ L4 signal**. The inner planes are ADDITIVE — the rev B outer pours stay
exactly where they were, so every hand-derived track in `routes.py` keeps its
lane and nothing had to be re-routed by hand.

### The defect this round nearly shipped

`release.py` listed its export layers as a literal:

    GERBER_LAYERS = ("F.Cu,B.Cu,F.Paste,...")

and `preflight.py` checked the zip against a hardcoded list of ten filenames.
Both survive a 2 → 4 layer change **without a word**. The board would have been
four layers, the fab would have built a four-layer stackup, and the gerbers
would have carried two **blank inner layers**. Every gate green, order placed,
five dead boards.

Both sides now derive the layer set from the board itself, independently:
`release.py` parses the `(layers …)` block; `preflight.py` cross-checks
`GetCopperLayerCount()` against the gerber job file's `LayerNumber` **and**
against the count of `Copper,Ln` FileFunction entries — matching on function
rather than filename, because the Protel inner-layer extensions (`.g1`, `.g2`)
are another convention not worth hardcoding.

Negative-tested: forcing the job file to claim 4 layers on the 2-layer board
gives `FAIL gbrjob says 4 copper layers, board has 2` → DO NOT ORDER.

### The second one, which is worse because it is invisible

`add_keepout()` and `add_round_keepout()` defaulted to `(F_Cu, B_Cu)`.

Adding inner planes without touching that leaves the four motor keepouts
holding copper clear on the outer layers **only**. The metal motor cans span
the full board thickness. An In2 VBAT plane and an In1 GND plane, both flooding
under a motor can, is a pack short through the airframe — and **DRC has nothing
to say about it**, because the planes are legitimately there and the motor can
is not in the design at all.

Both now default to `all_copper()`, derived from `COPPER_LAYERS`. Verified on
the built board: all four motor keepouts and the antenna keepout span
`F.Cu,In1.Cu,In2.Cu,B.Cu`, while the IMU keepout correctly stays F.Cu-only
(deliberate — the pour is wanted under the sensor as its reference).

### Also corrected

Copper layers were being sorted by KiCad's internal layer id, which numbers
F.Cu 0 and B.Cu 2 with the inners *above* both — yielding `F, B, In1, In2`.
That is not a stackup, and it was about to become the order in the gerber job.
Now sorted by physical position; the job file declares L1 Top, L2 Inr, L3 Inr,
L4 Bot.

### New: `make board`

One command for the whole chain, because every step of it has been got wrong at
least once — the two interpreters, the out-of-process zone fill, and above all
**never piping a gate**: `kicad-cli … --exit-code-violations | tail -3` reports
*tail's* exit code. A run that returned 5 with 199 violations was once read as
clean through exactly that. Every gate now writes to a file and is read after.

### New: plane renders

Inner copper appears in no other artefact — not the 3D render, not the assembly
drawings, not the silk. `docs/plane_in1_gnd.svg` and `plane_in2_vbat.svg` are
the only way a human ever sees L2 and L3, and both are staleness-gated.

### Result

| | rev B (2 layer) | rev C (4 layer) |
| --- | --- | --- |
| DRC, `--severity-all` | 0 / 0 | **0 / 0** |
| Zone-island audit | PASS | **PASS** |
| GND plane | B.Cu 2341.8 mm² | **In1.Cu 2839.1 mm², 1 polygon** |
| VBAT plane | F.Cu 2378.3 mm² | **In2.Cu 2858.2 mm², 1 polygon** |
| Gerber files in zip | 13 | **15** |
| Order gate | CLEAR | **CLEAR** |

The In2 VBAT plane is tied at **all four motor terminals plus J1, J4, J5 and
TP1** — eight through-hole pads — so it genuinely parallels the pack path
rather than merely existing. `check_ground.py` (itself repaired this week, see
§14) now selects the inner planes as the reference for both nets and reports
zero fragments off-plane.

**Netlist unchanged.** This round is four layers and nothing else, so it is a
clean baseline to add the INA226, the keyed J4, the merged battery input and
the flow connector against.

---

## 17. Rev C, part 2 — the belly caps, and what the board refused

`C10-C13`, the four VBAT bypass ceramics at the flybacks, have moved from the
**belly to the top face**. Four fewer hand joints, and bottom-side SMD drops
from five parts to one.

It was not a one-line move. Each step below was found by DRC, not by planning:

| attempt | what broke |
| --- | --- |
| move to top at perp 2.7 (also clearing the 2.5mm edge rule) | the cap's GROUND pad closed to **0.8mm centre-to-centre** with the flyback's VBAT via cluster at perp 1.9 — zero copper clearance, nine violations |
| back to perp 3.4, GND via at perp 4.6 | on **arm 2 only**, the via landed on the LED1_A lane |
| arm 2 via to perp 5.1 | still on it — `routes.py`'s own docstring says "lane +3.9", but measured in arm-local coordinates LED1_A actually **runs at perp 5.00** for r 26..35.8. 3.9 is where it enters, not where it runs. |
| arm 2 via to perp 4.2 | 0.15mm from the cap's own **VBAT** pad — the one pad it must not touch |
| arm 2 via to (r 30.2, perp 4.2) | clean |

The lesson is in row three. The docstring was not wrong so much as describing a
different thing, and three iterations were spent trusting it. Measuring the
actual track in arm-local coordinates settled it in one query.

Also: the arm legends moved from perp +3.9 to **−3.9**, because the caps came
up into +3.4 and silk printed across a mask aperture ends up on the solder.
−3.9 is where `PURSUIT-1` has always sat on arm 1, so the set is now consistent
rather than merely clear. The revision mark reads **REV C**.

### What the board refused

`C1`, the 220 µF 7343 polymer, was to become six 100 µF 1206 ceramics — the
reasoning was good (C1 is polarity-critical, hand-fitted, and sits across an
unfused LiPo, so reversed it vents) and it is **backed out**, because the board
will not take them.

A scan against every track, via, pad and courtyard on both faces, plus the real
`Edge.Cuts` loop and the four prop rings, found **exactly one site** on the
whole plate clearing all three constraints. The arms have none: a 12 mm arm
with a 2.5 mm component-to-edge rule leaves ±1.45 mm of usable width, and the
flyback diode is already in it.

So the swap is a **routing** job, not a placement job. Recorded as such in
`design.py` rather than left as an intention.

Two process errors of my own are worth recording, because they are opposites:

- the first placement scan checked **courtyards and edges but not copper** — 29
  violations, parts sitting on live tracks;
- the second checked **copper but not courtyards** — 5 violations, parts inside
  U1, J1 and J5.

On a board this dense, "no part is there" and "no copper is there" are entirely
different questions, and a site has to answer both.

### A gate earning its keep

Moving C10-C13 to machine assembly made preflight refuse the BOM twice, both
times correctly:

1. `LCSC code 'TBD' is not a C-number` — they had no part number, because
   nothing hand-fitted needs one.
2. `LCSC C14663 assigned to differing values ['100nF', '100nF/16V X7R']` — the
   value field had carried a *requirement* ("16V") where the assembler reads an
   *instruction*. Harmless while hand-fitted; wrong the moment a machine reads
   it. Normalised to `100nF`, the same string as C3 and C8, which is the same
   physical part.

### Result

| | rev B | rev C now |
| --- | --- | --- |
| Copper layers | 2 | **4** |
| Machine-assembled parts | 29 | **33** |
| Bottom-side SMD | 5 | **1** (C1 only) |
| DRC `--severity-all` | 0 / 0 | **0 / 0** |
| ERC | 0 | **0** |
| Order gate | CLEAR | **CLEAR** |

Still open, in order: C1's bulk-bank swap (needs routing), J2's Pololu breakout
→ bare VL53L1X (removes the last belly part and 7 THT joints), the keyed J4,
the J1/J5 merge with the INA226 shunt, and the flow connector.

---

## 18. A correction, and a swap that was a trap

### The correction

Sections 16 and 17 justify moving parts off the belly by saying that JLCPCB's
Economic PCBA is single-sided at any price, so one bottom-side part costs ~$58
in tier fees. **That reasoning does not apply to this board, and I was wrong to
state it.**

`gen_jlcpcb.py:95-97` skips through-hole parts and bottom-side parts as
hand-soldered, so they never enter the JLCPCB job at all. Verified against the
shipped CPL: **all 33 assembled parts are `Top`, and always were.** C1 and J2
are hand-fitted, so they have never affected the assembly tier.

The belly-to-top move of C10-C13 was still worth doing — it converted four
**hand-soldered** joints into machine-placed ones, and hand joints are the only
thing that has ever broken this project. But it saved joints, not dollars, and
the write-up should have said so.

### The trap

Replacing the Pololu VL53L1X carrier with the bare `VL53L1CXV0FY/1` was the next
planned change. It is **not being done**, and the reason inverts its own premise:

**The part is flagged "Standard Only" at JLCPCB.** One Standard-Only part removes
the *entire order* from Economic PCBA regardless of which side it sits on. And
the VL53L1X's optical windows are on the **top of the package, opposite the LGA
pads** — so a downward-looking rangefinder must be belly-mounted, making it the
only bottom-side machine part and forcing Standard **double-sided**.

| | setup + stencil |
| --- | --- |
| Economic single-side — where rev C sits today | $9.71 |
| Standard double-side — the only useful orientation | $67.54 |

**+$57.83** to save roughly $50 of Pololu carriers across five boards, and to
acquire: MSL 3 handling whose standard 125 °C rebake exceeds ST's own 85 °C
storage maximum; a 245 °C absolute-maximum reflow against house profiles that
commonly peak 245–250 °C; a no-wash, no-ultrasonic, convection-only process; and
a **protective liner over the optics that survives 260 °C reflow and must be
peeled afterwards** — with nobody able to confirm whether JLCPCB does it. If they
do not, five boards return with a blind rangefinder and no error anywhere.

LGA-12 also has no fallback: twelve 0.5 mm pads entirely under the body, no
castellations, so it cannot be hand-soldered if the machine declines it.

Seven hand joints is a real cost. It is not worth a blind-sensor risk, a tier
change and a new class of process constraint on a project whose entire problem
is boards that do not work.

The datasheet research is kept in `docs/VL53L1X_ONBOARD.md` — full
pin-by-pin map from ST DocID031281 Rev 3, the optical exclusion-cone geometry
(which constrains where the battery may sit *whatever* sensor is fitted), and
the calibration rules. Nobody should have to research it twice.

### What survives from it anyway

Two findings apply to the board as it stands, with the Pololu carrier:

- **The battery must not sit under the rangefinder, and its strap must not cross
  the optical cones.** Velcro, tape and heatshrink are all IR-reflective at
  940 nm and present as a permanent ~5 mm target — below the 4 cm minimum range.
  The sensor would return a constant instead of a height, silently.
- **Nothing may bear on the sensor cap** — 25 N maximum. A tensioned strap over
  it cracks the optics.

Neither is currently checked by anything.

---

## 19. Pre-order audit — everything checked against the artefacts, not the docs

Run before committing money. Five independent audits, none of which trusts the
generator that produced the thing being checked.

**1. Gate tables vs the live design.** Every component placed and every
placement a component (48/48). `REFS` covers all 48 and every shape it names
exists. No table — `REFS`, `ALLOWED`, `THT`, `ON_ARM`, `HAND_FITTED`,
`BACK_SIDE_ALLOWED` — names a part that no longer exists. The back-side
allow-list exactly equals what is actually on the back (`C1`, `J2`). 22 nets,
none with fewer than two pads, nothing PENDING. **PASS**

**2. The shipped drill file vs the board.** Counted independently out of
`pursuit.drl` and out of the board: **111 holes both ways**, seven tools, every
diameter matching (0.40 ×63, 0.75 ×2, 0.90 ×3, 1.00 ×29, 1.20 ×2, 1.50 ×2,
3.20 ×10). The zip carries 15 files. **PASS**

**3. Do the inner layers actually contain copper?** The specific disaster
§16 guards against is a four-layer stackup with blank inners. Counted apertures
and draw commands in the gerbers themselves: `In1_Cu` 141 kB / 5261 draws,
`In2_Cu` 156 kB / 5827 draws. Not blank. **PASS**

**4. Every historical defect, re-verified against the board.** This project's
recurring failure is a doc certifying a fix that was never on the copper, so all
21 were re-read out of the `.kicad_pcb`: thermal spoke 0.3 mm; MinResolvedSpokes
1; divider 10k; LED1 not white; C6/C7 dnf; R5–R8 1k; four copper layers; both
inner planes single polygons; all four motor keepouts spanning all four layers;
the IMU keepout still deliberately F.Cu-only; TP1/TP2/TP3 on VBAT/+3V3/GND; the
XIAO hole rows exactly 15.24 mm apart (the drill-offset bug); and the `+ C1`,
`J1+` and `REV C` silk markers present. **21/21 PASS**

**5. Every part number, for existence and stock.** 12 of 13 lines have between
265 k and 12.6 M in stock. One flag:

> **C25804 (the 10k divider) reports ZERO stock.**

Almost certainly a stale snapshot — it is UNI-ROYAL's standard 10k 0603 1% and
one of JLCPCB's most-used Basic parts — but that is not something to discover
after paying. **Check it live at order time.** A verified identical-spec drop-in
is recorded in `gen_jlcpcb.py`: **C2906982** (FRC0603F1002TS), same 100 mW /
10 kΩ / 75 V / thick film / ±1% / ±100 ppm/°C / 0603, 10.3 M in stock, Extended
so it costs one $3 setup fee.

And a trap worth naming: **C23192 is 510 kΩ, not 10 kΩ**, despite sitting one
digit away in the same UNI-ROYAL series. Substituting it would put 4.2 V
straight onto the ADC pin. It is called out in both the BOM and the LCSC map.

### Gate run, unpiped, exit codes honoured

| gate | exit |
| --- | --- |
| `design.py` netlist audit | 0 |
| `check_place.py` | 0 |
| `check_sch.py` netlist parity | 0 |
| `audit_islands.py` | 0 |
| `check_ground.py --net GND` / `--net VBAT` | 0 / 0 |
| `kicad-cli sch erc` | 0 |
| `kicad-cli pcb drc --severity-all` | 0 |
| `preflight.py` | 0 |
| firmware host tests (125) | 0 |

Ten for ten, none of them piped.

## 20. 2026-09-21: the motor pads leave the solid list, and a rules file that was never part of the build

Done on the day of ordering, because the question asked was "make sure these are working boards" and the
answer had to come from the copper, not from section 14's prose.

### What was measured

A ring was walked around every hand-soldered through-hole pad, half-way across the thermal gap, on every
copper layer that carries a pour for that pad's net, and the fraction of the ring that is copper recorded.
About 100% is a solid tie; 5 to 35% is spokes.

| pads | rev A (built, five cold joints) | rev C before today | now |
| --- | --- | --- | --- |
| U1.9 GND (XIAO header) | 80%, one layer | 5% B.Cu, 19% In1 | 19%, 19% |
| J2.3, TP1, TP3 | 12% | 10 to 16% | unchanged |
| **MT1-MT4 pad 1 (motor +)** | **89%, one layer** | **95% F.Cu AND 100% In2** | **9% and 9%** |
| J1, J5 (pack terminals), J4 | solid | solid, both layers | unchanged, deliberately |

So section 14's fix is real for the header and the socket, and it never reached the two motor pins that
made up the rest of the five. Rev B widened their annulus and left the tie; rev C's In2 plane then tied the
same four pads solidly into a second plane. The stated reason, that the motor terminals "carry the pack",
does not hold: one terminal carries one motor (2.75 A at stall) and each already ends a dedicated 1.2 mm
B.Cu VBAT spine, which is why `check_ground.py` still reports 18 VBAT pads on the plane and 0 off it.

### The change

`gen_pcb.py`: `("MT1", "MT2", "MT3", "MT4", "J1", "J4", "J5")` became `("J1", "J4", "J5")`. Nothing else.

### Proof that nothing else moved

The rebuilt board was compared structurally with the previous verified one (md5 `79a0a6d0d508...`):
58 / 58 footprints with identical positions, pads, nets and graphics, the only difference being the
zone-connection flag on MT1-MT4; 270 / 270 tracks and vias identical; 10 / 10 zone definitions identical;
330 / 330 board graphics identical; GND copper on B.Cu and In1 identical with zero changed patches; VBAT
copper with nothing added and exactly one pad-plus-gap knockout removed per motor pad (6.25 mm2 on F.Cu,
6.43 mm2 on In2, equal on all four arms). BOM, CPL and the orientation sheet are byte-identical.
New board md5 `8df9e419b62a9bf041c74feef6a6e31a`, preflight hash `063ea4f53800d7c1`.

### The rules file

The first rebuild did NOT reproduce the old copper: the pours grew by about 10% on every layer. Cause: an
untracked symlink, `hardware/pursuit.kicad_dru -> ~/hw/drc/JLCPCB_4L.kicad_dru`, created 2026-09-13, three
weeks after the board was last built and after every review in sections 14 to 19. KiCad loads a
`<project>.kicad_dru` automatically, and two of that file's rules are unconditional, so on refill they
override the board's own settings. Measured by refilling scratch copies under one rule at a time
(baseline B.Cu 2404.0 / In1 2839.1 / F.Cu 2334.2 / In2 2826.3 mm2):

| rules file contains | B.Cu | In1 | F.Cu | In2 |
| --- | --- | --- | --- | --- |
| only `(constraint clearance (min 0.127mm))` | +197.5 | +60.4 | +155.0 | +61.8 |
| only `(constraint edge_clearance (min 0.3mm))` | +44.9 | +45.0 | +48.9 | +45.2 |
| the whole file | +243.7 | +106.6 | +200.5 | +107.3 |

The clearance rule beats the pours' 0.3 mm local clearance and lets them close to 0.127 mm from tracks,
pads and via barrels; the edge rule beats `gen_pcb.py`'s 0.4 mm copper-to-edge and moves every pour 0.1 mm
nearer the outline, on all four layers of an airframe whose edge is 600 mm long. The same file's "Minimum solder mask sliver" rule is written
as a bare `silk_clearance`, fires on silk over any copper, and fills the report to KiCad's cap of 199 per
type, which is the 199 this Makefile already warns about. The old board had never been refilled since the
link appeared, so it kept 0.3 mm; nobody had run `make board-gates` since, or step 7 would have failed.

The link was moved aside (`hardware/pursuit.kicad_dru.linked_20260913_moved_aside`) and the board built
the way it was on 2026-08-22. The finished board was then checked, without refilling, against that
4-layer rules file minus the silk rule: 0 violations, 0 unconnected. Do not restore the link until the
shared file scopes its clearance rule away from zones, or the next refill silently re-tightens every pour.

### Gate run, unpiped, exit codes honoured

`make board` exit 0: footprints, `design.py`, `check_place.py` (0 violations, the 9 known edge warnings),
`gen_sch.py`, `gen_pcb.py`, ERC 0, DRC `--severity-all` 0 violations / 0 unconnected, `check_sch.py`,
`audit_islands.py` (every pour one polygon), `check_ground.py` GND and VBAT, `release.py`,
`gen_jlcpcb.py`, `preflight.py` 0 failures / 0 warnings.

What this does not show: that a joint is good. That is still an iron, flux and a meter, and BRINGUP's
"reflow every hand joint" stands. J1, J5 and J4 remain solid by design and want the most heat.

## 21. 2026-09-21, on the order page: all four motor FETs were 180 degrees round

Section 12 left the SOT-23 rotation "unresolved, read it off the preview". Read off the preview, it was wrong.

JLCPCB does not rotate KiCad's footprint; it rotates ITS OWN footprint for the LCSC code, and that footprint's
zero need not be KiCad's. Fitting KiCad's pads against JLCPCB's footprint for each polarised part
(`easyeda.com/api/products/<LCSC>/components`), by pin FUNCTION:

| part | LCSC | JLCPCB footprint fits KiCad's at | runner-up | CPL before | verdict |
| --- | --- | --- | --- | --- | --- |
| Q1-Q4 AO3400A (1=G 2=S 3=D in both) | C20917 | **180 deg**, 0.06 mm | 270 deg, 1.95 mm | raw KiCad angle | **180 deg out** |
| D1-D4 SS34 (1=K in both) | C8678 | 0 deg, 0.20 mm | 90 deg, 2.97 mm | raw | correct |
| LED2 red (1 = cathode in both) | C84256 | 0 deg, 0.16 mm | 90 deg, 1.45 mm | raw | correct |
| LED1 yellow | C84261 | 180 deg BY PAD NUMBER | | raw | correct: JLCPCB numbers the ANODE 1, KiCad the cathode, so by function it is 0 |
| U2 ICM-42688-P | C1850418 | 0 deg, 0.10 mm | 90 deg, 2.05 mm | raw | correct |

JLCPCB's preview agreed on every row: cathodes inboard, U2 pin 1 front-left, LED anodes outboard, and on Q1-Q4
the pin-1 dot OUTBOARD with the three leads straddling the gaps between pads, where this board puts the single
drain lead outboard. It is the same JLCPCB footprint (uuid `bade9066...`) that makes the RL car's Q1 carry +180;
that project derived its corrections on 2026-09-17, this one never did.

Fix: `gen_jlcpcb.JLC_ROTATION = {"C20917": 180.0}`, added to KiCad's angle when the CPL is written. `preflight.py`
states the same fact a second time, by designator family (`CPL_FRAME_OFFSET = {"Q": 180.0}`), so dropping the
correction from the generator fails the gate. Negative-tested with the old CPL: 4 failures, DO NOT ORDER. Only
four CPL rows changed (Q1 135 -> 315, Q2 45 -> 225, Q3 225 -> 45, Q4 315 -> 135); BOM, board and gerbers untouched
(board md5 `8df9e419...`, preflight hash `063ea4f53800d7c1`). After re-upload the preview shows all three leads on
pads and the drain outboard on all four.

What this does not replace: the DFM placement render JLCPCB produces after payment. Both orders were saved with
Confirm Parts Placement = Yes and "Do not confirm automatically" ticked, so nothing is assembled until that
render has been walked against `ORIENTATION_CHECK.csv` and approved by hand.
