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
| MAJOR | Five to ten placed parts violate JLCPCB's 2.5mm component-to-board-edge PCBA guideline (measured: LED1/LED2 ~0.5mm, D1-D4 ~0.7mm, C8/R11/R12/R14 ~1.2mm). | **Accepted, not fixed.** This is inherent to a 12mm-arm X-frame; meeting 2.5mm means redesigning the airframe. Documented in README as an expected DFM query with the bare-board fallback stated. |
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
