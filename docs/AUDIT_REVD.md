# PURSUIT-1 — full defect audit and the path to "best person-follower"

Audited 2026-08-30 across 8 domains: power, motor drive, sensors/I2C,
connectors/assembly, layout/RF, firmware safety, link, vision intelligence.
Every finding below was verified against the cited file — claims that did not
survive a source read were dropped (several were).

Severity is about *what it costs when it bites*, not likelihood.

---

## CRITICAL — fix before the board ever arms

### C1. Zero reverse-polarity protection anywhere in the VBAT path
`scripts/design.py:80-88` acknowledges the hazard ("stays real"), defers the
keyed J4 to a future rev, and the netlist contains **no P-FET, no fuse, no
series element** — `design.py:193` admits C1 "sits across an unfused LiPo, so
reversed it vents." A reversal at J1/J5 back-drives all four AO3400A body
diodes at amps each, vents C1 across an unfused pack, and kills the XIAO
through J4 — the exact accident that already destroyed one module on
2026-08-20.
**Now:** put an inline fuse holder in the battery lead (no copper change) and
a REVERSAL WARNING step in bringup group 0. **Rev D:** one SOT-23 P-FET at the
battery entry (~8 mm²).

### C2. The 10 A path depended on a joint the docs called "optional" — FIXED THIS AUDIT
`design.py:185`: the JST-PH "is the sag point at 11A burst; a soldered 20AWG
pigtail [J5] bypasses it." Yet `BOM.csv` said "J5: optional" and the bringup
solder groups never install it. A builder following the docs literally puts
9.82 A through contacts rated 2 A, adding ~0.2-0.4 V of sag on a rail whose
regulator drops out at ~3.4 V — brownout at full throttle, connector heating
against the pack. **BOM.csv now says MANDATORY with the preheat note; flight
on J1 alone is prohibited.**

### C3. ~~Fixed hover duty + no voltage compensation~~ — **FIXED 2026-08-30, host-tested**
`alt_hold.cpp:90` hard-codes `hover_(0.69f)`; the learner (`hover_learn_hz =
0.05`, ~20 s constant) cannot track battery sag during a seconds-long
failsafe descent. Physics: equivalent hover duty is ~0.61 on a full pack and
~0.71+ at 3.6 V sag. A failsafe entered late in the pack therefore descends
*faster than commanded or falls*. Feed `battery.cpp` voltage into a
`hover_eff = hover_ * (V_ref / V_now)` correction — firmware only, and it
also fixes ordinary alt-hold drift across the discharge. (The constant itself
is still UNMEASURED after being wrong three times — bench 'T' scale test
remains gate #1.)

### C4. Blocking I2C inside the 1 kHz loop — **WDT HALF IS FIXED 2026-08-30**
`icm42688.cpp:84-86` does a synchronous Wire burst: 14 bytes ≈ 380 µs = 38 %
of the loop budget; with ToF + baro landing in the same cycle, worst case
~790 µs (79 %). Wire's bus-hang timeout then stalls the loop **with motors at
their last duty**, and no `esp_task_wdt` exists anywhere in firmware to catch
it. **Done:** `esp_task_wdt` armed last in setup() (1 s, panic reset — a reset
drops motors to Hi-Z, which beats last-duty into the floor), fed at the top of
loop(). Target build clean. **Still open:** stagger ToF/baro off the IMU
cycle; pace reads from the wired-but-unused IMU INT on D9.

---

## MAJOR

- **M1. Socketed XIAO has no retention** — 14 friction contacts, ~10 g module,
  tethered by live BAT wires in a crash; no retention step in bringup.
  Zip-tie/band over the module through the existing strap holes. (verified
  from BOM.csv:20, gen_pcb.py:153-170)

  **2026-09-11 — the proposed fix cannot work. M1 stays OPEN.** Measured from
  `hardware/pursuit.kicad_pcb`: both strap holes are *aft* of the module — H1
  at (-16.30, 6.00) and H2 at (14.60, 2.50), mm from board centre, +y
  rearward. A band between them is a straight line, and that line crosses the
  centreline at **y = +4.15**, which is 1.65 mm *behind* the module's rear
  edge at **y = +2.5**. It never passes over the module at all. A band under
  tension takes the straight path between its anchors, so there is no routing
  that catches the module: pull it tight and it pulls itself off the back
  edge. The failure is silent — the band looks fitted and sits taut across
  bare laminate, while the module is still held by nothing but the 14
  friction contacts this item was raised about, and the first thing that
  tests it is a crash. M1 needs retention that grips the module body
  directly; these two holes can anchor such a part, but cannot be the whole
  of it.
- **M2. Rev C doubled the copper behind the 15 hand-soldered current-carrying
  pads and ships no process guidance** — the cold-joint class that caused all
  8 rev A/B failures, now into ~2× the heatsink. Preheat + ≥60 W + dwell +
  tug-test per joint added to the J5 BOM note this audit; the same belongs in
  bringup group 0 for MT1-4/J1/J4/TP1.
- **M3. No strain relief on J4 leads, J5 pigtail, or motor wires** — J5 is the
  pad that lost an annular ring on rev A; nothing anchors it against a crash
  yank. Service loops + the existing strap holes; zero copper.
- **M4. ~~Identity verified once, then held by body appearance~~ — FIXED
  2026-08-30.** The controller now runs a re-verify schedule: after
  `reverify_after_s` (12 s) of body-only confirmations it closes the standoff
  to 2.0 m (inside face range), a fresh face confirmation resets the clock,
  and if no face appears within `reverify_give_up_s` (8 s) it backs off and
  re-arms rather than pressing closer at someone who will not show their
  face. dt-integrated (deterministic), metric-standoff builds only, config
  gated. 5 new tests; 227/227 vision tests pass.
- **M5. RETRACTED — the audit was wrong.** A bounded, direction-aware
  search sweep already exists and is tested: `control.py:196-224` sweeps at
  `search_yaw_rate` toward the side the subject was last heading
  (`_search_dir` set at control.py:333 from the final yaw error), only after
  a real lock, only while genuinely LOST, only for `search_timeout_s`.
  Tests: `test_search_sweeps_toward_where_they_went` and three siblings. The
  original finding came from a truncated grep that missed line 333.

## MINOR (recorded, not blocking)

- One 0.3 mm bottom trace transits the antenna keepout's x-band
  (VERIFICATION.md:174 — documented, still present).
- The u.FL antenna's *mounting location on the airframe* is documented
  nowhere; where the FPC sits among four motors and a VBAT plane decides link
  range.
- `gen_jlcpcb.py` docstring claimed C10-C13 were excluded while shipping them
  in BOM/CPL — **fixed this audit**.
- Benchtest's 0.85 thrust unlock ('T') is serial-gated but compiled in; a
  USB-connected armed bench session can reach it accidentally.

## Cleared by this audit (claims that did NOT survive)

- AO3400A gate drive at 3.3 V: conduction is fine — 45-77 mW hover, 330-570 mW
  transient stall worst-case. Not a defect.
- The UDP protocol is **good**: sequence numbers, replay/reorder rejection
  (flight_state.cpp:142), staleness counters, RTT-measured lead compensation
  in vision. Better than most hobby stacks.
- Vision control already has metric standoff, latency lead, a deadman
  arbiter, asymmetric lock hysteresis. The intelligence base is strong.

---

## Making it the best person-follower — in cost order

**Tier 0 — free (laptop/firmware only, no board change):**
1. ~~Continuous identity re-verification~~ **DONE 2026-08-30** (see M4).
2. ~~Reacquisition behavior~~ **ALREADY EXISTED** (see M5 retraction).
3. ~~Voltage-compensated hover + WDT~~ **DONE 2026-08-30**: hover_ is now the
   duty at V_REF with V_REF/V scaling at use, learner normalises back to
   reference, LANDING cap and failsafe fallback use the compensated value.
   3 new host tests; 128/128 native pass; xiao build clean. IMU-INT pacing
   still open.
4. **Predictive standoff**: feed the KF's velocity into the distance error so
   the drone leads a walking person instead of trailing the error.

**Tier 1 — assembly-time (this board, ~£2):** inline fuse, XIAO retention
band, strain-relief loops, the solder process discipline.

**Tier 2 — rev D copper (the board is saturated; this is the honest list):**
P-FET reverse protection (~8 mm²), keyed JST at J4, INA226 current shunt
(the J1/J5 merge already sketched), optical-flow connector for indoor
position hold. Requires the full re-placement pass VERIFICATION.md §14
already concluded is needed — do it once, for all four.

**Order nothing and change nothing else until bringup items 1-6 pass — the
best drone here is still the one that has measured its own hover throttle.**

---
*2026-08-30 fixes landed:* BOM J5 wording, gen_jlcpcb docstring, alt_hold
voltage compensation (+3 tests), hardware watchdog. Vision items M4/M5 remain
open — note for whoever takes them: `lock.py` already has `_stash_memento` /
`_try_reacquire` and a MISSING state, so M5 is a *behavior* (yaw-sweep command
while MISSING), not new tracker machinery; M4 needs face-age in `LockResult`
plus a standoff schedule in `control.py`.

- 2026-09-03: C1 reverse-polarity mitigation made a real artifact: BRINGUP §6 REVERSAL GATE (build+meter pigtail first, meter and mark every pack, sign-off line) + BOM row Pigtail-J5. Fuse explicitly documented as NOT a silicon defence (body-diode path ~20A, ms vs s). P-FET stays rev D.
