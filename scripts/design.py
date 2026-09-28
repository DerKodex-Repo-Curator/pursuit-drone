"""PURSUIT-1 canonical circuit definition.

Single source of truth: the schematic generator, PCB generator, and the
netlist audit all read THIS file. A connection that isn't here doesn't exist.

Pin-number facts marked RESEARCH are locked from datasheet research before
generation; generators refuse to run while any PENDING marker remains.
"""
import sys

PENDING = "PENDING-RESEARCH"

# ---------------------------------------------------------------------------
# Nets
# ---------------------------------------------------------------------------
NETS = [
    "VBAT", "GND", "+3V3",
    "SDA", "SCL", "IMU_INT", "VBAT_SENSE",
    "M1_PWM", "M2_PWM", "M3_PWM", "M4_PWM",
    "M1_GATE", "M2_GATE", "M3_GATE", "M4_GATE",
    "M1_SW", "M2_SW", "M3_SW", "M4_SW",
    "LED_CTRL", "LED1_A", "LED2_A",
]

# ---------------------------------------------------------------------------
# Components: ref -> dict(value, footprint, lib, pins={pad_number: net})
# footprint 'lib' of None means a project-local .kicad_mod in pursuit.pretty
# ---------------------------------------------------------------------------
COMPONENTS = {
    # -- Flight controller module ------------------------------------------
    # Seeed XIAO ESP32S3 Sense on two 1x7 THT header rows.
    # Pad numbering = XIAO silk positions, USB connector at top:
    #   left row top->bottom  1..7  = D0 D1 D2 D3 D4 D5 D6
    #   right row top->bottom 8..14 = 5V GND 3V3 D10 D9 D8 D7
    "U1": dict(
        value="XIAO-ESP32S3-Sense",
        lib=None, footprint="XIAO_ESP32S3_Carrier",
        pins={
            # research-locked: camera/mic use NO header pins; D2=GPIO3 is a
            # strapping pin so it carries only the high-impedance divider;
            # D6/D7 stay as UART (ROM boot log toggles D6 at reset)
            "1": "M1_PWM",       # D0 / GPIO1
            "2": "M2_PWM",       # D1 / GPIO2
            "3": "VBAT_SENSE",   # D2 / GPIO3  ADC1_CH2 (strap pin: high-Z ok)
            "4": "M3_PWM",       # D3 / GPIO4
            "5": "SDA",          # D4 / GPIO5  (XIAO default SDA)
            "6": "SCL",          # D5 / GPIO6  (XIAO default SCL)
            "7": None,           # D6 / GPIO43 U0TXD — debug via USB-C or
                                 # tack wires to the module castellations
            "8": None,           # 5V = USB VBUS passthrough — unconnected
            "9": "GND",          # GND
            "10": "+3V3",        # 3V3 (600 mA buck budget; ~420 mA streaming)
            "11": "LED_CTRL",    # D10 / GPIO9
            "12": "IMU_INT",     # D9  / GPIO8
            "13": "M4_PWM",      # D8  / GPIO7
            "14": None,          # D7  / GPIO44 U0RXD — spare
        },
    ),
    # Solder-jumper pads to the XIAO's underside battery pads (2 short wires).
    #
    # ASKED AND ANSWERED 2026-08-22: could these four hand joints be deleted by
    # feeding board VBAT into U1 pin 8 instead -- the module's 5V pin, which is
    # already on the header and unconnected (see U1 "8" above)? NO.
    #
    # Read out of Seeed's own block diagram (XIAO ESP32S3 v1.4 schematic, sheet
    # 3/4), not from a forum post:
    #
    #     USB -> VBUS/5V --+-> BAT charger -> BAT 3.7V -> [MOSFET] --+
    #                      |                                         +-> DCDC 3V3
    #                      +------------------[DIODE]----------------+
    #
    # VBUS reaches the 3V3 regulator through a DIODE. BAT reaches it through a
    # MOSFET. J4 feeds BAT, so it pays ~0.05V; the 5V pin would pay ~0.35V.
    #
    # This rail already drops out around 3.4V AT THE REGULATOR, which is a pack
    # at ~3.45V today. Through the 5V pin that becomes ~3.75V -- roughly 0.3V of
    # usable pack surrendered, on the one rail whose sag already forces a
    # throttle limiter in firmware. Four solder joints are not worth that.
    #
    # The reversal hazard these two wires carry is real and stays real. The fix
    # for it is a KEYED connector at J4, not deletion (see also the J4 entry in
    # docs/VERIFICATION.md section 14 -- deleting J4 outright leaves the module
    # with no power path at all, because there is no regulator on this board).
    "J4": dict(
        value="XIAO-BAT-wires",
        lib=None, footprint="Wire_Pads_2x",
        pins={"1": "VBAT", "2": "GND"},
    ),

    # -- IMU ----------------------------------------------------------------
    # ICM-42688-P LGA-14, I2C mode, AD0 low -> addr 0x68.
    # Pin map locked from TDK DS-000347 v1.2 (research, confirmed-datasheet).
    "U2": dict(
        value="ICM-42688-P",
        lib=None, footprint="ICM42688P_LGA14",
        pins={
            "1": "GND",       # AP_SDO/AP_AD0 low -> addr 0x68 (never float)
            "2": None,        # RESV (solder, leave unconnected)
            "3": None,        # RESV
            "4": "IMU_INT",   # INT1
            "5": "+3V3",      # VDDIO
            "6": "GND",       # GND
            "7": "GND",       # RESV — MUST connect to GND
            "8": "+3V3",      # VDD
            "9": "GND",       # INT2/FSYNC — GND when FSYNC unused
            "10": None,       # RESV
            "11": None,       # RESV
            "12": "+3V3",     # AP_CS — MUST tie to VDDIO for I2C mode
            "13": "SCL",      # AP_SCL
            "14": "SDA",      # AP_SDA
        },
    ),

    # -- Motor drive stages (x4) -------------------------------------------
    # AO3400A SOT-23 low-side; SOT-23 pinout locked from datasheet research.
    **{
        f"Q{i}": dict(
            value="AO3400A",
            lib="Package_TO_SOT_SMD", footprint="SOT-23",
            pins={"1": f"M{i}_GATE", "2": "GND", "3": f"M{i}_SW"},
        ) for i in (1, 2, 3, 4)
    },
    **{  # gate series resistors
        f"R{i}": dict(
            value="100R", lib="Resistor_SMD", footprint="R_0603_1608Metric",
            pins={"1": f"M{i}_PWM", "2": f"M{i}_GATE"},
        ) for i in (1, 2, 3, 4)
    },
    # Gate pulldowns — motors stay off during ESP32 boot. 1k, NOT 10k.
    #
    # CORRECTION (round 6). An earlier revision justified 1k by claiming the
    # XIAO Sense expansion board pulls D8 up through 4.7k. That is FALSE and
    # came from a forum post, not a schematic. Read from Seeed's own
    # XIAO_ESP32S3_ExpBoard_v1.0_SCH.pdf (sha256 8224252d...040c0c) rendered
    # at 900dpi to resolve the junction dots, the three 4.7k pull-ups off JP3
    # land as: R4 -> D9/MISO, R5 -> D10/MOSI, R6 -> SD_CS (thence IO21 via
    # R12=0R; R11 to D2 is DNP). **D8/SCK carries only R2 = 22R in series to
    # the socket CLK pin and one ESD clamp — no pull-up.** So M4_PWM is
    # genuinely floating at reset, exactly like M1-M3, and all four gates sit
    # at ~0V with any sane pulldown.
    #
    # 1k is kept anyway, for a reason that survives checking: it bounds the
    # gate against an UNEXPECTED pull-up. The ESP32-S3's own internal pull-up
    # is ~45k, and if the ROM or a future module revision ever enabled one on
    # a PWM pin, 10k would put the gate at 3.3*10k/(45k+0.1k+10k) = 0.60V —
    # within 50mV of the AO3400A's 0.65V minimum threshold. At 1k the same
    # fault gives 72mV. It also discharges the gate ~10x faster when the pin
    # goes high-Z. The cost is gate drive 3.00V instead of 3.28V, which is
    # still above the 2.5V at which AO3400A Rds(on) is specified, and 3.0mA
    # of GPIO loading per channel.
    **{
        f"R{i+4}": dict(
            value="1k", lib="Resistor_SMD", footprint="R_0603_1608Metric",
            pins={"1": f"M{i}_GATE", "2": "GND"},
        ) for i in (1, 2, 3, 4)
    },
    **{  # flyback Schottky across each motor: anode = switched node, cathode = VBAT
        f"D{i}": dict(
            value="SS34", lib="Diode_SMD", footprint="D_SMA",
            pins={"1": "VBAT", "2": f"M{i}_SW"},  # 1=K, 2=A (KiCad D_SMA convention)
        ) for i in (1, 2, 3, 4)
    },
    **{  # motor wire pads: pad 1 = motor+, pad 2 = motor-
        f"MT{i}": dict(
            value=f"8520-M{i}", lib=None, footprint="Motor_Pads_2x",
            pins={"1": "VBAT", "2": f"M{i}_SW"},
        ) for i in (1, 2, 3, 4)
    },

    # -- Power --------------------------------------------------------------
    "J1": dict(  # 1S LiPo in. Right-angle: crash forces load the shroud in
        # shear, not peel (research). JST-PH polarity is NOT standardized —
        # board carries CHECK POLARITY silk; verify pack with a meter first.
        #
        # REV C ATTEMPTED AND BACKED OUT: the surface-mount S2B-PH-SM4-TB, to
        # turn two hand joints into zero. It does not fit. Its pads span
        # 8.2 x 9.2mm against the through-hole part's 6.9 x 8.6mm, and its
        # origin is the BODY CENTRE where the THT part's is PIN 1 -- so even
        # re-anchored correctly it lands on U2 (the IMU) and hangs over the
        # rear plate edge. There is nowhere else on the plate for it.
        value="JST-PH-2 BATT",
        lib="Connector_JST", footprint="JST_PH_S2B-PH-K_1x02_P2.00mm_Horizontal",
        pins={"1": "VBAT", "2": "GND"},
    ),
    "J5": dict(  # direct-solder pigtail pads paralleling J1 (PH is the sag
        # point at 11A burst; a soldered 20AWG pigtail bypasses it)
        value="BATT-pigtail",
        lib=None, footprint="Wire_Pads_2x_HD",
        pins={"1": "VBAT", "2": "GND"},
    ),
    # REV C ATTEMPTED AND BACKED OUT: replacing this with six 100uF 1206
    # ceramics. The reasons were good -- C1 is polarity-critical, hand-fitted,
    # and sits across an unfused LiPo, so reversed it vents -- but the board
    # will not take them. A scan against every track, via, pad, courtyard and
    # the real board outline found exactly ONE site on the whole plate that
    # clears all three constraints, and the arms have none: a 12mm arm with a
    # 2.5mm component-to-edge rule leaves +-1.45mm of usable width and the
    # flyback diode is already in it.
    #
    # So the swap is a ROUTING job, not a placement job, and it is queued as
    # one. Until then C1 stays: belly, polarised, hand-fitted, silk-marked.
    "C1": dict(
        value="220uF/6.3V polymer ESR<=30m",
        lib="Capacitor_Tantalum_SMD", footprint="CP_EIA-7343-31_Kemet-D",
        pins={"1": "VBAT", "2": "GND"},
    ),
    # VBAT ceramic bypass, one per FET. The low-side switch chops the pack
    # current 0 -> 2.75A in tens of ns; C1 is 30-45mm away from three of
    # the four FETs and cannot supply those edges. Without these the rail
    # rings and the whole airframe radiates at 32kHz harmonics.
    #
    # Value string is "100nF", matching C3 and C8, because it IS the same part
    # -- C14663, a 100nF 0603 X7R 50V. It used to read "100nF/16V X7R", which
    # was a REQUIREMENT written into a field that the assembler reads as an
    # INSTRUCTION. Harmless while these were hand-fitted; the moment rev C
    # moved them to machine assembly, preflight refused the BOM for assigning
    # one LCSC code two different values, which is exactly right. 50V on a
    # 4.2V rail is margin, not a mismatch.
    **{
        f"C{i+9}": dict(
            value="100nF", lib="Capacitor_SMD",
            footprint="C_0603_1608Metric",
            pins={"1": "VBAT", "2": "GND"},
        ) for i in (1, 2, 3, 4)
    },
    # Rev B: 10k/10k, was 100k/100k.
    #
    # Two 100k resistors present a 50kOhm source to the ADC. Espressif want
    # well under 10k for the ESP32-S3's sample-and-hold to settle, and at 50k
    # it does not: measured 2026-08-20, the raw channel returned 1.70V at 0%
    # motor duty and 0.00V mid-run while a multimeter read the pack at 3.64V.
    # Worse, the glitch RATE scaled with PWM activity, so the filtered reading
    # fell smoothly and linearly with throttle -- an almost perfect imitation
    # of ohmic pack sag, r^2 essentially 1. Two days were spent diagnosing a
    # high-impedance battery that did not exist.
    #
    # 10k/10k gives a 5kOhm source, inside spec with margin. The cost is idle
    # draw: 4.2V/20k = 210uA against 21uA before. Over an hour that is 0.21mAh
    # out of a 650mAh pack -- 0.03%, and the aircraft flies for four minutes.
    # Trading 190uA for a battery reading you can believe is not a close call.
    "R11": dict(value="10k", lib="Resistor_SMD", footprint="R_0603_1608Metric",
                pins={"1": "VBAT", "2": "VBAT_SENSE"}),
    "R12": dict(value="10k", lib="Resistor_SMD", footprint="R_0603_1608Metric",
                pins={"1": "VBAT_SENSE", "2": "GND"}),
    "C8": dict(value="100nF", lib="Capacitor_SMD", footprint="C_0603_1608Metric",
               pins={"1": "VBAT_SENSE", "2": "GND"}),

    # -- test points (rev B) -----------------------------------------------
    # One pad per power rail, reachable with a meter probe while the module is
    # seated and the aircraft is powered. Bring-up had none of these, so every
    # measurement was taken on a pad that was also a joint -- and a probe on a
    # marginal joint reads the probe, not the net. Two pads were eventually
    # lifted by repeated rework at points that only existed because there was
    # nowhere else to measure.
    "TP1": dict(value="VBAT", lib=None, footprint="TestPoint_1mm",
                pins={"1": "VBAT"}),
    "TP2": dict(value="3V3", lib=None, footprint="TestPoint_1mm",
                pins={"1": "+3V3"}),
    "TP3": dict(value="GND", lib=None, footprint="TestPoint_1mm",
                pins={"1": "GND"}),

    # -- IMU decoupling (TDK DS-000347 Table 10 BOM) ------------------------
    # NOTE the site<->value mapping. There is no +3V3 plane (F.Cu is the
    # VBAT pour), so +3V3 is a daisy chain and "which cap is on which pin"
    # is decided by the routing, not by the refdes. The C2 site routes to
    # U2 pin 5 (VDDIO) and the C3 site routes to U2 pin 8 (VDD), so the
    # values are assigned to match the datasheet intent:
    #   VDD  (pin 8): 100nF @3.9mm + 2.2uF, VDDIO (pin 5): 10nF + bulk.
    "C2": dict(value="10nF", lib="Capacitor_SMD", footprint="C_0603_1608Metric",
               pins={"1": "+3V3", "2": "GND"}),   # -> U2.5 VDDIO bypass
    "C4": dict(value="2.2uF", lib="Capacitor_SMD", footprint="C_0603_1608Metric",
               pins={"1": "+3V3", "2": "GND"}),   # -> U2.5 side bulk
    "C3": dict(value="100nF", lib="Capacitor_SMD", footprint="C_0603_1608Metric",
               pins={"1": "+3V3", "2": "GND"}),   # -> U2.8 VDD bypass
    # -- 3V3 rail bulk — PADS ONLY, DO NOT FIT -----------------------------
    #
    # These were sized for WiFi TX burst ride-through before anyone checked
    # what supplies +3V3. It is U3 on the XIAO core, an **SGM6029CYG** 4MHz
    # synchronous buck (L4 = 470nH, its own C5 + C6 = 4.7uF each, Seeed's
    # schematic annotates "Imax=600mA"). SGMicro's datasheet, Recommended
    # Operating Conditions:
    #     Effective Output Capacitance, COUT
    #     SGM6029A/SGM6029B/SGM6029C ............... 5uF to 10uF
    # That is a RANGE WITH AN UPPER BOUND, and the XIAO uses the C variant.
    # Anything hung on the 3V3 castellation lands on that same node. With
    # DC-bias derating C6 keeps ~18-20uF and C7 ~25-35uF, so fitting both put
    # ~50-64uF effective on a node specified for 5-10uF — 5x to 6x over. The
    # part's VOS pin feeds the modulator ramp from output ripple, and ripple
    # scales as 1/COUT, so over-capacitance attacks the control loop itself;
    # separately, charging ~50uF inside a few hundred us of soft-start needs
    # ~360mA on a 600mA rail before the ESP32-S3 has even booted.
    #
    # dnf=True keeps the footprints and the routing on the board but drops
    # them from the JLCPCB BOM/CPL. Capacitance can be hand-added to a bare
    # pad in five minutes; a reflowed 1206 sitting 7.3mm from a 0.5mm-pitch
    # LGA cannot be cheaply removed. C2/C3/C4 stay fitted — they are IMU-local
    # decoupling and add only ~1.9uF effective, leaving the node at ~7-9uF,
    # inside the window.
    "C6": dict(value="22uF", lib="Capacitor_SMD", footprint="C_0805_2012Metric",
               pins={"1": "+3V3", "2": "GND"}, dnf=True),
    "C7": dict(value="100uF/6.3V", lib="Capacitor_SMD", footprint="C_1206_3216Metric",
               pins={"1": "+3V3", "2": "GND"}, dnf=True),

    # -- I2C bus ------------------------------------------------------------
    "R9": dict(value="4.7k", lib="Resistor_SMD", footprint="R_0603_1608Metric",
               pins={"1": "SDA", "2": "+3V3"}),
    "R10": dict(value="4.7k", lib="Resistor_SMD", footprint="R_0603_1608Metric",
                pins={"1": "SCL", "2": "+3V3"}),

    # -- ToF altitude sensor header (belly, faces down) --------------------
    # Pololu 3415 pin order (research-confirmed): VDD VIN GND SDA SCL XSHUT
    # GPIO1. VDD is the breakout's 2.8V regulator OUTPUT — leave NC. XSHUT
    # has an onboard 47k pullup — safe to leave NC.
    # REV C ATTEMPTED AND BACKED OUT: a 4-pin JST-SH surface-mount connector
    # in place of this header. It is the single largest joint reduction on the
    # board -- fourteen, seven here and seven on the Pololu carrier -- and it
    # was placed legally (77 legal sites once the courtyard was read out of the
    # footprint instead of estimated). It failed on ROUTING, not placement:
    #
    #   * J5's pigtail pads own the west plate from x -20.45 to -16.75, and
    #     were in 34 of the 47 resulting DRC violations;
    #   * the +3V3 B.Cu trunk runs a solid wall at x=-10.2 from y-10.8 to
    #     y+5.4, so nothing crosses from U1 to the west plate on that layer;
    #   * SDA and SCL have to swap over between U1's pin row and Qwiic pin
    #     order, so they cannot both use the 5.6mm corridor between the
    #     connector's two mechanical tabs.
    #
    # All three are consequences of a layout solved for a DIFFERENT part set.
    # This needs the re-placement pass, like the bulk bank and the boost.
    "J2": dict(
        value="VL53L1X-Pololu",
        lib="Connector_PinHeader_2.54mm", footprint="PinHeader_1x07_P2.54mm_Vertical",
        pins={"1": None, "2": "+3V3", "3": "GND", "4": "SDA", "5": "SCL",
              "6": None, "7": None},
    ),

    # (No expansion power header: its bottom-layer 3V3 trunk had to wrap
    # the whole right side and blocked the motor fan-out and the VBAT arm
    # spines. The ToF header J2 already brings out 3V3 and GND.)

    # -- Orientation LEDs ---------------------------------------------------
    "R13": dict(value="150R", lib="Resistor_SMD", footprint="R_0603_1608Metric",
                pins={"1": "LED_CTRL", "2": "LED1_A"}),
    # White/blue is NOT an option here. Every white 0805 is an InGaN die with
    # Vf specified as a BIN RANGE of 2.6-3.6V; 2.6V is the bottom of that bin,
    # not the maximum. Driven from a 3.3V GPIO the top bins simply never
    # conduct, and no resistor value fixes that. Yellow (AlGaInP, 1.6-2.6V) is
    # the same NationStar 0805 family as the red rear LED, so front and rear
    # stay distinguishable and both light on any bin.
    "LED1": dict(value="LED-Yellow-Front-Vf2.6max", lib="LED_SMD", footprint="LED_0805_2012Metric",
                 pins={"1": "GND", "2": "LED1_A"}),  # 1=K, 2=A
    "R14": dict(value="150R", lib="Resistor_SMD", footprint="R_0603_1608Metric",
                pins={"1": "LED_CTRL", "2": "LED2_A"}),
    "LED2": dict(value="LED-Red-Rear", lib="LED_SMD", footprint="LED_0805_2012Metric",
                 pins={"1": "GND", "2": "LED2_A"}),
}


def audit():
    """Internal consistency: every net used exists, every net has >=2 pins."""
    from collections import Counter
    used = Counter()
    for ref, c in COMPONENTS.items():
        if c["pins"] == PENDING:
            print(f"  !! {ref}: pins {PENDING}")
            continue
        for pad, net in c["pins"].items():
            if net is None:
                continue
            assert net in NETS, f"{ref}.{pad}: unknown net {net}"
            used[net] += 1
    # A net with fewer than two pads connects nothing to nothing. This used
    # to PRINT '!!' and exit 0, so a stub net rode through every downstream
    # gate: the board builds it, DRC sees a one-pad net as trivially routed,
    # and preflight says CLEAR TO ORDER. Rev C adds several nets where a
    # single-pad slip is easy (the Kelvin sense taps, the flow UART, the
    # VBAT/VBAT_PACK split), so this now FAILS.
    stubs = [n for n in NETS if used[n] < 2]
    pend = [r for r, c in COMPONENTS.items() if c["pins"] == PENDING]
    for n in stubs:
        print(f"  !! net {n}: only {used[n]} connection(s)")
    for r in pend:
        print(f"  !! {r}: pins PENDING-RESEARCH")
    print(f"nets={len(NETS)} components={len(COMPONENTS)} connections={sum(used.values())}")
    if stubs or pend:
        print(f"FAIL: {len(stubs)} stub net(s), {len(pend)} PENDING component(s)")
        sys.exit(1)


if __name__ == "__main__":
    audit()
