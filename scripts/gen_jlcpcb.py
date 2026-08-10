"""Emit JLCPCB PCBA files: BOM.csv (their column format) and CPL.csv
(pick-and-place), for TOP-SIDE SMD assembly only.

Deliberately excludes:
  - bottom-side SMD (C1, C10-C13) and all THT/wire parts, which the builder
    hand-solders;
  - the XIAO module itself (it plugs into headers, it is not a placed part).

Rotation: KiCad's footprint rotation is emitted as-is. JLCPCB's part-library
orientation often differs, and a 180-degree error on a polarised part is
fatal, so every polarised part is listed in the ORIENTATION CHECK sheet for
manual verification against JLCPCB's assembly preview.
"""
import csv
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
import design
import pcbnew

HW = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "hardware"))
OUT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "gerbers"))
OX, OY = 150.0, 100.0

# LCSC part numbers. Only fill in codes that have been verified against a
# live LCSC/JLCPCB listing — a wrong code silently fits the wrong part.
# Verified against live LCSC / JLCPCB parts listings (Aug 2026).
LCSC = {
    "Q": ("C20917", "AO3400A"),
    "D": ("C8678", "SS34"),
    "R1": ("C22775", "100R"), "R2": ("C22775", "100R"),
    "R3": ("C22775", "100R"), "R4": ("C22775", "100R"),
    # 1k, not 10k: bounds the gate below threshold even against an
    # unexpected ~45k pull-up on a PWM pin. See design.py.
    "R5": ("C21190", "1k 0603 1% 0603WAF1001T5E"),
    "R6": ("C21190", "1k 0603 1% 0603WAF1001T5E"),
    "R7": ("C21190", "1k 0603 1% 0603WAF1001T5E"),
    "R8": ("C21190", "1k 0603 1% 0603WAF1001T5E"),
    "R9": ("C23162", "4.7k"), "R10": ("C23162", "4.7k"),
    "R11": ("C25803", "100k"), "R12": ("C25803", "100k"),
    "U2": ("C1850418", "ICM-42688-P"),
    "C2": ("C57112", "10nF 0603 X7R 50V"),
    "C3": ("C14663", "100nF 0603 X7R 50V"),
    "C8": ("C14663", "100nF 0603 X7R 50V"),
    "C4": ("C23630", "2.2uF 0603 X5R 16V"),
    "C6": ("C45783", "22uF 0805 X5R 25V"),
    "C7": ("C15008", "100uF 1206 X5R 6.3V"),
    "R13": ("C22808", "150R 0603 1% 0603WAF1500T5E"),
    "R14": ("C22808", "150R 0603 1% 0603WAF1500T5E"),
    "LED1": ("C84261", "LED 0805 yellow NCD0805Y1 Vf1.6-2.6V"),
    "LED2": ("C84256", "LED 0805 red NCD0805R1 Vf1.6-2.6V"),
}
PENDING = "TBD"          # filled from the sourcing agent's verified results

# Parts whose orientation is fatal if rotated 180 deg.
POLARISED = {
    "D": "SS34 SMA — CATHODE (banded end) must face VBAT, i.e. INBOARD "
         "toward the board centre along the arm.",
    "LED": "LED 0805 — cathode to GND. On this board GND is the pad nearer "
           "the board centre.",
    "Q": "AO3400A SOT-23 — pin1=Gate, pin2=Source, pin3=Drain. The single "
         "pin (Drain) faces OUTBOARD along the arm.",
    "U2": "ICM-42688-P LGA-14 — pin 1 dot is at the board's FRONT-LEFT of "
          "the package (silk dot marks it). A 90 or 180 error here also "
          "silently rotates the gyro axes.",
}


def main():
    b = pcbnew.LoadBoard(os.path.join(HW, "pursuit.kicad_pcb"))
    bom_rows, cpl_rows, check_rows, missing = [], [], [], []

    for fp in sorted(b.GetFootprints(), key=lambda f: f.GetReference()):
        ref = fp.GetReference()
        if ref.startswith("H") or ref not in design.COMPONENTS:
            continue
        pads = list(fp.Pads())
        if any(p.GetAttribute() == pcbnew.PAD_ATTRIB_PTH for p in pads):
            continue                      # THT: hand-soldered
        if fp.IsFlipped():
            continue                      # bottom side: hand-soldered
        comp = design.COMPONENTS[ref]
        if comp.get("dnf"):
            continue                      # pads only — see design.py
        key = ref if ref in LCSC else ref.rstrip("0123456789")
        code, _ = LCSC.get(key, (PENDING, ""))
        if code == PENDING:
            missing.append(f"{ref} ({comp['value']})")
        pos = fp.GetPosition()
        # ABSOLUTE page coordinates, Y negated — exactly what KiCad's own
        # position-file export emits, and therefore the same origin the
        # Gerbers use. Subtracting the board-centre offset here put every
        # part 150mm off the board in JLCPCB's viewer.
        x = pcbnew.ToMM(pos.x)
        y = -pcbnew.ToMM(pos.y)
        rot = fp.GetOrientationDegrees() % 360
        bom_rows.append([comp["value"], ref, comp["footprint"], code])
        cpl_rows.append([ref, f"{x:.4f}", f"{y:.4f}", "Top", f"{rot:.2f}"])
        # Match the polarity rule on the DESIGNATOR FAMILY, never on the
        # LCSC key — adding an explicit per-part code once changed `key`
        # from "LED" to "LED1" and both LEDs silently vanished from the
        # orientation checklist.
        fam = ref.rstrip("0123456789")
        for pk, note in POLARISED.items():
            if fam == pk or ref == pk:
                check_rows.append([ref, comp["value"], f"{rot:.1f}", note])

    # merge BOM rows that share value+footprint+code (JLCPCB wants one line
    # per distinct part, designators comma-joined)
    merged = {}
    for val, ref, fpn, code in bom_rows:
        merged.setdefault((val, fpn, code), []).append(ref)
    with open(os.path.join(OUT, "JLCPCB_BOM.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["Comment", "Designator", "Footprint", "LCSC Part #"])
        for (val, fpn, code), refs in sorted(merged.items(),
                                             key=lambda kv: kv[1][0]):
            w.writerow([val, ",".join(sorted(refs)), fpn, code])
    with open(os.path.join(OUT, "JLCPCB_CPL.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["Designator", "Mid X", "Mid Y", "Layer", "Rotation"])
        w.writerows(sorted(cpl_rows))
    with open(os.path.join(OUT, "ORIENTATION_CHECK.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["Designator", "Value", "KiCad rotation",
                    "What to confirm on JLCPCB's preview render"])
        w.writerows(sorted(check_rows))

    print("LCSC map (verify value AND package against JLCPCB's match):")
    for (val, fpn, code), refs in sorted(merged.items(), key=lambda kv: kv[1][0]):
        exp = next((d for k, (c, d) in LCSC.items() if c == code), "")
        print(f"   {','.join(sorted(refs)):14s} {val:26s} {fpn:22s} {code:10s} {exp}")
    print()
    print(f"assembled parts   : {len(cpl_rows)}")
    print(f"distinct BOM lines: {len(merged)}")
    print(f"polarised to check: {len(check_rows)}")
    if missing:
        print(f"LCSC # STILL NEEDED ({len(missing)}): {', '.join(missing)}")
    else:
        print("LCSC #: complete")


if __name__ == "__main__":
    main()
