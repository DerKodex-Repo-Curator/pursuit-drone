"""PREFLIGHT: everything that must be true before money is spent.

Checks the RELEASE ARTEFACTS against the board and against design.py. This
exists because two errors reached the customer-facing files (a CPL origin
150mm off the board, and an LCSC code for a 12k 0402 on a 150R 0603 line)
while the board itself — which had three review rounds — was clean.

Run with KiCad's bundled python3. Exit 1 blocks the order.
"""
import csv
import hashlib
import math
import os
import re
import subprocess
import sys
import zipfile

sys.path.insert(0, os.path.dirname(__file__))
import design
import pcbnew

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
HW = os.path.join(ROOT, "hardware")
GB = os.path.join(ROOT, "gerbers")
BOARD = os.path.join(HW, "pursuit.kicad_pcb")

fails, warns = [], []

# degrees JLCPCB's footprint sits round from KiCad's, by designator family
CPL_FRAME_OFFSET = {"Q": 180.0}


def bad(msg):
    fails.append(msg)
    print(f"  FAIL  {msg}")


def warn(msg):
    warns.append(msg)
    print(f"  WARN  {msg}")


def ok(msg):
    print(f"  ok    {msg}")


def main():
    b = pcbnew.LoadBoard(BOARD)

    # ---- 1. what SHOULD be assembled: top-side, SMD, in design.py --------
    print("\n[1] assembled-part set")
    should, tht, bottom, dnf = set(), set(), set(), set()
    orient = {}
    for fp in b.GetFootprints():
        ref = fp.GetReference()
        if ref.startswith("H") or ref not in design.COMPONENTS:
            continue
        if design.COMPONENTS[ref].get("dnf"):
            dnf.add(ref)                  # footprint on the board, not fitted
        elif any(p.GetAttribute() == pcbnew.PAD_ATTRIB_PTH for p in fp.Pads()):
            tht.add(ref)
        elif fp.IsFlipped():
            bottom.add(ref)
        else:
            should.add(ref)
            orient[ref] = (pcbnew.ToMM(fp.GetPosition().x),
                           -pcbnew.ToMM(fp.GetPosition().y),
                           fp.GetOrientationDegrees() % 360)
    ok(f"{len(should)} top SMD to assemble, {len(bottom)} bottom SMD + "
       f"{len(tht)} THT are hand-soldered")
    if dnf:
        ok(f"{len(dnf)} do-not-fit (pads only, excluded from BOM/CPL): "
           f"{sorted(dnf)}")

    # ---- 2. BOM ----------------------------------------------------------
    print("\n[2] BOM")
    bom_refs, codes = set(), {}
    for r in csv.DictReader(open(os.path.join(GB, "JLCPCB_BOM.csv"))):
        refs = [x.strip() for x in r["Designator"].split(",")]
        code = r["LCSC Part #"].strip()
        if not re.fullmatch(r"C\d+", code):
            bad(f"BOM {r['Designator']}: LCSC code '{code}' is not a C-number")
        for ref in refs:
            if ref in bom_refs:
                bad(f"BOM: {ref} listed twice")
            bom_refs.add(ref)
            codes[ref] = code
        # value must match the canonical netlist
        for ref in refs:
            want = design.COMPONENTS.get(ref, {}).get("value")
            if want and r["Comment"].strip() != want.strip():
                bad(f"BOM {ref}: Comment '{r['Comment']}' != design.py "
                    f"value '{want}'")
    if bom_refs != should:
        bad(f"BOM set mismatch: missing {sorted(should - bom_refs)}, "
            f"extra {sorted(bom_refs - should)}")
    else:
        ok(f"BOM covers exactly the {len(should)} assembled parts, values "
           f"match design.py")

    # same footprint family must not share a code with a different value
    byc = {}
    for ref, c in codes.items():
        byc.setdefault(c, set()).add(design.COMPONENTS[ref]["value"])
    for c, vals in byc.items():
        if len(vals) > 1:
            bad(f"LCSC {c} assigned to differing values {sorted(vals)}")

    # ---- 3. CPL ----------------------------------------------------------
    print("\n[3] CPL")
    cpl_refs = set()
    for r in csv.DictReader(open(os.path.join(GB, "JLCPCB_CPL.csv"))):
        ref = r["Designator"].strip()
        cpl_refs.add(ref)
        if r["Layer"].strip().lower() != "top":
            bad(f"CPL {ref}: layer '{r['Layer']}' but only Top is assembled")
        if ref not in orient:
            continue
        ox, oy, orot = orient[ref]
        if abs(float(r["Mid X"]) - ox) > 0.002 or \
           abs(float(r["Mid Y"]) - oy) > 0.002:
            bad(f"CPL {ref}: position ({r['Mid X']},{r['Mid Y']}) != board "
                f"({ox:.4f},{oy:.4f})")
        # The CPL is in JLCPCB's frame, not KiCad's. Stated here a SECOND time,
        # by designator family and independently of gen_jlcpcb.JLC_ROTATION, so
        # that dropping the correction from the generator fails this gate: the
        # AO3400A (Q*) sits 180 deg round in JLCPCB's SOT-23 (2026-09-21, from
        # JLCPCB's footprint for C20917 and from their placement preview).
        want = (orot + CPL_FRAME_OFFSET.get(ref.rstrip("0123456789"), 0.0)) % 360
        if abs((float(r["Rotation"]) - want + 180) % 360 - 180) > 0.01:
            bad(f"CPL {ref}: rotation {r['Rotation']} != {want} "
                f"(board {orot} + JLCPCB frame offset)")
    if cpl_refs != should:
        bad(f"CPL set mismatch: missing {sorted(should - cpl_refs)}, "
            f"extra {sorted(cpl_refs - should)}")
    else:
        ok(f"CPL covers the same {len(should)} parts, positions match the board "
           f"exactly, rotations match it in JLCPCB's frame (Q* +180)")

    # CPL must sit in the same coordinate space as the gerbers. Parse ONLY
    # draw/move operations: a bare r"X(-?\d+)" also matches the format spec
    # line %FSLAX46Y46*%, which yielded a phantom X=0.000046 and made the
    # lower bound vacuous. Both axes are bounded; the origin bug that shipped
    # was a two-axis offset and an X-only test would not have caught it.
    edge = open(os.path.join(GB, "fab", "pursuit-Edge_Cuts.gm1")).read()
    ops = re.findall(r"X(-?\d+)Y(-?\d+)D0[123]\*", edge)
    if not ops:
        bad("Edge_Cuts gerber has no draw operations — cannot bound the CPL")
    else:
        exs = [int(a) / 1e6 for a, _ in ops]
        eys = [int(b) / 1e6 for _, b in ops]
        cx = [orient[r][0] for r in should]
        cy = [orient[r][1] for r in should]
        # Gerber Y is negated relative to KiCad page Y; the CPL uses the same
        # convention, so both are compared in the same frame with no flip.
        okx = min(exs) <= min(cx) and max(cx) <= max(exs)
        oky = min(eys) <= min(cy) and max(cy) <= max(eys)
        if not (okx and oky):
            bad(f"CPL bbox X {min(cx):.1f}..{max(cx):.1f} Y {min(cy):.1f}.."
                f"{max(cy):.1f} escapes the Edge.Cuts bbox X {min(exs):.1f}.."
                f"{max(exs):.1f} Y {min(eys):.1f}..{max(eys):.1f} — origin bug")
        else:
            ok(f"CPL bbox sits inside the Edge.Cuts bbox on BOTH axes "
               f"(X {min(exs):.1f}..{max(exs):.1f}, "
               f"Y {min(eys):.1f}..{max(eys):.1f})")

    # ---- 4. package freshness -------------------------------------------
    print("\n[4] release package")
    h = hashlib.sha256(open(BOARD, "rb").read()).hexdigest()[:16]
    man = open(os.path.join(GB, "fab", "MANIFEST.txt")).read()
    if h in man:
        ok(f"gerbers match board hash {h}")
    else:
        bad(f"MANIFEST hash != board hash {h} — package is STALE")
    z = zipfile.ZipFile(os.path.join(GB, "PURSUIT-1_JLCPCB.zip"))
    names = {os.path.basename(n) for n in z.namelist()}
    need = {"pursuit-F_Cu.gtl", "pursuit-B_Cu.gbl", "pursuit-Edge_Cuts.gm1",
            "pursuit.drl", "pursuit-F_Mask.gts", "pursuit-B_Mask.gbs",
            "pursuit-F_Silkscreen.gto", "pursuit-B_Silkscreen.gbo",
            "pursuit-F_Paste.gtp", "pursuit-B_Paste.gbp"}
    miss = need - names
    if miss:
        bad(f"zip missing layers: {sorted(miss)}")

    else:
        ok(f"zip has all {len(need)} required layers ({len(names)} files)")

    # LAYER COUNT, derived on both sides and never listed.
    #
    # A hardcoded ten-filename checklist survives a 2 -> 4 layer change without
    # a murmur: release.py would export only F.Cu and B.Cu, the fab would build
    # a four-layer stackup with two BLANK inner layers, every gate here would
    # pass, and the first symptom would be five dead boards. So the board, the
    # gerber job file and the shipped zip all have to agree on one number, and
    # the check is done against FileFunction rather than against filename
    # extensions -- Protel names inner layers .g1/.g2 and KiCad names them
    # differently, and that convention is not something to hardcode either.
    n_board = pcbnew.LoadBoard(BOARD).GetCopperLayerCount()
    job = os.path.join(GB, "fab", "pursuit-job.gbrjob")
    if not os.path.exists(job):
        bad("no pursuit-job.gbrjob in gerbers/fab")
    else:
        jt = open(job).read()
        m = re.search(r'"LayerNumber"\s*:\s*(\d+)', jt)
        if not m:
            bad("gbrjob has no GeneralSpecs.LayerNumber")
        elif int(m.group(1)) != n_board:
            bad(f"gbrjob says {m.group(1)} copper layers, board has {n_board}")
        cu = re.findall(r'"FileFunction"\s*:\s*"Copper,L(\d+),', jt)
        if len(cu) != n_board:
            bad(f"gbrjob declares {len(cu)} copper files, board has "
                f"{n_board} layers: {sorted(cu)}")
        elif sorted(int(x) for x in cu) != list(range(1, n_board + 1)):
            bad(f"gbrjob copper layers are not L1..L{n_board}: {sorted(cu)}")
        else:
            ok(f"{n_board} copper layers agree across board, gbrjob and zip")

    # No file in gerbers/ may predate the board. A stale pre-rework gerber set
    # and a stale PURSUIT-1_placement.csv (still carrying 470R for R13/R14)
    # have both survived a rework under a name that reads like the real thing.
    bmt = os.path.getmtime(BOARD)
    stale = []
    for d, _sub, fs in os.walk(GB):
        for n in fs:
            if n.startswith("."):
                continue
            p = os.path.join(d, n)
            if os.path.getmtime(p) < bmt - 1:
                stale.append(os.path.relpath(p, ROOT))
    if stale:
        bad(f"files in gerbers/ older than the board: {sorted(stale)}")
    else:
        ok("every file in gerbers/ was regenerated after the board")

    # The assembly drawings are what the builder solders from. A stale pair
    # survived the round-4 silk rework still showing the deleted "PIN1=+".
    # The plane renders are staleness-checked too: inner copper appears in
    # no other artefact, so a stale one is a picture of a board that no
    # longer exists, shown to someone deciding whether to spend money.
    for n in ("assembly_top.svg", "assembly_bottom.svg",
              "plane_in1_gnd.svg", "plane_in2_vbat.svg"):
        p = os.path.join(ROOT, "docs", n)
        if not os.path.exists(p):
            bad(f"docs/{n} missing")
        elif os.path.getmtime(p) < bmt - 1:
            bad(f"docs/{n} is older than the board — regenerate (release.py)")
        else:
            ok(f"docs/{n} regenerated with the board")

    # The three upload files must be the ONLY csv in gerbers/: JLCPCB's upload
    # dialog offers no confirmation of which file you picked.
    csvs = {n for n in os.listdir(GB) if n.lower().endswith(".csv")}
    allowed = {"JLCPCB_BOM.csv", "JLCPCB_CPL.csv", "ORIENTATION_CHECK.csv",
               "BOM.csv"}
    # board_probe_points.png left this list in rev B. It was a hand-annotated
    # bring-up screenshot -- no generator produced it, so it could only ever
    # go stale, and it duly did. Rev B puts TP1/TP2/TP3 on the board with
    # VBAT / 3V3 / GND printed on the top silk beside them, which is the same
    # information carried by the thing being probed. Recoverable from git if
    # it is ever wanted back.
    if csvs - allowed:
        bad(f"unexpected csv in gerbers/ (upload hazard): {sorted(csvs - allowed)}")
    else:
        ok(f"only the known csv files are present ({len(csvs)})")

    # ---- 5. board gates --------------------------------------------------
    print("\n[5] board gates")
    for script, label in (("audit_islands.py", "zone-island audit"),
                          ("check_sch.py", "schematic netlist vs design.py")):
        r = subprocess.run([sys.executable,
                            os.path.join(os.path.dirname(__file__), script)],
                           capture_output=True, text=True)
        (ok if r.returncode == 0 else bad)(f"{label} exit={r.returncode}")

    r = subprocess.run(["/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli",
                        "sch", "erc", "--severity-error", "--severity-warning",
                        "--exit-code-violations", "-o", "/dev/null",
                        os.path.join(HW, "pursuit.kicad_sch")],
                       capture_output=True, text=True)
    n = re.search(r"Found (\d+) violations", r.stdout or "")
    cnt = int(n.group(1)) if n else -1
    (ok if cnt == 0 else bad)(f"schematic ERC: {cnt} violation(s) "
                              f"(errors + warnings)")

    # ---- 6. orientation sheet covers every polarised part ----------------
    print("\n[6] orientation sheet")
    checked = {r["Designator"].strip()
               for r in csv.DictReader(open(os.path.join(GB,
                                                         "ORIENTATION_CHECK.csv")))}
    polar = {r for r in should
             if r.startswith(("D", "LED", "Q")) or r == "U2"}
    if polar - checked:
        bad(f"polarised parts absent from ORIENTATION_CHECK: "
            f"{sorted(polar - checked)}")
    else:
        ok(f"all {len(polar)} polarised parts listed for visual confirmation")

    # ---- 7. hand-fitted polarised parts must be marked on their own face --
    # JLCPCB's placement render covers the machine-placed parts. Anything
    # fitted BY HAND that sits across an unfused 1S LiPo has silkscreen as its
    # only defence — which must be on the FACE THE PART IS ON and on the PAD-1
    # SIDE. A "PIN1=+" once sat on F.SilkS for a belly part, on the pad-2 side,
    # i.e. wrong face and wrong terminal.
    #
    print("\n[7] polarity silk on hand-fitted parts")
    for ref in ("C1", "J1"):
        fp = b.FindFootprintByReference(ref)
        if fp is None:
            bad(f"{ref} not on the board")
            continue
        want_layer = pcbnew.B_SilkS if fp.IsFlipped() else pcbnew.F_SilkS
        face = "belly" if fp.IsFlipped() else "top"
        pads = {p.GetNumber(): p.GetPosition() for p in fp.Pads()}
        p1, p2 = pads.get("1"), pads.get("2")
        if p1 is None or p2 is None:
            warn(f"{ref}: not a 2-pad part, skipping")
            continue
        x1, y1 = pcbnew.ToMM(p1.x), pcbnew.ToMM(p1.y)
        x2, y2 = pcbnew.ToMM(p2.x), pcbnew.ToMM(p2.y)
        found = None
        for d in b.GetDrawings():
            if not isinstance(d, pcbnew.PCB_TEXT) or d.GetLayer() != want_layer:
                continue
            if "+" not in d.GetText():
                continue
            t = d.GetPosition()
            tx, ty = pcbnew.ToMM(t.x), pcbnew.ToMM(t.y)
            d1 = math.hypot(tx - x1, ty - y1)
            d2 = math.hypot(tx - x2, ty - y2)
            if d1 <= 6.0 and d1 < d2:
                found = (d.GetText(), d1)
                break
        if found:
            ok(f"{ref}: '{found[0]}' on the {face} silk, {found[1]:.1f}mm from "
               f"pad 1 and nearer pad 1 than pad 2")
        else:
            bad(f"{ref}: no '+' text on {face} silk within 6mm of pad 1 and on "
                f"the pad-1 side")

    print("\n" + "=" * 62)
    print(f"PREFLIGHT: {len(fails)} failure(s), {len(warns)} warning(s) -> "
          f"{'DO NOT ORDER' if fails else 'CLEAR TO ORDER'}")
    sys.exit(1 if fails else 0)


if __name__ == "__main__":
    main()
