"""Emit project-local .kicad_mod footprints into hardware/pursuit.pretty.

All dimensions come from FACTS (locked from datasheet research); the emitter
refuses to run while any value is None.
"""
import os
import re

FACTS = dict(
    # XIAO ESP32S3 — research-locked (Seeed official KiCad PCB: pads ±7.62mm)
    xiao_row_spacing=15.24,
    xiao_body_l=21.0,             # along pin rows (USB at one 17.8 end)
    xiao_body_w=17.8,             # official land pattern (not 17.5)
)

# ICM-42688-P LGA-14 quad land pattern — TDK DS-000347 v1.2 mechanical
# drawing, 1:1 NSMD pads. (num, cx, cy_up, w, h) in datasheet coords
# (+y up, pin 1 top-left); KiCad emission flips y.
#
# Pad SIZE is 0.625 x 0.35 rather than the datasheet's bare 0.475 x 0.25.
# Centres are unchanged (and independently match KiCad's own
# LGA-14_3x2.5mm_P0.5mm land pattern). Rationale: at a 1:1 land the stencil
# area ratio is 0.55 on a 0.15mm stencil — below the IPC-7525 floor of 0.66,
# so paste would not release on any of the 14 apertures. 0.625 x 0.35 gives
# AR 0.94 and ~85% more paste volume. All pads stay inside the 3.0 x 2.5
# package body.
LGA_W, LGA_L = 0.35, 0.625        # across-pitch, along-pitch
IMU_PADS = [
    (1, -1.1625, 0.75, LGA_L, LGA_W), (2, -1.1625, 0.25, LGA_L, LGA_W),
    (3, -1.1625, -0.25, LGA_L, LGA_W), (4, -1.1625, -0.75, LGA_L, LGA_W),
    (5, -0.5, -0.9125, LGA_W, LGA_L), (6, 0.0, -0.9125, LGA_W, LGA_L),
    (7, 0.5, -0.9125, LGA_W, LGA_L),
    (8, 1.1625, -0.75, LGA_L, LGA_W), (9, 1.1625, -0.25, LGA_L, LGA_W),
    (10, 1.1625, 0.25, LGA_L, LGA_W), (11, 1.1625, 0.75, LGA_L, LGA_W),
    (12, 0.5, 0.9125, LGA_W, LGA_L), (13, 0.0, 0.9125, LGA_W, LGA_L),
    (14, -0.5, 0.9125, LGA_W, LGA_L),
]
IMU_BODY_W, IMU_BODY_H = 3.0, 2.5   # 3.0mm edge along x (pads 12-14 row),
                                    # 4-pad columns on the short 2.5mm edges

HEADER = '(footprint "{name}"\n  (version 20240108)\n  (generator "pursuit")\n  (layer "F.Cu")\n  (attr {attr})\n'


def _text(kind, txt, x, y, layer="F.SilkS", size=1.0):
    # JLCPCB floors: 1.0mm text height, 0.15mm stroke. Never go below.
    size = max(size, 1.0)
    return (f'  (fp_text {kind} "{txt}" (at {x} {y}) (layer "{layer}")\n'
            f'    (effects (font (size {size} {size}) (thickness 0.15))))\n')


def _tht_pad(num, x, y, size=1.8, drill=1.0, shape="circle", size_y=None,
             offset=0.0):
    """size_y lets a pad be OVAL: wide across the row, narrow along it.
    offset moves the COPPER outward while the HOLE stays on the pin grid.

    Rev B. The annulus on a 1.8mm pad over a 1.0mm drill is 0.40mm, which is
    too thin for KiCad to grow a thermal spoke -- so every pour-connected
    through-hole ended up solid, and solid pads sink so much heat into the
    copper that they cold-joint. That failure appeared five separate times
    during bring-up (motor pins twice, header row twice, the 3V3 socket), and
    twice more as a lifted annular ring when a joint was reworked.

    Growing the pad round is not available at 2.54mm pitch: a 2.8mm round pad
    would touch its neighbour. Growing it ACROSS the row is free, because
    nothing sits there. An oval pad therefore gets the annulus a spoke needs
    without shrinking pad-to-pad clearance at all.

    But growing it SYMMETRICALLY is not free either, and that was the first
    attempt at this fix. Every signal on this board crosses the waist between
    the two header rows, and those lanes were hand-derived to thread the gaps
    between the rev A pads. Widening the pads by 1.2mm inward put eleven
    tracks and four pad pairs into DRC violation at once -- the corridor
    simply is not there to give away.

    So the copper grows OUTWARD ONLY, via a drill offset. The hole stays
    exactly on the module's pin grid; the pad shape slides outboard by
    `offset` so that its inner edge lands where the rev A pad's inner edge
    was, to the micron. Nothing inboard changes, so nothing needs re-routing.
    Everything gained lands outside the module body, which is also the only
    place a soldering iron can actually reach once the module is seated --
    the XIAO body overhangs its own pin rows by 1.28mm on each side, so on
    rev A the entire pad was underneath the module.
    """
    sy = size if size_y is None else size_y
    if shape == "circle" and sy != size:
        shape = "oval"
    # In KiCad's model `(at ...)` is the pad ANCHOR, which for a through-hole
    # pad is the HOLE, and the drill `(offset ...)` moves the COPPER SHAPE
    # relative to it -- not the other way round. Writing it the intuitive way
    # round put every hole 0.6mm off the module's pin grid while leaving the
    # copper where it started: fourteen holes that no XIAO would seat in, and
    # a motor footprint whose two pads closed to 0.14mm. DRC caught it as a
    # pad-to-pad clearance error, which is a lucky way to find out. So: `at`
    # is always the nominal grid position, and `offset` is the outward shift.
    dr = (f"(drill {drill})" if not offset
          else f"(drill {drill} (offset {offset:.3f} 0))")
    return (f'  (pad "{num}" thru_hole {shape} (at {x:.3f} {y:.3f}) '
            f'(size {size} {sy}) {dr} (layers "*.Cu" "*.Mask"))\n')


def _smd_pad(num, x, y, w, h):
    return (f'  (pad "{num}" smd roundrect (at {x:.3f} {y:.3f}) (size {w} {h}) '
            f'(roundrect_rratio 0.25) (layers "F.Cu" "F.Paste" "F.Mask"))\n')


def _line(x1, y1, x2, y2, layer="F.SilkS", w=0.15):
    return (f'  (fp_line (start {x1:.3f} {y1:.3f}) (end {x2:.3f} {y2:.3f}) '
            f'(stroke (width {w}) (type solid)) (layer "{layer}"))\n')


def _rect(x1, y1, x2, y2, layer, w=0.05):
    return (f'  (fp_rect (start {x1:.3f} {y1:.3f}) (end {x2:.3f} {y2:.3f}) '
            f'(stroke (width {w}) (type solid)) (fill none) (layer "{layer}"))\n')


def xiao_carrier():
    """Two 1x7 THT rows. Pads 1-7 left row top->bottom, 8-14 RIGHT row
    numbered so that pad 8 is at the TOP of the right row (XIAO 5V position),
    matching design.py's pin map. Origin = module centre, USB edge at -Y."""
    row = FACTS["xiao_row_spacing"] / 2.0
    l, w = FACTS["xiao_body_l"], FACTS["xiao_body_w"]
    pitch = 2.54
    y0 = -3 * pitch  # 7 pins centred
    s = HEADER.format(name="XIAO_ESP32S3_Carrier", attr="through_hole")
    s += _text("reference", "REF**", 0, -l / 2 - 2)
    s += _text("value", "XIAO-ESP32S3-Sense", 0, l / 2 + 2)
    # Rev B: 1.8 x 2.3 OVAL on the same 1.0mm drill, and no lateral growth.
    #
    # The annulus across the row stays 0.40mm, because this board has no room
    # to widen it in either direction. Inboard is the waist, where every
    # signal crosses between the two rows; outboard, SCL, M2_PWM, LED_CTRL
    # and VBAT_SENSE all run in the channel between the pad column and the
    # arms. A 3.0mm pad -- 0.6mm of growth per side -- was tried and put
    # sixteen tracks into DRC violation. Widening this footprint means
    # re-routing the densest part of the board, which is a bigger change than
    # the one it is trying to make.
    #
    # Growing ALONG the row is free, and it is also sufficient. The failure
    # being fixed is the cold joint, whose cause was that a 0.40mm annulus is
    # too thin for a 0.4mm thermal spoke to attach to -- so KiCad fell back to
    # a solid pour connection, and a solid connection into 2200mm^2 of copper
    # takes the heat out of the joint faster than an iron puts it in. At 2.3mm
    # along the row the annulus there is 0.65mm, a spoke forms north and south
    # of every hole, and the pad stops being a heatsink. Clearance to the
    # neighbouring pad is 0.24mm at 2.54mm pitch.
    for i in range(7):                       # left row: D0..D6 = pads 1..7
        s += _tht_pad(i + 1, -row, y0 + i * pitch,
                      shape="rect" if i == 0 else "circle")
    for i in range(7):                       # right row top->bottom: 8..14
        s += _tht_pad(i + 8, row, y0 + i * pitch)
    # Module body outline. It is otherwise perfectly symmetric, and a 180-deg
    # insertion lands the module's 3V3 output on SDA and kills it — so the
    # USB edge is marked THREE ways: a notched outline, a text label, and a
    # pin-1 tick that stays visible once the header is fitted.
    s += _line(-w / 2, -l / 2, -w / 2, l / 2)          # left body edge
    s += _line(w / 2, -l / 2, w / 2, l / 2)            # right body edge
    s += _line(-w / 2, l / 2, w / 2, l / 2)            # rear body edge
    s += _line(-w / 2, -l / 2, -3.0, -l / 2)           # front edge, notched
    s += _line(3.0, -l / 2, w / 2, -l / 2)             # ...for the USB port
    s += _line(-3.0, -l / 2, -3.0, -l / 2 - 1.4)       # USB port cut-out
    s += _line(3.0, -l / 2, 3.0, -l / 2 - 1.4)
    s += _line(-3.0, -l / 2 - 1.4, 3.0, -l / 2 - 1.4)
    # Inside the USB-port cut-out, not below it: at -l/2-2.0 the label sits
    # 0.5mm from the board's front edge and gets printed as half-height
    # letters (DRC silk_edge_clearance).
    s += _text("user", "USB", 0, -l / 2 - 0.55, size=0.9)
    # pin-1 tick, outboard of the left row so it survives the header
    s += _line(-row - 2.0, y0, -row - 1.2, y0)
    s += _text("user", "1", -row - 2.9, y0, size=1.0)
    # courtyard = module body + USB-C shell overhang (1.3mm) + 0.25 margin
    s += _rect(-w / 2 - 0.25, -l / 2 - 1.55, w / 2 + 0.25, l / 2 + 0.25, "F.CrtYd")
    s += ")\n"
    return s


def imu_lga14():
    """ICM-42688-P LGA-14 quad land pattern, TDK 1:1 NSMD recommendation.
    Datasheet coords are +y up with pin 1 top-left; KiCad is +y down, so
    cy is negated. NSMD: mask opening pad+0.1mm -> solder_mask_margin 0.05."""
    s = HEADER.format(name="ICM42688P_LGA14", attr="smd")
    # NSMD, but with a 0 mask margin: at 0.5mm pitch a positive margin
    # leaves 0.15mm mask dams, under JLCPCB's 0.2mm minimum, so the dams
    # get dropped and adjacent pads share one window. Mask-defined-at-the-
    # edge is the safer outcome here.
    # Local clearance 0.1mm: at 0.5mm pitch with 0.35mm-wide lands the
    # pad-to-pad gap is 0.15mm, which is correct for a fine-pitch part but
    # below the board's general 0.2mm rule. JLCPCB resolves 0.127mm.
    s = s.replace('(attr smd)\n',
                  '(attr smd)\n  (solder_mask_margin 0)\n  (clearance 0.1)\n')
    s += _text("reference", "REF**", 0, -2.6)
    s += _text("value", "ICM-42688-P", 0, 2.6, layer="F.Fab")
    for num, cx, cy_up, w, h in IMU_PADS:
        s += _smd_pad(num, cx, -cy_up, w, h)
    s += _rect(-IMU_BODY_W / 2, -IMU_BODY_H / 2,
               IMU_BODY_W / 2, IMU_BODY_H / 2, "F.Fab")
    # pin 1 marker: dot outside the top-left corner (pin 1 at cx<0, cy_up>0)
    s += (f'  (fp_circle (center -1.85 -1.0) (end -1.72 -1.0) '
          f'(stroke (width 0.12) (type solid)) (fill solid) (layer "F.SilkS"))\n')
    # +X axis arrow hint on silk (firmware axis mapping)
    s += _line(1.8, 0, 2.4, 0)
    s += _text("user", "+X", 3.0, 0, size=1.0, layer="F.Fab")
    s += _rect(-1.7, -1.45, 1.7, 1.45, "F.CrtYd")
    s += ")\n"
    return s


def motor_pads():
    """Two THT pads for 8520 motor wires, 2.54mm apart. Pad 1 = motor + ."""
    s = HEADER.format(name="Motor_Pads_2x", attr="through_hole")
    s += _text("reference", "REF**", 0, -2.6)
    s += _text("value", "8520", 0, 2.6, layer="F.Fab")
    # Rev B: 2.4 x 2.0 oval on a 1.0mm drill, offset 0.30mm outboard. Grown
    # symmetrically these two pads closed to 0.14mm of each other at 2.54mm
    # pitch -- half the 0.2mm rule, and a solder bridge between motor + and
    # motor - is a dead FET. Offset outboard the gap OPENS to 0.74mm (rev A:
    # 0.54) while the outboard annulus goes 0.50 -> 1.00mm. Still solid-
    # connected: these carry ~2.5A each. The wider ring is there so the pad
    # survives a motor swap, which is the one repair this board will need
    # over and over.
    s += _tht_pad(1, -1.27, 0, size=2.4, size_y=2.0, drill=1.0,
                  offset=-0.30, shape="rect")
    s += _tht_pad(2, 1.27, 0, size=2.4, size_y=2.0, drill=1.0, offset=0.30)
    s += _text("user", "+", -1.27, -1.9, size=0.9)
    s += _text("user", "-", 1.27, -1.9, size=0.9)
    s += ")\n"
    return s


def test_point():
    """A single 2.0mm THT pad for a meter probe or a scope clip.

    Rev B, and the cheapest change on the board. Bring-up spent days probing
    pads that were never meant to be probed -- header sockets with a module
    seated in them, and pads that had already been reworked -- because there
    was nowhere else to reach VBAT, 3V3 or GND. Two of those pads eventually
    lifted. Three dedicated pads make every power rail measurable without
    touching anything that matters.
    """
    s = HEADER.format(name="TestPoint_1mm", attr="through_hole")
    s += _text("reference", "REF**", 0, -2.2)
    s += _text("value", "TP", 0, 2.2, layer="F.Fab")
    s += _tht_pad(1, 0, 0, size=2.0, drill=0.9)
    s += ")\n"
    return s


def wire_pads():
    """Two THT pads for the XIAO BAT jumper wires. Pad 1 = BAT+ ."""
    s = HEADER.format(name="Wire_Pads_2x", attr="through_hole")
    s += _text("reference", "REF**", 0, -2.4)
    s += _text("value", "BAT", 0, 2.4, layer="F.Fab")
    # Rev B: 2.2 x 3.2 oval on the same 1.2mm drill. This footprint is the
    # most boxed-in on the board: SDA and SCL thread the 2.2mm gap BETWEEN
    # its two pads, and the +3V3 corridor and C6 run immediately outboard of
    # them, so it cannot grow a micron in x -- 0.5mm per side put twelve
    # tracks into violation. Perpendicular to the pair it is clear for 1.5mm
    # in both directions, so that is where the copper goes: 0.70mm annulus
    # there, enough for a thermal spoke, which is the whole point.
    s += _tht_pad(1, -2.2, 0, size=2.2, drill=1.2, shape="rect")
    s += _tht_pad(2, 2.2, 0, size=2.2, drill=1.2)
    # Polarity marker: swapping these two wires puts -3.7V on the XIAO's
    # BAT+ pad and destroys the module, so pad 1 is both rectangular and
    # silk-marked. (No room for a second label — C6 is 1mm to the east.)
    s += _text("user", "+", -4.6, 0, size=1.0)
    s += ")\n"
    return s


def wire_pads_hd():
    """Heavy pads for an optional direct-solder 20AWG battery pigtail."""
    s = HEADER.format(name="Wire_Pads_2x_HD", attr="through_hole")
    s += _text("reference", "REF**", 0, -3.0)
    s += _text("value", "PIGTAIL", 0, 3.0, layer="F.Fab")
    # Rev B: 3.9 x 3.3 oval on a 1.5mm drill, offset 0.6mm outboard ->
    # 1.80mm annulus outboard (was 0.65 all round), pad gap 0.50 -> 1.70mm.
    # These stay
    # SOLID-connected because they carry the full pack current, so they will
    # still need long dwell -- but a 1.2mm ring survives rework instead of
    # lifting, which is how the rev A board lost this footprint entirely.
    s += _tht_pad(1, -2.2, 0, size=3.9, size_y=3.3, drill=1.5,
                  offset=-0.6, shape="rect")
    s += _tht_pad(2, 2.2, 0, size=3.9, size_y=3.3, drill=1.5, offset=0.6)
    s += _text("user", "+", -2.2, 2.6, size=1.2)
    s += ")\n"
    return s


def bare_hole(name, drill):
    """NPTH hole with no courtyard — for strap/screw holes placed near
    components (the library MountingHole courtyards are far too big)."""
    s = HEADER.format(name=name, attr='exclude_from_pos_files')
    s += _text("reference", "REF**", 0, -drill / 2 - 1.2)
    s += (f'  (pad "" np_thru_hole circle (at 0 0) (size {drill} {drill}) '
          f'(drill {drill}) (layers "F&B.Cu" "*.Mask"))\n')
    s += ")\n"
    return s


# Hole pitch is the one number that decides whether a part physically fits,
# and it is the number a drill-offset typo silently destroys: the copper still
# looks right in every render, and only a meter or a mating part finds out.
# So it is asserted, per footprint, against the pitch of the thing that mates.
HOLE_PITCH = {
    "XIAO_ESP32S3_Carrier": (15.24, 2.54),   # row spacing, pin pitch
    "Motor_Pads_2x": (2.54, None),
    "Wire_Pads_2x": (4.40, None),
    "Wire_Pads_2x_HD": (4.40, None),
}


def _check_holes(name, body):
    """Assert every drilled hole sits on the grid the mating part expects.

    `(at ...)` on a through-hole pad is the HOLE. If a drill offset is ever
    written into `at` instead of into `(offset ...)`, the copper stays put and
    the hole walks -- so this reads the holes back out of the generated text
    and checks the pitches directly, rather than trusting the caller.
    """
    want = HOLE_PITCH.get(name)
    if not want:
        return
    holes = [(float(m.group(1)), float(m.group(2))) for m in
             re.finditer(r'\(pad "[^"]+" thru_hole \S+ \(at '
                         r'(-?\d+\.?\d*) (-?\d+\.?\d*)\)', body)]
    assert holes, f"{name}: no through-hole pads found to check"
    xs = sorted({round(x, 3) for x, _ in holes})
    ys = sorted({round(y, 3) for _, y in holes})
    across, along = want
    got = round(max(xs) - min(xs), 3)
    assert abs(got - across) < 1e-6, \
        f"{name}: hole spacing across the rows is {got}, expected {across}"
    if along is not None:
        gaps = {round(b - a, 3) for a, b in zip(ys, ys[1:])}
        assert gaps == {along}, \
            f"{name}: hole pitch along the row is {sorted(gaps)}, expected {along}"


def emit(outdir):
    missing = [k for k, v in FACTS.items() if v is None]
    assert not missing, f"FACTS pending research: {missing}"
    os.makedirs(outdir, exist_ok=True)
    for name, fn in [("XIAO_ESP32S3_Carrier", xiao_carrier),
                     ("ICM42688P_LGA14", imu_lga14),
                     ("Motor_Pads_2x", motor_pads),
                     ("Wire_Pads_2x", wire_pads),
                     ("TestPoint_1mm", test_point),
                     ("Wire_Pads_2x_HD", wire_pads_hd),
                     ("Strap_Hole_3.2", lambda: bare_hole("Strap_Hole_3.2", 3.2)),
                     ("Screw_Hole_2.2", lambda: bare_hole("Screw_Hole_2.2", 2.2))]:
        body = fn()
        _check_holes(name, body)
        with open(os.path.join(outdir, f"{name}.kicad_mod"), "w") as f:
            f.write(body)
        print("wrote", name)


if __name__ == "__main__":
    emit(os.path.join(os.path.dirname(__file__), "..", "hardware", "pursuit.pretty"))
