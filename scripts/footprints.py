"""Emit project-local .kicad_mod footprints into hardware/pursuit.pretty.

All dimensions come from FACTS (locked from datasheet research); the emitter
refuses to run while any value is None.
"""
import os

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


def _tht_pad(num, x, y, size=1.8, drill=1.0, shape="circle"):
    return (f'  (pad "{num}" thru_hole {shape} (at {x:.3f} {y:.3f}) '
            f'(size {size} {size}) (drill {drill}) (layers "*.Cu" "*.Mask"))\n')


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
    s += _tht_pad(1, -1.27, 0, size=2.0, drill=1.0, shape="rect")
    s += _tht_pad(2, 1.27, 0, size=2.0, drill=1.0)
    s += _text("user", "+", -1.27, -1.9, size=0.9)
    s += _text("user", "-", 1.27, -1.9, size=0.9)
    s += ")\n"
    return s


def wire_pads():
    """Two THT pads for the XIAO BAT jumper wires. Pad 1 = BAT+ ."""
    s = HEADER.format(name="Wire_Pads_2x", attr="through_hole")
    s += _text("reference", "REF**", 0, -2.4)
    s += _text("value", "BAT", 0, 2.4, layer="F.Fab")
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
    s += _tht_pad(1, -2.2, 0, size=2.8, drill=1.5, shape="rect")
    s += _tht_pad(2, 2.2, 0, size=2.8, drill=1.5)
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


def emit(outdir):
    missing = [k for k, v in FACTS.items() if v is None]
    assert not missing, f"FACTS pending research: {missing}"
    os.makedirs(outdir, exist_ok=True)
    for name, fn in [("XIAO_ESP32S3_Carrier", xiao_carrier),
                     ("ICM42688P_LGA14", imu_lga14),
                     ("Motor_Pads_2x", motor_pads),
                     ("Wire_Pads_2x", wire_pads),
                     ("Wire_Pads_2x_HD", wire_pads_hd),
                     ("Strap_Hole_3.2", lambda: bare_hole("Strap_Hole_3.2", 3.2)),
                     ("Screw_Hole_2.2", lambda: bare_hole("Screw_Hole_2.2", 2.2))]:
        with open(os.path.join(outdir, f"{name}.kicad_mod"), "w") as f:
            f.write(fn())
        print("wrote", name)


if __name__ == "__main__":
    emit(os.path.join(os.path.dirname(__file__), "..", "hardware", "pursuit.pretty"))
