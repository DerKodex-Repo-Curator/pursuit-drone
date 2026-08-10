"""Generate hardware/pursuit.kicad_pcb from design.py via the KiCad 10
pcbnew API. Run with KiCad's bundled python3.

Board coords: origin at board centre, +y = REAR, -y = FRONT (camera end).
Everything below in mm; converted at the API boundary.
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
import design
import footprints as fplib_mod

try:                      # absent under plain python3 (check_place only
    import pcbnew         # needs the PLACE tables, not the KiCad API)
    from pcbnew import VECTOR2I, FromMM
except ModuleNotFoundError:
    pcbnew = None

HW = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "hardware"))
KFP = "/Applications/KiCad/KiCad.app/Contents/SharedSupport/footprints"
OX, OY = 150.0, 100.0        # canvas offset of board centre

# ---------------------------------------------------------------- geometry --
WHEELBASE = 100.0
MOTOR_R = WHEELBASE / 2                    # 50.0 radial
C45 = math.cos(math.radians(45))
MX = MOTOR_R * C45                         # 35.355 → motor centres (±MX, ±MY)
RING_OUT = 10.6                            # motor ring outer radius
MOTOR_HOLE_D = 8.70                        # CLEARANCE fit, not press fit.
# An 8.45mm hole for an 8.50mm can is a 0.05mm interference, but JLCPCB's
# routed-outline tolerance is +/-0.2mm: at -0.2 the interference becomes
# 0.25mm and the hoop stress at the tie-hole edge reaches ~4.5x FR4's
# tensile strength (the ring cracks as the motor goes in); at +0.2 it is a
# 0.15mm clearance and there is no press fit at all. There is no tolerance
# window, so the fit is made a deliberate clearance and retention comes
# from the zip ties (+ a drop of CA if needed).
ARM_HALF = 6.0                             # arm width 12.0. Research said
# >=10mm for crash survival; 12 also gives the VBAT pour a clear channel
# past the 2mm M_SW jog and the LED lane, which at 10mm pinched the arm
# copper into an island. Costs ~0.7g total.
PLATE_HALF = 21.0                          # centre plate 42x42
TIE_HOLE_OFFSET = 7.5                      # motor zip-tie holes, perp to arm
# Webs with the above: hole edge -> tie hole = 7.5-1.6-4.35 = 1.55mm,
# tie hole -> ring edge = 10.6-7.5-1.6 = 1.50mm. Worst case at +/-0.2 on
# both routed edges: 1.15 / 1.10mm, still above JLCPCB's 1.0mm minimum.
MOTOR_CU_KEEPOUT_R = 5.7                   # no copper within this radius of
# a motor centre: the metal can spans all 1.6mm of board thickness, and the
# pours otherwise come within 0.41mm of the routed wall on BOTH faces —
# one mask scratch and the can bridges VBAT to GND across the pack.
# Camera-bracket hardware must be NYLON (see BOM). A metal M2 head sits on
# the VBAT pour and a metal nut on the GND pour, 1.6mm apart through the
# board — that is a dead short across the pack if either scratches mask.
ARC_STEP_DEG = 4.0

FAB = dict(                                # JLCPCB, locked from research
    edge_clearance=0.4, via_drill=0.4, via_dia=0.8,
    sig_w=0.3, pwr_w=0.6, motor_w=2.0, fine_w=0.15, src_w=1.2,
    mid_w=0.8,
)

# Motor numbering: M1 FL(-x,-y)  M2 FR(+x,-y)  M3 RL(-x,+y)  M4 RR(+x,+y)
MOTOR_POS = {1: (-MX, -MX), 2: (MX, -MX), 3: (-MX, MX), 4: (MX, MX)}
SIGN = {1: (-1, -1), 2: (1, -1), 3: (-1, 1), 4: (1, 1)}

# ------------------------------------------------------------- placements --
# ref: (x, y, rot_deg, side)  rot is KiCad CCW; side 'B' flips about x-axis
#
# Architecture: motor-drive stages march out along each arm (gate R + pull-
# down at the plate corner, Q at the corner tip, flyback D on the arm,
# motor pads at radial 39.5) so the centre plate stays uncongested.
def _arm_rot(i):
    sx, sy = SIGN[i]
    return math.degrees(math.atan2(-sy, sx))     # long axis along the arm


def _radial(i, r, perp=0.0):
    sx, sy = SIGN[i]
    ux, uy = sx * C45, sy * C45
    px, py = -uy, ux
    return (r * ux + perp * px, r * uy + perp * py)


PLACE = {
    "U1":  (0.0, -8.0, 0, "F"),      # XIAO, USB/camera edge at -y (front)
    "J4":  (0.0, 4.2, 0, "F"),       # BAT wire pads just behind module
    "U2":  (0.0, 10.2, 0, "F"),      # IMU at centre, axes board-aligned
    "C2":  (-4.5, 10.4, 90, "F"),     # VDD bypass
    "C4":  (-6.8, 10.2, 90, "F"),    # VDD bulk
    "C3":  (4.5, 10.4, 90, "F"),      # VDDIO bypass
    "C6":  (5.4, 4.3, 0, "F"),       # 3V3 bulk
    "C7":  (7.3, 11.4, 90, "F"),     # 3V3 WiFi-burst bulk
    "R9":  (-11.9, 10.2, 270, "F"),  # SDA pullup (pad1=SDA south)
    "R10": (-9.4, 10.2, 270, "F"),   # SCL pullup (pad1=SCL south)
    "R11": (19.0, -6.0, 90, "F"),    # VBAT divider top
    "R12": (19.0, -2.0, 90, "F"),    # divider bottom
    "C8":  (19.0, -9.5, 90, "F"),    # sense filter
    "J1":  (-3.2, 13.9, 0, "F"),     # JST battery, rear-centre, opening rear
"J5":  (-18.6, -9.5, 90, "F"),   # pigtail pads, left edge
    "C1":  (-10.5, 15.8, 0, "B"),    # bulk cap, rear-left belly
    "J2":  (-7.4, 7.0, 270, "B"),    # ToF header row, belly; breakout lies
                                     # flat rearward (battery is belly-front)
    "R13": (10.75, -10.9, 0, "F"),   # LED1 series R
    "R14": (11.5, 19.0, 180, "F"),   # LED2 series R
    "LED1": _radial(2, 34.2, 2.8) + (_arm_rot(2), "F"),   # front-right arm
    "LED2": (7.5, 19.5, 0, "F"),     # rear plate edge, red
    **{f"R{i}": _radial(i, 19.2) + (_arm_rot(i), "F") for i in (1, 2, 3, 4)},
    **{f"R{i+4}": _radial(i, 18.65, -2.55) + (_arm_rot(i), "F")
       for i in (1, 2, 3, 4)},
    **{f"Q{i}": _radial(i, 23.2) + (_arm_rot(i), "F") for i in (1, 2, 3, 4)},
    **{f"D{i}": _radial(i, 31.0) + (_arm_rot(i), "F") for i in (1, 2, 3, 4)},
    **{f"MT{i}": _radial(i, 39.5) + (_arm_rot(i) + 90, "F") for i in (1, 2, 3, 4)},
    # VBAT ceramic bypass, one per motor stage, on the BELLY directly
    # opposite the flyback diode (radial 29 = D's VBAT pad). This is where
    # the commutation edge actually happens: the low-side FET chops the
    # pack current 0 -> 2.75A in tens of ns and the loop closes through D.
    # Belly placement puts the GND pad straight into the B.Cu pour with no
    # return routing, and the VBAT side is tied explicitly to D pad 1
    # rather than relying on pour continuity along the arm.
    **{f"C{i+9}": _radial(i, 29.0, 3.4) + (_arm_rot(i), "B")
       for i in (1, 2, 3, 4)},
}

MOUNT_HOLES = [  # (x, y, footprint)
    (-16.3, 6.0, "Strap_Hole_3.2"),            # rear battery strap pair
    (14.6, 2.5, "Strap_Hole_3.2"),             # (front of pack: velcro pad)
]
# No camera-bracket screw holes: every position that suited a bracket sat on
# top of a motor-drive stage or its ground vias, and metal M2 hardware there
# would bridge the VBAT pour (head) to the GND pour (nut) through the board.
# The camera mounts on a foam/printed clip gripping the front edge instead —
# standard practice at this scale, lighter, and non-conductive.
# motor-retention zip-tie holes: pair flanking each motor, perpendicular to
# the arm (tie loops over the motor can — press-fit backup)
for _i, (_mx, _my) in MOTOR_POS.items():
    _n = math.hypot(_mx, _my)
    _px, _py = -_my / _n, _mx / _n                       # perpendicular unit
    MOUNT_HOLES.append((_mx + TIE_HOLE_OFFSET * _px, _my + TIE_HOLE_OFFSET * _py,
                        "Strap_Hole_3.2"))
    MOUNT_HOLES.append((_mx - TIE_HOLE_OFFSET * _px, _my - TIE_HOLE_OFFSET * _py,
                        "Strap_Hole_3.2"))


def P(x, y):
    return VECTOR2I(FromMM(x + OX), FromMM(y + OY))


# ------------------------------------------------------------------ outline --
def quadrant_path():
    """CCW outline path for quadrant (+x, +y), from (0, PLATE_HALF) to
    (PLATE_HALF, 0), arm + motor-ring arc included. Returns list of (x, y)."""
    c = C45
    m = (MX, MX)
    # arm edge lines: diagonal dir u=(c,c), offsets ±ARM_HALF along p=(-c,c)
    # plate exit points
    p_top = (PLATE_HALF - ARM_HALF / c + ARM_HALF * c, PLATE_HALF)      # on y=PLATE_HALF
    # solved directly: line t*u + a*p crosses y=PLATE_HALF at
    t_top = (PLATE_HALF - ARM_HALF * c) / c
    p_top = (t_top * c - ARM_HALF * c, PLATE_HALF)
    t_right = (PLATE_HALF - ARM_HALF * c) / c
    p_right = (PLATE_HALF, t_right * c - ARM_HALF * c)
    # arm edge / ring intersections (near side), from |Q-M|=RING_OUT
    s = math.sqrt(RING_OUT ** 2 / (2 * c * c) - ARM_HALF ** 2)   # along-diag half-chord
    t_hit = MOTOR_R - s
    q_a = ((t_hit) * c - ARM_HALF * c, (t_hit) * c + ARM_HALF * c)   # +offset edge
    q_b = ((t_hit) * c + ARM_HALF * c, (t_hit) * c - ARM_HALF * c)   # -offset edge
    # arc from q_a around the OUTSIDE (through the tip) to q_b
    a1 = math.degrees(math.atan2(q_a[1] - m[1], q_a[0] - m[0]))
    a2 = math.degrees(math.atan2(q_b[1] - m[1], q_b[0] - m[0]))
    # sweep CCW-negative (clockwise in math coords) from a1 down to a2 passes
    # through 45° (the tip pointing away from origin) — verify and build
    def norm(a):
        while a < 0:
            a += 360
        return a % 360
    a1n, a2n = norm(a1), norm(a2)
    # go from a1 to a2 the long way that includes 45°
    pts = [p_top, q_a]
    # choose direction: try decreasing from a1n, wrapping, until a2n
    sweep_ccw = (a2n - a1n) % 360
    sweep_cw = (a1n - a2n) % 360
    mid_ccw = norm(a1n + sweep_ccw / 2)
    use_ccw = min(abs(mid_ccw - 45), 360 - abs(mid_ccw - 45)) < 90
    sweep = sweep_ccw if use_ccw else -sweep_cw
    n = max(8, int(abs(sweep) / ARC_STEP_DEG))
    for k in range(1, n + 1):
        a = math.radians(a1n + sweep * k / n)
        pts.append((m[0] + RING_OUT * math.cos(a), m[1] + RING_OUT * math.sin(a)))
    pts += [q_b, p_right]
    return pts


def outline_points():
    q = quadrant_path()                     # (+x,+y) quadrant, top edge → right edge
    # full CCW loop in math coords (y up): quadrant reflections
    pth = []
    pth += q                                              # (+,+)
    pth += [(x, -y) for (x, y) in reversed(q)]            # (+,-)
    pth += [(-x, -y) for (x, y) in q]                     # (-,-)
    pth += [(-x, y) for (x, y) in reversed(q)]            # (-,+)
    return pth


def add_outline(board):
    pts = outline_points()
    layer = pcbnew.Edge_Cuts
    for i in range(len(pts)):
        x1, y1 = pts[i]
        x2, y2 = pts[(i + 1) % len(pts)]
        if abs(x1 - x2) < 1e-9 and abs(y1 - y2) < 1e-9:
            continue
        seg = pcbnew.PCB_SHAPE(board, pcbnew.SHAPE_T_SEGMENT)
        seg.SetStart(P(x1, y1))
        seg.SetEnd(P(x2, y2))
        seg.SetLayer(layer)
        seg.SetWidth(FromMM(0.1))
        board.Add(seg)
    # motor holes: circles on Edge.Cuts
    for i, (mx, my) in MOTOR_POS.items():
        c = pcbnew.PCB_SHAPE(board, pcbnew.SHAPE_T_CIRCLE)
        c.SetCenter(P(mx, my))
        c.SetEnd(P(mx + MOTOR_HOLE_D / 2, my))
        c.SetLayer(layer)
        c.SetWidth(FromMM(0.1))
        board.Add(c)


# ---------------------------------------------------------------- build up --
def load_footprint(comp):
    lib = comp["lib"]
    name = comp["footprint"]
    if lib is None:
        path = os.path.join(HW, "pursuit.pretty")
    else:
        path = os.path.join(KFP, lib + ".pretty")
    fp = pcbnew.FootprintLoad(path, name)
    assert fp, f"footprint {name} not found in {path}"
    return fp


def build():
    assert MOTOR_HOLE_D, "MOTOR_HOLE_D pending research"
    board = pcbnew.CreateEmptyBoard() if hasattr(pcbnew, "CreateEmptyBoard") \
        else pcbnew.BOARD()

    nets = {}
    for name in design.NETS:
        ni = pcbnew.NETINFO_ITEM(board, name)
        board.Add(ni)
        nets[name] = ni

    bds = board.GetDesignSettings()
    bds.m_CopperEdgeClearance = FromMM(FAB["edge_clearance"])
    bds.m_TrackMinWidth = FromMM(0.15)
    bds.m_ViasMinSize = FromMM(0.5)
    bds.m_MinThroughDrill = FromMM(0.3)

    placed = {}
    for ref, comp in design.COMPONENTS.items():
        assert comp["pins"] != design.PENDING, f"{ref} PENDING"
        assert ref in PLACE, f"no placement for {ref}"
        fp = load_footprint(comp)
        fp.SetReference(ref)
        fp.SetValue(comp["value"])
        x, y, rot, side = PLACE[ref]
        board.Add(fp)
        if side == "B":
            fp.Flip(P(x, y), False)
        fp.SetPosition(P(x, y))
        fp.SetOrientationDegrees(rot)
        for pad in fp.Pads():
            net = comp["pins"].get(pad.GetNumber())
            if net:
                pad.SetNet(nets[net])
        # Battery-current terminals stay solid-connected to the pour: these
        # carry up to ~10A and a 4-spoke relief would be the bottleneck.
        # Everything else keeps thermal relief so it can be hand-soldered.
        # MT/J1/J5 carry battery current. U1 and J4 are here for a
        # different and less happy reason: their annulus (1.8-2.2mm pad on a
        # 1.0-1.2mm drill) is too narrow for a thermal spoke to form, so
        # KiCad starves the relief. Solid is the only option that fabricates;
        # budget extra iron dwell on those two.
        if ref in ("MT1", "MT2", "MT3", "MT4", "J1", "J5", "U1", "J4"):
            for pad in fp.Pads():
                pad.SetLocalZoneConnection(pcbnew.ZONE_CONNECTION_FULL)
        # Reference designators live on the Fab layer (assembly-drawing
        # convention): at 2.5mm passive pitch, silk refdes collide with each
        # other and sit over pads. The build aid is docs/assembly_*.svg.
        fab = pcbnew.B_Fab if side == "B" else pcbnew.F_Fab
        fp.Reference().SetLayer(fab)
        fp.Reference().SetTextSize(pcbnew.VECTOR2I(FromMM(0.8), FromMM(0.8)))
        fp.Reference().SetTextThickness(FromMM(0.12))
        fp.Value().SetLayer(fab)
        fp.Value().SetVisible(False)
        placed[ref] = fp

    for i, (x, y, name) in enumerate(MOUNT_HOLES):
        lib = os.path.join(HW, "pursuit.pretty") if name.startswith(("Screw", "Strap")) \
            else os.path.join(KFP, "MountingHole.pretty")
        fp = pcbnew.FootprintLoad(lib, name)
        assert fp, f"hole footprint {name} missing"
        fp.SetReference(f"H{i+1}")
        fp.Reference().SetVisible(False)
        fp.Reference().SetLayer(pcbnew.F_Fab)
        fp.Value().SetVisible(False)
        fp.Value().SetLayer(pcbnew.F_Fab)
        board.Add(fp)
        fp.SetPosition(P(x, y))

    add_outline(board)
    return board, nets, placed


def add_zone(board, net, layer, margin=0.0):
    """Full-board rectangular zone; the fill clips to the outline."""
    z = pcbnew.ZONE(board)
    z.SetLayer(layer)
    z.SetNet(net)
    z.SetLocalClearance(FromMM(0.3))
    z.SetMinThickness(FromMM(0.25))
    # Thermal relief, not solid. Both pours are ~2200mm^2 of 1oz copper; a
    # solid tie makes every hand-soldered plane pad a heat sink that a
    # student's iron cannot wet, which would put cold joints on exactly the
    # pads carrying 10A. Four 0.5mm spokes still pass ~3A. The genuine
    # high-current terminals are overridden back to solid in build().
    z.SetPadConnection(pcbnew.ZONE_CONNECTION_THERMAL)
    # Delete pour fragments that reach no pad. Floating copper is not
    # harmful here but it hides real islands in the audit, and a fragment
    # that DOES hold a pad must be caught rather than buried in noise.
    z.SetIslandRemovalMode(pcbnew.ISLAND_REMOVAL_MODE_ALWAYS)
    z.SetThermalReliefGap(FromMM(0.25))
    z.SetThermalReliefSpokeWidth(FromMM(0.4))
    z.SetIsFilled(False)
    ext = MX + RING_OUT + 1
    pts = [(-ext, -ext), (ext, -ext), (ext, ext), (-ext, ext)]
    chain = pcbnew.SHAPE_LINE_CHAIN()
    for (x, y) in pts:
        chain.Append(P(x, y))
    chain.SetClosed(True)
    z.Outline().AddOutline(chain)
    board.Add(z)
    return z


def _rule_area(board, pts, name, layers, no_tracks):
    z = pcbnew.ZONE(board)
    z.SetIsRuleArea(True)
    z.SetDoNotAllowZoneFills(True)
    z.SetDoNotAllowVias(True)
    z.SetDoNotAllowTracks(no_tracks)
    z.SetDoNotAllowPads(False)
    z.SetDoNotAllowFootprints(False)
    ls = pcbnew.LSET()
    for lyr in layers:
        ls.AddLayer(lyr)
    z.SetLayerSet(ls)
    z.SetZoneName(name)
    chain = pcbnew.SHAPE_LINE_CHAIN()
    for (x, y) in pts:
        chain.Append(P(x, y))
    chain.SetClosed(True)
    z.Outline().AddOutline(chain)
    board.Add(z)


def add_keepout(board, x1, y1, x2, y2, name, layers=None, no_tracks=False):
    """Rectangular no-copper rule area."""
    if layers is None:
        layers = (pcbnew.F_Cu, pcbnew.B_Cu)
    _rule_area(board, [(x1, y1), (x2, y1), (x2, y2), (x1, y2)],
               name, layers, no_tracks)


def add_round_keepout(board, cx, cy, r, name, layers=None, no_tracks=False):
    """Circular no-copper rule area — used to hold both pours away from the
    metal motor cans and the M2 camera-bracket hardware, either of which
    otherwise bridges VBAT (top) to GND (bottom) through the board."""
    if layers is None:
        layers = (pcbnew.F_Cu, pcbnew.B_Cu)
    n = 32
    pts = [(cx + r * math.cos(2 * math.pi * k / n),
            cy + r * math.sin(2 * math.pi * k / n)) for k in range(n)]
    _rule_area(board, pts, name, layers, no_tracks)


def add_silk(board):
    """Silkscreen: functional markings + brand identity.

    Type hierarchy — wordmark 2.2 / motor IDs 1.8 / brand 1.5 / warnings 1.2
    / labels 1.0. Reference designators are NOT here (they live on the Fab
    layer); see docs/assembly_*.svg.
    """
    def text(s, x, y, layer=pcbnew.F_SilkS, size=1.2, angle=0):
        t = pcbnew.PCB_TEXT(board)
        t.SetText(s)
        t.SetPosition(P(x, y))
        t.SetLayer(layer)
        t.SetTextSize(pcbnew.VECTOR2I(FromMM(size), FromMM(size)))
        t.SetTextThickness(FromMM(max(0.15, size * 0.15)))
        t.SetTextAngleDegrees(angle)
        if layer == pcbnew.B_SilkS:
            t.SetMirrored(True)
        board.Add(t)

    def line(x1, y1, x2, y2, layer=pcbnew.F_SilkS, width=0.25):
        g = pcbnew.PCB_SHAPE(board, pcbnew.SHAPE_T_SEGMENT)
        g.SetStart(P(x1, y1))
        g.SetEnd(P(x2, y2))
        g.SetLayer(layer)
        g.SetWidth(FromMM(width))
        board.Add(g)

    def arm_text(s, i, radial, perp, size, layer=pcbnew.F_SilkS):
        """Text running along arm i. radial/perp are arm-local coords.

        Arms 1&4 and 2&3 are collinear, so half of them would render
        upside-down at the raw arm angle; rotate those 180 deg so every
        label reads the same way round the airframe.
        """
        sx, sy = SIGN[i]
        ux, uy = sx * C45, sy * C45
        px, py = -uy, ux
        x, y = radial * ux + perp * px, radial * uy + perp * py
        ang = _arm_rot(i)
        if ang > 90 or ang <= -90:
            ang += 180
        text(s, x, y, layer=layer, size=size, angle=ang)

    # ---- nose chevron: orientation cue, and the board's visual signature --
    for w in (0.0, 1.6):
        line(-4.0 + w * 0.6, -18.85, 0.0, -20.55 + w * 0.33,
             layer=pcbnew.B_SilkS, width=0.3)
        line(4.0 - w * 0.6, -18.85, 0.0, -20.55 + w * 0.33,
             layer=pcbnew.B_SilkS, width=0.3)

    # ---- brand, on diagonally opposite arms (front-left / rear-right) -----
    arm_text("PURSUIT-1", 1, 30.0, -3.9, 1.5)
    arm_text("Designed by Pai", 4, 30.0, 3.9, 1.0)

    # ---- motor IDs + rotation, belly, on each arm centreline -------------
    for i in (1, 2, 3, 4):
        arm_text(f"M{i}", i, 30.5, 0.0, 1.8, layer=pcbnew.B_SilkS)
        arm_text("CW" if i in (1, 4) else "CCW", i, 35.0, 0.0, 1.1,
                 layer=pcbnew.B_SilkS)

    # ---- belly wordmark + safety -----------------------------------------
    text("PURSUIT-1", 2.5, 19.2, layer=pcbnew.B_SilkS, size=2.2)
    text("BAT: CHECK POLARITY", 5.5, 16.2, layer=pcbnew.B_SilkS, size=1.2)
    text("v1.0", 13.0, 19.3, layer=pcbnew.B_SilkS, size=1.0)

    # ---- polarity markings, each one BESIDE ITS OWN PART, ON ITS OWN FACE --
    # A free-floating "PIN1=+" used to sit at (-9.0, 19.6). That is 5.8mm off
    # J1, and it lands over C1's belly outline on the pad-2 (GND) side — so
    # whichever part you read it as, it points at the wrong terminal. Both
    # markers below are glyphs placed at the + pad of the part they belong to.
    # "J1+", not a bare "+": a lone glyph here measures 2.95mm to C4 pad 1
    # and 3.20mm to J1 pad 1, i.e. marginally nearer the wrong part. The
    # designator makes it self-identifying regardless of distance.
    text("J1+", -6.7, 13.9, size=1.0)               # F.SilkS, at J1 pad 1
    # C1 is a polymer/tantalum bulk cap directly across an unfused 1S LiPo,
    # hand-fitted on the BELLY, so its marker must be on B.SilkS (mirrored)
    # and on the pad-1 side. Its footprint silk bar is also at this end.
    text("+ C1", -18.0, 15.8, layer=pcbnew.B_SilkS, size=1.1)
    # ---- functional labels ------------------------------------------------
    text("ToF", -12.0, 4.4, layer=pcbnew.B_SilkS, size=1.0)
    text("P1", -9.9, 7.0, layer=pcbnew.B_SilkS, size=1.0)
    text("ANT", -5.0, 0.5, size=1.0)


def enforce_silk_minimums(board):
    """Raise every silk stroke to JLCPCB's 0.153mm floor.

    Our own emitters already clamp, but the STOCK KiCad footprints do not:
    127 of the 141 silk graphics on this board arrive at 0.12mm, including
    C1's polarity bar, all four diode cathode bars, the LED cathode marks and
    U2's pin-1 dot — i.e. precisely the marks a hand-builder reads. Below the
    floor JLCPCB's own DFM either thins them to the edge of legibility or
    drops them. VERIFICATION.md certified this as fixed for two rounds while
    it was only ever true of the text.
    """
    floor = FromMM(0.15)
    n = 0
    silk = (pcbnew.F_SilkS, pcbnew.B_SilkS)
    for it in list(board.GetDrawings()):
        if it.GetLayer() in silk and hasattr(it, "GetWidth"):
            if it.GetWidth() < floor:
                it.SetWidth(floor)
                n += 1
    for fp in board.GetFootprints():
        for it in fp.GraphicalItems():
            if it.GetLayer() in silk and hasattr(it, "GetWidth"):
                if it.GetWidth() < floor:
                    it.SetWidth(floor)
                    n += 1
    print(f"silk: raised {n} stroke(s) to 0.15mm")


def fill_zones(board):
    filler = pcbnew.ZONE_FILLER(board)
    filler.Fill(board.Zones())


def main():
    board, nets, placed = build()
    add_zone(board, nets["VBAT"], pcbnew.F_Cu)
    add_zone(board, nets["GND"], pcbnew.B_Cu)
    # XIAO U.FL antenna corner (module rear-left) — keep copper clear
    add_keepout(board, -9.5, -2.5, -0.5, 3.0, "xiao-antenna")
    # Metal motor cans and M2 camera hardware span the full board thickness
    # and would otherwise sit 0.4mm from live copper on BOTH faces.
    for _i, (_mx, _my) in MOTOR_POS.items():
        add_round_keepout(board, _mx, _my, MOTOR_CU_KEEPOUT_R, f"motor{_i}-cu")
    # NOTE: no copper keepout at the camera holes. Sterilising copper there
    # would push the Q1 via cluster and R6's ground via off the arm corner,
    # and the cleaner fix for "metal M2 hardware bridges VBAT to GND" is to
    # not use metal hardware: the BOM specifies NYLON M2 screws and nuts,
    # which are also lighter and are standard for micro-drone camera mounts.
    # no copper/vias under the IMU body on any layer (TDK layout rule)
    # IMU body: F.Cu ONLY. Previously this spanned both layers, which
    # punched a 3.6x3.2mm hole in the B.Cu ground pour 1.6mm below the
    # part — where copper is mechanically irrelevant and actively wanted as
    # the sensor's reference and heat spreader. Restricting it to F.Cu
    # keeps the pour off the mounting layer (no copper plateau under the
    # LGA) and gives the IMU a solid ground plane again.
    # Tracks stay permitted: the AP_CS->VDDIO tie has no legal path around
    # the package (pin 13/SCL owns the only escape corridor north of pin
    # 12), so it is an accepted, documented deviation — see VERIFICATION.md.
    add_keepout(board, -1.8, 8.6, 1.8, 11.8, "imu-body",
                layers=(pcbnew.F_Cu,), no_tracks=False)
    add_silk(board)
    enforce_silk_minimums(board)
    import routes
    routes.add_routes(board, nets, placed, P, FromMM, FAB)
    # zone fill happens out-of-process: the in-process ZONE_FILLER segfaults
    # headless; `kicad-cli pcb drc --refill-zones --save-board` fills instead
    out = os.path.join(HW, "pursuit.kicad_pcb")
    pcbnew.SaveBoard(out, board)
    print("wrote", out)
    # pad position dump for the routing table + audits
    for ref in sorted(placed):
        fp = placed[ref]
        for pad in fp.Pads():
            p = pad.GetPosition()
            print(f"PAD {ref}.{pad.GetNumber()} {pcbnew.ToMM(p.x)-OX:.2f} "
                  f"{pcbnew.ToMM(p.y)-OY:.2f} net={pad.GetNetname()}")


if __name__ == "__main__":
    main()
