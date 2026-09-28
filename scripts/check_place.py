"""Fast placement-collision checker for PURSUIT-1 — models courtyards as
polygons and checks pairwise overlap, board-edge margins (incl. the corner
cuts), keepouts, and NPTH hole clearance, without invoking KiCad.

Run: python3 check_place.py   -> prints violations, exits 1 if any.
"""
import math
import sys

# half-extents (x, y) at rot=0 INCLUDING courtyard margin, or ('circle', r)
SHAPES = {
    "R_0603": (1.05, 0.72), "C_0603": (1.05, 0.72), "C_0805": (1.25, 0.85),
    "C_1206": (1.9, 1.05), "SOT23": (1.75, 1.7), "SMA": (3.45, 1.65),
    "CP63": (4.4, 2.9),             # 7343/D polymer land
    "LED_0805": (1.25, 0.85),
    # rev C: JST-SH 4P SMD. READ OUT OF THE FOOTPRINT'S F.CrtYd layer, not
    # estimated from the pad span -- the first attempt used (3.60, 2.98) from
    # pad extents and missed a real overlap with R5 that DRC then found. The
    # shroud is bigger than the pads. Origin is the body centre.
    "JST_SH4": (3.90, 3.28),
    "JST_S2B": (3.45, 4.3),         # S2B-PH-K 2pos: court -2.45..4.45 x,
                                    # -1.85..6.75 y (origin = pin1)
    # rev C: the SMD S2B-PH-SM4-TB. Pads span 8.2 x 9.2mm (two signal + two
    # mounting tabs), origin at the body centre, so no OFFSET entry.
    "JST_S2B_SMD": (4.35, 4.85),
        "HDR_1x7": (1.15, 9.15),
    "XIAO": (9.15, 11.55),          # rev B: oval along the row only
    "IMU": (1.75, 1.5),
    "MT": ("circle", 3.4),          # rev B: pads offset 0.3 outboard
    # rev B: pads grew OUTBOARD only (drill offset), so the inner edge is
    # unchanged and only these outer half-extents move.
    "W2": (3.1, 1.3), "W2HD": (4.95, 1.85),
    "TP": ("circle", 1.25),         # 2.0mm pad + courtyard
    "HOLE32": ("circle", 1.85),     # 3.2 NPTH + 0.25 clearance ring
    "HOLE22": ("circle", 1.35),
}
# footprint origin offsets (pin1 vs centre) for headers: origin at pin1,
# body extends +y; model centre offset (0, (n-1)*1.27)
OFFSET = {"HDR_1x7": (0.0, 7.62),
          "XIAO": (0.0, -0.15), "JST_S2B": (1.0, 2.45)}
# JST_S2B_SMD deliberately has NO offset entry: its footprint origin is the
# body centre. The THT part's was pin 1, which is why the swap needed the
# PLACE coordinate moving by exactly that offset.

# physically-intended overlaps (screw heads under the elevated module, the
# BAT wire pads tucked at the module rear edge)
ALLOWED = {frozenset(p) for p in
           [("H1", "U1"), ("H2", "U1"), ("J4", "U1")]}

REFS = {
    "U1": "XIAO", "U2": "IMU", "J1": "JST_S2B", "J2": "HDR_1x7",
    "J4": "W2", "J5": "W2HD", "C1": "CP63",
    "C2": "C_0603", "C3": "C_0603", "C4": "C_0603", "C6": "C_0805",
    "C7": "C_1206", "C8": "C_0603",
    "LED1": "LED_0805", "LED2": "LED_0805",
    **{f"Q{i}": "SOT23" for i in (1, 2, 3, 4)},
    **{f"D{i}": "SMA" for i in (1, 2, 3, 4)},
    **{f"R{i}": "R_0603" for i in list(range(1, 15)) if i != 15},
    **{f"C{i+9}": "C_0603" for i in (1, 2, 3, 4)},
    **{f"MT{i}": "MT" for i in (1, 2, 3, 4)},
    "TP1": "TP", "TP2": "TP", "TP3": "TP",
}

PLATE = 21.0
DIAG = 37.05          # corner-cut line (tracks ARM_HALF)
EDGE_M = 0.35         # courtyard-ish margin to straight edges
DIAG_M = 0.55

KEEPOUTS = []         # checked visually; pads allowed inside

# Which refdes are THROUGH-HOLE. A THT part pierces both faces, so it can
# collide with anything on either side; an SMD part cannot collide with an SMD
# part on the far side. This USED to be guessed from the refdes prefix
# (`r[0] in "CRQDLU" and r != "U1"`), which is wrong the moment a through-hole
# part is named with an S, T or J -- and rev C adds a shunt RS1, probe pads
# TP4-TP8 and a flow connector J6. Guessing here does not fail loudly; it
# silently skips a collision test.
THT = {"U1", "J1", "J2", "J4", "J5", "J6",
       "MT1", "MT2", "MT3", "MT4",
       "TP1", "TP2", "TP3", "TP4", "TP5", "TP6", "TP7", "TP8"}

# Parts that legitimately sit out on the arms rather than on the centre plate,
# and are therefore exempt from the plate-edge rule. Also previously a prefix
# test -- `startswith(("MT","Q","D","R","LED"))` -- which exempted EVERY part
# whose name starts with R, including plate-mounted ones, and would exempt a
# shunt called RS1 from the board-edge check entirely.
# PACKAGE BODY half-extents, in mm. Distinct from SHAPES, which carries
# COURTYARDS -- a courtyard is a keep-clear zone, but JLCPCB's rule is about
# where the physical component body sits relative to the board edge, and the
# two differ by up to a millimetre. Only machine-placed SMD packages appear
# here; hand-fitted parts are exempt and listed in HAND_FITTED.
BODIES = {
    "R_0603": (0.80, 0.40), "C_0603": (0.80, 0.40),
    "C_0805": (1.00, 0.63), "LED_0805": (1.00, 0.63),
    "C_1206": (1.60, 0.80),
    "SOT23": (1.45, 0.65),          # AO3400A body, leads excluded
    "JST_SH4": (3.0, 2.25),         # SM04B-SRSS-TB body incl. shroud
    "JST_S2B_SMD": (3.0, 3.5),      # S2B-PH-SM4-TB body, shroud included
    "SMA": (2.30, 1.35),            # DO-214AC body
    "IMU": (1.25, 1.50),            # LGA-14 2.5 x 3.0
}
# Exempt from the component-to-edge rule: JLCPCB does not place these, so
# their proximity to an edge is the builder's problem, not a yield risk.
HAND_FITTED = {"U1", "J1", "J2", "J4", "J5", "J6", "C1", "C6", "C7",
               "MT1", "MT2", "MT3", "MT4",
               "TP1", "TP2", "TP3", "TP4", "TP5", "TP6", "TP7", "TP8"}

# JLCPCB: components must clear the board edge by 2.5mm for their assembly
# yield commitment to apply, and depanelling tolerance is +-0.2mm so anything
# under 0.3mm is at risk of being routed through.
EDGE_FAIL_MM = 0.30
EDGE_WARN_MM = 2.50

# Placements allowed on the BACK of the board. JLCPCB's Economic PCBA has no
# double-sided option AT ANY PRICE -- one belly part moves the whole order to
# Standard, which is roughly $58 of fixed fees, about ten times what the two
# extra copper layers cost. So this list may only ever SHRINK. Seeded with rev
# B's actual offenders so the gate starts honest rather than green.
# rev C: C1 and J2 remain on the belly, both blocked on the re-placement
# pass. This list may only ever SHRINK.
BACK_SIDE_ALLOWED = {"C1", "J2"}

ON_ARM = ({f"MT{i}" for i in (1, 2, 3, 4)} |
          {f"Q{i}" for i in (1, 2, 3, 4)} |
          {f"D{i}" for i in (1, 2, 3, 4)} |
          {f"R{i}" for i in range(1, 9)} |
          {"R13", "R14", "LED1", "LED2"} |
          {"C10", "C11", "C12", "C13"})


def corners(x, y, rot, shape):
    s = SHAPES[shape]
    if s[0] == "circle":
        r = s[1]
        return [(x + r * math.cos(a), y + r * math.sin(a))
                for a in [k * math.pi / 8 for k in range(16)]], None
    hx, hy = s
    ox, oy = OFFSET.get(shape, (0.0, 0.0))
    rad = math.radians(rot)
    c, sn = math.cos(rad), math.sin(rad)
    pts = []
    for px, py in [(-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy)]:
        qx, qy = px + ox, py + oy
        # KiCad rotation is CCW in board coords (y down): screen effect
        rx = qx * c + qy * sn
        ry = -qx * sn + qy * c
        pts.append((x + rx, y + ry))
    return pts, None


def _seg_dist(px, py, x1, y1, x2, y2):
    dx, dy = x2 - x1, y2 - y1
    l2 = dx * dx + dy * dy
    t = 0.0 if l2 == 0 else max(0.0, min(1.0, ((px - x1) * dx + (py - y1) * dy) / l2))
    return math.hypot(px - (x1 + t * dx), py - (y1 + t * dy))


def dist_to_outline(px, py, outline, motor_rings):
    """Shortest distance from a point to the real Edge.Cuts loop.

    The existing EDGE check models the board as PLATE/DIAG constants. That is a
    MODEL of the outline, maintained by hand, and it has already drifted from
    the generator: README and VERIFICATION both claim D1-D4 sit 0.7mm from an
    edge and LED1 0.5mm, and both numbers are wrong -- measured against the
    real outline they are 4.70mm and 2.58mm. Measuring the actual polyline is
    the only way that claim stops being folklore."""
    d = min(_seg_dist(px, py, outline[i][0], outline[i][1],
                      outline[(i + 1) % len(outline)][0],
                      outline[(i + 1) % len(outline)][1])
            for i in range(len(outline)))
    for (mx, my, r) in motor_rings:      # the prop rings are edges too
        d = min(d, abs(math.hypot(px - mx, py - my) - r))
    return d


def poly_overlap(p1, p2):
    """Separating-axis test for convex polygons."""
    for poly_a, poly_b in ((p1, p2), (p2, p1)):
        n = len(poly_a)
        for i in range(n):
            ax, ay = poly_a[i]
            bx, by = poly_a[(i + 1) % n]
            nx, ny = -(by - ay), (bx - ax)
            amin = min(px * nx + py * ny for px, py in poly_a)
            amax = max(px * nx + py * ny for px, py in poly_a)
            bmin = min(px * nx + py * ny for px, py in poly_b)
            bmax = max(px * nx + py * ny for px, py in poly_b)
            if amax < bmin or bmax < amin:
                return False
    return True


def check(place, holes, outline=None, rings=(), warn=None):
    bad = []
    if warn is None:
        warn = []
    polys = {}
    for ref, (x, y, rot, side) in place.items():
        shape = REFS.get(ref)
        if shape is None:
            bad.append(f"NO-SHAPE {ref} is placed but has no entry in REFS -- "
                       f"it was never collision-checked")
            continue
        if side == "B" and SHAPES[shape][0] != "circle":
            rot = -rot  # mirror; adequate for our symmetric shapes
        polys[ref] = corners(x, y, rot, shape)[0]
    # Size each hole from its own footprint name. This was `"HOLE32" if i >= 2
    # else "HOLE22"`, sizing by POSITION IN THE LIST -- but every MOUNT_HOLES
    # entry is a Strap_Hole_3.2, so the two pack-strap holes were modelled at
    # 2.2mm, a millimetre undersize, for as long as the checker has existed.
    HOLE_SHAPE = {"Strap_Hole_3.2": "HOLE32", "Screw_Hole_2.2": "HOLE22"}
    for i, h in enumerate(holes):
        hx, hy = h[0], h[1]
        fp = h[2] if len(h) > 2 else None
        shape = HOLE_SHAPE.get(fp)
        assert shape, f"hole {i+1} has unknown footprint {fp!r} -- add it to " \
                      f"HOLE_SHAPE rather than letting it default"
        polys[f"H{i+1}"] = corners(hx, hy, 0, shape)[0]

    sides = {r: p[3] for r, p in place.items()}
    # A bare hole pierces the board, so it is THT for this purpose. Previously
    # holes carried no side at all and `sides.get(a, "F")` defaulted them to
    # front, so a 3.2mm strap hole was never checked against a belly part.
    is_tht = lambda r: r in THT or r.startswith("H")

    refs = sorted(polys)
    for i in range(len(refs)):
        for j in range(i + 1, len(refs)):
            a, b = refs[i], refs[j]
            if a.startswith("H") and b.startswith("H"):
                continue
            if frozenset((a, b)) in ALLOWED:
                continue
            # SMD parts on opposite sides cannot collide. A THT part on
            # either side can collide with anything, because it pierces.
            sa, sb = sides.get(a, "F"), sides.get(b, "F")
            if sa != sb and not is_tht(a) and not is_tht(b):
                continue
            if poly_overlap(polys[a], polys[b]):
                bad.append(f"OVERLAP {a} <-> {b}")
    for ref, pts in polys.items():
        if ref.startswith("H") and not ref.startswith("HDR"):
            continue          # bare holes may sit anywhere incl. motor rings
        for (px, py) in pts:
            if abs(px) > PLATE - EDGE_M and abs(px) < PLATE + 4 \
                    and abs(py) < PLATE + 4 and abs(py) > PLATE - EDGE_M:
                pass
            # parts that legitimately sit out on the arms rather than the plate
            on_arm_ok = ref in ON_ARM
            if (abs(px) > PLATE - EDGE_M or abs(py) > PLATE - EDGE_M) \
                    and not on_arm_ok:
                bad.append(f"EDGE {ref} corner ({px:.1f},{py:.1f})")
                break
            if abs(px) + abs(py) > DIAG - DIAG_M and \
                    not (ref.startswith("MT") or ref.startswith("HOLE")):
                r = math.hypot(px, py)
                # on-arm parts are fine while inside the 10mm arm corridor
                d_perp = abs(abs(px) - abs(py)) / math.sqrt(2)
                if d_perp > 6.0 - 0.55:
                    bad.append(f"ARM-EDGE {ref} ({px:.1f},{py:.1f}) perp={d_perp:.2f}")
                    break
    # ---- rev C gate: nothing on the back -------------------------------
    for ref, (x, y, rot, side) in sorted(place.items()):
        if side == "B" and ref not in BACK_SIDE_ALLOWED:
            bad.append(f"BACK-SIDE {ref} is placed on B.Cu -- Economic PCBA "
                       f"is single-sided only; add to BACK_SIDE_ALLOWED only "
                       f"if you mean to pay for Standard")

    # ---- rev C gate: component body to board edge -----------------------
    if outline:
        for ref, (x, y, rot, side) in sorted(place.items()):
            if ref in HAND_FITTED:
                continue
            body = BODIES.get(REFS.get(ref))
            if body is None:
                continue
            hx, hy = body
            rad = math.radians(-rot if side == "B" else rot)
            c, sn = math.cos(rad), math.sin(rad)
            worst = 1e9
            for bx, by in ((-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy)):
                cx = x + bx * c + by * sn
                cy = y - bx * sn + by * c
                worst = min(worst, dist_to_outline(cx, cy, outline, rings))
            if worst < EDGE_FAIL_MM:
                bad.append(f"EDGE-FAIL {ref} body {worst:.2f}mm from the "
                           f"outline (<{EDGE_FAIL_MM}mm, depanel risk)")
            elif worst < EDGE_WARN_MM:
                warn.append(f"EDGE-WARN {ref} body {worst:.2f}mm from the "
                            f"outline (JLCPCB wants {EDGE_WARN_MM}mm for "
                            f"their yield commitment)")
    return bad


if __name__ == "__main__":
    import gen_pcb
    place = dict(gen_pcb.PLACE)
    holes = list(gen_pcb.MOUNT_HOLES)     # (x, y, footprint) — the name matters
    outline = gen_pcb.outline_points()
    rings = [(mx, my, gen_pcb.RING_OUT)
             for mx, my in gen_pcb.MOTOR_POS.values()]
    warn = []
    bad = check(place, holes, outline, rings, warn)
    for b in bad:
        print(b)
    for w in sorted(warn):
        print(w)
    print(f"{len(bad)} violations, {len(warn)} warnings")
    sys.exit(1 if bad else 0)
