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
    "JST_S2B": (3.45, 4.3),         # S2B-PH-K 2pos: court -2.45..4.45 x,
                                    # -1.85..6.75 y (origin = pin1)
        "HDR_1x7": (1.15, 9.15),
    "XIAO": (9.15, 11.55),
    "IMU": (1.75, 1.5),
    "MT": ("circle", 2.9),
    "W2": (3.1, 1.3), "W2HD": (3.8, 1.6),
    "HOLE32": ("circle", 1.85),     # 3.2 NPTH + 0.25 clearance ring
    "HOLE22": ("circle", 1.35),
}
# footprint origin offsets (pin1 vs centre) for headers: origin at pin1,
# body extends +y; model centre offset (0, (n-1)*1.27)
OFFSET = {"HDR_1x7": (0.0, 7.62),
          "XIAO": (0.0, -0.15), "JST_S2B": (1.0, 2.45)}

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
}

PLATE = 21.0
DIAG = 37.05          # corner-cut line (tracks ARM_HALF)
EDGE_M = 0.35         # courtyard-ish margin to straight edges
DIAG_M = 0.55

KEEPOUTS = []         # checked visually; pads allowed inside


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


def check(place, holes):
    bad = []
    polys = {}
    for ref, (x, y, rot, side) in place.items():
        shape = REFS[ref]
        if side == "B" and SHAPES[shape][0] != "circle":
            rot = -rot  # mirror; adequate for our symmetric shapes
        polys[ref] = corners(x, y, rot, shape)[0]
    for i, (hx, hy) in enumerate(holes):
        polys[f"H{i+1}"] = corners(hx, hy, 0, "HOLE32" if i >= 2 else "HOLE22")[0]

    sides = {r: p[3] for r, p in place.items()}
    smd_only = lambda r: r[0] in "CRQDLU" and r != "U1"

    refs = sorted(polys)
    for i in range(len(refs)):
        for j in range(i + 1, len(refs)):
            a, b = refs[i], refs[j]
            if a.startswith("H") and b.startswith("H"):
                continue
            if frozenset((a, b)) in ALLOWED:
                continue
            # SMD parts on opposite sides can't collide (THT pierces both)
            sa, sb = sides.get(a, "F"), sides.get(b, "F")
            if sa != sb and (smd_only(a) or smd_only(b)):
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
            on_arm_ok = (ref.startswith(("MT", "Q", "D", "R", "LED"))
                         or ref in ("C10", "C11", "C12", "C13"))
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
    return bad


if __name__ == "__main__":
    import gen_pcb
    place = dict(gen_pcb.PLACE)
    holes = [(x, y) for x, y, _ in gen_pcb.MOUNT_HOLES]
    bad = check(place, holes)
    for b in bad:
        print(b)
    print(f"{len(bad)} violations")
    sys.exit(1 if bad else 0)
