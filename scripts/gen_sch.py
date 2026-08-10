"""Generate hardware/pursuit.kicad_sch from design.py — as a real schematic.

The previous version dropped every symbol on a 5-column grid and hung a global
label off every pin. That is a netlist rendered as wallpaper: nothing is wired,
nothing is grouped, and you cannot read the circuit off it.

This one draws functional blocks with real wires, local power symbols, and
signal flow left to right: power in on the left, the controller and sensors in
the middle, the four motor drive stages on the right. Cross-block signals use
net labels; supplies use power symbols rather than labels on every pin.

Placement geometry is not guessed. KiCad's symbol transform was measured by
generating probe sheets and reading back `kicad-cli sch export netlist`:

    angle 0    dx =  px   dy = -py        mirror x   dx =  px   dy =  py
    angle 90   dx = -py   dy = -px        mirror y   dx = -px   dy = -py
    angle 180  dx = -px   dy =  py
    angle 270  dx =  py   dy =  px

(mirror is applied to the library coordinate first, then the rotation.)
`scripts/check_sch.py` re-derives the netlist from the emitted file and diffs
it against design.py, so a wire that misses a pin by 0.01mm is a hard failure
rather than a drawing that merely looks plausible.

Run with the KiCad-bundled or system python3 (pure text emission, no pcbnew).
"""
import os
import re
import sys
import uuid

sys.path.insert(0, os.path.dirname(__file__))
import design

NS = uuid.UUID("6ba7b810-9dad-11d1-80b4-00c04fd430c8")
KLIB = "/Applications/KiCad/KiCad.app/Contents/SharedSupport/symbols"
GRID = 1.27


def uid(key):
    return str(uuid.uuid5(NS, "pursuit-sch:" + key))


# ---------------------------------------------------------------- s-expr ---
def tokenize(s):
    return re.findall(r'"(?:[^"\\]|\\.)*"|[()]|[^\s()"]+', s)


def parse(tokens, i=0):
    assert tokens[i] == "("
    out = []
    i += 1
    while tokens[i] != ")":
        if tokens[i] == "(":
            node, i = parse(tokens, i)
            out.append(node)
        else:
            out.append(tokens[i])
            i += 1
    return out, i + 1


def sx_str(node, indent=0):
    pad = "  " * indent
    if isinstance(node, str):
        return node
    parts = [sx_str(n, indent + 1) for n in node]
    flat = "(" + " ".join(parts) + ")"
    if len(flat) < 100 and "\n" not in flat:
        return flat
    head = parts[0]
    body = ("\n" + pad + "  ").join(parts[1:])
    return f"({head} " + "\n" + pad + "  " + body + "\n" + pad + ")"


def load_lib_symbol(libname, symname):
    path = os.path.join(KLIB, libname + ".kicad_sym")
    with open(path) as f:
        tree, _ = parse(tokenize(f.read()))
    for node in tree:
        if isinstance(node, list) and node and node[0] == "symbol" \
                and node[1] == f'"{symname}"':
            assert not any(isinstance(n, list) and n and n[0] == "extends"
                           for n in node), f"{symname} is a derived symbol"
            node = list(node)
            node[1] = f'"{libname}:{symname}"'
            return node
    raise KeyError(f"{symname} not in {libname}")


def symbol_pins(sym):
    """{number: (x, y, angle)} in library coords (Y up)."""
    pins = {}

    def walk(node):
        if isinstance(node, list):
            if node and node[0] == "pin":
                at = next(n for n in node if isinstance(n, list) and n[0] == "at")
                num = next(n for n in node if isinstance(n, list) and n[0] == "number")
                pins[num[1].strip('"')] = (float(at[1]), float(at[2]),
                                           float(at[3]) if len(at) > 3 else 0.0)
            for n in node:
                walk(n)

    walk(sym)
    return pins


# ------------------------------------------------------- authored symbols ---
def box_symbol(name, left_pins, right_pins, width=30.48):
    """left/right: list of (number, pin_name, electrical_type) top -> bottom."""
    half = width / 2
    n = max(len(left_pins), len(right_pins))
    htop = (n - 1) * 2.54 / 2 + 2.54
    L = [f'(symbol "pursuit_sym:{name}" (pin_names (offset 0.762)) '
         f'(exclude_from_sim no) (in_bom yes) (on_board yes)',
         f'  (property "Reference" "U" (at 0 {htop + 3.0:.2f} 0) '
         '(effects (font (size 1.524 1.524)) (justify left)))',
         f'  (property "Value" "{name}" (at 0 {-htop - 3.0:.2f} 0) '
         '(effects (font (size 1.524 1.524)) (justify left)))',
         f'  (symbol "{name}_0_1"',
         f'    (rectangle (start {-half:.2f} {htop:.2f}) (end {half:.2f} '
         f'{-htop:.2f}) (stroke (width 0.254) (type default)) '
         '(fill (type background))))',
         f'  (symbol "{name}_1_1"']
    for side, items, sign, ang in (("L", left_pins, -1, 0), ("R", right_pins, 1, 180)):
        for i, (num, pname, etype) in enumerate(items):
            y = htop - 2.54 * (i + 1)
            L.append(f'    (pin {etype} line (at {sign * (half + 3.81):.2f} '
                     f'{y:.2f} {ang}) (length 3.81) '
                     f'(name "{pname}" (effects (font (size 1.27 1.27)))) '
                     f'(number "{num}" (effects (font (size 1.016 1.016)))))')
    L += ['  )', ')']
    return parse(tokenize("\n".join(L)))[0]


def power_symbol(name, glyph):
    """A power port. Its pin sits at the origin, so rotation only spins the
    graphic — the connection point never moves. That is what makes local
    power symbols cheap to place against any wire end."""
    if glyph == "gnd":
        art = ('(polyline (pts (xy 0 0) (xy 0 -1.27)) (stroke (width 0.254) '
               '(type default)) (fill (type none)))'
               '(polyline (pts (xy -1.905 -1.27) (xy 1.905 -1.27) (xy 0 -3.048) '
               '(xy -1.905 -1.27)) (stroke (width 0.254) (type default)) '
               '(fill (type outline)))')
        prop_y, pin_ang = -4.5, 90
    else:
        art = ('(polyline (pts (xy 0 0) (xy 0 1.27)) (stroke (width 0.254) '
               '(type default)) (fill (type none)))'
               '(polyline (pts (xy -1.905 1.27) (xy 1.905 1.27) (xy 0 3.048) '
               '(xy -1.905 1.27)) (stroke (width 0.254) (type default)) '
               '(fill (type outline)))')
        prop_y, pin_ang = 4.5, 270
    txt = (f'(symbol "pursuit_sym:{name}" (power) (pin_names (offset 0)) '
           f'(exclude_from_sim no) (in_bom no) (on_board yes)'
           f'  (property "Reference" "#PWR" (at 0 -7.62 0) '
           '(effects (font (size 1.27 1.27)) hide))'
           f'  (property "Value" "{name}" (at 0 {prop_y} 0) '
           '(effects (font (size 1.27 1.27))))'
           f'  (symbol "{name}_0_1" {art})'
           f'  (symbol "{name}_1_1" (pin power_in line (at 0 0 {pin_ang}) '
           f'(length 0) hide (name "{name}" (effects (font (size 1.27 1.27)))) '
           '(number "1" (effects (font (size 1.27 1.27))))))'
           ')')
    return parse(tokenize(txt))[0]


# ------------------------------------------------------------------ sheet ---
def xform(px, py, angle, mirror):
    """Library coords -> sheet offset. Measured, not assumed (see module doc)."""
    if mirror == "y":
        px = -px
    elif mirror == "x":
        py = -py
    if angle == 0:
        return px, -py
    if angle == 90:
        return -py, -px
    if angle == 180:
        return -px, py
    if angle == 270:
        return py, px
    raise ValueError(angle)


def snap(v):
    """KiCad wants every pin and wire end on the 1.27mm (50 mil) connection
    grid; off-grid geometry connects electrically but throws a warning per
    item and is a nightmare to edit by hand. Every library pin used here is
    already on grid, so snapping the symbol ORIGINS and the wire waypoints is
    sufficient to put the whole sheet on grid."""
    return round(round(v / GRID) * GRID, 3)


def snap_pt(p):
    return (snap(p[0]), snap(p[1]))


class Sheet:
    def __init__(self):
        self.libs, self.syms = {}, []
        self.wires, self.juncs, self.labels = [], [], []
        self.ncs, self.boxes, self.texts = [], [], []
        self._pwr = 0

    def _lib(self, sym):
        lid = sym[1].strip('"')
        self.libs.setdefault(lid, sym)
        return lid

    def add(self, ref, sym, x, y, angle=0, mirror=None,
            value=None, footprint=None, in_bom=True, hide_val=False,
            rprop=(0, -6.5), vprop=(0, 6.5)):
        lid = self._lib(sym)
        x, y = snap(x), snap(y)
        self.syms.append(dict(ref=ref, lid=lid, x=x, y=y, angle=angle,
                              mirror=mirror, value=value, fp=footprint,
                              pins=symbol_pins(sym), in_bom=in_bom,
                              hide_val=hide_val, rprop=rprop, vprop=vprop))
        return ref

    def pin(self, ref, num):
        s = next(z for z in self.syms if z["ref"] == ref)
        px, py, _ = s["pins"][str(num)]
        dx, dy = xform(px, py, s["angle"], s["mirror"])
        return (round(s["x"] + dx, 3), round(s["y"] + dy, 3))

    def wire(self, *pts):
        pts = [snap_pt(p) for p in pts]
        for a, b in zip(pts, pts[1:]):
            if a == b:
                continue
            assert abs(a[0] - b[0]) < 1e-6 or abs(a[1] - b[1]) < 1e-6, \
                f"non-orthogonal wire {a}->{b}"
            self.wires.append((a, b))

    def junc(self, *pts):
        self.juncs.extend(snap_pt(p) for p in pts)

    def label(self, net, pt, side="left", above=True):
        j = "left" if side == "left" else "right"
        self.labels.append((net, snap_pt(pt), j, above))

    def nc(self, pt):
        self.ncs.append(snap_pt(pt))

    def box(self, x1, y1, x2, y2, title):
        self.boxes.append((x1, y1, x2, y2))
        self.texts.append((title, x1 + 3, y1 + 5.0, 2.2, "left"))

    def text(self, s, x, y, size=1.6, just="left"):
        self.texts.append((s, x, y, size, just))

    # -- power ports ------------------------------------------------------
    def power(self, kind, pt, angle=0):
        self._pwr += 1
        ref = f"#PWR{self._pwr:03d}"
        sym = POWER[kind]
        self.add(ref, sym, pt[0], pt[1], angle=angle, value=kind,
                 footprint="", in_bom=False, hide_val=True)
        return ref

    def gnd(self, pt, angle=0):
        return self.power("GND", pt, angle)

    def v3(self, pt, angle=0):
        return self.power("+3V3", pt, angle)

    def vbat(self, pt, angle=0):
        return self.power("VBAT", pt, angle)

    def flag(self, pt, angle=0):
        return self.power("PWR_FLAG", pt, angle)

    # -- emit -------------------------------------------------------------
    def render(self, paper="A2"):
        o = ['(kicad_sch (version 20231120) (generator "pursuit_gen") '
             f'(uuid "{uid("root")}") (paper "{paper}")',
             '  (title_block (title "PURSUIT-1 — person-following PCB-frame '
             'micro quadcopter") (company "Designed by Pai") (rev "A") '
             '(comment 1 "Generated from scripts/design.py — that file is the '
             'authoritative netlist") (comment 2 "Verified by scripts/check_sch.py '
             '(schematic netlist vs design.py) and the PCB netlist diff"))',
             '  (lib_symbols']
        for s in self.libs.values():
            o.append("    " + sx_str(s, 2))
        o.append('  )')

        for i, (x1, y1, x2, y2) in enumerate(self.boxes):
            o.append(f'  (rectangle (start {x1} {y1}) (end {x2} {y2}) '
                     '(stroke (width 0.254) (type dash)) (fill (type none)) '
                     f'(uuid "{uid(f"box{i}")}"))')
        for i, (s, x, y, size, just) in enumerate(self.texts):
            o.append(f'  (text "{s}" (at {x} {y} 0) (effects (font '
                     f'(size {size} {size}) (bold yes)) (justify {just})) '
                     f'(uuid "{uid(f"tx{i}")}"))')
        for i, (a, b) in enumerate(self.wires):
            o.append(f'  (wire (pts (xy {a[0]} {a[1]}) (xy {b[0]} {b[1]})) '
                     f'(stroke (width 0) (type default)) (uuid "{uid(f"w{i}")}"))')
        for i, p in enumerate(self.juncs):
            o.append(f'  (junction (at {p[0]} {p[1]}) (diameter 0) '
                     f'(color 0 0 0 0) (uuid "{uid(f"j{i}")}"))')
        for i, p in enumerate(self.ncs):
            o.append(f'  (no_connect (at {p[0]} {p[1]}) (uuid "{uid(f"n{i}")}"))')
        for i, (net, p, just, above) in enumerate(self.labels):
            vert = "bottom" if above else "top"
            o.append(f'  (label "{net}" (at {p[0]} {p[1]} 0) (effects (font '
                     f'(size 1.27 1.27)) (justify {just} {vert})) '
                     f'(uuid "{uid(f"lb{i}")}"))')

        for s in self.syms:
            mir = f' (mirror {s["mirror"]})' if s["mirror"] else ""
            u = uid("sym-" + s["ref"])
            o.append(f'  (symbol (lib_id "{s["lid"]}") (at {s["x"]} {s["y"]} '
                     f'{s["angle"]}){mir} (unit 1)')
            o.append(f'    (exclude_from_sim no) (in_bom '
                     f'{"yes" if s["in_bom"] else "no"}) (on_board yes) (dnp no)')
            o.append(f'    (uuid "{u}")')
            # KiCad adds the symbol's rotation to a property's own angle, so
            # cancel it: every designator and value stays horizontal.
            pa = (360 - s["angle"]) % 360
            rdx, rdy = s["rprop"]
            vdx, vdy = s["vprop"]
            o.append(f'    (property "Reference" "{s["ref"]}" '
                     f'(at {s["x"] + rdx} {s["y"] + rdy} {pa}) (effects (font '
                     f'(size 1.27 1.27)){" hide" if not s["in_bom"] else ""}))')
            hv = " hide" if s["hide_val"] else ""
            o.append(f'    (property "Value" "{s["value"]}" '
                     f'(at {s["x"] + vdx} {s["y"] + vdy} {pa}) '
                     f'(effects (font (size 1.27 1.27)){hv}))')
            if s["fp"] is not None:
                o.append(f'    (property "Footprint" "{s["fp"]}" '
                         f'(at {s["x"]} {s["y"]} 0) '
                         '(effects (font (size 1.27 1.27)) hide))')
            for num in s["pins"]:
                o.append(f'    (pin "{num}" (uuid "{uid(f"p-{s["ref"]}-{num}")}"))')
            o.append(f'    (instances (project "pursuit" (path "/{uid("root")}" '
                     f'(reference "{s["ref"]}") (unit 1))))')
            o.append('  )')

        o.append('  (sheet_instances (path "/" (page "1")))')
        o.append(')')
        return "\n".join(o) + "\n"


def pwr_flag():
    """ERC needs one power_out on each supply net or it reports every power
    port as undriven. VBAT and GND arrive from a battery connector, which has
    no power_out pin, so they get an explicit flag."""
    txt = ('(symbol "pursuit_sym:PWR_FLAG" (power) (pin_numbers hide) '
           '(pin_names (offset 0) hide) (exclude_from_sim no) (in_bom no) '
           '(on_board yes)'
           '  (property "Reference" "#FLG" (at 0 1.905 0) '
           '(effects (font (size 1.27 1.27)) hide))'
           '  (property "Value" "PWR_FLAG" (at 0 3.556 0) '
           '(effects (font (size 1.27 1.27))))'
           '  (symbol "PWR_FLAG_0_0" (pin power_out line (at 0 0 90) '
           '(length 0) (name "pwr") (number "1")))'
           '  (symbol "PWR_FLAG_0_1" (polyline (pts (xy 0 0) (xy 0 1.27) '
           '(xy -1.016 1.905) (xy 0 2.54) (xy 1.016 1.905) (xy 0 1.27)) '
           '(stroke (width 0) (type default)) (fill (type none))))'
           ')')
    return parse(tokenize(txt))[0]


POWER = {"GND": power_symbol("GND", "gnd"),
         "+3V3": power_symbol("+3V3", "vcc"),
         "VBAT": power_symbol("VBAT", "vcc"),
         "PWR_FLAG": pwr_flag()}


def fp_of(ref):
    c = design.COMPONENTS[ref]
    return (f"{c['lib']}:{c['footprint']}" if c["lib"]
            else f"pursuit:{c['footprint']}")


def val_of(ref):
    return design.COMPONENTS[ref]["value"]


# ------------------------------------------------------------- the sheet ---
def build():
    sh = Sheet()
    R = lambda: load_lib_symbol("Device", "R")
    C = lambda: load_lib_symbol("Device", "C")
    CP = lambda: load_lib_symbol("Device", "C_Polarized")
    LED = lambda: load_lib_symbol("Device", "LED")
    DSK = lambda: load_lib_symbol("Device", "D_Schottky")
    FET = lambda: load_lib_symbol("Transistor_FET", "Q_NMOS_GSD")
    CN2 = lambda: load_lib_symbol("Connector_Generic", "Conn_01x02")
    CN7 = lambda: load_lib_symbol("Connector_Generic", "Conn_01x07")

    # =================================================== MOTOR DRIVE x4 ===
    # One block per stage, identical geometry, so the four read as a set.
    for i in (1, 2, 3, 4):
        X, Y = 380.0, 18.0 + (i - 1) * 86.0
        sh.box(X, Y, X + 192, Y + 78, f"MOTOR DRIVE {i}   (M{i})")

        vb, sw, sig, gnd = Y + 14, Y + 30, Y + 42, Y + 72

        # gate path: PWM -> series R -> gate node -> FET gate
        sh.add(f"R{i}", R(), X + 40, sig, angle=90, value=val_of(f"R{i}"),
               footprint=fp_of(f"R{i}"))
        p1, p2 = sh.pin(f"R{i}", 1), sh.pin(f"R{i}", 2)
        sh.wire((X + 12, sig), p1)
        sh.label(f"M{i}_PWM", (X + 12, sig), "left")
        sh.wire(p2, (X + 57, sig))

        sh.add(f"R{i + 4}", R(), X + 57, sig + 12, angle=0,
               value=val_of(f"R{i + 4}"), footprint=fp_of(f"R{i + 4}"))
        g1, g2 = sh.pin(f"R{i + 4}", 1), sh.pin(f"R{i + 4}", 2)
        sh.wire((X + 57, sig), g1)
        sh.wire(g2, (X + 57, gnd))
        sh.junc((X + 57, sig))

        sh.add(f"Q{i}", FET(), X + 77, sig, angle=0, value=val_of(f"Q{i}"),
               footprint=fp_of(f"Q{i}"), rprop=(11, -6), vprop=(11, 8))
        qg, qs, qd = (sh.pin(f"Q{i}", 1), sh.pin(f"Q{i}", 2), sh.pin(f"Q{i}", 3))
        sh.wire((X + 57, sig), qg)
        sh.label(f"M{i}_GATE", (X + 62, sig), "left")
        sh.wire(qs, (qs[0], gnd))
        sh.wire(qd, (qd[0], sw))

        # switched node -> flyback anode + motor -
        sh.wire((qd[0], sw), (X + 141, sw))
        sh.label(f"M{i}_SW", (X + 100, sw), "left")

        sh.add(f"D{i}", DSK(), X + 115, vb + 8, angle=270, value=val_of(f"D{i}"),
               footprint=fp_of(f"D{i}"))
        dk, da = sh.pin(f"D{i}", 1), sh.pin(f"D{i}", 2)
        sh.wire(dk, (dk[0], vb))
        sh.junc((dk[0], vb))
        sh.wire(da, (da[0], sw))
        sh.junc((da[0], sw))

        # motor pads
        sh.add(f"MT{i}", CN2(), X + 158, vb + 12, angle=0, value=val_of(f"MT{i}"),
               footprint=fp_of(f"MT{i}"), rprop=(6, -5), vprop=(6, 6))
        m1, m2 = sh.pin(f"MT{i}", 1), sh.pin(f"MT{i}", 2)
        sh.wire(m1, (X + 146, m1[1]), (X + 146, vb))
        sh.wire(m2, (X + 141, m2[1]), (X + 141, sw))
        sh.junc((X + 146, vb))

        # VBAT ceramic bypass at the flyback (belly part)
        sh.add(f"C{i + 9}", C(), X + 180, vb + 8, angle=0,
               value=val_of(f"C{i + 9}"), footprint=fp_of(f"C{i + 9}"))
        c1, c2 = sh.pin(f"C{i + 9}", 1), sh.pin(f"C{i + 9}", 2)
        sh.wire(c1, (c1[0], vb))
        sh.wire(c2, (c2[0], gnd))

        # rails
        sh.wire((X + 105, vb), (X + 180, vb))
        sh.vbat((X + 105, vb))
        sh.wire((X + 57, gnd), (X + 180, gnd))
        sh.junc((qs[0], gnd))
        sh.gnd((X + 120, gnd))
        sh.junc((X + 120, gnd))

    # ================================================== POWER INPUT ======
    sh.box(15, 18, 185, 128, "POWER INPUT  —  1S LiPo")
    VB, GB = 46.0, 108.0                      # VBAT rail y, GND return y

    sh.add("J1", CN2(), 40, VB, angle=0, mirror="y", value=val_of("J1"),
           footprint=fp_of("J1"), rprop=(-9, -5), vprop=(-9, 5))
    j1a, j1b = sh.pin("J1", 1), sh.pin("J1", 2)
    sh.wire(j1a, (70, j1a[1]))
    sh.wire(j1b, (58, j1b[1]), (58, GB))
    sh.text("JST-PH  (CHECK PACK POLARITY)", 22, 34, 1.4)

    sh.add("J5", CN2(), 40, VB + 30, angle=0, mirror="y", value=val_of("J5"),
           footprint=fp_of("J5"), rprop=(-9, -5), vprop=(-9, 5))
    j5a, j5b = sh.pin("J5", 1), sh.pin("J5", 2)
    sh.wire(j5a, (70, j5a[1]), (70, VB))
    sh.wire(j5b, (64, j5b[1]), (64, GB))
    sh.junc((70, VB), (64, GB))
    sh.text("20AWG pigtail pads", 22, VB + 24, 1.4)

    sh.add("C1", CP(), 100, VB + 22, angle=0, value=val_of("C1"),
           footprint=fp_of("C1"))
    c1a, c1b = sh.pin("C1", 1), sh.pin("C1", 2)
    sh.wire(c1a, (c1a[0], VB))
    sh.wire(c1b, (c1b[0], GB))
    sh.junc((c1a[0], VB), (c1b[0], GB))

    sh.add("J4", CN2(), 150, VB + 14, angle=0, mirror="y", value=val_of("J4"),
           footprint=fp_of("J4"), rprop=(-9, -5), vprop=(-9, 5))
    j4a, j4b = sh.pin("J4", 1), sh.pin("J4", 2)
    sh.wire(j4a, (140, j4a[1]), (140, VB))
    sh.wire(j4b, (134, j4b[1]), (134, GB))
    sh.junc((140, VB), (134, GB))
    sh.text("wires to XIAO BAT pads", 128, VB + 26, 1.4)

    sh.wire((70, VB), (165, VB))
    sh.label("VBAT", (86, VB), "left")
    sh.vbat((165, VB), angle=0)
    sh.wire((152.4, VB), (152.4, VB - 10.16))
    sh.flag((152.4, VB - 10.16))
    sh.junc((152.4, VB))
    sh.wire((58, GB), (134, GB))
    sh.gnd((96, GB))
    sh.junc((96, GB))
    sh.wire((111.76, GB), (111.76, GB + 10.16))
    sh.flag((111.76, GB + 10.16), angle=180)
    sh.junc((111.76, GB))

    # ================================================ FLIGHT CONTROLLER ==
    sh.box(195, 18, 370, 112, "FLIGHT CONTROLLER")
    u1 = box_symbol("XIAO_ESP32S3_Sense",
                    [("1", "D0/GPIO1", "output"), ("2", "D1/GPIO2", "output"),
                     ("3", "D2/GPIO3", "input"), ("4", "D3/GPIO4", "output"),
                     ("5", "D4/GPIO5", "bidirectional"),
                     ("6", "D5/GPIO6", "bidirectional"),
                     ("7", "D6/GPIO43", "passive")],
                    [("8", "5V", "passive"), ("9", "GND", "power_in"),
                     ("10", "3V3", "power_out"), ("11", "D10/GPIO9", "output"),
                     ("12", "D9/GPIO8", "input"), ("13", "D8/GPIO7", "output"),
                     ("14", "D7/GPIO44", "passive")], width=33.02)
    sh.add("U1", u1, 275, 70, value=val_of("U1"), footprint=fp_of("U1"),
           rprop=(0, -15), vprop=(0, 15))
    sh.text("USB-C + OV2640 camera + WiFi on this module", 200, 106, 1.4)

    left_nets = {1: "M1_PWM", 2: "M2_PWM", 3: "VBAT_SENSE", 4: "M3_PWM",
                 5: "SDA", 6: "SCL"}
    for n, net in left_nets.items():
        p = sh.pin("U1", n)
        sh.wire(p, (225, p[1]))
        sh.label(net, (225, p[1]), "right")
    sh.nc(sh.pin("U1", 7))
    sh.nc(sh.pin("U1", 8))
    sh.nc(sh.pin("U1", 14))

    p = sh.pin("U1", 9)                        # GND
    sh.wire(p, (p[0] + 14, p[1]))
    sh.gnd((p[0] + 14, p[1]), angle=270)
    p = sh.pin("U1", 10)                       # 3V3 out
    sh.wire(p, (p[0] + 26, p[1]), (p[0] + 26, 40))
    sh.v3((p[0] + 26, 40))
    sh.label("+3V3", (p[0] + 26, 46), "left")
    for n, net in ((11, "LED_CTRL"), (12, "IMU_INT"), (13, "M4_PWM")):
        p = sh.pin("U1", n)
        sh.wire(p, (330, p[1]))
        sh.label(net, (330, p[1]), "left")

    # ============================================================= IMU ===
    sh.box(195, 122, 370, 268, "INERTIAL MEASUREMENT UNIT")
    u2 = box_symbol("ICM-42688-P",
                    [("1", "AP_AD0", "input"), ("2", "RESV", "passive"),
                     ("3", "RESV", "passive"), ("4", "INT1", "output"),
                     ("5", "VDDIO", "power_in"), ("6", "GND", "power_in"),
                     ("7", "RESV", "passive")],
                    [("8", "VDD", "power_in"), ("9", "INT2/FSYNC", "input"),
                     ("10", "RESV", "passive"), ("11", "RESV", "passive"),
                     ("12", "AP_CS", "input"), ("13", "AP_SCL", "input"),
                     ("14", "AP_SDA", "bidirectional")], width=38.1)
    sh.add("U2", u2, 262, 200, value=val_of("U2"), footprint=fp_of("U2"),
           rprop=(0, -15), vprop=(0, 15))
    sh.text("I2C mode: AD0 low -> 0x68, AP_CS tied to VDDIO", 200, 262, 1.4)

    gbus = 213.36                                    # left-hand ground bus
    gpins = [sh.pin("U2", n) for n in (1, 6, 7)]
    for p in gpins:
        sh.wire(p, (gbus, p[1]))
    sh.wire((gbus, gpins[0][1]), (gbus, gpins[-1][1]))
    sh.junc((gbus, gpins[1][1]))
    sh.gnd((gbus, gpins[-1][1]))
    sh.nc(sh.pin("U2", 2))
    sh.nc(sh.pin("U2", 3))
    sh.nc(sh.pin("U2", 10))
    sh.nc(sh.pin("U2", 11))

    p = sh.pin("U2", 4)                              # INT1 out to the MCU
    sh.wire(p, (232.41, p[1]))
    sh.label("IMU_INT", (232.41, p[1]), "right")
    p = sh.pin("U2", 5)                              # VDDIO
    sh.wire(p, (224.79, p[1]))
    sh.v3((224.79, p[1]))

    vbus = 300.99                                    # right-hand supply bus
    v8, v12 = sh.pin("U2", 8), sh.pin("U2", 12)
    sh.wire(v8, (vbus, v8[1]))
    sh.wire(v12, (vbus, v12[1]))
    sh.wire((vbus, v8[1]), (vbus, v12[1]))
    sh.v3((vbus, v8[1]))
    p = sh.pin("U2", 9)                              # INT2/FSYNC -> GND
    sh.wire(p, (292.1, p[1]))
    sh.gnd((292.1, p[1]), angle=270)
    for n, net in ((13, "SCL"), (14, "SDA")):
        p = sh.pin("U2", n)
        sh.wire(p, (311.15, p[1]))
        sh.label(net, (311.15, p[1]), "left")

    # I2C pull-ups
    for ref, net, x in (("R9", "SDA", 215.9), ("R10", "SCL", 227.33)):
        sh.add(ref, R(), x, 158.75, mirror="x", value=val_of(ref),
               footprint=fp_of(ref), rprop=(4.4, -3.0), vprop=(4.4, 3.0))
        sig, sup = sh.pin(ref, 1), sh.pin(ref, 2)      # 1 = bus, 2 = +3V3
        sh.wire(sup, (sup[0], 148.59))
        sh.v3((sup[0], 148.59))
        sh.wire(sig, (sig[0], 168.91))
        sh.label(net, (sig[0], 168.91), "left")
    sh.text("I2C pull-ups", 205.74, 144.78, 1.4)

    # IMU decoupling, in datasheet order
    for ref, x, note in (("C2", 300, "VDDIO"), ("C3", 316, "VDD"),
                         ("C4", 332, "VDDIO bulk")):
        sh.add(ref, C(), x, 236, angle=0, value=val_of(ref), footprint=fp_of(ref))
        a, b = sh.pin(ref, 1), sh.pin(ref, 2)
        sh.wire(a, (a[0], 226))
        sh.wire(b, (b[0], 248))
        sh.text(note, x - 6, 254, 1.2)
    sh.wire((300, 226), (332, 226))
    sh.junc((316, 226))
    sh.v3((300, 226), angle=0)
    sh.wire((300, 248), (332, 248))
    sh.junc((316, 248))
    sh.gnd((332, 248))

    # ============================================== BATTERY TELEMETRY ====
    sh.box(15, 138, 185, 226, "BATTERY TELEMETRY  —  4.2V -> 2.1V")
    sh.add("R11", R(), 70, 164, angle=0, value=val_of("R11"), footprint=fp_of("R11"))
    sh.add("R12", R(), 70, 192, angle=0, value=val_of("R12"), footprint=fp_of("R12"))
    a11, b11 = sh.pin("R11", 1), sh.pin("R11", 2)
    a12, b12 = sh.pin("R12", 1), sh.pin("R12", 2)
    sh.wire(a11, (70, 154))
    sh.label("VBAT", (70, 154), "left")
    sh.vbat((70, 154))
    sh.wire(b11, a12)
    sh.junc((70, 178))
    sh.wire(b12, (70, 214))
    sh.gnd((70, 214))
    sh.wire((70, 178), (110, 178))
    sh.label("VBAT_SENSE", (92, 178), "left")

    sh.add("C8", C(), 130, 190, angle=0, value=val_of("C8"), footprint=fp_of("C8"))
    a8, b8 = sh.pin("C8", 1), sh.pin("C8", 2)
    sh.wire((110, 178), (130, 178), a8)
    sh.wire(b8, (130, 214))
    sh.gnd((130, 214))
    sh.text("ADC anti-alias / noise filter", 100, 222, 1.4)

    # ================================================ ALTITUDE SENSOR ====
    sh.box(15, 236, 185, 300, "ALTITUDE SENSOR")
    sh.add("J2", CN7(), 110, 268, angle=0, mirror="y", value=val_of("J2"),
           footprint=fp_of("J2"), rprop=(-13, -13), vprop=(-13, 13))
    j2 = design.COMPONENTS["J2"]["pins"]
    for n in sorted(j2, key=int):
        p = sh.pin("J2", n)
        net = j2[n]
        if net is None:
            sh.nc(p)
            continue
        sh.wire(p, (p[0] + 18, p[1]))
        if net == "GND":
            sh.gnd((p[0] + 18, p[1]), angle=270)
        elif net == "+3V3":
            sh.v3((p[0] + 18, p[1]), angle=270)
        else:
            sh.label(net, (p[0] + 18, p[1]), "left")
    sh.text("Pololu VL53L1X carrier #3415, belly-down on a right-angle header",
            20, 293, 1.4)

    # ================================================ ORIENTATION LEDS ===
    sh.box(15, 310, 185, 368, "ORIENTATION LEDS")
    for ref_r, ref_d, y, note in (("R13", "LED1", 330, "nose (yellow)"),
                                  ("R14", "LED2", 352, "tail (red)")):
        sh.add(ref_r, R(), 70, y, angle=90, value=val_of(ref_r),
               footprint=fp_of(ref_r))
        a, b = sh.pin(ref_r, 1), sh.pin(ref_r, 2)
        sh.wire((45, y), a)
        sh.label("LED_CTRL", (45, y), "right")
        sh.add(ref_d, LED(), 105, y, angle=0, mirror="y",
               value=val_of(ref_d), footprint=fp_of(ref_d),
               rprop=(0, -6.5), vprop=(0, 7.5))
        k, an = sh.pin(ref_d, 1), sh.pin(ref_d, 2)
        sh.wire(b, an)
        sh.label(f"LED{ref_d[-1]}_A", ((b[0] + an[0]) / 2, y), "left")
        sh.wire(k, (k[0] + 14, y))
        sh.gnd((k[0] + 14, y), angle=270)
        sh.text(note, 132, y + 2, 1.4)

    # ==================================================== 3V3 RAIL ======
    sh.box(195, 278, 370, 368, "3V3 RAIL BULK  —  DO NOT FIT")
    for ref, x in (("C6", 240), ("C7", 275)):
        sh.add(ref, C(), x, 312, angle=0, value=val_of(ref), footprint=fp_of(ref))
        a, b = sh.pin(ref, 1), sh.pin(ref, 2)
        sh.wire(a, (x, 300))
        sh.v3((x, 300))
        sh.wire(b, (x, 326))
        sh.gnd((x, 326))
    sh.text("Pads only. The XIAO's 3V3 comes from an SGM6029C buck whose", 200, 345, 1.3)
    sh.text("datasheet specifies 5-10uF EFFECTIVE Cout; the module already", 200, 350, 1.3)
    sh.text("carries 9.4uF. Fit only if bring-up shows the rail sagging.", 200, 355, 1.3)
    return sh


def main():
    for ref, c in design.COMPONENTS.items():
        assert c["pins"] != design.PENDING, f"{ref} still PENDING"
    sh = build()
    # Nothing may sit outside a block. Three decoupling annotations once
    # landed 2mm below the IMU box, floating in the gutter between blocks —
    # invisible in the generator, obvious on screen. This is the gate.
    def inside(px, py, pad=0.0):
        return any(x1 - pad <= px <= x2 + pad and y1 - pad <= py <= y2 + pad
                   for x1, y1, x2, y2 in sh.boxes)

    orphans = [f"text {t[0]!r} at ({t[1]}, {t[2]})"
               for t in sh.texts if not inside(t[1], t[2])]
    orphans += [f"symbol {z['ref']} at ({z['x']}, {z['y']})"
                for z in sh.syms if not inside(z["x"], z["y"], pad=6.0)]
    orphans += [f"label {l[0]!r} at {l[1]}"
                for l in sh.labels if not inside(l[1][0], l[1][1], pad=2.0)]
    assert not orphans, "outside every block:\n  " + "\n  ".join(orphans)

    placed = {s["ref"] for s in sh.syms if not s["ref"].startswith("#")}
    missing = set(design.COMPONENTS) - placed
    extra = placed - set(design.COMPONENTS)
    assert not missing, f"components not drawn: {sorted(missing)}"
    assert not extra, f"drawn but not in design.py: {sorted(extra)}"

    hw = os.path.join(os.path.dirname(__file__), "..", "hardware")

    # The authored symbols (the two ICs and the power ports) exist only inside
    # the .kicad_sch unless we also ship them as a real library — without that,
    # ERC reports every one of them as "configuration does not include the
    # symbol library". Emit the library and register it, so the project is
    # self-contained and opens clean on any machine.
    own = {k: v for k, v in sh.libs.items() if k.startswith("pursuit_sym:")}
    lib = ['(kicad_symbol_lib (version 20231120) (generator "pursuit_gen")']
    for lid, sym in sorted(own.items()):
        node = list(sym)
        node[1] = f'"{lid.split(":", 1)[1]}"'
        lib.append("  " + sx_str(node, 1))
    lib.append(")")
    with open(os.path.join(hw, "pursuit_sym.kicad_sym"), "w") as f:
        f.write("\n".join(lib) + "\n")
    with open(os.path.join(hw, "sym-lib-table"), "w") as f:
        f.write('(sym_lib_table\n  (version 7)\n  (lib (name "pursuit_sym")'
                '(type "KiCad")(uri "${KIPRJMOD}/pursuit_sym.kicad_sym")'
                '(options "")(descr "PURSUIT-1 authored symbols"))\n)\n')

    dest = os.path.join(hw, "pursuit.kicad_sch")
    with open(dest, "w") as f:
        f.write(sh.render())
    print(f"wrote {os.path.abspath(dest)}")
    print(f"       + pursuit_sym.kicad_sym ({len(own)} authored symbols)" + " + sym-lib-table")
    print(f"  {len(placed)} components, {len(sh.syms) - len(placed)} power ports, "
          f"{len(sh.wires)} wires, {len(sh.labels)} labels, "
          f"{len(sh.juncs)} junctions, {len(sh.ncs)} no-connects")


if __name__ == "__main__":
    main()
