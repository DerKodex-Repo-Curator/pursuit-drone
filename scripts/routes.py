"""Explicit routing for PURSUIT-1. All coordinates in board-mm.

Endpoints snap to actual pad centres; waypoints are hand-derived and
DRC-iterated. Arm paths use radial/perp coords via AP().

Waist-region lane plan (between J4 row y4.2 and J2 row y7):
  B: SDA col x-0.3 -> J2.3/4 gap x-1.05 -> via north of row
  B: SCL col x0.55 -> J2.4/5 gap x1.49 -> via north of row
  B: IMU_INT col x6.35 -> J2.6/7 gap x6.57 -> west along y8.55
  F: +3V3 corridor y5.7 from J2.2 east to C6 (top is otherwise clear)
Arm lanes: M_SW jog at perp -3 (arms 1-3) / +3 (arm 4); LED1 on arm 2 at
perp +2.8 (lane +3.9), LED2 on arm 4 at perp -2.8 (lane -4.0).
"""
import math

C45 = math.cos(math.radians(45))
SIGN = {1: (-1, -1), 2: (1, -1), 3: (-1, 1), 4: (1, 1)}
JOG_P = {1: -3.0, 2: -3.0, 3: -3.0, 4: -3.0}


def AP(i, r, p=0.0):
    sx, sy = SIGN[i]
    ux, uy = sx * C45, sy * C45
    return (r * ux - uy * p, r * uy + ux * p)


def add_routes(board, nets, placed, P, FromMM, FAB):
    import pcbnew

    def pad(ref, num):
        fp = placed[ref]
        pd = fp.FindPadByNumber(str(num))
        assert pd, f"{ref}.{num} missing"
        pos = pd.GetPosition()
        return (pcbnew.ToMM(pos.x) - 150.0, pcbnew.ToMM(pos.y) - 100.0)

    F, B = pcbnew.F_Cu, pcbnew.B_Cu
    tracks, vias = [], []

    def T(net, layer, wkey, pts):
        tracks.append((net, layer, wkey, pts))

    def V(net, x, y):
        vias.append((net, x, y))

    # ---------------- motor PWM fan-out (top) -----------------------------
    T("M1_PWM", F, "sig_w", [pad("U1", 1), (-9.5, -14.5), pad("R1", 1)])
    # M2_PWM runs on B.Cu across the front: on F.Cu it cut arm 2's VBAT
    # pour in half, stranding that motor's supply on an island.
    T("M2_PWM", B, "sig_w", [pad("U1", 2), (-6.0, -14.35), (10.5, -14.35),
                             (12.0, -12.6)])
    V("M2_PWM", 12.0, -12.6)
    T("M2_PWM", F, "sig_w", [(12.0, -12.6), pad("R2", 1)])
    T("M3_PWM", F, "sig_w", [pad("U1", 4), (-14.2, -6.5), (-14.2, 11.5),
                             pad("R3", 1)])
    T("M4_PWM", F, "sig_w", [pad("U1", 13), (10.6, -1.0), (10.6, 10.5),
                             pad("R4", 1)])

    # ---------------- gate nets -------------------------------------------
    T("M1_GATE", F, "sig_w", [pad("R5", 1), (-14.16, -12.5), pad("R1", 2),
                              pad("Q1", 1)])
    T("M2_GATE", F, "sig_w", [pad("R6", 1), (12.5, -14.16), pad("R2", 2),
                              pad("Q2", 1)])
    T("M3_GATE", F, "sig_w", [pad("R7", 1), (-12.5, 14.16), pad("R3", 2),
                              pad("Q3", 1)])
    T("M4_GATE", F, "sig_w", [pad("R8", 1), (14.16, 12.5), pad("R4", 2),
                              pad("Q4", 1)])

    # ---------------- switched motor nodes (2mm, top, along arms) ---------
    for i in (1, 2, 3, 4):
        j = JOG_P[i]
        T(f"M{i}_SW", F, "motor_w",
          [pad(f"Q{i}", 3), AP(i, 25.6, j), AP(i, 31.4, j),
           pad(f"D{i}", 2), AP(i, 34.5, 0.0), pad(f"MT{i}", 2)])

    # ---------------- battery telemetry -----------------------------------
    T("VBAT_SENSE", B, "sig_w", [pad("U1", 3), (-6.6, -9.27), (13.8, -9.27)])
    V("VBAT_SENSE", 13.8, -9.27)
    T("VBAT_SENSE", F, "sig_w", [(13.8, -9.27), (17.6, -8.9), pad("C8", 1)])
    T("VBAT_SENSE", F, "sig_w", [pad("C8", 1), pad("R11", 2)])
    T("VBAT_SENSE", F, "sig_w",
      [pad("R11", 2), (18.0, -6.0), (18.0, -2.2), pad("R12", 1)])

    # ---------------- LED control + anodes --------------------------------
    T("LED_CTRL", F, "sig_w", [pad("U1", 11), (9.7, -9.4), pad("R13", 1)])
    T("LED_CTRL", B, "sig_w", [pad("U1", 11), (9.65, -6.8), (9.65, 6.6),
                               (11.2, 8.5), (11.9, 16.5), (11.5, 17.5)])
    V("LED_CTRL", 11.5, 17.5)
    T("LED_CTRL", F, "sig_w", [(11.5, 17.5), pad("R14", 1)])

        # enters arm 2 outboard of the Q2 ground-via cluster (radial <=22.65)
    T("LED1_A", F, "sig_w", [pad("R13", 2), (18.6, -11.6), (20.2, -13.6),
                             AP(2, 26, 5.0), AP(2, 35.8, 5.0),
                             pad("LED1", 2)])
    T("LED2_A", F, "sig_w", [pad("R14", 2), pad("LED2", 2)])

    # ---------------- I2C bus ----------------------------------------------
    # SDA: B column through the J4 gap, up the J2.3/4 pin gap, via, top fan
    T("SDA", B, "sig_w", [pad("U1", 5), (-0.3, -5.0), (-0.3, 5.05),
                          (-1.05, 5.9), (-1.05, 7.75)])
    V("SDA", -1.05, 7.75)
    T("SDA", F, "sig_w", [(-1.05, 7.75), (0.0, 7.5), pad("J2", 4)])
    T("SDA", F, "sig_w", [(-1.05, 7.75), (-0.68, 8.7), pad("U2", 14)])
    T("SDA", F, "sig_w", [(-1.05, 7.75), (-2.0, 8.35), (-8.5, 8.35),
                          (-11.3, 8.55), pad("R9", 1)])
    # SCL: B column x0.55, J2.4/5 gap, via, top fan
    # SCL: west dive through the U1.6/7 gap, south under SDA, east along
    # y-6.73 (U1 4/5 pin gap), up the J2.5/6 gap, via NE of J2.5
    T("SCL", B, "sig_w", [pad("U1", 6), (-8.9, -1.65), (-8.9, -6.73),
                          (3.9, -6.73), (4.03, 4.2), (4.03, 7.5),
                          (3.55, 8.3), (2.9, 8.6)])
    V("SCL", 2.9, 8.6)
    T("SCL", F, "sig_w", [(2.9, 8.6), (2.76, 7.6), pad("J2", 5)])
    T("SCL", F, "sig_w", [(2.9, 8.6), (0.9, 8.5), (0.0, 8.75),
                          pad("U2", 13)])
    T("SCL", B, "sig_w", [(2.9, 8.6), (-3.0, 9.1), (-8.0, 9.3),
                          (-8.5, 9.6)])
    V("SCL", -8.5, 9.6)
    T("SCL", F, "sig_w", [(-8.5, 9.6), pad("R10", 1)])

    # ---------------- IMU interrupt (B, J2.6/7 gap, west along y8.55) -----
    T("IMU_INT", B, "sig_w",
      [pad("U1", 12), (6.35, -4.6), (6.35, 5.9), (6.57, 6.4), (6.57, 7.6),
       (5.2, 8.9), (-1.6, 10.6), (-2.8, 10.95)])
    V("IMU_INT", -2.8, 10.95)
    T("IMU_INT", F, "sig_w", [(-2.8, 10.95), pad("U2", 4)])

    # ---------------- +3V3 (0.3mm sensor rail) ----------------------------
    # B trunk west between U1 pin gaps, around antenna keepout, to J2.2
    T("+3V3", B, "sig_w",
      [pad("U1", 10), (6.4, -11.81), (-6.2, -11.81), (-9.6, -11.81),
       (-10.2, -10.8), (-10.2, 5.4), (-4.86, 5.4), pad("J2", 2)])
    T("+3V3", B, "sig_w", [(-10.2, 5.4), (-12.6, 7.4), (-13.2, 10.9)])
    V("+3V3", -13.2, 10.9)
    T("+3V3", F, "sig_w", [(-13.2, 10.9), pad("R9", 2)])
    T("+3V3", F, "sig_w", [pad("R9", 2), pad("R10", 2)])
    T("+3V3", F, "sig_w", [pad("R10", 2), (-8.2, 11.03), pad("C4", 1)])
    T("+3V3", F, "sig_w", [pad("C4", 1), pad("C2", 1)])
    T("+3V3", F, "sig_w", [pad("C2", 1), (-3.4, 12.2), (-0.55, 11.85),
                           pad("U2", 5)])
    # top corridor from J2.2 (THT) east to the capacitor chain
    T("+3V3", F, "sig_w", [pad("J2", 2), (-4.5, 5.7), (3.5, 5.7),
                           (4.45, 4.9), pad("C6", 1)])
    T("+3V3", F, "sig_w", [pad("C6", 1), (4.03, 5.2), (4.03, 8.0),
                           (5.35, 8.8), (5.35, 10.7), pad("C3", 1)])
    T("+3V3", F, "sig_w", [pad("C3", 1), (5.2, 12.2), pad("C7", 1)])
    T("+3V3", F, "sig_w", [pad("C3", 1), (2.6, 11.7), (1.16, 11.4),
                           pad("U2", 8)])
    # AP_CS ties to VDDIO (pin 5) through the package centre — 0.15mm
    T("+3V3", F, "fine_w", [pad("U2", 12), (0.62, 9.9), (0.0, 10.5),
                            (-0.55, 10.65), pad("U2", 5)])

    # ---------------- VBAT: bottom bulk cap into the top pour -------------
    _c1x, _c1y = pad("C1", 1)
    _v1 = (_c1x + 0.0, _c1y + 2.4)
    _v2 = (_c1x + 1.4, _c1y + 2.2)
    T("VBAT", B, "pwr_w", [(_c1x, _c1y), _v1])
    T("VBAT", B, "pwr_w", [(_c1x, _c1y), _v2])
    V("VBAT", *_v1)
    V("VBAT", *_v2)

    # ---------------- GND stitching ---------------------------------------
    gnd = [
        ("C2", 2, [], (-3.7, 10.2)),
        ("C3", 2, [(4.35, 10.1)], (3.6, 10.4)),
        ("C4", 2, [], (-5.75, 10.0)),
        ("C6", 2, [], (7.5, 4.6)),
        ("J4", 2, [(2.9, 3.3)], (2.9, 2.4)),
        ("C7", 2, [], (8.6, 9.3)),
        ("C8", 2, [], (20.2, -11.0)),
        ("R12", 2, [], (19.6, -4.0)),
        # NOTE: Q source stubs are NOT in this list — they carry the full
        # motor current and are handled separately below.
        # R5-R8 (gate pulldowns) are handled arm-aware below: their old
        # fixed vias put R6's inside the camera-hole copper keepout.
        ("U2", 1, [(-2.5, 9.6), (-3.3, 10.0)], (-3.7, 10.2)),  # shares C2's
        ("U2", 6, [(0.0, 12.5)], (0.3, 13.15)),
        ("U2", 7, [(0.4, 12.3)], (0.3, 13.15)),
        ("U2", 9, [(2.4, 10.45)], (3.6, 10.4)),     # shares C3's via
        ("LED1", 1, [AP(2, 31.9, 1.9)], AP(2, 31.9, 1.9)),
        ("LED2", 1, [], (5.4, 18.6)),
    ]
    # ---- FET source: the motor return, same 2.75A as the drain ----------
    # This is the return leg of the motor loop and it is the ONLY path (Q is
    # SMD on F.Cu, the GND pour is on B.Cu). It was previously a 0.3mm stub
    # and a single 0.4mm via — 7x less copper than the 2.0mm drain trace for
    # identical current, and a single point of failure on an airframe that
    # flexes. Now 2.0mm into a 4-via cluster placed hard against the pad,
    # which also shrinks the source loop inductance.
    # Q source sits at arm-local (radial 22.25, perp +0.95); the drain is at
    # perp 0 and the M_SW jog at perp -3.0, so the cluster goes further out
    # along +perp where the arm is empty.
    for i in (1, 2, 3, 4):
        qx, qy = pad(f"Q{i}", 2)
        cxy = AP(i, 21.8, 3.0)
        T("GND", F, "src_w", [(qx, qy), cxy])
        for dr, dp in ((-0.45, -0.45), (0.45, -0.45),
                       (-0.45, 0.45), (0.45, 0.45)):
            V("GND", *AP(i, 21.8 + dr, 3.0 + dp))

    # ---- gate-pulldown grounds, placed relative to their own pad --------
    for i in (1, 2, 3, 4):
        sx, sy = SIGN[i]
        ux, uy = sx * C45, sy * C45
        px, py = -uy, ux
        rx, ry = pad(f"R{i+4}", 2)
        vx, vy = rx - 1.35 * px, ry - 1.35 * py   # >=0.25mm to pad edge
        T("GND", F, "sig_w", [(rx, ry), (vx, vy)])
        V("GND", vx, vy)

    # ---- VBAT ceramic bypass at each FET (belly) ------------------------
    # Pad 2 (GND) lands straight in the B.Cu GND pour — no return routing.
    # Pad 1 (VBAT) gets a via immediately beside it, lifting to the F.Cu
    # VBAT pour. Keeps the bypass loop to a few millimetres.
    for i in (1, 2, 3, 4):
        sx, sy = SIGN[i]
        ux, uy = sx * C45, sy * C45
        px, py = -uy, ux                     # arm perpendicular unit
        bx, by = pad(f"C{i+9}", 1)
        vxy = AP(i, 29.0, 1.9)           # beside D's VBAT pad, clear of it
        T("VBAT", B, "sig_w", [(bx, by), vxy])
        V("VBAT", *vxy)
        # Redundant vias at every flyback cathode. One via per motor was the
        # blocker on the FET source; the same single-point-of-failure then
        # reappeared here when arm 2's pour pinched into an island. Three
        # barrels means a cracked one is not a dead motor.
        for _dr in (-1.5, 1.4):
            V("VBAT", *AP(i, 29.0 + _dr, 1.9))
            T("VBAT", B, "sig_w", [vxy, AP(i, 29.0 + _dr, 1.9)])
        T("VBAT", F, "pwr_w", [pad(f"D{i}", 1), vxy])
        # Explicit VBAT spine down the arm on B.Cu. The F.Cu pour is the
        # nominal path, but the M_SW jog, the flyback pads and (on arm 2)
        # the LED lane pinch it, and a motor losing VBAT in flight is not
        # something to leave to pour topology. This is a deterministic
        # 1.2mm conductor from the plate to each flyback's VBAT pad.
        # entry point differs per arm: each plate corner has its own
        # congestion (M_PWM fan-out, the +3V3 tap, R9/R10, LED_CTRL)
        # Entry is always on the PLATE side of the arm. That matters on
        # arm 2, whose F.Cu pour is severed around radial 23 where the
        # LED1_A lane crosses the source cluster — the spine bridges it.
        pr, pp = {1: (15.0, -1.5), 2: (20.0, -5.0),
                  3: (16.0, -1.5), 4: (20.0, -1.5)}[i]
        pxy = AP(i, pr, pp)
        T("VBAT", B, "src_w", [pxy, AP(i, 27.0, -1.5), vxy,
                               AP(i, 34.0, -1.0), pad(f"MT{i}", 1)])
        V("VBAT", *pxy)

    # The waist GND pocket (holding J4.2, the XIAO's battery-negative wire
    # pad) is fenced off from the main B.Cu pour by the SDA/SCL/IMU_INT/3V3
    # trunks. Its whole return path is this F.Cu bridge, so give it real
    # width and a second via at each end rather than one 0.3mm strand.
    T("GND", F, "mid_w", [(2.9, 2.4), (5.0, 2.4), (6.9, 3.3), (7.5, 4.0),
                          (7.5, 4.6)])
    T("GND", F, "mid_w", [(2.9, 2.4), (2.9, 1.5)])
    V("GND", 2.9, 1.5)
    T("GND", F, "mid_w", [(7.5, 4.6), (8.3, 4.0)])
    V("GND", 8.3, 4.0)

    seen = set()
    for ref, num, stub, (vx, vy) in gnd:
        pts = [pad(ref, num)] + stub
        if pts[-1] != (vx, vy):
            pts.append((vx, vy))
        T("GND", F, "sig_w", pts)
        if (vx, vy) not in seen:
            V("GND", vx, vy)
            seen.add((vx, vy))

    # ---------------- emit -------------------------------------------------
    for net, layer, wkey, pts in tracks:
        for a, b in zip(pts, pts[1:]):
            if abs(a[0] - b[0]) < 1e-6 and abs(a[1] - b[1]) < 1e-6:
                continue
            t = pcbnew.PCB_TRACK(board)
            t.SetStart(P(*a))
            t.SetEnd(P(*b))
            t.SetWidth(FromMM(FAB[wkey]))
            t.SetLayer(layer)
            t.SetNet(nets[net])
            board.Add(t)
    for net, x, y in vias:
        v = pcbnew.PCB_VIA(board)
        v.SetPosition(P(x, y))
        v.SetDrill(FromMM(FAB["via_drill"]))
        v.SetWidth(FromMM(FAB["via_dia"]))
        v.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu)
        v.SetNet(nets[net])
        board.Add(v)
