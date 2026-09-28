"""HARD RELEASE GATE: no pour may split a PAD onto its own fragment.

KiCad's DRC does not catch a pinched-off pour island as long as a single via
bridges it — which is how D2's flyback cathode ended up hanging off one
0.4mm via after the rework. Prose in VERIFICATION.md is not a substitute for
running this. Exit code 1 blocks the release.

Run with KiCad's bundled python3.
"""
import math
import os
import sys

import pcbnew

HW = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "hardware"))
OX, OY = 150.0, 100.0


def main():
    b = pcbnew.LoadBoard(os.path.join(HW, "pursuit.kicad_pcb"))
    bad = 0
    for z in b.Zones():
        if z.GetIsRuleArea():
            continue
        lyr = z.GetLayer()
        polys = z.GetFilledPolysList(lyr)
        n = polys.OutlineCount()
        net, layer = z.GetNetname(), b.GetLayerName(lyr)
        # Which of this net's pads/vias land inside each fill polygon? A
        # fragment holding a pad is a functional island: that pad's only
        # path to the rest of the net is whatever single via bridges it.
        # A pad is attributed to the fragment that SURROUNDS it, not to the
        # one containing its centre. Those differ the moment a pad is
        # thermally relieved: the fill carries a keep-out ring around the pad,
        # so the pad centre sits in a hole and PointInside says no -- for
        # every relieved pad on the board. Before rev B almost every pour pad
        # was solid and the distinction never showed; testing a ring of points
        # just outside the relief gap works for both kinds.
        feats = []
        for fp in b.GetFootprints():
            for p in fp.Pads():
                if p.GetNetname() != net or not p.IsOnLayer(lyr):
                    continue
                sz = p.GetSize()
                r = max(sz.x, sz.y) / 2 + pcbnew.FromMM(0.35)
                pos = p.GetPosition()
                ring = [pcbnew.VECTOR2I(int(pos.x + r * math.cos(a)),
                                        int(pos.y + r * math.sin(a)))
                        for a in [i * math.pi / 6 for i in range(12)]]
                feats.append((f"{fp.GetReference()}.{p.GetNumber()}",
                              [pos] + ring))
        for t in b.GetTracks():
            if isinstance(t, pcbnew.PCB_VIA) and t.GetNetname() == net:
                feats.append(("via", [t.GetPosition()]))
        held = []
        for k in range(n):
            o = polys.Outline(k)
            inside = [nm for nm, pts in feats
                      if any(o.PointInside(pt) for pt in pts)]
            held.append((o.Area() / 1e12, inside))
        # A fragment that holds a pad is only acceptable if it escapes
        # through MORE THAN ONE via — that is exactly what separated the
        # D2 blocker (one 0.4mm barrel carrying a motor's freewheel
        # current) from a deliberately bridged pocket. Named exceptions
        # only, so this cannot drift silently.
        ALLOWED = {("GND", "J4.2")}          # waist pocket, 0.8mm bridge + 2 vias
        main = max(range(n), key=lambda k: held[k][0]) if n else None
        offenders = []
        for k, (area, inside) in enumerate(held):
            if k == main or not inside:
                continue
            pads = [x for x in inside if x != "via"]
            vias = sum(1 for x in inside if x == "via")
            excepted = pads and all((net, p) in ALLOWED for p in pads)
            if vias < 2 or not excepted:
                offenders.append((k, area, inside, vias))
        if not offenders:
            extra = [k for k in range(n) if k != main and held[k][1]]
            note = (f", {len(extra)} reviewed bridged fragment(s)"
                    if extra else "")
            print(f"  OK   {net:5s} on {layer:5s}: {n} polygon(s){note}")
            continue
        bad += 1
        print(f"  FAIL {net:5s} on {layer:5s}: {n} polygons, "
              f"{len(offenders)} unbridged fragment(s) holding pads")
        for k, (area, inside) in enumerate(held):
            o = polys.Outline(k); bb = o.BBox()
            print(f"        #{k} area={area:8.2f} mm2 "
                  f"x {bb.GetLeft()/1e6-OX:7.2f}..{bb.GetRight()/1e6-OX:7.2f} "
                  f"y {bb.GetTop()/1e6-OY:7.2f}..{bb.GetBottom()/1e6-OY:7.2f}"
                  f"  holds: {', '.join(inside[:8]) if inside else '(dead copper)'}")
    print(f"\nzone-island audit: {'FAIL' if bad else 'PASS'}"
          f" ({bad} pour(s) fragmented)")
    sys.exit(1 if bad else 0)


if __name__ == "__main__":
    main()
