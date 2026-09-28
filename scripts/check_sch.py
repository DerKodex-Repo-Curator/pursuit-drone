"""Prove the drawn schematic carries the design.py netlist.

A schematic that LOOKS wired is worth nothing — a wire that stops 0.01mm short
of a pin renders identically to one that lands on it. So this exports the
netlist with KiCad's own netlister and compares it to design.py two ways:

  1. PARTITION — which pins share a node, ignoring names. This is the real
     question and it catches a wire that misses.
  2. NAMES — every net that design.py names must appear under that name, so a
     label typo or a power port pointing at the wrong rail is caught too.

Power ports (#PWR*) are ignored as components but their nets are not.
Exit 1 on any mismatch.
"""
import os
import re
import subprocess
import sys
import tempfile

sys.path.insert(0, os.path.dirname(__file__))
import design

CLI = "/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli"
ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
SCH = os.path.join(ROOT, "hardware", "pursuit.kicad_sch")


def export_netlist():
    with tempfile.TemporaryDirectory() as td:
        out = os.path.join(td, "n.net")
        r = subprocess.run([CLI, "sch", "export", "netlist", "--format",
                            "kicadsexpr", "-o", out, SCH],
                           capture_output=True, text=True)
        if r.returncode != 0:
            sys.exit(f"netlist export failed:\n{r.stdout}\n{r.stderr}")
        return open(out).read()


def parse_nets(text):
    """-> {net_name: {(ref, pin), ...}} for non-power refs."""
    body = text[text.find("(nets"):]
    nets = {}
    for chunk in re.split(r"\(net\b", body)[1:]:
        m = re.search(r'\(name "([^"]+)"\)', chunk)
        if not m:
            continue
        name = m.group(1).lstrip("/")
        nodes = set()
        for ref, pin in re.findall(r'\(ref "([^"]+)"\)\s*\(pin "([^"]+)"\)', chunk):
            if not ref.startswith("#"):
                nodes.add((ref, pin))
        nets[name] = nodes
    return nets


def main():
    want = {}
    pending = [r for r, c in design.COMPONENTS.items()
               if c["pins"] == design.PENDING]
    if pending:
        # Without this the loop below does .items() on a string and dies with
        # an AttributeError, which reads like a broken tool rather than a
        # failed gate -- and a gate that looks broken gets skipped.
        print(f"  FAIL  pins still PENDING: {sorted(pending)}")
        sys.exit(1)
    for ref, c in design.COMPONENTS.items():
        for pin, net in c["pins"].items():
            if net:
                want.setdefault(net, set()).add((ref, pin))

    got = parse_nets(export_netlist())
    fails = []

    # 1. every design.py net must exist by name with exactly its pins
    for net, pins in sorted(want.items()):
        if net not in got:
            fails.append(f"net {net!r} missing from the schematic "
                         f"(expected {sorted(pins)})")
            continue
        if got[net] != pins:
            miss = sorted(pins - got[net])
            extra = sorted(got[net] - pins)
            fails.append(f"net {net!r}: missing {miss}, unexpected {extra}")

    # 2. nothing may be connected that design.py does not connect. Any net the
    #    schematic invents with >1 pin is a short.
    for net, pins in sorted(got.items()):
        if net in want or len(pins) < 2:
            continue
        fails.append(f"schematic invents net {net!r} joining {sorted(pins)}")

    # 3. every pin design.py declares must appear somewhere
    declared = {(r, p) for r, c in design.COMPONENTS.items()
                for p, n in c["pins"].items() if n}
    seen = {pp for pins in got.values() for pp in pins}
    for pp in sorted(declared - seen):
        fails.append(f"pin {pp[0]}.{pp[1]} is not connected in the schematic")

    for f in fails:
        print(f"  FAIL  {f}")
    n_pins = sum(len(v) for v in want.values())
    if fails:
        print(f"\nSCHEMATIC NETLIST: {len(fails)} mismatch(es) vs design.py")
        sys.exit(1)
    print(f"  ok    {len(want)} nets / {n_pins} pin connections match design.py "
          f"exactly (names and partition)")


if __name__ == "__main__":
    main()
