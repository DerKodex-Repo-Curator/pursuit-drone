"""Build the release package: gerbers, drill, renders, schematic PDF, zip.

Exists so the export flags are recorded in code rather than in a shell history.
The shipped package was once exported WITH --subtract-soldermask and later
re-exported without it; nothing downstream noticed, because "which flags did we
use" lived only in a terminal. Now it lives here.

Plain python3 is fine — this only shells out to kicad-cli.
"""
import hashlib
import os
import shutil
import subprocess
import sys
import zipfile
from datetime import datetime

CLI = "/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli"
ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
HW = os.path.join(ROOT, "hardware")
GB = os.path.join(ROOT, "gerbers")
DOCS = os.path.join(ROOT, "docs")
FAB = os.path.join(GB, "fab")
BOARD = os.path.join(HW, "pursuit.kicad_pcb")
SCH = os.path.join(HW, "pursuit.kicad_sch")

# Protel extensions, absolute (page) coordinates, mask subtracted from silk.
# The CPL is emitted in the same absolute page coordinates, so the two agree
# only as long as no origin flag is added here.
#
# The copper layers are READ FROM THE BOARD, never listed here. A literal
# "F.Cu,B.Cu" survives a 2 -> 4 layer change without complaining and simply
# stops exporting the inner planes: the fab receives a four-layer stackup with
# two blank inner layers, every gate still passes, and the first sign of
# trouble is five dead boards. This is the same class of defect as the round-3
# --subtract-soldermask incident that made this file exist.
NON_COPPER = ("F.Paste", "B.Paste", "F.Silkscreen", "B.Silkscreen",
              "F.Mask", "B.Mask", "Edge.Cuts")


def copper_layers(board_path):
    """Copper layer names, in stackup order, straight out of the board."""
    import re
    with open(board_path) as f:
        raw = f.read()
    m = re.search(r"\n\t\(layers\n(.*?)\n\t\)", raw, re.S)
    assert m, "no (layers ...) block in the board -- format changed?"
    found = re.findall(r'\(\s*(\d+)\s+"([^"]+)"\s+(signal|power|mixed|jumper)\b',
                       m.group(1))
    assert found, "no copper layers found in the board's (layers ...) block"
    names = [t[1] for t in found]

    # Sort by STACKUP position, not by KiCad's internal layer id. Those are not
    # the same order: KiCad numbers F.Cu 0 and B.Cu 2, with In1/In2 ABOVE both,
    # so sorting by id yields F, B, In1, In2 -- a physically meaningless
    # sequence that would quietly become the order in the gerber job file.
    def stack_pos(name):
        if name == "F.Cu":
            return -1
        if name == "B.Cu":
            return 10_000
        m2 = re.fullmatch(r"In(\d+)\.Cu", name)
        assert m2, f"unrecognised copper layer {name!r}"
        return int(m2.group(1))

    return sorted(names, key=stack_pos)


def gerber_layer_arg(board_path):
    cu = copper_layers(board_path)
    print(f"  copper layers from board: {', '.join(cu)}")
    return ",".join(cu + list(NON_COPPER))


def run(*args):
    r = subprocess.run(args, capture_output=True, text=True)
    if r.returncode != 0:
        sys.exit(f"FAILED: {' '.join(args)}\n{r.stdout}\n{r.stderr}")


def main():
    shutil.rmtree(FAB, ignore_errors=True)
    os.makedirs(FAB, exist_ok=True)

    run(CLI, "pcb", "export", "gerbers", "--output", FAB,
        "--layers", gerber_layer_arg(BOARD), "--subtract-soldermask", BOARD)
    run(CLI, "pcb", "export", "drill", "--output", FAB, "--format", "excellon",
        "--drill-origin", "absolute", "--generate-map",
        "--map-format", "gerberx2", BOARD)
    run(CLI, "sch", "export", "pdf", "--output",
        os.path.join(GB, "PURSUIT-1_schematic.pdf"), SCH)
    for side, name in (("top", "board_top.png"), ("bottom", "board_bottom.png")):
        run(CLI, "pcb", "render", "--output", os.path.join(GB, name),
            "--side", side, "--quality", "high", "--width", "1600",
            "--height", "1600", BOARD)

    # Assembly drawings. These are what the builder actually solders from, so
    # they must be regenerated with the board — a stale pair survived the
    # round-4 silk rework still showing the deleted "PIN1=+" marking.
    for side, name, layers in (
        ("top", "assembly_top.svg", "F.Fab,F.Silkscreen,F.Cu,Edge.Cuts"),
        ("bottom", "assembly_bottom.svg", "B.Fab,B.Silkscreen,B.Cu,Edge.Cuts"),
    ):
        args = [CLI, "pcb", "export", "svg", "--output",
                os.path.join(DOCS, name), "--layers", layers,
                "--page-size-mode", "2", "--exclude-drawing-sheet", BOARD]
        if side == "bottom":
            args.insert(-1, "--mirror")
        run(*args)

    # Plane renders. Inner copper is invisible in every other artefact -- it
    # does not appear in the 3D render, the assembly drawings or the silk, so
    # a plane that fragmented or flooded a keepout would ship unseen. These
    # are the only way a human ever looks at L2 and L3.
    for lyr, name in (("In1.Cu", "plane_in1_gnd.svg"),
                      ("In2.Cu", "plane_in2_vbat.svg")):
        if lyr not in copper_layers(BOARD):
            continue
        run(CLI, "pcb", "export", "svg", "--output", os.path.join(DOCS, name),
            "--layers", f"{lyr},Edge.Cuts", "--page-size-mode", "2",
            "--exclude-drawing-sheet", BOARD)

    h = hashlib.sha256(open(BOARD, "rb").read()).hexdigest()[:16]
    mtime = datetime.fromtimestamp(os.path.getmtime(BOARD))
    with open(os.path.join(FAB, "MANIFEST.txt"), "w") as f:
        f.write(
            "PURSUIT-1 fabrication package\n"
            f"board sha256[:16] : {h}\n"
            f"board mtime       : {mtime}\n"
            f"exported          : {datetime.now()}\n"
            f"export flags      : --subtract-soldermask, absolute drill origin\n"
            "If the board hash above does not match hardware/pursuit.kicad_pcb,\n"
            "THIS PACKAGE IS STALE - regenerate before ordering.\n")

    zpath = os.path.join(GB, "PURSUIT-1_JLCPCB.zip")
    with zipfile.ZipFile(zpath, "w", zipfile.ZIP_DEFLATED) as z:
        for n in sorted(os.listdir(FAB)):
            z.write(os.path.join(FAB, n), n)

    print(f"board hash {h}")
    print(f"wrote {FAB} ({len(os.listdir(FAB))} files)")
    print(f"wrote {zpath}")
    print("now run: <KiCad python3> scripts/gen_jlcpcb.py && "
          "<KiCad python3> scripts/preflight.py")


if __name__ == "__main__":
    main()
