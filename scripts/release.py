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
GERBER_LAYERS = ("F.Cu,B.Cu,F.Paste,B.Paste,F.Silkscreen,B.Silkscreen,"
                 "F.Mask,B.Mask,Edge.Cuts")


def run(*args):
    r = subprocess.run(args, capture_output=True, text=True)
    if r.returncode != 0:
        sys.exit(f"FAILED: {' '.join(args)}\n{r.stdout}\n{r.stderr}")


def main():
    shutil.rmtree(FAB, ignore_errors=True)
    os.makedirs(FAB, exist_ok=True)

    run(CLI, "pcb", "export", "gerbers", "--output", FAB,
        "--layers", GERBER_LAYERS, "--subtract-soldermask", BOARD)
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
