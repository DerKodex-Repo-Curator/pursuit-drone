# PURSUIT-1 — one entry point for every check.
#
# `make` on its own runs everything that does not need hardware or model
# weights, which is the thing a reviewer wants and the thing CI runs.

PY      ?= vision/.venv/bin/python
PIO     ?= pio

# The board toolchain needs TWO interpreters and mixing them up wastes an hour:
#   KPY  KiCad's bundled 3.9 — anything that imports pcbnew
#   SPY  a system 3.12+      — gen_sch.py and the pure-python gates (f-strings
#                              KiCad's 3.9 cannot parse)
KPY     ?= /Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/3.9/bin/python3
SPY     ?= /opt/homebrew/Cellar/platformio/6.1.19_2/libexec/bin/python3
CLI     ?= /Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli
HWDIR   ?= $(HOME)/hw/scripts

.PHONY: help all test vision sim firmware target e2e sweep verify quant \
        protocol hygiene docker demo clean board board-gates

help:
	@echo "make            everything that needs no hardware and no weights"
	@echo "make vision     Python suite for the off-board stack"
	@echo "make sim        closed-loop flight model + its tests"
	@echo "make firmware   host tests for the flight-critical logic"
	@echo "make target     build for the ESP32-S3"
	@echo "make e2e        whole chain under packet loss and corruption"
	@echo "make sweep      transport-delay stability sweep (writes the plot)"
	@echo "make verify     face-verification ROC (needs an enrolled gallery)"
	@echo "make quant      INT8 quantisation table (needs weights)"
	@echo "make protocol   regenerate the wire header and check it is current"
	@echo "make hygiene    no secrets, no biometrics, no blobs tracked"
	@echo "make docker     build and run the reproducible container"
	@echo "make demo       run the pipeline on the webcam (needs weights)"
	@echo ""
	@echo "make board      regenerate the PCB and run EVERY order gate"
	@echo "make board-gates  the gates only, without regenerating"

all: hygiene protocol vision sim firmware target
	@echo ""
	@echo "================= ALL GREEN ================="

# ---------------------------------------------------------------- board ---
# One command, in the one order that works. Encoded here because every step
# of it has been got wrong at least once:
#
#   * the two interpreters (see KPY/SPY above)
#   * zone fill MUST happen out of process — the in-process ZONE_FILLER
#     segfaults headless, so gen_pcb.py deliberately saves with zones unfilled
#     and `kicad-cli pcb drc --refill-zones --save-board` is what fills them
#   * NEVER PIPE A GATE. `kicad-cli ... --exit-code-violations | tail -3`
#     reports TAIL's exit code, not kicad-cli's. A run that returned 5 with
#     199 violations was once read as clean through exactly that mistake.
#     Every gate below writes to a file and is read afterwards.
#   * gerbers/BOM.csv is hand-maintained, so it has no generator to refresh
#     its mtime; preflight fails any file in gerbers/ older than the board.
DRC_OUT := /tmp/pursuit_drc.json
ERC_OUT := /tmp/pursuit_erc.json

board:
	@echo "--- 1/9  footprints (hole pitch asserted) ---"
	$(SPY) scripts/footprints.py
	@echo "--- 2/9  netlist audit ---"
	$(SPY) scripts/design.py
	@echo "--- 3/9  placement + edge + back-side gates ---"
	$(SPY) scripts/check_place.py
	@echo "--- 4/9  schematic ---"
	$(SPY) scripts/gen_sch.py
	@echo "--- 5/9  board ---"
	$(KPY) scripts/gen_pcb.py > /dev/null
	@$(MAKE) --no-print-directory board-gates

board-gates:
	@echo "--- 6/9  ERC (unpiped, exit code honoured) ---"
	@$(CLI) sch erc --severity-error --severity-warning \
	    --exit-code-violations -o $(ERC_OUT) --format json \
	    hardware/pursuit.kicad_sch > /tmp/pursuit_erc.txt 2>&1; \
	  rc=$$?; grep -i '^found' /tmp/pursuit_erc.txt; exit $$rc
	@echo "--- 7/9  DRC, zones refilled out of process, board saved ---"
	@$(CLI) pcb drc --severity-all --exit-code-violations \
	    --refill-zones --save-board -o $(DRC_OUT) --format json \
	    hardware/pursuit.kicad_pcb > /tmp/pursuit_drc.txt 2>&1; \
	  rc=$$?; grep -i '^found' /tmp/pursuit_drc.txt; exit $$rc
	@echo "--- 8/9  netlist parity, pour islands, plane continuity ---"
	$(SPY) scripts/check_sch.py
	$(KPY) scripts/audit_islands.py
	$(SPY) $(HWDIR)/check_ground.py hardware/pursuit.kicad_pcb --net GND
	$(SPY) $(HWDIR)/check_ground.py hardware/pursuit.kicad_pcb --net VBAT
	@echo "--- 9/9  release package + order gate ---"
	$(KPY) scripts/release.py
	$(KPY) scripts/gen_jlcpcb.py > /dev/null
	@touch gerbers/BOM.csv
	$(KPY) scripts/preflight.py

test: all

vision:
	@echo "--- vision suite ---"
	$(PY) -m pytest vision/tests -q

sim:
	@echo "--- flight simulation ---"
	$(PY) -m pytest sim/flight -q

firmware:
	@echo "--- firmware host tests ---"
	cd firmware && $(PIO) test -e native

target:
	@echo "--- ESP32-S3 build ---"
	cd firmware && $(PIO) run -e xiao

e2e:
	@echo "--- end-to-end, 10% loss + 5% corruption ---"
	$(PY) vision/tools/e2e_loop.py

sweep:
	$(PY) sim/flight/lag_sweep.py

verify:
	$(PY) vision/tools/verify_eval.py

quant:
	$(PY) vision/tools/quantize.py

protocol:
	@echo "--- generated wire header is current ---"
	@cp firmware/include/pursuit_link.h /tmp/pursuit_link.committed.h
	@$(PY) vision/gen_protocol_header.py >/dev/null
	@if diff -q /tmp/pursuit_link.committed.h firmware/include/pursuit_link.h \
	    >/dev/null; then \
	  echo "ok  header matches protocol.py"; \
	else \
	  echo "STALE — pursuit_link.h does not match protocol.py"; exit 1; \
	fi

hygiene:
	@echo "--- nothing that should never be public ---"
	@fail=0; \
	if git ls-files | grep -qE '\.npz$$'; then \
	  echo "FAIL a face embedding is tracked"; fail=1; fi; \
	if git ls-files | grep -qE 'firmware/include/secrets\.h$$'; then \
	  echo "FAIL secrets.h is tracked"; fail=1; fi; \
	if git ls-files | grep -qE '\.onnx$$'; then \
	  echo "FAIL a model blob is tracked"; fail=1; fi; \
	if [ $$fail -eq 0 ]; then echo "ok  clean"; fi; \
	exit $$fail

docker:
	docker build -t pursuit . && docker run --rm pursuit

demo:
	$(PY) vision/run.py --target $${TARGET:-you}

clean:
	rm -rf firmware/.pio vision/models/int8
	find . -name __pycache__ -type d -prune -exec rm -rf {} + 2>/dev/null || true
	find . -name .pytest_cache -type d -prune -exec rm -rf {} + 2>/dev/null || true
