# PURSUIT-1 — one entry point for every check.
#
# `make` on its own runs everything that does not need hardware or model
# weights, which is the thing a reviewer wants and the thing CI runs.

PY      ?= vision/.venv/bin/python
PIO     ?= pio

.PHONY: help all test vision sim firmware target e2e sweep verify quant \
        protocol hygiene docker demo clean

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

all: hygiene protocol vision sim firmware target
	@echo ""
	@echo "================= ALL GREEN ================="

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
