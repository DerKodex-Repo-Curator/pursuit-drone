#!/usr/bin/env bash
# Default container action: prove the thing works, without weights or hardware.
set -euo pipefail

echo "=============================================================="
echo " PURSUIT-1 — off-board stack, CPU only, no models, no aircraft"
echo "=============================================================="
python -c "import sys, onnxruntime as ort, numpy, cv2
print(f'python           {sys.version.split()[0]}')
print(f'onnxruntime      {ort.__version__}')
print(f'providers        {\", \".join(ort.get_available_providers())}')
print(f'numpy            {numpy.__version__}')
print(f'opencv           {cv2.__version__}')"

echo
echo "--- vision suite (weights absent: model-dependent tests skip) ---"
python -m pytest vision/tests -q --no-header -rs

echo
echo "--- closed-loop flight simulation ---"
python -m pytest sim/flight -q --no-header

echo
echo "--- transport-delay stability sweep ---"
python sim/flight/lag_sweep.py --max-ms 68 --step-ms 34

echo
echo "=============================================================="
echo " All green. For the full pipeline: python vision/fetch_models.py"
echo "=============================================================="
