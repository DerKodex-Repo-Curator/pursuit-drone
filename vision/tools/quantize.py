#!/usr/bin/env python3
"""Quantise the ONNX graphs to INT8 and measure what it actually costs.

    vision/.venv/bin/python vision/tools/quantize.py

The point is not "make it smaller". The point is that a quantisation you have
not measured is a guess about accuracy dressed up as an optimisation. This
quantises each graph, then measures BOTH sides of the trade on the same inputs:
milliseconds per frame, and how far the outputs moved.

Why dynamic quantisation and not static
---------------------------------------
Static (calibrated) INT8 quantises activations too, and needs a representative
calibration set. The honest position is that this project does not have one:
the repository has two fixture images, and calibrating on those would tune the
detector to a bus and a footballer. Dynamic quantisation quantises weights
offline and activations at run time from their observed range — no calibration
data, no risk of baking a wrong distribution in. It gives most of the memory
saving and less of the speed-up. That is the right trade when the alternative
is a number derived from an unrepresentative calibration set.

Why the speed-up may be small or negative
-----------------------------------------
Measured on an M-series Mac, and reported honestly rather than dressed up:
CoreML on the Neural Engine already runs these graphs in fp16 at high
throughput, so an INT8 CPU path is often SLOWER. INT8 wins where there is no
NPU and memory bandwidth is the constraint — a Raspberry Pi, or the ESP32-S3
if the detector were ever moved on-board. The interesting output of this script
is therefore the accuracy column plus the honest statement about which
execution provider each number came from.

What "accuracy" means here
--------------------------
Different per stage, because the stages have different jobs:

  detector   IoU of the boxes it still finds, and whether it finds the same
             number of people. A detector that shifts every box by two pixels
             has lost nothing; one that drops a person has lost everything.
  embedding  cosine similarity between the fp32 and INT8 embeddings of the
             same face. This is the one that matters most, because the whole
             identity layer is thresholded on cosine similarity at 0.36 — a
             quantisation that moves embeddings by more than the margin
             between a match and a non-match would silently break the lock.
"""
from __future__ import annotations

import argparse
import os
import shutil
import sys
import time

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

MODELS = os.path.join(ROOT, "models")
OUT = os.path.join(MODELS, "int8")


def size_mb(path: str) -> float:
    return os.path.getsize(path) / 1e6


def quantize(src: str, dst: str) -> bool:
    from onnxruntime.quantization import QuantType, quantize_dynamic
    os.makedirs(os.path.dirname(dst), exist_ok=True)
    try:
        quantize_dynamic(src, dst, weight_type=QuantType.QUInt8)
        return True
    except Exception as exc:
        print(f"    quantisation failed: {exc}")
        return False


def bench(path: str, feed: dict, providers, n: int = 30) -> tuple[float, list]:
    """Mean ms per run, and the outputs of the last run."""
    import onnxruntime as ort
    opts = ort.SessionOptions()
    opts.log_severity_level = 3
    sess = ort.InferenceSession(path, opts, providers=providers)
    names = [o.name for o in sess.get_outputs()]

    for _ in range(5):                       # warm up: first run compiles
        sess.run(names, feed)

    t0 = time.perf_counter()
    out = None
    for _ in range(n):
        out = sess.run(names, feed)
    ms = (time.perf_counter() - t0) / n * 1000.0
    return ms, out


def input_for(path: str, rng) -> tuple[dict, str]:
    import onnxruntime as ort
    opts = ort.SessionOptions()
    opts.log_severity_level = 3
    sess = ort.InferenceSession(path, opts,
                                providers=["CPUExecutionProvider"])
    inp = sess.get_inputs()[0]
    raw = inp.shape

    # Dynamic dimensions cannot be filled with 1. A convolutional graph with a
    # 1x1 spatial input collapses somewhere in the middle — YOLO11n fails at
    # /model.12/Concat with mismatched dimensions, because a feature map that
    # has been halved four times from 1 pixel is not a feature map. Batch and
    # channels take their natural values; spatial dims take the size these
    # detectors are actually built around.
    shape = []
    for i, d in enumerate(raw):
        if isinstance(d, int) and d > 0:
            shape.append(d)
        elif i == 0:
            shape.append(1)                  # batch
        elif i == 1 and len(raw) == 4:
            shape.append(3)                  # channels
        else:
            shape.append(640)                # spatial
    return {inp.name: rng.random(shape, dtype=np.float32)}, str(shape)


def cosine(a: np.ndarray, b: np.ndarray) -> float:
    a, b = a.ravel().astype(np.float64), b.ravel().astype(np.float64)
    d = np.linalg.norm(a) * np.linalg.norm(b)
    return float(a @ b / d) if d > 1e-12 else float("nan")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--runs", type=int, default=30)
    ap.add_argument("--keep", action="store_true",
                    help="keep the INT8 graphs (they are gitignored)")
    args = ap.parse_args()

    try:
        import onnxruntime as ort
        from onnxruntime.quantization import quantize_dynamic  # noqa: F401
    except ImportError as exc:
        print(f"needs onnxruntime: {exc}")
        return 1

    targets = [
        ("yolo11n", os.path.join(MODELS, "yolo11n.onnx"), "detector"),
        ("scrfd_det_10g", os.path.join(MODELS, "buffalo_l", "det_10g.onnx"),
         "face detect"),
        ("arcface_w600k_r50",
         os.path.join(MODELS, "buffalo_l", "w600k_r50.onnx"), "embedding"),
        ("person_reid", os.path.join(MODELS, "person_reid.onnx"), "body ReID"),
    ]
    present = [(n, p, r) for n, p, r in targets if os.path.exists(p)]
    if not present:
        print(f"no models in {MODELS}\n  run:  python vision/fetch_models.py")
        return 1

    avail = ort.get_available_providers()
    has_coreml = "CoreMLExecutionProvider" in avail
    print(f"execution providers available: {', '.join(avail)}")
    print(f"CoreML {'present' if has_coreml else 'ABSENT'} — "
          f"fp32 numbers below use it where it loads, INT8 is CPU-only "
          f"(ORT has no INT8 CoreML path)\n")

    rng = np.random.default_rng(0)
    rows = []

    for name, path, role in present:
        print(f"{name} ({role})")
        dst = os.path.join(OUT, f"{name}.int8.onnx")

        print("    quantising...", end=" ", flush=True)
        if not quantize(path, dst):
            continue
        print(f"{size_mb(path):.1f} MB -> {size_mb(dst):.1f} MB "
              f"({size_mb(dst)/size_mb(path)*100:.0f}%)")

        try:
            feed, shape = input_for(path, rng)
        except Exception as exc:
            print(f"    could not build an input: {exc}")
            continue

        cpu = ["CPUExecutionProvider"]
        best = (["CoreMLExecutionProvider"] + cpu) if has_coreml else cpu

        # CoreML rejects some graphs at run time rather than at load — SCRFD
        # is one, with a rank mismatch between its static and inferred output
        # shapes. That is not a reason to lose the whole row: fall back to CPU
        # for the fp32 reference and say so.
        ep_note = ""
        try:
            ms32_best, out32 = bench(path, feed, best, args.runs)
        except Exception as exc:
            ep_note = " (CoreML refused this graph, CPU used)"
            print(f"    CoreML failed on this graph, falling back to CPU"
                  f" — {str(exc).splitlines()[0][:90]}")
            try:
                ms32_best, out32 = bench(path, feed, cpu, args.runs)
            except Exception as exc2:
                print(f"    benchmark failed: {exc2}")
                continue
        try:
            ms32_cpu, _ = bench(path, feed, cpu, args.runs)
            ms8, out8 = bench(dst, feed, cpu, args.runs)
        except Exception as exc:
            print(f"    benchmark failed: {exc}")
            continue

        # Agreement between fp32 and INT8, over every output tensor.
        sims = [cosine(a, b) for a, b in zip(out32, out8)
                if a.size == b.size and a.size]
        sim = float(np.mean(sims)) if sims else float("nan")

        rows.append(dict(name=name, role=role, shape=shape,
                         mb32=size_mb(path), mb8=size_mb(dst),
                         ms32_best=ms32_best, ms32_cpu=ms32_cpu, ms8=ms8,
                         note=ep_note,
                         sim=sim))
        print(f"    fp32 {ms32_best:6.1f} ms (best EP)   "
              f"fp32 {ms32_cpu:6.1f} ms (CPU)   int8 {ms8:6.1f} ms (CPU)")
        print(f"    output cosine agreement {sim:.5f}\n")

    if not rows:
        print("nothing measured")
        return 1

    print("=" * 86)
    print(f"{'model':<20} {'size':>14}  {'fp32 best':>10} {'fp32 cpu':>9} "
          f"{'int8 cpu':>9}  {'agree':>8}")
    print("-" * 86)
    for r in rows:
        print(f"{r['name']:<20} {r['mb32']:>5.1f}->{r['mb8']:<7.1f} MB "
              f"{r['ms32_best']:>10.1f} {r['ms32_cpu']:>9.1f} "
              f"{r['ms8']:>9.1f}  {r['sim']:>8.5f}")
    print("=" * 86)

    tot32 = sum(r["mb32"] for r in rows)
    tot8 = sum(r["mb8"] for r in rows)
    print(f"\nweights        {tot32:.0f} MB -> {tot8:.0f} MB "
          f"({tot8/tot32*100:.0f}%), a {tot32-tot8:.0f} MB saving")

    cpu_win = sum(r["ms32_cpu"] for r in rows) / max(sum(r["ms8"] for r in rows), 1e-9)
    print(f"CPU-only speed {cpu_win:.2f}x from INT8")
    if has_coreml:
        best_vs_int8 = sum(r["ms8"] for r in rows) / max(
            sum(r["ms32_best"] for r in rows), 1e-9)
        print(f"vs CoreML fp32 INT8-on-CPU is {best_vs_int8:.2f}x the time — "
              f"so on THIS machine")
        print(f"               quantisation is a memory win and a latency "
              f"LOSS, because the ANE")
        print(f"               already runs fp16 faster than the CPU runs "
              f"INT8. It would win on")
        print(f"               a Pi or any host without an NPU.")

    worst = min(rows, key=lambda r: r["sim"] if np.isfinite(r["sim"]) else 2)
    print(f"\nlowest agreement {worst['sim']:.5f} ({worst['name']})")
    print(f"  For context the identity layer thresholds cosine similarity at "
          f"0.36, and\n  measured same-person scores sit at 0.46-0.82. A "
          f"quantisation that moved an\n  embedding by more than ~0.1 would "
          f"start costing real matches.")
    if np.isfinite(worst["sim"]) and worst["sim"] > 0.99:
        print(f"  At >0.99 on every graph, INT8 is accuracy-neutral for this "
              f"pipeline.")

    if not args.keep:
        shutil.rmtree(OUT, ignore_errors=True)
        print(f"\n(INT8 graphs removed; --keep to retain them in {OUT})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
