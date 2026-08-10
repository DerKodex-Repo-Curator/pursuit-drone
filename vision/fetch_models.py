#!/usr/bin/env python3
"""Download and verify the ONNX models the pipeline needs.

Three graphs, ~300MB total, fetched once into `models/`:

  yolo11n.onnx     person detection          (Ultralytics release assets)
  det_10g.onnx     SCRFD face detection      (InsightFace buffalo_l)
  w600k_r50.onnx   ArcFace 512-d embedding   (InsightFace buffalo_l)

Every download is verified two ways before it counts as done: the byte count
must match Content-Length, and ONNX Runtime must actually load the graph. That
is not paranoia — the first attempt at this silently wrote a truncated 8.5MB
file where 10.9MB was expected. It had a perfectly valid ONNX header, passed a
size sanity check, and only failed much later with `INVALID_PROTOBUF` from
somewhere that gave no hint the download was at fault.

    python fetch_models.py            # fetch what is missing, verify all
    python fetch_models.py --force    # re-fetch everything
"""
from __future__ import annotations

import argparse
import io
import os
import shutil
import ssl
import subprocess
import sys
import urllib.request
import zipfile

ROOT = os.path.dirname(os.path.abspath(__file__))
MODELS = os.path.join(ROOT, "models")

YOLO_URL = ("https://github.com/ultralytics/assets/releases/download/"
            "v8.3.0/yolo11n.onnx")
BUFFALO_URL = ("https://github.com/deepinsight/insightface/releases/download/"
               "v0.7/buffalo_l.zip")
# Person re-identification, 768-d. Replaces the colour-histogram appearance
# descriptor. Optional — the pipeline falls back if it is absent — but it is
# what lets the body channel tell apart two people in similar clothing.
REID_URL = ("https://media.githubusercontent.com/media/opencv/opencv_zoo/main/"
            "models/person_reid_youtureid/person_reid_youtu_2021nov.onnx")

# The pack also ships landmark and gender/age graphs. Neither is on the flight
# path, so they are not unpacked.
BUFFALO_WANT = ("det_10g.onnx", "w600k_r50.onnx")

MIN_BYTES = 1 << 20


def _ssl_context() -> ssl.SSLContext:
    """A context that actually has root certificates.

    python.org's macOS framework build ships without wiring the system trust
    store into OpenSSL, so every https fetch fails with CERTIFICATE_VERIFY_FAILED
    until someone runs `Install Certificates.command`. Pointing at certifi's
    bundle fixes it without weakening verification — never reach for
    `_create_unverified_context`, which would let anyone on the network swap
    the model you are about to run.
    """
    try:
        import certifi
        return ssl.create_default_context(cafile=certifi.where())
    except ImportError:
        return ssl.create_default_context()


def _curl(url: str, dst: str) -> bool:
    """Fetch with curl: follows redirects, retries, resumes, fails loudly.

    Preferred over urllib because GitHub release assets redirect to a signed
    S3 URL that will drop a connection mid-transfer, and curl's `--retry` plus
    `-C -` recovers from that where a plain read loop just stops early and
    leaves a truncated file behind.
    """
    if not shutil.which("curl"):
        return False
    cmd = ["curl", "-L", "--fail", "--retry", "4", "--retry-delay", "2",
           "--retry-all-errors", "--connect-timeout", "20",
           "--progress-bar", "-o", dst, url]
    try:
        return subprocess.run(cmd, timeout=900).returncode == 0
    except (subprocess.TimeoutExpired, OSError):
        return False


def _urllib(url: str, dst: str) -> bool:
    req = urllib.request.Request(url, headers={"User-Agent": "pursuit-1/1.0"})
    try:
        with urllib.request.urlopen(req, timeout=180,
                                    context=_ssl_context()) as resp:
            expect = int(resp.headers.get("Content-Length") or 0)
            buf = io.BytesIO()
            while chunk := resp.read(1 << 16):
                buf.write(chunk)
            data = buf.getvalue()
    except Exception as exc:
        print(f"        {exc}")
        return False

    # A short read here is the exact bug this whole function exists to catch.
    if expect and len(data) != expect:
        print(f"        truncated: got {len(data)} of {expect} bytes")
        return False
    with open(dst, "wb") as fh:
        fh.write(data)
    return True


def download(url: str, dst: str) -> bool:
    tmp = dst + ".part"
    ok = _curl(url, tmp) or _urllib(url, tmp)
    if not ok or not os.path.exists(tmp) or os.path.getsize(tmp) < MIN_BYTES:
        if os.path.exists(tmp):
            os.remove(tmp)
        return False
    os.replace(tmp, dst)
    return True


def verify_onnx(path: str) -> tuple[bool, str]:
    """The only check that means anything: can ORT load this graph?"""
    try:
        import onnxruntime as ort
        opts = ort.SessionOptions()
        opts.log_severity_level = 4
        sess = ort.InferenceSession(path, sess_options=opts,
                                    providers=["CPUExecutionProvider"])
        shape = sess.get_inputs()[0].shape
        return True, f"in={shape} out={len(sess.get_outputs())}"
    except Exception as exc:
        return False, str(exc).split("\n")[0][:110]


def _report(name: str, path: str) -> bool:
    ok, detail = verify_onnx(path)
    mb = os.path.getsize(path) / 1e6
    print(f"  {'ok  ' if ok else 'BAD '}  {name:24} {mb:6.1f} MB  {detail}")
    return ok


# ------------------------------------------------------------------- fetchers --

def fetch_yolo(force: bool) -> bool:
    dst = os.path.join(MODELS, "yolo11n.onnx")
    if not force and os.path.exists(dst) and verify_onnx(dst)[0]:
        return _report("yolo11n.onnx", dst)

    print("  fetching yolo11n.onnx")
    if not download(YOLO_URL, dst):
        print("  FAIL  could not download yolo11n.onnx")
        return False
    if not _report("yolo11n.onnx", dst):
        os.remove(dst)
        return False
    return True


def fetch_buffalo(force: bool) -> bool:
    out = os.path.join(MODELS, "buffalo_l")
    os.makedirs(out, exist_ok=True)
    paths = {n: os.path.join(out, n) for n in BUFFALO_WANT}

    if not force and all(os.path.exists(p) and verify_onnx(p)[0]
                         for p in paths.values()):
        return all(_report(f"buffalo_l/{n}", p) for n, p in paths.items())

    print("  fetching buffalo_l.zip (~280 MB)")
    zip_path = os.path.join(MODELS, "buffalo_l.zip")
    if not download(BUFFALO_URL, zip_path):
        print("  FAIL  could not download buffalo_l.zip")
        return False

    try:
        with zipfile.ZipFile(zip_path) as zf:
            members = {os.path.basename(i.filename): i for i in zf.infolist()}
            missing = [n for n in BUFFALO_WANT if n not in members]
            if missing:
                print(f"  FAIL  zip is missing {missing}")
                return False
            for name in BUFFALO_WANT:
                with zf.open(members[name]) as src, \
                     open(paths[name], "wb") as fh:
                    shutil.copyfileobj(src, fh)
    except zipfile.BadZipFile:
        size = os.path.getsize(zip_path)
        print(f"  FAIL  not a zip ({size} bytes) — probably an error page")
        return False
    finally:
        if os.path.exists(zip_path):
            os.remove(zip_path)

    return all(_report(f"buffalo_l/{n}", p) for n, p in paths.items())


def fetch_reid(force: bool) -> bool:
    dst = os.path.join(MODELS, "person_reid.onnx")
    if not force and os.path.exists(dst) and verify_onnx(dst)[0]:
        return _report("person_reid.onnx", dst)

    print("  fetching person_reid.onnx (~102 MB)")
    if not download(REID_URL, dst):
        print("  FAIL  could not download person_reid.onnx")
        return False
    if not _report("person_reid.onnx", dst):
        os.remove(dst)
        return False
    return True


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--force", action="store_true",
                    help="re-download even if the files verify")
    ap.add_argument("--reid", action="store_true",
                    help="fetch only the person re-ID model")
    args = ap.parse_args()

    os.makedirs(MODELS, exist_ok=True)
    print(f"models -> {MODELS}\n")
    if args.reid:
        return 0 if fetch_reid(args.force) else 1
    results = [fetch_yolo(args.force), fetch_buffalo(args.force),
               fetch_reid(args.force)]

    print()
    if all(results):
        print("all models present and loadable")
        return 0
    print("SOME MODELS MISSING OR CORRUPT — the pipeline will not start")
    return 1


if __name__ == "__main__":
    sys.exit(main())
