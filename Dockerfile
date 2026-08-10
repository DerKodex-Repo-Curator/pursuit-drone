# PURSUIT-1 — reproducible environment for the off-board stack.
#
# This exists because the project was developed on Python 3.14 with CoreML on
# Apple silicon, which is a configuration nobody reviewing it will have. A
# reader should be able to run the tests and the simulation without matching
# any of that.
#
#   docker build -t pursuit .
#   docker run --rm pursuit                  # tests + the stability sweep
#   docker run --rm pursuit pytest vision/tests -q
#
# Deliberately CPU-only. There is no CoreML inside a Linux container and no GPU
# assumed, so ONNX Runtime falls back to the CPU execution provider — which is
# exactly the path `pursuit/onnxbase.py` was written to degrade to, and running
# it here proves that degradation works.
#
# The ~309 MB of model weights are NOT baked in. They are not redistributable
# under their own licences (the InsightFace models are non-commercial research
# use only), they would make this image enormous, and everything that does not
# need weights — the protocol, the link, the lock machine, the arbiter, the
# control maths, the geometry, the whole flight simulation — runs without them.
# `python vision/fetch_models.py` inside the container if you want the rest.

FROM python:3.12-slim

# libGL and libglib are OpenCV's runtime dependencies; the slim image has
# neither, and `import cv2` fails with an import error that names a .so rather
# than the package that actually needs installing.
RUN apt-get update && apt-get install -y --no-install-recommends \
        libgl1 \
        libglib2.0-0 \
        curl \
        ca-certificates \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /pursuit

# Pinned. An unpinned reproducibility container reproduces whatever PyPI
# happened to be serving that morning, which is the opposite of the point.
COPY requirements-docker.txt .
RUN pip install --no-cache-dir -r requirements-docker.txt

COPY . .

ENV PYTHONPATH=/pursuit:/pursuit/vision
ENV PURSUIT_HEADLESS=1

CMD ["bash", "docker-entrypoint.sh"]
