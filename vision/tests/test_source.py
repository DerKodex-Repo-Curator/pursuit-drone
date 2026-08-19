"""Which sources get mirrored, and which must never be.

A laptop webcam faces the operator, so mirroring it makes left/right read
naturally while testing. The drone's camera faces AWAY from the operator and is
already in the airframe's frame of reference. Mirroring that feed puts a target
on the aircraft's left into the right half of the image, and every lateral
correction is then computed with the wrong sign -- the aircraft steers away
from the person it is following, faster the further off-centre they get.

This was a live bug until 2026-08-20: the flip was gated on `live`, which is
true for a network stream as well as a webcam. It was never caught because the
pipeline had only ever been run against a laptop webcam. The tests below exist
so that pointing --source at the aircraft cannot silently invert control again.
"""
from __future__ import annotations

import os
import sys
from unittest import mock

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import run                                                     # noqa: E402
from pursuit.config import Config                              # noqa: E402


class _FakeCap:
    """Minimal cv2.VideoCapture stand-in: opened, and reports its mode."""

    def __init__(self, *_a, **_k):
        pass

    def isOpened(self):
        return True

    def set(self, *_a):
        return True

    def get(self, _prop):
        return 0


@pytest.fixture
def cfg():
    return Config()


def _open(cfg, source):
    with mock.patch.object(run.cv2, "VideoCapture", _FakeCap):
        return run.open_source(cfg, source)


@pytest.mark.parametrize("url", [
    "http://192.168.4.1:81/",
    "https://192.168.4.1:81/stream",
    "rtsp://192.168.4.1/live",
    "udp://0.0.0.0:5000",
])
def test_a_network_stream_is_never_mirrored(cfg, url):
    """The aircraft's own view. Mirroring it inverts every lateral command."""
    _cap, _label, live, mirror = _open(cfg, url)
    assert live is True          # it IS live — that is not the question
    assert mirror is False


def test_the_default_webcam_is_mirrored(cfg):
    cfg.source.flip = True
    _cap, _label, live, mirror = _open(cfg, None)
    assert (live, mirror) == (True, True)


def test_an_indexed_webcam_is_mirrored(cfg):
    cfg.source.flip = True
    _cap, _label, live, mirror = _open(cfg, "1")
    assert (live, mirror) == (True, True)


def test_flip_false_disables_webcam_mirroring(cfg):
    cfg.source.flip = False
    _cap, _label, _live, mirror = _open(cfg, None)
    assert mirror is False


def test_flip_true_still_cannot_mirror_a_stream(cfg):
    """The config flag is about the webcam. It must not reach the aircraft."""
    cfg.source.flip = True
    _cap, _label, _live, mirror = _open(cfg, "http://192.168.4.1:81/")
    assert mirror is False


def test_a_video_file_is_neither_live_nor_mirrored(cfg):
    cfg.source.flip = True
    _cap, _label, live, mirror = _open(cfg, "clip.mp4")
    assert (live, mirror) == (False, False)


def test_a_configured_url_beats_the_webcam_default(cfg):
    """cfg.source.url is how the drone is wired in without a --source flag."""
    cfg.source.url = "http://192.168.4.1:81/"
    cfg.source.flip = True
    _cap, label, _live, mirror = _open(cfg, None)
    assert label.startswith("http://")
    assert mirror is False
