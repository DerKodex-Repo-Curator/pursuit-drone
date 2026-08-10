"""End-to-end checks against real imagery.

Everything else in the suite is unit-level and runs on synthetic boxes. This
file is the one that would catch a pipeline that is individually correct
everywhere and useless as a whole — a letterbox inverse that shifts every box
by ten pixels, a face crop that never aligns, a lock that acquires on whoever
is nearest rather than whoever is enrolled.

Slow (loads ~200MB of ONNX), so it is marked and can be deselected:

    pytest tests/ -m "not slow"
"""
from __future__ import annotations

import os
import sys

import cv2
import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from pursuit.config import Config                                # noqa: E402
from pursuit.detect import PersonDetector                        # noqa: E402
from pursuit.face import FaceAnalyzer, align_face, umeyama       # noqa: E402
from pursuit.gallery import Gallery                              # noqa: E402
from pursuit.lock import LockState                               # noqa: E402
from pursuit.pipeline import Pipeline                            # noqa: E402

DATA = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data")
MODELS_PRESENT = os.path.exists(
    os.path.join(os.path.dirname(DATA), "..", "models", "yolo11n.onnx"))

pytestmark = [
    pytest.mark.slow,
    pytest.mark.skipif(not os.path.isdir(DATA) or not os.listdir(DATA),
                       reason="run fetch_models.py and download test images"),
]


@pytest.fixture(scope="module")
def cfg():
    return Config()


@pytest.fixture(scope="module")
def detector(cfg):
    return PersonDetector(cfg.detector, cfg.providers)


@pytest.fixture(scope="module")
def analyzer(cfg):
    return FaceAnalyzer(cfg.face, cfg.providers)


def img(name):
    path = os.path.join(DATA, name)
    im = cv2.imread(path)
    assert im is not None, f"missing test image {path}"
    return im


# --------------------------------------------------------------- geometry ---

def test_umeyama_recovers_a_known_transform():
    """If alignment is wrong, ArcFace silently loses about half its accuracy,
    so the transform gets a closed-form check rather than an eyeball."""
    src = np.array([[0, 0], [1, 0], [1, 1], [0, 1], [.5, .5]], dtype=np.float64)
    angle, scale, shift = np.deg2rad(23.0), 2.5, np.array([7.0, -3.0])
    rot = np.array([[np.cos(angle), -np.sin(angle)],
                    [np.sin(angle), np.cos(angle)]])
    dst = (scale * src @ rot.T) + shift

    m = umeyama(src, dst)
    got = (np.hstack([src, np.ones((len(src), 1))]) @ m.T)
    assert np.allclose(got, dst, atol=1e-6)


def test_detector_boxes_land_on_the_subject(detector):
    """A broken letterbox inverse still produces plausible-looking boxes —
    just in the wrong place. Pin them against known content."""
    im = img("zidane.jpg")
    people = detector(im)
    assert len(people) == 2, f"expected 2 people, got {len(people)}"

    h, w = im.shape[:2]
    for p in people:
        assert 0 <= p.xyxy[0] < p.xyxy[2] <= w
        assert 0 <= p.xyxy[1] < p.xyxy[3] <= h
        # Both subjects fill most of the frame vertically, so a box much
        # shorter than this would mean the letterbox inverse is off.
        assert p.h > h * 0.5, "box far too short to be either subject"

    # Deliberately no aspect-ratio assertion. One subject here has his arms
    # stretched wide, giving a box 993px across and 511px tall — a person is
    # only reliably "taller than wide" if they keep their arms down. This is
    # the concrete reason the controller derives distance from box HEIGHT and
    # never from width or area.
    widest = max(people, key=lambda p: p.w)
    assert widest.w > widest.h, "expected the outstretched-arms subject here"


def test_detector_finds_the_whole_group(detector):
    people = detector(img("bus.jpg"))
    assert len(people) >= 4, f"only found {len(people)} of 4 people"


def test_detector_ignores_noise(detector):
    rng = np.random.default_rng(0)
    noise = (rng.random((720, 1280, 3)) * 255).astype(np.uint8)
    assert detector(noise) == []


# ------------------------------------------------------------------ faces ---

def test_faces_are_found_and_aligned(analyzer):
    faces = analyzer.detect(img("zidane.jpg"))
    assert len(faces) == 2
    for f in faces:
        assert f.kps.shape == (5, 2)
        crop = align_face(img("zidane.jpg"), f.kps, 112)
        assert crop.shape == (112, 112, 3)
        assert crop.std() > 10, "aligned crop is flat — alignment missed"


def test_embeddings_separate_two_people(analyzer):
    """The claim the whole product rests on: this is the enrolled friend and
    that is a stranger."""
    faces = analyzer.detect_and_embed(img("zidane.jpg"))
    assert len(faces) == 2
    a, b = faces[0].embedding, faces[1].embedding

    assert np.isclose(np.linalg.norm(a), 1.0, atol=1e-4)
    assert np.isclose(np.linalg.norm(b), 1.0, atol=1e-4)

    between = float(a @ b)
    assert between < Config().face.match_thres, \
        f"two different people scored {between:.3f} — would follow the wrong one"


@pytest.mark.parametrize("scale,blur,gain,floor", [
    (0.6, 0, 1.0, 0.75),      # further away
    (0.4, 3, 1.0, 0.60),      # further still, and soft
    (1.0, 0, 0.65, 0.75),     # underexposed
    (1.0, 5, 1.25, 0.70),     # motion blur, bright
])
def test_identity_survives_flight_conditions(analyzer, scale, blur, gain, floor):
    """A drone never sees the enrolment photo again — it sees small, blurred,
    badly-lit versions. The margin has to survive that, not just a still."""
    im = img("zidane.jpg")
    ref = analyzer.detect_and_embed(im)
    ref.sort(key=lambda f: -f.w * f.h)
    target, other = ref[0].embedding, ref[1].embedding

    warped = cv2.resize(im, None, fx=scale, fy=scale)
    if blur:
        warped = cv2.GaussianBlur(warped, (blur, blur), 0)
    warped = np.clip(warped.astype(np.float32) * gain, 0, 255).astype(np.uint8)

    faces = analyzer.detect_and_embed(warped)
    assert faces, "lost the face entirely under these conditions"

    sims = sorted((float(target @ f.embedding) for f in faces), reverse=True)
    assert sims[0] >= floor, f"same person dropped to {sims[0]:.3f}"
    if len(sims) > 1:
        assert sims[1] < Config().face.match_thres

    # The margin, not just the threshold, is what makes this safe in the air.
    assert sims[0] - float(target @ other) > 0.35


# ------------------------------------------------------------- face ROIs ---

def test_roi_detection_finds_the_same_faces(detector, analyzer):
    im = img("zidane.jpg")
    people = detector(im)
    rois = [analyzer.head_roi(p.xyxy, im.shape[1], im.shape[0])
            for p in people]

    full = analyzer.detect(im)
    roi = analyzer.detect_in_rois(im, rois)
    assert len(roi) == len(full) == 2, \
        f"ROI pass found {len(roi)} faces, full frame found {len(full)}"


def test_roi_detection_preserves_identity_separation(analyzer):
    """The ROI crop shifts boxes and landmarks slightly, so embeddings are not
    identical to the full-frame ones. What has to survive is the SEPARATION —
    the same person must still clear the match threshold and a stranger must
    still fall under the veto floor."""
    im = img("zidane.jpg")
    cfg = Config()

    enrolled = analyzer.detect_and_embed(im)
    assert len(enrolled) == 2

    from pursuit.detect import PersonDetector
    people = PersonDetector(cfg.detector, cfg.providers)(im)
    rois = [analyzer.head_roi(p.xyxy, im.shape[1], im.shape[0])
            for p in people]
    found = analyzer.detect_in_rois(im, rois)
    for f in found:
        analyzer.embed(im, f)
    assert len(found) == 2

    for ref in enrolled:
        # Pair by geometry, not by size rank: ROI boxes differ enough from
        # full-frame ones that sorting on area can silently swap the two
        # people and make a passing result look like a failure.
        scores = sorted((float(ref.embedding @ f.embedding) for f in found),
                        reverse=True)
        assert scores[0] >= cfg.face.match_thres, \
            f"same person fell to {scores[0]:.3f} through the ROI path"
        assert scores[1] < cfg.face.veto_thres, \
            f"a stranger reached {scores[1]:.3f} through the ROI path"
        assert scores[0] - scores[1] > 0.5


def test_head_roi_covers_the_face(detector, analyzer):
    """A crop that misses the head returns a confident 'nobody here' rather
    than an obvious error, so check the geometry directly."""
    im = img("zidane.jpg")
    people = detector(im)
    faces = analyzer.detect(im)
    h, w = im.shape[:2]

    for face in faces:
        fcx = (face.xyxy[0] + face.xyxy[2]) * 0.5
        fcy = (face.xyxy[1] + face.xyxy[3]) * 0.5
        owner = None
        for p in people:
            if p.xyxy[0] <= fcx <= p.xyxy[2] and p.xyxy[1] <= fcy <= p.xyxy[3]:
                owner = p
                break
        assert owner is not None, "face sits inside no person box"
        rx1, ry1, rx2, ry2 = analyzer.head_roi(owner.xyxy, w, h)
        assert rx1 <= fcx <= rx2 and ry1 <= fcy <= ry2, \
            "head ROI does not contain the face it should"


# ------------------------------------------------------------------ reid ---

@pytest.fixture(scope="module")
def reid(cfg):
    from pursuit import reid as reid_mod
    model = reid_mod.load(cfg.body, cfg.providers)
    if model is None:
        pytest.skip("person_reid.onnx not fetched")
    return model


def test_reid_embeddings_are_unit_length(reid, detector):
    im = img("bus.jpg")
    people = detector(im)
    vecs = reid.embed_many(im, [p.xyxy for p in people])
    assert all(v is not None for v in vecs)
    for v in vecs:
        assert v.shape == (768,)
        assert np.isclose(np.linalg.norm(v), 1.0, atol=1e-4)


def test_reid_beats_the_histogram_on_similar_clothing(reid, detector, cfg):
    """The reason the learned model was added, stated as a measurement.

    Four people in similar dark clothing. The colour histogram scores one
    pair at 0.655 — above its own 0.55 threshold, so it would confirm the
    wrong person as the target. The learned descriptor keeps every different
    pair below its threshold.
    """
    from pursuit import body as body_mod

    im = img("bus.jpg")
    people = detector(im)
    assert len(people) >= 4, "need the crowded frame for this comparison"

    embeds = reid.embed_many(im, [p.xyxy for p in people])
    hists = [body_mod.describe(im, p.xyxy, cfg.body) for p in people]

    n = len(people)
    worst_reid = max(float(embeds[i] @ embeds[j])
                     for i in range(n) for j in range(n) if i != j)
    worst_hist = max(body_mod.similarity(hists[i], hists[j])
                     for i in range(n) for j in range(n) if i != j)

    assert worst_hist > cfg.body.match_thres, \
        ("the histogram no longer confuses anyone in this image, so this "
         "comparison has stopped testing anything")
    assert worst_reid < cfg.body.reid_match_thres, \
        f"learned re-ID confused two different people at {worst_reid:.3f}"
    assert worst_reid < worst_hist


def test_reid_batches_consistently(reid, detector):
    """Batched and single-crop embedding must agree, or the appearance model
    would depend on how many people happened to be in frame."""
    im = img("bus.jpg")
    people = detector(im)[:3]
    batched = reid.embed_many(im, [p.xyxy for p in people])
    for person, want in zip(people, batched):
        got = reid.embed(im, person.xyxy)
        assert float(got @ want) > 0.999


def test_reid_handles_degenerate_boxes(reid):
    im = img("bus.jpg")
    out = reid.embed_many(im, [
        np.array([0, 0, 2, 2], dtype=np.float32),        # far too small
        np.array([-50, -50, 10, 10], dtype=np.float32),   # mostly off-frame
    ])
    assert out[0] is None
    assert all(v is None or np.isfinite(v).all() for v in out)


# --------------------------------------------------------------- pipeline ---

def pan_sequence(im, frames=45, amplitude=340):
    """Slide the scene across the frame so the subject crosses the centre.

    Exercises detection, tracking, the lock and the control loop together, and
    gives the yaw command a sign change to be checked against.
    """
    h, w = im.shape[:2]
    out = []
    for i in range(frames):
        dx = int(amplitude * np.sin(2 * np.pi * i / frames))
        m = np.float32([[1, 0, dx], [0, 1, 0]])
        out.append(cv2.warpAffine(im, m, (w, h), borderMode=cv2.BORDER_REPLICATE))
    return out


def test_full_pipeline_locks_and_steers(tmp_path, analyzer):
    im = img("zidane.jpg")

    # Enrol the larger of the two subjects.
    faces = analyzer.detect_and_embed(im)
    faces.sort(key=lambda f: -f.w * f.h)
    gallery = Gallery(str(tmp_path))
    gallery.save("target", faces[0].embedding[None, :])

    cfg = Config()
    cfg.gallery = str(tmp_path)
    cfg.link.enabled = False
    pipe = Pipeline(cfg, target_name="target", quiet=True)

    states, yaws, centres = [], [], []
    for frame in pan_sequence(im):
        r = pipe.process(frame)
        states.append(r.lock.state)
        yaws.append(r.command.yaw_rate)
        centres.append(None if not r.lock.has_target else
                       float((r.lock.xyxy[0] + r.lock.xyxy[2]) * 0.5))

    assert LockState.LOCKED in states, "never acquired the enrolled target"
    locked = states.count(LockState.LOCKED)
    assert locked > len(states) * 0.5, \
        f"only held lock for {locked}/{len(states)} frames"

    # Yaw must actually respond, and in the right direction: while the target
    # is right of centre the command should be positive, and vice versa.
    half = im.shape[1] / 2
    agree = total = 0
    for cx, yaw in zip(centres, yaws):
        if cx is None or abs(cx - half) < 90:
            continue
        total += 1
        agree += (cx > half) == (yaw > 0)
    assert total > 5, "target never moved far enough off centre to judge"
    assert agree / total > 0.85, \
        f"yaw pointed the wrong way in {total-agree}/{total} frames"


def test_pipeline_ignores_an_unenrolled_person(tmp_path, analyzer):
    """Nobody enrolled is in this scene, so nothing may be followed."""
    im = img("zidane.jpg")
    stranger = analyzer.detect_and_embed(img("bus.jpg"))
    assert stranger, "no face in the decoy image"

    gallery = Gallery(str(tmp_path))
    gallery.save("someone_else", stranger[0].embedding[None, :])

    cfg = Config()
    cfg.gallery = str(tmp_path)
    cfg.link.enabled = False
    pipe = Pipeline(cfg, target_name="someone_else", quiet=True)

    for frame in pan_sequence(im, frames=25):
        r = pipe.process(frame)
        assert r.lock.state is not LockState.LOCKED, \
            "locked onto someone who was never enrolled"
        assert r.command.yaw_rate == pytest.approx(0.0, abs=1e-6)
