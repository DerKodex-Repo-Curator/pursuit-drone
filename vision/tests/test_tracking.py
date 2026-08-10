"""Tracking and target-lock behaviour.

The lock tests matter more than they look: the failure they guard against is
the drone confidently following the wrong person, which is both the most
embarrassing demo failure and the hardest to notice from behind the camera.
"""
from __future__ import annotations

import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from pursuit.bytetrack import ByteTracker, Track                 # noqa: E402
from pursuit.config import (BodyConfig, FaceConfig, LockConfig,  # noqa: E402
                            TrackerConfig)
from pursuit.detect import Detection                             # noqa: E402
from pursuit.gallery import Gallery, Identity                    # noqa: E402
from pursuit.lock import LockState, TargetLock                   # noqa: E402
from pursuit.onnxbase import iou_matrix, nms                     # noqa: E402


def det(cx, cy, w=80, h=200, score=0.9):
    return Detection(np.array([cx - w / 2, cy - h / 2,
                               cx + w / 2, cy + h / 2], dtype=np.float32),
                     score)


@pytest.fixture(autouse=True)
def _fresh_ids():
    Track.reset_ids()


# ------------------------------------------------------------------- helpers --

def test_nms_collapses_duplicates():
    boxes = np.array([[0, 0, 10, 10], [1, 1, 11, 11], [50, 50, 60, 60]],
                     dtype=np.float32)
    scores = np.array([0.9, 0.8, 0.7], dtype=np.float32)
    keep = nms(boxes, scores, 0.5)
    assert len(keep) == 2 and keep[0] == 0


def test_iou_matrix_basics():
    a = np.array([[0, 0, 10, 10]], dtype=np.float32)
    b = np.array([[0, 0, 10, 10], [10, 10, 20, 20]], dtype=np.float32)
    m = iou_matrix(a, b)
    assert m[0, 0] == pytest.approx(1.0)
    assert m[0, 1] == pytest.approx(0.0)


# ------------------------------------------------------------------ tracking --

def test_id_is_stable_while_walking():
    tr = ByteTracker(TrackerConfig())
    ids = set()
    for i in range(30):
        tracks = tr.update([det(200 + i * 12, 360)])
        if tracks:
            ids.add(tracks[0].track_id)
    assert len(ids) == 1, f"identity fragmented into {ids}"


def test_two_people_keep_separate_ids():
    tr = ByteTracker(TrackerConfig())
    for i in range(20):
        tracks = tr.update([det(300 + i * 5, 360), det(900 - i * 5, 360)])
    assert len({t.track_id for t in tracks}) == 2


def test_track_survives_a_brief_occlusion():
    """The whole point of `predicted_xyxy` — someone walks behind a doorframe
    and the drone must keep pointing where they will emerge."""
    cfg = TrackerConfig()
    tr = ByteTracker(cfg)
    for i in range(15):
        tracks = tr.update([det(300 + i * 10, 360)])
    original = tracks[0].track_id

    for _ in range(10):                       # detector sees nothing
        tr.update([])
    coasting = tr.by_id(original)
    assert coasting is not None, "track dropped during a 10-frame occlusion"
    assert coasting.predicted_xyxy is not None

    for i in range(5):                        # reappears roughly on course
        tracks = tr.update([det(450 + i * 10, 360)])
    assert original in {t.track_id for t in tracks}, "identity lost on return"


def test_reacquire_does_not_steal_a_neighbour():
    """The proximity gate for lost tracks is deliberately loose, so prove it
    cannot hand one person's identity to somebody standing nearby."""
    tr = ByteTracker(TrackerConfig())
    for _ in range(12):
        tracks = tr.update([det(300, 360), det(800, 360)])
    left = min(t.track_id for t in tracks)
    right = max(t.track_id for t in tracks)

    # The left person is occluded; the right one keeps being detected.
    for _ in range(12):
        tracks = tr.update([det(800, 360)])

    live = {t.track_id for t in tracks}
    assert right in live, "the visible person lost their track"
    assert left not in live, "an occluded track stole the visible person"


def test_reacquire_rejects_an_implausible_size_jump():
    """A much nearer or further person must not inherit a coasting track."""
    tr = ByteTracker(TrackerConfig())
    for _ in range(12):
        tracks = tr.update([det(400, 360, h=200)])
    tid = tracks[0].track_id

    for _ in range(8):
        tr.update([])
    # Same place, but four times the height — a different person, much closer.
    for _ in range(3):
        tracks = tr.update([det(430, 360, h=800)])
    assert tid not in {t.track_id for t in tracks}


def test_track_expires_after_the_buffer():
    cfg = TrackerConfig(track_buffer=10)
    tr = ByteTracker(cfg)
    for _ in range(5):
        tracks = tr.update([det(300, 360)])
    tid = tracks[0].track_id
    for _ in range(40):
        tr.update([])
    assert tr.by_id(tid) is None


def test_low_confidence_detections_rescue_a_track():
    """The 'Byte' in ByteTrack: a partly-occluded person scores low but is
    still there, and discarding them fragments the track."""
    tr = ByteTracker(TrackerConfig())
    for i in range(10):
        tracks = tr.update([det(300 + i * 8, 360, score=0.9)])
    tid = tracks[0].track_id
    for i in range(6):                        # heavily occluded, low score
        tracks = tr.update([det(380 + i * 8, 360, score=0.25)])
    assert tracks and tracks[0].track_id == tid


# ---------------------------------------------------------------------- lock --

class FakeFace:
    def __init__(self, vec):
        self.embedding = np.asarray(vec, dtype=np.float32)
        self.xyxy = np.array([0, 0, 40, 40], dtype=np.float32)
        self.score = 0.99
        self.w = self.h = 40.0


def unit(seed, dim=512):
    rng = np.random.default_rng(seed)
    v = rng.normal(size=dim).astype(np.float32)
    return v / np.linalg.norm(v)


def at_cosine(v, target, seed=0):
    """A unit vector at exactly `target` cosine from `v`.

    Blending along an orthogonal direction gives the cosine in closed form,
    which matters because these tests probe narrow bands between thresholds
    and "add some noise" lands wherever it lands.
    """
    n = unit(seed, len(v))
    perp = n - (n @ v) * v
    perp /= np.linalg.norm(perp)
    out = target * v + np.sqrt(max(1.0 - target ** 2, 0.0)) * perp
    return (out / np.linalg.norm(out)).astype(np.float32)


@pytest.fixture
def rig(tmp_path):
    target_vec = unit(1)
    gallery = Gallery(str(tmp_path))
    gallery.identities["alex"] = Identity("alex", target_vec[None, :])
    lock = TargetLock(LockConfig(), FaceConfig(), BodyConfig(),
                      gallery, "alex")
    frame = np.full((720, 1280, 3), 60, dtype=np.uint8)
    frame[200:500, 500:700] = (40, 90, 180)      # something to describe
    return lock, gallery, frame, target_vec


def test_lock_requires_several_confirmations(rig):
    """Acquiring on one frame would let a single bad match steal the lock."""
    lock, _, frame, vec = rig
    tr = ByteTracker(TrackerConfig())
    cfg = LockConfig()

    states = []
    for i in range(cfg.lock_frames + 2):
        tracks = tr.update([det(600, 360)])
        result = lock.update(frame, tracks, {tracks[0].track_id: FakeFace(vec)})
        states.append(result.state)

    assert states[0] is LockState.SEARCHING
    assert states[cfg.lock_frames - 2] is LockState.SEARCHING
    assert states[-1] is LockState.LOCKED


def test_stranger_never_acquires_the_lock(rig):
    lock, _, frame, _ = rig
    tr = ByteTracker(TrackerConfig())
    other = unit(99)
    for _ in range(40):
        tracks = tr.update([det(600, 360)])
        result = lock.update(frame, tracks,
                             {tracks[0].track_id: FakeFace(other)})
    assert result.state is LockState.SEARCHING
    assert lock.target_id is None


def test_lock_holds_through_a_gap_then_gives_up(rig):
    lock, _, frame, vec = rig
    tr = ByteTracker(TrackerConfig())
    cfg = LockConfig()

    for _ in range(cfg.lock_frames + 1):
        tracks = tr.update([det(600, 360)])
        lock.update(frame, tracks, {tracks[0].track_id: FakeFace(vec)})
    assert lock.state is LockState.LOCKED

    # Subject vanishes entirely. Coasting first...
    for _ in range(cfg.coast_frames // 2):
        result = lock.update(frame, tr.update([]), {})
    assert result.state is LockState.COASTING

    # ...then the lock is released rather than followed forever.
    for _ in range(cfg.unlock_frames + 10):
        result = lock.update(frame, tr.update([]), {})
    assert result.state in (LockState.LOST, LockState.SEARCHING)
    assert lock.target_id is None


def test_body_sustains_the_lock_without_a_face(rig):
    """The behaviour that makes following usable: the subject turns around and
    the drone keeps going instead of stopping dead."""
    lock, _, frame, vec = rig
    tr = ByteTracker(TrackerConfig())

    for _ in range(LockConfig().lock_frames + 4):
        tracks = tr.update([det(600, 360)])
        lock.update(frame, tracks, {tracks[0].track_id: FakeFace(vec)})
    assert lock.state is LockState.LOCKED
    assert lock.memory.ready

    for _ in range(20):                        # face gone, same body
        tracks = tr.update([det(600, 360)])
        result = lock.update(frame, tracks, {})
    assert result.state is LockState.LOCKED
    assert result.source == "body"


def test_wrong_face_overrides_a_matching_body(rig):
    """The failure this guards against: two people in similar clothing, the
    tracker swaps IDs during a crossing, and colour alone keeps confirming the
    stranger forever because a non-matching face was treated as no evidence."""
    lock, _, frame, vec = rig
    tr = ByteTracker(TrackerConfig())

    for _ in range(LockConfig().lock_frames + 4):
        tracks = tr.update([det(600, 360)])
        lock.update(frame, tracks, {tracks[0].track_id: FakeFace(vec)})
    assert lock.state is LockState.LOCKED
    assert lock.memory.ready

    # Same position, same clothing (identical frame, so the body descriptor
    # matches perfectly) — but now the face is somebody else's.
    stranger = FakeFace(unit(99))
    for _ in range(6):
        tracks = tr.update([det(600, 360)])
        result = lock.update(frame, tracks,
                             {tracks[0].track_id: stranger})

    assert result.source != "body", \
        "colour sustained the lock despite a clearly wrong face"
    assert result.state is not LockState.LOCKED
    assert not lock.memory.ready, \
        "appearance model still primed on the wrong person"


def test_a_bad_angle_of_the_right_person_does_not_veto(rig):
    """The veto must not fire on a poor-but-genuine match, or the lock would
    break every time the subject glanced away."""
    lock, _, frame, vec = rig
    tr = ByteTracker(TrackerConfig())

    for _ in range(LockConfig().lock_frames + 4):
        tracks = tr.update([det(600, 360)])
        lock.update(frame, tracks, {tracks[0].track_id: FakeFace(vec)})
    assert lock.state is LockState.LOCKED

    # A degraded view of the same person: well under the 0.36 match threshold
    # but far above the 0.15 veto floor. Body should carry the lock.
    # Built to an exact cosine rather than by adding noise and hoping — a unit
    # vector plus 0.9x noise still correlates at 0.73, nowhere near the gap.
    noisy = at_cosine(vec, 0.25, seed=7)
    assert 0.15 < float(vec @ noisy) < 0.36, "test vector is not in the gap"

    for _ in range(6):
        tracks = tr.update([det(600, 360)])
        result = lock.update(frame, tracks, {tracks[0].track_id: FakeFace(noisy)})
    assert result.state is LockState.LOCKED
    assert result.source == "body"


class FakeReid:
    """Stand-in for the learned appearance model, so the re-acquisition path
    can be tested without loading a 102MB ONNX graph."""

    def __init__(self, vec_for_box):
        self.vec_for_box = vec_for_box

    def embed(self, frame, xyxy):
        return self.vec_for_box(xyxy)


def reid_rig(tmp_path, vec_for_box):
    target_vec = unit(1)
    gallery = Gallery(str(tmp_path))
    gallery.identities["alex"] = Identity("alex", target_vec[None, :])
    lock = TargetLock(LockConfig(), FaceConfig(), BodyConfig(), gallery,
                      "alex", reid=FakeReid(vec_for_box))
    frame = np.full((720, 1280, 3), 60, dtype=np.uint8)
    return lock, frame, target_vec


def test_reacquires_by_appearance_after_a_total_loss(tmp_path):
    """Without this, one walk behind a tree ends the follow: the appearance
    model is discarded and only a face can acquire again, which needs the
    subject back within about three metres."""
    body = unit(21)
    lock, frame, face_vec = reid_rig(tmp_path, lambda box: body)
    tr = ByteTracker(TrackerConfig())
    cfg = LockConfig()

    for _ in range(cfg.lock_frames + 6):
        tracks = tr.update([det(600, 360)])
        lock.update(frame, tracks, {tracks[0].track_id: FakeFace(face_vec)})
    assert lock.state is LockState.LOCKED

    # Subject disappears entirely for long enough to release the lock.
    for _ in range(cfg.unlock_frames + 5):
        lock.update(frame, tr.update([]), {})
    assert lock.target_id is None

    # Reappears elsewhere. No face readable — only the body.
    sources = []
    for _ in range(cfg.reacquire_frames + 3):
        tracks = tr.update([det(900, 360)])
        result = lock.update(frame, tracks, {})
        sources.append(result.source)

    assert result.state is LockState.LOCKED, "failed to re-acquire by body"
    # "reacquired" is the single frame that recovers the lock; afterwards it
    # sustains normally on "body", so check the transition happened rather
    # than the final frame's label.
    assert "reacquired" in sources
    assert sources[-1] == "body"


def test_a_stranger_cannot_steal_the_reacquisition(tmp_path):
    """The re-acquire threshold is much stricter than the sustain threshold
    precisely so this cannot happen — there is no face to confirm it."""
    body = unit(21)
    stranger = unit(77)

    boxes = {}
    lock, frame, face_vec = reid_rig(
        tmp_path, lambda box: boxes.get(round(float(box[0])), stranger))
    tr = ByteTracker(TrackerConfig())
    cfg = LockConfig()

    boxes[560] = body            # the real target's box position
    for _ in range(cfg.lock_frames + 6):
        tracks = tr.update([det(600, 360)])
        lock.update(frame, tracks, {tracks[0].track_id: FakeFace(face_vec)})
    assert lock.state is LockState.LOCKED

    for _ in range(cfg.unlock_frames + 5):
        lock.update(frame, tr.update([]), {})

    # Someone else walks through. Everything about them says "not the target".
    for _ in range(cfg.reacquire_frames + 10):
        tracks = tr.update([det(300, 360)])
        result = lock.update(frame, tracks, {})
    assert result.state is not LockState.LOCKED
    assert lock.target_id is None


def test_the_memento_expires(tmp_path):
    """An appearance model from ten minutes ago is not evidence about who is
    in front of the drone now."""
    body = unit(21)
    lock, frame, face_vec = reid_rig(tmp_path, lambda box: body)
    tr = ByteTracker(TrackerConfig())
    cfg = LockConfig()

    for _ in range(cfg.lock_frames + 6):
        tracks = tr.update([det(600, 360)])
        lock.update(frame, tracks, {tracks[0].track_id: FakeFace(face_vec)})
    for _ in range(cfg.unlock_frames + 5):
        lock.update(frame, tr.update([]), {})
    assert lock._memento is not None

    # Wind the stash back past its grace window.
    lock._memento_at -= cfg.reacquire_seconds + 1.0
    for _ in range(cfg.reacquire_frames + 5):
        tracks = tr.update([det(900, 360)])
        result = lock.update(frame, tracks, {})
    assert result.state is not LockState.LOCKED
    assert lock._memento is None


def test_reacquire_needs_consecutive_frames(tmp_path):
    """One lucky frame must not be enough — there is no face to check it."""
    body = unit(21)
    lock, frame, face_vec = reid_rig(tmp_path, lambda box: body)
    tr = ByteTracker(TrackerConfig())
    cfg = LockConfig()

    for _ in range(cfg.lock_frames + 6):
        tracks = tr.update([det(600, 360)])
        lock.update(frame, tracks, {tracks[0].track_id: FakeFace(face_vec)})
    for _ in range(cfg.unlock_frames + 5):
        lock.update(frame, tr.update([]), {})

    tracks = tr.update([det(900, 360)])
    result = lock.update(frame, tracks, {})
    assert result.state is not LockState.LOCKED, "re-acquired on one frame"


def test_histogram_backend_never_reacquires(tmp_path):
    """A colour histogram is far too weak to pick someone out of a crowd with
    no face to confirm it, so the path is disabled without a learned model."""
    target_vec = unit(1)
    gallery = Gallery(str(tmp_path))
    gallery.identities["alex"] = Identity("alex", target_vec[None, :])
    lock = TargetLock(LockConfig(), FaceConfig(), BodyConfig(), gallery,
                      "alex", reid=None)          # histogram backend
    frame = np.full((720, 1280, 3), 60, dtype=np.uint8)
    frame[200:500, 500:700] = (40, 90, 180)
    tr = ByteTracker(TrackerConfig())
    cfg = LockConfig()

    for _ in range(cfg.lock_frames + 6):
        tracks = tr.update([det(600, 360)])
        lock.update(frame, tracks, {tracks[0].track_id: FakeFace(target_vec)})
    for _ in range(cfg.unlock_frames + 5):
        lock.update(frame, tr.update([]), {})
    assert lock._memento is None


def test_lock_moves_to_the_better_face_match(rig):
    """Guards against following a stranger after an ID swap in a crossing."""
    lock, _, frame, vec = rig
    tr = ByteTracker(TrackerConfig())

    for _ in range(LockConfig().lock_frames + 1):
        tracks = tr.update([det(400, 360)])
        lock.update(frame, tracks, {tracks[0].track_id: FakeFace(vec)})
    first_id = lock.target_id

    for _ in range(6):
        tracks = tr.update([det(400, 360), det(900, 360)])
        by_id = {t.track_id: t for t in tracks}
        wrong = min(by_id)
        right = max(by_id)
        result = lock.update(frame, tracks, {
            wrong: FakeFace(unit(99)),        # imposter on the old track
            right: FakeFace(vec),             # real target on the other
        })

    assert lock.target_id == right != first_id
    assert result.state is LockState.LOCKED
