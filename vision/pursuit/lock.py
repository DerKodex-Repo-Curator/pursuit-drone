"""Target lock — the state machine that decides who the drone is following.

    SEARCHING ──face confirmed N frames──► LOCKED
        ▲                                    │
        │                                    │ no confirmation
        │                                    ▼
        └────────── unlock timeout ───── COASTING ──►(re-confirm)──► LOCKED
                                             │
                                             ▼
                                           LOST  ─► failsafe, then SEARCHING

Two design points carry most of the weight:

**Acquisition is slow, holding is fast.** Locking needs `lock_frames`
consecutive face confirmations; holding survives `unlock_frames` of nothing at
all. Symmetric thresholds make the lock oscillate between two people standing
together, and the drone yaws back and forth between them.

**Coasting is graded, not binary.** For the first `coast_frames` after losing
the subject the drone keeps chasing the Kalman prediction, because that is
almost always a brief occlusion — someone walked in front, or the subject
turned and the detector blinked. After that it holds position but *keeps the
track identity*, so when the subject reappears the lock is instant instead of
paying the acquisition cost again.
"""
from __future__ import annotations

import time
from enum import Enum

import numpy as np

from . import body as body_mod
from .config import BodyConfig, FaceConfig, LockConfig
from .gallery import Gallery


class LockState(Enum):
    SEARCHING = "SEARCHING"
    LOCKED = "LOCKED"
    COASTING = "COASTING"
    LOST = "LOST"


class LockResult:
    """What the control loop needs to know this frame."""

    __slots__ = ("state", "track", "xyxy", "confidence", "source",
                 "target_name", "miss_frames", "commanding")

    def __init__(self, state: LockState, track=None, xyxy=None,
                 confidence: float = 0.0, source: str = "none",
                 target_name: str | None = None, miss_frames: int = 0,
                 commanding: bool = False):
        self.state = state
        self.track = track
        self.xyxy: np.ndarray | None = xyxy
        self.confidence = confidence
        self.source = source          # "face" | "body" | "predicted" | "none"
        self.target_name = target_name
        self.miss_frames = miss_frames
        # True when the vision stack believes it knows where the subject is
        # well enough to steer. False means the controller must go neutral.
        self.commanding = commanding

    @property
    def velocity(self) -> tuple[float, float] | None:
        """Target centre velocity in px/frame, straight from the Kalman filter.

        The tracker estimates this every frame anyway; before this existed the
        controller ignored it and aimed at where the subject *was*, which at
        walking pace leaves a permanent lag of the order of 75px.
        """
        track = self.track
        if track is None or not hasattr(track, "mean"):
            return None
        # Freshly-created tracks have a zeroed velocity that means "unknown",
        # not "stationary"; leading on it would be leading on noise.
        if getattr(track, "hits", 0) < 3:
            return None
        return float(track.mean[4]), float(track.mean[5])

    @property
    def has_target(self) -> bool:
        return self.xyxy is not None

    def __repr__(self) -> str:                            # pragma: no cover
        return (f"<Lock {self.state.value} {self.source} "
                f"conf={self.confidence:.2f} miss={self.miss_frames}>")


class TargetLock:
    def __init__(self, cfg: LockConfig, face_cfg: FaceConfig,
                 body_cfg: BodyConfig, gallery: Gallery,
                 target_name: str | None = None,
                 reid=None):
        self.cfg = cfg
        self.face_cfg = face_cfg
        self.body_cfg = body_cfg
        self.gallery = gallery
        self.target_name = target_name
        # Optional learned appearance model. When absent the colour histogram
        # carries the body channel, which works but confuses similarly dressed
        # people — see body.py.
        self.reid = reid

        self.state = LockState.SEARCHING
        self.target_id: int | None = None
        self.memory = body_mod.BodyMemory(body_cfg, learned=reid is not None)

        self._candidate_id: int | None = None
        self._candidate_hits = 0
        self.miss_frames = 0
        self.last_confidence = 0.0
        self.last_source = "none"

        # Kept across a lost lock so the subject can be recovered without
        # walking back into face range. Holds the appearance model, when it
        # was lost, and how many consecutive frames a candidate has matched.
        self._memento: np.ndarray | None = None
        self._memento_at: float = 0.0
        self._reacq_id: int | None = None
        self._reacq_hits = 0

    # ---------------------------------------------------------------- reset --
    def _describe(self, frame: np.ndarray, xyxy) -> np.ndarray | None:
        """Appearance descriptor for a person box, from whichever backend is
        loaded. Single entry point so the two never get mixed in one memory."""
        if self.reid is not None:
            return self.reid.embed(frame, xyxy)
        return body_mod.describe(frame, xyxy, self.body_cfg)

    def reset(self, keep_memento: bool = False) -> None:
        # Losing the lock discards the appearance model by default, because a
        # stale one is how the drone latches onto a stranger. `keep_memento`
        # is the deliberate exception: an *expiring* copy, used only by the
        # much stricter re-acquisition path below.
        if not keep_memento:
            self._memento = None
            self._reacq_id = None
            self._reacq_hits = 0
        self.state = LockState.SEARCHING
        self.target_id = None
        self.memory.reset()
        self._candidate_id = None
        self._candidate_hits = 0
        self.miss_frames = 0

    def _stash_memento(self, now: float) -> None:
        """Preserve the appearance model as the lock is being released."""
        if not self.cfg.reacquire_enabled:
            return
        if self.reid is None or not self.memory.ready:
            return          # a colour histogram is far too weak for this
        vec = self.memory.vector
        if vec is None:
            return
        self._memento = vec.copy()
        self._memento_at = now
        self._reacq_id = None
        self._reacq_hits = 0

    def _try_reacquire(self, frame, by_id, now: float) -> LockResult | None:
        """Recover the lock from appearance alone, after a total loss.

        Only ever reached when there is no face to go on. The bar is set much
        higher than the one used to sustain a lock: sustaining only has to
        tell the target apart from the previous frame, whereas this has to
        pick them out of everyone in view with nothing to confirm it.
        """
        cfg = self.cfg
        if self._memento is None or not by_id:
            return None
        if (now - self._memento_at) > cfg.reacquire_seconds:
            self._memento = None      # too old to trust
            return None

        best_id, best_sim = None, -1.0
        for tid, track in by_id.items():
            desc = self._describe(frame, track.xyxy)
            if desc is None:
                continue
            sim = float(np.dot(self._memento, desc))
            if sim > best_sim:
                best_id, best_sim = tid, sim

        if best_id is None or best_sim < cfg.reacquire_thres:
            self._reacq_id, self._reacq_hits = None, 0
            return None

        # Consecutive frames on the SAME track, so a single lucky frame on a
        # passer-by in similar clothing cannot take the lock.
        if best_id == self._reacq_id:
            self._reacq_hits += 1
        else:
            self._reacq_id, self._reacq_hits = best_id, 1

        if self._reacq_hits < cfg.reacquire_frames:
            return None

        track = by_id[best_id]
        self.target_id = best_id
        track.is_target = True
        self.state = LockState.LOCKED
        self.miss_frames = 0
        self.memory.reset()
        # Seeded, not observed: this model was learned over many face-confirmed
        # frames before the loss. Treating it as one fresh observation would
        # leave the memory not `ready`, and the lock would coast straight back
        # out of the state it just recovered.
        self.memory.seed(self._memento)
        self.memory.observe(self._describe(frame, track.xyxy))
        self.last_confidence, self.last_source = best_sim, "reacquired"
        self._memento = None
        self._reacq_id, self._reacq_hits = None, 0
        return LockResult(LockState.LOCKED, track, track.xyxy, best_sim,
                          "reacquired", self.target_name, 0, True)

    def set_target(self, name: str | None) -> None:
        self.target_name = name
        self.reset()

    # ---------------------------------------------------------------- update --
    def update(self, frame: np.ndarray, tracks, face_by_track: dict) -> LockResult:
        """Advance one frame.

        `face_by_track` maps track_id -> Face (with `.embedding` filled in) for
        whichever tracks had a readable face this frame.
        """
        cfg = self.cfg
        by_id = {t.track_id: t for t in tracks}

        # --- score every track that showed a face --------------------------
        face_scores: dict[int, float] = {}
        for tid, face in face_by_track.items():
            if face is None or face.embedding is None:
                continue
            _, sim = self.gallery.best_match(face.embedding,
                                             restrict=self.target_name)
            face_scores[tid] = sim

        if self.target_id is not None and self.target_id in by_id:
            return self._update_locked(frame, by_id, face_scores)
        if self.target_id is not None:
            # Track object is gone from the tracker entirely (buffer expired).
            return self._update_missing(face_scores, by_id)
        return self._update_searching(frame, by_id, face_scores)

    # ------------------------------------------------------------ searching --
    def _update_searching(self, frame, by_id, face_scores) -> LockResult:
        cfg = self.cfg
        thres = self.face_cfg.match_thres

        best_id, best_sim = None, -1.0
        for tid, sim in face_scores.items():
            if sim >= thres and sim > best_sim:
                best_id, best_sim = tid, sim

        if best_id is None:
            self._candidate_id, self._candidate_hits = None, 0
            # No face to go on. Before giving up, see whether the appearance
            # model from a recently-lost lock recognises anyone in view — this
            # is what lets the subject walk behind an obstacle and be picked up
            # again on the far side without returning to face range.
            recovered = self._try_reacquire(frame, by_id, time.monotonic())
            if recovered is not None:
                return recovered
            self.state = LockState.SEARCHING
            return LockResult(LockState.SEARCHING,
                              target_name=self.target_name)

        # Consecutive confirmations on the *same* track, not just any track.
        if best_id == self._candidate_id:
            self._candidate_hits += 1
        else:
            self._candidate_id, self._candidate_hits = best_id, 1

        if self._candidate_hits >= cfg.lock_frames:
            self.target_id = best_id
            self.state = LockState.LOCKED
            self.miss_frames = 0
            self.memory.reset()
            track = by_id[best_id]
            track.is_target = True
            desc = self._describe(frame, track.xyxy)
            self.memory.observe(desc)
            self.last_confidence, self.last_source = best_sim, "face"
            return LockResult(LockState.LOCKED, track, track.xyxy,
                              best_sim, "face", self.target_name, 0, True)

        return LockResult(LockState.SEARCHING, by_id.get(best_id),
                          None, best_sim, "none", self.target_name)

    # --------------------------------------------------------------- locked --
    def _update_locked(self, frame, by_id, face_scores) -> LockResult:
        cfg = self.cfg
        track = by_id[self.target_id]
        face_thres = self.face_cfg.match_thres
        body_thres = self.memory.match_thres

        face_sim = face_scores.get(self.target_id, -1.0)
        descriptor = self._describe(frame, track.xyxy)

        # --- identity theft check -------------------------------------------
        # If some *other* track matches the enrolled face clearly better, the
        # lock is on the wrong person — the tracker swapped IDs during a
        # crossing. Hand the lock over rather than confidently following a
        # stranger for the rest of the flight.
        for tid, sim in face_scores.items():
            if tid == self.target_id:
                continue
            if sim >= face_thres and sim > max(face_sim, 0.0) + 0.08:
                track.is_target = False
                self.target_id = tid
                track = by_id[tid]
                track.is_target = True
                face_sim = sim
                self.memory.reset()
                descriptor = self._describe(frame, track.xyxy)
                break

        confirmed, source, confidence = False, "none", 0.0

        # A face we could read but that clearly is not the target is evidence
        # AGAINST this lock, not merely an absence of evidence. Treating it as
        # neutral lets the colour descriptor sustain an ID swap between two
        # people in similar clothing forever — the drone follows the wrong
        # person and never rechecks, because the body keeps agreeing.
        saw_face = self.target_id in face_scores
        wrong_face = saw_face and face_sim < self.face_cfg.veto_thres

        if face_sim >= face_thres:
            confirmed, source, confidence = True, "face", face_sim
            # Only a face-confirmed frame is allowed to teach the appearance
            # model. Updating it on body-only frames lets it drift onto
            # whoever the tracker is following, which is exactly the failure
            # the body model is supposed to catch.
            self.memory.observe(descriptor)
        elif wrong_face:
            # Drop the appearance model too: it was built from the person we
            # are now looking at, so it would happily re-confirm them.
            self.memory.reset()
        elif self.memory.ready:
            body_sim = self.memory.score(descriptor)
            if body_sim >= body_thres:
                confirmed, source, confidence = True, "body", body_sim

        if confirmed:
            self.miss_frames = 0
            self.state = LockState.LOCKED
            self.last_confidence, self.last_source = confidence, source
            return LockResult(LockState.LOCKED, track, track.xyxy,
                              confidence, source, self.target_name, 0, True)

        # Track is alive but unconfirmed — treat exactly like a miss so the
        # same timeout governs "wrong-looking person" and "no person at all".
        return self._coast(track, track.predicted_xyxy)

    # -------------------------------------------------------------- missing --
    def _update_missing(self, face_scores, by_id) -> LockResult:
        """Target track object no longer exists in the tracker."""
        thres = self.face_cfg.match_thres
        # A confident face re-acquisition jumps straight back to LOCKED and
        # re-binds to the new track id, which is what makes walking back into
        # frame feel instant rather than costing another acquisition.
        for tid, sim in face_scores.items():
            if sim >= thres and tid in by_id:
                self.target_id = tid
                by_id[tid].is_target = True
                self.miss_frames = 0
                self.state = LockState.LOCKED
                self.last_confidence, self.last_source = sim, "face"
                return LockResult(LockState.LOCKED, by_id[tid],
                                  by_id[tid].xyxy, sim, "face",
                                  self.target_name, 0, True)
        return self._coast(None, None)

    # ---------------------------------------------------------------- coast --
    def _coast(self, track, predicted) -> LockResult:
        cfg = self.cfg
        self.miss_frames += 1

        if self.miss_frames > cfg.unlock_frames:
            # Preserve the appearance model on the way out, so the subject can
            # be recovered without walking back into face range.
            self._stash_memento(time.monotonic())
            self.reset(keep_memento=True)
            self.state = LockState.LOST
            return LockResult(LockState.LOST, target_name=self.target_name,
                              miss_frames=cfg.unlock_frames + 1)

        self.state = LockState.COASTING
        # Inside the coast window keep steering toward the prediction; past it
        # hold station but keep the identity so re-acquisition stays cheap.
        commanding = self.miss_frames <= cfg.coast_frames and predicted is not None
        return LockResult(LockState.COASTING, track,
                          predicted if commanding else None,
                          self.last_confidence if commanding else 0.0,
                          "predicted" if commanding else "none",
                          self.target_name, self.miss_frames, commanding)
