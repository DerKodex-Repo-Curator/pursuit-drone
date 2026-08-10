"""One frame in, one set of flight setpoints out.

    frame ─► detect people ─► track ─► read faces ─► lock ─► control ─► Command

The face stage is the expensive one and it runs on a cadence rather than every
frame: while the lock is held the appearance model sustains identity, so faces
only need re-checking every few frames. While searching or coasting it runs
flat out, because those are exactly the moments identity is in doubt.

Every stage is timed separately. On a follow-me system the interesting number
is never the average frame rate — it is which stage ate the budget when the
frame rate dropped, and whether the control loop still got its packet out.
"""
from __future__ import annotations

import time

import numpy as np

from .body import describe as describe_body
from .bytetrack import ByteTracker
from .config import Config
from .control import FollowController
from .detect import PersonDetector
from .face import FaceAnalyzer, assign_faces_to_people
from .gallery import Gallery
from . import reid as reid_mod
from .lock import LockResult, LockState, TargetLock
from .protocol import Command


class Timings:
    __slots__ = ("capture", "detect", "track", "face", "lock", "control",
                 "total", "fps", "face_ran")

    def __init__(self):
        self.capture = self.detect = self.track = 0.0
        self.face = self.lock = self.control = self.total = 0.0
        self.fps = 0.0
        # Explicit, because `face > 0` is not the same question. The timer
        # brackets the whole stage, so a skipped frame still records a few
        # microseconds of overhead and would be miscounted as a face frame.
        self.face_ran = False

    def as_row(self) -> str:                              # pragma: no cover
        return (f"det {self.detect:5.1f}  face {self.face:5.1f}  "
                f"trk {self.track:4.1f}  ctl {self.control:4.1f}  "
                f"tot {self.total:5.1f}ms  {self.fps:4.1f}fps")


class FrameResult:
    __slots__ = ("frame", "detections", "tracks", "faces", "lock", "command",
                 "timings", "frame_id")

    def __init__(self, frame, detections, tracks, faces, lock, command,
                 timings, frame_id):
        self.frame = frame
        self.detections = detections
        self.tracks = tracks
        self.faces = faces
        self.lock: LockResult = lock
        self.command: Command = command
        self.timings: Timings = timings
        self.frame_id = frame_id


class Pipeline:
    def __init__(self, cfg: Config, target_name: str | None = None,
                 quiet: bool = False):
        self.cfg = cfg
        self.frame_id = 0
        self._fps_ema = 0.0
        self._last_t = time.monotonic()

        def say(msg: str) -> None:
            if not quiet:
                print(msg)

        say("loading models...")
        t0 = time.perf_counter()
        self.detector = PersonDetector(cfg.detector, cfg.providers)
        say(f"  detector  {self.detector.session.provider.replace('ExecutionProvider','')}"
            f"  {self.detector.imgsz}px")

        self.faces = FaceAnalyzer(cfg.face, cfg.providers)
        say(f"  face      {self.faces.det.provider.replace('ExecutionProvider','')}"
            f"  SCRFD + ArcFace-512")

        self.gallery = Gallery(cfg.gallery)
        n = self.gallery.load_all()
        say(f"  gallery   {n} enrolled: {', '.join(self.gallery.names()) or '(none)'}")

        if target_name and target_name not in self.gallery:
            raise SystemExit(
                f"target '{target_name}' is not enrolled.\n"
                f"  enrolled: {', '.join(self.gallery.names()) or '(none)'}\n"
                f"  run:  python enroll.py --name {target_name}")

        self.reid = reid_mod.load(cfg.body, cfg.providers)
        if self.reid is not None:
            say(f"  reid      {self.reid.session.provider.replace('ExecutionProvider','')}"
                f"  {self.reid.dim}-d learned appearance")
        else:
            say("  reid      colour histogram (no learned model)")

        self.tracker = ByteTracker(cfg.tracker)
        self.lock = TargetLock(cfg.lock, cfg.face, cfg.body,
                               self.gallery, target_name, reid=self.reid)

        # A subject height turns the standoff into a real distance instead of
        # a box-height fraction. Recorded at enrolment; absent for anyone
        # enrolled before that existed, which the controller handles.
        height_m = None
        if target_name and target_name in self.gallery:
            height_m = self.gallery.identities[target_name].height_m
        self.controller = FollowController(cfg.control, cfg.camera,
                                           subject_height_m=height_m)
        if height_m:
            say(f"  standoff  {cfg.control.standoff_m:.1f}m "
                f"(subject {height_m:.2f}m)")
        say(f"  ready in {time.perf_counter()-t0:.1f}s\n")

    # ------------------------------------------------------------------ misc --
    def set_target(self, name: str | None) -> None:
        self.lock.set_target(name)
        self.controller.reset()

    def reset(self) -> None:
        self.tracker.reset()
        self.lock.reset()
        self.controller.reset()

    def _want_faces(self) -> bool:
        state = self.lock.state
        if state is LockState.LOCKED:
            return self.frame_id % max(1, self.cfg.face.interval_locked) == 0
        return True          # SEARCHING / COASTING / LOST: identity in doubt

    # --------------------------------------------------------------- process --
    def process(self, frame: np.ndarray, armed: bool = False,
                attitude: tuple[float, float] | None = None) -> FrameResult:
        """`attitude` is the drone's measured (roll, pitch) in degrees.

        Feed it from telemetry whenever the link is up. Without it the control
        loop couples: the camera is bolted to the airframe, so tilting to chase
        moves the whole image and the altitude axis reads that as real motion.
        """
        t = Timings()
        self.frame_id += 1
        frame_start = time.perf_counter()

        h, w = frame.shape[:2]

        # --- people ----------------------------------------------------------
        t0 = time.perf_counter()
        detections = self.detector(frame)
        t.detect = (time.perf_counter() - t0) * 1e3

        # --- tracking --------------------------------------------------------
        t0 = time.perf_counter()
        tracks = self.tracker.update(detections)
        t.track = (time.perf_counter() - t0) * 1e3

        # --- faces -----------------------------------------------------------
        t0 = time.perf_counter()
        faces: list = []
        face_by_track: dict[int, object] = {}
        if tracks and self._want_faces():
            t.face_ran = True
            if self.cfg.face.use_roi:
                rois = [self.faces.head_roi(tr.xyxy, w, h) for tr in tracks]
                faces = self.faces.detect_in_rois(frame, rois)
            else:
                faces = self.faces.detect(frame)
            if faces:
                # Faces are matched to *tracks*, not raw detections, so the
                # identity survives the frames where the detector misses and
                # the tracker is coasting on prediction.
                mapping = assign_faces_to_people(tracks, faces)
                for ti, fi in mapping.items():
                    face = faces[fi]
                    if min(face.w, face.h) < self.cfg.face.min_face_px:
                        continue
                    self.faces.embed(frame, face)
                    face_by_track[tracks[ti].track_id] = face
                    tracks[ti].face_score = face.score
                    # Name EVERY recognised face, not only the target's. The
                    # lock still follows one person; this just lets the HUD
                    # say "that is Sina" instead of "#2", which is the whole
                    # point of showing that it can tell people apart.
                    # best_match with no restrict scores the full gallery.
                    nm, sim = self.lock.gallery.best_match(face.embedding)
                    if nm and sim >= self.cfg.face.match_thres:
                        tracks[ti].identity = nm
                        tracks[ti].identity_score = float(sim)
        t.face = (time.perf_counter() - t0) * 1e3

        # --- identity --------------------------------------------------------
        t0 = time.perf_counter()
        for tr in tracks:
            tr.is_target = False
        lock = self.lock.update(frame, tracks, face_by_track)
        if lock.track is not None:
            lock.track.is_target = True
        t.lock = (time.perf_counter() - t0) * 1e3

        # --- control ---------------------------------------------------------
        now = time.monotonic()
        dt = now - self._last_t
        self._last_t = now

        t0 = time.perf_counter()
        command = self.controller.update(lock, w, h, dt, armed=armed,
                                         attitude=attitude)
        t.control = (time.perf_counter() - t0) * 1e3

        t.total = (time.perf_counter() - frame_start) * 1e3
        inst = 1.0 / dt if dt > 1e-6 else 0.0
        self._fps_ema = inst if self._fps_ema == 0.0 else \
            0.9 * self._fps_ema + 0.1 * inst
        t.fps = self._fps_ema

        return FrameResult(frame, detections, tracks, faces, lock, command,
                           t, self.frame_id)
