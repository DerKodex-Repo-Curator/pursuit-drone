"""ByteTrack — multi-object tracking by two-stage association.

Implemented here rather than pulled from a package because the package versions
drag in torch, and because the drone needs one behaviour the stock
implementation does not advertise clearly: a *lost* track has to keep producing
a predicted box. When the subject walks behind a doorframe the detector returns
nothing, but the control loop still needs somewhere to point for the next half
second. That is what `Track.predicted_xyxy` is for.

The "Byte" idea is the second association pass. Most trackers throw away
low-confidence detections; ByteTrack matches them against tracks that survived
the first pass, on the reasoning that a box the detector is unsure about is
usually a real object that is partly occluded rather than noise.
"""
from __future__ import annotations

import numpy as np
from scipy.optimize import linear_sum_assignment

from .config import TrackerConfig
from .onnxbase import iou_matrix

TRACKED, LOST, REMOVED = 0, 1, 2


class KalmanBoxFilter:
    """Constant-velocity Kalman filter on (cx, cy, aspect, height).

    Aspect and height rather than width and height: a person's height is a far
    more stable quantity frame to frame than their width, so putting height in
    the state directly gives the distance estimate (and therefore the pitch
    command) a much quieter signal to work with.
    """

    def __init__(self):
        ndim, dt = 4, 1.0
        self.F = np.eye(2 * ndim, dtype=np.float32)
        for i in range(ndim):
            self.F[i, ndim + i] = dt
        self.H = np.eye(ndim, 2 * ndim, dtype=np.float32)
        # Process/measurement noise scale with the object's height: a person
        # filling the frame genuinely moves more pixels per frame than a
        # distant one, and a fixed noise model over-smooths the near case.
        self.std_pos = 1.0 / 20
        self.std_vel = 1.0 / 160

    def initiate(self, meas: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        mean = np.concatenate([meas, np.zeros(4, dtype=np.float32)])
        h = meas[3]
        std = np.array([2 * self.std_pos * h, 2 * self.std_pos * h,
                        1e-2, 2 * self.std_pos * h,
                        10 * self.std_vel * h, 10 * self.std_vel * h,
                        1e-5, 10 * self.std_vel * h], dtype=np.float32)
        return mean.astype(np.float32), np.diag(np.square(std)).astype(np.float32)

    def predict(self, mean, cov):
        h = mean[3]
        std = np.array([self.std_pos * h, self.std_pos * h, 1e-2, self.std_pos * h,
                        self.std_vel * h, self.std_vel * h, 1e-5, self.std_vel * h],
                       dtype=np.float32)
        q = np.diag(np.square(std)).astype(np.float32)
        mean = self.F @ mean
        cov = self.F @ cov @ self.F.T + q
        return mean, cov

    def update(self, mean, cov, meas):
        h = mean[3]
        std = np.array([self.std_pos * h, self.std_pos * h, 1e-1, self.std_pos * h],
                       dtype=np.float32)
        r = np.diag(np.square(std)).astype(np.float32)

        s = self.H @ cov @ self.H.T + r
        kalman_gain = cov @ self.H.T @ np.linalg.inv(s)
        innovation = meas - self.H @ mean

        mean = mean + kalman_gain @ innovation
        cov = cov - kalman_gain @ s @ kalman_gain.T
        return mean.astype(np.float32), cov.astype(np.float32)


def xyxy_to_xyah(box: np.ndarray) -> np.ndarray:
    w = box[2] - box[0]
    h = box[3] - box[1]
    return np.array([box[0] + w * 0.5, box[1] + h * 0.5,
                     w / max(h, 1e-6), h], dtype=np.float32)


def xyah_to_xyxy(state: np.ndarray) -> np.ndarray:
    cx, cy, a, h = state[:4]
    w = a * h
    return np.array([cx - w * 0.5, cy - h * 0.5,
                     cx + w * 0.5, cy + h * 0.5], dtype=np.float32)


class Track:
    _next_id = 1

    def __init__(self, det, kf: KalmanBoxFilter, frame_id: int):
        self.kf = kf
        self.mean, self.cov = kf.initiate(xyxy_to_xyah(det.xyxy))
        self.track_id = Track._next_id
        Track._next_id += 1

        self.state = TRACKED
        self.score = det.score
        self.start_frame = frame_id
        self.frame_id = frame_id
        self.hits = 1
        self.age = 0
        self.time_since_update = 0

        # Slots the identity layer fills in; the tracker itself never reads
        # them, which keeps geometric tracking and re-ID cleanly separate.
        self.body_descriptor: np.ndarray | None = None
        self.face_score: float = 0.0
        self.is_target: bool = False
        # Who this track is, if a face was recognised against the gallery.
        # Distinct from is_target: the aircraft follows ONE person, but it is
        # useful — on the HUD and in a demo — to name the others it can see.
        self.identity: str | None = None
        self.identity_score: float = 0.0

    @classmethod
    def reset_ids(cls) -> None:
        cls._next_id = 1

    @property
    def xyxy(self) -> np.ndarray:
        return xyah_to_xyxy(self.mean)

    @property
    def predicted_xyxy(self) -> np.ndarray:
        """Where the target should be even when nothing was detected."""
        return xyah_to_xyxy(self.mean)

    # How fast a lost track's velocity decays per frame. Someone who walks
    # behind a doorframe is about as likely to have stopped as to have kept
    # going, so extrapolating at full speed for half a second puts the
    # prediction further from the truth than simply freezing it would.
    LOST_VELOCITY_DECAY = 0.82

    def predict(self) -> None:
        mean = self.mean.copy()
        if self.state != TRACKED:
            mean[7] = 0.0        # a lost track should not keep growing
            mean[4:7] *= self.LOST_VELOCITY_DECAY
        self.mean, self.cov = self.kf.predict(mean, self.cov)
        self.age += 1
        self.time_since_update += 1

    def update(self, det, frame_id: int) -> None:
        self.mean, self.cov = self.kf.update(self.mean, self.cov,
                                             xyxy_to_xyah(det.xyxy))
        self.state = TRACKED
        self.score = det.score
        self.frame_id = frame_id
        self.hits += 1
        self.time_since_update = 0

    def mark_lost(self) -> None:
        self.state = LOST

    def mark_removed(self) -> None:
        self.state = REMOVED

    def __repr__(self) -> str:                            # pragma: no cover
        s = {TRACKED: "T", LOST: "L", REMOVED: "R"}[self.state]
        return f"<Track#{self.track_id}{s} {self.score:.2f} age={self.age}>"


def _associate(tracks: list[Track], dets: list, thresh: float):
    """Hungarian assignment on IoU cost. Returns (matches, un_tracks, un_dets)."""
    if not tracks or not dets:
        return [], list(range(len(tracks))), list(range(len(dets)))

    t_boxes = np.stack([t.xyxy for t in tracks])
    d_boxes = np.stack([d.xyxy for d in dets])
    iou = iou_matrix(t_boxes, d_boxes)
    cost = 1.0 - iou

    rows, cols = linear_sum_assignment(cost)
    matches, un_t, un_d = [], [], []
    matched_t, matched_d = set(), set()

    for r, c in zip(rows, cols):
        # linear_sum_assignment returns a complete assignment; anything above
        # the gate is a forced pairing of two unrelated boxes, so reject it.
        if cost[r, c] <= thresh:
            matches.append((r, c))
            matched_t.add(int(r))
            matched_d.add(int(c))

    un_t = [i for i in range(len(tracks)) if i not in matched_t]
    un_d = [i for i in range(len(dets)) if i not in matched_d]
    return matches, un_t, un_d


def _reacquire(tracks: list[Track], dets: list, cfg: TrackerConfig):
    """Third pass: match long-lost tracks by proximity rather than overlap.

    IoU is the wrong tool once a track has been coasting for a while. It is
    effectively binary — the moment the predicted box stops overlapping the
    real one it reads 0.0 and stays there, giving the optimiser no gradient to
    follow no matter how nearly right the prediction was.

    So lost tracks get a distance-based gate instead, measured in multiples of
    the subject's height and widened by how long they have been missing. That
    is the covariance argument made cheaply: uncertainty about where someone
    went grows with time, so the search region should grow with it.

    Two guards keep the loose gate from causing identity theft:
      * height ratio must be plausible, so a near person cannot inherit a
        distant person's track
      * the gate is capped, so a track lost for a long time does not end up
        matching anything anywhere in the frame
    """
    if not tracks or not dets:
        return [], list(range(len(dets)))

    t_boxes = np.stack([t.xyxy for t in tracks])
    d_boxes = np.stack([d.xyxy for d in dets])

    t_c = np.stack([(t_boxes[:, 0] + t_boxes[:, 2]) * 0.5,
                    (t_boxes[:, 1] + t_boxes[:, 3]) * 0.5], axis=1)
    d_c = np.stack([(d_boxes[:, 0] + d_boxes[:, 2]) * 0.5,
                    (d_boxes[:, 1] + d_boxes[:, 3]) * 0.5], axis=1)
    t_h = np.maximum(t_boxes[:, 3] - t_boxes[:, 1], 1.0)
    d_h = np.maximum(d_boxes[:, 3] - d_boxes[:, 1], 1.0)

    dist = np.linalg.norm(t_c[:, None, :] - d_c[None, :, :], axis=2)
    scale = 0.5 * (t_h[:, None] + d_h[None, :])
    cost = dist / scale

    ratio = np.maximum(t_h[:, None] / d_h[None, :], d_h[None, :] / t_h[:, None])
    gates = np.array([min(cfg.reacquire_gate_max,
                          cfg.reacquire_gate + cfg.reacquire_gate_growth
                          * t.time_since_update) for t in tracks])

    blocked = (cost > gates[:, None]) | (ratio > cfg.reacquire_height_ratio)
    cost = np.where(blocked, 1e6, cost)

    rows, cols = linear_sum_assignment(cost)
    matches, used = [], set()
    for r, c in zip(rows, cols):
        if cost[r, c] < 1e5:
            matches.append((int(r), int(c)))
            used.add(int(c))
    return matches, [i for i in range(len(dets)) if i not in used]


class ByteTracker:
    def __init__(self, cfg: TrackerConfig):
        self.cfg = cfg
        self.kf = KalmanBoxFilter()
        self.tracks: list[Track] = []
        self.lost: list[Track] = []
        self.frame_id = 0

    def reset(self) -> None:
        self.tracks.clear()
        self.lost.clear()
        self.frame_id = 0
        Track.reset_ids()

    def update(self, detections: list) -> list[Track]:
        cfg = self.cfg
        self.frame_id += 1

        dets = [d for d in detections if d.area >= cfg.min_box_area]
        high = [d for d in dets if d.score >= cfg.high_thresh]
        low = [d for d in dets
               if cfg.low_thresh <= d.score < cfg.high_thresh]

        pool = self.tracks + self.lost
        for t in pool:
            t.predict()

        # --- pass 1: confident detections against every track --------------
        matches, un_t, un_d_high = _associate(pool, high, cfg.match_thresh)
        activated: list[Track] = []
        for ti, di in matches:
            pool[ti].update(high[di], self.frame_id)
            activated.append(pool[ti])

        # --- pass 2: the Byte step ------------------------------------------
        # Only tracks that were healthy last frame get a second chance. A track
        # already LOST should not be resurrected by a 0.15-confidence blob.
        remaining = [pool[i] for i in un_t if pool[i].state == TRACKED]
        m2, un_t2, _ = _associate(remaining, low, 0.5)
        for ti, di in m2:
            remaining[ti].update(low[di], self.frame_id)
            activated.append(remaining[ti])

        # --- pass 3: re-acquire coasting tracks by proximity ------------------
        # Anything still unmatched and already LOST gets one more chance
        # against the leftover confident detections, on a gate that widens
        # with how long it has been missing.
        matched_ids = {id(t) for t in activated}
        stale = [t for t in pool
                 if id(t) not in matched_ids and t.state == LOST]
        leftovers = [high[i] for i in un_d_high]
        if stale and leftovers:
            m3, un_left = _reacquire(stale, leftovers, cfg)
            for ti, di in m3:
                stale[ti].update(leftovers[di], self.frame_id)
                activated.append(stale[ti])
            leftovers = [leftovers[i] for i in un_left]

        matched_ids = {id(t) for t in activated}
        still_lost = []
        for t in pool:
            if id(t) in matched_ids:
                continue
            if t.state == TRACKED:
                t.mark_lost()
            if self.frame_id - t.frame_id > cfg.track_buffer:
                t.mark_removed()
            else:
                still_lost.append(t)

        # --- births -----------------------------------------------------------
        for d in leftovers:
            if d.score >= cfg.new_track_thresh:
                activated.append(Track(d, self.kf, self.frame_id))

        self.tracks = [t for t in activated if t.state == TRACKED]
        self.lost = [t for t in still_lost if t.state == LOST]
        return self.tracks

    def all_tracks(self) -> list[Track]:
        """Live tracks plus coasting ones, for the lock layer and the HUD."""
        return self.tracks + self.lost

    def by_id(self, track_id: int) -> Track | None:
        for t in self.all_tracks():
            if t.track_id == track_id:
                return t
        return None
