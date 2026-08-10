"""Appearance descriptor — the fallback when there is no face to read.

Face recognition answers "who is this?" and is the only thing that can acquire
a lock safely. But in flight the face is unavailable most of the time: the
subject turns around, walks away until their head is twelve pixels tall, or
looks down. A face-only system drops lock every few seconds and the drone
stops dead each time.

So identity is split in two:

  face  ->  ACQUIRE and RE-CONFIRM.  Authoritative, strict threshold.
  body  ->  SUSTAIN.  Answers only "is this the same body I was following?",
            which is a far easier question than "who is this?".

The descriptor is a normalised HSV histogram over the torso. Deliberately not a
learned re-ID network: those cost another 30ms and another 100MB of graph to
solve a problem this does not have. Distinguishing two arbitrary strangers is
hard; distinguishing the person wearing the red jacket that we confirmed by face
0.4 seconds ago from everyone else in the room is not.
"""
from __future__ import annotations

import cv2
import numpy as np

from .config import BodyConfig


def torso_crop(frame: np.ndarray, xyxy: np.ndarray,
               cfg: BodyConfig) -> np.ndarray | None:
    """Cut the torso out of a person box.

    Head and legs are excluded on purpose. The head is mostly skin and hair
    (nearly identical between people) and the legs swing in and out of the box
    as someone walks, which makes a whole-box histogram flicker.
    """
    h, w = frame.shape[:2]
    x1, y1, x2, y2 = xyxy
    bw, bh = x2 - x1, y2 - y1
    if bw < 8 or bh < 16:
        return None

    tx1 = int(round(x1 + bw * cfg.torso_inset))
    tx2 = int(round(x2 - bw * cfg.torso_inset))
    ty1 = int(round(y1 + bh * cfg.torso_top))
    ty2 = int(round(y1 + bh * cfg.torso_bottom))

    tx1, tx2 = max(0, tx1), min(w, tx2)
    ty1, ty2 = max(0, ty1), min(h, ty2)
    if tx2 - tx1 < 4 or ty2 - ty1 < 4:
        return None
    return frame[ty1:ty2, tx1:tx2]


def describe(frame: np.ndarray, xyxy: np.ndarray,
             cfg: BodyConfig) -> np.ndarray | None:
    """HSV histogram of the torso, L1-normalised."""
    crop = torso_crop(frame, xyxy, cfg)
    if crop is None:
        return None

    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
    # Very dark or very desaturated pixels carry no reliable hue — in shadow
    # every jacket is the same colour. Masking them out is what stops the
    # descriptor collapsing the moment the subject walks past a window.
    mask = cv2.inRange(hsv, (0, 40, 40), (180, 255, 255))
    if int(cv2.countNonZero(mask)) < 32:
        mask = None

    hist = cv2.calcHist([hsv], [0, 1, 2], mask,
                        [cfg.bins_h, cfg.bins_s, cfg.bins_v],
                        [0, 180, 0, 256, 0, 256])
    hist = hist.reshape(-1).astype(np.float32)
    total = float(hist.sum())
    if total < 1e-6:
        return None
    return hist / total


def similarity(a: np.ndarray | None, b: np.ndarray | None) -> float:
    """Bhattacharyya coefficient — 1.0 identical, 0.0 disjoint.

    Chosen over correlation or chi-square because it is bounded to [0, 1]
    without tuning, which lets the threshold in config mean the same thing
    regardless of how many bins the histogram has.
    """
    if a is None or b is None:
        return 0.0
    return float(np.sum(np.sqrt(np.maximum(a * b, 0.0))))


class BodyMemory:
    """Running appearance model of the locked target.

    Updated by exponential moving average, and *only* while the face agrees
    that this is the right person. That constraint is the whole point: an
    unconditioned EMA slowly walks onto whoever the tracker happens to be
    following, so the descriptor would drift onto a stranger over about a
    minute and then confidently confirm them.

    Backs onto either descriptor type. A learned re-ID embedding and a colour
    histogram need different maths — cosine on a unit sphere versus a
    Bhattacharyya coefficient on a probability vector — and, importantly,
    different thresholds. Sharing one number between them would silently put
    one of the two at the wrong operating point.
    """

    def __init__(self, cfg: BodyConfig, learned: bool = False):
        self.cfg = cfg
        self.learned = learned
        self.vector: np.ndarray | None = None
        self.updates = 0

    @property
    def match_thres(self) -> float:
        return self.cfg.reid_match_thres if self.learned else self.cfg.match_thres

    @property
    def _alpha(self) -> float:
        return self.cfg.reid_ema_alpha if self.learned else self.cfg.ema_alpha

    def reset(self) -> None:
        self.vector = None
        self.updates = 0

    def observe(self, descriptor: np.ndarray | None) -> None:
        if descriptor is None:
            return
        if self.vector is None:
            self.vector = descriptor.copy()
        else:
            a = self._alpha
            self.vector = (1.0 - a) * self.vector + a * descriptor
            if self.learned:
                # Renormalise onto the unit sphere: the mean of two unit
                # vectors is not a unit vector, and cosine similarity against
                # a shortened one reads low for no reason.
                n = float(np.linalg.norm(self.vector))
                if n > 1e-9:
                    self.vector /= n
            else:
                total = float(self.vector.sum())
                if total > 1e-6:
                    self.vector /= total
        self.updates += 1

    def seed(self, vector: np.ndarray | None) -> None:
        """Restore a previously-learned model, already trusted.

        Distinct from `observe()` because that treats every sample as one
        weak observation and needs several before it will vouch for anything.
        A model being restored after a re-acquisition is not a fresh
        observation — it is the accumulated result of many, and re-earning
        `ready` from scratch would leave the lock unable to sustain itself
        for the first few frames after recovering.
        """
        if vector is None:
            return
        self.vector = vector.copy()
        self.updates = max(self.updates, 3)

    def score(self, descriptor: np.ndarray | None) -> float:
        if descriptor is None or self.vector is None:
            return 0.0
        if self.learned:
            return float(np.dot(self.vector, descriptor))
        return similarity(self.vector, descriptor)

    @property
    def ready(self) -> bool:
        # One frame of torso colour is not an appearance model. Requiring a
        # few observations stops a single blurred frame at acquisition time
        # from defining the target for the rest of the flight.
        return self.vector is not None and self.updates >= 3
