"""Every tunable in one place.

The pipeline reads a single `Config`. Nothing else in the package holds a
magic number, so retuning the follow behaviour never means editing logic.
`Config.load()` overlays a YAML file on the defaults, which is how you keep a
"living room" profile and an "outdoors" profile without touching code.
"""
from __future__ import annotations

import dataclasses
import os
from dataclasses import dataclass, field
from typing import Any

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _path(*parts: str) -> str:
    return os.path.join(ROOT, *parts)


@dataclass
class SourceConfig:
    """Where frames come from."""
    camera: int = 0             # cv2 device index
    width: int = 1280
    height: int = 720
    fps: int = 30
    flip: bool = True           # mirror the webcam so left/right read naturally
    # A laptop webcam is the *test* rig. On the drone the frames arrive over
    # WiFi, so `url` (an MJPEG/RTSP endpoint) takes priority when set.
    url: str = ""


@dataclass
class CameraConfig:
    """Lens geometry, needed to turn pixels into angles.

    Everything that reasons about the world rather than the image — undoing the
    airframe's own attitude, estimating a real distance — needs a focal length.
    One number (the horizontal field of view) gives it, and the XIAO Sense's
    OV2640 with the stock lens is about 65-70 degrees.

    Measure yours if you want the distance estimate to be trustworthy: put a
    metre rule at a known distance, count the pixels it spans, and
    f = pixels * distance / width_in_metres.
    """
    hfov_deg: float = 65.0

    def focal_px(self, frame_w: int) -> float:
        import math
        half = math.radians(self.hfov_deg) * 0.5
        return (frame_w * 0.5) / max(math.tan(half), 1e-6)


@dataclass
class DetectorConfig:
    """YOLO11n, person class only."""
    model: str = field(default_factory=lambda: _path("models", "yolo11n.onnx"))
    # 640 is the trained size. 480 is ~40% faster and costs you distant
    # people; 320 is faster still and only sees subjects filling the frame.
    # Must be a multiple of 32.
    imgsz: int = 640
    # Deliberately low, and set from measurement rather than taste. On
    # simulated drone-camera frames (tools/degradation_sweep.py), people found
    # against a reference of 2:
    #
    #                       0.30  0.25  0.20  0.15  0.10
    #     underexposed        1     2     2     2     2
    #     blurred + sheared   0     0     0     1     1
    #
    # Motion blur and underexposure push YOLO's confidence down without
    # destroying the detection, so a threshold tuned on clean images throws
    # away frames that are still perfectly usable. 0.15 is the knee; below it
    # nothing further is recovered.
    #
    # Cheap for two structural reasons. ByteTrack only CREATES tracks above
    # `tracker.new_track_thresh` (0.60), so a weak detection can rescue an
    # existing track without spawning a spurious one. And anything that does
    # get through still has to satisfy the face/ReID identity layer before the
    # drone will follow it. No false positives were observed down to 0.10.
    conf_thres: float = 0.15
    nms_iou: float = 0.55
    max_det: int = 32
    # COCO class 0 is `person`. Nothing else on the drone's mind.
    class_id: int = 0


@dataclass
class FaceConfig:
    """SCRFD detection + ArcFace 512-d embedding (InsightFace buffalo_l)."""
    det_model: str = field(
        default_factory=lambda: _path("models", "buffalo_l", "det_10g.onnx"))
    rec_model: str = field(
        default_factory=lambda: _path("models", "buffalo_l", "w600k_r50.onnx"))
    det_size: int = 640
    det_thres: float = 0.5
    nms_iou: float = 0.4
    # Scan each track's head region instead of the whole frame. The person
    # detector has already found everyone worth looking at, and a face outside
    # every person box is a face on somebody the drone could not follow anyway.
    # Roughly a tenth of the cost of the full-frame pass.
    use_roi: bool = True
    roi_size: int = 192            # rounded down to a multiple of 32
    # Cosine similarity on L2-normalised ArcFace vectors. 0.28 is InsightFace's
    # own "same person" operating point at 1e-4 FAR; 0.36 is deliberately
    # stricter because locking onto the WRONG person is the expensive failure.
    match_thres: float = 0.36
    # Below ~28px a face embedding is noise. Skip it and let the body
    # descriptor carry the track instead of poisoning the gallery.
    min_face_px: int = 28
    # A readable face scoring below this is positive evidence that the lock is
    # on the WRONG person, and overrides a body match. Without it, an ID swap
    # between two people in similar clothing is sustained indefinitely by
    # colour alone. Sits in the empty gap between measured strangers (<=0.08)
    # and the worst same-person match (>=0.5 under blur, scale and low light),
    # so it never fires on a bad angle of the right person.
    veto_thres: float = 0.15
    # Face detection is the most expensive stage. While locked, the body
    # descriptor sustains identity between checks, so re-confirming every
    # `interval_locked` frames buys most of the frame rate back without
    # letting the appearance model drift unsupervised for long.
    interval_locked: int = 5


@dataclass
class BodyConfig:
    """Appearance descriptor used when the face is not visible.

    A drone loses the face constantly — the subject turns around, walks away,
    or the frame is simply too small. Face-only re-ID drops lock every few
    seconds. This is a cheap HSV colour-histogram descriptor over the torso,
    which is weak at telling strangers apart but strong at answering the only
    question that matters mid-flight: "is this the same body I saw last frame?"
    """
    bins_h: int = 12
    bins_s: int = 8
    bins_v: int = 4
    # Torso crop as a fraction of the person box: skip head and legs, both of
    # which change appearance fast as the subject moves.
    torso_top: float = 0.18
    torso_bottom: float = 0.62
    torso_inset: float = 0.15
    match_thres: float = 0.55
    # Running gallery of the locked target's appearance, updated only while the
    # face confirms identity, so it cannot drift onto a passer-by.
    ema_alpha: float = 0.12

    # --- learned re-ID (preferred when the model is present) ----------------
    # A 768-d embedding trained for person re-identification, instead of the
    # colour histogram above. It encodes build, clothing structure and texture
    # rather than average torso colour, which is what lets it tell apart two
    # people the histogram sees as identical. 1.1ms per crop on CoreML.
    # Falls back to the histogram automatically if the file is missing.
    use_reid: bool = True
    reid_model: str = field(
        default_factory=lambda: _path("models", "person_reid.onnx"))
    # Cosine similarity, a different scale from the histogram's Bhattacharyya
    # coefficient — hence a separate threshold rather than reusing the one
    # above, which would be a silently wrong operating point.
    reid_match_thres: float = 0.55
    reid_ema_alpha: float = 0.10


@dataclass
class TrackerConfig:
    """ByteTrack association thresholds."""
    high_thresh: float = 0.50      # first association pass
    low_thresh: float = 0.10       # second pass rescues occluded boxes
    new_track_thresh: float = 0.60
    match_thresh: float = 0.80     # 1 - IoU cost ceiling
    track_buffer: int = 45         # frames a lost track survives (~1.5s @30fps)
    min_box_area: float = 400.0

    # Re-acquisition of a coasting track, matched by proximity rather than
    # overlap. Distances are in multiples of the subject's height, so the same
    # numbers work whether they are two metres away or ten.
    reacquire_gate: float = 0.55        # radius immediately after going lost
    reacquire_gate_growth: float = 0.11  # extra radius per frame missing
    reacquire_gate_max: float = 2.2     # never search the whole frame
    reacquire_height_ratio: float = 1.6  # reject implausible size jumps


@dataclass
class LockConfig:
    """Hysteresis on the target lock.

    Without hysteresis the lock flips between two people standing near each
    other and the drone yaws back and forth. Acquiring is deliberately slower
    than holding.
    """
    lock_frames: int = 5           # consecutive confirmations to acquire
    unlock_frames: int = 45        # consecutive misses before giving up (~1.5s)
    coast_frames: int = 20         # keep flying on Kalman prediction this long
    # Face is authoritative; body only sustains an existing lock.
    body_sustains_only: bool = True

    # --- re-acquisition after a total loss -----------------------------
    # Without this, losing the lock means starting over: the appearance model
    # is discarded and only a face can acquire again, which needs the subject
    # within about three metres. So one walk behind a tree ends the follow
    # until you walk back to the drone.
    #
    # Instead the learned embedding is kept for a grace window and a strong
    # body match can recover the lock on its own. Deliberately much stricter
    # than the threshold used to *sustain* a lock: sustaining only has to
    # distinguish the target from the last frame, whereas re-acquiring has to
    # pick them out of everyone in view, with no face to confirm it.
    reacquire_enabled: bool = True
    reacquire_seconds: float = 15.0
    reacquire_thres: float = 0.75   # cosine, vs 0.55 to sustain
    reacquire_frames: int = 8       # consecutive, vs 5 for a face acquire


@dataclass
class ControlConfig:
    """Image-space error -> flight setpoints.

    All three axes are PD. There is no integrator anywhere: a drone chasing a
    moving person has a permanently non-zero error, and an I term would just
    wind up and overshoot every time the subject stops.
    """
    # --- yaw: horizontal offset of the target from frame centre -------------
    yaw_kp: float = 95.0           # deg/s per unit normalised error
    yaw_kd: float = 9.0
    yaw_max: float = 120.0         # deg/s
    yaw_deadband: float = 0.04     # ±4% of half-width: stops centre jitter

    # --- pitch: hold standoff distance --------------------------------------
    # Box height as a fraction of frame height is the distance proxy. Height,
    # not width or area: a person's height is nearly invariant to which way
    # they are facing, whereas width halves when they turn sideways.
    target_height_frac: float = 0.45
    pitch_kp: float = 34.0         # deg per unit height error
    pitch_kd: float = 4.5
    pitch_max: float = 14.0        # deg — modest; T:W is only ~1.6
    pitch_deadband: float = 0.03

    # --- altitude: vertical offset trims the ToF hold setpoint ---------------
    alt_kp: float = 0.85           # m/s of setpoint rate per unit error
    alt_max_rate: float = 0.5      # m/s
    alt_min: float = 0.40          # m — below this the ToF and prop wash argue
    alt_max: float = 2.00          # m — no barometer, ToF tops out
    alt_deadband: float = 0.06
    alt_default: float = 1.20      # m — where it sits when it first locks

    roll_max: float = 10.0         # deg (unused by default; kept for strafing)

    # A person box that runs off the bottom of the frame has a centroid biased
    # upward — their legs are missing. The vertical loop reads that as "target
    # too low", descends, which brings the drone closer to the subject, which
    # clips MORE of them: a positive feedback loop straight into the ground.
    # When a box touches a horizontal frame edge its centroid is not usable for
    # altitude, so the loop holds instead of chasing it.
    edge_margin_px: float = 4.0

    # --- shaping ------------------------------------------------------------
    # Slew limit is the last line of defence: even a wild vision glitch cannot
    # command more than this much change per second.
    slew_deg_per_s: float = 110.0
    slew_alt_per_s: float = 0.6
    # While COASTING, decay the last command toward neutral instead of holding
    # it. Holding a stale yaw command is how a drone spins into a wall.
    coast_decay: float = 0.85
    # Where the target's centroid should sit, as a fraction of half-height from
    # frame centre. Positive = below centre. A standing person's box centroid
    # is around their waist, so centring it points the camera at their belt;
    # holding the centroid slightly low aims it at the chest instead.
    aim_y_bias: float = 0.12

    # --- standoff in metres -------------------------------------------------
    # Preferred over `target_height_frac` whenever the subject's real height is
    # known (set at enrolment), because it is a physical quantity: it means the
    # same thing on a different camera, lens or resolution.
    standoff_m: float = 3.0
    use_metric_standoff: bool = True

    # --- lead compensation --------------------------------------------------
    # Aim where the subject WILL be, not where they were. End-to-end latency is
    # vision (~40-70ms) + WiFi + the control loop; at 1.5 m/s and 3 m range a
    # walking person is ~75px ahead of their last reported position, which the
    # controller would otherwise chase forever without catching.
    lead_time_s: float = 0.15
    # Kalman velocity is noisy on a freshly-created track, so bound how far the
    # aim point may be thrown from the measured position.
    lead_max_px: float = 220.0

    # --- search after losing the target -------------------------------------
    # Sweep toward wherever they were last heading rather than hovering blind.
    # Only ever after a real lock: a drone that has never seen anyone must sit
    # still, not spin looking for someone.
    search_enabled: bool = True
    search_yaw_rate: float = 30.0     # deg/s
    search_timeout_s: float = 6.0     # give up and hold after this long


@dataclass
class LinkConfig:
    """UDP to the drone, telemetry back."""
    host: str = "192.168.4.1"      # ESP32 SoftAP default
    port: int = 14550
    telem_port: int = 14551
    send_hz: float = 30.0
    # If the drone hears nothing for this long it must hover then land. The
    # laptop-side mirror of the same number drives the HUD's link warning.
    timeout_ms: int = 400
    enabled: bool = True

    # Transmit each command this many times. WiFi loses packets in bursts, and
    # a burst long enough to cross `timeout_ms` puts the aircraft into failsafe
    # — which stops the follow. Copies are free to send safely because the
    # firmware rejects any sequence it has already accepted, so a duplicate is
    # discarded rather than re-applied.
    #
    # The cost is trivial: a command is 30 bytes, so 3x at 30Hz is 2.7 KB/s
    # against a camera stream running at ~500 KB/s.
    redundancy: int = 3


@dataclass
class Config:
    source: SourceConfig = field(default_factory=SourceConfig)
    camera: CameraConfig = field(default_factory=CameraConfig)
    detector: DetectorConfig = field(default_factory=DetectorConfig)
    face: FaceConfig = field(default_factory=FaceConfig)
    body: BodyConfig = field(default_factory=BodyConfig)
    tracker: TrackerConfig = field(default_factory=TrackerConfig)
    lock: LockConfig = field(default_factory=LockConfig)
    control: ControlConfig = field(default_factory=ControlConfig)
    link: LinkConfig = field(default_factory=LinkConfig)

    gallery: str = field(default_factory=lambda: _path("targets"))
    providers: tuple[str, ...] = ("CoreMLExecutionProvider",
                                  "CPUExecutionProvider")

    # ---------------------------------------------------------------- load --
    @classmethod
    def load(cls, path: str | None = None) -> "Config":
        cfg = cls()
        if not path:
            return cfg
        try:
            import yaml   # optional: only needed for --config profiles
        except ImportError:
            raise SystemExit(
                "--config needs PyYAML, which is not installed.\n"
                "  .venv/bin/python -m pip install PyYAML") from None
        if not os.path.exists(path):
            raise SystemExit(f"config not found: {path}")
        with open(path) as fh:
            raw = yaml.safe_load(fh) or {}
        for section, values in raw.items():
            if not hasattr(cfg, section):
                raise KeyError(f"{path}: unknown config section '{section}'")
            target = getattr(cfg, section)
            if not dataclasses.is_dataclass(target):
                setattr(cfg, section, values)
                continue
            known = {f.name for f in dataclasses.fields(target)}
            for key, value in (values or {}).items():
                if key not in known:
                    raise KeyError(f"{path}: unknown key '{section}.{key}'")
                setattr(target, key, value)
        return cfg

    def as_dict(self) -> dict[str, Any]:
        return dataclasses.asdict(self)
