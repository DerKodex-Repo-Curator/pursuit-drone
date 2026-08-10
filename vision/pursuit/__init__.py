"""PURSUIT-1 off-board vision — person detection, re-identification and
follow-control for the PCB-frame quadcopter.

The drone streams camera frames to a laptop over WiFi; this package turns them
into flight setpoints and sends them back over UDP. Everything heavy runs here
rather than on the ESP32-S3, which does stabilisation and nothing else.

Pipeline:

    frame -> YOLO11n person boxes -> ByteTrack -> SCRFD face -> ArcFace 512-d
          -> target lock (face confirms, colour sustains) -> PD control
          -> CRC-checked UDP command

All inference is ONNX Runtime on CoreML. No torch anywhere.
"""

__version__ = "1.0.0"

from .config import Config                                        # noqa: F401
from .lock import LockState                                       # noqa: F401
from .pipeline import FrameResult, Pipeline                        # noqa: F401
from .protocol import Command, Telemetry                          # noqa: F401

__all__ = ["Config", "Pipeline", "FrameResult", "LockState",
           "Command", "Telemetry", "__version__"]
