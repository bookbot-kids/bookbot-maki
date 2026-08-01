"""Vision: camera capture, face detection, and the face-tracking control law.

Ported from the ROS `maki_vision` package (`face_tracker_node.py`) with the
ROS plumbing removed.  The split mirrors the rest of maki_puppet:

  * :mod:`.detector`  — pure detection backends (YuNet DNN / Haar cascade)
  * :mod:`.tracker`   — the pure control law (no I/O, fully testable)
  * :mod:`.service`   — background capture+detect thread, like MotionService
  * :mod:`.behavior`  — autonomous face following, armed by the robot itself

The tracker feeds the ``tracking`` motion layer (priority 50), which the
LayerBlender already reserves for exactly this purpose.
"""

from .behavior import FaceTrackingBehavior
from .detector import FaceDetection, build_detector
from .service import VisionService
from .tracker import FaceTracker, TrackerConfig, TrackerOutput

__all__ = [
    "FaceDetection",
    "FaceTracker",
    "FaceTrackingBehavior",
    "TrackerConfig",
    "TrackerOutput",
    "VisionService",
    "build_detector",
]
