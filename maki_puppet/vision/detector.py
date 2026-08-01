"""Face detection backends, ported from face_tracker_node.py.

Three backends behind one interface (``detect(frame) -> list[FaceDetection]``):

  * ``yunet`` — OpenCV's YuNet DNN face detector. Best accuracy; needs the
    ONNX model file. This is the ROS node's default.
  * ``haar``  — Haar cascade. No model download, ships with OpenCV, weaker.
  * ``sim``   — no OpenCV at all: replays a scripted face path so the whole
    tracking stack runs and is testable on a dev machine.

OpenCV is imported lazily inside the real backends so this module — and
therefore ``--sim`` — imports cleanly on machines without it.
"""

from __future__ import annotations

import logging
import math
import os
from dataclasses import dataclass
from typing import Any, List, Mapping, Optional, Sequence

log = logging.getLogger(__name__)

_CASCADE_NAME = "haarcascade_frontalface_default.xml"

# Vendored in models/ (see models/README.md) so a fresh deploy needs no model
# download. The ROS node resolved the same file from its ament share dir.
YUNET_MODEL_NAME = "face_detection_yunet_2023mar.onnx"
_MODELS_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
    "models",
)


def default_yunet_model_path() -> Optional[str]:
    """The vendored YuNet model, or None if it isn't present."""
    path = os.path.join(_MODELS_DIR, YUNET_MODEL_NAME)
    return path if os.path.isfile(path) else None


@dataclass(frozen=True)
class FaceDetection:
    """One detected face, in original (pre-downscale) frame pixels."""

    bbox_x: float
    bbox_y: float
    bbox_w: float
    bbox_h: float
    confidence: float
    frame_w: int
    frame_h: int

    @property
    def cx(self) -> float:
        return self.bbox_x + self.bbox_w / 2.0

    @property
    def cy(self) -> float:
        return self.bbox_y + self.bbox_h / 2.0


def select_primary(faces: Sequence[FaceDetection]) -> Optional[FaceDetection]:
    """The face to track: highest confidence wins."""
    if not faces:
        return None
    return max(faces, key=lambda f: f.confidence)


def _load_default_cascade() -> str:
    """Locate the bundled Haar cascade.

    ``cv2.data.haarcascades`` only exists in pip's opencv-python, not in the
    system python3-opencv package — so probe pip first, then the Debian paths
    used on the Pi.
    """
    import cv2

    candidates = [
        (getattr(cv2, "data", None) and cv2.data.haarcascades + _CASCADE_NAME),
        f"/usr/share/opencv4/haarcascades/{_CASCADE_NAME}",
        f"/usr/share/opencv/haarcascades/{_CASCADE_NAME}",
    ]
    for path in candidates:
        if path and os.path.isfile(path):
            return path
    raise FileNotFoundError(
        f"cannot find {_CASCADE_NAME}; install python3-opencv or opencv-python"
    )


class YuNetDetector:
    """OpenCV YuNet DNN detector."""

    name = "yunet"

    def __init__(self, config: Optional[Mapping[str, Any]] = None) -> None:
        cfg = dict(config or {})
        # Empty config value = auto-resolve the vendored model, matching the
        # ROS node's behaviour.
        self.model_path = str(cfg.get("yunet_model_path") or "").strip() or (
            default_yunet_model_path() or ""
        )
        self.score_threshold = float(cfg.get("yunet_score_threshold", 0.6))
        self.nms_threshold = float(cfg.get("yunet_nms_threshold", 0.3))
        self.top_k = int(cfg.get("yunet_top_k", 5))
        self.detection_scale = float(cfg.get("detection_scale", 0.5))
        self._detector: Any = None
        self._input_size = (0, 0)

    def _ensure(self) -> Any:
        if self._detector is None:
            # Checked before importing cv2: a missing model is the common
            # misconfiguration, and its message is more useful than an
            # ImportError from a machine that simply has no OpenCV.
            if not self.model_path or not os.path.isfile(self.model_path):
                raise FileNotFoundError(
                    f"YuNet model not found at {self.model_path!r}; set "
                    "vision.detector.yunet_model_path or use face_detector: haar"
                )
            import cv2

            self._detector = cv2.FaceDetectorYN.create(
                self.model_path, "", (320, 320),
                self.score_threshold, self.nms_threshold, self.top_k,
            )
        return self._detector

    def detect(self, frame: Any) -> List[FaceDetection]:
        import cv2

        detector = self._ensure()
        oh, ow = frame.shape[:2]
        scale = self.detection_scale
        small = (
            cv2.resize(frame, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
            if scale < 1.0 else frame
        )
        h, w = small.shape[:2]
        if self._input_size != (w, h):
            detector.setInputSize((w, h))
            self._input_size = (w, h)
        _retval, faces = detector.detect(small)
        if faces is None or len(faces) == 0:
            return []
        inv = (1.0 / scale) if scale < 1.0 else 1.0
        return [
            FaceDetection(
                bbox_x=float(row[0]) * inv,
                bbox_y=float(row[1]) * inv,
                bbox_w=float(row[2]) * inv,
                bbox_h=float(row[3]) * inv,
                confidence=float(row[-1]),
                frame_w=ow,
                frame_h=oh,
            )
            for row in faces
        ]


class HaarDetector:
    """Haar cascade detector. Confidence is synthetic (0.8) — the cascade
    reports none, and the tracker only uses it to rank faces."""

    name = "haar"

    def __init__(self, config: Optional[Mapping[str, Any]] = None) -> None:
        cfg = dict(config or {})
        self.cascade_path = cfg.get("cascade_path") or None
        self.scale_factor = float(cfg.get("face_scale_factor", 1.1))
        self.min_neighbors = int(cfg.get("face_min_neighbors", 5))
        self.min_size_px = int(cfg.get("face_min_size_px", 40))
        self.detection_scale = float(cfg.get("detection_scale", 0.5))
        self._cascade: Any = None

    def _ensure(self) -> Any:
        if self._cascade is None:
            import cv2

            path = self.cascade_path or _load_default_cascade()
            cascade = cv2.CascadeClassifier(str(path))
            if cascade.empty():
                raise FileNotFoundError(f"failed to load Haar cascade from {path!r}")
            self._cascade = cascade
        return self._cascade

    def detect(self, frame: Any) -> List[FaceDetection]:
        import cv2

        cascade = self._ensure()
        oh, ow = frame.shape[:2]
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        scale = self.detection_scale
        small = (
            cv2.resize(gray, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
            if scale < 1.0 else gray
        )
        min_sz = max(8, int(self.min_size_px * scale))
        faces = cascade.detectMultiScale(
            small,
            scaleFactor=max(1.01, self.scale_factor),
            minNeighbors=max(1, self.min_neighbors),
            minSize=(min_sz, min_sz),
        )
        if len(faces) == 0:
            return []
        inv = (1.0 / scale) if scale < 1.0 else 1.0
        return [
            FaceDetection(
                bbox_x=float(x) * inv, bbox_y=float(y) * inv,
                bbox_w=float(w) * inv, bbox_h=float(h) * inv,
                confidence=0.8, frame_w=ow, frame_h=oh,
            )
            for (x, y, w, h) in faces
        ]


class SimFaceDetector:
    """Scripted detector for ``--sim``: a synthetic person standing in front of
    the robot, so the whole tracking stack runs and can be tuned without a
    camera.

    Deliberately CLOSED-LOOP. The face is scripted in *world* coordinates —
    a bearing and elevation relative to the robot's body — and then projected
    back into pixels through the current head pose. That means turning the
    head really does bring the face toward the centre of frame, and turning
    too far really does lose it out of frame. An open-loop version that just
    swept a pixel position would let the head integrate away to its mechanical
    limit, which is the one failure mode the real geometry can't produce.

    ``pose_provider`` supplies the live head pose; without one the head is
    assumed level, which degrades to the open-loop behaviour.
    """

    name = "sim"

    def __init__(
        self,
        config: Optional[Mapping[str, Any]] = None,
        *,
        pose_provider: Optional[Any] = None,
    ) -> None:
        cfg = dict(config or {})
        self.frame_w = int(cfg.get("width", 640))
        self.frame_h = int(cfg.get("height", 480))
        self.hfov = math.radians(float(cfg.get("camera_hfov_deg", 62.0)))
        self.vfov = math.radians(float(cfg.get("camera_vfov_deg", 49.0)))
        self.period_s = float(cfg.get("sim_face_period_s", 12.0))
        # How far the synthetic person wanders either side of centre. 0.45 rad
        # (~26 deg) is well inside head_pan's range but outside a single
        # frame's FOV, so the head genuinely has to follow.
        self.bearing_amplitude_rad = float(cfg.get("sim_face_bearing_rad", 0.45))
        self.elevation_amplitude_rad = float(cfg.get("sim_face_elevation_rad", 0.10))
        self.face_px = int(cfg.get("sim_face_size_px", 120))
        self.pose_provider = pose_provider

    def world_position(self, t: float) -> tuple:
        """Scripted (bearing, elevation) of the synthetic face, in radians."""
        phase = 2.0 * math.pi * (t / max(self.period_s, 0.1))
        return (
            self.bearing_amplitude_rad * math.sin(phase),
            self.elevation_amplitude_rad * math.sin(2.0 * phase),
        )

    def detect(self, frame: Any, now: Optional[float] = None) -> List[FaceDetection]:
        import time as _time

        t = _time.monotonic() if now is None else now
        bearing, elevation = self.world_position(t)

        pose = {}
        if self.pose_provider is not None:
            try:
                pose = self.pose_provider() or {}
            except Exception:  # pragma: no cover - defensive
                log.exception("sim pose provider failed")
        head_pan = float(pose.get("head_pan", 0.0))
        head_tilt = float(pose.get("head_tilt", 0.0))

        # Inverse of FaceTracker.pixel_to_body_angles.
        nx = (head_pan - bearing) / self.hfov
        ny = (elevation - head_tilt) / self.vfov
        cx = (nx + 0.5) * self.frame_w
        cy = (ny + 0.5) * self.frame_h

        half = self.face_px / 2.0
        if not (half <= cx <= self.frame_w - half and half <= cy <= self.frame_h - half):
            return []  # the person is out of shot — exactly as a real camera would report
        return [
            FaceDetection(
                bbox_x=cx - half,
                bbox_y=cy - half,
                bbox_w=float(self.face_px),
                bbox_h=float(self.face_px),
                confidence=1.0,
                frame_w=self.frame_w,
                frame_h=self.frame_h,
            )
        ]


_BACKENDS = {"yunet": YuNetDetector, "haar": HaarDetector, "sim": SimFaceDetector}


def build_detector(
    config: Optional[Mapping[str, Any]] = None,
    *,
    sim: bool = False,
    pose_provider: Optional[Any] = None,
) -> Any:
    """Build the configured detector. ``sim`` forces the scripted backend.

    ``pose_provider`` is only meaningful for the sim backend, which needs the
    live head pose to project its world-frame face into the frame.
    """
    cfg = dict(config or {})
    name = "sim" if sim else str(cfg.get("face_detector", "yunet")).lower()
    backend = _BACKENDS.get(name)
    if backend is None:
        raise KeyError(
            f"unknown face detector '{name}'; known: {', '.join(sorted(_BACKENDS))}"
        )
    if backend is SimFaceDetector:
        return backend(cfg, pose_provider=pose_provider)
    return backend(cfg)
