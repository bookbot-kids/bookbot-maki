"""VisionService — background capture + face detection.

Mirrors MotionService's shape: owns a daemon thread, is safe to call from the
asyncio loop, and exposes a small thread-safe snapshot API. It does NOT touch
the servos — detection and control are deliberately separate, so the control
law (:class:`~maki_puppet.vision.tracker.FaceTracker`) stays pure and the
`track` action decides when the robot actually moves.

Detection runs continuously while started; a face is considered current for
``lost_timeout_s`` after its last sighting, which keeps the tracker steady
through the occasional dropped detection frame.
"""

from __future__ import annotations

import logging
import threading
import time
from typing import Any, Callable, Mapping, Optional

from .detector import FaceDetection, build_detector, select_primary

log = logging.getLogger(__name__)

DEFAULT_LOST_TIMEOUT_S = 0.5

# How long the loop naps after a failed capture, so a disconnected camera
# doesn't spin the thread at full tilt.
_CAPTURE_RETRY_S = 0.1


class VisionService:
    """Runs the camera + detector in a thread; publishes the newest face."""

    def __init__(
        self,
        camera: Any,
        config: Optional[Mapping[str, Any]] = None,
        *,
        sim: bool = False,
        detector: Optional[Any] = None,
        pose_provider: Optional[Callable[[], Mapping[str, float]]] = None,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        cfg = dict(config or {})
        self._camera = camera
        self._clock = clock
        detector_cfg = dict(cfg.get("detector") or {})
        detector_cfg.setdefault("width", getattr(camera, "width", 640))
        detector_cfg.setdefault("height", getattr(camera, "height", 480))
        # The sim detector projects its world-frame face through the live head
        # pose, so it needs the same FOV the tracker uses to invert it.
        for key in ("camera_hfov_deg", "camera_vfov_deg"):
            if key in (cfg.get("tracking") or {}):
                detector_cfg.setdefault(key, (cfg["tracking"])[key])
        self._detector = detector if detector is not None else build_detector(
            detector_cfg, sim=sim, pose_provider=pose_provider
        )
        self._lost_timeout_s = float(cfg.get("lost_timeout_s", DEFAULT_LOST_TIMEOUT_S))
        # Detect on every Nth frame. Detection dominates the CPU cost, and the
        # control loop is capped at 20 Hz anyway, so a 30 fps camera loses
        # nothing at N=2 — it just stops burning a core on the Pi.
        self._every_n = max(1, int(cfg.get("process_every_n_frames", 1)))
        self._frame_counter = 0

        self._lock = threading.Lock()
        self._face: Optional[FaceDetection] = None
        self._face_ts = 0.0
        self._frames = 0
        self._detections = 0
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()

    # ── Lifecycle ──────────────────────────────────────────────────────

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._camera.open()
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._run, name="vision", daemon=True
        )
        self._thread.start()
        log.info("vision service started (detector: %s)", getattr(self._detector, "name", "?"))

    def stop(self) -> None:
        self._stop.set()
        thread, self._thread = self._thread, None
        if thread is not None:
            thread.join(timeout=2.0)
        try:
            self._camera.close()
        except Exception:  # pragma: no cover - defensive
            log.exception("camera close failed")
        with self._lock:
            self._face = None
        log.debug("vision service stopped")

    @property
    def running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    # ── Snapshot API (thread-safe; call from the event loop) ───────────

    def latest_face(self, now: Optional[float] = None) -> Optional[FaceDetection]:
        """The most recent face, or None if none has been seen within
        ``lost_timeout_s``."""
        now = self._clock() if now is None else now
        with self._lock:
            if self._face is None or now - self._face_ts > self._lost_timeout_s:
                return None
            return self._face

    def status(self) -> dict:
        """Diagnostics for `state` payloads and the smoke script."""
        with self._lock:
            face = self._face
            age = self._clock() - self._face_ts if face is not None else None
            return {
                "running": self.running,
                "detector": getattr(self._detector, "name", "unknown"),
                "frames": self._frames,
                "detections": self._detections,
                "face_age_s": round(age, 3) if age is not None else None,
            }

    # ── Thread body ────────────────────────────────────────────────────

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                frame = self._camera.read()
            except Exception:
                log.exception("camera read failed")
                self._stop.wait(_CAPTURE_RETRY_S)
                continue
            if frame is None and not self._is_frameless():
                self._stop.wait(_CAPTURE_RETRY_S)
                continue
            self._frame_counter += 1
            if self._frame_counter % self._every_n:
                continue      # grabbed to keep the capture buffer live, not detected on
            try:
                faces = self._detector.detect(frame)
            except Exception:
                # A detector fault must not kill the thread — the robot simply
                # stops seeing faces and the track action releases its layer.
                log.exception("face detection failed")
                self._stop.wait(_CAPTURE_RETRY_S)
                continue
            primary = select_primary(faces)
            with self._lock:
                self._frames += 1
                if primary is not None:
                    self._detections += 1
                    self._face = primary
                    self._face_ts = self._clock()

    def _is_frameless(self) -> bool:
        """SimCamera yields None frames by design; a real camera returning
        None means a failed grab."""
        return getattr(self._detector, "name", "") == "sim"
