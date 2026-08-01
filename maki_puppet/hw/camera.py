"""Camera drivers for MAKI puppet mode.

Same shape as :mod:`.servo_bus` / :mod:`.sim`: a real driver whose heavy
import (cv2) is deferred until :meth:`open`, and a simulator with no
dependencies at all so the gateway runs on a dev machine.

Interface (both classes): ``open()``, ``close()``, ``read() -> frame | None``,
``width``, ``height``. A frame is whatever the detector understands — a numpy
BGR array for the real camera, ``None`` for the simulator (SimFaceDetector
ignores its frame argument).
"""

from __future__ import annotations

import logging
import time
from typing import Any, Optional

log = logging.getLogger(__name__)

DEFAULT_WIDTH = 640
DEFAULT_HEIGHT = 480
DEFAULT_FPS = 30.0


class Camera:
    """OpenCV VideoCapture wrapper (USB / CSI camera via V4L2)."""

    def __init__(self, config: Optional[dict] = None) -> None:
        cfg = dict(config or {})
        self.device = cfg.get("device", 0)
        self.width = int(cfg.get("width", DEFAULT_WIDTH))
        self.height = int(cfg.get("height", DEFAULT_HEIGHT))
        self.fps = float(cfg.get("fps", DEFAULT_FPS))
        self.flip_horizontal = bool(cfg.get("flip_horizontal", False))
        self.flip_vertical = bool(cfg.get("flip_vertical", False))
        self._cap: Any = None

    def open(self) -> None:
        if self._cap is not None:
            return
        import cv2

        device = int(self.device) if str(self.device).isdigit() else self.device
        cap = cv2.VideoCapture(device)
        if not cap.isOpened():
            cap.release()
            raise RuntimeError(f"cannot open camera {self.device!r}")
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        cap.set(cv2.CAP_PROP_FPS, self.fps)
        # A 1-frame buffer keeps read() near-live; without it the tracker
        # chases a face position several frames stale.
        try:
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        except Exception:  # pragma: no cover - not every backend supports it
            pass
        self._cap = cap
        log.info("camera %r open (%dx%d @ %.0f fps)", self.device, self.width, self.height, self.fps)

    def close(self) -> None:
        if self._cap is None:
            return
        try:
            self._cap.release()
        finally:
            self._cap = None
        log.debug("camera closed")

    def read(self) -> Optional[Any]:
        """Grab one frame, or None if the capture failed this tick."""
        if self._cap is None:
            return None
        ok, frame = self._cap.read()
        if not ok or frame is None:
            return None
        # Orientation must be corrected here, before detection: the tracker
        # derives its bearings from pixel positions, so an inverted frame sends
        # the head the wrong way on both axes.
        if self.flip_horizontal or self.flip_vertical:
            import cv2

            if self.flip_horizontal and self.flip_vertical:
                frame = cv2.flip(frame, -1)
            elif self.flip_horizontal:
                frame = cv2.flip(frame, 1)
            else:
                frame = cv2.flip(frame, 0)
        return frame


class SimCamera:
    """Frameless stand-in: paces reads at the configured fps and returns None.

    Pairs with :class:`~maki_puppet.vision.detector.SimFaceDetector`, which
    synthesizes detections without looking at the frame.
    """

    def __init__(self, config: Optional[dict] = None) -> None:
        cfg = dict(config or {})
        self.width = int(cfg.get("width", DEFAULT_WIDTH))
        self.height = int(cfg.get("height", DEFAULT_HEIGHT))
        self.fps = float(cfg.get("fps", DEFAULT_FPS))
        self._open = False
        self._last_read = 0.0

    def open(self) -> None:
        self._open = True
        self._last_read = 0.0
        log.debug("SimCamera open (%dx%d)", self.width, self.height)

    def close(self) -> None:
        self._open = False
        log.debug("SimCamera closed")

    def read(self) -> Optional[Any]:
        if not self._open:
            return None
        period = 1.0 / max(self.fps, 1.0)
        now = time.monotonic()
        remaining = period - (now - self._last_read)
        if remaining > 0:
            time.sleep(remaining)
        self._last_read = time.monotonic()
        return None
