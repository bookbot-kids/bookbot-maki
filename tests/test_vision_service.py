"""VisionService tests: the capture thread's lifecycle, staleness handling,
and its refusal to die on a faulty camera or detector."""

from __future__ import annotations

import threading
import time

import pytest

from maki_puppet.hw.camera import SimCamera
from maki_puppet.vision.detector import FaceDetection
from maki_puppet.vision.service import VisionService


def face(cx=320.0) -> FaceDetection:
    return FaceDetection(
        bbox_x=cx - 50, bbox_y=190, bbox_w=100, bbox_h=100,
        confidence=1.0, frame_w=640, frame_h=480,
    )


class FakeCamera:
    width, height = 640, 480

    def __init__(self, frames=None, fail=False):
        self.opened = False
        self.closed = False
        self.fail = fail
        self._frames = frames

    def open(self):
        self.opened = True

    def close(self):
        self.closed = True

    def read(self):
        if self.fail:
            raise RuntimeError("camera unplugged")
        return "frame"


class FakeDetector:
    name = "fake"

    def __init__(self, faces=None, fail=False):
        self.faces = faces if faces is not None else [face()]
        self.fail = fail
        self.calls = 0
        self.seen = threading.Event()

    def detect(self, frame):
        self.calls += 1
        self.seen.set()
        if self.fail:
            raise RuntimeError("detector exploded")
        return list(self.faces)


def wait_for(predicate, timeout=2.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.01)
    return False


# ── Lifecycle ────────────────────────────────────────────────────────────


def test_start_opens_the_camera_and_stop_closes_it():
    camera, detector = FakeCamera(), FakeDetector()
    service = VisionService(camera, {}, detector=detector)
    service.start()
    try:
        assert camera.opened is True
        assert service.running is True
    finally:
        service.stop()
    assert camera.closed is True
    assert service.running is False


def test_start_is_idempotent():
    camera, detector = FakeCamera(), FakeDetector()
    service = VisionService(camera, {}, detector=detector)
    service.start()
    thread = service._thread
    service.start()
    try:
        assert service._thread is thread
    finally:
        service.stop()


def test_stop_clears_the_last_face():
    camera, detector = FakeCamera(), FakeDetector()
    service = VisionService(camera, {}, detector=detector)
    service.start()
    assert wait_for(lambda: service.latest_face() is not None)
    service.stop()
    assert service.latest_face() is None


# ── Detection publishing ─────────────────────────────────────────────────


def test_latest_face_publishes_the_detected_face():
    camera, detector = FakeCamera(), FakeDetector([face(cx=200.0)])
    service = VisionService(camera, {}, detector=detector)
    service.start()
    try:
        assert wait_for(lambda: service.latest_face() is not None)
        assert service.latest_face().cx == 200.0
    finally:
        service.stop()


def test_latest_face_picks_the_most_confident_of_several():
    faces = [
        FaceDetection(0, 0, 50, 50, 0.4, 640, 480),
        FaceDetection(300, 200, 50, 50, 0.99, 640, 480),
    ]
    service = VisionService(FakeCamera(), {}, detector=FakeDetector(faces))
    service.start()
    try:
        assert wait_for(lambda: service.latest_face() is not None)
        assert service.latest_face().confidence == 0.99
    finally:
        service.stop()


def test_a_face_goes_stale_after_lost_timeout():
    """A stale detection must not keep the head locked onto someone who has
    already left the room."""
    clock = [1000.0]
    service = VisionService(
        FakeCamera(), {"lost_timeout_s": 0.5},
        detector=FakeDetector(), clock=lambda: clock[0],
    )
    service.start()
    try:
        assert wait_for(lambda: service.latest_face(1000.0) is not None)
        assert service.latest_face(1000.4) is not None   # inside the window
        assert service.latest_face(1000.6) is None       # past it
    finally:
        service.stop()


def test_no_detections_means_no_face():
    service = VisionService(FakeCamera(), {}, detector=FakeDetector([]))
    service.start()
    try:
        assert wait_for(lambda: service._detector.calls > 2)
        assert service.latest_face() is None
    finally:
        service.stop()


# ── Fault tolerance ──────────────────────────────────────────────────────


def test_a_failing_detector_does_not_kill_the_thread():
    """The robot simply stops seeing faces; the gateway stays up."""
    detector = FakeDetector(fail=True)
    service = VisionService(FakeCamera(), {}, detector=detector)
    service.start()
    try:
        assert wait_for(lambda: detector.calls >= 2)
        assert service.running is True
        assert service.latest_face() is None
    finally:
        service.stop()


def test_a_failing_camera_does_not_kill_the_thread():
    service = VisionService(FakeCamera(fail=True), {}, detector=FakeDetector())
    service.start()
    try:
        time.sleep(0.25)
        assert service.running is True
        assert service.latest_face() is None
    finally:
        service.stop()


# ── Diagnostics ──────────────────────────────────────────────────────────


def test_status_reports_the_backend_and_counters():
    service = VisionService(FakeCamera(), {}, detector=FakeDetector())
    service.start()
    try:
        assert wait_for(lambda: service.status()["detections"] > 0)
        status = service.status()
        assert status["running"] is True
        assert status["detector"] == "fake"
        assert status["frames"] >= status["detections"] > 0
        assert status["face_age_s"] is not None
    finally:
        service.stop()


# ── Sim wiring ───────────────────────────────────────────────────────────


def test_sim_camera_and_detector_produce_faces_end_to_end():
    service = VisionService(
        SimCamera({"fps": 60}), {}, sim=True,
        pose_provider=lambda: {"head_pan": 0.0, "head_tilt": 0.0},
    )
    service.start()
    try:
        assert wait_for(lambda: service.status()["frames"] > 2)
        assert service.status()["detector"] == "sim"
    finally:
        service.stop()
