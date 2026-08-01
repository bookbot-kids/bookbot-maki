"""FaceTrackingBehavior tests: arming, standing down, and layer hygiene.

The behavior moves the robot with nobody asking it to, so the cases that
matter most are the ones where it must NOT move: e-stop, client work in
flight, and no face present.
"""

from __future__ import annotations

import pytest

from maki_puppet.vision.behavior import FaceTrackingBehavior
from maki_puppet.vision.detector import FaceDetection
from maki_puppet.vision.tracker import TrackerConfig

FRAME_W, FRAME_H = 640, 480


def face_at(cx=FRAME_W * 0.8, cy=FRAME_H / 2) -> FaceDetection:
    return FaceDetection(
        bbox_x=cx - 50, bbox_y=cy - 50, bbox_w=100, bbox_h=100,
        confidence=1.0, frame_w=FRAME_W, frame_h=FRAME_H,
    )


class FakeEngine:
    def __init__(self, estopped=False, client_work=False):
        self.estopped = estopped
        self._client_work = client_work

    def has_client_work(self):
        return self._client_work


class FakeMotion:
    def __init__(self):
        self.layers = {}
        self.released = []
        self.posture = {}

    def set_layer(self, layer, targets):
        self.layers[layer] = dict(targets)

    def release_layer(self, layer):
        self.released.append(layer)
        self.layers.pop(layer, None)

    def set_posture(self, targets):
        self.posture = dict(targets)

    def clear_posture(self):
        self.posture = {}

    def has_posture(self):
        return bool(self.posture)

    def pose_rad(self):
        return {"head_pan": 0.0, "head_tilt": 0.0, "eyes_pan": 0.0, "eyes_tilt": 0.0}


class FakeVision:
    def __init__(self, face=None):
        self.face = face

    def latest_face(self, now=None):
        return self.face


class Clock:
    def __init__(self, t=1000.0):
        self.t = t

    def __call__(self):
        return self.t

    def advance(self, dt):
        self.t += dt


def make(**kw):
    clock = kw.pop("clock", None) or Clock()
    engine = kw.pop("engine", None) or FakeEngine()
    vision = kw.pop("vision", None) or FakeVision(face_at())
    motion = FakeMotion()
    behavior = FaceTrackingBehavior(
        engine, motion, vision,
        TrackerConfig(face_smoothing_alpha=0.0, enable_eye_saccades=False),
        kw or {},
        clock=clock,
    )
    return behavior, motion, vision, engine, clock


def tick(behavior, clock, n=1, dt=0.1):
    for _ in range(n):
        behavior._poll()
        clock.advance(dt)


# ── Arming ───────────────────────────────────────────────────────────────


def test_arms_and_drives_the_tracking_layer_when_a_face_appears():
    behavior, motion, _, _, clock = make()
    tick(behavior, clock, n=3)
    assert behavior.armed is True
    assert "tracking" in motion.layers
    assert set(motion.layers["tracking"]) == {
        "head_pan", "head_tilt", "eyes_pan", "eyes_tilt"
    }


def test_does_not_arm_without_a_face():
    behavior, motion, _, _, clock = make(vision=FakeVision(None))
    tick(behavior, clock, n=5)
    assert behavior.armed is False
    assert motion.layers == {}


def test_eyes_only_never_writes_head_joints():
    behavior, motion, _, _, clock = make(eyes_only=True)
    tick(behavior, clock, n=3)
    assert set(motion.layers["tracking"]) == {"eyes_pan", "eyes_tilt"}


# ── Standing down ────────────────────────────────────────────────────────


def test_stands_down_under_estop():
    """Nothing autonomous may move the robot while e-stopped."""
    engine = FakeEngine()
    behavior, motion, _, _, clock = make(engine=engine)
    tick(behavior, clock, n=3)
    assert behavior.armed is True

    engine.estopped = True
    tick(behavior, clock, n=1)
    assert behavior.armed is False
    assert "tracking" in motion.released


def test_stands_down_while_a_client_performance_is_running():
    """A client `track` act writes the same layer; two authors on one layer is
    a fight neither wins."""
    engine = FakeEngine()
    behavior, motion, _, _, clock = make(engine=engine)
    tick(behavior, clock, n=3)
    assert behavior.armed is True

    engine._client_work = True
    tick(behavior, clock, n=1)
    assert behavior.armed is False
    assert "tracking" in motion.released


def test_rearms_after_client_work_finishes():
    engine = FakeEngine()
    behavior, motion, _, _, clock = make(engine=engine)
    tick(behavior, clock, n=3)
    engine._client_work = True
    tick(behavior, clock, n=1)
    assert behavior.armed is False

    engine._client_work = False
    tick(behavior, clock, n=3)
    assert behavior.armed is True
    assert "tracking" in motion.layers


# ── Reading: yield to a held posture ─────────────────────────────────────


def test_a_held_posture_stops_tracking():
    """Opening a book installs a head-down posture. Following a face over the
    top of it would pull the head straight back off the page."""
    behavior, motion, _, _, clock = make()
    tick(behavior, clock, n=3)
    assert behavior.armed is True

    motion.set_posture({"head_pan": 0.0, "head_tilt": 1.0})   # book opened
    tick(behavior, clock, n=2)
    assert behavior.armed is False
    assert "tracking" in motion.released


def test_tracking_resumes_when_the_posture_is_cleared():
    """Closing the book clears the posture; face-following comes back on its
    own with no further prompting."""
    behavior, motion, _, _, clock = make()
    tick(behavior, clock, n=3)
    motion.set_posture({"head_tilt": 1.0})
    tick(behavior, clock, n=2)
    assert behavior.armed is False

    motion.clear_posture()                                     # book closed
    tick(behavior, clock, n=3)
    assert behavior.armed is True
    assert "tracking" in motion.layers


def test_the_head_is_left_alone_for_the_whole_time_a_book_is_open():
    """The layer must stay released for the entire reading session, not just
    the first tick after the posture appears."""
    behavior, motion, _, _, clock = make()
    tick(behavior, clock, n=3)
    motion.set_posture({"head_tilt": 1.0})
    tick(behavior, clock, n=2)
    motion.layers.pop("tracking", None)
    motion.released.clear()

    tick(behavior, clock, n=40, dt=0.1)      # 4 s of "reading"
    assert "tracking" not in motion.layers, "must not touch the head while reading"
    assert behavior.armed is False


def test_yield_to_posture_can_be_turned_off():
    behavior, motion, _, _, clock = make(yield_to_posture=False)
    tick(behavior, clock, n=3)
    motion.set_posture({"head_tilt": 1.0})
    tick(behavior, clock, n=2)
    assert behavior.armed is True


def test_a_motion_service_without_the_posture_hook_still_works():
    """Degrades rather than raising every tick."""
    class Older(FakeMotion):
        has_posture = None

    behavior, motion, _, _, clock = make()
    behavior._motion = Older()
    tick(behavior, clock, n=3)
    assert behavior.armed is True


# ── Losing the face ──────────────────────────────────────────────────────


def test_holds_briefly_then_releases_when_the_face_goes():
    behavior, motion, vision, _, clock = make(release_after_s=2.0)
    tick(behavior, clock, n=3)
    held = dict(motion.layers["tracking"])

    vision.face = None
    tick(behavior, clock, n=5, dt=0.1)          # 0.5 s — inside the grace
    assert behavior.armed is True
    assert motion.layers["tracking"] == held, "should hold, not snap anywhere"

    clock.advance(2.0)                           # past release_after_s
    behavior._poll()
    assert behavior.armed is False
    assert "tracking" in motion.released


def test_a_brief_detection_dropout_does_not_disarm():
    behavior, motion, vision, _, clock = make(release_after_s=2.0)
    tick(behavior, clock, n=3)
    vision.face = None
    tick(behavior, clock, n=2, dt=0.1)
    vision.face = face_at()
    tick(behavior, clock, n=2, dt=0.1)
    assert behavior.armed is True
    assert motion.released == []


def test_a_new_sighting_resets_the_anchor():
    """Re-arming with a stale anchor would drive toward where the LAST person
    stood, not the new one."""
    behavior, motion, vision, _, clock = make(release_after_s=0.1)
    tick(behavior, clock, n=3)
    vision.face = None
    clock.advance(1.0)
    behavior._poll()
    assert behavior.armed is False

    vision.face = face_at()
    behavior._poll()
    assert behavior.armed is True
    assert behavior._tracker._seeded is False or behavior._tracker._smooth_cx is not None


# ── Disabling ────────────────────────────────────────────────────────────


def test_disabled_by_config_never_starts():
    behavior, motion, _, _, clock = make(enabled=False)
    assert behavior.enabled is False
    behavior.start()
    assert behavior._task is None


def test_no_vision_service_means_disabled():
    behavior = FaceTrackingBehavior(FakeEngine(), FakeMotion(), None, TrackerConfig(), {})
    assert behavior.enabled is False
    behavior.start()
    assert behavior._task is None


def test_stop_releases_the_layer():
    behavior, motion, _, _, clock = make()
    tick(behavior, clock, n=3)
    assert behavior.armed is True
    behavior.stop()
    assert behavior.armed is False
    assert "tracking" in motion.released


# ── Robustness ───────────────────────────────────────────────────────────


def test_a_failing_tick_does_not_leave_the_layer_claimed_forever():
    """A raising motion service must not kill the behavior task."""
    behavior, motion, _, _, clock = make()

    def boom(*a, **k):
        raise RuntimeError("motion service down")

    motion.set_layer = boom
    with pytest.raises(RuntimeError):
        behavior._poll()      # _run swallows this; _poll itself propagates
