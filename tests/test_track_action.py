"""`track` action tests: validation, layer usage, duration, cancellation, and
the reject path when no camera is wired up."""

from __future__ import annotations

import asyncio

import pytest

from maki_puppet.actions import ACTIONS
from maki_puppet.actions.context import ActionContext
from maki_puppet.actions.library import run_track
from maki_puppet.protocol import (
    E_VISION_UNAVAILABLE,
    ProtocolError,
    action_channels,
    parse_step,
)
from maki_puppet.vision.detector import FaceDetection
from maki_puppet.vision.tracker import TrackerConfig

# No pytestmark here: asyncio_mode = "auto" already handles the async tests,
# and a blanket mark would warn on the sync validation ones.

FRAME_W, FRAME_H = 640, 480


def face_at(cx, cy=FRAME_H / 2) -> FaceDetection:
    return FaceDetection(
        bbox_x=cx - 50, bbox_y=cy - 50, bbox_w=100, bbox_h=100,
        confidence=1.0, frame_w=FRAME_W, frame_h=FRAME_H,
    )


class FakeMotion:
    def __init__(self, pose=None):
        self.layers = {}
        self.released = []
        self.feeds = []
        self._pose = pose or {"head_pan": 0.0, "head_tilt": 0.0,
                              "eyes_pan": 0.0, "eyes_tilt": 0.0}

    def set_layer(self, layer, targets):
        self.layers[layer] = dict(targets)
        self.feeds.append((layer, dict(targets)))

    def release_layer(self, layer):
        self.released.append(layer)
        self.layers.pop(layer, None)

    def pose_rad(self):
        return dict(self._pose)


class FakeVision:
    def __init__(self, face=None):
        self.face = face
        self.calls = 0

    def latest_face(self, now=None):
        self.calls += 1
        return self.face


class FakeClock:
    """Virtual time: ctx.sleep advances the clock instead of really waiting,
    so a 5 s track action runs instantly and deterministically."""

    def __init__(self):
        self.t = 1000.0

    def now(self):
        return self.t

    async def sleep(self, seconds):
        self.t += max(seconds, 0.0)
        await asyncio.sleep(0)


def make_ctx(vision, motion=None, clock=None):
    clock = clock or FakeClock()
    return ActionContext(
        motion=motion or FakeMotion(),
        led=None,
        vision=vision,
        tracker_config=TrackerConfig(face_smoothing_alpha=0.0),
        clock=clock.now,
        sleep=clock.sleep,
    ), clock


# ── Registration and validation ──────────────────────────────────────────


def test_track_is_a_registered_motion_action():
    assert "track" in ACTIONS
    assert action_channels("track") == frozenset({"motion"})


def test_track_defaults():
    step = parse_step({"kind": "track"})
    assert step.args == {"target": "face", "duration_ms": 5000, "eyes_only": False}


def test_track_accepts_explicit_args():
    step = parse_step({"kind": "track", "target": "face",
                       "duration_ms": 2500, "eyes_only": True})
    assert step.args == {"target": "face", "duration_ms": 2500, "eyes_only": True}


@pytest.mark.parametrize("bad", [
    {"kind": "track", "target": "hand"},
    {"kind": "track", "eyes_only": "yes"},
    {"kind": "track", "duration_ms": -1},
    {"kind": "track", "duration_ms": 60001},
])
def test_track_rejects_bad_args(bad):
    with pytest.raises(ProtocolError):
        parse_step(bad)


def test_track_composes_inside_par_and_seq():
    step = parse_step({"par": [{"kind": "track", "duration_ms": 1000},
                               {"kind": "led", "color": {"r": 0, "g": 0, "b": 255}}]})
    assert len(step.children) == 2


# ── Execution ────────────────────────────────────────────────────────────


async def test_track_feeds_the_tracking_layer():
    vision = FakeVision(face_at(FRAME_W * 0.8))
    ctx, _ = make_ctx(vision)
    await run_track(ctx, {"target": "face", "duration_ms": 1000, "eyes_only": False})

    assert "tracking" in ctx.motion.layers
    assert ctx.touched_layers == {"tracking"}, "engine must release exactly this layer"
    assert set(ctx.motion.layers["tracking"]) == {
        "head_pan", "head_tilt", "eyes_pan", "eyes_tilt"
    }


async def test_track_runs_for_the_requested_duration():
    vision = FakeVision(face_at(FRAME_W * 0.8))
    ctx, clock = make_ctx(vision)
    start = clock.now()
    await run_track(ctx, {"target": "face", "duration_ms": 3000, "eyes_only": False})
    assert clock.now() - start == pytest.approx(3.0, abs=0.06)


async def test_eyes_only_track_leaves_the_head_alone():
    vision = FakeVision(face_at(FRAME_W * 0.8))
    ctx, _ = make_ctx(vision)
    await run_track(ctx, {"target": "face", "duration_ms": 1000, "eyes_only": True})
    assert set(ctx.motion.layers["tracking"]) == {"eyes_pan", "eyes_tilt"}


async def test_track_holds_its_last_targets_when_the_face_is_lost():
    """A person stepping out of frame should not snap the head anywhere — the
    layer's own claim timeout is what eventually hands the joints back."""
    vision = FakeVision(face_at(FRAME_W * 0.8))
    ctx, _ = make_ctx(vision)
    task = asyncio.ensure_future(
        run_track(ctx, {"target": "face", "duration_ms": 2000, "eyes_only": False})
    )
    await asyncio.sleep(0)
    while not task.done() and len(ctx.motion.feeds) < 3:
        await asyncio.sleep(0)
    held = dict(ctx.motion.layers["tracking"])
    vision.face = None                      # face disappears
    await task

    assert ctx.motion.layers["tracking"] == held, "targets should be held, not reset"
    assert "tracking" not in ctx.motion.released, "the action must not release it early"


async def test_track_with_no_face_at_all_never_feeds_the_layer():
    ctx, _ = make_ctx(FakeVision(None))
    await run_track(ctx, {"target": "face", "duration_ms": 1000, "eyes_only": False})
    assert ctx.motion.layers == {}


async def test_track_is_cancellable():
    vision = FakeVision(face_at(FRAME_W * 0.8))
    ctx = ActionContext(motion=FakeMotion(), led=None, vision=vision,
                        tracker_config=TrackerConfig())
    task = asyncio.ensure_future(
        run_track(ctx, {"target": "face", "duration_ms": 60000, "eyes_only": False})
    )
    await asyncio.sleep(0.12)
    task.cancel()
    with pytest.raises(asyncio.CancelledError):
        await task
    assert ctx.touched_layers == {"tracking"}, "engine still releases what it touched"


# ── Unavailable vision ───────────────────────────────────────────────────


async def test_track_without_a_vision_service_rejects():
    """Better a clear error than a robot that accepted 'follow me' and stood
    perfectly still."""
    ctx = ActionContext(motion=FakeMotion(), led=None, vision=None)
    with pytest.raises(ProtocolError) as exc:
        await run_track(ctx, {"target": "face", "duration_ms": 1000, "eyes_only": False})
    assert exc.value.code == E_VISION_UNAVAILABLE
