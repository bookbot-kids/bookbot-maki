"""IdleBehavior tests: arming after inactivity, keepalive + blinks through
the engine, instant preemption by client work, and estop pause."""

from __future__ import annotations

import asyncio
import random

import pytest

from maki_puppet.engine import ActionEngine, Performance
from maki_puppet.idle import IdleBehavior
from maki_puppet.protocol import parse_step

pytestmark = pytest.mark.asyncio


class FakeMotion:
    def __init__(self):
        self.layers: dict = {}
        self.feed_counts: dict = {}
        self.released: list = []

    def set_layer(self, layer, targets):
        self.layers[layer] = dict(targets)
        self.feed_counts[layer] = self.feed_counts.get(layer, 0) + 1

    def release_layer(self, layer):
        self.released.append(layer)
        self.layers.pop(layer, None)

    def pose_rad(self):
        return {"head_pan": 0.0, "head_tilt": 0.05}

    def estop(self, engaged):
        pass


class FakeLed:
    def __init__(self):
        self.calls: list = []
        self.current = {"animation": "boot_pulse"}

    def set_animation(self, name):
        self.calls.append(name)
        self.current = {"animation": name}

    def set_color(self, r, g, b):
        self.current = {"color": {"r": r, "g": g, "b": b}}


class FakeClient:
    name = "tester"


IDLE_CFG = {
    "delay_s": 0.05,
    "led_animation": "breathing_cyan",
    "blink_interval_s": [0.2, 0.2],
}


@pytest.fixture
def rig():
    motion, led = FakeMotion(), FakeLed()
    engine = ActionEngine(motion, led, get_gesture=lambda n: (_ for _ in ()).throw(KeyError(n)))
    idle = IdleBehavior(engine, motion, led, IDLE_CFG, rng=random.Random(7))
    return engine, motion, led, idle


async def test_idle_arms_after_delay_and_blinks(rig):
    engine, motion, led, idle = rig
    idle.start()
    try:
        for _ in range(40):     # up to ~2 s for arm + keepalive + first blink
            await asyncio.sleep(0.05)
            if idle.armed and "expression" in motion.feed_counts:
                break
        assert idle.armed
        assert led.current == {"animation": "breathing_cyan"}
        # keepalive republishes the held pose on the idle layer
        assert motion.feed_counts.get("idle", 0) >= 1
        assert motion.layers["idle"] == {"head_pan": 0.0, "head_tilt": 0.05}
        # a blink ran through the engine (expression layer eyelid dip)
        assert motion.feed_counts.get("expression", 0) >= 1
    finally:
        idle.stop()
        await engine.stop_all()


async def test_client_work_disarms_idle(rig):
    engine, motion, led, idle = rig
    idle.start()
    try:
        for _ in range(20):
            await asyncio.sleep(0.05)
            if idle.armed:
                break
        assert idle.armed
        perf = Performance(
            id="c-1",
            client=FakeClient(),
            step=parse_step({"kind": "eyelids", "openness": 1.0, "duration_ms": 700}),
        )
        engine.submit(perf)
        for _ in range(20):
            await asyncio.sleep(0.05)
            if not idle.armed:
                break
        assert not idle.armed
        assert "idle" in motion.released
    finally:
        idle.stop()
        await engine.stop_all()


async def test_estop_pauses_idle(rig):
    engine, motion, led, idle = rig
    idle.start()
    try:
        for _ in range(20):
            await asyncio.sleep(0.05)
            if idle.armed:
                break
        assert idle.armed
        engine.set_estop(True)
        for _ in range(20):
            await asyncio.sleep(0.05)
            if not idle.armed:
                break
        assert not idle.armed
        assert led.current == {"animation": "alarm_red"}   # idle didn't fight it
        blinks_during_estop = motion.feed_counts.get("expression", 0)
        await asyncio.sleep(0.5)
        assert motion.feed_counts.get("expression", 0) == blinks_during_estop
        engine.set_estop(False)
    finally:
        idle.stop()
        await engine.stop_all()


async def test_blink_duration_is_configurable():
    """Blink speed must be tunable without a code change."""
    motion, led = FakeMotion(), FakeLed()
    engine = ActionEngine(motion, led,
                          get_gesture=lambda n: (_ for _ in ()).throw(KeyError(n)))
    submitted = []
    engine.submit = submitted.append
    idle = IdleBehavior(engine, motion, led,
                        {**IDLE_CFG, "blink_duration_ms": 110},
                        rng=random.Random(7))
    idle._submit_blink()
    assert submitted[-1].step.args["duration_ms"] == 110


async def test_blink_duration_defaults_to_150ms():
    motion, led = FakeMotion(), FakeLed()
    engine = ActionEngine(motion, led,
                          get_gesture=lambda n: (_ for _ in ()).throw(KeyError(n)))
    submitted = []
    engine.submit = submitted.append
    idle = IdleBehavior(engine, motion, led, IDLE_CFG, rng=random.Random(7))
    idle._submit_blink()
    assert submitted[-1].step.args["duration_ms"] == 150
