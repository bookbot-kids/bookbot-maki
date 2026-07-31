"""App-event bridge tests: dispatch plumbing, RobotAPI submissions, and the
end-to-end path (Flutter-style `event` frame → bridge handler → ack)."""

from __future__ import annotations

import asyncio
import json
from pathlib import Path

import pytest
import websockets

from maki_puppet import protocol
from maki_puppet.app import PuppetApp
from maki_puppet.bridge import AppEventBridge, RobotAPI, par, seq

from .test_server import CONFIG, OVERRIDES, WsClient

pytestmark = pytest.mark.asyncio


# ── Dispatch plumbing (no server) ───────────────────────────────────────────


class RecordingBridge(AppEventBridge):
    def __init__(self):
        super().__init__(robot=None)  # handlers below never touch self.robot
        self.calls = []

    async def on_tap_book(self, book, level):
        self.calls.append(("tap_book", book, level))

    async def on_read_to_me(self):
        self.calls.append(("read_to_me",))

    async def on_book_rate(self, rating):
        self.calls.append(("book_rate", rating))


async def test_dispatch_maps_params_to_kwargs():
    b = RecordingBridge()
    await b.dispatch("tap_book", {"book": "Dog", "level": "7"})
    assert b.calls == [("tap_book", "Dog", "7")]


async def test_dispatch_missing_params_arrive_as_none_extras_ignored():
    b = RecordingBridge()
    await b.dispatch("tap_book", {"book": "Dog", "unexpected": True})
    await b.dispatch("book_rate", {})
    assert b.calls == [("tap_book", "Dog", None), ("book_rate", None)]


async def test_dispatch_no_param_event():
    b = RecordingBridge()
    await b.dispatch("read_to_me", {})
    assert b.calls == [("read_to_me",)]


async def test_every_event_has_a_handler_and_vice_versa():
    b = AppEventBridge(robot=None)
    for name in b.EVENT_PARAMS:
        assert callable(getattr(b, f"on_{name}", None)), f"missing on_{name}"
    handlers = {n[3:] for n in dir(b) if n.startswith("on_")}
    assert handlers == set(b.EVENT_PARAMS), "handler without EVENT_PARAMS entry"


class _RecordingRobot:
    """Stands in for RobotAPI, recording what each handler asked for."""

    def __init__(self):
        self.calls: list = []

    def __getattr__(self, name):
        def record(*args, **kwargs):
            self.calls.append((name, args, kwargs))
            return "perf-id"
        return record


# Realistic params per event (book_rate needs a real int, not a placeholder).
SAMPLE_PARAMS = {
    "tap_profile": {"profile": "ada"},
    "tap_category": {"category": "Minecraft"},
    "tap_series": {"series": "Dogman"},
    "tap_book": {"book": "Cat Hat", "level": "3"},
    "tap_starred": {"book": "Cat Hat", "level": "3"},
    "close_book": {"book": "Cat Hat", "level": "3"},
    "practice_correct": {"word": "dog"},
    "practice_incorrect": {"word": "dog"},
    "focus_word_correct": {"word": "dog"},
    "focus_word_incorrect": {"word": "dog"},
    "read_to_me": {},
    "listen": {},
    "mute": {},
    "tap_page": {"page": 3},
    "book_rate": {"rating": 5},
}


async def test_every_event_produces_a_robot_reaction():
    """No handler may be a silent no-op — each event must drive the robot."""
    assert set(SAMPLE_PARAMS) == set(AppEventBridge.EVENT_PARAMS)
    for name, params in SAMPLE_PARAMS.items():
        robot = _RecordingRobot()
        b = AppEventBridge(robot)
        await b.dispatch(name, params)
        assert robot.calls, f"on_{name} did nothing"


async def test_book_rate_maps_each_rating_to_its_colour():
    from maki_puppet.bridge import GREEN, RATING_COLORS

    for rating, expected in RATING_COLORS.items():
        robot = _RecordingRobot()
        await AppEventBridge(robot).dispatch("book_rate", {"rating": rating})
        step = robot.calls[0][1][0]           # the composed seq step
        first_led = step["seq"][0]["color"]
        last_led = step["seq"][-1]["color"]
        assert (first_led["r"], first_led["g"], first_led["b"]) == expected
        # Every rating settles on green afterwards.
        assert (last_led["r"], last_led["g"], last_led["b"]) == GREEN


async def test_book_rate_ignores_out_of_range_and_missing():
    for bad in (None, 0, 6):
        robot = _RecordingRobot()
        await AppEventBridge(robot).dispatch("book_rate", {"rating": bad})
        assert robot.calls == [], f"rating {bad!r} should be ignored"


# ── RobotAPI against the live sim stack ─────────────────────────────────────


@pytest.fixture
async def app():
    app = PuppetApp(CONFIG, sim=True, config_overrides=OVERRIDES)
    await app.start()
    yield app
    await app.stop()


async def connect(app) -> WsClient:
    ws = await websockets.connect(f"ws://127.0.0.1:{app.server.port}/ws")
    return WsClient(ws)


async def test_robot_api_validates_against_catalogs(app):
    robot = app.bridge.robot
    with pytest.raises(protocol.ProtocolError):
        robot.led("no_such_animation")
    with pytest.raises(protocol.ProtocolError):
        robot.gesture("no_such_gesture")
    with pytest.raises(protocol.ProtocolError):
        robot.act({"kind": "look", "pan": 5.0, "tilt": 0.0})  # out of range


async def test_robot_api_submits_and_robot_acts(app):
    robot = app.bridge.robot
    robot.led("warm_pulse")
    robot.act(seq({"kind": "blink"},
                  par({"kind": "gesture", "name": "nod"},
                      {"kind": "wait", "duration_ms": 50})))
    await asyncio.sleep(0.3)
    assert app.led.current == {"animation": "warm_pulse"}


async def test_robot_api_play_runs_choreography(app):
    robot = app.bridge.robot
    assert robot.play("celebrate").startswith("bridge-")
    await asyncio.sleep(0.2)
    assert app.led.current == {"animation": "attention_sweep"}


# ── End-to-end: event frame → bridge → ack ──────────────────────────────────


async def test_app_event_acks_completed_bridge(app):
    c = await connect(app)
    welcome = await c.hello()
    assert "tap_book" in welcome["payload"]["events"]  # advertised in catalog
    mid = await c.send("event", {"name": "tap_book",
                                 "params": {"book": "Dog", "level": "7"}})
    acks = await c.acks_until_terminal(mid)
    assert acks[-1]["status"] == "completed"
    assert acks[-1]["detail"] == "bridge"


async def test_app_event_handler_runs_robot_action(app):
    class LedOnListen(AppEventBridge):
        async def on_listen(self):
            self.robot.led("concurrent_listen")

    app.server._bridge = app.bridge = LedOnListen(app.bridge.robot)
    c = await connect(app)
    await c.hello()
    mid = await c.send("event", {"name": "listen", "params": {}})
    acks = await c.acks_until_terminal(mid)
    assert acks[-1]["status"] == "completed"
    await asyncio.sleep(0.2)
    assert app.led.current == {"animation": "concurrent_listen"}


async def test_app_event_handler_exception_acks_error(app):
    class Broken(AppEventBridge):
        async def on_mute(self):
            raise RuntimeError("boom")

    app.server._bridge = app.bridge = Broken(app.bridge.robot)
    c = await connect(app)
    await c.hello()
    mid = await c.send("event", {"name": "mute", "params": {}})
    acks = await c.acks_until_terminal(mid)
    assert acks[-1]["status"] == "error"
    assert acks[-1]["code"] == "internal"


async def test_app_event_rebroadcast_to_subscribers(app):
    sender = await connect(app)
    await sender.hello("sender")
    watcher = await connect(app)
    await watcher.hello("watcher", subscribe=("events",))
    await sender.send("event", {"name": "tap_page", "params": {"page": 3}})
    msg = await watcher.recv(type_="event")
    assert msg["payload"]["name"] == "tap_page"
    assert msg["payload"]["params"] == {"page": 3}
    assert msg["payload"]["origin"] == "sender"


async def test_unknown_event_still_rejected(app):
    c = await connect(app)
    await c.hello()
    mid = await c.send("event", {"name": "tap_nothing", "params": {}})
    acks = await c.acks_until_terminal(mid)
    assert acks[-1]["status"] == "error"
    assert acks[-1]["code"] == "unknown_event"
