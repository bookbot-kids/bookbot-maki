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


async def test_book_rate_shows_its_colour_and_holds_it():
    """The rating colour is a standing indicator, not a momentary flash: it
    must be set once with no timed revert back to some resting colour."""
    from maki_puppet.bridge import RATING_COLORS

    for rating, expected in RATING_COLORS.items():
        robot = _RecordingRobot()
        await AppEventBridge(robot).dispatch("book_rate", {"rating": rating})
        assert [c[0] for c in robot.calls] == ["led_color"]
        assert robot.calls[0][1] == expected


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


# ── Head posture across a reading session ───────────────────────────────────


def _submitted(robot):
    """The step passed to the recording robot's `act` call.

    Handlers may query the robot first (e.g. led_state), so the act call is
    not necessarily calls[0].
    """
    act = next(c for c in robot.calls if c[0] == "act")
    return act[1][0]


def _flatten(step):
    """All leaf action dicts in a composed seq/par step, in order."""
    if isinstance(step, dict) and "seq" in step:
        return [a for c in step["seq"] for a in _flatten(c)]
    if isinstance(step, dict) and "par" in step:
        return [a for c in step["par"] for a in _flatten(c)]
    return [step]


async def test_tap_book_holds_head_down_as_a_posture_not_a_pose():
    """A `pose` would decay when the gesture layer's claim expires, so the head
    must be held with `posture` to stay down for the whole book."""
    from maki_puppet.bridge import HEAD_DOWN

    robot = _RecordingRobot()
    await AppEventBridge(robot).dispatch("tap_book", {"book": "Cat Hat", "level": "3"})
    actions = _flatten(_submitted(robot))
    postures = [a for a in actions if a.get("kind") == "posture"]
    assert len(postures) == 1, "tap_book must hold exactly one posture"
    assert postures[0]["joints"]["head_tilt"] == HEAD_DOWN
    assert not any(a.get("kind") == "pose" for a in actions), (
        "head-down must not be a pose — it would drift back up"
    )


async def test_close_book_clears_the_posture_before_lifting_the_head():
    from maki_puppet.bridge import HEAD_LEVEL

    robot = _RecordingRobot()
    await AppEventBridge(robot).dispatch("close_book", {"book": "Cat Hat", "level": "3"})
    actions = _flatten(_submitted(robot))
    kinds = [a.get("kind") for a in actions]
    assert "posture" in kinds and "pose" in kinds
    clear = next(a for a in actions if a.get("kind") == "posture")
    assert clear.get("clear") is True
    # The clear has to land before the head is driven back up.
    assert kinds.index("posture") < kinds.index("pose")
    lift = next(a for a in actions if a.get("kind") == "pose")
    assert lift["joints"]["head_tilt"] == HEAD_LEVEL


# ── Rainbow restores the pre-rainbow LED state ──────────────────────────────


class _RobotWithLed(_RecordingRobot):
    def __init__(self, state):
        super().__init__()
        self._state = state

    def led_state(self):
        return self._state


async def test_flourish_settles_back_to_the_colour_it_started_from():
    from maki_puppet.bridge import FLOURISH_COLORS, FLOURISH_STEP_MS

    robot = _RobotWithLed({"color": {"r": 12, "g": 34, "b": 56}})
    await AppEventBridge(robot).dispatch("tap_page", {"page": 3})
    actions = _flatten(_submitted(robot))

    # red, yellow, green — each held for FLOURISH_STEP_MS — then the original.
    shown = [(a["color"]["r"], a["color"]["g"], a["color"]["b"])
             for a in actions if a.get("kind") == "led" and "color" in a]
    assert shown[:3] == list(FLOURISH_COLORS)
    waits = [a["duration_ms"] for a in actions if a.get("kind") == "wait"]
    assert waits == [FLOURISH_STEP_MS] * len(FLOURISH_COLORS)
    assert actions[-1]["color"] == {"r": 12, "g": 34, "b": 56}


async def test_flourish_writes_no_animation_only_static_colours():
    """The rainbow rewrote all 48 pixels 50x a second, which is what surfaced
    the servo-bus noise (docs/LED_NOISE.md). The flourish must stay static."""
    robot = _RobotWithLed({"color": {"r": 12, "g": 34, "b": 56}})
    await AppEventBridge(robot).dispatch("tap_page", {"page": 3})
    actions = _flatten(_submitted(robot))
    assert not any("animation" in a for a in actions), (
        "flourish must not use an animation"
    )


async def test_rainbow_restores_an_animation_not_just_a_colour():
    robot = _RobotWithLed({"animation": "breathing_blue"})
    await AppEventBridge(robot).dispatch("tap_page", {"page": 3})
    actions = _flatten(_submitted(robot))
    assert actions[-1] == {"kind": "led", "animation": "breathing_blue"}


async def test_rainbow_falls_back_to_blue_when_led_state_is_unavailable():
    from maki_puppet.bridge import BLUE

    robot = _RobotWithLed(None)
    await AppEventBridge(robot).dispatch("tap_page", {"page": 1})
    actions = _flatten(_submitted(robot))
    assert (actions[-1]["color"]["r"], actions[-1]["color"]["g"],
            actions[-1]["color"]["b"]) == BLUE


async def test_browsing_clears_a_held_rating_colour_to_blue():
    """Navigating back out to the library/home is what releases the rating
    colour, so browsing must settle on BLUE rather than restore what was up."""
    from maki_puppet.bridge import BLUE

    for event, params in (("tap_profile", {"profile": "ada"}),
                          ("tap_category", {"category": "Minecraft"}),
                          ("tap_series", {"series": "Dogman"}),
                          ("tap_starred", {"book": "b", "level": "1"})):
        robot = _RobotWithLed({"color": {"r": 255, "g": 0, "b": 0}})  # a rating
        await AppEventBridge(robot).dispatch(event, params)
        last = _flatten(_submitted(robot))[-1]
        assert (last["color"]["r"], last["color"]["g"], last["color"]["b"]) == BLUE, (
            f"{event} left the rating colour up"
        )
