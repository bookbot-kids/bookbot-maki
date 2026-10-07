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
    "show_library": {},
    "practice_start": {"book": "Cat Hat"},
    "practice_correct": {"word": "dog"},
    "practice_incorrect": {"word": "dog"},
    "focus_word_correct": {"word": "dog"},
    "focus_word_incorrect": {"word": "dog"},
    "read_to_me": {},
    "listen": {},
    "mute": {},
    "tap_page": {"page": 3},
    "page_start": {"page": 3},
    "reading_word_incorrect": {"word": "dog"},
    "page_end": {"page": 3, "errors": 1},
    "book_end": {"book": "Cat Hat", "level": "3"},
    "book_rate": {"rating": 5},
}

PHASES = ("library", "book", "practice", "reading", "page_end", "book_end")

# Events that never move the session to a new phase, so leave the ring alone.
RING_UNTOUCHED = {"tap_starred", "focus_word_correct", "mute"}


def _acts(robot):
    """The steps passed to the recording robot's `act` calls, in order."""
    return [c[1][0] for c in robot.calls if c[0] == "act"]


def _flatten(step):
    """All leaf action dicts in a composed seq/par step, in order."""
    if isinstance(step, dict) and "seq" in step:
        return [a for c in step["seq"] for a in _flatten(c)]
    if isinstance(step, dict) and "par" in step:
        return [a for c in step["par"] for a in _flatten(c)]
    return [step]


def _actions(robot):
    return [a for step in _acts(robot) for a in _flatten(step)]


def _shown(robot):
    """Every LED state the handlers asked for: an RGB tuple or an animation."""
    out = []
    for a in _actions(robot):
        if a.get("kind") != "led":
            continue
        if "color" in a:
            c = a["color"]
            out.append((c["r"], c["g"], c["b"]))
        else:
            out.append(a["animation"])
    return out


def _cancelled_glance(robot):
    from maki_puppet.bridge import GLANCE_TAG

    return ("cancel_tag", (GLANCE_TAG,), {}) in robot.calls


async def _bridge_in(phase, page_errors=0):
    robot = _RecordingRobot()
    b = AppEventBridge(robot)
    b.phase = phase
    b.page_errors = page_errors
    return b, robot


async def test_every_event_is_handled_from_every_phase():
    assert set(SAMPLE_PARAMS) == set(AppEventBridge.EVENT_PARAMS)
    for phase in PHASES:
        for name, params in SAMPLE_PARAMS.items():
            b, _ = await _bridge_in(phase)
            await b.dispatch(name, params)


async def test_every_phase_changing_event_drives_the_robot():
    """Only the deliberate no-ops may do nothing, and only those."""
    for name, params in SAMPLE_PARAMS.items():
        b, robot = await _bridge_in("library")
        await b.dispatch(name, params)
        if name in RING_UNTOUCHED:
            assert robot.calls == [], f"on_{name} should leave the robot alone"
        else:
            assert robot.calls, f"on_{name} did nothing"


async def test_palette_is_neutral_blue_purple_and_book_end_green_only():
    """No red, orange, yellow or white, from any event in any phase."""
    from maki_puppet.bridge import BLUE, BOOK_END_ANIMATION, NEUTRAL, PURPLE

    allowed = {NEUTRAL, BLUE, PURPLE, BOOK_END_ANIMATION}
    for phase in PHASES:
        for name, params in SAMPLE_PARAMS.items():
            b, robot = await _bridge_in(phase)
            await b.dispatch(name, params)
            stray = set(_shown(robot)) - allowed
            assert not stray, f"{name} in {phase} showed {stray}"


async def test_green_only_at_book_end():
    from maki_puppet.bridge import BOOK_END_ANIMATION

    for phase in PHASES:
        for name, params in SAMPLE_PARAMS.items():
            b, robot = await _bridge_in(phase)
            await b.dispatch(name, params)
            if BOOK_END_ANIMATION in _shown(robot):
                assert name in ("book_end", "book_rate"), (
                    f"{name} in {phase} showed the book-end green"
                )


async def test_no_multi_colour_flourish_anywhere():
    """The red→yellow→green tap flourish was the 'random red flash'. Every
    act may show at most one colour, except a flagged word's purple→blue."""
    for phase in PHASES:
        for name, params in SAMPLE_PARAMS.items():
            b, robot = await _bridge_in(phase)
            await b.dispatch(name, params)
            for step in _acts(robot):
                leds = [a for a in _flatten(step) if a.get("kind") == "led"]
                if name in ("reading_word_incorrect", "focus_word_incorrect"):
                    assert len(leds) <= 2
                else:
                    assert len(leds) <= 1, f"{name} in {phase} flashed {leds}"


async def test_browsing_and_library_are_neutral_not_blue():
    from maki_puppet.bridge import NEUTRAL

    for event in ("tap_profile", "tap_category", "tap_series", "tap_book",
                  "show_library", "close_book"):
        b, robot = await _bridge_in("book_end")  # e.g. back from a finished book
        await b.dispatch(event, SAMPLE_PARAMS[event])
        assert _shown(robot) == [NEUTRAL], f"{event} -> {_shown(robot)}"


async def test_returning_to_the_library_clears_the_book_end_green():
    from maki_puppet.bridge import BOOK_END_ANIMATION, NEUTRAL

    robot = _RecordingRobot()
    b = AppEventBridge(robot)
    for name in ("tap_book", "page_start", "page_end", "book_end", "book_rate",
                 "show_library"):
        await b.dispatch(name, SAMPLE_PARAMS[name])
    shown = _shown(robot)
    assert BOOK_END_ANIMATION in shown
    assert shown[-1] == NEUTRAL
    assert b.phase == "library"


async def test_book_end_green_holds_through_rating_and_starring():
    for name in ("book_rate", "tap_starred", "listen", "mute",
                 "focus_word_correct"):
        b, robot = await _bridge_in("book_end")
        await b.dispatch(name, SAMPLE_PARAMS[name])
        assert _shown(robot) == [], f"{name} changed the book-end green"


async def test_book_rate_shows_book_end_green_for_any_rating():
    """Ratings no longer have colours of their own (they were red/orange/
    yellow); rating is part of the book end, whose colour is green."""
    from maki_puppet.bridge import BOOK_END_ANIMATION

    for rating in (1, 2, 3, 4, 5, None):
        b, robot = await _bridge_in("reading")
        await b.dispatch("book_rate", {"rating": rating})
        assert _shown(robot) == [BOOK_END_ANIMATION]


async def test_practice_words_are_continuous_purple():
    """Purple from practice_start, and nothing during the phase changes it —
    not a correct word, not the app listening, not a mute."""
    from maki_puppet.bridge import PURPLE

    robot = _RecordingRobot()
    b = AppEventBridge(robot)
    await b.dispatch("tap_book", SAMPLE_PARAMS["tap_book"])
    robot.calls.clear()
    await b.dispatch("practice_start", SAMPLE_PARAMS["practice_start"])
    assert _shown(robot) == [PURPLE]
    robot.calls.clear()
    for name in ("listen", "practice_correct", "practice_incorrect",
                 "focus_word_correct", "focus_word_incorrect", "mute",
                 "practice_correct"):
        await b.dispatch(name, SAMPLE_PARAMS[name])
    assert _shown(robot) == [], "the practice phase changed colour"
    assert b.phase == "practice"


async def test_practice_turns_purple_even_without_practice_start():
    from maki_puppet.bridge import PURPLE

    b, robot = await _bridge_in("book")
    await b.dispatch("practice_correct", {"word": "dog"})
    assert _shown(robot) == [PURPLE]


async def test_reading_is_blue():
    from maki_puppet.bridge import BLUE

    for name in ("listen", "read_to_me", "page_start", "tap_page"):
        b, robot = await _bridge_in("book")
        await b.dispatch(name, SAMPLE_PARAMS[name])
        assert _shown(robot) == [BLUE], f"{name} -> {_shown(robot)}"


async def test_reading_leaves_practice_on_a_page_but_not_on_listen():
    from maki_puppet.bridge import BLUE

    b, robot = await _bridge_in("practice")
    await b.dispatch("listen", {})
    assert _shown(robot) == [] and b.phase == "practice"
    await b.dispatch("page_start", {"page": 1})
    assert _shown(robot) == [BLUE] and b.phase == "reading"


async def test_flagged_word_turns_purple_at_once_then_back_to_blue():
    """Purple must be the first thing the act does (no delay), and the
    revert must live in the same act so a newer event can cancel it."""
    from maki_puppet.bridge import BLUE, FLAG_HOLD_MS, PURPLE

    for name in ("reading_word_incorrect", "focus_word_incorrect"):
        b, robot = await _bridge_in("reading")
        await b.dispatch(name, {"word": "dog"})
        acts = _acts(robot)
        assert len(acts) == 1
        actions = _flatten(acts[0])
        assert actions[0] == {"kind": "led",
                              "color": dict(zip("rgb", PURPLE))}
        assert actions[1] == {"kind": "wait", "duration_ms": FLAG_HOLD_MS}
        assert actions[2] == {"kind": "led", "color": dict(zip("rgb", BLUE))}


async def test_correct_word_never_changes_colour():
    for phase in ("reading", "practice", "page_end"):
        b, robot = await _bridge_in(phase)
        await b.dispatch("focus_word_correct", {"word": "dog"})
        assert robot.calls == []


async def test_listen_and_tap_page_do_not_cut_a_flagged_word_short():
    b, robot = await _bridge_in("reading")
    await b.dispatch("reading_word_incorrect", {"word": "dog"})
    robot.calls.clear()
    await b.dispatch("listen", {})
    await b.dispatch("mute", {})
    assert robot.calls == []


async def test_practice_and_mistakes_look_at_the_child_never_down():
    cases = [("practice_start", "book", 0), ("practice_correct", "practice", 0),
             ("practice_incorrect", "practice", 0),
             ("focus_word_incorrect", "reading", 0),
             ("reading_word_incorrect", "reading", 0),
             ("page_end", "reading", 2)]
    for name, phase, errors in cases:
        b, robot = await _bridge_in(phase, errors)
        params = dict(SAMPLE_PARAMS[name])
        if name == "page_end":
            params["errors"] = errors
        await b.dispatch(name, params)
        assert not any(a.get("kind") in ("look", "posture", "pose")
                       for a in _actions(robot)), f"{name} moved the head"
        assert _cancelled_glance(robot), f"{name} left a glance running"


async def test_glance_down_at_page_start_head_and_eyes():
    """A glance, not a posture: motion-only, tagged so mistakes can cancel
    it, and with tilt at the bottom of the range (look drives the eyes too)."""
    from maki_puppet.bridge import GLANCE_TAG, HEAD_DOWN

    b, robot = await _bridge_in("reading")
    await b.dispatch("page_start", {"page": 2})
    glances = [c for c in robot.calls
               if c[0] == "act" and c[2].get("tag") == GLANCE_TAG]
    assert len(glances) == 1
    actions = _flatten(glances[0][1][0])
    assert actions and all(a["kind"] == "look" for a in actions)
    assert all(a["tilt"] == HEAD_DOWN and a.get("eyes_only") is not True
               for a in actions)


async def test_glances_only_at_page_start_and_clean_page_end():
    from maki_puppet.bridge import GLANCE_TAG

    for phase in PHASES:
        for name, params in SAMPLE_PARAMS.items():
            b, robot = await _bridge_in(phase)
            await b.dispatch(name, params)
            glanced = any(c[0] == "act" and c[2].get("tag") == GLANCE_TAG
                          for c in robot.calls)
            looked = any(a.get("kind") == "look" for a in _actions(robot))
            assert glanced == looked, "a look outside the tagged glance"
            if glanced:
                assert name == "page_start" or (
                    name == "page_end" and not params.get("errors")
                ), f"{name} in {phase} glanced down"


async def test_nothing_holds_the_head_down_any_more():
    for phase in PHASES:
        for name, params in SAMPLE_PARAMS.items():
            b, robot = await _bridge_in(phase)
            await b.dispatch(name, params)
            assert not any(a.get("kind") == "posture" for a in _actions(robot))


async def test_page_end_after_errors_is_purple_all_correct_stays_blue():
    from maki_puppet.bridge import BLUE, GLANCE_TAG, PURPLE

    b, robot = await _bridge_in("reading")
    await b.dispatch("page_end", {"page": 1, "errors": 3})
    assert _shown(robot) == [PURPLE]

    b, robot = await _bridge_in("reading")
    await b.dispatch("page_end", {"page": 1, "errors": 0})
    assert _shown(robot) == [BLUE]
    assert any(c[2].get("tag") == GLANCE_TAG for c in robot.calls if c[0] == "act")


async def test_page_end_counts_flagged_words_when_the_app_sends_no_count():
    from maki_puppet.bridge import BLUE, PURPLE

    b, robot = await _bridge_in("book")
    await b.dispatch("page_start", {"page": 1})
    await b.dispatch("reading_word_incorrect", {"word": "dog"})
    robot.calls.clear()
    await b.dispatch("page_end", {"page": 1})
    assert _shown(robot) == [PURPLE]

    # A new page starts clean.
    await b.dispatch("page_start", {"page": 2})
    robot.calls.clear()
    await b.dispatch("page_end", {"page": 2, "errors": None})
    assert _shown(robot) == [BLUE]


async def test_page_end_with_a_garbage_count_falls_back_to_its_own():
    from maki_puppet.bridge import PURPLE

    b, robot = await _bridge_in("reading", page_errors=1)
    await b.dispatch("page_end", {"page": 1, "errors": "lots"})
    assert _shown(robot) == [PURPLE]


async def test_page_end_purple_holds_until_the_page_turns():
    from maki_puppet.bridge import BLUE

    b, robot = await _bridge_in("page_end", page_errors=2)
    for name in ("listen", "mute", "focus_word_correct"):
        await b.dispatch(name, SAMPLE_PARAMS[name])
    assert robot.calls == []
    await b.dispatch("tap_page", {"page": 4})
    assert _shown(robot) == [BLUE]


async def test_palette_constants_match_animations_yaml():
    """The bridge's colours and the named animations must agree, or the
    library light (an animation at boot, a colour after) would differ."""
    import yaml

    from maki_puppet.bridge import BLUE, BOOK_END_ANIMATION, NEUTRAL, PURPLE

    anims = yaml.safe_load(
        (Path(__file__).parent.parent / "config" / "animations.yaml").read_text()
    )["animations"]
    assert tuple(anims["steady_white"]["color"]) == NEUTRAL
    assert tuple(anims["steady_blue"]["color"]) == BLUE
    assert tuple(anims["steady_purple"]["color"]) == PURPLE
    green = anims[BOOK_END_ANIMATION]
    assert green["color"][1] > max(green["color"][0], green["color"][2])
    assert green.get("brightness_scale", 1.0) > 1.0

    puppet = yaml.safe_load(CONFIG.read_text())
    assert puppet["led"]["default_animation"] == "steady_white"
    assert puppet["idle"]["led_animation"] == "steady_white"


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
    assert app.led.current == {"animation": "steady_blue"}


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


# ── A whole session against the live sim stack ──────────────────────────────


async def test_full_session_drives_the_ring_through_the_colour_scheme(app):
    """Event by event, what the (sim) ring actually shows."""
    from maki_puppet.bridge import BLUE, NEUTRAL, PURPLE

    def colour(rgb):
        return {"color": dict(zip("rgb", rgb))}

    steps = [
        ("tap_book", colour(NEUTRAL)),
        ("practice_start", colour(PURPLE)),
        ("practice_correct", colour(PURPLE)),
        ("listen", colour(PURPLE)),
        ("page_start", colour(BLUE)),
        ("reading_word_incorrect", colour(PURPLE)),
        ("page_end", colour(PURPLE)),
        ("tap_page", colour(BLUE)),
        ("book_end", {"animation": "book_end_green"}),
        ("book_rate", {"animation": "book_end_green"}),
        ("show_library", colour(NEUTRAL)),
    ]
    for name, expected in steps:
        await app.bridge.dispatch(name, SAMPLE_PARAMS[name])
        await asyncio.sleep(0.05)
        assert app.led.current == expected, f"after {name}"


async def test_flagged_word_reverts_to_blue_on_the_live_stack(app, monkeypatch):
    from maki_puppet import bridge as bridge_mod
    from maki_puppet.bridge import BLUE, PURPLE

    monkeypatch.setattr(bridge_mod, "FLAG_HOLD_MS", 100)
    app.bridge.phase = "reading"
    await app.bridge.dispatch("reading_word_incorrect", {"word": "dog"})
    await asyncio.sleep(0.03)
    assert app.led.current == {"color": dict(zip("rgb", PURPLE))}
    await asyncio.sleep(0.25)
    assert app.led.current == {"color": dict(zip("rgb", BLUE))}


def _live_glances(app):
    from maki_puppet.bridge import GLANCE_TAG

    return [p for p in app.engine._live_performances() if p.tag == GLANCE_TAG]


async def test_mistake_cancels_a_running_glance_on_the_live_stack(app):
    app.bridge.phase = "reading"
    await app.bridge.dispatch("page_start", {"page": 1})
    await asyncio.sleep(0.1)
    assert _live_glances(app), "page_start should be glancing down"
    await app.bridge.dispatch("reading_word_incorrect", {"word": "dog"})
    await asyncio.sleep(0.1)
    assert not _live_glances(app), "the glance outlived the mistake"


async def test_glance_ends_on_its_own(app, monkeypatch):
    from maki_puppet import bridge as bridge_mod

    monkeypatch.setattr(bridge_mod, "GLANCE_MS", (100, 100))
    await app.bridge.dispatch("page_start", {"page": 1})
    await asyncio.sleep(0.05)
    assert _live_glances(app)
    await asyncio.sleep(0.4)
    assert not _live_glances(app)


async def test_page_turn_before_page_end_keeps_the_error_count():
    """The app may send the turn before page_end; the count must survive."""
    from maki_puppet.bridge import PURPLE

    b, robot = await _bridge_in("reading")
    await b.dispatch("reading_word_incorrect", {"word": "dog"})
    await b.dispatch("tap_page", {"page": 2})
    robot.calls.clear()
    await b.dispatch("page_end", {"page": 1})
    assert _shown(robot) == [PURPLE]


async def test_look_keeps_the_eyes_when_the_head_follows(app):
    """`gesture` replaces its targets on every feed, so the head's feed must
    carry the eyes too — or a glance drops the eyes 80 ms in."""
    feeds = []
    real = app.motion.set_layer

    def record(layer, targets):
        if layer == "gesture":
            feeds.append(dict(targets))
        return real(layer, targets)

    app.motion.set_layer = record
    try:
        app.bridge.robot.look(0.0, 1.0, duration_ms=200)
        await asyncio.sleep(0.4)
    finally:
        app.motion.set_layer = real
    assert len(feeds) >= 2
    assert {"eyes_pan", "eyes_tilt", "head_pan", "head_tilt"} <= set(feeds[-1])


async def test_idle_rearming_leaves_a_held_session_colour_alone(app):
    """A client act disarms idle; when it re-arms it must not reset a held
    colour (e.g. book-end green) to the resting white."""
    await app.bridge.dispatch("book_end", SAMPLE_PARAMS["book_end"])
    await asyncio.sleep(0.05)
    app.idle._arm(0.0)
    assert app.led.current == {"animation": "book_end_green"}

    await app.bridge.dispatch("show_library", {})
    await asyncio.sleep(0.05)
    app.led.set_animation("steady_blue")      # pretend something else drew
    app.idle._arm(0.0)
    assert app.led.current == {"animation": "steady_white"}
