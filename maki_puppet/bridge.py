"""App-event bridge: Bookbot app events → robot actions.

This works like a Flutter platform channel, robot-side. The Flutter app calls
a typed method on ``MakiAppEvents`` (lib/maki_bridge.dart), which emits an
``event {name, params}`` frame over the WebSocket; the gateway routes any
event named below to the matching ``on_<event>`` method here — one method per
event, exactly like a ``MethodCallHandler`` switching on ``call.method``.

Each handler drives the robot through ``self.robot`` (a :class:`RobotAPI`).
The reaction vocabulary (colours, rainbow, head positions) is defined as
module constants below, so retuning the feel means editing one place:

    async def on_celebrate_example(self, word: str | None) -> None:
        self.robot.gesture("happy_wiggle", repeat=2)
        self.robot.led("attention_sweep")
        # or an existing choreography from choreographies.yaml:
        self.robot.play("celebrate")
        # or a composed step (wire Step grammar, PROTOCOL.md §10):
        self.robot.act(seq(
            par({"kind": "gesture", "name": "nod"},
                {"kind": "led", "animation": "warm_pulse"}),
            {"kind": "neutral"},
        ), priority="high", on_busy="replace")

Handlers are async and run on the gateway's event loop: do not block (no
time.sleep, no requests) — ``RobotAPI`` calls are fire-and-forget submissions
into the action engine, so returning quickly is the normal shape. A handler
that raises sends the client an ``error/internal`` ack; a handler that
returns sends ``completed``.

Bridge events take precedence over choreographies.yaml entries with the same
name and are advertised in ``welcome.events`` alongside them.
"""

from __future__ import annotations

import itertools
import logging
from typing import Any, Dict, Mapping, Optional, Sequence, Tuple

from . import protocol
from .engine import ActionEngine, Performance
from .choreography import Choreographer

log = logging.getLogger(__name__)

# ── Reaction vocabulary ─────────────────────────────────────────────────────
# Colours are wire RGB 0-255; the LED ring scales them by its own brightness
# caps, so these are hues rather than absolute intensities.
BLUE = (0, 80, 200)        # resting / not reading — matches `steady_blue`
WHITE = (255, 255, 255)    # actively listening to the child read
GREEN = (0, 200, 60)       # correct
ORANGE = (255, 120, 0)     # incorrect (deliberately not red — a miss while
                           # learning to read should not read as an error)
RED = (255, 0, 0)
YELLOW = (255, 200, 0)

# Spatial rotating rainbow (vs `rainbow_swirl`, which cycles the whole ring
# through one hue at a time and does not read as "rotating").
RAINBOW = "chase_rainbow"
# One full revolution of `chase_rainbow`, i.e. its `period` in
# config/animations.yaml. Rainbows are one-shot: they spin exactly once and
# then settle, rather than spinning forever until some later event.
RAINBOW_MS = 1500

RATING_COLORS = {1: RED, 2: ORANGE, 3: YELLOW, 4: GREEN, 5: BLUE}

FEEDBACK_HOLD_MS = 2000    # correct/incorrect colour dwell
RATING_FLASH_MS = 1000     # book-rating colour dwell

# Normalized head targets (positive tilt = DOWN — see PROTOCOL.md §3).
HEAD_DOWN = 1.0            # looking at the book
HEAD_LEVEL = 0.0           # looking at the child


def seq(*steps: Any) -> dict:
    """Compose steps to run one after another (wire Step grammar)."""
    return {"seq": list(steps)}


def par(*steps: Any) -> dict:
    """Compose steps to run at the same time (wire Step grammar)."""
    return {"par": list(steps)}


class RobotAPI:
    """What a bridge handler is allowed to do to the robot.

    Every method validates against the live catalogs and submits an internal
    performance to the action engine (client=None: no acks, bypasses client
    locks — same as the idle behavior). Calls return immediately; the robot
    acts asynchronously. Raises ``protocol.ProtocolError`` on invalid input
    (unknown animation/gesture/joint, out-of-range values).
    """

    def __init__(self, engine: ActionEngine, choreographer: Choreographer,
                 animations: Sequence[str]) -> None:
        self._engine = engine
        self._choreo = choreographer
        self._animations = list(animations)
        self._ids = itertools.count(1)

    # ── Core ───────────────────────────────────────────────────────────

    def act(self, step: Any, *, priority: int | str = "normal",
            on_busy: str = "replace", tag: Optional[str] = None) -> str:
        """Submit one Step (plain dict in the wire grammar, or a parsed Step).

        Returns the internal performance id. ``on_busy`` defaults to
        "replace" so a fresh app event takes the robot over immediately.
        """
        if not isinstance(step, (protocol.Action, protocol.Seq, protocol.Par)):
            step = protocol.parse_step(
                step, animations=self._animations,
                gestures=self._choreo.gesture_names,
            )
        perf = Performance(
            id=f"bridge-{next(self._ids)}",
            client=None,
            step=step,
            priority=protocol.parse_priority(priority, default=50),
            on_busy=on_busy,
            tag=tag,
        )
        self._engine.submit(perf)
        return perf.id

    def play(self, event_name: str, **params: Any) -> str:
        """Run an existing choreography from choreographies.yaml by name."""
        resolved = self._choreo.resolve(event_name, params)
        if resolved is None:  # listed under ignored_events → nothing to do
            return ""
        perf = Performance(
            id=f"bridge-{next(self._ids)}",
            client=None,
            step=resolved.step,
            priority=resolved.priority,
            on_busy=resolved.on_busy,
            choreography=resolved.name,
        )
        self._engine.submit(perf)
        return perf.id

    # ── Convenience wrappers (one wire action each) ────────────────────

    def blink(self, duration_ms: int = 150) -> str:
        return self.act({"kind": "blink", "duration_ms": duration_ms})

    def look(self, pan: float, tilt: float, duration_ms: int = 600) -> str:
        return self.act({"kind": "look", "pan": pan, "tilt": tilt,
                         "duration_ms": duration_ms})

    def gesture(self, name: str, repeat: int = 1, intensity: float = 1.0) -> str:
        return self.act({"kind": "gesture", "name": name, "repeat": repeat,
                         "intensity": intensity})

    def led(self, animation: str) -> str:
        return self.act({"kind": "led", "animation": animation})

    def led_color(self, r: int, g: int, b: int) -> str:
        return self.act({"kind": "led", "color": {"r": r, "g": g, "b": b}})

    def pose(self, joints: Mapping[str, float], duration_ms: int = 800) -> str:
        return self.act({"kind": "pose", "joints": dict(joints),
                         "duration_ms": duration_ms})

    def neutral(self, duration_ms: int = 700) -> str:
        return self.act({"kind": "neutral", "duration_ms": duration_ms})

    def cancel_all(self) -> int:
        """Cancel every queued and running performance (incl. other clients')."""
        return self._engine.cancel("all")


class AppEventBridge:
    """One ``on_<event>`` handler per Bookbot app event. Fill in the blanks.

    ``EVENT_PARAMS`` is the wire contract shared with lib/maki_bridge.dart:
    for each event, the ordered param keys the Flutter side sends. Params
    arrive as keyword arguments; a param the app omitted arrives as None.
    """

    EVENT_PARAMS: Dict[str, Tuple[str, ...]] = {
        "tap_profile": ("profile",),
        "tap_category": ("category",),
        "tap_series": ("series",),
        "tap_book": ("book", "level"),
        "tap_starred": ("book", "level"),
        "close_book": ("book", "level"),
        "practice_correct": ("word",),
        "practice_incorrect": ("word",),
        "focus_word_correct": ("word",),
        "focus_word_incorrect": ("word",),
        "read_to_me": (),
        "listen": (),
        "mute": (),
        "tap_page": ("page",),
        "book_rate": ("rating",),
    }

    def __init__(self, robot: RobotAPI) -> None:
        self.robot = robot

    # ── Dispatch plumbing (no need to touch) ───────────────────────────

    @property
    def event_names(self) -> list:
        return list(self.EVENT_PARAMS)

    def handles(self, name: str) -> bool:
        return name in self.EVENT_PARAMS

    async def dispatch(self, name: str, params: Mapping[str, Any]) -> None:
        """Route one app event to its ``on_<name>`` handler."""
        keys = self.EVENT_PARAMS[name]
        kwargs = {k: params.get(k) for k in keys}
        handler = getattr(self, f"on_{name}")
        log.info("app event %s %s", name, kwargs or "")
        await handler(**kwargs)

    # ── Reaction helpers ───────────────────────────────────────────────

    def _color(self, rgb: Tuple[int, int, int]) -> dict:
        """A `led` action step for one solid colour."""
        r, g, b = rgb
        return {"kind": "led", "color": {"r": r, "g": g, "b": b}}

    def _hold_then(
        self, rgb: Tuple[int, int, int], hold_ms: int,
        then: Tuple[int, int, int],
    ) -> None:
        """Show *rgb* for *hold_ms*, then settle on *then*.

        Submitted as ONE act so the revert is part of the same performance:
        if a newer event replaces this one mid-hold, the stale revert is
        cancelled with it instead of stomping the new colour a second later.
        """
        self.robot.act(seq(
            self._color(rgb),
            {"kind": "wait", "duration_ms": hold_ms},
            self._color(then),
        ))

    def _rainbow_then(self, rgb: Tuple[int, int, int]) -> None:
        """Spin the rainbow exactly once, then settle on *rgb*."""
        self.robot.act(seq(
            {"kind": "led", "animation": RAINBOW},
            {"kind": "wait", "duration_ms": RAINBOW_MS},
            self._color(rgb),
        ))

    def _browsing(self) -> None:
        """Library browsing: one rainbow revolution, back to resting blue."""
        self._rainbow_then(BLUE)

    # ── Handlers: one per app event ────────────────────────────────────

    async def on_tap_profile(self, profile: Optional[str]) -> None:
        """A profile was tapped on the profile-select screen."""
        self._browsing()

    async def on_tap_category(self, category: Optional[str]) -> None:
        """A category tile was tapped in the library."""
        self._browsing()

    async def on_tap_series(self, series: Optional[str]) -> None:
        """A series was tapped in the library."""
        self._browsing()

    async def on_tap_book(self, book: Optional[str],
                          level: Optional[str]) -> None:
        """A book was opened: rainbow, and look down at the book."""
        self.robot.act(par(
            seq({"kind": "led", "animation": RAINBOW},
                {"kind": "wait", "duration_ms": RAINBOW_MS},
                self._color(BLUE)),
            {"kind": "pose",
             "joints": {"head_pan": 0.0, "head_tilt": HEAD_DOWN},
             "duration_ms": 900},
        ))

    async def on_tap_starred(self, book: Optional[str],
                             level: Optional[str]) -> None:
        """A book was starred/favourited."""
        self._browsing()

    async def on_close_book(self, book: Optional[str],
                            level: Optional[str]) -> None:
        """Book closed: back to blue, head level again."""
        self.robot.act(par(
            self._color(BLUE),
            {"kind": "pose",
             "joints": {"head_pan": 0.0, "head_tilt": HEAD_LEVEL},
             "duration_ms": 900},
        ))

    async def on_practice_correct(self, word: Optional[str]) -> None:
        """A practice word was read correctly."""
        self._hold_then(GREEN, FEEDBACK_HOLD_MS, WHITE)

    async def on_practice_incorrect(self, word: Optional[str]) -> None:
        """A practice word was read incorrectly."""
        self._hold_then(ORANGE, FEEDBACK_HOLD_MS, WHITE)

    async def on_focus_word_correct(self, word: Optional[str]) -> None:
        """The focus word was read correctly."""
        self._hold_then(GREEN, FEEDBACK_HOLD_MS, WHITE)

    async def on_focus_word_incorrect(self, word: Optional[str]) -> None:
        """The focus word was read incorrectly."""
        self._hold_then(ORANGE, FEEDBACK_HOLD_MS, WHITE)

    async def on_read_to_me(self) -> None:
        """The child chose 'read to me' (narration) mode."""
        self.robot.led_color(*BLUE)

    async def on_listen(self) -> None:
        """The app started listening to the child read."""
        self.robot.led_color(*WHITE)

    async def on_mute(self) -> None:
        """Sound was muted."""
        self.robot.led_color(*BLUE)

    async def on_tap_page(self, page: Optional[int]) -> None:
        """A page was tapped/turned: one rainbow revolution, back to listening."""
        self._rainbow_then(WHITE)

    async def on_book_rate(self, rating: Optional[int]) -> None:
        """The child rated the book 1-5: flash that rating's colour, then green."""
        if rating is None:
            return
        rgb = RATING_COLORS.get(int(rating))
        if rgb is None:
            log.warning("book_rate: rating %r outside 1-5, ignoring", rating)
            return
        self._hold_then(rgb, RATING_FLASH_MS, GREEN)
