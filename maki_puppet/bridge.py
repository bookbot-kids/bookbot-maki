"""App-event bridge: Bookbot app events → robot actions.

This works like a Flutter platform channel, robot-side. The Flutter app calls
a typed method on ``MakiAppEvents`` (lib/maki_bridge.dart), which emits an
``event {name, params}`` frame over the WebSocket; the gateway routes any
event named below to the matching ``on_<event>`` method here — one method per
event, exactly like a ``MethodCallHandler`` switching on ``call.method``.

Each handler drives the robot through ``self.robot`` (a :class:`RobotAPI`).
The reaction vocabulary (colours, flourish, head positions) is defined as
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

# Tap flourish: three static colours in sequence, then back to whatever the
# ring was showing. Replaces the spinning rainbow — see docs/LED_NOISE.md. The
# rainbow rewrote all 48 pixels 50x a second, which is what made servo-bus
# noise visible as white flashes; this is four SPI writes in total.
FLOURISH_COLORS = (RED, YELLOW, GREEN)
FLOURISH_STEP_MS = 200     # dwell per colour

RATING_COLORS = {1: RED, 2: ORANGE, 3: YELLOW, 4: GREEN, 5: BLUE}

FEEDBACK_HOLD_MS = 2000    # correct/incorrect colour dwell

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
                 animations: Sequence[str], led: Any = None) -> None:
        self._engine = engine
        self._choreo = choreographer
        self._animations = list(animations)
        self._led = led
        self._ids = itertools.count(1)

    def led_state(self) -> Optional[dict]:
        """The ring's current colour/animation, as an ``led`` step's args.

        Returns ``{"color": {...}}`` or ``{"animation": name}`` — whichever the
        ring is showing right now — or None if no ring is attached. Used to
        capture a state worth returning to after a transient flourish.
        """
        if self._led is None:
            return None
        return self._led.current

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

    def _restore_led(
        self, fallback: Tuple[int, int, int] = BLUE
    ) -> dict:
        """An `led` step returning the ring to whatever it shows right now.

        Captured at submit time, before the flourish starts, so a transient
        effect hands the ring back exactly as it found it instead of forcing
        one hardcoded colour. Falls back to *fallback* when there is no ring
        (sim/tests) or its state is unreadable.
        """
        state = self.robot.led_state()
        if isinstance(state, dict):
            if "animation" in state:
                return {"kind": "led", "animation": state["animation"]}
            color = state.get("color")
            if isinstance(color, dict) and {"r", "g", "b"} <= set(color):
                return {"kind": "led", "color": dict(color)}
        return self._color(fallback)

    def _flourish_steps(self, then: Optional[dict] = None) -> list:
        """The tap flourish: red, yellow, green, then *then*.

        Defaults to returning the ring to whatever it was showing before the
        flourish started. Replaces the old spinning rainbow, which needed the
        ring rewritten 50x a second and so was the animation that showed the
        servo-bus noise most clearly (docs/LED_NOISE.md). This is three static
        colours instead — four SPI writes in total rather than ~125.
        """
        steps: list = []
        for rgb in FLOURISH_COLORS:
            steps.append(self._color(rgb))
            steps.append({"kind": "wait", "duration_ms": FLOURISH_STEP_MS})
        steps.append(then if then is not None else self._restore_led())
        return steps

    def _flourish_then(self, rgb: Optional[Tuple[int, int, int]] = None) -> None:
        """Play the flourish once, then settle.

        With no argument the ring returns to its pre-flourish state; pass *rgb*
        to force a specific settle colour instead.
        """
        then = None if rgb is None else self._color(rgb)
        self.robot.act(seq(*self._flourish_steps(then)))

    def _browsing(self) -> None:
        """Library/home browsing: the flourish, then resting blue.

        Deliberately settles on BLUE rather than restoring the previous colour:
        leaving a book for the library is what clears a held rating colour.
        """
        self._flourish_then(BLUE)

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
        """A book was opened: the flourish, and settle looking down at the book.

        The head-down pose is a `posture`, not a `pose`: a pose lives on the
        gesture layer, whose claim expires ~1 s after it is set, so the head
        would drift back up on its own. A posture is held until close_book
        clears it, and gestures (nod, wiggle, page-turn glances) play over the
        top and settle back down to the book rather than to level.
        """
        self.robot.act(par(
            seq(*self._flourish_steps()),
            {"kind": "posture",
             "joints": {"head_pan": 0.0, "head_tilt": HEAD_DOWN}},
        ))

    async def on_tap_starred(self, book: Optional[str],
                             level: Optional[str]) -> None:
        """A book was starred/favourited."""
        self._browsing()

    async def on_close_book(self, book: Optional[str],
                            level: Optional[str]) -> None:
        """Book closed: back to blue, head level again.

        Releases the head-down posture set by on_tap_book, then drives the head
        up explicitly — clearing the posture alone would only blend back to
        whatever idle happens to want.
        """
        self.robot.act(par(
            self._color(BLUE),
            seq(
                {"kind": "posture", "clear": True},
                {"kind": "pose",
                 "joints": {"head_pan": 0.0, "head_tilt": HEAD_LEVEL},
                 "duration_ms": 900},
            ),
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
        """A page was tapped/turned: the flourish, back to how it was."""
        self._flourish_then()

    async def on_book_rate(self, rating: Optional[int]) -> None:
        """The child rated the book 1-5: show that rating's colour and hold it.

        The colour stays until something else claims the ring — a different
        rating, or navigating back out to the library/home screen (which the
        browsing handlers settle to BLUE). It is a standing indicator of the
        rating just given, not a momentary flash.
        """
        if rating is None:
            return
        rgb = RATING_COLORS.get(int(rating))
        if rgb is None:
            log.warning("book_rate: rating %r outside 1-5, ignoring", rating)
            return
        self.robot.led_color(*rgb)
