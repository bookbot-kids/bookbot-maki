"""App-event bridge: Bookbot app events → robot actions.

This works like a Flutter platform channel, robot-side. The Flutter app calls
a typed method on ``MakiAppEvents`` (lib/maki_bridge.dart), which emits an
``event {name, params}`` frame over the WebSocket; the gateway routes any
event named below to the matching ``on_<event>`` method here — one method per
event, exactly like a ``MethodCallHandler`` switching on ``call.method``.

Each handler drives the robot through ``self.robot`` (a :class:`RobotAPI`).
The reaction vocabulary (colours, glance, session phases) is defined as
module constants below, so retuning the feel means editing one place:

    async def on_celebrate_example(self, word: str | None) -> None:
        self.robot.gesture("happy_wiggle", repeat=2)
        self.robot.led("steady_blue")
        # or an existing choreography from choreographies.yaml:
        self.robot.play("celebrate")
        # or a composed step (wire Step grammar, PROTOCOL.md §10):
        self.robot.act(seq(
            par({"kind": "gesture", "name": "nod"},
                {"kind": "led", "animation": "steady_purple"}),
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
#
# The palette is deliberately tiny (the product colour scheme): a neutral for
# browsing, blue for reading, purple for "needs practice", and green for the
# end of a book only. No red, orange or yellow anywhere in the reading flow,
# and no multi-colour flourishes — a colour on the ring always means one thing.
NEUTRAL = (150, 150, 150)  # library / book selection — matches `steady_white`
BLUE = (0, 80, 200)        # reading — matches `steady_blue`
PURPLE = (150, 0, 255)     # a word needs practice; the practice-words phase
BOOK_END_ANIMATION = "book_end_green"  # green, brighter than the rest; book end only

FLAG_HOLD_MS = 2000        # purple dwell for a word flagged while reading

# Normalized look target (positive tilt = DOWN — see PROTOCOL.md §3). A `look`
# drives head AND eyes from the same tilt, so 1.0 puts both at the bottom of
# their safe range: head tick 2300, eyes tick 2200 (~13 deg).
HEAD_DOWN = 1.0            # looking at the book

# A down-glance at the page. Chained short looks rather than one long one: the
# gesture layer's claim expires 1.0 s after its last update, so a single long
# look would drift back up halfway through; 600 ms steps keep a wide margin. When the act ends the layer blends
# out and face tracking takes the head back to the child.
GLANCE_MS = (600, 600, 600)
GLANCE_TAG = "glance"

# Where the app is in a session. Each phase owns one standing ring colour.
LIBRARY = "library"        # profile select, library, series — NEUTRAL
BOOK = "book"              # book opened, nothing read yet — NEUTRAL
PRACTICE = "practice"      # practice words before reading — PURPLE
READING = "reading"        # reading pages — BLUE (purple per flagged word)
PAGE_END = "page_end"      # page finished — PURPLE if it had errors, else BLUE
BOOK_END = "book_end"      # last page done, rating screen — green


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

    def cancel_tag(self, tag: str) -> int:
        """Cancel the queued/running performances submitted with *tag*."""
        return self._engine.cancel(f"tag:{tag}")


class AppEventBridge:
    """One ``on_<event>`` handler per Bookbot app event. Fill in the blanks.

    ``EVENT_PARAMS`` is the wire contract shared with lib/maki_bridge.dart:
    for each event, the ordered param keys the Flutter side sends. Params
    arrive as keyword arguments; a param the app omitted arrives as None.

    The bridge tracks which phase of a session the app is in (``phase``) and
    gives each phase one standing ring colour. Events that do not move the
    session to a new phase leave the ring alone, so a correct word, a mute or a
    repeated ``listen`` can never flash a colour or cut a purple signal short.
    """

    EVENT_PARAMS: Dict[str, Tuple[str, ...]] = {
        "tap_profile": ("profile",),
        "tap_category": ("category",),
        "tap_series": ("series",),
        "tap_book": ("book", "level"),
        "tap_starred": ("book", "level"),
        "close_book": ("book", "level"),
        "show_library": (),
        "practice_start": ("book",),
        "practice_correct": ("word",),
        "practice_incorrect": ("word",),
        "focus_word_correct": ("word",),
        "focus_word_incorrect": ("word",),
        "read_to_me": (),
        "listen": (),
        "mute": (),
        "tap_page": ("page",),
        "page_start": ("page",),
        "reading_word_incorrect": ("word",),
        "page_end": ("page", "errors"),
        "book_end": ("book", "level"),
        "book_rate": ("rating",),
    }

    def __init__(self, robot: RobotAPI) -> None:
        self.robot = robot
        # The ring boots on led.default_animation (steady_white), which is
        # the LIBRARY colour, so the two start in agreement.
        self.phase = LIBRARY
        # Words flagged on the current page; the fallback when page_end
        # arrives without an `errors` count.
        self.page_errors = 0

    @property
    def holds_ring(self) -> bool:
        """True while a session colour is up (anything but the library).

        IdleBehavior checks this before switching the ring to its resting
        light, so idle re-arming can never wipe a held colour mid-book.
        """
        return self.phase != LIBRARY

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

    def _phase_led(self) -> dict:
        """The `led` step for the current phase's standing colour."""
        if self.phase == BOOK_END:
            return {"kind": "led", "animation": BOOK_END_ANIMATION}
        if self.phase == PRACTICE:
            return self._color(PURPLE)
        if self.phase == READING:
            return self._color(BLUE)
        if self.phase == PAGE_END:
            return self._color(PURPLE if self.page_errors else BLUE)
        return self._color(NEUTRAL)

    def _enter(self, phase: str) -> None:
        """Move to *phase* and show its colour.

        Always submits, even when the phase is unchanged: the new act replaces
        whatever the LED channel is running, including a pending revert.
        """
        self.phase = phase
        self.robot.act(self._phase_led())

    def _glance_down(self) -> None:
        """Glance down at the page, then let face tracking take the head back.

        Motion-only and tagged, so it never touches the ring and a practice
        word or a mistake can cancel it (see _look_at_child).
        """
        self.robot.act(
            seq(*({"kind": "look", "pan": 0.0, "tilt": HEAD_DOWN,
                   "duration_ms": ms} for ms in GLANCE_MS)),
            tag=GLANCE_TAG,
        )

    def _look_at_child(self) -> None:
        """Cut any running glance short so the head goes back to the child.

        Practice words and mistakes must be met looking at the child, never
        at the page. Cancelling releases the gesture layer, and face tracking
        (which runs whenever nothing outranks it) takes over again.
        """
        self.robot.cancel_tag(GLANCE_TAG)

    def _flag_word(self) -> None:
        """A word needs practice mid-reading: purple now, then back to blue.

        One act, so the revert belongs to the same performance: if the next
        event replaces it mid-hold, the stale revert is cancelled with it
        instead of stomping the new colour.
        """
        self.page_errors += 1
        self.phase = READING
        self._look_at_child()
        self.robot.act(seq(
            self._color(PURPLE),
            {"kind": "wait", "duration_ms": FLAG_HOLD_MS},
            self._color(BLUE),
        ))

    def _practice(self) -> None:
        """Practice words: purple throughout, eyes on the child."""
        self._look_at_child()
        if self.phase != PRACTICE:
            self._enter(PRACTICE)

    # ── Handlers: one per app event ────────────────────────────────────

    # Library / browsing — neutral light.

    async def on_tap_profile(self, profile: Optional[str]) -> None:
        """A profile was tapped on the profile-select screen."""
        self._enter(LIBRARY)

    async def on_tap_category(self, category: Optional[str]) -> None:
        """A category tile was tapped in the library."""
        self._enter(LIBRARY)

    async def on_tap_series(self, series: Optional[str]) -> None:
        """A series was tapped in the library."""
        self._enter(LIBRARY)

    async def on_show_library(self) -> None:
        """The library screen appeared — including the return from a finished
        book, which is what clears the book-end green."""
        self.page_errors = 0
        self._enter(LIBRARY)

    async def on_tap_starred(self, book: Optional[str],
                             level: Optional[str]) -> None:
        """A book was starred/favourited.

        Leaves the ring alone: starring can happen on the book-end screen, and
        it must not knock the book-end green back to neutral.
        """

    async def on_tap_book(self, book: Optional[str],
                          level: Optional[str]) -> None:
        """A book was opened: still neutral until practice or reading starts.

        No head-down posture any more: MAKI looks at the child (face
        tracking) for the whole book and only glances down at page start and
        page end.
        """
        self.page_errors = 0
        self._enter(BOOK)

    async def on_close_book(self, book: Optional[str],
                            level: Optional[str]) -> None:
        """Book closed: back to the neutral library light."""
        self.page_errors = 0
        self._enter(LIBRARY)

    # Practice words — continuous purple, looking at the child.

    async def on_practice_start(self, book: Optional[str]) -> None:
        """The practice-words phase began, before the first word."""
        self._look_at_child()
        self._enter(PRACTICE)

    async def on_practice_correct(self, word: Optional[str]) -> None:
        """A practice word was read correctly: no colour change."""
        self._practice()

    async def on_practice_incorrect(self, word: Optional[str]) -> None:
        """A practice word was read incorrectly: stays purple."""
        self._practice()

    async def on_focus_word_correct(self, word: Optional[str]) -> None:
        """The focus word was read correctly: no colour change, ever."""

    async def on_focus_word_incorrect(self, word: Optional[str]) -> None:
        """The focus word was read incorrectly.

        Mid-reading it is a flagged word (purple, then back to blue); anywhere
        else it is part of the practice words.
        """
        if self.phase in (READING, PAGE_END):
            self._flag_word()
        else:
            self._practice()

    # Reading — blue, purple per flagged word.

    async def on_read_to_me(self) -> None:
        """The child chose 'read to me' (narration) mode: reading starts."""
        if self.phase in (LIBRARY, BOOK, PRACTICE):
            self._enter(READING)

    async def on_listen(self) -> None:
        """The app started listening to the child read.

        Starts reading when nothing else has yet. During practice words the
        app listens too, and that must not turn the purple to blue; mid-page
        it must not cut a flagged word's purple short.
        """
        if self.phase in (LIBRARY, BOOK):
            self._enter(READING)

    async def on_mute(self) -> None:
        """Sound was muted: nothing to show."""

    async def on_tap_page(self, page: Optional[int]) -> None:
        """A page was turned: back to reading blue for the new page.

        No glance here — page_start and page_end are the glance moments, and
        a third per page would stop them reading as deliberate. The error
        count is left to page_start: if the app sends the turn before
        page_end, resetting here would lose the page's errors.
        """
        if self.phase != READING:
            self._enter(READING)

    async def on_page_start(self, page: Optional[int]) -> None:
        """The child began reading a page: blue, and a glance at the page."""
        self.page_errors = 0
        if self.phase != READING:
            self._enter(READING)
        self._glance_down()

    async def on_reading_word_incorrect(self, word: Optional[str]) -> None:
        """A word was flagged for practice while reading: purple at once."""
        self._flag_word()

    async def on_page_end(self, page: Optional[int],
                          errors: Optional[int]) -> None:
        """The child finished reading the page.

        With errors: purple held until the page turns, looking at the child
        (like the practice words). All correct: stays blue, with a glance down
        at the finished page. Trusts the app's ``errors`` count; falls back to
        the words flagged on this page when it is missing.
        """
        if errors is not None:
            try:
                self.page_errors = max(0, int(errors))
            except (TypeError, ValueError):
                log.warning("page_end: errors %r is not a count; using %d",
                            errors, self.page_errors)
        self._enter(PAGE_END)
        if self.page_errors:
            self._look_at_child()
        else:
            self._glance_down()

    # Book end — brighter green, held until the library.

    async def on_book_end(self, book: Optional[str],
                          level: Optional[str]) -> None:
        """The last page was finished: green, held until the library shows."""
        self._enter(BOOK_END)

    async def on_book_rate(self, rating: Optional[int]) -> None:
        """The child rated the book (1-5).

        Rating is part of the book end, and green is the book end's colour, so
        the rating itself shows no colour of its own — the old per-rating
        palette was red/orange/yellow, which the scheme rules out. Re-enters
        book end so the green is up even if book_end was missed.
        """
        if self.phase != BOOK_END:
            self._enter(BOOK_END)
