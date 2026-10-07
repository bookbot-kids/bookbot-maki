"""IdleBehavior — MAKI's autonomous heartbeat while no client is playing.

Arms after ``idle.delay_s`` of engine inactivity: the resting LED animation
(skipped while the app bridge holds a session colour — see ``led_hold``),
a 2 Hz keepalive republish of the held pose on the `idle` motion layer (so
the LayerBlender claim stays alive and the planner's micro-breathing keeps
running), and randomized blinks submitted THROUGH the engine at priority 5
with on_busy=drop — any client performance preempts/outranks them instantly.
Pauses entirely during e-stop.
"""

from __future__ import annotations

import asyncio
import logging
import random
import time
from typing import Any, Callable, Mapping, Optional

from .engine import ActionEngine, Performance
from .motion import joints as joints_mod
from .protocol import Action

log = logging.getLogger(__name__)

IDLE_PRIORITY = 5          # below every client priority (client floor is 0,
                           # but aliases/defaults are >= 20)
KEEPALIVE_PERIOD_S = 0.5   # 2 Hz idle-layer republish
POLL_PERIOD_S = 0.25


class IdleBehavior:
    def __init__(
        self,
        engine: ActionEngine,
        motion: Any,
        led: Any,
        config: Mapping[str, Any],
        *,
        rng: Optional[random.Random] = None,
        led_hold: Optional[Callable[[], bool]] = None,
    ) -> None:
        cfg = dict(config or {})
        # True while someone else owns the ring's colour (the app bridge
        # holding a session colour: practice purple, reading blue, book-end
        # green). Idle then leaves the LED alone, so re-arming after a client
        # act can never reset a held colour to the resting light mid-book.
        self._led_hold = led_hold
        self._engine = engine
        self._motion = motion
        self._led = led
        self._delay_s = float(cfg.get("delay_s", 5.0))
        self._led_animation = str(cfg.get("led_animation", "breathing_cyan"))
        interval = cfg.get("blink_interval_s", [4.0, 8.0])
        self._blink_lo = float(interval[0])
        self._blink_hi = float(interval[1])
        self._blink_duration_ms = int(cfg.get("blink_duration_ms", 150))
        self._rng = rng or random.Random()

        self._task: Optional[asyncio.Task] = None
        self._armed = False
        self._held_pose: dict = {}
        self._last_keepalive = 0.0
        self._next_blink = 0.0
        self._blink_counter = 0

    @property
    def armed(self) -> bool:
        return self._armed

    def start(self) -> None:
        if self._task is None or self._task.done():
            self._task = asyncio.ensure_future(self._run())

    def stop(self) -> None:
        if self._task is not None:
            self._task.cancel()
            self._task = None
        if self._armed:
            self._disarm()

    async def _run(self) -> None:
        try:
            while True:
                await asyncio.sleep(POLL_PERIOD_S)
                self._poll()
        except asyncio.CancelledError:
            raise

    def _poll(self) -> None:
        now = time.monotonic()
        if self._engine.estopped:
            if self._armed:
                self._disarm()
            return
        idle_for = now - self._engine.last_client_activity
        if self._engine.has_client_work() or idle_for < self._delay_s:
            if self._armed:
                self._disarm()
            return
        if not self._armed:
            self._arm(now)
        # 2 Hz keepalive: republish the held pose so the idle layer's claim
        # (and the planner's micro-breathing on claimed joints) stays alive.
        if now - self._last_keepalive >= KEEPALIVE_PERIOD_S and self._held_pose:
            self._last_keepalive = now
            try:
                self._motion.set_layer("idle", self._held_pose)
            except Exception:  # pragma: no cover - defensive
                log.exception("idle keepalive failed")
        if now >= self._next_blink:
            self._next_blink = now + self._rng.uniform(self._blink_lo, self._blink_hi)
            self._submit_blink()

    def _arm(self, now: float) -> None:
        self._armed = True
        pose = self._motion.pose_rad()
        # The mouth is externally driven (viseme stream, §5.13); idle must not
        # hold it, or every keepalive would fight the speech stream.
        self._held_pose = (
            {k: v for k, v in pose.items() if k != "mouth"}
            if pose
            else joints_mod.neutral_pose_rad(exclude=("mouth",))
        )
        self._last_keepalive = 0.0
        self._next_blink = now + self._rng.uniform(self._blink_lo, self._blink_hi)
        if self._led_held():
            log.debug("idle LED skipped: ring colour is held")
        else:
            try:
                self._led.set_animation(self._led_animation)
            except Exception:
                log.exception("idle LED animation %r failed", self._led_animation)
        log.debug("idle behavior armed")

    def _led_held(self) -> bool:
        if self._led_hold is None:
            return False
        try:
            return bool(self._led_hold())
        except Exception:  # pragma: no cover - defensive
            log.exception("idle led_hold check failed")
            return False

    def _disarm(self) -> None:
        self._armed = False
        try:
            self._motion.release_layer("idle")
        except Exception:  # pragma: no cover - defensive
            log.exception("idle layer release failed")
        log.debug("idle behavior disarmed")

    def _submit_blink(self) -> None:
        """Submit one idle blink.

        Tuning note: this only controls *when* and *how long*. How deep the
        blink looks is set by the eyelid acceleration ceiling in
        ``puppet.yaml`` (``motion.profiles.left_eyelid``) — the lid has a
        52.7 deg stroke to cover inside the close phase, so too little
        acceleration yields a shallow twitch no matter what duration is used.
        For softer blinks lower the eyelids' ``max_velocity_dps``, NOT their
        ``max_acceleration_dps2``. See the "TUNING BLINKS" block in
        ``config/puppet.yaml`` for the full reasoning.
        """
        self._blink_counter += 1
        perf = Performance(
            id=f"idle-blink-{self._blink_counter}",
            client=None,                    # internal: no acks, bypasses locks
            step=Action(kind="blink", args={"duration_ms": self._blink_duration_ms}),
            priority=IDLE_PRIORITY,
            on_busy="drop",                 # never queue behind client work
        )
        try:
            self._engine.submit(perf)
        except Exception:
            # estopped/locked race — the blink just doesn't happen
            log.debug("idle blink skipped", exc_info=True)
