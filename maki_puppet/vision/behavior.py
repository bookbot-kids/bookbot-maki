"""FaceTrackingBehavior — autonomous face following.

MAKI's own reflex: while it is awake and nobody is driving it, it looks at
whoever is in front of it.  This is the counterpart to the client-driven
`track` action — same control law, but armed by the robot rather than asked
for over the wire.

It is modelled on :class:`~maki_puppet.idle.IdleBehavior`: a polling task that
feeds a motion layer directly rather than submitting performances through the
engine.  Continuous behaviors don't fit the performance model — a performance
has a duration and terminal ack, and re-submitting one every 50 ms would churn
the arbitration queues for no benefit.

Arbitration is left entirely to the LayerBlender, which is what it is for:

  idle (30) < tracking (50) < gesture (55)

so autonomous tracking overrides the idle pose but yields to any deliberate
client gesture — MAKI keeps looking at you until an act tells it to look
somewhere else, then goes back to you when that act finishes.  On top of that
the behavior stands down entirely while any client performance is live, so a
client's `track` action never fights this one for the same layer.

It also stands down while a **posture** is held.  That is the "reading" case:
opening a book installs a head-down posture (see ``AppEventBridge.on_tap_book``)
which persists until the book closes, and a reflex that kept pulling the head
up to the reader's face would defeat the entire point of it.  Closing the book
clears the posture and face-following resumes on its own.
"""

from __future__ import annotations

import asyncio
import logging
import time
from typing import Any, Mapping, Optional

from .tracker import FaceTracker, TrackerConfig

log = logging.getLogger(__name__)

# Control rate. Matches the tracker's control_min_dt_s — polling faster just
# burns CPU, since update() rejects ticks that arrive early.
TICK_S = 0.05  # 20 Hz


class FaceTrackingBehavior:
    """Follows a face whenever one is visible and nothing else is driving."""

    def __init__(
        self,
        engine: Any,
        motion: Any,
        vision: Any,
        tracker_config: Optional[TrackerConfig] = None,
        config: Optional[Mapping[str, Any]] = None,
        *,
        clock=time.monotonic,
    ) -> None:
        cfg = dict(config or {})
        self._engine = engine
        self._motion = motion
        self._vision = vision
        self._clock = clock
        self._enabled = bool(cfg.get("enabled", True))
        # How long to keep holding the last targets after the face goes away,
        # before handing the joints back. Short enough that MAKI doesn't stare
        # at an empty chair; long enough to ride out a few dropped detections
        # and someone turning their head away briefly.
        self._release_after_s = float(cfg.get("release_after_s", 2.0))
        self._eyes_only = bool(cfg.get("eyes_only", False))
        # Yield to a held posture (e.g. head down while a book is open).
        # Turning this off makes MAKI keep following faces over the top of the
        # book pose, which is almost never what you want.
        self._yield_to_posture = bool(cfg.get("yield_to_posture", True))

        self._tracker = FaceTracker(tracker_config or TrackerConfig(), clock=clock)
        self._task: Optional[asyncio.Task] = None
        self._armed = False
        self._last_face_at = 0.0

    @property
    def armed(self) -> bool:
        return self._armed

    @property
    def enabled(self) -> bool:
        return self._enabled and self._vision is not None

    # ── Lifecycle ──────────────────────────────────────────────────────

    def start(self) -> None:
        if not self.enabled:
            log.debug("face tracking behavior disabled (no vision or turned off)")
            return
        if self._task is None or self._task.done():
            self._task = asyncio.ensure_future(self._run())
            log.info("autonomous face tracking armed")

    def stop(self) -> None:
        if self._task is not None:
            self._task.cancel()
            self._task = None
        if self._armed:
            self._disarm()

    async def _run(self) -> None:
        while True:
            await asyncio.sleep(TICK_S)
            try:
                self._poll()
            except asyncio.CancelledError:
                raise
            except Exception:
                # A fault here must never kill the behavior: it would leave the
                # tracking layer claimed with stale targets until it timed out.
                log.exception("face tracking tick failed")

    # ── Control ────────────────────────────────────────────────────────

    def _poll(self) -> None:
        now = self._clock()

        # Stand down for e-stop and for any client-owned work. The client may
        # be running its own `track`, which writes the same layer; two authors
        # on one layer is a fight neither wins.
        if self._engine.estopped or self._engine.has_client_work():
            if self._armed:
                self._disarm()
            return

        # Stand down while a posture is held. A posture is the app stating
        # where a joint lives until it says otherwise — "head down at the book
        # until it closes" — and following a face would drag the head straight
        # back off the page. Layer priority can't express this on its own:
        # `tracking` (50) deliberately outranks `posture` (45) so an explicitly
        # requested `track` act still wins, so the reflex has to yield here.
        if self._suppressed_by_posture():
            if self._armed:
                self._disarm()
            return

        face = self._vision.latest_face(now)
        if face is None:
            # Hold the last targets briefly, then release. Releasing lets the
            # layer blend out gracefully rather than snapping home.
            if self._armed and now - self._last_face_at >= self._release_after_s:
                self._disarm()
            return

        if not self._armed:
            self._arm()
        self._last_face_at = now

        output = self._tracker.update(face, self._motion.pose_rad(), now)
        if output is not None:
            self._motion.set_layer(
                "tracking", output.as_targets(eyes_only=self._eyes_only)
            )

    def _suppressed_by_posture(self) -> bool:
        """True when a posture is held and we're configured to yield to it.

        Tolerant of a motion service without the hook so the behavior still
        works against older/stub services rather than failing every tick.
        """
        if not self._yield_to_posture:
            return False
        has_posture = getattr(self._motion, "has_posture", None)
        return bool(has_posture()) if callable(has_posture) else False

    def _arm(self) -> None:
        self._armed = True
        # Fresh control state per sighting: a tracker carrying the anchor from
        # the last person would drive toward where THEY were standing.
        self._tracker.reset()
        log.info("face acquired - tracking")

    def _disarm(self) -> None:
        self._armed = False
        try:
            self._motion.release_layer("tracking")
        except Exception:  # pragma: no cover - defensive
            log.exception("tracking layer release failed")
        log.info("face lost - tracking released")
