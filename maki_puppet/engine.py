"""ActionEngine — per-channel arbitration and execution of performances.

One performance = one `act` or resolved `event` (or an internal idle
behavior).  A performance claims the UNION of the channels used anywhere in
its Step tree, from `started` until its terminal ack (PROTOCOL.md §7).

Arbitration (PROTOCOL.md §9): per-channel priority + on_busy
(queue | replace | drop), per-channel FIFO queues of depth 16, exclusive
per-connection channel locks with TTL, and an e-stop that beats everything.

All engine methods must be called from the asyncio event-loop thread; the
engine talks to MotionService/LedRing through their thread-safe interfaces.
"""

from __future__ import annotations

import asyncio
import logging
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, Iterable, List, Optional

from . import protocol
from .actions import ACTIONS, ActionContext, execute_step
from .protocol import (
    CHANNEL_ORDER,
    Channel,
    E_ESTOPPED,
    E_LOCKED,
    E_QUEUE_FULL,
    ProtocolError,
    Seq,
    Step,
)

log = logging.getLogger(__name__)

QUEUE_DEPTH = 16

# Terminal ack statuses
TERMINAL = ("completed", "cancelled", "superseded", "dropped", "error")

AckCallback = Callable[["Performance", str, dict], None]


@dataclass
class Performance:
    """One unit of arbitration/execution."""

    id: str
    client: Any                       # owning connection object, or None = internal
    step: Step
    priority: int = 50
    on_busy: str = "queue"
    tag: Optional[str] = None
    choreography: Optional[str] = None   # resolved event name, for acks
    channels: frozenset = frozenset()    # filled from the step if empty
    # runtime
    state: str = "pending"            # pending | queued | active | done
    task: Optional[asyncio.Task] = None
    ctx: Optional[ActionContext] = None
    cancel_status: Optional[str] = None  # "cancelled" | "superseded" while unwinding
    started_at: float = 0.0

    def __post_init__(self) -> None:
        if not self.channels:
            self.channels = protocol.step_channels(self.step)

    def channel_names(self) -> List[str]:
        return [c.value for c in CHANNEL_ORDER if c in self.channels]


class ActionEngine:
    def __init__(
        self,
        motion: Any,
        led: Any,
        *,
        get_gesture: Callable[[str], list],
        ack_cb: Optional[AckCallback] = None,
        state_changed_cb: Optional[Callable[[], None]] = None,
        alarm_animation: str = "alarm_red",
        default_animation: str = "breathing_cyan",
        runners: Dict[str, Any] = ACTIONS,
    ) -> None:
        self._motion = motion
        self._led = led
        self._get_gesture = get_gesture
        self.ack_cb = ack_cb
        self.state_changed_cb = state_changed_cb
        self._alarm_animation = alarm_animation
        self._default_animation = default_animation
        self._runners = runners

        self._active: Dict[Channel, Performance] = {}
        self._queues: Dict[Channel, deque] = {ch: deque() for ch in Channel}
        self._locks: Dict[Channel, tuple] = {}   # channel -> (client, expires_at)
        self._layer_owner: Dict[str, Performance] = {}
        self._estop = False
        self._last_client_activity = time.monotonic()

    # ── Introspection (used by server / idle) ─────────────────────────

    @property
    def estopped(self) -> bool:
        return self._estop

    @property
    def last_client_activity(self) -> float:
        return self._last_client_activity

    def has_client_work(self) -> bool:
        """True while any client-owned performance is active or queued."""
        return any(p.client is not None for p in self._live_performances())

    def queue_snapshot(self) -> dict:
        queued = {
            id(p): p for q in self._queues.values() for p in q if p.client is not None
        }
        active = None
        for ch in CHANNEL_ORDER:
            p = self._active.get(ch)
            if p is not None and p.client is not None:
                active = p
                break
        return {
            "depth": len(queued),
            "active_id": active.id if active else None,
            "active_client": getattr(active.client, "name", None) if active else None,
        }

    def lock_snapshot(self) -> dict:
        now = time.monotonic()
        holder = None
        scopes = []
        for ch in CHANNEL_ORDER:
            client = self._lock_holder(ch, now)
            if client is not None:
                holder = getattr(client, "name", str(client))
                scopes.append(ch.value)
        return {"held_by": holder, "scopes": scopes}

    def _live_performances(self) -> List[Performance]:
        seen: Dict[int, Performance] = {}
        for p in self._active.values():
            seen[id(p)] = p
        for q in self._queues.values():
            for p in q:
                seen[id(p)] = p
        return list(seen.values())

    # ── Submission / arbitration ───────────────────────────────────────

    def submit(self, perf: Performance) -> str:
        """Apply priority + on_busy semantics and start/queue/drop.

        Returns the initial disposition: "started" | "queued" | "dropped".
        Raises ProtocolError (estopped / locked / queue_full) — the caller
        acks those as a single terminal error.
        """
        if self._estop:
            raise ProtocolError(E_ESTOPPED, "e-stop engaged; act/event rejected")
        self._check_locks(perf)
        if perf.client is not None:
            self._last_client_activity = time.monotonic()

        chans = perf.channels
        if not chans:
            # wait-only tree: claims no channel, never contested (§7)
            self._ack(perf, "accepted", channels=[])
            self._start(perf)
            return "started"

        busy = [ch for ch in chans if self._active.get(ch) is not None]
        queued_ahead = any(self._queues[ch] for ch in chans)

        if not busy and not queued_ahead:
            self._ack(perf, "accepted", channels=perf.channel_names())
            self._start(perf)
            return "started"

        if perf.on_busy == "drop":
            # single terminal ack, no preceding accepted (§8.1)
            perf.state = "done"
            self._ack(perf, "dropped")
            return "dropped"

        if perf.on_busy == "replace" and busy:
            victims = {id(self._active[ch]): self._active[ch] for ch in busy}
            if all(perf.priority >= v.priority for v in victims.values()):
                self._ack(perf, "accepted", channels=perf.channel_names())
                for victim in victims.values():
                    self._preempt(victim, "superseded")
                self._start(perf)
                return "started"
            # lower priority never preempts — degrade to queue (§9.2)

        for ch in chans:
            if len(self._queues[ch]) >= QUEUE_DEPTH:
                raise ProtocolError(
                    E_QUEUE_FULL, f"channel '{ch.value}' queue is full ({QUEUE_DEPTH})"
                )
        self._ack(perf, "accepted", channels=perf.channel_names())
        pos = 0
        for ch in chans:
            self._queues[ch].append(perf)
            pos = max(pos, len(self._queues[ch]) - 1)
        perf.state = "queued"
        self._ack(perf, "queued", channels=perf.channel_names(), queue_pos=pos)
        self._state_changed()
        return "queued"

    def _check_locks(self, perf: Performance) -> None:
        if perf.client is None:
            return  # locks never stop the server's own idle/safety work (§9.3)
        now = time.monotonic()
        for ch in perf.channels:
            holder = self._lock_holder(ch, now)
            if holder is not None and holder is not perf.client:
                raise ProtocolError(
                    E_LOCKED,
                    f"channel '{ch.value}' locked by "
                    f"'{getattr(holder, 'name', 'other client')}'",
                )

    # ── Execution ──────────────────────────────────────────────────────

    def _start(self, perf: Performance) -> None:
        for ch in perf.channels:
            self._active[ch] = perf
        perf.state = "active"
        perf.started_at = time.monotonic()
        fields: dict = {"channels": perf.channel_names()}
        if isinstance(perf.step, Seq):
            fields["progress"] = {"step": 1, "of": len(perf.step.children)}
        self._ack(perf, "started", **fields)
        perf.ctx = ActionContext(
            motion=self._motion,
            led=self._led,
            get_gesture=self._get_gesture,
            claim_layer=lambda layer, p=perf: self._layer_owner.__setitem__(layer, p),
        )
        perf.task = asyncio.ensure_future(self._run_perf(perf))
        perf.task.add_done_callback(lambda _t, p=perf: self._ensure_finished(p))
        self._state_changed()

    async def _run_perf(self, perf: Performance) -> None:
        try:
            await execute_step(perf.step, perf.ctx, self._runners)
        except asyncio.CancelledError:
            self._finish(perf, perf.cancel_status or "cancelled")
            raise
        except ProtocolError as e:
            self._finish(perf, "error", code=e.code, detail=e.detail)
        except Exception as e:  # pragma: no cover - defensive
            log.exception("performance %s failed", perf.id)
            self._finish(perf, "error", code=protocol.E_INTERNAL, detail=str(e))
        else:
            duration_ms = int((time.monotonic() - perf.started_at) * 1000)
            self._finish(perf, "completed", duration_ms=duration_ms)

    def _ensure_finished(self, perf: Performance) -> None:
        """Safety net: a task cancelled before it ever ran never reaches
        _run_perf's handlers — finish it here."""
        if perf.state != "done":
            self._finish(perf, perf.cancel_status or "cancelled")

    def _finish(self, perf: Performance, status: str, **fields: Any) -> None:
        if perf.state == "done":
            return
        perf.state = "done"
        if perf.client is not None:
            self._last_client_activity = time.monotonic()
        # Release every motion layer this performance touched — but only if
        # it is still the most recent feeder (a superseding performance may
        # already be driving the same layer).
        ctx = perf.ctx
        if ctx is not None:
            for layer in ctx.touched_layers:
                if self._layer_owner.get(layer) is perf:
                    del self._layer_owner[layer]
                    try:
                        self._motion.release_layer(layer)
                    except Exception:  # pragma: no cover - defensive
                        log.exception("release_layer(%r) failed", layer)
        for ch, active in list(self._active.items()):
            if active is perf:
                del self._active[ch]
        self._ack(perf, status, **fields)
        self._pump()
        self._state_changed()

    def _preempt(self, perf: Performance, status: str) -> None:
        """Cancel a running performance; its terminal ack carries `status`."""
        perf.cancel_status = status
        for ch, active in list(self._active.items()):
            if active is perf:
                del self._active[ch]
        if perf.task is not None:
            perf.task.cancel()
        else:  # active but task not created — should not happen
            self._finish(perf, status)

    def _dequeue(self, perf: Performance, status: str) -> None:
        for q in self._queues.values():
            try:
                q.remove(perf)
            except ValueError:
                pass
        perf.state = "done"
        self._ack(perf, status)

    def _pump(self) -> None:
        """Start queue heads whose channels are all free (multi-channel
        performances start atomically: head of every queue they claim)."""
        progressed = True
        while progressed:
            progressed = False
            for ch in CHANNEL_ORDER:
                q = self._queues[ch]
                if not q or self._active.get(ch) is not None:
                    continue
                head = q[0]
                startable = all(
                    self._active.get(c) is None
                    and self._queues[c]
                    and self._queues[c][0] is head
                    for c in head.channels
                )
                if startable:
                    for c in head.channels:
                        self._queues[c].popleft()
                    self._start(head)
                    progressed = True

    # ── Cancel / e-stop / locks ────────────────────────────────────────

    def cancel(self, target: str, client: Any = None) -> int:
        """`cancel {target}`: id (requester's own), "all", or "tag:<x>".

        Returns the number of performances cancelled (0 is a no-op, not an
        error).
        """
        live = self._live_performances()
        if target == "all":
            victims = [p for p in live if p.client is not None]
        elif target.startswith("tag:"):
            tag = target[4:]
            victims = [p for p in live if p.tag == tag]
        else:
            victims = [p for p in live if p.id == target and p.client is client]
        for p in victims:
            self._cancel_perf(p, "cancelled")
        return len(victims)

    def _cancel_perf(self, perf: Performance, status: str) -> None:
        if perf.state == "queued":
            self._dequeue(perf, status)
            self._pump()
            self._state_changed()
        elif perf.state == "active":
            self._preempt(perf, status)

    def set_estop(self, engaged: bool, reason: str = "") -> None:
        """Engage: flush queues + cancel everything, freeze motion, LED alarm.
        Disengage: resume motion, LED back to the default animation."""
        if engaged and not self._estop:
            self._estop = True
            log.warning("E-STOP engaged%s", f": {reason}" if reason else "")
            for p in self._live_performances():
                self._cancel_perf(p, "cancelled")
            for q in self._queues.values():
                q.clear()
            self._motion.estop(True)
            self._set_led(self._alarm_animation)
        elif not engaged and self._estop:
            self._estop = False
            log.info("E-STOP disengaged%s", f": {reason}" if reason else "")
            self._motion.estop(False)
            self._set_led(self._default_animation)
            self._last_client_activity = time.monotonic()
        self._state_changed()

    def _set_led(self, animation: str) -> None:
        try:
            self._led.set_animation(animation)
        except Exception:
            log.exception("LED animation %r failed", animation)

    def lock(self, client: Any, scopes: Iterable[Channel], ttl_s: float) -> None:
        now = time.monotonic()
        scopes = list(scopes)
        for ch in scopes:
            holder = self._lock_holder(ch, now)
            if holder is not None and holder is not client:
                raise ProtocolError(
                    E_LOCKED,
                    f"channel '{ch.value}' already locked by "
                    f"'{getattr(holder, 'name', 'other client')}'",
                )
        for ch in scopes:
            self._locks[ch] = (client, now + float(ttl_s))
        self._state_changed()

    def unlock(self, client: Any, scopes: Iterable[Channel]) -> None:
        """Releases only scopes this client holds; anything else is a no-op."""
        now = time.monotonic()
        for ch in scopes:
            if self._lock_holder(ch, now) is client:
                del self._locks[ch]
        self._state_changed()

    def _lock_holder(self, ch: Channel, now: float) -> Any:
        entry = self._locks.get(ch)
        if entry is None:
            return None
        client, expires_at = entry
        if now >= expires_at:   # TTL expiry is lazy
            del self._locks[ch]
            return None
        return client

    def release_client(self, client: Any, abort: bool = False) -> None:
        """Connection dropped: auto-release its locks; if it asked for
        abort_on_disconnect, cancel its queued + running work."""
        for ch in list(self._locks):
            if self._locks[ch][0] is client:
                del self._locks[ch]
        if abort:
            for p in self._live_performances():
                if p.client is client:
                    self._cancel_perf(p, "cancelled")
        self._state_changed()

    # ── Shutdown ───────────────────────────────────────────────────────

    async def stop_all(self) -> None:
        """Cancel every queued and running performance (incl. internal)."""
        tasks = []
        for p in self._live_performances():
            if p.state == "active" and p.task is not None:
                tasks.append(p.task)
            self._cancel_perf(p, "cancelled")
        for q in self._queues.values():
            q.clear()
        if tasks:
            await asyncio.gather(*tasks, return_exceptions=True)

    # ── Ack / state plumbing ───────────────────────────────────────────

    def _ack(self, perf: Performance, status: str, **fields: Any) -> None:
        if perf.choreography is not None:
            fields.setdefault("choreography", perf.choreography)
        if self.ack_cb is not None:
            try:
                self.ack_cb(perf, status, fields)
            except Exception:  # pragma: no cover - defensive
                log.exception("ack callback failed")

    def _state_changed(self) -> None:
        if self.state_changed_cb is not None:
            try:
                self.state_changed_cb()
            except Exception:  # pragma: no cover - defensive
                log.exception("state-changed callback failed")
