"""Pure-Python client SDK for the MAKI puppet gateway (MPP/1).

Implements the client side of ``maki_puppet/PROTOCOL.md`` over a single
WebSocket. Depends only on ``websockets`` and the standard library — it must
never import from ``maki_puppet`` (this SDK runs on laptops and other
machines, not just the robot).

Typical use::

    from maki_client import Maki

    async with await Maki.connect("ws://localhost:8765/ws") as m:
        await m.emit("celebrate")     # fire-and-forget (ignore the handle)
        h = await m.blink()           # direct action
        await h                       # block until the terminal ack
"""

from __future__ import annotations

import asyncio
import contextlib
import json
import logging
import random
import time
from dataclasses import dataclass, field
from typing import Any, Callable, Iterable, Optional

import websockets
from websockets.exceptions import ConnectionClosed, WebSocketException

log = logging.getLogger("maki_client")

DEFAULT_URL = "ws://localhost:8765/ws"
PROTOCOL_VERSION = 1

#: Ack statuses that end an action's lifecycle (PROTOCOL.md §8.1).
TERMINAL_STATUSES = frozenset({"completed", "cancelled", "superseded", "dropped", "error"})

_CALLBACK_KINDS = ("state", "event", "disconnect")


# ── Errors ─────────────────────────────────────────────────────────────────


class MakiError(Exception):
    """Base class for every error raised by this SDK."""


class MakiConnectionError(MakiError):
    """Could not connect, the connection was lost, or a request timed out."""


class MakiProtocolError(MakiError):
    """The server rejected our handshake or violated MPP/1."""


class MakiActionError(MakiError):
    """An act/event was acked ``status: "error"`` (codes in PROTOCOL.md §8.2)."""

    def __init__(self, code: str, detail: str = "", ref: Optional[str] = None):
        super().__init__(f"{code}: {detail}" if detail else code)
        self.code = code
        self.detail = detail
        self.ref = ref


class MakiCancelled(MakiError):
    """An act/event terminated without completing.

    ``status`` is one of ``"cancelled"`` (cancel / e-stop / abort_on_disconnect),
    ``"superseded"`` (replaced by a higher-priority act) or ``"dropped"``
    (``on_busy: drop`` while the channel was busy).
    """

    def __init__(self, status: str, ref: Optional[str] = None):
        super().__init__(status)
        self.status = status
        self.ref = ref


# ── Step builders (PROTOCOL.md §7) ─────────────────────────────────────────


def seq(*steps: dict) -> dict:
    """Compose steps to run in order: ``seq(a, b, c)`` → ``{"seq": [a, b, c]}``."""
    if not steps:
        raise ValueError("seq() needs at least one step")
    return {"seq": list(steps)}


def par(*steps: dict) -> dict:
    """Compose steps to run concurrently: ``par(a, b)`` → ``{"par": [a, b]}``."""
    if not steps:
        raise ValueError("par() needs at least one step")
    return {"par": list(steps)}


def _action(kind: str, **args: Any) -> dict:
    """Build an Action dict, omitting arguments left as None (server defaults)."""
    step: dict[str, Any] = {"kind": kind}
    for key, value in args.items():
        if value is not None:
            step[key] = value
    return step


# ── State snapshot ─────────────────────────────────────────────────────────


@dataclass
class RobotState:
    """One ``state`` payload (PROTOCOL.md §11), wire joints and units."""

    pose: dict = field(default_factory=dict)
    led: dict = field(default_factory=dict)
    queue: dict = field(default_factory=dict)
    estop: bool = False
    lock: dict = field(default_factory=dict)
    clients: list = field(default_factory=list)
    health: dict = field(default_factory=dict)
    raw: dict = field(default_factory=dict, repr=False)

    @classmethod
    def from_payload(cls, payload: dict) -> "RobotState":
        return cls(
            pose=payload.get("pose") or {},
            led=payload.get("led") or {},
            queue=payload.get("queue") or {},
            estop=bool(payload.get("estop", False)),
            lock=payload.get("lock") or {},
            clients=payload.get("clients") or [],
            health=payload.get("health") or {},
            raw=payload,
        )


# ── Action handle ──────────────────────────────────────────────────────────


class ActionHandle:
    """Tracks one act/event through its ack lifecycle. Awaitable.

    - ``await handle`` (or ``await handle.wait(timeout)``) blocks until the
      terminal ack: ``completed`` returns the ack payload; ``error`` raises
      :class:`MakiActionError`; ``cancelled``/``superseded``/``dropped`` raise
      :class:`MakiCancelled`; a connection drop raises
      :class:`MakiConnectionError`.
    - Fire-and-forget is fine: an unawaited handle never warns or leaks.
    """

    def __init__(self, msg_id: str, desc: str = ""):
        self.id = msg_id
        self.desc = desc
        self.status: str = "sent"  # latest known ack status
        self.acks: list[dict] = []  # every ack payload received, in order
        self.result: Optional[dict] = None  # terminal payload when completed
        self._future: asyncio.Future = asyncio.get_running_loop().create_future()

    @property
    def done(self) -> bool:
        """True once the action reached a terminal state (or the link died)."""
        return self._future.done()

    def __await__(self):
        return self.wait().__await__()

    async def wait(self, timeout: Optional[float] = None) -> dict:
        """Wait for the terminal ack; ``timeout`` in seconds (None = forever)."""
        fut = asyncio.shield(self._future)
        if timeout is None:
            return await fut
        try:
            return await asyncio.wait_for(fut, timeout)
        except asyncio.TimeoutError as exc:
            raise MakiConnectionError(
                f"timed out after {timeout}s waiting for {self.desc or self.id} "
                f"(last status: {self.status})"
            ) from exc

    def __repr__(self) -> str:  # pragma: no cover - debugging aid
        return f"<ActionHandle {self.id} {self.desc!r} status={self.status}>"

    # Internal — called from the client's receive loop.

    def _on_ack(self, payload: dict) -> None:
        status = str(payload.get("status", ""))
        self.acks.append(payload)
        if status:
            self.status = status
        if status not in TERMINAL_STATUSES or self._future.done():
            return
        if status == "completed":
            self.result = payload
            self._future.set_result(payload)
            return
        if status == "error":
            exc: MakiError = MakiActionError(
                str(payload.get("code", "internal")),
                str(payload.get("detail", "")),
                ref=self.id,
            )
        else:  # cancelled | superseded | dropped
            exc = MakiCancelled(status, ref=self.id)
        self._future.set_exception(exc)
        # Mark retrieved so fire-and-forget handles never log
        # "exception was never retrieved"; awaiting still raises.
        self._future.exception()

    def _fail(self, exc: MakiError) -> None:
        if not self._future.done():
            self._future.set_exception(exc)
            self._future.exception()


# ── Client ─────────────────────────────────────────────────────────────────


class Maki:
    """Async client for the MAKI puppet gateway. Create via :meth:`connect`."""

    def __init__(self, **cfg: Any):
        self._url: str = cfg["url"]
        self._name: str = cfg["name"]
        self._kind: str = cfg["kind"]
        self._version: str = cfg["version"]
        self._priority: int = cfg["priority"]
        self._subscribe: list[str] = list(cfg["subscribe"])
        self._abort_on_disconnect: bool = cfg["abort_on_disconnect"]
        self._reconnect: bool = cfg["reconnect"]
        self._handshake_timeout: float = cfg["handshake_timeout"]
        self._backoff_initial: float = cfg["backoff_initial"]
        self._backoff_max: float = cfg["backoff_max"]

        self._ws: Optional[Any] = None
        self._welcome: dict = {}
        self._connected = False
        self._closed = False
        self._counter = 0
        self._runner: Optional[asyncio.Task] = None
        self._pending: dict[str, ActionHandle] = {}
        self._state_waiters: dict[str, asyncio.Future] = {}
        self._ping_waiters: dict[str, asyncio.Future] = {}
        self._last_state: Optional[RobotState] = None
        self._callbacks: dict[str, list[Callable[[Any], Any]]] = {
            k: [] for k in _CALLBACK_KINDS
        }

    # ── Lifecycle ──────────────────────────────────────────────────────

    @classmethod
    async def connect(
        cls,
        url: str = DEFAULT_URL,
        *,
        name: str = "python-client",
        kind: str = "python",
        version: str = "0.1.0",
        priority: int = 50,
        subscribe: Iterable[str] = ("state",),
        abort_on_disconnect: bool = False,
        reconnect: bool = True,
        handshake_timeout: float = 10.0,
        backoff_initial: float = 0.5,
        backoff_max: float = 8.0,
    ) -> "Maki":
        """Connect and complete the MPP/1 handshake; raises on failure.

        The *initial* connection always fails fast (so a bad URL is loud);
        ``reconnect=True`` governs recovery after a later drop: exponential
        backoff ``backoff_initial → backoff_max`` (doubling, ±25 % jitter),
        re-``hello`` on success. Handles outstanding at the moment of a drop
        are resolved with :class:`MakiConnectionError`.
        """
        self = cls(
            url=url,
            name=name,
            kind=kind,
            version=version,
            priority=priority,
            subscribe=subscribe,
            abort_on_disconnect=abort_on_disconnect,
            reconnect=reconnect,
            handshake_timeout=handshake_timeout,
            backoff_initial=backoff_initial,
            backoff_max=backoff_max,
        )
        await self._connect_once()
        self._runner = asyncio.create_task(self._run(), name="maki-client")
        return self

    async def close(self) -> None:
        """Close the connection and resolve all outstanding handles. Idempotent."""
        if self._closed:
            return
        self._closed = True
        self._reconnect = False
        self._connected = False
        runner, self._runner = self._runner, None
        if runner is not None and runner is not asyncio.current_task():
            runner.cancel()
            with contextlib.suppress(asyncio.CancelledError, Exception):
                await runner
        ws, self._ws = self._ws, None
        if ws is not None:
            with contextlib.suppress(Exception):
                await ws.close()
        self._fail_pending(MakiConnectionError("client closed"))

    async def __aenter__(self) -> "Maki":
        return self

    async def __aexit__(self, *exc_info: Any) -> None:
        await self.close()

    # ── Welcome-derived properties ─────────────────────────────────────

    @property
    def connected(self) -> bool:
        return self._connected

    @property
    def welcome(self) -> dict:
        """The full ``welcome`` payload from the current session."""
        return dict(self._welcome)

    @property
    def session(self) -> str:
        return str(self._welcome.get("session", ""))

    @property
    def capabilities(self) -> list[str]:
        return list(self._welcome.get("capabilities", []))

    @property
    def animations(self) -> list[str]:
        return list(self._welcome.get("animations", []))

    @property
    def gestures(self) -> list[str]:
        return list(self._welcome.get("gestures", []))

    @property
    def events(self) -> list[str]:
        return list(self._welcome.get("events", []))

    @property
    def joints(self) -> dict:
        """Wire joints and their normalized ranges, from ``welcome.joints``."""
        return dict(self._welcome.get("joints", {}))

    # ── Callbacks ──────────────────────────────────────────────────────

    def on(self, kind: str, callback: Callable[[Any], Any]) -> Callable[[], None]:
        """Register a callback; returns a zero-arg unsubscribe function.

        - ``on("state", cb)`` — cb(:class:`RobotState`) for every state push
          (requires ``"state"`` in ``subscribe``).
        - ``on("event", cb)`` — cb(payload dict with ``name``/``params``/
          ``origin``) for other clients' rebroadcast events (requires
          ``"events"`` in ``subscribe``).
        - ``on("disconnect", cb)`` — cb(exception or None) when the link drops.

        Callbacks may be plain functions or coroutine functions.
        """
        if kind not in self._callbacks:
            raise ValueError(f"unknown callback kind {kind!r}; use one of {_CALLBACK_KINDS}")
        self._callbacks[kind].append(callback)

        def unsubscribe() -> None:
            with contextlib.suppress(ValueError):
                self._callbacks[kind].remove(callback)

        return unsubscribe

    # ── Actions ────────────────────────────────────────────────────────

    async def act(
        self,
        step: dict,
        *,
        priority: Any = None,
        on_busy: Optional[str] = None,
        tag: Optional[str] = None,
    ) -> ActionHandle:
        """Send an ``act`` with an arbitrary Step tree (PROTOCOL.md §6–7)."""
        payload: dict[str, Any] = {"do": step}
        if priority is not None:
            payload["priority"] = priority
        if on_busy is not None:
            payload["on_busy"] = on_busy
        if tag is not None:
            payload["tag"] = tag
        desc = step.get("kind") or ("seq" if "seq" in step else "par")
        return await self._send_tracked("act", payload, desc=f"act:{desc}")

    async def emit(self, name: str, /, **params: Any) -> ActionHandle:
        """Send a semantic event (the preferred path): ``emit("word_read", word="cat")``."""
        return await self._send_tracked(
            "event", {"name": name, "params": params}, desc=f"event:{name}"
        )

    async def blink(self, duration_ms: Optional[int] = None, **opts: Any) -> ActionHandle:
        return await self.act(_action("blink", duration_ms=duration_ms), **opts)

    async def look(
        self,
        pan: float = 0.0,
        tilt: float = 0.0,
        *,
        eyes_only: Optional[bool] = None,
        duration_ms: Optional[int] = None,
        **opts: Any,
    ) -> ActionHandle:
        return await self.act(
            _action("look", pan=float(pan), tilt=float(tilt),
                    eyes_only=eyes_only, duration_ms=duration_ms),
            **opts,
        )

    async def eyelids(
        self, openness: float, *, duration_ms: Optional[int] = None, **opts: Any
    ) -> ActionHandle:
        return await self.act(
            _action("eyelids", openness=float(openness), duration_ms=duration_ms), **opts
        )

    async def mouth(
        self, openness: float, *, duration_ms: Optional[int] = None, **opts: Any
    ) -> ActionHandle:
        return await self.act(
            _action("mouth", openness=float(openness), duration_ms=duration_ms), **opts
        )

    async def pose(
        self, joints: dict, *, duration_ms: Optional[int] = None, **opts: Any
    ) -> ActionHandle:
        """Move a subset of wire joints: ``pose({"head_pan": 0.5, "eyelids": 1.0})``."""
        return await self.act(
            _action("pose", joints=dict(joints), duration_ms=duration_ms), **opts
        )

    async def gesture(
        self,
        name: str,
        *,
        repeat: Optional[int] = None,
        intensity: Optional[float] = None,
        **opts: Any,
    ) -> ActionHandle:
        return await self.act(
            _action("gesture", name=name, repeat=repeat, intensity=intensity), **opts
        )

    async def led(
        self, animation: Optional[str] = None, *, color: Any = None, **opts: Any
    ) -> ActionHandle:
        """Set a named LED animation, or a solid ``color=(r, g, b)``."""
        if (animation is None) == (color is None):
            raise ValueError("led(): pass exactly one of animation= or color=")
        if color is not None:
            if isinstance(color, (tuple, list)):
                color = {"r": int(color[0]), "g": int(color[1]), "b": int(color[2])}
            return await self.act({"kind": "led", "color": color}, **opts)
        return await self.act({"kind": "led", "animation": animation}, **opts)

    async def say(self, text: str, **opts: Any) -> ActionHandle:
        """TTS is deferred server-side: expect ``MakiActionError('tts_unavailable')``."""
        return await self.act({"kind": "say", "text": str(text)}, **opts)

    async def neutral(self, duration_ms: Optional[int] = None, **opts: Any) -> ActionHandle:
        return await self.act(_action("neutral", duration_ms=duration_ms), **opts)

    # ── Control messages ───────────────────────────────────────────────

    async def cancel(self, target: str = "all") -> dict:
        """Cancel by id, ``"all"``, or ``"tag:<x>"``; returns the completed ack."""
        handle = await self._send_tracked("cancel", {"target": target}, desc="cancel")
        return await handle.wait(self._handshake_timeout)

    async def estop(self, engage: bool = True, reason: str = "") -> dict:
        """Engage (freeze-and-hold) or disengage the e-stop; returns the ack."""
        payload: dict[str, Any] = {"engage": bool(engage)}
        if reason:
            payload["reason"] = reason
        handle = await self._send_tracked("estop", payload, desc="estop")
        return await handle.wait(self._handshake_timeout)

    async def state(self, fresh: bool = False, *, timeout: float = 5.0) -> RobotState:
        """Latest robot state. ``fresh=True`` forces a ``state.get`` round-trip;
        otherwise the last pushed snapshot is returned when available."""
        if not fresh and self._last_state is not None:
            return self._last_state
        msg_id = self._next_id()
        fut: asyncio.Future = asyncio.get_running_loop().create_future()
        self._state_waiters[msg_id] = fut
        try:
            await self._send_frame("state.get", {}, msg_id)
            state: RobotState = await asyncio.wait_for(asyncio.shield(fut), timeout)
        except asyncio.TimeoutError as exc:
            raise MakiConnectionError(f"state.get timed out after {timeout}s") from exc
        finally:
            self._state_waiters.pop(msg_id, None)
        self._last_state = state
        return state

    async def ping(self, *, timeout: float = 5.0) -> float:
        """Round-trip an application-level ping; returns latency in seconds."""
        msg_id = self._next_id()
        fut: asyncio.Future = asyncio.get_running_loop().create_future()
        self._ping_waiters[msg_id] = fut
        t0 = time.monotonic()
        try:
            await self._send_frame("ping", {}, msg_id)
            await asyncio.wait_for(asyncio.shield(fut), timeout)
        except asyncio.TimeoutError as exc:
            raise MakiConnectionError(f"ping timed out after {timeout}s") from exc
        finally:
            self._ping_waiters.pop(msg_id, None)
        return time.monotonic() - t0

    # ── Internals ──────────────────────────────────────────────────────

    def _next_id(self) -> str:
        self._counter += 1
        return f"c-{self._counter}"

    async def _send_frame(self, msg_type: str, payload: dict, msg_id: str) -> None:
        ws = self._ws
        if ws is None or not self._connected:
            raise MakiConnectionError("not connected")
        frame = {"type": msg_type, "id": msg_id, "ts": int(time.time() * 1000),
                 "payload": payload}
        try:
            await ws.send(json.dumps(frame))
        except (ConnectionClosed, OSError) as exc:
            raise MakiConnectionError(f"send failed: {exc}") from exc

    async def _send_tracked(self, msg_type: str, payload: dict, desc: str = "") -> ActionHandle:
        if self._closed:
            raise MakiConnectionError("client closed")
        msg_id = self._next_id()
        handle = ActionHandle(msg_id, desc=desc)
        self._pending[msg_id] = handle  # register before send: no ack race
        try:
            await self._send_frame(msg_type, payload, msg_id)
        except MakiError:
            self._pending.pop(msg_id, None)
            raise
        return handle

    async def _connect_once(self) -> None:
        """Open the socket and complete hello → welcome. Raises MakiError."""
        try:
            ws = await websockets.connect(self._url, open_timeout=self._handshake_timeout)
        except (OSError, WebSocketException, asyncio.TimeoutError) as exc:
            raise MakiConnectionError(f"cannot connect to {self._url}: {exc}") from exc
        try:
            hello_id = self._next_id()
            await ws.send(json.dumps({
                "type": "hello",
                "id": hello_id,
                "ts": int(time.time() * 1000),
                "payload": {
                    "protocol": PROTOCOL_VERSION,
                    "client": {"name": self._name, "kind": self._kind,
                               "version": self._version},
                    "priority": self._priority,
                    "subscribe": list(self._subscribe),
                    "abort_on_disconnect": self._abort_on_disconnect,
                },
            }))
            welcome = await asyncio.wait_for(
                self._read_welcome(ws), self._handshake_timeout
            )
        except MakiError:
            with contextlib.suppress(Exception):
                await ws.close()
            raise
        except (ConnectionClosed, OSError, asyncio.TimeoutError) as exc:
            with contextlib.suppress(Exception):
                await ws.close()
            if getattr(ws, "close_code", None) == 4400:
                raise MakiProtocolError(
                    f"server does not support MPP/{PROTOCOL_VERSION} (close 4400)"
                ) from exc
            raise MakiConnectionError(f"handshake failed: {exc}") from exc

        prev_session = self._welcome.get("session")
        if prev_session is not None and welcome.get("session") != prev_session:
            self._last_state = None  # new session voids cached state (§12.6)
        self._welcome = welcome
        self._ws = ws
        self._connected = True
        log.info("connected to %s (session %s)", self._url, welcome.get("session"))

    async def _read_welcome(self, ws: Any) -> dict:
        async for raw in ws:
            try:
                msg = json.loads(raw)
            except ValueError as exc:
                raise MakiProtocolError(f"invalid JSON during handshake: {exc}") from exc
            if not isinstance(msg, dict):
                continue
            if msg.get("type") == "welcome":
                return msg.get("payload") or {}
            payload = msg.get("payload") or {}
            if msg.get("type") == "ack" and payload.get("status") == "error":
                raise MakiProtocolError(
                    f"handshake rejected: {payload.get('code', 'internal')}: "
                    f"{payload.get('detail', '')}"
                )
            # Tolerate anything else before welcome (forward compatibility).
        raise MakiConnectionError("connection closed during handshake")

    async def _run(self) -> None:
        """Owns the connection after the initial handshake: receive, reconnect."""
        try:
            while True:
                exc: Optional[BaseException] = None
                try:
                    await self._recv_loop()
                except asyncio.CancelledError:
                    raise
                except Exception as e:  # noqa: BLE001 — any transport failure
                    exc = e
                self._connected = False
                self._ws = None
                if self._closed:
                    return
                self._fail_pending(MakiConnectionError(
                    f"connection lost: {exc}" if exc else "connection lost"
                ))
                self._dispatch("disconnect", exc)
                if not self._reconnect:
                    return
                delay = self._backoff_initial
                while not self._closed:
                    await asyncio.sleep(delay * random.uniform(0.75, 1.25))
                    try:
                        await self._connect_once()
                        break
                    except MakiError as e:
                        log.debug("reconnect to %s failed: %s", self._url, e)
                        delay = min(delay * 2, self._backoff_max)
                if self._closed:
                    return
        finally:
            self._connected = False

    async def _recv_loop(self) -> None:
        ws = self._ws
        if ws is None:
            return
        async for raw in ws:
            try:
                msg = json.loads(raw)
            except ValueError:
                log.warning("server sent invalid JSON; frame ignored")
                continue
            if isinstance(msg, dict):
                await self._handle(msg)

    async def _handle(self, msg: dict) -> None:
        msg_type = msg.get("type")
        payload = msg.get("payload") or {}
        if msg_type == "ack":
            ref = payload.get("ref")
            handle = self._pending.get(ref)
            if handle is not None:
                handle._on_ack(payload)
                if handle.done:
                    self._pending.pop(ref, None)
            waiter = self._state_waiters.get(ref)
            if (waiter is not None and not waiter.done()
                    and payload.get("status") == "error"):
                waiter.set_exception(MakiActionError(
                    str(payload.get("code", "internal")),
                    str(payload.get("detail", "")), ref=ref))
        elif msg_type == "state":
            state = RobotState.from_payload(payload)
            ref = payload.get("ref")
            waiter = self._state_waiters.get(ref) if ref else None
            if waiter is not None:
                if not waiter.done():
                    waiter.set_result(state)
                return  # a state.get reply is not a push
            self._last_state = state
            self._dispatch("state", state)
        elif msg_type == "event":
            self._dispatch("event", payload)
        elif msg_type == "ping":
            with contextlib.suppress(MakiError):
                await self._send_frame("pong", {"ref": msg.get("id")}, self._next_id())
        elif msg_type == "pong":
            waiter = self._ping_waiters.pop(payload.get("ref"), None)
            if waiter is not None and not waiter.done():
                waiter.set_result(None)
        elif msg_type == "welcome":
            self._welcome = payload  # catalog refresh; normally handshake-only
        # Unknown types are ignored (forward compatibility, PROTOCOL.md §13).

    def _dispatch(self, kind: str, arg: Any) -> None:
        for cb in list(self._callbacks[kind]):
            try:
                out = cb(arg)
                if asyncio.iscoroutine(out):
                    asyncio.ensure_future(out)
            except Exception:  # noqa: BLE001 — client callbacks must not kill the loop
                log.exception("%r callback raised", kind)

    def _fail_pending(self, exc: MakiError) -> None:
        pending, self._pending = self._pending, {}
        for handle in pending.values():
            handle._fail(exc)
        for waiters in (self._state_waiters, self._ping_waiters):
            for fut in waiters.values():
                if not fut.done():
                    fut.set_exception(exc)
            waiters.clear()
