"""MPP/1 WebSocket gateway server.

Enforces the handshake (hello-first → 4401, protocol mismatch → 4400),
routes client messages into the ActionEngine/Choreographer, forwards ack
lifecycle callbacks to the owning connection, pushes `state` (on change +
1 Hz) to subscribers, rebroadcasts semantic events, and runs the 10 s
JSON-level heartbeat (2 missed pongs → close 1001).
"""

from __future__ import annotations

import asyncio
import logging
import secrets
from dataclasses import dataclass, field
from typing import Any, Iterable, List, Mapping, Optional, Set

from websockets.asyncio.server import serve
from websockets.exceptions import ConnectionClosed

from . import protocol
from .choreography import Choreographer
from .engine import ActionEngine, Performance
from .motion import joints as joints_mod
from .protocol import (
    CLOSE_FRAME_BEFORE_HELLO,
    CLOSE_UNSUPPORTED_PROTOCOL,
    MAX_FRAME_BYTES,
    PROTOCOL_VERSION,
    Envelope,
    ProtocolError,
    make_envelope,
)

log = logging.getLogger(__name__)

SERVER_NAME = "maki-puppet"
SERVER_VERSION = "0.1.0"
DEFAULT_PING_INTERVAL_S = 10.0
STATE_HEARTBEAT_S = 1.0


@dataclass
class ClientConn:
    """One connected, handshaken client."""

    ws: Any
    name: str = ""
    kind: str = ""
    version: str = ""
    priority: int = 50
    subscribe: Set[str] = field(default_factory=set)
    abort_on_disconnect: bool = False
    outbox: asyncio.Queue = field(default_factory=asyncio.Queue)
    awaiting_pong: bool = False
    alive: bool = True
    _sid: int = 0

    def next_sid(self) -> str:
        self._sid += 1
        return f"s-{self._sid}"

    def enqueue(self, text: str) -> None:
        if self.alive:
            self.outbox.put_nowait(text)


class GatewayServer:
    def __init__(
        self,
        config: Mapping[str, Any],
        *,
        engine: ActionEngine,
        motion: Any,
        led: Any,
        choreographer: Choreographer,
        animations: Iterable[str],
        capabilities: Iterable[str] = ("motion", "mouth", "led"),
        sim: bool = False,
        motion_rate_hz: float = 50.0,
        bridge: Any = None,
    ) -> None:
        cfg = dict(config or {})
        self.host = str(cfg.get("host", "0.0.0.0"))
        self.port = int(cfg.get("port", 8765))
        self._ping_interval = float(cfg.get("ping_interval_s", DEFAULT_PING_INTERVAL_S))
        self._engine = engine
        self._motion = motion
        self._led = led
        self._choreo = choreographer
        self._bridge = bridge
        self._animations = list(animations)
        self._capabilities = list(capabilities)
        self._sim = sim
        self._motion_rate_hz = float(motion_rate_hz)

        self.session = secrets.token_hex(3)
        self._conns: List[ClientConn] = []
        self._server: Any = None
        self._push_task: Optional[asyncio.Task] = None
        self._dirty = asyncio.Event()

        # Wire the engine's lifecycle callbacks to this server.
        self._engine.ack_cb = self.on_engine_ack
        self._engine.state_changed_cb = self.mark_dirty
        self._choreo.on_reload = self.mark_dirty

    # ── Lifecycle ──────────────────────────────────────────────────────

    async def start(self) -> None:
        self._server = await serve(
            self._handler,
            self.host,
            self.port,
            max_size=MAX_FRAME_BYTES,   # oversize frames close 1009
            ping_interval=None,         # heartbeat is JSON-level (§4.3)
        )
        # Resolve the actual bound port (port 0 → ephemeral, used by tests)
        for sock in self._server.sockets:
            self.port = sock.getsockname()[1]
            break
        self._push_task = asyncio.ensure_future(self._push_loop())
        log.info("MPP/1 gateway listening on ws://%s:%d/ws", self.host, self.port)

    async def stop(self) -> None:
        if self._push_task is not None:
            self._push_task.cancel()
            self._push_task = None
        for conn in list(self._conns):
            try:
                await conn.ws.close(1001, "server shutting down")
            except Exception:
                pass
        if self._server is not None:
            self._server.close()
            await self._server.wait_closed()
            self._server = None

    # ── Engine / choreography callbacks ───────────────────────────────

    def on_engine_ack(self, perf: Performance, status: str, fields: dict) -> None:
        conn = perf.client
        if not isinstance(conn, ClientConn) or not conn.alive:
            return
        payload = {"ref": perf.id, "status": status, **fields}
        self._send(conn, "ack", payload)

    def mark_dirty(self) -> None:
        self._dirty.set()

    # ── Connection handling ────────────────────────────────────────────

    async def _handler(self, ws: Any) -> None:
        conn = ClientConn(ws=ws)
        writer = asyncio.ensure_future(self._writer(conn))
        heartbeat: Optional[asyncio.Task] = None
        try:
            if not await self._handshake(conn):
                return
            self._conns.append(conn)
            self.mark_dirty()
            heartbeat = asyncio.ensure_future(self._heartbeat(conn))
            async for raw in ws:
                await self._dispatch(conn, raw)
        except ConnectionClosed:
            pass
        finally:
            conn.alive = False
            if heartbeat is not None:
                heartbeat.cancel()
            writer.cancel()
            if conn in self._conns:
                self._conns.remove(conn)
                self._engine.release_client(conn, abort=conn.abort_on_disconnect)
                self.mark_dirty()

    async def _handshake(self, conn: ClientConn) -> bool:
        """First frame must be a valid hello (§4.1). Handshake errors are
        sent directly (awaited) so they flush before the close frame."""
        try:
            raw = await conn.ws.recv()
        except ConnectionClosed:
            return False
        try:
            env = protocol.parse_envelope(raw)
        except ProtocolError as e:
            await self._send_direct_error(conn, e.ref, protocol.E_BAD_JSON, e.detail)
            await conn.ws.close(CLOSE_FRAME_BEFORE_HELLO, "hello required")
            return False
        if env.type != "hello":
            await conn.ws.close(CLOSE_FRAME_BEFORE_HELLO, "first frame must be hello")
            return False
        try:
            hello = protocol.parse_hello(env.payload)
        except ProtocolError as e:
            await self._send_direct_error(conn, env.id, e.code, e.detail)
            await conn.ws.close(CLOSE_FRAME_BEFORE_HELLO, "invalid hello")
            return False
        if hello.protocol != PROTOCOL_VERSION:
            await self._send_direct_error(
                conn, env.id, protocol.E_UNSUPPORTED_PROTOCOL,
                f"server speaks protocol {PROTOCOL_VERSION}",
            )
            await conn.ws.close(CLOSE_UNSUPPORTED_PROTOCOL, "unsupported protocol")
            return False
        conn.name = hello.name
        conn.kind = hello.kind
        conn.version = hello.version
        conn.priority = hello.priority
        conn.subscribe = set(hello.subscribe)
        conn.abort_on_disconnect = hello.abort_on_disconnect
        self._send(conn, "welcome", self._welcome_payload(env.id))
        log.info("client connected: %s (%s)", conn.name, conn.kind)
        return True

    def _welcome_payload(self, ref: str) -> dict:
        return {
            "ref": ref,
            "protocol": PROTOCOL_VERSION,
            "server": {"name": SERVER_NAME, "version": SERVER_VERSION},
            "session": self.session,
            "capabilities": list(self._capabilities),
            "joints": protocol.wire_joints_catalog(),
            "animations": list(self._animations),
            "gestures": self._choreo.gesture_names,
            "events": self._event_catalog(),
        }

    def _event_catalog(self) -> List[str]:
        """Choreography events plus app-bridge events (deduped, order kept)."""
        names = list(self._choreo.event_names)
        if self._bridge is not None:
            names += [n for n in self._bridge.event_names if n not in names]
        return names

    async def _writer(self, conn: ClientConn) -> None:
        try:
            while True:
                msg = await conn.outbox.get()
                await conn.ws.send(msg)
        except (ConnectionClosed, asyncio.CancelledError):
            pass

    async def _send_direct_error(self, conn: ClientConn, ref: Optional[str],
                                 code: str, detail: str = "") -> None:
        payload: dict = {"ref": ref, "status": "error", "code": code}
        if detail:
            payload["detail"] = detail
        try:
            await conn.ws.send(make_envelope("ack", conn.next_sid(), payload).to_json())
        except ConnectionClosed:
            pass

    async def _heartbeat(self, conn: ClientConn) -> None:
        missed = 0
        try:
            while True:
                await asyncio.sleep(self._ping_interval)
                if conn.awaiting_pong:
                    missed += 1
                else:
                    missed = 0
                if missed >= 2:  # two consecutive missed pongs → 1001 (§4.3)
                    log.warning("client %s missed 2 pongs; closing", conn.name)
                    await conn.ws.close(1001, "heartbeat timeout")
                    return
                conn.awaiting_pong = True
                self._send(conn, "ping", {})
        except (ConnectionClosed, asyncio.CancelledError):
            pass

    # ── Message routing ────────────────────────────────────────────────

    async def _dispatch(self, conn: ClientConn, raw: Any) -> None:
        try:
            env = protocol.parse_envelope(raw)
        except ProtocolError as e:
            self._send_ack_error(conn, e.ref, e.code, e.detail)
            return
        try:
            self._route(conn, env)
        except ProtocolError as e:
            self._send_ack_error(conn, env.id, e.code, e.detail)
        except Exception as e:  # pragma: no cover - defensive
            log.exception("error handling %s from %s", env.type, conn.name)
            self._send_ack_error(conn, env.id, protocol.E_INTERNAL, str(e))

    def _route(self, conn: ClientConn, env: Envelope) -> None:
        t = env.type
        if t == "act":
            self._handle_act(conn, env)
        elif t == "event":
            self._handle_event(conn, env)
        elif t == "mouth":
            self._handle_mouth(conn, env)
        elif t == "cancel":
            msg = protocol.parse_cancel(env.payload)
            n = self._engine.cancel(msg.target, client=conn)
            detail = f"cancelled {n}" if n else "no-op"
            self._send_ack(conn, env.id, "completed", detail=detail)
        elif t == "estop":
            msg = protocol.parse_estop(env.payload)
            self._engine.set_estop(msg.engage, msg.reason)
            self._send_ack(conn, env.id, "completed")
        elif t == "state.get":
            msg = protocol.parse_state_get(env.payload)
            self._send(conn, "state", self._state_payload(msg.fields, ref=env.id))
        elif t == "lock":
            msg = protocol.parse_lock(env.payload)
            self._engine.lock(conn, msg.scopes, msg.ttl_s)
            self._send_ack(conn, env.id, "completed")
        elif t == "unlock":
            msg = protocol.parse_unlock(env.payload)
            self._engine.unlock(conn, msg.scopes)
            self._send_ack(conn, env.id, "completed")
        elif t == "ping":
            self._send(conn, "pong", {"ref": env.id})
        elif t == "pong":
            conn.awaiting_pong = False
        elif t == "hello":
            raise ProtocolError(
                protocol.E_UNKNOWN_TYPE, "hello only valid as the first frame"
            )
        else:
            raise ProtocolError(protocol.E_UNKNOWN_TYPE, f"unknown type '{t}'")

    def _handle_act(self, conn: ClientConn, env: Envelope) -> None:
        act = protocol.parse_act(
            env.payload,
            animations=self._animations,
            gestures=self._choreo.gesture_names,
        )
        if "tts" not in self._capabilities and protocol.step_contains_kind(
            act.step, "say"
        ):
            raise ProtocolError(
                protocol.E_TTS_UNAVAILABLE, "tts is not in this build's capabilities"
            )
        # Same shape as the tts gate: reject the whole act up front rather than
        # letting a `track` step fail partway through a performance that has
        # already started moving the robot.
        if "vision" not in self._capabilities and protocol.step_contains_kind(
            act.step, "track"
        ):
            raise ProtocolError(
                protocol.E_VISION_UNAVAILABLE,
                "face tracking is not in this build's capabilities",
            )
        perf = Performance(
            id=env.id,
            client=conn,
            step=act.step,
            priority=act.priority if act.priority is not None else conn.priority,
            on_busy=act.on_busy,
            tag=act.tag,
        )
        self._engine.submit(perf)

    def _handle_event(self, conn: ClientConn, env: Envelope) -> None:
        msg = protocol.parse_event(env.payload)
        log.info("event %s from %s %s", msg.name, conn.name, msg.params or "")
        if self._bridge is not None and self._bridge.handles(msg.name):
            # App events route to the bridge (platform-channel style); bridge
            # names shadow same-named choreographies.
            asyncio.ensure_future(
                self._run_bridge_event(conn, env.id, msg.name, msg.params)
            )
            self._rebroadcast_event(conn, msg.name, msg.params)
            return
        resolved = self._choreo.resolve(msg.name, msg.params)  # may raise
        if resolved is None:  # ignored event → completed/no-op (§5.4)
            self._send_ack(conn, env.id, "completed", detail="no-op")
            self._rebroadcast_event(conn, msg.name, msg.params)
            return
        perf = Performance(
            id=env.id,
            client=conn,
            step=resolved.step,
            priority=resolved.priority,
            on_busy=resolved.on_busy,
            choreography=resolved.name,
        )
        disposition = self._engine.submit(perf)
        log.info("event %s → choreography %s (%s)", msg.name, resolved.name, disposition)
        if disposition != "dropped":
            self._rebroadcast_event(conn, msg.name, msg.params)

    def _handle_mouth(self, conn: ClientConn, env: Envelope) -> None:
        """One viseme sample → straight onto the expression layer (§5.13).

        This deliberately bypasses the action engine: at 30–60 Hz a
        Performance per sample would thrash the queue, and mouth motion must
        not arbitrate against head gestures. The expression layer merges
        targets, so a concurrent blink keeps its eyelid targets.

        No ack — the sender is streaming and cannot act on one. E-stop is
        still honoured downstream: MotionService freezes output while
        engaged, so layer writes have no effect.
        """
        msg = protocol.parse_mouth(env.payload)
        self._motion.set_layer(
            "expression", {"mouth": joints_mod.mouth_openness_to_rad(msg.openness)}
        )

    async def _run_bridge_event(self, conn: ClientConn, ref: str,
                                name: str, params: dict) -> None:
        """Run one bridge handler and ack its outcome to the sender."""
        try:
            await self._bridge.dispatch(name, params)
        except ProtocolError as e:
            self._send_ack_error(conn, ref, e.code, e.detail)
        except Exception as e:  # a broken handler must not kill the server
            log.exception("bridge handler on_%s failed", name)
            self._send_ack_error(conn, ref, protocol.E_INTERNAL, str(e))
        else:
            self._send_ack(conn, ref, "completed", detail="bridge")

    def _rebroadcast_event(self, origin: ClientConn, name: str, params: dict) -> None:
        payload = {"name": name, "params": params, "origin": origin.name}
        for conn in self._conns:
            if conn is not origin and "events" in conn.subscribe:
                self._send(conn, "event", payload)

    # ── Outbound helpers ───────────────────────────────────────────────

    def _send(self, conn: ClientConn, type_: str, payload: dict) -> None:
        conn.enqueue(make_envelope(type_, conn.next_sid(), payload).to_json())

    def _send_ack(self, conn: ClientConn, ref: Optional[str], status: str,
                  **fields: Any) -> None:
        self._send(conn, "ack", {"ref": ref, "status": status, **fields})

    def _send_ack_error(self, conn: ClientConn, ref: Optional[str], code: str,
                        detail: str = "") -> None:
        payload: dict = {"ref": ref, "status": "error", "code": code}
        if detail:
            payload["detail"] = detail
        self._send(conn, "ack", payload)

    # ── State pushes ───────────────────────────────────────────────────

    def _state_payload(self, fields: Optional[tuple] = None,
                       ref: Optional[str] = None) -> dict:
        full = self.build_state()
        if fields is None:
            payload = full
        else:
            payload = {k: full[k] for k in fields if k in full}
            if "catalog" in fields:  # pseudo-field (§5.8)
                payload["catalog"] = {
                    "animations": list(self._animations),
                    "gestures": self._choreo.gesture_names,
                    "events": self._event_catalog(),
                }
        if ref is not None:
            payload = {"ref": ref, **payload}
        return payload

    def build_state(self) -> dict:
        return {
            "pose": self._wire_pose(),
            "led": self._led.current,
            "queue": self._engine.queue_snapshot(),
            "estop": self._engine.estopped,
            "lock": self._engine.lock_snapshot(),
            "clients": [{"name": c.name, "kind": c.kind} for c in self._conns],
            "health": {
                "servo": "sim" if self._sim else "up",
                "led": "sim" if self._sim else "up",
                "vision": self._vision_health(),
                "loop_hz": self._motion_rate_hz,
            },
        }

    def _vision_health(self) -> str:
        """"off" when no camera is configured, "sim" / "up" / "down" otherwise.

        "down" means vision was configured but its thread is not running — a
        camera that failed to open or died — which is worth surfacing rather
        than reporting as simply absent.
        """
        vision = getattr(self._engine, "vision", None)
        if vision is None:
            return "off"
        if not vision.running:
            return "down"
        return "sim" if self._sim else "up"

    def _wire_pose(self) -> dict:
        pose_rad = self._motion.pose_rad()
        pose: dict = {}
        for j in ("head_pan", "head_tilt", "eyes_pan", "eyes_tilt"):
            rad = pose_rad.get(j, joints_mod.JOINTS[j].neutral_rad)
            pose[j] = round(joints_mod.rad_to_norm(j, rad), 4)
        lid_vals = [
            joints_mod.eyelids_rad_to_openness(pose_rad[j])
            for j in ("left_eyelid", "right_eyelid")
            if j in pose_rad
        ]
        pose["eyelids"] = round(sum(lid_vals) / len(lid_vals), 4) if lid_vals else 1.0
        if "mouth" in pose_rad:
            pose["mouth"] = round(joints_mod.mouth_rad_to_openness(pose_rad["mouth"]), 4)
        else:
            pose["mouth"] = 0.0
        return pose

    async def _push_loop(self) -> None:
        """Push state to subscribers on change and as a 1 Hz heartbeat."""
        while True:
            try:
                await asyncio.wait_for(self._dirty.wait(), timeout=STATE_HEARTBEAT_S)
            except asyncio.TimeoutError:
                pass
            self._dirty.clear()
            if not any("state" in c.subscribe for c in self._conns):
                continue
            payload = self._state_payload()
            for conn in self._conns:
                if "state" in conn.subscribe:
                    self._send(conn, "state", payload)
