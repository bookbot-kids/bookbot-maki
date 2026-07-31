"""Tests for the Python client SDK (clients/python/maki_client).

Runs the SDK against a minimal in-test stub gateway built on `websockets`.
The stub answers `hello` with the `welcome` fixture from tests/fixtures/
(the shared anti-drift corpus); per-test behavior for everything after the
handshake is injected via a `respond` coroutine.

The SDK is pure websockets + stdlib and is deliberately NOT imported through
maki_puppet — these tests exercise it exactly as an external client would.
"""

from __future__ import annotations

import asyncio
import json
import sys
import time
from pathlib import Path

import pytest
import websockets

_SDK_DIR = Path(__file__).resolve().parents[1] / "clients" / "python"
if str(_SDK_DIR) not in sys.path:
    sys.path.insert(0, str(_SDK_DIR))

from maki_client import (  # noqa: E402
    ActionHandle,
    Maki,
    MakiActionError,
    MakiCancelled,
    MakiConnectionError,
    par,
    seq,
)

FIXTURES = Path(__file__).resolve().parent / "fixtures"


def load_fixture(name: str) -> dict:
    return json.loads((FIXTURES / f"{name}.json").read_text())


WELCOME_PAYLOAD = load_fixture("welcome")["payload"]


def frame(msg_type: str, msg_id: str, payload: dict) -> str:
    return json.dumps({"type": msg_type, "id": msg_id, "ts": 0, "payload": payload})


class StubServer:
    """Minimal MPP/1 gateway double.

    Answers every `hello` with the welcome fixture (ref + per-connection
    session set); `pong` frames are recorded but otherwise ignored; all other
    post-handshake frames are delegated to the injected `respond(server, ws,
    msg)` coroutine.
    """

    def __init__(self, respond=None):
        self.respond = respond
        self.connections = 0
        self.received: list[dict] = []
        self.hellos: list[dict] = []
        self.last_ws = None
        self.url = ""
        self._server = None
        self._sid = 0

    def next_sid(self) -> str:
        self._sid += 1
        return f"s-{self._sid}"

    def ack(self, ref: str, status: str, **extra) -> str:
        return frame("ack", self.next_sid(), {"ref": ref, "status": status, **extra})

    async def __aenter__(self) -> "StubServer":
        self._server = await websockets.serve(self._handler, "127.0.0.1", 0)
        port = self._server.sockets[0].getsockname()[1]
        self.url = f"ws://127.0.0.1:{port}/ws"
        return self

    async def __aexit__(self, *exc_info) -> None:
        self._server.close()
        await self._server.wait_closed()

    async def _handler(self, ws) -> None:
        self.connections += 1
        self.last_ws = ws
        session = f"sess-{self.connections}"
        try:
            async for raw in ws:
                msg = json.loads(raw)
                self.received.append(msg)
                if msg["type"] == "hello":
                    self.hellos.append(msg)
                    payload = dict(WELCOME_PAYLOAD, ref=msg["id"], session=session)
                    await ws.send(frame("welcome", self.next_sid(), payload))
                elif msg["type"] == "pong":
                    continue
                elif self.respond is not None:
                    await self.respond(self, ws, msg)
        except websockets.exceptions.ConnectionClosed:
            pass

    def frames_of_type(self, msg_type: str) -> list[dict]:
        return [f for f in self.received if f["type"] == msg_type]


async def _wait_until(predicate, timeout: float = 5.0) -> None:
    deadline = time.monotonic() + timeout
    while not predicate():
        assert time.monotonic() < deadline, "timed out waiting for condition"
        await asyncio.sleep(0.01)


# ── Handshake ──────────────────────────────────────────────────────────────


async def test_handshake_populates_catalogs_from_welcome():
    async with StubServer() as srv:
        m = await Maki.connect(srv.url, name="test-client", reconnect=False)
        try:
            assert m.connected
            assert m.session == "sess-1"
            assert m.capabilities == ["motion", "led"]  # tts deferred
            assert "breathing_cyan" in m.animations
            assert "alarm_red" in m.animations
            assert m.gestures == ["nod", "head_shake", "happy_wiggle",
                                  "curious_tilt", "wake_up", "sleepy"]
            assert "celebrate" in m.events and "word_read" in m.events
            assert set(m.joints) == {"head_pan", "head_tilt", "eyes_pan",
                                     "eyes_tilt", "eyelids", "mouth"}
            assert m.joints["eyelids"] == {"min": 0.0, "max": 1.0, "neutral": 1.0}

            hello = srv.hellos[0]["payload"]
            assert hello["protocol"] == 1
            assert hello["client"] == {"name": "test-client", "kind": "python",
                                       "version": "0.1.0"}
            assert hello["priority"] == 50
            assert hello["subscribe"] == ["state"]
            assert hello["abort_on_disconnect"] is False
        finally:
            await m.close()
        assert not m.connected


async def test_context_manager_closes():
    async with StubServer() as srv:
        async with await Maki.connect(srv.url, reconnect=False) as m:
            assert m.connected
        assert not m.connected


# ── Ack lifecycle → handle resolution ──────────────────────────────────────


async def test_emit_resolves_on_completed_ack():
    async def respond(srv, ws, msg):
        if msg["type"] == "event":
            await ws.send(srv.ack(msg["id"], "accepted", channels=["motion"]))
            await ws.send(srv.ack(msg["id"], "started"))
            await ws.send(srv.ack(msg["id"], "completed",
                                  choreography=msg["payload"]["name"],
                                  duration_ms=42))

    async with StubServer(respond) as srv:
        async with await Maki.connect(srv.url, reconnect=False) as m:
            handle = await m.emit("celebrate", streak=3)
            assert isinstance(handle, ActionHandle)
            result = await asyncio.wait_for(handle.wait(), 5)
            assert handle.status == "completed"
            assert handle.done
            assert result["choreography"] == "celebrate"
            assert result["duration_ms"] == 42
            # full lifecycle observed, in order
            assert [a["status"] for a in handle.acks] == ["accepted", "started",
                                                          "completed"]
            # the wire frame carried name + params per PROTOCOL.md §5.4
            sent = srv.frames_of_type("event")[0]
            assert sent["payload"] == {"name": "celebrate", "params": {"streak": 3}}


async def test_error_ack_raises_action_error():
    error_fixture = load_fixture("ack_error")["payload"]

    async def respond(srv, ws, msg):
        if msg["type"] == "act":
            await ws.send(srv.ack(msg["id"], "error",
                                  code=error_fixture["code"],
                                  detail=error_fixture["detail"]))

    async with StubServer(respond) as srv:
        async with await Maki.connect(srv.url, reconnect=False) as m:
            handle = await m.led("sparkle_red")
            with pytest.raises(MakiActionError) as excinfo:
                await asyncio.wait_for(handle.wait(), 5)
            assert excinfo.value.code == "unknown_animation"
            assert "sparkle_red" in excinfo.value.detail
            assert handle.status == "error"


async def test_cancelled_and_dropped_raise_maki_cancelled():
    async def respond(srv, ws, msg):
        if msg["type"] != "act":
            return
        kind = msg["payload"]["do"].get("kind")
        if kind == "blink":
            await ws.send(srv.ack(msg["id"], "accepted", channels=["motion"]))
            await ws.send(srv.ack(msg["id"], "cancelled"))
        else:
            await ws.send(srv.ack(msg["id"], "dropped"))

    async with StubServer(respond) as srv:
        async with await Maki.connect(srv.url, reconnect=False) as m:
            handle = await m.blink()
            with pytest.raises(MakiCancelled) as excinfo:
                await asyncio.wait_for(handle.wait(), 5)
            assert excinfo.value.status == "cancelled"

            handle = await m.neutral(on_busy="drop")
            with pytest.raises(MakiCancelled) as excinfo:
                await asyncio.wait_for(handle.wait(), 5)
            assert excinfo.value.status == "dropped"
            assert handle.status == "dropped"


async def test_act_wire_shape_matches_fixture():
    """m.act() must emit exactly the payload shape of act_simple.json."""
    expected = load_fixture("act_simple")["payload"]

    async def respond(srv, ws, msg):
        if msg["type"] == "act":
            await ws.send(srv.ack(msg["id"], "completed"))

    async with StubServer(respond) as srv:
        async with await Maki.connect(srv.url, reconnect=False) as m:
            handle = await m.act({"kind": "blink"}, priority="normal",
                                 on_busy="queue", tag="ui-idle")
            await asyncio.wait_for(handle.wait(), 5)
            sent = srv.frames_of_type("act")[0]
            assert sent["payload"] == expected
            assert sent["id"].startswith("c-")
            assert isinstance(sent["ts"], int)


def test_seq_par_step_builders():
    step = seq({"kind": "blink"},
               par({"kind": "led", "animation": "attention_sweep"},
                   {"kind": "gesture", "name": "nod"}))
    assert step == {"seq": [
        {"kind": "blink"},
        {"par": [{"kind": "led", "animation": "attention_sweep"},
                 {"kind": "gesture", "name": "nod"}]},
    ]}
    with pytest.raises(ValueError):
        seq()
    with pytest.raises(ValueError):
        par()


# ── state / heartbeat ──────────────────────────────────────────────────────


async def test_state_fresh_roundtrips_state_get():
    state_fixture = load_fixture("state")["payload"]

    async def respond(srv, ws, msg):
        if msg["type"] == "state.get":
            await ws.send(frame("state", srv.next_sid(),
                                dict(state_fixture, ref=msg["id"])))

    async with StubServer(respond) as srv:
        async with await Maki.connect(srv.url, reconnect=False) as m:
            state = await m.state(fresh=True)
            assert state.pose["head_pan"] == 0.12
            assert state.led == {"animation": "breathing_cyan"}
            assert state.estop is False
            assert state.queue["depth"] == 2
            assert state.health["loop_hz"] == 49.9
            # cached copy now serves the non-fresh path without a round-trip
            assert await m.state() is state


async def test_state_push_fires_callback_and_updates_cache():
    async with StubServer() as srv:
        async with await Maki.connect(srv.url, reconnect=False) as m:
            seen = []
            m.on("state", seen.append)
            payload = load_fixture("state")["payload"]
            await srv.last_ws.send(frame("state", srv.next_sid(), payload))
            await _wait_until(lambda: seen)
            assert seen[0].pose["eyes_tilt"] == 0.1
            assert (await m.state()).raw == payload


async def test_server_ping_answered_with_pong():
    async with StubServer() as srv:
        async with await Maki.connect(srv.url, reconnect=False) as m:
            assert m.connected
            ping_id = srv.next_sid()
            await srv.last_ws.send(frame("ping", ping_id, {}))
            await _wait_until(lambda: srv.frames_of_type("pong"))
            pong = srv.frames_of_type("pong")[0]
            assert pong["payload"]["ref"] == ping_id


async def test_event_rebroadcast_fires_event_callback():
    async with StubServer() as srv:
        async with await Maki.connect(srv.url, reconnect=False,
                                      subscribe=("state", "events")) as m:
            seen = []
            m.on("event", seen.append)
            rebroadcast = load_fixture("event_rebroadcast")
            await srv.last_ws.send(json.dumps(rebroadcast))
            await _wait_until(lambda: seen)
            assert seen[0]["name"] == "word_read"
            assert seen[0]["origin"] == "bookbot-flutter"


# ── Reconnect ──────────────────────────────────────────────────────────────


async def test_reconnect_after_drop_fails_outstanding_then_recovers():
    async def respond(srv, ws, msg):
        if msg["type"] != "act":
            return
        if srv.connections == 1:
            # ack accepted, then drop the link without a terminal ack
            await ws.send(srv.ack(msg["id"], "accepted", channels=["motion"]))
            await ws.close(code=1011)
        else:
            await ws.send(srv.ack(msg["id"], "accepted", channels=["motion"]))
            await ws.send(srv.ack(msg["id"], "completed"))

    async with StubServer(respond) as srv:
        m = await Maki.connect(srv.url, reconnect=True,
                               backoff_initial=0.05, backoff_max=0.2)
        try:
            drops = []
            m.on("disconnect", drops.append)

            handle = await m.blink()
            with pytest.raises(MakiConnectionError):
                await asyncio.wait_for(handle.wait(), 5)

            # the client re-helloed on a new connection...
            await _wait_until(lambda: len(srv.hellos) >= 2 and m.connected)
            assert srv.connections >= 2
            assert m.session == "sess-2"  # new session picked up from welcome
            assert drops, "disconnect callback did not fire"

            # ...and is fully usable again
            handle = await m.blink()
            result = await asyncio.wait_for(handle.wait(), 5)
            assert result["status"] == "completed"
            assert handle.status == "completed"
        finally:
            await m.close()


async def test_no_reconnect_when_disabled():
    async def respond(srv, ws, msg):
        if msg["type"] == "act":
            await ws.close(code=1011)

    async with StubServer(respond) as srv:
        m = await Maki.connect(srv.url, reconnect=False)
        try:
            handle = await m.blink()
            with pytest.raises(MakiConnectionError):
                await asyncio.wait_for(handle.wait(), 5)
            await asyncio.sleep(0.2)
            assert srv.connections == 1  # never re-helloed
            assert not m.connected
            with pytest.raises(MakiConnectionError):
                await m.blink()
        finally:
            await m.close()


async def test_connect_refused_raises_connection_error():
    # Grab a port with no listener.
    async with StubServer() as srv:
        url = srv.url
    with pytest.raises(MakiConnectionError):
        await Maki.connect(url, reconnect=False, handshake_timeout=2.0)
