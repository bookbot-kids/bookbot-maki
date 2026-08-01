"""End-to-end gateway tests: real websockets client against the full sim
stack (SimServoBus + SimLedRing + MotionService + engine + choreographies)
on an ephemeral port."""

from __future__ import annotations

import asyncio
import json
from pathlib import Path

import pytest
import websockets
from websockets.exceptions import ConnectionClosed

from maki_puppet import protocol
from maki_puppet.app import PuppetApp

pytestmark = pytest.mark.asyncio

CONFIG = Path(__file__).parent.parent / "config" / "puppet.yaml"

OVERRIDES = {
    "server": {"host": "127.0.0.1", "port": 0},
    "idle": {"delay_s": 300},   # keep idle blinks out of arbitration tests
    # Skip the wake/sleep flourishes: they would add ~4.5 s to every fixture.
    # Covered explicitly in test_boot_sequences.py.
    "boot": {"startup": {"duration_s": 0.0}, "shutdown": {"duration_s": 0.0}},
    # Vision ships enabled, but most tests here don't want a capture thread
    # running; `vision_app` opts back in for the ones that do.
    "vision": {"enabled": False},
}


@pytest.fixture
async def app():
    app = PuppetApp(CONFIG, sim=True, config_overrides=OVERRIDES)
    await app.start()
    yield app
    await app.stop()


@pytest.fixture
async def vision_app():
    """The same stack with the sim camera enabled, so `track` is available."""
    app = PuppetApp(
        CONFIG, sim=True,
        config_overrides={**OVERRIDES, "vision": {"enabled": True}},
    )
    await app.start()
    yield app
    await app.stop()


class WsClient:
    """Minimal MPP/1 test client."""

    def __init__(self, ws):
        self.ws = ws
        self._n = 0
        self._buffer: list = []   # frames read but not yet matched

    async def send(self, type_: str, payload: dict) -> str:
        self._n += 1
        mid = f"c-{self._n}"
        await self.ws.send(json.dumps(
            {"type": type_, "id": mid, "ts": protocol.now_ms(), "payload": payload}
        ))
        return mid

    async def recv(self, *, type_=None, ref=None, timeout=5.0) -> dict:
        """Next frame matching the filter; unmatched frames are buffered so
        interleaved acks for other requests are never lost."""

        def matches(msg):
            if type_ is not None and msg["type"] != type_:
                return False
            if ref is not None and msg["payload"].get("ref") != ref:
                return False
            return True

        for i, msg in enumerate(self._buffer):
            if matches(msg):
                return self._buffer.pop(i)
        deadline = asyncio.get_event_loop().time() + timeout
        while True:
            remaining = deadline - asyncio.get_event_loop().time()
            raw = await asyncio.wait_for(self.ws.recv(), timeout=max(0.01, remaining))
            msg = json.loads(raw)
            if matches(msg):
                return msg
            self._buffer.append(msg)

    async def hello(self, name="pytest", *, subscribe=(), priority=50, protocol_v=1):
        await self.send("hello", {
            "protocol": protocol_v,
            "client": {"name": name, "kind": "python", "version": "0"},
            "priority": priority,
            "subscribe": list(subscribe),
        })
        return await self.recv(type_="welcome")

    async def acks_until_terminal(self, ref, timeout=8.0) -> list:
        terminal = {"completed", "cancelled", "superseded", "dropped", "error"}
        acks = []
        while True:
            msg = await self.recv(type_="ack", ref=ref, timeout=timeout)
            acks.append(msg["payload"])
            if msg["payload"]["status"] in terminal:
                return acks

    async def state_get(self, fields=None) -> dict:
        payload = {} if fields is None else {"fields": list(fields)}
        mid = await self.send("state.get", payload)
        return (await self.recv(type_="state", ref=mid))["payload"]


async def connect(app) -> WsClient:
    ws = await websockets.connect(f"ws://127.0.0.1:{app.server.port}/ws")
    return WsClient(ws)


# ── Handshake ───────────────────────────────────────────────────────────────


async def test_handshake_welcome_catalogs(app):
    c = await connect(app)
    welcome = await c.hello()
    p = welcome["payload"]
    assert p["protocol"] == 1
    assert p["capabilities"] == ["motion", "mouth", "led"]   # tts deferred
    assert p["joints"] == protocol.wire_joints_catalog()
    assert "breathing_cyan" in p["animations"] and "alarm_red" in p["animations"]
    assert set(p["gestures"]) == {
        "nod", "head_shake", "happy_wiggle", "curious_tilt", "wake_up", "sleepy",
    }
    assert "celebrate" in p["events"] and "sentence_read" in p["events"]
    assert p["session"] == app.server.session
    await c.ws.close()


async def test_frame_before_hello_closes_4401(app):
    c = await connect(app)
    await c.send("act", {"do": {"kind": "blink"}})
    with pytest.raises(ConnectionClosed) as ei:
        await asyncio.wait_for(c.ws.recv(), timeout=3)
    assert ei.value.rcvd.code == 4401


async def test_unsupported_protocol_closes_4400(app):
    c = await connect(app)
    await c.send("hello", {
        "protocol": 99, "client": {"name": "x", "kind": "debug"},
    })
    msg = await c.recv(type_="ack")
    assert msg["payload"]["status"] == "error"
    assert msg["payload"]["code"] == "unsupported_protocol"
    with pytest.raises(ConnectionClosed) as ei:
        await asyncio.wait_for(c.ws.recv(), timeout=3)
    assert ei.value.rcvd.code == 4400


# ── Events ──────────────────────────────────────────────────────────────────


async def test_event_lifecycle_word_read(app):
    c = await connect(app)
    await c.hello()
    ref = await c.send("event", {"name": "word_read", "params": {"word": "cat"}})
    acks = await c.acks_until_terminal(ref)
    assert [a["status"] for a in acks] == ["accepted", "started", "completed"]
    assert acks[0]["channels"] == ["motion"]
    assert acks[-1]["choreography"] == "word_read"
    assert "duration_ms" in acks[-1]
    await c.ws.close()


async def test_event_celebrate_drives_leds(app):
    c = await connect(app)
    await c.hello()
    ref = await c.send("event", {"name": "celebrate"})
    acks = await c.acks_until_terminal(ref, timeout=15.0)
    assert acks[-1]["status"] == "completed"
    assert acks[0]["channels"] == ["motion", "led"]
    # choreography ends by restoring the resting animation
    assert app.led.current == {"animation": app.config["led"]["default_animation"]}
    await c.ws.close()


async def test_unknown_event(app):
    c = await connect(app)
    await c.hello()
    ref = await c.send("event", {"name": "moonwalk"})
    acks = await c.acks_until_terminal(ref)
    assert acks[-1]["status"] == "error"
    assert acks[-1]["code"] == "unknown_event"
    assert len(acks) == 1          # single terminal ack
    await c.ws.close()


async def test_ignored_event_is_noop_completed(app):
    c = await connect(app)
    await c.hello()
    ref = await c.send("event", {"name": "sentence_read"})
    acks = await c.acks_until_terminal(ref)
    assert len(acks) == 1
    assert acks[0]["status"] == "completed"
    assert acks[0]["detail"] == "no-op"
    await c.ws.close()


async def test_event_rebroadcast_with_origin(app):
    talker = await connect(app)
    await talker.hello(name="talker")
    listener = await connect(app)
    await listener.hello(name="listener", subscribe=["events"])
    await talker.send("event", {"name": "word_read", "params": {"word": "dog"}})
    msg = await listener.recv(type_="event")
    assert msg["payload"] == {
        "name": "word_read", "params": {"word": "dog"}, "origin": "talker",
    }
    await talker.ws.close()
    await listener.ws.close()


# ── Acts ────────────────────────────────────────────────────────────────────


async def test_act_blink(app):
    c = await connect(app)
    await c.hello()
    ref = await c.send("act", {"do": {"kind": "blink"}})
    acks = await c.acks_until_terminal(ref)
    assert [a["status"] for a in acks] == ["accepted", "started", "completed"]
    await c.ws.close()


async def test_act_say_rejected_tts_unavailable(app):
    c = await connect(app)
    await c.hello()
    ref = await c.send("act", {"do": {"kind": "say", "text": "hello"}})
    acks = await c.acks_until_terminal(ref)
    assert len(acks) == 1
    assert acks[0]["code"] == "tts_unavailable"
    await c.ws.close()


async def test_act_track_rejected_when_no_camera_is_configured(app):
    c = await connect(app)
    await c.hello()
    ref = await c.send("act", {"do": {"kind": "track", "duration_ms": 500}})
    acks = await c.acks_until_terminal(ref)
    assert len(acks) == 1
    assert acks[0]["code"] == "vision_unavailable"
    await c.ws.close()


async def test_vision_capability_is_advertised_only_with_a_camera(app, vision_app):
    plain = await connect(app)
    assert "vision" not in (await plain.hello())["payload"]["capabilities"]
    await plain.ws.close()

    seeing = await connect(vision_app)
    assert "vision" in (await seeing.hello())["payload"]["capabilities"]
    await seeing.ws.close()


async def test_act_track_runs_against_the_sim_camera(vision_app):
    c = await connect(vision_app)
    await c.hello()
    ref = await c.send("act", {"do": {"kind": "track", "duration_ms": 400}})
    acks = await c.acks_until_terminal(ref)
    assert [a["status"] for a in acks] == ["accepted", "started", "completed"]
    await c.ws.close()


async def test_state_reports_vision_health(app, vision_app):
    plain = await connect(app)
    await plain.hello()
    assert (await plain.state_get())["health"]["vision"] == "off"
    await plain.ws.close()

    seeing = await connect(vision_app)
    await seeing.hello()
    assert (await seeing.state_get())["health"]["vision"] == "sim"
    await seeing.ws.close()


async def test_act_unknown_animation(app):
    c = await connect(app)
    await c.hello()
    ref = await c.send("act", {"do": {"kind": "led", "animation": "sparkle_red"}})
    acks = await c.acks_until_terminal(ref)
    assert acks[0]["status"] == "error"
    assert acks[0]["code"] == "unknown_animation"
    await c.ws.close()


async def test_act_priority_out_of_range(app):
    c = await connect(app)
    await c.hello()
    ref = await c.send("act", {"do": {"kind": "blink"}, "priority": 90})
    acks = await c.acks_until_terminal(ref)
    assert acks[0]["code"] == "out_of_range"
    await c.ws.close()


async def test_cancel_active_act(app):
    c = await connect(app)
    await c.hello()
    ref = await c.send("act", {
        "do": {"kind": "eyelids", "openness": 0.5, "duration_ms": 5000},
        "tag": "slow",
    })
    started = await c.recv(type_="ack", ref=ref)
    assert started["payload"]["status"] == "accepted"
    cancel_ref = await c.send("cancel", {"target": ref})
    cancel_ack = (await c.recv(type_="ack", ref=cancel_ref))["payload"]
    assert cancel_ack["status"] == "completed"
    acks = await c.acks_until_terminal(ref)
    assert acks[-1]["status"] == "cancelled"
    await c.ws.close()


# ── E-stop ──────────────────────────────────────────────────────────────────


async def test_estop_engage_disengage(app):
    c = await connect(app)
    await c.hello()
    # long-running act to be flushed
    act_ref = await c.send("act", {
        "do": {"kind": "eyelids", "openness": 0.2, "duration_ms": 5000},
    })
    engage_ref = await c.send("estop", {"engage": True, "reason": "test"})
    acks = await c.acks_until_terminal(act_ref)
    assert acks[-1]["status"] == "cancelled"
    engage_ack = (await c.recv(type_="ack", ref=engage_ref))["payload"]
    assert engage_ack["status"] == "completed"
    assert app.led.current == {"animation": "alarm_red"}
    assert app.motion.estopped

    state = await c.state_get()
    assert state["estop"] is True

    # act/event rejected while engaged
    ref = await c.send("event", {"name": "celebrate"})
    acks = await c.acks_until_terminal(ref)
    assert acks[0]["code"] == "estopped"

    # any client may disengage
    ref = await c.send("estop", {"engage": False})
    ack = (await c.recv(type_="ack", ref=ref))["payload"]
    assert ack["status"] == "completed"
    # Releasing e-stop restores the configured resting animation, whatever it is.
    assert app.led.current == {
        "animation": app.config["led"]["default_animation"]
    }
    assert not app.motion.estopped
    state = await c.state_get()
    assert state["estop"] is False
    await c.ws.close()


# ── state.get / state pushes ───────────────────────────────────────────────


async def test_state_get_full_shape(app):
    c = await connect(app)
    await c.hello(name="shape-checker")
    state = await c.state_get()
    assert set(state) >= {"pose", "led", "queue", "estop", "lock", "clients", "health"}
    assert set(state["pose"]) == set(protocol.WIRE_JOINTS)
    assert state["health"]["servo"] == "sim"
    assert state["health"]["led"] == "sim"
    assert {"name": "shape-checker", "kind": "python"} in state["clients"]
    assert state["queue"] == {"depth": 0, "active_id": None, "active_client": None}
    await c.ws.close()


async def test_state_get_fields_filter_and_catalog(app):
    c = await connect(app)
    await c.hello()
    state = await c.state_get(fields=["pose", "catalog"])
    assert "pose" in state and "catalog" in state
    assert "led" not in state and "queue" not in state
    assert "celebrate" in state["catalog"]["events"]
    assert "nod" in state["catalog"]["gestures"]
    assert "breathing_cyan" in state["catalog"]["animations"]
    await c.ws.close()


async def test_state_pushed_to_subscribers(app):
    c = await connect(app)
    await c.hello(subscribe=["state"])
    msg = await c.recv(type_="state", timeout=3.0)   # 1 Hz heartbeat
    assert "pose" in msg["payload"]
    await c.ws.close()


# ── Locks ───────────────────────────────────────────────────────────────────


async def test_lock_blocks_other_connection(app):
    a = await connect(app)
    await a.hello(name="locker")
    b = await connect(app)
    await b.hello(name="blocked")

    ref = await a.send("lock", {"scopes": ["motion"], "ttl_s": 30})
    ack = (await a.recv(type_="ack", ref=ref))["payload"]
    assert ack["status"] == "completed"

    ref = await b.send("act", {"do": {"kind": "blink"}})
    acks = await b.acks_until_terminal(ref)
    assert acks[0]["code"] == "locked"

    state = await a.state_get(fields=["lock"])
    assert state["lock"] == {"held_by": "locker", "scopes": ["motion"]}

    ref = await a.send("unlock", {"scopes": ["motion"]})
    await a.recv(type_="ack", ref=ref)
    ref = await b.send("act", {"do": {"kind": "blink"}})
    acks = await b.acks_until_terminal(ref)
    assert acks[-1]["status"] == "completed"
    await a.ws.close()
    await b.ws.close()


async def test_lock_released_on_disconnect(app):
    a = await connect(app)
    await a.hello(name="locker")
    ref = await a.send("lock", {"scopes": ["motion", "led"], "ttl_s": 300})
    await a.recv(type_="ack", ref=ref)
    await a.ws.close()
    await asyncio.sleep(0.1)

    b = await connect(app)
    await b.hello(name="after")
    ref = await b.send("act", {"do": {"kind": "blink"}})
    acks = await b.acks_until_terminal(ref)
    assert acks[-1]["status"] == "completed"
    await b.ws.close()


# ── Misc plumbing ───────────────────────────────────────────────────────────


async def test_client_ping_pong(app):
    c = await connect(app)
    await c.hello()
    ref = await c.send("ping", {})
    pong = await c.recv(type_="pong")
    assert pong["payload"]["ref"] == ref
    await c.ws.close()


async def test_bad_json_gets_error_ack(app):
    c = await connect(app)
    await c.hello()
    await c.ws.send("this is not json")
    msg = await c.recv(type_="ack")
    assert msg["payload"]["status"] == "error"
    assert msg["payload"]["code"] == "bad_json"
    assert msg["payload"]["ref"] is None
    await c.ws.close()


async def test_unknown_type_gets_error_ack(app):
    c = await connect(app)
    await c.hello()
    ref = await c.send("teleport", {})
    msg = await c.recv(type_="ack", ref=ref)
    assert msg["payload"]["code"] == "unknown_type"
    await c.ws.close()


async def test_on_busy_drop_via_wire(app):
    c = await connect(app)
    await c.hello()
    slow_ref = await c.send("act", {
        "do": {"kind": "eyelids", "openness": 0.3, "duration_ms": 1500},
    })
    await c.recv(type_="ack", ref=slow_ref)   # accepted
    drop_ref = await c.send("act", {"do": {"kind": "blink"}, "on_busy": "drop"})
    acks = await c.acks_until_terminal(drop_ref)
    assert [a["status"] for a in acks] == ["dropped"]
    cancel_ref = await c.send("cancel", {"target": slow_ref})
    await c.recv(type_="ack", ref=cancel_ref)
    await c.ws.close()


# ── Mouth stream (§5.11) ────────────────────────────────────────────────────


async def test_mouth_frame_moves_mouth_without_ack(app):
    """A `mouth` frame drives the joint and is deliberately un-acked."""
    c = await connect(app)
    await c.hello()
    closed = app.motion.pose_rad()["mouth"]
    await c.send("mouth", {"openness": 1.0})
    await asyncio.sleep(0.3)
    assert app.motion.pose_rad()["mouth"] > closed
    # Nothing came back: a viseme sender at 60 Hz cannot consume acks.
    with pytest.raises(asyncio.TimeoutError):
        await c.recv(timeout=0.3)
    await c.ws.close()


async def test_mouth_frame_rejects_out_of_range(app):
    c = await connect(app)
    await c.hello()
    ref = await c.send("mouth", {"openness": 4.0})
    msg = await c.recv(type_="ack", ref=ref)
    assert msg["payload"]["code"] == "out_of_range"
    await c.ws.close()


async def test_mouth_stream_does_not_block_gestures(app):
    """The whole point of the MOUTH channel: speech and gesture coexist."""
    c = await connect(app)
    await c.hello()
    ref = await c.send("act", {"do": {"kind": "gesture", "name": "nod"}})
    accepted = await c.recv(type_="ack", ref=ref)
    assert accepted["payload"]["channels"] == ["motion"]
    # Stream visemes for the duration of the nod.
    for i in range(20):
        await c.send("mouth", {"openness": 1.0 if i % 2 else 0.0})
        await asyncio.sleep(0.02)
    acks = await c.acks_until_terminal(ref, timeout=10.0)
    assert acks[-1]["status"] == "completed"   # never superseded or dropped
    await c.ws.close()


async def test_mouth_action_claims_mouth_channel_not_motion(app):
    c = await connect(app)
    await c.hello()
    ref = await c.send("act", {
        "do": {"kind": "mouth", "openness": 0.5, "duration_ms": 100},
    })
    accepted = await c.recv(type_="ack", ref=ref)
    assert accepted["payload"]["channels"] == ["mouth"]
    await c.ws.close()


async def test_neutral_does_not_command_mouth(app):
    """`neutral` must not snap the mouth shut mid-word."""
    c = await connect(app)
    await c.hello()
    await c.send("mouth", {"openness": 1.0})
    await asyncio.sleep(0.3)
    open_rad = app.motion.pose_rad()["mouth"]
    ref = await c.send("act", {"do": {"kind": "neutral", "duration_ms": 200}})
    acks = await c.acks_until_terminal(ref)
    assert acks[-1]["status"] == "completed"
    assert app.motion.pose_rad()["mouth"] == pytest.approx(open_rad, abs=0.05)
    await c.ws.close()
