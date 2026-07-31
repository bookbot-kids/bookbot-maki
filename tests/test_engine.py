"""ActionEngine arbitration tests: priority, on_busy, cancel, locks, estop.

Uses fake motion/led objects (recording calls) and short real actions, so
these tests exercise the actual executor and the actual action runners
without hardware or the 50 Hz motion thread.
"""

from __future__ import annotations

import asyncio

import pytest

from maki_puppet.engine import ActionEngine, Performance, QUEUE_DEPTH
from maki_puppet.protocol import Action, Channel, ProtocolError, Seq, parse_step

pytestmark = pytest.mark.asyncio


class FakeMotion:
    def __init__(self):
        self.layers: dict = {}
        self.released: list = []
        self.estop_calls: list = []

    def set_layer(self, layer, targets):
        self.layers[layer] = dict(targets)

    def release_layer(self, layer):
        self.released.append(layer)
        self.layers.pop(layer, None)

    def pose_rad(self):
        return {}

    def estop(self, engaged):
        self.estop_calls.append(engaged)


class FakeLed:
    def __init__(self):
        self.calls: list = []
        self.current = {"animation": "breathing_cyan"}

    def set_animation(self, name):
        if name == "nope":
            raise KeyError(name)
        self.calls.append(name)
        self.current = {"animation": name}

    def set_color(self, r, g, b):
        self.calls.append((r, g, b))
        self.current = {"color": {"r": r, "g": g, "b": b}}


class FakeClient:
    def __init__(self, name="tester"):
        self.name = name


GESTURES = {
    "nod": [
        {"t_ms": 0, "joints": {"head_tilt": 0.0}},
        {"t_ms": 60, "joints": {"head_tilt": -0.5}},
        {"t_ms": 120, "joints": {"head_tilt": 0.0}},
    ],
}


@pytest.fixture
def rig():
    motion, led = FakeMotion(), FakeLed()
    acks: list = []
    engine = ActionEngine(
        motion,
        led,
        get_gesture=lambda name: GESTURES[name],
        ack_cb=lambda perf, status, fields: acks.append((perf.id, status, dict(fields))),
    )
    return engine, motion, led, acks


def make_perf(perf_id, step_obj, *, client=None, priority=50, on_busy="queue", tag=None,
              choreography=None):
    return Performance(
        id=perf_id,
        client=client or FakeClient(),
        step=parse_step(step_obj) if isinstance(step_obj, dict) else step_obj,
        priority=priority,
        on_busy=on_busy,
        tag=tag,
        choreography=choreography,
    )


def statuses(acks, perf_id):
    return [s for i, s, f in acks if i == perf_id]


async def wait_status(acks, perf_id, status, timeout=3.0):
    deadline = asyncio.get_event_loop().time() + timeout
    while asyncio.get_event_loop().time() < deadline:
        if status in statuses(acks, perf_id):
            return
        await asyncio.sleep(0.01)
    raise AssertionError(f"{perf_id} never reached {status}; got {statuses(acks, perf_id)}")


MOTION_50MS = {"kind": "eyelids", "openness": 0.5, "duration_ms": 50}
MOTION_300MS = {"kind": "eyelids", "openness": 1.0, "duration_ms": 300}


# ── Basic lifecycle ─────────────────────────────────────────────────────────


async def test_single_act_lifecycle(rig):
    engine, motion, led, acks = rig
    perf = make_perf("c-1", MOTION_50MS)
    assert engine.submit(perf) == "started"
    await wait_status(acks, "c-1", "completed")
    assert statuses(acks, "c-1") == ["accepted", "started", "completed"]
    # expression layer touched then released
    assert "expression" in motion.released
    # completed carries duration_ms
    completed = [f for i, s, f in acks if i == "c-1" and s == "completed"][0]
    assert "duration_ms" in completed
    accepted = [f for i, s, f in acks if i == "c-1" and s == "accepted"][0]
    assert accepted["channels"] == ["motion"]


async def test_wait_only_claims_no_channel(rig):
    engine, motion, led, acks = rig
    engine.submit(make_perf("c-1", MOTION_300MS))
    engine.submit(make_perf("c-2", {"kind": "wait", "duration_ms": 20}))
    await wait_status(acks, "c-2", "completed")
    # c-2 was never queued despite c-1 being busy on motion
    assert statuses(acks, "c-2") == ["accepted", "started", "completed"]


async def test_choreography_field_on_event_acks(rig):
    engine, motion, led, acks = rig
    engine.submit(make_perf("c-9", MOTION_50MS, choreography="celebrate"))
    await wait_status(acks, "c-9", "completed")
    assert all(f.get("choreography") == "celebrate" for i, s, f in acks if i == "c-9")


async def test_seq_progress_on_started(rig):
    engine, motion, led, acks = rig
    step = parse_step({"seq": [{"kind": "wait", "duration_ms": 10},
                               {"kind": "wait", "duration_ms": 10}]})
    engine.submit(make_perf("c-1", step))
    await wait_status(acks, "c-1", "completed")
    started = [f for i, s, f in acks if i == "c-1" and s == "started"][0]
    assert started["progress"] == {"step": 1, "of": 2}


# ── on_busy semantics ───────────────────────────────────────────────────────


async def test_queue_then_fifo(rig):
    engine, motion, led, acks = rig
    engine.submit(make_perf("c-1", MOTION_300MS))
    assert engine.submit(make_perf("c-2", MOTION_50MS, on_busy="queue")) == "queued"
    assert statuses(acks, "c-2") == ["accepted", "queued"]
    await wait_status(acks, "c-2", "completed")
    assert statuses(acks, "c-1") == ["accepted", "started", "completed"]
    assert statuses(acks, "c-2") == ["accepted", "queued", "started", "completed"]


async def test_drop_while_busy(rig):
    engine, motion, led, acks = rig
    engine.submit(make_perf("c-1", MOTION_300MS))
    assert engine.submit(make_perf("c-2", MOTION_50MS, on_busy="drop")) == "dropped"
    # single terminal ack, no preceding accepted (§8.1)
    assert statuses(acks, "c-2") == ["dropped"]
    await wait_status(acks, "c-1", "completed")


async def test_drop_starts_when_idle(rig):
    engine, motion, led, acks = rig
    engine.submit(make_perf("c-1", MOTION_50MS, on_busy="drop"))
    await wait_status(acks, "c-1", "completed")


async def test_replace_supersedes_equal_priority(rig):
    engine, motion, led, acks = rig
    engine.submit(make_perf("c-1", MOTION_300MS, priority=50))
    assert engine.submit(make_perf("c-2", MOTION_50MS, priority=50, on_busy="replace")) == "started"
    await wait_status(acks, "c-1", "superseded")
    await wait_status(acks, "c-2", "completed")
    assert statuses(acks, "c-1") == ["accepted", "started", "superseded"]


async def test_replace_degrades_to_queue_for_lower_priority(rig):
    engine, motion, led, acks = rig
    engine.submit(make_perf("c-1", MOTION_300MS, priority=70))
    assert engine.submit(make_perf("c-2", MOTION_50MS, priority=20, on_busy="replace")) == "queued"
    await wait_status(acks, "c-2", "completed")
    # c-1 ran to completion — it was never superseded
    assert statuses(acks, "c-1") == ["accepted", "started", "completed"]


async def test_queue_full(rig):
    engine, motion, led, acks = rig
    engine.submit(make_perf("c-0", MOTION_300MS))
    for i in range(QUEUE_DEPTH):
        engine.submit(make_perf(f"c-{i + 1}", MOTION_50MS))
    with pytest.raises(ProtocolError) as ei:
        engine.submit(make_perf("c-overflow", MOTION_50MS))
    assert ei.value.code == "queue_full"
    await engine.stop_all()


async def test_multichannel_claims_union(rig):
    engine, motion, led, acks = rig
    both = {"seq": [{"kind": "eyelids", "openness": 0.5, "duration_ms": 200},
                    {"kind": "led", "animation": "glow_gold"}]}
    engine.submit(make_perf("c-1", both))
    # led-only act must wait: the seq holds the LED channel for its whole duration
    assert engine.submit(make_perf("c-2", {"kind": "led", "animation": "warm_pulse"})) == "queued"
    await wait_status(acks, "c-2", "completed")
    assert led.calls == ["glow_gold", "warm_pulse"]


async def test_multichannel_replace_supersedes_all_victims(rig):
    engine, motion, led, acks = rig
    engine.submit(make_perf("c-m", MOTION_300MS))
    engine.submit(make_perf("c-l", {"seq": [{"kind": "led", "animation": "glow_gold"},
                                            {"kind": "wait", "duration_ms": 300}]}))
    both = {"par": [{"kind": "eyelids", "openness": 1.0, "duration_ms": 50},
                    {"kind": "led", "animation": "warm_pulse"}]}
    assert engine.submit(make_perf("c-2", both, on_busy="replace")) == "started"
    await wait_status(acks, "c-m", "superseded")
    await wait_status(acks, "c-l", "superseded")
    await wait_status(acks, "c-2", "completed")


# ── cancel ──────────────────────────────────────────────────────────────────


async def test_cancel_active_by_id(rig):
    engine, motion, led, acks = rig
    client = FakeClient()
    engine.submit(make_perf("c-1", MOTION_300MS, client=client))
    await asyncio.sleep(0.02)   # let the runner start and touch its layer
    assert engine.cancel("c-1", client=client) == 1
    await wait_status(acks, "c-1", "cancelled")
    assert "expression" in motion.released


async def test_cancel_id_is_per_connection(rig):
    engine, motion, led, acks = rig
    owner, other = FakeClient("owner"), FakeClient("other")
    engine.submit(make_perf("c-1", MOTION_300MS, client=owner))
    assert engine.cancel("c-1", client=other) == 0   # not yours → no-op
    assert engine.cancel("c-1", client=owner) == 1
    await wait_status(acks, "c-1", "cancelled")


async def test_cancel_queued(rig):
    engine, motion, led, acks = rig
    client = FakeClient()
    engine.submit(make_perf("c-1", MOTION_300MS, client=client))
    engine.submit(make_perf("c-2", MOTION_50MS, client=client))
    assert engine.cancel("c-2", client=client) == 1
    assert statuses(acks, "c-2") == ["accepted", "queued", "cancelled"]
    await wait_status(acks, "c-1", "completed")


async def test_cancel_by_tag_any_connection(rig):
    engine, motion, led, acks = rig
    a, b = FakeClient("a"), FakeClient("b")
    engine.submit(make_perf("c-1", MOTION_300MS, client=a, tag="ui"))
    engine.submit(make_perf("c-2", MOTION_50MS, client=b, tag="ui"))
    assert engine.cancel("tag:ui", client=b) == 2
    await wait_status(acks, "c-1", "cancelled")
    await wait_status(acks, "c-2", "cancelled")


async def test_cancel_all(rig):
    engine, motion, led, acks = rig
    engine.submit(make_perf("c-1", MOTION_300MS))
    engine.submit(make_perf("c-2", MOTION_50MS))
    assert engine.cancel("all") == 2
    await wait_status(acks, "c-1", "cancelled")
    await wait_status(acks, "c-2", "cancelled")
    assert engine.cancel("all") == 0   # idempotent no-op


# ── estop ───────────────────────────────────────────────────────────────────


async def test_estop_flushes_and_freezes(rig):
    engine, motion, led, acks = rig
    engine.submit(make_perf("c-1", MOTION_300MS))
    engine.submit(make_perf("c-2", MOTION_50MS))
    engine.set_estop(True, "test")
    assert engine.estopped
    await wait_status(acks, "c-1", "cancelled")
    await wait_status(acks, "c-2", "cancelled")
    assert motion.estop_calls == [True]
    assert led.calls[-1] == "alarm_red"
    with pytest.raises(ProtocolError) as ei:
        engine.submit(make_perf("c-3", MOTION_50MS))
    assert ei.value.code == "estopped"
    engine.set_estop(False)
    assert not engine.estopped
    assert motion.estop_calls == [True, False]
    assert led.calls[-1] == "breathing_cyan"
    engine.submit(make_perf("c-4", MOTION_50MS))
    await wait_status(acks, "c-4", "completed")


async def test_estop_engage_idempotent(rig):
    engine, motion, led, acks = rig
    engine.set_estop(True)
    engine.set_estop(True)
    assert motion.estop_calls == [True]
    engine.set_estop(False)


# ── locks ───────────────────────────────────────────────────────────────────


async def test_locks_exclusive_per_connection(rig):
    engine, motion, led, acks = rig
    a, b = FakeClient("a"), FakeClient("b")
    engine.lock(a, [Channel.MOTION], ttl_s=30)
    with pytest.raises(ProtocolError) as ei:
        engine.submit(make_perf("c-1", MOTION_50MS, client=b))
    assert ei.value.code == "locked"
    with pytest.raises(ProtocolError) as ei:
        engine.lock(b, [Channel.MOTION], ttl_s=30)
    assert ei.value.code == "locked"
    # holder may act; internal (client=None) work bypasses locks
    engine.submit(make_perf("c-2", MOTION_50MS, client=a))
    await wait_status(acks, "c-2", "completed")
    internal = make_perf("idle-1", MOTION_50MS)
    internal.client = None
    engine.submit(internal)
    # unlock releases only held scopes; b can then act
    engine.unlock(b, [Channel.MOTION])   # no-op: b doesn't hold it
    with pytest.raises(ProtocolError):
        engine.submit(make_perf("c-3", MOTION_50MS, client=b))
    engine.unlock(a, [Channel.MOTION])
    engine.submit(make_perf("c-4", MOTION_50MS, client=b))
    await wait_status(acks, "c-4", "completed")


async def test_lock_ttl_expires(rig):
    engine, motion, led, acks = rig
    a, b = FakeClient("a"), FakeClient("b")
    engine.lock(a, [Channel.LED], ttl_s=0.01)
    await asyncio.sleep(0.05)
    engine.submit(make_perf("c-1", {"kind": "led", "animation": "glow_gold"}, client=b))
    await wait_status(acks, "c-1", "completed")


async def test_release_client_unlocks_and_aborts(rig):
    engine, motion, led, acks = rig
    a, b = FakeClient("a"), FakeClient("b")
    engine.lock(a, [Channel.MOTION], ttl_s=60)
    engine.submit(make_perf("c-1", MOTION_300MS, client=a))
    engine.release_client(a, abort=True)
    await wait_status(acks, "c-1", "cancelled")
    engine.submit(make_perf("c-2", MOTION_50MS, client=b))  # lock released
    await wait_status(acks, "c-2", "completed")


# ── executor details ────────────────────────────────────────────────────────


async def test_gesture_and_led_runners(rig):
    engine, motion, led, acks = rig
    step = {"par": [{"kind": "gesture", "name": "nod"},
                    {"kind": "led", "animation": "attention_sweep"}]}
    engine.submit(make_perf("c-1", step))
    await wait_status(acks, "c-1", "completed")
    assert led.calls == ["attention_sweep"]
    assert "gesture" in motion.released     # keyframes fed the gesture layer


async def test_unknown_gesture_at_runtime_errors(rig):
    engine, motion, led, acks = rig
    # bypass catalog validation (parse without catalogs) to hit the runtime path
    engine.submit(make_perf("c-1", {"kind": "gesture", "name": "vanished"}))
    await wait_status(acks, "c-1", "error")
    err = [f for i, s, f in acks if i == "c-1" and s == "error"][0]
    assert err["code"] == "unknown_gesture"


async def test_say_is_skipped_in_execution(rig):
    engine, motion, led, acks = rig
    # choreography path: say steps are skipped, not fatal (§10.5)
    step = {"seq": [{"kind": "say", "text": "hello"},
                    {"kind": "led", "animation": "glow_gold"}]}
    engine.submit(make_perf("c-1", step, choreography="custom"))
    await wait_status(acks, "c-1", "completed")
    assert led.calls == ["glow_gold"]


async def test_superseded_replacement_keeps_layer(rig):
    engine, motion, led, acks = rig
    engine.submit(make_perf("c-1", MOTION_300MS))
    await asyncio.sleep(0.02)
    engine.submit(make_perf("c-2", {"kind": "eyelids", "openness": 0.2, "duration_ms": 80},
                            on_busy="replace"))
    await wait_status(acks, "c-1", "superseded")
    # give the superseded task's cleanup a chance to (wrongly) release
    await asyncio.sleep(0.02)
    assert "expression" in motion.layers   # still fed by c-2
    await wait_status(acks, "c-2", "completed")


async def test_stop_all(rig):
    engine, motion, led, acks = rig
    engine.submit(make_perf("c-1", MOTION_300MS))
    engine.submit(make_perf("c-2", MOTION_50MS))
    await engine.stop_all()
    assert "cancelled" in statuses(acks, "c-1")
    assert "cancelled" in statuses(acks, "c-2")
    assert not engine.has_client_work()


async def test_queue_snapshot(rig):
    engine, motion, led, acks = rig
    client = FakeClient("snapper")
    engine.submit(make_perf("c-1", MOTION_300MS, client=client))
    engine.submit(make_perf("c-2", MOTION_50MS, client=client))
    snap = engine.queue_snapshot()
    assert snap == {"depth": 1, "active_id": "c-1", "active_client": "snapper"}
    await engine.stop_all()
    snap = engine.queue_snapshot()
    assert snap["active_id"] is None and snap["depth"] == 0
