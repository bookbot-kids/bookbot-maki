"""Protocol codec tests: every fixture round-trips, client fixtures are
accepted by the server-side validators, and the Step grammar enforces its
documented limits and error codes."""

from __future__ import annotations

import json
from pathlib import Path

import pytest
import yaml

from maki_puppet import protocol
from maki_puppet.protocol import (
    Action,
    Channel,
    Par,
    ProtocolError,
    Seq,
    parse_envelope,
    parse_priority,
    parse_step,
    step_channels,
)

FIXTURES = Path(__file__).parent / "fixtures"
CONFIG = Path(__file__).parent.parent / "config"


def _load_catalogs():
    with open(CONFIG / "animations.yaml") as fh:
        animations = list(yaml.safe_load(fh)["animations"].keys())
    with open(CONFIG / "choreographies.yaml") as fh:
        doc = yaml.safe_load(fh)
    gestures = list(doc["gestures"].keys())
    return animations, gestures


ANIMATIONS, GESTURES = _load_catalogs()
ALL_FIXTURES = sorted(FIXTURES.glob("*.json"))


# ── Envelope round-trips ────────────────────────────────────────────────────


@pytest.mark.parametrize("path", ALL_FIXTURES, ids=lambda p: p.stem)
def test_fixture_roundtrip(path):
    raw = path.read_text()
    env = parse_envelope(raw)
    serialized = env.to_json()
    env2 = parse_envelope(serialized)
    assert env2 == env
    assert json.loads(serialized) == json.loads(raw)


CLIENT_VALIDATORS = {
    "hello": lambda p: protocol.parse_hello(p),
    "act_simple": lambda p: protocol.parse_act(p, animations=ANIMATIONS, gestures=GESTURES),
    "act_composed": lambda p: protocol.parse_act(p, animations=ANIMATIONS, gestures=GESTURES),
    "event": lambda p: protocol.parse_event(p),
    "cancel": lambda p: protocol.parse_cancel(p),
    "estop": lambda p: protocol.parse_estop(p),
    "state_get": lambda p: protocol.parse_state_get(p),
    "lock": lambda p: protocol.parse_lock(p),
    "unlock": lambda p: protocol.parse_unlock(p),
    "pong": lambda p: protocol.parse_pong(p),
}


@pytest.mark.parametrize("name", sorted(CLIENT_VALIDATORS), ids=str)
def test_client_fixture_accepted(name):
    env = parse_envelope((FIXTURES / f"{name}.json").read_text())
    CLIENT_VALIDATORS[name](env.payload)  # must not raise


def test_act_composed_shape():
    env = parse_envelope((FIXTURES / "act_composed.json").read_text())
    act = protocol.parse_act(env.payload, animations=ANIMATIONS, gestures=GESTURES)
    assert isinstance(act.step, Seq)
    assert isinstance(act.step.children[0], Par)
    assert act.priority == 70          # "high"
    assert act.on_busy == "replace"
    assert step_channels(act.step) == {Channel.MOTION, Channel.LED}


# ── Envelope validation ─────────────────────────────────────────────────────


@pytest.mark.parametrize("raw", [
    "not json",
    "[1, 2]",
    '{"id": "c-1", "ts": 1, "payload": {}}',            # no type
    '{"type": "act", "ts": 1, "payload": {}}',           # no id
    '{"type": "act", "id": "c-1", "payload": {}}',       # no ts
    '{"type": "act", "id": "c-1", "ts": 1}',             # no payload
    '{"type": "act", "id": "c-1", "ts": 1, "payload": 3}',
])
def test_bad_envelopes(raw):
    with pytest.raises(ProtocolError) as ei:
        parse_envelope(raw)
    assert ei.value.code == "bad_json"


def test_bad_envelope_ref_extraction():
    with pytest.raises(ProtocolError) as ei:
        parse_envelope('{"id": "c-9", "ts": 1, "payload": {}}')
    assert ei.value.ref == "c-9"
    with pytest.raises(ProtocolError) as ei:
        parse_envelope("{{{")
    assert ei.value.ref is None


# ── Step grammar limits ─────────────────────────────────────────────────────


def test_depth_limit():
    step = {"kind": "blink"}
    for _ in range(3):  # depth 4: OK
        step = {"seq": [step]}
    parse_step(step)
    with pytest.raises(ProtocolError) as ei:
        parse_step({"seq": [step]})  # depth 5
    assert ei.value.code == "out_of_range"


def test_action_count_limit():
    parse_step({"seq": [{"kind": "blink"}] * 64})
    with pytest.raises(ProtocolError) as ei:
        parse_step({"seq": [{"kind": "blink"}] * 65})
    assert ei.value.code == "out_of_range"


def test_empty_seq_rejected():
    with pytest.raises(ProtocolError) as ei:
        parse_step({"seq": []})
    assert ei.value.code == "out_of_range"


def test_unknown_kind():
    with pytest.raises(ProtocolError) as ei:
        parse_step({"kind": "backflip"})
    assert ei.value.code == "unknown_type"
    assert "backflip" in ei.value.detail


# ── Action argument validation ──────────────────────────────────────────────


def test_defaults_filled():
    step = parse_step({"kind": "blink"})
    assert step.args == {"duration_ms": 150}
    step = parse_step({"kind": "look"})
    assert step.args == {"pan": 0.0, "tilt": 0.0, "eyes_only": False, "duration_ms": 600}


def test_pose_unknown_joint():
    with pytest.raises(ProtocolError) as ei:
        parse_step({"kind": "pose", "joints": {"left_eyelid": 0.5}})
    assert ei.value.code == "unknown_joint"


def test_pose_out_of_range():
    with pytest.raises(ProtocolError) as ei:
        parse_step({"kind": "pose", "joints": {"head_pan": 1.5}})
    assert ei.value.code == "out_of_range"
    with pytest.raises(ProtocolError) as ei:
        parse_step({"kind": "pose", "joints": {"eyelids": -0.1}})
    assert ei.value.code == "out_of_range"


def test_eyelids_openness_required():
    with pytest.raises(ProtocolError) as ei:
        parse_step({"kind": "eyelids"})
    assert ei.value.code == "out_of_range"


def test_wait_duration_required_and_capped():
    with pytest.raises(ProtocolError):
        parse_step({"kind": "wait"})
    with pytest.raises(ProtocolError):
        parse_step({"kind": "wait", "duration_ms": 60001})
    assert parse_step({"kind": "wait", "duration_ms": 0}).args["duration_ms"] == 0


def test_led_exactly_one_argument():
    with pytest.raises(ProtocolError) as ei:
        parse_step({"kind": "led"})
    assert ei.value.code == "out_of_range"
    with pytest.raises(ProtocolError) as ei:
        parse_step(
            {"kind": "led", "animation": "breathing_cyan", "color": {"r": 1, "g": 2, "b": 3}},
            animations=ANIMATIONS,
        )
    assert ei.value.code == "out_of_range"


def test_led_unknown_animation():
    with pytest.raises(ProtocolError) as ei:
        parse_step({"kind": "led", "animation": "sparkle_red"}, animations=ANIMATIONS)
    assert ei.value.code == "unknown_animation"
    # without a catalog the name is structurally accepted
    parse_step({"kind": "led", "animation": "sparkle_red"})


def test_led_color_range():
    parse_step({"kind": "led", "color": {"r": 0, "g": 128, "b": 255}})
    with pytest.raises(ProtocolError):
        parse_step({"kind": "led", "color": {"r": 0, "g": 300, "b": 0}})
    with pytest.raises(ProtocolError):
        parse_step({"kind": "led", "color": {"r": 0, "g": 0}})


def test_gesture_validation():
    step = parse_step({"kind": "gesture", "name": "nod"}, gestures=GESTURES)
    assert step.args == {"name": "nod", "repeat": 1, "intensity": 1.0}
    with pytest.raises(ProtocolError) as ei:
        parse_step({"kind": "gesture", "name": "moonwalk"}, gestures=GESTURES)
    assert ei.value.code == "unknown_gesture"
    with pytest.raises(ProtocolError):
        parse_step({"kind": "gesture", "name": "nod", "repeat": 11}, gestures=GESTURES)
    with pytest.raises(ProtocolError):
        parse_step({"kind": "gesture", "name": "nod", "intensity": 0}, gestures=GESTURES)
    with pytest.raises(ProtocolError):
        parse_step({"kind": "gesture", "name": "nod", "intensity": 2.5}, gestures=GESTURES)


# ── Priority / on_busy / channels ──────────────────────────────────────────


def test_priority_aliases_and_range():
    assert parse_priority("low") == 20
    assert parse_priority("normal") == 50
    assert parse_priority("high") == 70
    assert parse_priority(0) == 0
    assert parse_priority(79) == 79
    assert parse_priority(None, 42) == 42
    for bad in (80, 100, -1, "urgent", 1.5, True):
        with pytest.raises(ProtocolError) as ei:
            parse_priority(bad)
        assert ei.value.code == "out_of_range"


def test_on_busy():
    assert protocol.parse_on_busy(None) == "queue"
    assert protocol.parse_on_busy("replace") == "replace"
    with pytest.raises(ProtocolError):
        protocol.parse_on_busy("panic")


def test_channels_union_and_wait_claims_nothing():
    step = parse_step({"seq": [
        {"kind": "wait", "duration_ms": 10},
        {"par": [{"kind": "blink"}, {"kind": "led", "animation": "x"}]},
    ]})
    assert step_channels(step) == {Channel.MOTION, Channel.LED}
    wait_only = parse_step({"seq": [{"kind": "wait", "duration_ms": 5}] * 3})
    assert step_channels(wait_only) == frozenset()
    say = parse_step({"kind": "say", "text": "hi"})
    assert step_channels(say) == {Channel.VOICE}
    assert Channel.VOICE.value == "tts"


def test_lock_parsing():
    msg = protocol.parse_lock({"scopes": ["motion", "led"], "ttl_s": 30})
    assert msg.scopes == (Channel.MOTION, Channel.LED)
    with pytest.raises(ProtocolError) as ei:
        protocol.parse_lock({"scopes": ["motion"], "ttl_s": 301})
    assert ei.value.code == "out_of_range"
    with pytest.raises(ProtocolError):
        protocol.parse_lock({"scopes": ["engine"], "ttl_s": 5})


def test_step_serialization_roundtrip():
    obj = {"seq": [
        {"kind": "gesture", "name": "nod", "repeat": 2, "intensity": 1.0},
        {"par": [{"kind": "blink", "duration_ms": 150},
                 {"kind": "led", "animation": "glow_gold"}]},
    ]}
    step = parse_step(obj, animations=ANIMATIONS, gestures=GESTURES)
    again = parse_step(protocol.step_to_obj(step), animations=ANIMATIONS, gestures=GESTURES)
    assert protocol.step_to_obj(again) == protocol.step_to_obj(step)


def test_say_contains_detection():
    step = parse_step({"seq": [{"kind": "blink"},
                               {"par": [{"kind": "say", "text": "hi"}]}]})
    assert protocol.step_contains_kind(step, "say")
    assert not protocol.step_contains_kind(parse_step({"kind": "blink"}), "say")


# ── posture action ──────────────────────────────────────────────────────────


def test_posture_validates_joints_like_pose():
    a = protocol.parse_step(
        {"kind": "posture", "joints": {"head_tilt": 1.0, "head_pan": -0.5}}
    )
    assert a.kind == "posture"
    assert a.args["joints"] == {"head_tilt": 1.0, "head_pan": -0.5}
    assert a.args["clear"] is False


def test_posture_clear_needs_no_joints():
    a = protocol.parse_step({"kind": "posture", "clear": True})
    assert a.args["clear"] is True


def test_posture_rejects_missing_joints():
    with pytest.raises(protocol.ProtocolError) as e:
        protocol.parse_step({"kind": "posture"})
    assert e.value.code == protocol.E_OUT_OF_RANGE


def test_posture_rejects_unknown_joint_and_out_of_range():
    with pytest.raises(protocol.ProtocolError) as e:
        protocol.parse_step({"kind": "posture", "joints": {"elbow": 0.0}})
    assert e.value.code == protocol.E_UNKNOWN_JOINT
    with pytest.raises(protocol.ProtocolError) as e:
        protocol.parse_step({"kind": "posture", "joints": {"head_tilt": 5.0}})
    assert e.value.code == protocol.E_OUT_OF_RANGE


def test_posture_occupies_the_motion_channel():
    assert protocol.action_channels("posture") == frozenset({protocol.Channel.MOTION})
