"""MPP/1 wire protocol: envelope codec, message parsing, Step grammar.

This is the single parser/validator for both the WebSocket wire and
`choreographies.yaml` (PROTOCOL.md is normative).  Everything here is pure
data validation — no I/O, no hardware, no asyncio.

Validation philosophy (PROTOCOL.md §1.2, §6): out-of-range values are
REJECTED, not clamped — an out-of-range value is a client bug.
"""

from __future__ import annotations

import json
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Iterable, Mapping, Optional, Union

PROTOCOL_VERSION = 1
MAX_FRAME_BYTES = 64 * 1024
MAX_STEP_DEPTH = 4
MAX_STEP_ACTIONS = 64
MAX_LOCK_TTL_S = 300.0
MAX_CLIENT_PRIORITY = 79   # >= 80 is reserved for the server-internal safety layer
MAX_DURATION_MS = 60000

PRIORITY_ALIASES = {"low": 20, "normal": 50, "high": 70}
ON_BUSY_VALUES = ("queue", "replace", "drop")

# Close codes (PROTOCOL.md §2.1)
CLOSE_GOING_AWAY = 1001
CLOSE_TOO_BIG = 1009
CLOSE_UNSUPPORTED_PROTOCOL = 4400
CLOSE_FRAME_BEFORE_HELLO = 4401


# ── Error codes (PROTOCOL.md §8.2) ─────────────────────────────────────────

E_BAD_JSON = "bad_json"
E_UNSUPPORTED_PROTOCOL = "unsupported_protocol"
E_UNKNOWN_TYPE = "unknown_type"
E_UNKNOWN_EVENT = "unknown_event"
E_UNKNOWN_ANIMATION = "unknown_animation"
E_UNKNOWN_GESTURE = "unknown_gesture"
E_UNKNOWN_JOINT = "unknown_joint"
E_OUT_OF_RANGE = "out_of_range"
E_BUSY = "busy"
E_LOCKED = "locked"
E_ESTOPPED = "estopped"
E_QUEUE_FULL = "queue_full"
E_TTS_UNAVAILABLE = "tts_unavailable"
E_VISION_UNAVAILABLE = "vision_unavailable"
E_INTERNAL = "internal"


class ProtocolError(Exception):
    """A protocol violation that maps to an `ack {status: error}` frame."""

    def __init__(self, code: str, detail: str = "", ref: Optional[str] = None):
        super().__init__(f"{code}: {detail}" if detail else code)
        self.code = code
        self.detail = detail
        self.ref = ref  # best-effort envelope id of the offending frame


# ── Channels ────────────────────────────────────────────────────────────────


class Channel(str, Enum):
    """Arbitration channels. Wire names are 'motion' | 'mouth' | 'led' | 'tts'.

    MOUTH is separate from MOTION so a viseme stream (§5.13) never contends
    with head/eye gestures: speech and gesture are independent behaviors that
    must be able to run at the same time.
    """

    MOTION = "motion"
    MOUTH = "mouth"
    LED = "led"
    VOICE = "tts"


CHANNEL_ORDER = (Channel.MOTION, Channel.MOUTH, Channel.LED, Channel.VOICE)
CHANNEL_NAMES = tuple(c.value for c in CHANNEL_ORDER)


def channel_from_name(name: str) -> Channel:
    for ch in Channel:
        if ch.value == name:
            return ch
    raise ProtocolError(E_OUT_OF_RANGE, f"unknown channel scope '{name}'")


# ── Envelope ────────────────────────────────────────────────────────────────


@dataclass
class Envelope:
    type: str
    id: str
    ts: int
    payload: dict

    def to_json(self) -> str:
        return json.dumps(
            {"type": self.type, "id": self.id, "ts": self.ts, "payload": self.payload},
            separators=(",", ":"),
        )


def now_ms() -> int:
    return int(time.time() * 1000)


def make_envelope(type_: str, id_: str, payload: dict) -> Envelope:
    return Envelope(type=type_, id=id_, ts=now_ms(), payload=payload)


def parse_envelope(raw: Union[str, bytes]) -> Envelope:
    """Parse one wire frame. Raises ProtocolError(bad_json) with a
    best-effort ``ref`` (the frame's id, when it could be extracted)."""
    try:
        obj = json.loads(raw)
    except (ValueError, TypeError):
        raise ProtocolError(E_BAD_JSON, "frame is not valid JSON", ref=None)
    if not isinstance(obj, dict):
        raise ProtocolError(E_BAD_JSON, "frame is not a JSON object", ref=None)
    ref = obj.get("id") if isinstance(obj.get("id"), str) else None
    type_ = obj.get("type")
    if not isinstance(type_, str) or not type_:
        raise ProtocolError(E_BAD_JSON, "envelope 'type' must be a non-empty string", ref=ref)
    if ref is None or not obj.get("id"):
        raise ProtocolError(E_BAD_JSON, "envelope 'id' must be a non-empty string", ref=ref)
    ts = obj.get("ts")
    if isinstance(ts, bool) or not isinstance(ts, (int, float)):
        raise ProtocolError(E_BAD_JSON, "envelope 'ts' must be a number", ref=ref)
    payload = obj.get("payload")
    if not isinstance(payload, dict):
        raise ProtocolError(E_BAD_JSON, "envelope 'payload' must be an object", ref=ref)
    return Envelope(type=type_, id=obj["id"], ts=int(ts), payload=payload)


# ── Wire joints (normalized units, PROTOCOL.md §1.2) ───────────────────────

# name -> (min, max, neutral)
WIRE_JOINTS: dict[str, tuple[float, float, float]] = {
    "head_pan": (-1.0, 1.0, 0.0),
    "head_tilt": (-1.0, 1.0, 0.0),
    "eyes_pan": (-1.0, 1.0, 0.0),
    "eyes_tilt": (-1.0, 1.0, 0.0),
    "eyelids": (0.0, 1.0, 1.0),   # openness: 0 = closed, 1 = open
    "mouth": (0.0, 1.0, 0.0),     # openness
}


def wire_joints_catalog() -> dict:
    """`welcome.joints` payload fragment."""
    return {
        name: {"min": lo, "max": hi, "neutral": neutral}
        for name, (lo, hi, neutral) in WIRE_JOINTS.items()
    }


# ── Step grammar (PROTOCOL.md §6–7) ─────────────────────────────────────────


@dataclass
class Action:
    kind: str
    args: dict  # validated, defaults filled


@dataclass
class Seq:
    children: list


@dataclass
class Par:
    children: list


Step = Union[Action, Seq, Par]


def _is_number(v: Any) -> bool:
    return isinstance(v, (int, float)) and not isinstance(v, bool)


def _num(args: Mapping, key: str, default: Optional[float], lo: float, hi: float,
         *, kind: str, required: bool = False) -> float:
    if key not in args or args[key] is None:
        if required:
            raise ProtocolError(E_OUT_OF_RANGE, f"{kind}.{key} is required")
        return float(default)  # type: ignore[arg-type]
    v = args[key]
    if not _is_number(v):
        raise ProtocolError(E_OUT_OF_RANGE, f"{kind}.{key} must be a number")
    v = float(v)
    if not (lo <= v <= hi):
        raise ProtocolError(
            E_OUT_OF_RANGE, f"{kind}.{key}={v} outside [{lo}, {hi}]"
        )
    return v


def _duration(args: Mapping, kind: str, default: Optional[int], *, required: bool = False) -> int:
    return int(_num(args, "duration_ms", default, 0, MAX_DURATION_MS,
                    kind=kind, required=required))


def _validate_blink(a: Mapping) -> dict:
    return {"duration_ms": _duration(a, "blink", 150)}


def _validate_look(a: Mapping) -> dict:
    return {
        "pan": _num(a, "pan", 0.0, -1.0, 1.0, kind="look"),
        "tilt": _num(a, "tilt", 0.0, -1.0, 1.0, kind="look"),
        "eyes_only": bool(a.get("eyes_only", False)),
        "duration_ms": _duration(a, "look", 600),
    }


def _validate_eyelids(a: Mapping) -> dict:
    return {
        "openness": _num(a, "openness", None, 0.0, 1.0, kind="eyelids", required=True),
        "duration_ms": _duration(a, "eyelids", 300),
    }


def _validate_mouth(a: Mapping) -> dict:
    return {
        "openness": _num(a, "openness", None, 0.0, 1.0, kind="mouth", required=True),
        "duration_ms": _duration(a, "mouth", 200),
    }


def _validate_pose(a: Mapping) -> dict:
    joints = a.get("joints")
    if not isinstance(joints, dict) or not joints:
        raise ProtocolError(E_OUT_OF_RANGE, "pose.joints must be a non-empty object")
    out: dict[str, float] = {}
    for name, value in joints.items():
        if name not in WIRE_JOINTS:
            raise ProtocolError(
                E_UNKNOWN_JOINT,
                f"unknown joint '{name}'; wire joints: {', '.join(WIRE_JOINTS)}",
            )
        lo, hi, _ = WIRE_JOINTS[name]
        if not _is_number(value):
            raise ProtocolError(E_OUT_OF_RANGE, f"pose.joints.{name} must be a number")
        v = float(value)
        if not (lo <= v <= hi):
            raise ProtocolError(
                E_OUT_OF_RANGE, f"pose.joints.{name}={v} outside [{lo}, {hi}]"
            )
        out[name] = v
    return {"joints": out, "duration_ms": _duration(a, "pose", 800)}


def _validate_gesture(a: Mapping, gestures: Optional[Iterable[str]]) -> dict:
    name = a.get("name")
    if not isinstance(name, str) or not name:
        raise ProtocolError(E_OUT_OF_RANGE, "gesture.name is required")
    if gestures is not None and name not in set(gestures):
        raise ProtocolError(
            E_UNKNOWN_GESTURE, f"no gesture '{name}'; see welcome.gestures"
        )
    repeat = a.get("repeat", 1)
    if isinstance(repeat, bool) or not isinstance(repeat, int) or not (1 <= repeat <= 10):
        raise ProtocolError(E_OUT_OF_RANGE, "gesture.repeat must be an integer 1..10")
    intensity = a.get("intensity", 1.0)
    if not _is_number(intensity) or not (0.0 < float(intensity) <= 2.0):
        raise ProtocolError(E_OUT_OF_RANGE, "gesture.intensity must be in (0, 2]")
    return {"name": name, "repeat": repeat, "intensity": float(intensity)}


def _validate_led(a: Mapping, animations: Optional[Iterable[str]]) -> dict:
    has_anim = a.get("animation") is not None
    has_color = a.get("color") is not None
    if has_anim == has_color:
        raise ProtocolError(
            E_OUT_OF_RANGE, "led requires exactly one of 'animation' or 'color'"
        )
    if has_anim:
        name = a["animation"]
        if not isinstance(name, str):
            raise ProtocolError(E_OUT_OF_RANGE, "led.animation must be a string")
        if animations is not None and name not in set(animations):
            raise ProtocolError(
                E_UNKNOWN_ANIMATION, f"no animation '{name}'; see welcome.animations"
            )
        return {"animation": name}
    color = a["color"]
    if not isinstance(color, dict):
        raise ProtocolError(E_OUT_OF_RANGE, "led.color must be {r, g, b}")
    out = {}
    for comp in ("r", "g", "b"):
        v = color.get(comp)
        if isinstance(v, bool) or not isinstance(v, int) or not (0 <= v <= 255):
            raise ProtocolError(
                E_OUT_OF_RANGE, f"led.color.{comp} must be an integer 0..255"
            )
        out[comp] = v
    return {"color": out}


def _validate_say(a: Mapping) -> dict:
    text = a.get("text")
    if not isinstance(text, str):
        raise ProtocolError(E_OUT_OF_RANGE, "say.text is required")
    return {"text": text}


def _validate_wait(a: Mapping) -> dict:
    return {"duration_ms": _duration(a, "wait", None, required=True)}


def _validate_neutral(a: Mapping) -> dict:
    return {"duration_ms": _duration(a, "neutral", 700)}


TRACK_TARGETS = ("face",)


def _validate_track(a: Mapping) -> dict:
    target = a.get("target", "face")
    if target not in TRACK_TARGETS:
        raise ProtocolError(
            E_OUT_OF_RANGE,
            f"unknown track.target '{target}'; known: {', '.join(TRACK_TARGETS)}",
        )
    eyes_only = a.get("eyes_only", False)
    if not isinstance(eyes_only, bool):
        raise ProtocolError(E_OUT_OF_RANGE, "track.eyes_only must be a boolean")
    return {
        "target": target,
        "duration_ms": _duration(a, "track", 5000),
        "eyes_only": eyes_only,
    }


def _validate_posture(a: Mapping) -> dict:
    """`posture` holds a baseline pose until it is explicitly cleared.

    ``{"kind": "posture", "clear": true}`` releases it; otherwise ``joints``
    is required and is validated exactly like ``pose``.
    """
    if a.get("clear"):
        return {"clear": True}
    joints = a.get("joints")
    if not isinstance(joints, dict) or not joints:
        raise ProtocolError(
            E_OUT_OF_RANGE,
            "posture needs a non-empty 'joints' object, or 'clear': true",
        )
    out: dict[str, float] = {}
    for name, value in joints.items():
        if name not in WIRE_JOINTS:
            raise ProtocolError(
                E_UNKNOWN_JOINT,
                f"unknown joint '{name}'; wire joints: {', '.join(WIRE_JOINTS)}",
            )
        lo, hi, _ = WIRE_JOINTS[name]
        if not _is_number(value):
            raise ProtocolError(
                E_OUT_OF_RANGE, f"posture.joints.{name} must be a number"
            )
        v = float(value)
        if not (lo <= v <= hi):
            raise ProtocolError(
                E_OUT_OF_RANGE, f"posture.joints.{name}={v} outside [{lo}, {hi}]"
            )
        out[name] = v
    return {"joints": out, "clear": False}


# kind -> (channels, validator).  `wait` occupies no channel by itself (§6).
_ACTION_TABLE: dict[str, tuple[frozenset, Any]] = {
    "blink": (frozenset({Channel.MOTION}), lambda a, an, ge: _validate_blink(a)),
    "look": (frozenset({Channel.MOTION}), lambda a, an, ge: _validate_look(a)),
    "eyelids": (frozenset({Channel.MOTION}), lambda a, an, ge: _validate_eyelids(a)),
    "mouth": (frozenset({Channel.MOUTH}), lambda a, an, ge: _validate_mouth(a)),
    "pose": (frozenset({Channel.MOTION}), lambda a, an, ge: _validate_pose(a)),
    "gesture": (frozenset({Channel.MOTION}), lambda a, an, ge: _validate_gesture(a, ge)),
    "led": (frozenset({Channel.LED}), lambda a, an, ge: _validate_led(a, an)),
    "say": (frozenset({Channel.VOICE}), lambda a, an, ge: _validate_say(a)),
    "wait": (frozenset(), lambda a, an, ge: _validate_wait(a)),
    "neutral": (frozenset({Channel.MOTION}), lambda a, an, ge: _validate_neutral(a)),
    "posture": (frozenset({Channel.MOTION}), lambda a, an, ge: _validate_posture(a)),
    "track": (frozenset({Channel.MOTION}), lambda a, an, ge: _validate_track(a)),
}

ACTION_KINDS = tuple(_ACTION_TABLE)


def action_channels(kind: str) -> frozenset:
    return _ACTION_TABLE[kind][0]


def parse_step(
    obj: Any,
    *,
    animations: Optional[Iterable[str]] = None,
    gestures: Optional[Iterable[str]] = None,
) -> Step:
    """Parse and validate one Step tree (wire `act.do` or a choreography step).

    ``animations``/``gestures`` are the live catalogs; pass None to skip
    catalog checks (structural validation only).
    """
    animations = set(animations) if animations is not None else None
    gestures = set(gestures) if gestures is not None else None
    count = [0]

    def walk(node: Any, depth: int) -> Step:
        if depth > MAX_STEP_DEPTH:
            raise ProtocolError(
                E_OUT_OF_RANGE, f"step nesting exceeds depth {MAX_STEP_DEPTH}"
            )
        if not isinstance(node, dict):
            raise ProtocolError(E_BAD_JSON, "each step must be a JSON object")
        if "seq" in node or "par" in node:
            key = "seq" if "seq" in node else "par"
            children = node[key]
            if not isinstance(children, list) or not children:
                raise ProtocolError(
                    E_OUT_OF_RANGE, f"'{key}' must be a non-empty array of steps"
                )
            parsed = [walk(c, depth + 1) for c in children]
            return Seq(parsed) if key == "seq" else Par(parsed)
        kind = node.get("kind")
        if not isinstance(kind, str) or not kind:
            raise ProtocolError(E_BAD_JSON, "step must have 'kind', 'seq' or 'par'")
        entry = _ACTION_TABLE.get(kind)
        if entry is None:
            raise ProtocolError(E_UNKNOWN_TYPE, f"unknown action kind '{kind}'")
        count[0] += 1
        if count[0] > MAX_STEP_ACTIONS:
            raise ProtocolError(
                E_OUT_OF_RANGE, f"step tree exceeds {MAX_STEP_ACTIONS} actions"
            )
        _, validator = entry
        return Action(kind=kind, args=validator(node, animations, gestures))

    return walk(obj, 1)


def step_channels(step: Step) -> frozenset:
    """Union of channels claimed anywhere in the Step tree (§7)."""
    if isinstance(step, Action):
        return action_channels(step.kind)
    out: frozenset = frozenset()
    for child in step.children:
        out |= step_channels(child)
    return out


def step_contains_kind(step: Step, kind: str) -> bool:
    if isinstance(step, Action):
        return step.kind == kind
    return any(step_contains_kind(c, kind) for c in step.children)


def step_action_count(step: Step) -> int:
    if isinstance(step, Action):
        return 1
    return sum(step_action_count(c) for c in step.children)


def step_to_obj(step: Step) -> dict:
    """Inverse of parse_step (validated form, defaults filled)."""
    if isinstance(step, Action):
        return {"kind": step.kind, **step.args}
    key = "seq" if isinstance(step, Seq) else "par"
    return {key: [step_to_obj(c) for c in step.children]}


# ── Priority / on_busy ──────────────────────────────────────────────────────


def parse_priority(value: Any, default: Optional[int] = None) -> Optional[int]:
    """`"low"|"normal"|"high"` or int 0..79. >= 80 → out_of_range."""
    if value is None:
        return default
    if isinstance(value, str):
        if value in PRIORITY_ALIASES:
            return PRIORITY_ALIASES[value]
        raise ProtocolError(
            E_OUT_OF_RANGE, f"priority '{value}' is not low|normal|high or 0..79"
        )
    if isinstance(value, bool) or not isinstance(value, int):
        raise ProtocolError(E_OUT_OF_RANGE, "priority must be an integer or alias")
    if not (0 <= value <= MAX_CLIENT_PRIORITY):
        raise ProtocolError(
            E_OUT_OF_RANGE,
            f"priority {value} outside 0..{MAX_CLIENT_PRIORITY} (>=80 is reserved)",
        )
    return value


def parse_on_busy(value: Any, default: str = "queue") -> str:
    if value is None:
        return default
    if value not in ON_BUSY_VALUES:
        raise ProtocolError(
            E_OUT_OF_RANGE, f"on_busy '{value}' is not one of {ON_BUSY_VALUES}"
        )
    return str(value)


# ── Client → gateway message payloads ───────────────────────────────────────


@dataclass
class HelloMsg:
    protocol: int
    name: str
    kind: str
    version: str = ""
    priority: int = 50
    subscribe: tuple = ()
    abort_on_disconnect: bool = False


def parse_hello(payload: Mapping) -> HelloMsg:
    protocol = payload.get("protocol")
    if isinstance(protocol, bool) or not isinstance(protocol, int):
        raise ProtocolError(E_BAD_JSON, "hello.protocol must be an integer")
    client = payload.get("client")
    if not isinstance(client, dict):
        raise ProtocolError(E_BAD_JSON, "hello.client must be an object")
    name = client.get("name")
    kind = client.get("kind")
    if not isinstance(name, str) or not name:
        raise ProtocolError(E_BAD_JSON, "hello.client.name is required")
    if not isinstance(kind, str) or not kind:
        raise ProtocolError(E_BAD_JSON, "hello.client.kind is required")
    priority = parse_priority(payload.get("priority"), 50)
    subscribe = payload.get("subscribe") or []
    if not isinstance(subscribe, list):
        raise ProtocolError(E_BAD_JSON, "hello.subscribe must be an array")
    return HelloMsg(
        protocol=protocol,
        name=name,
        kind=kind,
        version=str(client.get("version", "")),
        priority=int(priority),  # type: ignore[arg-type]
        subscribe=tuple(str(s) for s in subscribe),
        abort_on_disconnect=bool(payload.get("abort_on_disconnect", False)),
    )


@dataclass
class ActMsg:
    step: Step
    priority: Optional[int]  # None → use the connection's hello.priority
    on_busy: str = "queue"
    tag: Optional[str] = None


def parse_act(
    payload: Mapping,
    *,
    animations: Optional[Iterable[str]] = None,
    gestures: Optional[Iterable[str]] = None,
) -> ActMsg:
    if "do" not in payload:
        raise ProtocolError(E_BAD_JSON, "act.do is required")
    step = parse_step(payload["do"], animations=animations, gestures=gestures)
    tag = payload.get("tag")
    if tag is not None and not isinstance(tag, str):
        raise ProtocolError(E_BAD_JSON, "act.tag must be a string")
    return ActMsg(
        step=step,
        priority=parse_priority(payload.get("priority"), None),
        on_busy=parse_on_busy(payload.get("on_busy")),
        tag=tag,
    )


@dataclass
class EventMsg:
    name: str
    params: dict = field(default_factory=dict)


def parse_event(payload: Mapping) -> EventMsg:
    name = payload.get("name")
    if not isinstance(name, str) or not name:
        raise ProtocolError(E_BAD_JSON, "event.name is required")
    params = payload.get("params") or {}
    if not isinstance(params, dict):
        raise ProtocolError(E_BAD_JSON, "event.params must be an object")
    return EventMsg(name=name, params=dict(params))


@dataclass
class MouthMsg:
    openness: float


def parse_mouth(payload: Mapping) -> MouthMsg:
    """`mouth` frame (§5.13): a single viseme sample, fire-and-forget.

    Deliberately minimal — this frame is sent at speech rate (30–60 Hz), so
    it carries no duration, no priority and gets no ack.
    """
    return MouthMsg(
        openness=_num(payload, "openness", None, 0.0, 1.0, kind="mouth", required=True)
    )


@dataclass
class CancelMsg:
    target: str


def parse_cancel(payload: Mapping) -> CancelMsg:
    target = payload.get("target")
    if not isinstance(target, str) or not target:
        raise ProtocolError(E_BAD_JSON, "cancel.target is required")
    return CancelMsg(target=target)


@dataclass
class EstopMsg:
    engage: bool
    reason: str = ""


def parse_estop(payload: Mapping) -> EstopMsg:
    engage = payload.get("engage")
    if not isinstance(engage, bool):
        raise ProtocolError(E_BAD_JSON, "estop.engage must be a boolean")
    return EstopMsg(engage=engage, reason=str(payload.get("reason", "")))


@dataclass
class StateGetMsg:
    fields: Optional[tuple] = None  # None = everything


def parse_state_get(payload: Mapping) -> StateGetMsg:
    fields = payload.get("fields")
    if fields is None:
        return StateGetMsg(fields=None)
    if not isinstance(fields, list):
        raise ProtocolError(E_BAD_JSON, "state.get fields must be an array")
    return StateGetMsg(fields=tuple(str(f) for f in fields))


@dataclass
class LockMsg:
    scopes: tuple
    ttl_s: float = 30.0


def parse_lock(payload: Mapping) -> LockMsg:
    scopes = payload.get("scopes")
    if not isinstance(scopes, list) or not scopes:
        raise ProtocolError(E_BAD_JSON, "lock.scopes must be a non-empty array")
    chans = tuple(channel_from_name(str(s)) for s in scopes)
    ttl = payload.get("ttl_s", 30)
    if not _is_number(ttl) or float(ttl) <= 0 or float(ttl) > MAX_LOCK_TTL_S:
        raise ProtocolError(
            E_OUT_OF_RANGE, f"lock.ttl_s must be in (0, {int(MAX_LOCK_TTL_S)}]"
        )
    return LockMsg(scopes=chans, ttl_s=float(ttl))


@dataclass
class UnlockMsg:
    scopes: tuple


def parse_unlock(payload: Mapping) -> UnlockMsg:
    scopes = payload.get("scopes")
    if not isinstance(scopes, list) or not scopes:
        raise ProtocolError(E_BAD_JSON, "unlock.scopes must be a non-empty array")
    return UnlockMsg(scopes=tuple(channel_from_name(str(s)) for s in scopes))


@dataclass
class PongMsg:
    ref: Optional[str]


def parse_pong(payload: Mapping) -> PongMsg:
    ref = payload.get("ref")
    return PongMsg(ref=ref if isinstance(ref, str) else None)


CLIENT_MESSAGE_TYPES = (
    "hello", "act", "event", "mouth", "cancel", "estop", "state.get",
    "lock", "unlock", "ping", "pong",
)
