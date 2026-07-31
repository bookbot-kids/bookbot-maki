"""choreographies.yaml loader/validator, templating, hot-reload.

Semantic events (`event {name, params}`) resolve here into Step trees using
the exact wire Step grammar — protocol.parse_step is the ONE parser for both
paths (PROTOCOL.md §10).
"""

from __future__ import annotations

import asyncio
import json
import logging
import os
import re
from dataclasses import dataclass
from typing import Any, Callable, Dict, Iterable, List, Mapping, Optional

import yaml

from . import protocol
from .protocol import E_UNKNOWN_EVENT, ProtocolError, Step

log = logging.getLogger(__name__)

_TEMPLATE_RE = re.compile(r"\{\{\s*([A-Za-z0-9_]+)\s*(?:\|((?:[^}]|\}(?!\}))*))?\}\}")
_WHOLE_TEMPLATE_RE = re.compile(
    r"^\{\{\s*([A-Za-z0-9_]+)\s*(?:\|((?:[^}]|\}(?!\}))*))?\}\}$"
)


def _coerce_default(text: str) -> Any:
    """Best-effort typed default for whole-value templates ("500" → 500)."""
    try:
        return json.loads(text)
    except (ValueError, TypeError):
        return text


def apply_templates(value: Any, params: Mapping[str, Any]) -> Any:
    """Substitute `{{param}}` / `{{param|default}}` in every string of a
    step tree.  A string that is exactly one template keeps the parameter's
    JSON type (PROTOCOL.md §10.3)."""
    if isinstance(value, str):
        whole = _WHOLE_TEMPLATE_RE.match(value)
        if whole:
            name, default = whole.group(1), whole.group(2)
            if name in params:
                return params[name]
            return _coerce_default(default) if default is not None else ""

        def sub(m: "re.Match[str]") -> str:
            name, default = m.group(1), m.group(2)
            if name in params:
                return str(params[name])
            return default if default is not None else ""

        return _TEMPLATE_RE.sub(sub, value)
    if isinstance(value, dict):
        return {k: apply_templates(v, params) for k, v in value.items()}
    if isinstance(value, list):
        return [apply_templates(v, params) for v in value]
    return value


@dataclass(frozen=True)
class Choreography:
    name: str
    priority: int
    on_busy: str
    raw_steps: tuple          # untemplated step list (implicit top-level seq)
    description: str = ""


@dataclass(frozen=True)
class ResolvedEvent:
    name: str
    step: Step
    priority: int
    on_busy: str


class Choreographer:
    """Owns the parsed choreographies + gesture library with hot-reload.

    ``animations`` is the live LED animation catalog (for validating `led`
    steps).  An invalid file — at load or on reload — never becomes live:
    the previous version stays active and the error is logged.
    """

    def __init__(
        self,
        path: os.PathLike,
        animations: Iterable[str],
        *,
        on_reload: Optional[Callable[[], None]] = None,
    ) -> None:
        self._path = os.fspath(path)
        self._animations = set(animations)
        self.on_reload = on_reload
        self._mtime: Optional[float] = None
        self._watch_task: Optional[asyncio.Task] = None

        self._choreographies: Dict[str, Choreography] = {}
        self._gestures: Dict[str, List[dict]] = {}
        self._ignored: List[str] = []

    # ── Catalogs ───────────────────────────────────────────────────────

    @property
    def event_names(self) -> List[str]:
        """Every event name accepted without unknown_event (welcome.events)."""
        names = list(self._choreographies)
        names.extend(n for n in self._ignored if n not in self._choreographies)
        return names

    @property
    def gesture_names(self) -> List[str]:
        return list(self._gestures)

    @property
    def ignored_events(self) -> List[str]:
        return list(self._ignored)

    def get_gesture(self, name: str) -> List[dict]:
        """Keyframes for a named gesture; raises KeyError if unknown."""
        return self._gestures[name]

    # ── Loading / validation ───────────────────────────────────────────

    def load(self) -> None:
        """Parse + validate the file; raises ValueError if invalid."""
        choreographies, gestures, ignored = self._parse_file(self._path)
        self._choreographies = choreographies
        self._gestures = gestures
        self._ignored = ignored
        try:
            self._mtime = os.stat(self._path).st_mtime
        except OSError:
            self._mtime = None
        log.info(
            "Loaded %d choreographies, %d gestures (%d ignored events) from %s",
            len(choreographies), len(gestures), len(ignored), self._path,
        )

    def _parse_file(self, path: str):
        with open(path, "r", encoding="utf-8") as fh:
            doc = yaml.safe_load(fh)
        if not isinstance(doc, dict):
            raise ValueError(f"{path}: top level must be a mapping")

        defaults = doc.get("defaults") or {}
        try:
            default_priority = protocol.parse_priority(defaults.get("priority"), 50)
            default_on_busy = protocol.parse_on_busy(defaults.get("on_busy"), "replace")
        except ProtocolError as e:
            raise ValueError(f"{path}: defaults: {e.detail or e.code}")

        ignored_raw = doc.get("ignored_events") or []
        if not isinstance(ignored_raw, list):
            raise ValueError(f"{path}: ignored_events must be a list")
        ignored = [str(n) for n in ignored_raw]

        gestures = self._parse_gestures(doc.get("gestures") or {}, path)

        choreographies: Dict[str, Choreography] = {}
        raw_choreos = doc.get("choreographies") or {}
        if not isinstance(raw_choreos, dict):
            raise ValueError(f"{path}: choreographies must be a mapping")
        for name, spec in raw_choreos.items():
            if not isinstance(spec, dict):
                raise ValueError(f"{path}: choreography '{name}' must be a mapping")
            steps = spec.get("steps")
            if not isinstance(steps, list) or not steps:
                raise ValueError(
                    f"{path}: choreography '{name}' needs a non-empty steps list"
                )
            try:
                priority = protocol.parse_priority(
                    spec.get("priority"), default_priority
                )
                on_busy = protocol.parse_on_busy(spec.get("on_busy"), default_on_busy)
                # Validate with defaults substituted for absent params —
                # catches bad joints/animations/gesture refs/Step limits now.
                self._build_step(steps, {}, gestures)
            except ProtocolError as e:
                raise ValueError(
                    f"{path}: choreography '{name}': {e.detail or e.code}"
                )
            choreographies[str(name)] = Choreography(
                name=str(name),
                priority=int(priority),  # type: ignore[arg-type]
                on_busy=on_busy,
                raw_steps=tuple(steps),
                description=str(spec.get("description", "")),
            )
        return choreographies, gestures, ignored

    def _parse_gestures(self, raw: Mapping, path: str) -> Dict[str, List[dict]]:
        if not isinstance(raw, Mapping):
            raise ValueError(f"{path}: gestures must be a mapping")
        gestures: Dict[str, List[dict]] = {}
        for name, spec in raw.items():
            keyframes = (spec or {}).get("keyframes")
            if not isinstance(keyframes, list) or not keyframes:
                raise ValueError(
                    f"{path}: gesture '{name}' needs a non-empty keyframes list"
                )
            prev_t = None
            cleaned: List[dict] = []
            for i, kf in enumerate(keyframes):
                if not isinstance(kf, dict):
                    raise ValueError(f"{path}: gesture '{name}' keyframe {i} malformed")
                t_ms = kf.get("t_ms")
                if isinstance(t_ms, bool) or not isinstance(t_ms, (int, float)) or t_ms < 0:
                    raise ValueError(
                        f"{path}: gesture '{name}' keyframe {i}: t_ms must be >= 0"
                    )
                if i == 0 and t_ms != 0:
                    raise ValueError(
                        f"{path}: gesture '{name}': first keyframe must be at t_ms 0"
                    )
                if prev_t is not None and t_ms <= prev_t:
                    raise ValueError(
                        f"{path}: gesture '{name}': t_ms must be strictly increasing"
                    )
                prev_t = t_ms
                joints = kf.get("joints")
                if not isinstance(joints, dict) or not joints:
                    raise ValueError(
                        f"{path}: gesture '{name}' keyframe {i}: joints required"
                    )
                for joint, value in joints.items():
                    rng = protocol.WIRE_JOINTS.get(joint)
                    if rng is None:
                        raise ValueError(
                            f"{path}: gesture '{name}': unknown joint '{joint}'"
                        )
                    lo, hi, _ = rng
                    if isinstance(value, bool) or not isinstance(value, (int, float)) \
                            or not (lo <= value <= hi):
                        raise ValueError(
                            f"{path}: gesture '{name}': {joint}={value!r} "
                            f"outside [{lo}, {hi}]"
                        )
                cleaned.append(
                    {"t_ms": int(t_ms), "joints": {j: float(v) for j, v in joints.items()}}
                )
            gestures[str(name)] = cleaned
        return gestures

    def _build_step(
        self, raw_steps: Iterable, params: Mapping[str, Any],
        gestures: Mapping[str, Any],
    ) -> Step:
        """Template + parse a choreography's step list (implicit seq)."""
        templated = apply_templates(list(raw_steps), params)
        obj = templated[0] if len(templated) == 1 else {"seq": templated}
        return protocol.parse_step(
            obj, animations=self._animations, gestures=gestures
        )

    # ── Resolution ─────────────────────────────────────────────────────

    def resolve(self, name: str, params: Mapping[str, Any]) -> Optional[ResolvedEvent]:
        """Event name → ResolvedEvent, or None for ignored (no-op) events.

        Raises ProtocolError(unknown_event) for names in neither list; step
        parse errors after templating propagate as ProtocolError too.
        """
        choreo = self._choreographies.get(name)
        if choreo is None:
            if name in self._ignored:
                return None
            raise ProtocolError(
                E_UNKNOWN_EVENT, f"no choreography '{name}'; see welcome.events"
            )
        step = self._build_step(choreo.raw_steps, params, self._gestures)
        return ResolvedEvent(
            name=name, step=step, priority=choreo.priority, on_busy=choreo.on_busy
        )

    # ── Hot reload ─────────────────────────────────────────────────────

    def try_reload(self) -> bool:
        """Re-validate and atomically swap. Invalid file → keep old, log."""
        try:
            choreographies, gestures, ignored = self._parse_file(self._path)
        except Exception as e:
            log.error("choreographies reload rejected (keeping old): %s", e)
            return False
        self._choreographies = choreographies
        self._gestures = gestures
        self._ignored = ignored
        log.info("choreographies hot-reloaded from %s", self._path)
        if self.on_reload is not None:
            try:
                self.on_reload()
            except Exception:  # pragma: no cover - defensive
                log.exception("choreography on_reload callback failed")
        return True

    def start_watch(self, interval_s: float = 1.0) -> None:
        if self._watch_task is None or self._watch_task.done():
            self._watch_task = asyncio.ensure_future(self._watch(interval_s))

    def stop_watch(self) -> None:
        if self._watch_task is not None:
            self._watch_task.cancel()
            self._watch_task = None

    async def _watch(self, interval_s: float) -> None:
        while True:
            await asyncio.sleep(interval_s)
            try:
                mtime = os.stat(self._path).st_mtime
            except OSError:
                continue
            if self._mtime is None or mtime != self._mtime:
                self._mtime = mtime
                self.try_reload()
