"""Action runners for every MPP/1 action kind, plus the Step executor.

Each runner is ``async def run(ctx: ActionContext, args: dict)`` where
``args`` has already been validated (defaults filled) by
:func:`maki_puppet.protocol.parse_step`.  Runners are duration-aware and
cancellation-safe: they only await ``ctx.sleep`` and feed motion layers via
``ctx.set_motion_layer`` so the engine can release everything they touched.

Motion layer usage:
  * ``expression`` — blink / eyelids / mouth (merge_targets layer)
  * ``gesture``    — look / pose / gesture keyframes / neutral
  * ``tracking``   — track (priority 50, below gesture: a deliberate gesture
    outranks face-following, which is what you want when an act says "look
    left" while someone is standing to the right)
"""

from __future__ import annotations

import asyncio
import logging
from dataclasses import dataclass
from typing import Any, Awaitable, Callable, Dict, FrozenSet, Iterable, List, Tuple

from .. import protocol
from ..motion import joints as joints_mod
from ..protocol import Action, Channel, Par, ProtocolError, Seq, Step
from ..vision.tracker import FaceTracker
from .context import ActionContext

log = logging.getLogger(__name__)

# Keyframe gesture playback rate (targets/s); the MotionService S-curve
# planner interpolates and smooths between them.
GESTURE_TICK_S = 0.05  # ~20 Hz

# How far ahead of the head the eyes are commanded in a `look` (eyes lead).
LOOK_EYES_LEAD_S = 0.08

# Face-tracking control rate. Matches the ROS node's control_min_dt_s (0.05):
# ticking faster only burns cycles, since the tracker rejects them anyway.
TRACK_TICK_S = 0.05  # 20 Hz


def _wire_to_rad(joints: Dict[str, float]) -> Dict[str, float]:
    """Wire joints (normalized / openness) → physical command radians."""
    out: Dict[str, float] = {}
    for name, value in joints.items():
        if name == "eyelids":
            out.update(joints_mod.eyelids_openness_to_rad(value))
        elif name == "mouth":
            out["mouth"] = joints_mod.mouth_openness_to_rad(value)
        else:
            out[name] = joints_mod.norm_to_rad(name, value)
    return out


def _current_eyelid_openness(ctx: ActionContext) -> float:
    pose = ctx.motion.pose_rad()
    if "left_eyelid" in pose:
        return joints_mod.eyelids_rad_to_openness(pose["left_eyelid"])
    return 1.0


# ── Runners ─────────────────────────────────────────────────────────────────


async def run_blink(ctx: ActionContext, args: dict) -> None:
    """Close and reopen the eyelids once (expression layer dip)."""
    total = args["duration_ms"] / 1000.0
    restore = max(_current_eyelid_openness(ctx), 0.6)  # reopen at least mostly
    ctx.set_motion_layer("expression", joints_mod.eyelids_openness_to_rad(0.0))
    await ctx.sleep(total * 0.45)
    ctx.set_motion_layer("expression", joints_mod.eyelids_openness_to_rad(restore))
    await ctx.sleep(total * 0.55)


async def run_eyelids(ctx: ActionContext, args: dict) -> None:
    ctx.set_motion_layer(
        "expression", joints_mod.eyelids_openness_to_rad(args["openness"])
    )
    await ctx.sleep(args["duration_ms"] / 1000.0)


async def run_mouth(ctx: ActionContext, args: dict) -> None:
    ctx.set_motion_layer(
        "expression", {"mouth": joints_mod.mouth_openness_to_rad(args["openness"])}
    )
    await ctx.sleep(args["duration_ms"] / 1000.0)


async def run_look(ctx: ActionContext, args: dict) -> None:
    """Orient toward (pan, tilt): the eyes lead, the head follows."""
    total = args["duration_ms"] / 1000.0
    eye_targets = {
        "eyes_pan": joints_mod.norm_to_rad("eyes_pan", args["pan"]),
        "eyes_tilt": joints_mod.norm_to_rad("eyes_tilt", args["tilt"]),
    }
    ctx.set_motion_layer("gesture", eye_targets)
    if args["eyes_only"]:
        await ctx.sleep(total)
        return
    lead = min(LOOK_EYES_LEAD_S, total)
    await ctx.sleep(lead)
    ctx.set_motion_layer("gesture", {
        "head_pan": joints_mod.norm_to_rad("head_pan", args["pan"]),
        "head_tilt": joints_mod.norm_to_rad("head_tilt", args["tilt"]),
    })
    await ctx.sleep(max(0.0, total - lead))


async def run_pose(ctx: ActionContext, args: dict) -> None:
    ctx.set_motion_layer("gesture", _wire_to_rad(args["joints"]))
    await ctx.sleep(args["duration_ms"] / 1000.0)


def _keyframe_tracks(keyframes: List[dict]) -> Dict[str, List[Tuple[float, float]]]:
    """Per-joint (t_s, value) polylines from the keyframe list."""
    tracks: Dict[str, List[Tuple[float, float]]] = {}
    for kf in keyframes:
        t = float(kf["t_ms"]) / 1000.0
        for joint, value in kf["joints"].items():
            tracks.setdefault(joint, []).append((t, float(value)))
    return tracks


def _sample_track(track: List[Tuple[float, float]], t: float) -> float:
    """Linear interpolation over one joint's keyframe polyline."""
    if t <= track[0][0]:
        return track[0][1]
    for (t0, v0), (t1, v1) in zip(track, track[1:]):
        if t <= t1:
            if t1 <= t0:
                return v1
            f = (t - t0) / (t1 - t0)
            return v0 + f * (v1 - v0)
    return track[-1][1]


def _scale_wire_value(joint: str, value: float, intensity: float) -> float:
    """Scale a normalized wire value's amplitude about the joint's wire
    neutral, clamping the result into the joint's wire range."""
    lo, hi, neutral = protocol.WIRE_JOINTS.get(joint, (-1.0, 1.0, 0.0))
    scaled = neutral + (value - neutral) * intensity
    return min(max(scaled, lo), hi)


async def run_gesture(ctx: ActionContext, args: dict) -> None:
    """Keyframe player: ~20 Hz linear-interp targets onto the gesture layer."""
    try:
        keyframes = ctx.get_gesture(args["name"])
    except KeyError:
        raise ProtocolError(
            protocol.E_UNKNOWN_GESTURE, f"no gesture '{args['name']}'"
        )
    tracks = _keyframe_tracks(keyframes)
    if not tracks:
        return
    duration = max(t for track in tracks.values() for t, _ in track)
    intensity = args["intensity"]

    for _ in range(args["repeat"]):
        start = ctx.clock()
        while True:
            t = ctx.clock() - start
            sample_t = min(t, duration)
            targets = {
                joint: _scale_wire_value(joint, _sample_track(track, sample_t), intensity)
                for joint, track in tracks.items()
            }
            ctx.set_motion_layer("gesture", _wire_to_rad(targets))
            if t >= duration:
                break
            await ctx.sleep(min(GESTURE_TICK_S, duration - t))


async def run_led(ctx: ActionContext, args: dict) -> None:
    if "animation" in args:
        try:
            ctx.led.set_animation(args["animation"])
        except KeyError:
            raise ProtocolError(
                protocol.E_UNKNOWN_ANIMATION, f"no animation '{args['animation']}'"
            )
    else:
        c = args["color"]
        ctx.led.set_color(c["r"], c["g"], c["b"])


async def run_say(ctx: ActionContext, args: dict) -> None:
    """TTS is deferred. Wire acts containing `say` are rejected whole before
    execution (tts_unavailable); this runner is only reachable from
    choreographies, where `say` steps are skipped with a warning (§10.5)."""
    log.warning("say step skipped (TTS deferred): %r", args.get("text", ""))


async def run_wait(ctx: ActionContext, args: dict) -> None:
    await ctx.sleep(args["duration_ms"] / 1000.0)


async def run_neutral(ctx: ActionContext, args: dict) -> None:
    """Home pose, then release the layer (subsequent idle takes over).

    The mouth is excluded: it is driven externally by the viseme stream, so
    homing it here would snap it shut mid-word whenever a choreography ends.
    """
    ctx.set_motion_layer("gesture", joints_mod.neutral_pose_rad(exclude=("mouth",)))
    await ctx.sleep(args["duration_ms"] / 1000.0)
    ctx.motion.release_layer("gesture")


async def run_posture(ctx: ActionContext, args: dict) -> None:
    """Install (or clear) the sustained baseline posture.

    Deliberately bypasses ``ctx.set_motion_layer``: that records the layer in
    ``touched_layers`` so the engine releases it when the performance ends,
    which is exactly what a posture must NOT do — it outlives the act that set
    it and persists until something clears it.
    """
    if args["clear"]:
        ctx.motion.clear_posture()
        return
    ctx.motion.set_posture(_wire_to_rad(args["joints"]))


async def run_track(ctx: ActionContext, args: dict) -> None:
    """Follow a face for ``duration_ms``, feeding the ``tracking`` layer.

    Runs the ported control law at ``TRACK_TICK_S`` against whatever
    VisionService has most recently seen. Frames where no face is current
    (either none detected, or the detection has aged past the service's
    ``lost_timeout_s``) simply hold the last targets — the layer's own
    claim_timeout is what eventually hands the joints back to idle, so a
    person stepping out of frame produces a graceful drift home rather than a
    snap.

    Errors, rather than silently doing nothing, when no camera is wired up:
    an act that asked to track and got a still robot is a bug worth surfacing.
    """
    if ctx.vision is None:
        raise ProtocolError(
            protocol.E_VISION_UNAVAILABLE,
            "face tracking unavailable (no vision service)",
        )
    tracker = FaceTracker(ctx.tracker_config, clock=ctx.clock)
    eyes_only = args["eyes_only"]
    deadline = ctx.clock() + args["duration_ms"] / 1000.0
    while True:
        now = ctx.clock()
        if now >= deadline:
            return
        face = ctx.vision.latest_face(now)
        output = tracker.update(face, ctx.motion.pose_rad(), now)
        if output is not None:
            ctx.set_motion_layer("tracking", output.as_targets(eyes_only=eyes_only))
        await ctx.sleep(min(TRACK_TICK_S, deadline - now))


Runner = Callable[[ActionContext, dict], Awaitable[None]]


@dataclass(frozen=True)
class ActionSpec:
    """Registry entry for one action kind."""

    name: str
    factory: Runner
    channels: FrozenSet[Channel]
    validate: Callable[..., dict]  # protocol.py validator (args, animations, gestures)


def _spec(name: str, factory: Runner) -> ActionSpec:
    channels, validator = protocol._ACTION_TABLE[name]
    return ActionSpec(name=name, factory=factory, channels=channels, validate=validator)


ACTIONS: Dict[str, ActionSpec] = {
    spec.name: spec
    for spec in (
        _spec("blink", run_blink),
        _spec("look", run_look),
        _spec("eyelids", run_eyelids),
        _spec("mouth", run_mouth),
        _spec("pose", run_pose),
        _spec("gesture", run_gesture),
        _spec("led", run_led),
        _spec("say", run_say),
        _spec("wait", run_wait),
        _spec("neutral", run_neutral),
        _spec("posture", run_posture),
        _spec("track", run_track),
    )
}


async def execute_step(
    step: Step, ctx: ActionContext, runners: Dict[str, ActionSpec] = ACTIONS
) -> None:
    """Execute one Step tree: Action | seq (in order) | par (concurrently).

    Cancellation propagates into every running child; a failing par child
    cancels its siblings before the error is re-raised.
    """
    if isinstance(step, Action):
        spec = runners.get(step.kind)
        if spec is None:
            raise ProtocolError(protocol.E_UNKNOWN_TYPE, f"unknown action kind '{step.kind}'")
        await spec.factory(ctx, step.args)
        return
    if isinstance(step, Seq):
        for child in step.children:
            await execute_step(child, ctx, runners)
        return
    if isinstance(step, Par):
        tasks = [
            asyncio.ensure_future(execute_step(child, ctx, runners))
            for child in step.children
        ]
        try:
            await asyncio.gather(*tasks)
        except BaseException:
            for t in tasks:
                t.cancel()
            await asyncio.gather(*tasks, return_exceptions=True)
            raise
        return
    raise ProtocolError(protocol.E_INTERNAL, f"unexecutable step {step!r}")
