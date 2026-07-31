"""MotionService — the 50 Hz motion loop for the MAKI puppet stack.

Forked from maki-rpi5-v0.2.13-rc.4/src/maki_motion/maki_motion/motion_planner_node.py
(the ``_tick()`` pipeline); guards preserved — see plan checklist:

* tick order: layer blend → target EMA → deadband → LOAD-REACTIVE DAMPENING →
  S-curve step → single write (its absence causes head oscillation under load);
* dt clamping to 0.1 s (no huge jumps after pauses);
* feedback seeding: planners/deadbands reset from servo feedback on start and
  on first feedback, with the choose_seed_position fallback chain;
* idle micro-breathing: deadband-exempt additive sine, only applied to joints
  some layer currently claims, suppressed while the planner is chasing;
* e-stop = freeze-and-hold (planner reset + hold-position writes each tick);
  disengage reseeds planners from feedback and resumes;
* release-layer cosine blend-out toward feedback via LayerBlender fallback
  positions (verified: LayerBlender._blend_joint interpolates the fading top
  layer against the next layer or the feedback fallback).

Intentionally retired from the vendored node (puppet mode has no publishers
for them): the system-ready/brain-warm boot gate, resource-mode rate scaling,
mood/engagement/orchestrator intensity scaling (velocity_scale carries the
load-dampen factor only), velocity feedforward (was default-off), and the ROS
passthrough layer.
"""

from __future__ import annotations

import logging
import math
import threading
import time
import traceback
from typing import Any, Dict, List, Mapping, Optional, Tuple

from . import joints as joints_mod
from .deadband import DeadbandFilter
from .layer_blender import LayerBlender, LayerConfig
from .s_curve import SCurvePlanner, profile_from_dps

logger = logging.getLogger(__name__)

# Joint names recognized by the planner (must match ServoBus)
ALL_JOINTS = [
    "head_pan",
    "head_tilt",
    "eyes_pan",
    "eyes_tilt",
    "left_eyelid",
    "right_eyelid",
    "mouth",
]

# Default S-curve profiles (degrees/s units — converted at init).
# Values are the SHIPPED vendored maki_motion.yaml servo_profiles (which
# override the code defaults in motion_planner_node.py — config beats code).
DEFAULT_SERVO_PROFILES: Dict[str, Dict[str, Any]] = {
    "head_pan":      {"planning_enabled": True,  "max_velocity_dps": 140.0,  "max_acceleration_dps2": 260.0,  "max_jerk_dps3": 500.0,   "deadband_rad": 0.025, "target_smoothing_alpha": 0.25, "settling_radius": 0.04, "settling_stiffness": 25.0, "short_move_radius": 0.15, "short_move_min_scale": 0.15, "load_dampening_threshold": 0.15, "load_dampening_floor": 0.12},
    "head_tilt":     {"planning_enabled": True,  "max_velocity_dps": 118.0,  "max_acceleration_dps2": 230.0,  "max_jerk_dps3": 400.0,   "deadband_rad": 0.025, "target_smoothing_alpha": 0.25, "settling_radius": 0.04, "settling_stiffness": 25.0, "short_move_radius": 0.15, "short_move_min_scale": 0.15, "load_dampening_threshold": 0.12, "load_dampening_floor": 0.12},
    "eyes_pan":      {"planning_enabled": True,  "max_velocity_dps": 400.0,  "max_acceleration_dps2": 1600.0, "max_jerk_dps3": 8000.0,  "deadband_rad": 0.008, "target_smoothing_alpha": 0.0,  "settling_radius": 0.02, "settling_stiffness": 25.0, "short_move_radius": 0.0,  "short_move_min_scale": 0.15, "load_dampening_threshold": 1.0,  "load_dampening_floor": 0.20},
    "eyes_tilt":     {"planning_enabled": True,  "max_velocity_dps": 400.0,  "max_acceleration_dps2": 1600.0, "max_jerk_dps3": 8000.0,  "deadband_rad": 0.008, "target_smoothing_alpha": 0.0,  "settling_radius": 0.02, "settling_stiffness": 25.0, "short_move_radius": 0.0,  "short_move_min_scale": 0.15, "load_dampening_threshold": 1.0,  "load_dampening_floor": 0.20},
    "left_eyelid":   {"planning_enabled": True,  "max_velocity_dps": 1500.0, "max_acceleration_dps2": 8000.0, "max_jerk_dps3": 50000.0, "deadband_rad": 0.005, "target_smoothing_alpha": 0.0,  "settling_radius": 0.0,  "settling_stiffness": 25.0, "short_move_radius": 0.0,  "short_move_min_scale": 0.15, "load_dampening_threshold": 1.0,  "load_dampening_floor": 0.20},
    "right_eyelid":  {"planning_enabled": True,  "max_velocity_dps": 1500.0, "max_acceleration_dps2": 8000.0, "max_jerk_dps3": 50000.0, "deadband_rad": 0.005, "target_smoothing_alpha": 0.0,  "settling_radius": 0.0,  "settling_stiffness": 25.0, "short_move_radius": 0.0,  "short_move_min_scale": 0.15, "load_dampening_threshold": 1.0,  "load_dampening_floor": 0.20},
    "mouth":         {"planning_enabled": False, "max_velocity_dps": 200.0,  "max_acceleration_dps2": 800.0,  "max_jerk_dps3": 4000.0,  "deadband_rad": 0.010, "target_smoothing_alpha": 0.0,  "settling_radius": 0.0,  "settling_stiffness": 25.0, "short_move_radius": 0.0,  "short_move_min_scale": 0.15, "load_dampening_threshold": 1.0,  "load_dampening_floor": 0.20},
}

# Default layer definitions. Priorities and claim timeouts follow the pinned
# MPP/1 contract (safety=100 so it beats any wire priority, which caps at 79;
# tracking claim_timeout 1.0 s); blend times come from the vendored
# maki_motion.yaml.
DEFAULT_LAYERS: Dict[str, Dict[str, Any]] = {
    "idle":       {"priority": 30,  "blend_in_time_s": 0.5,  "blend_out_time_s": 0.8, "claim_timeout_s": 5.0, "merge_targets": False},
    "expression": {"priority": 40,  "blend_in_time_s": 0.2,  "blend_out_time_s": 0.3, "claim_timeout_s": 1.5, "merge_targets": True},
    "tracking":   {"priority": 50,  "blend_in_time_s": 0.3,  "blend_out_time_s": 0.5, "claim_timeout_s": 1.0, "merge_targets": False},
    "gesture":    {"priority": 55,  "blend_in_time_s": 0.25, "blend_out_time_s": 0.5, "claim_timeout_s": 1.0, "merge_targets": False},
    "safety":     {"priority": 100, "blend_in_time_s": 0.0,  "blend_out_time_s": 0.0, "claim_timeout_s": 5.0, "merge_targets": False},
}

# How fast smoothed load readings update (guard preserved: EMA-filtered to
# avoid spiky load→velocity scaling; was 0.15, raised to 0.40 upstream).
_LOAD_EMA_ALPHA = 0.40


def choose_seed_position(
    feedback_position: Optional[float],
    last_commanded_position: Optional[float],
    incoming_target: float,
) -> float:
    """Choose a safe planner seed position before fast feedback is available.

    Priority order:
      1. actual servo feedback (if available),
      2. last command written by this service,
      3. current incoming target.

    The final fallback avoids planning the first command from 0.0 rad.
    """
    if feedback_position is not None:
        return feedback_position
    if last_commanded_position is not None:
        return last_commanded_position
    return incoming_target


class MotionService:
    """Blends layered joint targets and streams smooth commands to the bus.

    ``bus`` is a ServoBus/SimServoBus (pinned interface: ``joint_names``,
    ``read_feedback()``, ``write_targets_rad()``).  ``config`` is
    ``puppet.yaml["motion"]``.  All targets are command RADIANS; normalized
    wire units are converted by the gateway via :mod:`.joints`.
    """

    def __init__(self, bus: Any, config: Mapping[str, Any]) -> None:
        self._bus = bus
        cfg = dict(config or {})
        self._rate_hz = float(cfg.get("rate_hz", 50.0))
        self._load_ema_alpha = float(cfg.get("load_ema_alpha", _LOAD_EMA_ALPHA))

        self._joint_names: List[str] = list(bus.joint_names)
        joint_set = set(self._joint_names)

        # ── Build S-curve planners and deadband filters ────────────────
        profiles_cfg: Mapping[str, Mapping[str, Any]] = cfg.get("profiles") or {}
        self._planners: Dict[str, SCurvePlanner] = {}
        self._deadbands: Dict[str, DeadbandFilter] = {}
        self._target_alphas: Dict[str, float] = {}     # per-joint target EMA alpha
        self._smoothed_targets: Dict[str, float] = {}  # EMA state
        self._planning_joints: set[str] = set()        # joints with S-curve enabled

        # Load-reactive dampening: per-joint config + smoothed load state
        self._load_dampening_threshold: Dict[str, float] = {}
        self._load_dampening_floor: Dict[str, float] = {}
        self._smoothed_loads: Dict[str, float] = {}

        for joint_name in self._joint_names:
            p = dict(DEFAULT_SERVO_PROFILES.get(joint_name, DEFAULT_SERVO_PROFILES["mouth"]))
            p.update(profiles_cfg.get(joint_name, {}))

            profile = profile_from_dps(
                float(p["max_velocity_dps"]),
                float(p["max_acceleration_dps2"]),
                float(p["max_jerk_dps3"]),
                settling_radius=float(p["settling_radius"]),
                settling_stiffness=float(p["settling_stiffness"]),
                short_move_radius=float(p["short_move_radius"]),
                short_move_min_scale=float(p["short_move_min_scale"]),
            )
            self._planners[joint_name] = SCurvePlanner(profile=profile)
            self._deadbands[joint_name] = DeadbandFilter(deadband_rad=float(p["deadband_rad"]))
            self._target_alphas[joint_name] = min(max(float(p["target_smoothing_alpha"]), 0.0), 1.0)
            self._load_dampening_threshold[joint_name] = float(p["load_dampening_threshold"])
            self._load_dampening_floor[joint_name] = float(p["load_dampening_floor"])
            self._smoothed_loads[joint_name] = 0.0

            if bool(p.get("planning_enabled", False)):
                self._planning_joints.add(joint_name)

        # ── Build layer blender ────────────────────────────────────────
        layers_cfg: Mapping[str, Mapping[str, Any]] = cfg.get("layers") or {}
        layer_defs: Dict[str, Dict[str, Any]] = {
            name: dict(d) for name, d in DEFAULT_LAYERS.items()
        }
        for name, d in layers_cfg.items():
            layer_defs.setdefault(name, {}).update(d)

        self._blender = LayerBlender()
        for name, d in layer_defs.items():
            self._blender.add_layer(LayerConfig(
                name=name,
                default_priority=int(d.get("priority", d.get("default_priority", 40))),
                blend_in_time_s=float(d.get("blend_in_time_s", 0.3)),
                blend_out_time_s=float(d.get("blend_out_time_s", 0.5)),
                claim_timeout_s=float(d.get("claim_timeout_s", 2.0)),
                merge_targets=bool(d.get("merge_targets", False)),
            ))
        self._layer_names = set(layer_defs)

        # ── Idle micro-breathing (deadband-exempt additive sine) ──────
        b = dict(cfg.get("idle_breathing") or {})
        self._breathing_enabled = bool(b.get("enabled", True))
        global_amp = math.radians(float(b.get("amplitude_deg", 0.3)))
        global_freq = float(b.get("frequency_hz", 0.2))
        self._breathing_phase = float(b.get("phase_offset_s", 0.0))
        per_joint: Mapping[str, Mapping[str, Any]] = b.get("per_joint") or {}
        self._breathing_per_joint: Dict[str, Tuple[float, float]] = {}
        for jn in b.get("joints", ["head_tilt", "head_pan"]):
            override = per_joint.get(jn, {})
            amp = math.radians(float(override["amplitude_deg"])) if "amplitude_deg" in override else global_amp
            freq = float(override.get("frequency_hz", global_freq))
            if jn in joint_set:
                self._breathing_per_joint[jn] = (amp, freq)

        # ── Runtime state ──────────────────────────────────────────────
        self._lock = threading.RLock()
        self._pending: List[Tuple[str, str, Dict[str, float]]] = []  # staged layer ops
        self._feedback: Dict[str, Any] = {}            # latest JointFeedback per joint
        self._feedback_positions: Dict[str, float] = {}
        self._feedback_received = False
        self._last_commanded: Dict[str, float] = {}
        self._seeded_joints: set[str] = set()
        self._estop = False
        self._last_tick_time = time.monotonic()

        self._thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()

    # ── Lifecycle ──────────────────────────────────────────────────────

    def start(self) -> None:
        """Seed planners from servo feedback and spawn the tick thread."""
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop_event = threading.Event()
        self._seed_from_feedback()
        self._last_tick_time = time.monotonic()
        self._thread = threading.Thread(
            target=self._run, name="maki-motion", daemon=True
        )
        self._thread.start()

    def stop(self) -> None:
        """Stop the tick thread. Idempotent; does not touch servo torque."""
        self._stop_event.set()
        thread = self._thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=2.0)
        self._thread = None

    # ── Public API (thread-safe) ───────────────────────────────────────

    def set_layer(self, layer: str, targets: Dict[str, float]) -> None:
        """Feed radian targets to a named layer (partial joint sets fine).

        Unknown joints are ignored; unknown layers raise ``KeyError``.
        Targets are clamped to each joint's safe range so the planner never
        chases an unreachable position (ServoBus clamps ticks again at the
        single hardware choke point).
        """
        if layer not in self._layer_names:
            raise KeyError(f"unknown motion layer: {layer!r}")
        filtered: Dict[str, float] = {}
        for name, value in targets.items():
            if name not in self._planners:
                continue
            rad = float(value)
            if name in joints_mod.JOINTS:
                rad = joints_mod.clamp_rad(name, rad)
            filtered[name] = rad
        if not filtered:
            return
        with self._lock:
            self._pending.append(("feed", layer, filtered))

    def release_layer(self, layer: str) -> None:
        """Explicitly release a layer — it cosine-blends out toward the next
        layer or the servo-feedback fallback (LayerBlender semantics)."""
        if layer not in self._layer_names:
            raise KeyError(f"unknown motion layer: {layer!r}")
        with self._lock:
            self._pending.append(("clear", layer, {}))

    def pose_rad(self) -> Dict[str, float]:
        """Latest servo feedback positions (command radians)."""
        with self._lock:
            return dict(self._feedback_positions)

    def estop(self, engaged: bool) -> None:
        """Engage/disengage freeze-and-hold.

        Engage: freeze all planners at their current position; the tick keeps
        writing the frozen hold pose so servos actively maintain posture
        (soft stop).  Disengage: reseed planners from feedback and resume.
        """
        with self._lock:
            was_active = self._estop
            self._estop = bool(engaged)
            if engaged and not was_active:
                # Freeze all planners at current position
                for joint_name in self._planning_joints:
                    planner = self._planners[joint_name]
                    planner.reset(planner.state.position, 0.0)
                logger.warning("Emergency stop ACTIVE: planners frozen")
            elif not engaged and was_active:
                # Resume from current feedback positions
                for joint_name in self._planning_joints:
                    if joint_name in self._feedback_positions:
                        self._planners[joint_name].reset(
                            self._feedback_positions[joint_name], velocity=0.0
                        )
                logger.info("Emergency stop CLEARED: planners resumed")

    @property
    def estopped(self) -> bool:
        return self._estop

    # ── Internals ──────────────────────────────────────────────────────

    def _run(self) -> None:
        period = 1.0 / max(1.0, self._rate_hz)
        next_deadline = time.monotonic()
        while not self._stop_event.is_set():
            try:
                self._tick_once()
            except Exception:
                logger.error("Motion tick error:\n%s", traceback.format_exc())
            next_deadline += period
            delay = next_deadline - time.monotonic()
            if delay > 0:
                self._stop_event.wait(delay)
            else:
                next_deadline = time.monotonic()  # fell behind — resync, don't spiral

    def _seed_from_feedback(self) -> None:
        """Initialize planners/deadbands/fallbacks from actual servo positions."""
        try:
            feedback = self._bus.read_feedback()
        except Exception:
            logger.warning("Feedback seed read failed:\n%s", traceback.format_exc())
            return
        with self._lock:
            self._ingest_feedback(feedback)

    def _ingest_feedback(self, feedback: Mapping[str, Any]) -> None:
        """Track positions and EMA-smoothed loads; first feedback seeds state."""
        for name, fb in feedback.items():
            if name not in self._planners:
                continue
            self._feedback[name] = fb
            self._feedback_positions[name] = float(fb.position_rad)
            # Load tracking: normalized present-load, EMA-filtered so spiky
            # readings don't whipsaw the dampening scale.
            raw_load = abs(float(fb.load))
            prev = self._smoothed_loads.get(name, 0.0)
            self._smoothed_loads[name] = prev + self._load_ema_alpha * (raw_load - prev)

        if not self._feedback_received and self._feedback_positions:
            self._feedback_received = True
            for joint_name, pos in self._feedback_positions.items():
                self._planners[joint_name].reset(pos)
                self._deadbands[joint_name].reset(pos)
                self._seeded_joints.add(joint_name)
            self._blender.set_fallback_positions(self._feedback_positions)
            logger.info(
                "Initialized from servo feedback: %s",
                list(self._feedback_positions.keys()),
            )

    def _ensure_joint_seeded(self, joint_name: str, target: float) -> None:
        """Initialize planner/deadband state before first feedback arrives."""
        if joint_name in self._seeded_joints:
            return
        if self._feedback_received and joint_name in self._feedback_positions:
            seed_pos = self._feedback_positions[joint_name]
        else:
            seed_pos = choose_seed_position(
                self._feedback_positions.get(joint_name),
                self._last_commanded.get(joint_name),
                target,
            )
        self._planners[joint_name].reset(seed_pos, velocity=0.0)
        self._deadbands[joint_name].reset(seed_pos)
        self._seeded_joints.add(joint_name)

    def _tick_once(self, now: Optional[float] = None) -> None:
        """One pass of the motion pipeline (called at rate_hz by the thread).

        Bus I/O happens outside the state lock; the compute stage runs under
        it so set_layer/estop from other threads stay consistent.
        """
        t_now = time.monotonic() if now is None else now
        dt = t_now - self._last_tick_time
        self._last_tick_time = t_now
        # Clamp dt to avoid huge jumps after pauses
        dt = min(max(dt, 0.0), 0.1)

        try:
            feedback = self._bus.read_feedback()
        except Exception:
            logger.error("Feedback read error:\n%s", traceback.format_exc())
            feedback = {}

        with self._lock:
            output = self._compute(dt, t_now, feedback)

        if output:
            self._bus.write_targets_rad(output)

    def _compute(
        self, dt: float, now: float, feedback: Mapping[str, Any]
    ) -> Optional[Dict[str, float]]:
        """blend → target EMA → deadband → load-dampen → S-curve → targets."""
        self._ingest_feedback(feedback)

        # Consume staged layer ops (thread-safe handoff from set_layer/release)
        pending, self._pending = self._pending, []
        for op, layer, targets in pending:
            if op == "feed":
                self._blender.feed_layer(layer, targets, None, now)
            else:
                self._blender.clear_layer(layer)

        # ── Emergency stop: freeze-and-hold ───────────────────────────
        if self._estop:
            # Keep writing frozen hold positions so servos actively maintain
            # posture (soft stop). Fall back to feedback if nothing was ever
            # commanded.
            if self._last_commanded:
                return dict(self._last_commanded)
            if self._feedback_positions:
                return dict(self._feedback_positions)
            return None

        # Update fallback positions from feedback (release blends toward these)
        if self._feedback_positions:
            self._blender.set_fallback_positions(self._feedback_positions)

        # ── Step 1: Layer blending ─────────────────────────────────────
        blended = self._blender.update(dt, now)
        if not blended:
            return None  # nothing to do

        # ── Step 2–3: Target EMA → deadband → S-curve per joint ───────
        output: Dict[str, float] = {}
        for joint_name, target in blended.items():
            if joint_name not in self._planners:
                continue
            if joint_name in joints_mod.JOINTS:
                target = joints_mod.clamp_rad(joint_name, target)

            planner = self._planners[joint_name]
            deadband = self._deadbands[joint_name]

            if joint_name in self._planning_joints:
                self._ensure_joint_seeded(joint_name, target)

                # Optional per-joint target EMA — smooths noisy targets
                # before the deadband sees them.
                alpha = self._target_alphas.get(joint_name, 0.0)
                if alpha > 0.0:
                    prev = self._smoothed_targets.get(joint_name)
                    smoothed = target if prev is None else prev + alpha * (target - prev)
                    self._smoothed_targets[joint_name] = smoothed
                    target = smoothed

                # Apply deadband filter to the (possibly smoothed) target
                filtered = deadband.update(target)
                if filtered is not None:
                    planner.set_target(filtered)

                # ── Load-reactive dampening ────────────────────────────
                # When servo load exceeds the threshold, linearly reduce the
                # velocity/accel scale from 1.0 down to load_dampening_floor.
                # This breaks the load→position-error→correction→more-load
                # feedback loop that causes oscillation with the heavy head.
                ld_thresh = self._load_dampening_threshold[joint_name]
                ld_floor = self._load_dampening_floor[joint_name]
                load = self._smoothed_loads.get(joint_name, 0.0)
                if load > ld_thresh and ld_thresh < 1.0:
                    # Linear ramp: threshold → 1.0 maps to 1.0 → floor
                    frac = min((load - ld_thresh) / (1.0 - ld_thresh), 1.0)
                    scale = 1.0 - frac * (1.0 - ld_floor)
                else:
                    scale = 1.0
                planner.velocity_scale = scale
                planner.acceleration_scale = scale

                # Step the S-curve planner
                output[joint_name] = planner.update(dt)
            else:
                # Passthrough — no S-curve for this joint
                output[joint_name] = target

        # ── Step 3.5: Idle breathing (deadband-exempt additive) ───────
        # Only touches joints some layer currently claims (they're in
        # `output`), and only when the planner has arrived (idle/settled).
        if self._breathing_enabled and output:
            for joint_name, (amp, freq) in self._breathing_per_joint.items():
                if joint_name not in output:
                    continue
                planner = self._planners.get(joint_name)
                if planner is not None and joint_name in self._planning_joints:
                    if not planner.arrived:
                        continue  # actively chasing a target — suppress
                output[joint_name] += amp * math.sin(
                    2.0 * math.pi * freq * now + self._breathing_phase
                )

        if output:
            self._last_commanded.update(output)
            return output
        return None
