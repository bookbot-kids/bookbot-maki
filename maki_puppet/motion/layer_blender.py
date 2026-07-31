"""
Priority-based layer blending engine for the MAKI motion planner.

Each "layer" represents a behavior source (idle, tracking, expression,
gesture, safety).  Every tick the blender:

  1.  Updates per-layer weights (blend-in / blend-out ramps).
  2.  For each joint, selects the highest-priority active layer that has a
      target for that joint.
  3.  During weight transitions (0→1 or 1→0), cosine-interpolates between
      the incoming and outgoing layer.

Joint ownership matrix (reference — enforced by what each behavior node
actually publishes):

+------------+-----------+--------+-----------+--------+--------+
| Layer      | head_pan  | h_tilt | eyes_pan  | e_tilt | eyelid |
+------------+-----------+--------+-----------+--------+--------+
| idle (30)  |    ✓      |   ✓    |    ✓      |   ✓    |   ✓    |
| expression |    –      |   –    |    ✓      |   ✓    |   ✓    |
|   (40)     |           |        |           |        |        |
| tracking   |    ✓      |   ✓    |    ✓ *    |   ✓ *  |   –    |
|   (50)     |           |        |  * may    |  * may |        |
| gesture    |    ✓      |   ✓    |    ✓      |   ✓    |   –    |
|   (55)     |           |        |           |        |        |
| safety(80) |    ✓      |   ✓    |    ✓      |   ✓    |   ✓    |
+------------+-----------+--------+-----------+--------+--------+

The blender does NOT invent ownership — it only blends joints that the
upstream publisher actually includes in its ``JointState`` message.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional


@dataclass
class LayerConfig:
    """Static configuration for a single motion layer."""

    name: str
    default_priority: int = 40
    blend_in_time_s: float = 0.3
    blend_out_time_s: float = 0.5
    claim_timeout_s: float = 2.0
    merge_targets: bool = False
    """When True, ``feed()`` merges new joints into existing targets
    instead of replacing them.  Use for shared-topic layers (e.g. the
    Phase 0/1 passthrough layer where multiple behavior nodes publish
    different joint subsets to the same topic)."""


@dataclass
class LayerState:
    """Runtime state of a single motion layer."""

    config: LayerConfig

    # Current blend weight [0.0, 1.0]
    weight: float = 0.0

    # Per-joint targets from the last received message
    joint_targets: Dict[str, float] = field(default_factory=dict)

    # Per-joint priority overrides (from effort[] field).
    # If empty, config.default_priority is used for all joints.
    joint_priorities: Dict[str, int] = field(default_factory=dict)

    # Wall-clock time of last received message
    last_update_time: float = 0.0

    # True when a message has been received at least once
    _has_data: bool = False

    # Weight ramping direction: +1 = blending in, -1 = blending out
    _ramp_direction: int = 0

    def feed(
        self,
        joint_targets: Dict[str, float],
        joint_priorities: Optional[Dict[str, int]] = None,
        now: Optional[float] = None,
    ) -> None:
        """Update the layer with new joint targets from a ROS message.

        When ``config.merge_targets`` is True, incoming joints are merged
        into the existing targets dict so that different publishers sharing
        the same topic (e.g. passthrough layer in Phase 0/1) do not erase
        each other's joints.
        """
        if self.config.merge_targets and self.joint_targets:
            self.joint_targets.update(joint_targets)
            if joint_priorities:
                self.joint_priorities.update(joint_priorities)
        else:
            self.joint_targets = dict(joint_targets)
            self.joint_priorities = dict(joint_priorities) if joint_priorities else {}
        self.last_update_time = now if now is not None else time.monotonic()
        if not self._has_data:
            self._has_data = True
            self._ramp_direction = 1  # start ramping in

    def priority_for(self, joint_name: str) -> int:
        """Return the effective priority of this layer for a joint."""
        return self.joint_priorities.get(joint_name, self.config.default_priority)

    @property
    def active(self) -> bool:
        """True if the layer has data and a non-zero weight."""
        return self._has_data and self.weight > 0.0


@dataclass
class LayerBlender:
    """Blends multiple motion layers into a single target per joint.

    Usage::

        blender = LayerBlender()
        blender.add_layer(LayerConfig(name="idle", default_priority=30, ...))
        blender.add_layer(LayerConfig(name="tracking", default_priority=50, ...))

        # When a ROS message arrives for a layer:
        blender.feed_layer("tracking", {"head_pan": 0.2, "head_tilt": -0.1})

        # Each tick (50 Hz):
        targets = blender.update(dt=0.02)
        # targets = {"head_pan": 0.2, "head_tilt": -0.1, ...}
    """

    layers: Dict[str, LayerState] = field(default_factory=dict)

    # Fallback positions for joints that have no active layer.
    # Used when an incoming layer is mid-blend-in and needs something to
    # interpolate from.
    _fallback_positions: Dict[str, float] = field(default_factory=dict)

    def add_layer(self, config: LayerConfig) -> None:
        """Register a new layer configuration."""
        self.layers[config.name] = LayerState(config=config)

    def feed_layer(
        self,
        name: str,
        joint_targets: Dict[str, float],
        joint_priorities: Optional[Dict[str, int]] = None,
        now: Optional[float] = None,
    ) -> None:
        """Feed new joint targets into a named layer."""
        layer = self.layers.get(name)
        if layer is None:
            return  # unknown layer — ignore
        layer.feed(joint_targets, joint_priorities, now)
        if layer._ramp_direction <= 0:
            layer._ramp_direction = 1  # (re-)start blend-in

    def clear_layer(self, name: str) -> None:
        """Explicitly release a named layer, triggering immediate blend-out.

        Use this for fast handoffs (e.g. AVERT→ENGAGE) rather than waiting
        for ``claim_timeout_s``.  The layer will cosine-ramp down over its
        ``blend_out_time_s``; timeout-based fade-out remains the fallback
        when a node dies unexpectedly.
        """
        layer = self.layers.get(name)
        if layer is None:
            return
        if layer._has_data and layer._ramp_direction >= 0:
            layer._ramp_direction = -1  # start blend-out

    def set_fallback_positions(self, positions: Dict[str, float]) -> None:
        """Set fallback positions (typically from servo feedback)."""
        self._fallback_positions.update(positions)

    def update(self, dt: float, now: Optional[float] = None) -> Dict[str, float]:
        """Advance all layer weights and compute blended targets.

        Returns a dict of ``{joint_name: target_rad}`` for every joint that
        has at least one active layer.
        """
        t_now = now if now is not None else time.monotonic()

        # ── Update layer weights ──────────────────────────────────────
        for layer in self.layers.values():
            self._update_layer_weight(layer, dt, t_now)

        # ── Collect all joints that any layer targets ─────────────────
        all_joints: set[str] = set()
        for layer in self.layers.values():
            if layer.active:
                all_joints.update(layer.joint_targets.keys())

        # ── Blend per joint ───────────────────────────────────────────
        result: Dict[str, float] = {}
        for joint_name in all_joints:
            blended = self._blend_joint(joint_name, t_now)
            if blended is not None:
                result[joint_name] = blended

        return result

    def get_diagnostics(self, now: Optional[float] = None) -> Dict[str, dict]:
        """Return diagnostic info per layer."""
        t_now = now if now is not None else time.monotonic()
        out = {}
        for name, layer in self.layers.items():
            age = t_now - layer.last_update_time if layer._has_data else -1
            out[name] = {
                "weight": round(layer.weight, 3),
                "has_data": layer._has_data,
                "age_ms": round(age * 1000) if age >= 0 else -1,
                "ramp": layer._ramp_direction,
                "joints": list(layer.joint_targets.keys()),
            }
        return out

    def get_joint_ownership(self, now: Optional[float] = None) -> Dict[str, dict]:
        """Return per-joint ownership diagnostics.

        For each joint that any active layer targets, returns::

            {
                "owner": <layer_name>,
                "priority": <effective_priority>,
                "weight": <owner_weight>,
                "transition": "stable" | "blending_in" | "blending_out",
            }
        """
        t_now = now if now is not None else time.monotonic()
        # Gather all joints across active layers
        all_joints: set[str] = set()
        for layer in self.layers.values():
            if layer.active:
                all_joints.update(layer.joint_targets.keys())

        result: Dict[str, dict] = {}
        for joint_name in sorted(all_joints):
            # Find the highest-priority active layer for this joint
            best_layer: Optional[LayerState] = None
            best_priority = -1
            for layer in self.layers.values():
                if (
                    layer.active
                    and joint_name in layer.joint_targets
                    and layer.weight > 0.0
                ):
                    p = layer.priority_for(joint_name)
                    if p > best_priority:
                        best_priority = p
                        best_layer = layer

            if best_layer is not None:
                if best_layer.weight >= 1.0 and best_layer._ramp_direction >= 0:
                    transition = "stable"
                elif best_layer._ramp_direction > 0:
                    transition = "blending_in"
                elif best_layer._ramp_direction < 0:
                    transition = "blending_out"
                else:
                    transition = "stable"
                result[joint_name] = {
                    "owner": best_layer.config.name,
                    "priority": best_priority,
                    "weight": round(best_layer.weight, 3),
                    "transition": transition,
                }
        return result

    # ── Internal ──────────────────────────────────────────────────────

    def _update_layer_weight(
        self, layer: LayerState, dt: float, now: float
    ) -> None:
        """Ramp layer weight in/out based on config and claim expiry."""
        cfg = layer.config

        if not layer._has_data:
            return  # never received a message — stay at 0

        # Check claim expiry
        age = now - layer.last_update_time
        if age > cfg.claim_timeout_s and layer._ramp_direction >= 0:
            layer._ramp_direction = -1  # start blend-out

        # Ramp weight
        if layer._ramp_direction > 0:
            # Blend in
            if cfg.blend_in_time_s <= 0.0:
                layer.weight = 1.0
            else:
                rate = 1.0 / cfg.blend_in_time_s
                layer.weight = min(1.0, layer.weight + rate * dt)
        elif layer._ramp_direction < 0:
            # Blend out
            if cfg.blend_out_time_s <= 0.0:
                layer.weight = 0.0
            else:
                rate = 1.0 / cfg.blend_out_time_s
                layer.weight = max(0.0, layer.weight - rate * dt)
            if layer.weight <= 0.0:
                layer._ramp_direction = 0
                layer._has_data = False  # fully faded out

    def _blend_joint(self, joint_name: str, now: float) -> Optional[float]:
        """Compute the blended target for a single joint."""
        # Gather layers that have this joint and have weight > 0
        active: List[LayerState] = []
        for layer in self.layers.values():
            if (
                layer.active
                and joint_name in layer.joint_targets
                and layer.weight > 0.0
            ):
                active.append(layer)

        if not active:
            return None

        # Sort by effective priority for this joint (descending)
        active.sort(key=lambda L: L.priority_for(joint_name), reverse=True)

        top = active[0]
        top_target = top.joint_targets[joint_name]

        if top.weight >= 1.0:
            # Fully faded in — top layer owns the joint
            return top_target

        # Partially faded in — interpolate with the next-priority layer
        # or the fallback position.
        if len(active) > 1:
            second_target = active[1].joint_targets[joint_name]
        else:
            second_target = self._fallback_positions.get(joint_name, top_target)

        # Cosine ease for perceptually smooth handoff
        t = 0.5 * (1.0 - math.cos(math.pi * top.weight))
        return second_target * (1.0 - t) + top_target * t
