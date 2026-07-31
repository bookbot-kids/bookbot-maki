"""
S-curve (jerk-limited) motion profile for a single servo axis.

Implements a **reactive per-tick** planner rather than pre-computing the full
7-segment trajectory.  At each tick the planner:

  1. Computes the stopping distance from the current velocity using
     jerk-limited deceleration.
  2. If the servo is within the stopping distance of the target, begins
     deceleration.
  3. Otherwise, accelerates (respecting jerk and acceleration limits)
     toward the maximum velocity.

Direction reversals are handled naturally: the planner first decelerates to
zero before accelerating in the new direction.

All units are in **radians**, **radians/s**, **radians/s²**, **radians/s³**
unless stated otherwise.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field


def _sign(x: float) -> float:
    """Return -1.0, 0.0, or 1.0."""
    if x > 0.0:
        return 1.0
    elif x < 0.0:
        return -1.0
    return 0.0


@dataclass
class SCurveState:
    """Instantaneous kinematic state of a single servo axis."""

    position: float = 0.0
    velocity: float = 0.0
    acceleration: float = 0.0


@dataclass
class SCurveProfile:
    """Per-servo S-curve tuning parameters.

    Parameters
    ----------
    max_velocity : float
        Maximum allowed velocity (rad/s).  Must be > 0.
    max_acceleration : float
        Maximum allowed acceleration magnitude (rad/s²).  Must be > 0.
    max_jerk : float
        Maximum allowed jerk magnitude (rad/s³).  Controls the smoothness
        of acceleration changes — lower = smoother but slower response.
        Must be > 0.
    settling_radius : float
        When within this distance (radians) of the target AND velocity is
        below the settling entry threshold, switch to exponential decay for
        a smooth, snap-free arrival.  Set to 0.0 to disable.
    settling_stiffness : float
        Exponential decay rate (1/s) inside the settling zone.  Higher =
        faster convergence but sharper stop.  20.0 gives ~0.15 s settling.
    short_move_radius : float
        Moves shorter than this distance (radians) use proportionally
        reduced velocity and acceleration limits.  This prevents the
        planner from ramping to full speed for tiny corrections, making
        small moves inherently smoother.  0.0 disables distance scaling.
    short_move_min_scale : float
        Minimum scaling factor for very short moves (0.0–1.0).  Even the
        tiniest move uses at least ``max_velocity * min_scale``.
    """

    max_velocity: float = 3.14          # ~180 deg/s
    max_acceleration: float = 6.28      # ~360 deg/s²
    max_jerk: float = 20.94             # ~1200 deg/s³
    settling_radius: float = 0.04       # ~2.3° — settling zone entry
    settling_stiffness: float = 25.0    # 1/s — exponential decay rate
    short_move_radius: float = 0.15     # ~8.6° — distance-adaptive scaling
    short_move_min_scale: float = 0.15  # floor for distance scaling

    def __post_init__(self) -> None:
        if self.max_velocity <= 0:
            raise ValueError(f"max_velocity must be > 0, got {self.max_velocity}")
        if self.max_acceleration <= 0:
            raise ValueError(f"max_acceleration must be > 0, got {self.max_acceleration}")
        if self.max_jerk <= 0:
            raise ValueError(f"max_jerk must be > 0, got {self.max_jerk}")
        if self.settling_radius < 0:
            raise ValueError(f"settling_radius must be >= 0, got {self.settling_radius}")
        if self.settling_stiffness < 0:
            raise ValueError(f"settling_stiffness must be >= 0, got {self.settling_stiffness}")
        if self.short_move_radius < 0:
            raise ValueError(f"short_move_radius must be >= 0, got {self.short_move_radius}")
        if not (0.0 <= self.short_move_min_scale <= 1.0):
            raise ValueError(f"short_move_min_scale must be in [0, 1], got {self.short_move_min_scale}")


@dataclass
class SCurvePlanner:
    """Reactive per-tick S-curve planner for one servo axis.

    Usage::

        planner = SCurvePlanner(profile=SCurveProfile(...))
        planner.set_target(1.0)

        # In a 50 Hz loop:
        pos = planner.update(dt=0.02)
        # → write pos to servo
    """

    profile: SCurveProfile = field(default_factory=SCurveProfile)
    state: SCurveState = field(default_factory=SCurveState)
    target: float = 0.0

    # ── External dynamic scaling (set by the node each tick) ──────────
    # These multiply the profile limits before each update.  The motion
    # planner node adjusts them based on servo load, temperature, etc.
    velocity_scale: float = field(default=1.0, repr=False)
    acceleration_scale: float = field(default=1.0, repr=False)

    # ── Internal bookkeeping ──────────────────────────────────────────

    _arrived: bool = field(default=False, repr=False)
    # Effective limits for the current tick (set by update())
    _eff_v_max: float = field(default=3.14, repr=False, init=False)
    _eff_a_max: float = field(default=6.28, repr=False, init=False)
    _eff_j_max: float = field(default=20.94, repr=False, init=False)

    def __post_init__(self) -> None:
        """Sync effective limits with the profile defaults."""
        self._eff_v_max = self.profile.max_velocity
        self._eff_a_max = self.profile.max_acceleration
        self._eff_j_max = self.profile.max_jerk

    # ── Public API ────────────────────────────────────────────────────

    def set_target(self, target: float) -> None:
        """Set (or change) the target position."""
        if target != self.target:
            self.target = target
            self._arrived = False

    def reset(self, position: float, velocity: float = 0.0) -> None:
        """Hard-reset the planner to a known state (e.g. from servo feedback)."""
        self.state = SCurveState(position=position, velocity=velocity, acceleration=0.0)
        self._arrived = False

    @property
    def arrived(self) -> bool:
        return self._arrived

    def update(self, dt: float) -> float:
        """Advance the planner by *dt* seconds and return the new position.

        This is the core per-tick function.  It must be called at a steady
        rate (e.g. 50 Hz / dt=0.02).

        The effective velocity, acceleration, and jerk limits are computed
        dynamically each tick based on:
          - *distance-adaptive scaling*: short moves use proportionally
            reduced limits so the servo never ramps to full speed for tiny
            corrections.
          - *external scaling* (``velocity_scale``, ``acceleration_scale``):
            the motion planner node sets these each tick based on servo
            load, distance, or other context.
        """
        if dt <= 0.0:
            return self.state.position

        if self._arrived:
            return self.state.position

        s = self.state
        p = self.profile

        error = self.target - s.position
        dist = abs(error)
        direction = _sign(error)

        # ── Compute effective limits for this tick ────────────────────
        # 1. Distance-adaptive scaling — proportionally reduce caps for
        #    short moves so the head never ramps to full speed then brakes
        #    immediately.
        dist_scale = 1.0
        if p.short_move_radius > 0.0 and dist < p.short_move_radius:
            raw = dist / p.short_move_radius
            # Blend between min_scale and 1.0 with a sqrt curve so that
            # medium-range moves keep a healthy fraction of full speed.
            dist_scale = p.short_move_min_scale + (1.0 - p.short_move_min_scale) * math.sqrt(raw)

        # 2. External scaling — load dampening, temperature, etc.
        ext_vel_s = max(0.05, min(self.velocity_scale, 1.0))
        ext_acc_s = max(0.05, min(self.acceleration_scale, 1.0))

        # 3. Combined effective limits (never below 5% of profile)
        v_max = p.max_velocity * dist_scale * ext_vel_s
        a_max = p.max_acceleration * dist_scale * ext_acc_s
        j_max = p.max_jerk * dist_scale  # jerk scales with distance only

        # Store for helpers to use this tick
        self._eff_v_max = v_max
        self._eff_a_max = a_max
        self._eff_j_max = j_max

        # ── Tiny-error snap ───────────────────────────────────────────
        # If the remaining distance is negligible and velocity is near-zero,
        # snap to target and stop.
        vel_threshold = j_max * dt * dt  # minimum meaningful velocity
        if dist < 1e-6 and abs(s.velocity) < vel_threshold:
            s.position = self.target
            s.velocity = 0.0
            s.acceleration = 0.0
            self._arrived = True
            return s.position

        # ── Settling zone ─────────────────────────────────────────────
        # Near the target with low velocity, switch to exponential decay
        # for a smooth, snap-free arrival.  This replaces the last fraction
        # of a degree of S-curve deceleration with a critically-damped
        # approach that naturally converges to zero velocity at the target.
        _settling_vel_limit = p.settling_stiffness * p.settling_radius * 2.5
        if (
            p.settling_radius > 0.0
            and dist < p.settling_radius
            and abs(s.velocity) < _settling_vel_limit
        ):
            desired_vel = error * p.settling_stiffness
            # Never accelerate in the settling zone — only decelerate.
            # This prevents a velocity discontinuity at the zone entry
            # when proportional velocity exceeds the S-curve's residual.
            if abs(desired_vel) > abs(s.velocity) and abs(s.velocity) > vel_threshold:
                desired_vel = _sign(error) * abs(s.velocity)
            new_pos = s.position + desired_vel * dt
            s.velocity = desired_vel
            s.acceleration = 0.0
            s.position = new_pos
            # Arrival check — converged close enough
            if abs(self.target - s.position) < 1e-4 and abs(desired_vel) < vel_threshold:
                s.position = self.target
                s.velocity = 0.0
                self._arrived = True
            return s.position

        # ── Direction reversal ────────────────────────────────────────
        # If velocity is in the opposite direction of the error, we must
        # decelerate to zero first.
        moving_wrong_way = (s.velocity != 0.0 and _sign(s.velocity) != direction)

        if moving_wrong_way:
            # Decelerate toward zero velocity using jerk-limited profile.
            new_accel, new_vel, new_pos = self._decelerate_to_zero(dt)
            s.acceleration = new_accel
            s.velocity = new_vel
            s.position = new_pos
            return s.position

        # ── Stopping distance computation ─────────────────────────────
        # Compute the minimum distance required to stop from current velocity
        # with a jerk-limited deceleration.
        stop_dist = self._stopping_distance(abs(s.velocity), abs(s.acceleration))

        if dist <= stop_dist + 1e-9:
            # ── Deceleration phase ────────────────────────────────────
            new_accel, new_vel, new_pos = self._decelerate_toward_target(dt, direction)
        else:
            # ── Acceleration / cruise phase ───────────────────────────
            new_accel, new_vel, new_pos = self._accelerate(dt, direction)

        s.acceleration = new_accel
        s.velocity = new_vel
        s.position = new_pos

        # Post-step arrival check
        new_error = self.target - s.position
        if abs(new_error) < 1e-6 and abs(s.velocity) < vel_threshold:
            s.position = self.target
            s.velocity = 0.0
            s.acceleration = 0.0
            self._arrived = True

        # Overshoot guard: if we crossed the target, clamp position.
        # With settling zone enabled, place the planner at the target with
        # zero velocity — the settling zone will catch it on the next tick
        # if the target moves again.  This avoids the hard "snap" of the
        # old instant-stop behavior.
        elif _sign(new_error) != direction and direction != 0.0:
            s.position = self.target
            s.velocity = 0.0
            s.acceleration = 0.0
            # Only mark arrived if settling is disabled; otherwise let the
            # settling zone's convergence check decide.
            if p.settling_radius <= 0.0:
                self._arrived = True

        return s.position

    # ── Internal helpers ──────────────────────────────────────────────

    def _stopping_distance(self, vel: float, accel: float) -> float:
        """Minimum distance to stop from *vel* with jerk-limited decel.

        Uses the kinematic relationship for a jerk-limited stop:
        - Phase A: ramp accel from current down to 0 (if currently accelerating)
        - Phase B: ramp accel from 0 to -max_accel
        - Phase C: hold -max_accel until velocity is mostly gone
        - Phase D: ramp accel from -max_accel back to 0

        For simplicity we use a conservative upper-bound formula:

            d_stop ≈ v² / (2 * a_max) + v * a_max / (2 * j_max)
                    + a² / (2 * j_max) [residual accel contribution]

        This slightly overestimates, causing the planner to begin decelerating
        a little early — which is safe and produces smoother results.
        """
        a_max = self._eff_a_max
        j_max = self._eff_j_max

        # Distance to dissipate current acceleration
        d_accel_residual = (accel * accel) / (2.0 * j_max) if accel > 0 else 0.0

        if vel < 1e-9:
            return d_accel_residual

        # Main stopping distance (trapezoidal decel + jerk ramp overhead)
        d_main = (vel * vel) / (2.0 * a_max) + (vel * a_max) / (2.0 * j_max)

        return d_main + d_accel_residual

    def _accelerate(self, dt: float, direction: float) -> tuple[float, float, float]:
        """Accelerate toward *direction*, respecting jerk/accel/vel limits."""
        s = self.state
        j_max = self._eff_j_max
        a_max = self._eff_a_max
        v_max = self._eff_v_max

        vel = s.velocity
        accel = s.acceleration

        target_accel = direction * a_max  # desired peak acceleration

        # Jerk-limited ramp toward target acceleration
        accel_error = target_accel - accel
        max_accel_change = j_max * dt
        if abs(accel_error) <= max_accel_change:
            new_accel = target_accel
        else:
            new_accel = accel + _sign(accel_error) * max_accel_change

        # Velocity limit: if near v_max, start reducing acceleration
        new_vel = vel + new_accel * dt
        if abs(new_vel) > v_max:
            new_vel = direction * v_max
            # Back-compute to reduce acceleration so we don't exceed v_max
            new_accel = (new_vel - vel) / dt if dt > 0 else 0.0

        new_pos = s.position + vel * dt + 0.5 * new_accel * dt * dt

        return new_accel, new_vel, new_pos

    def _decelerate_toward_target(
        self, dt: float, direction: float
    ) -> tuple[float, float, float]:
        """Jerk-limited deceleration toward the target position."""
        s = self.state
        j_max = self._eff_j_max
        a_max = self._eff_a_max

        vel = s.velocity
        accel = s.acceleration

        # We want to reach velocity=0 at the target. Compute desired
        # deceleration = braking acceleration opposing current velocity.
        target_accel = -direction * a_max  # brake in opposite direction of motion

        # But if velocity is already very low, reduce target accel proportionally
        # to avoid overshoot near the end.
        remaining_speed = abs(vel)
        if remaining_speed < a_max * (a_max / j_max):
            # In the low-speed regime, use a proportional decel
            scale = remaining_speed / max(a_max * (a_max / j_max), 1e-9)
            target_accel = target_accel * max(scale, 0.05)

        # Jerk-limited ramp toward target deceleration
        accel_error = target_accel - accel
        max_accel_change = j_max * dt
        if abs(accel_error) <= max_accel_change:
            new_accel = target_accel
        else:
            new_accel = accel + _sign(accel_error) * max_accel_change

        new_vel = vel + new_accel * dt

        # If velocity crosses zero during decel, clamp to zero
        if _sign(new_vel) != _sign(vel) and vel != 0.0:
            new_vel = 0.0
            new_accel = 0.0

        new_pos = s.position + vel * dt + 0.5 * new_accel * dt * dt

        return new_accel, new_vel, new_pos

    def _decelerate_to_zero(self, dt: float) -> tuple[float, float, float]:
        """Decelerate to zero velocity (direction reversal)."""
        s = self.state
        j_max = self._eff_j_max
        a_max = self._eff_a_max

        vel = s.velocity
        accel = s.acceleration
        vel_dir = _sign(vel)

        # Target accel opposes current velocity
        target_accel = -vel_dir * a_max

        # Scale down for low speeds to finish cleanly
        remaining_speed = abs(vel)
        if remaining_speed < a_max * (a_max / j_max):
            scale = remaining_speed / max(a_max * (a_max / j_max), 1e-9)
            target_accel = target_accel * max(scale, 0.05)

        accel_error = target_accel - accel
        max_accel_change = j_max * dt
        if abs(accel_error) <= max_accel_change:
            new_accel = target_accel
        else:
            new_accel = accel + _sign(accel_error) * max_accel_change

        new_vel = vel + new_accel * dt

        # Clamp at zero — don't start moving the other way
        if _sign(new_vel) != vel_dir and vel_dir != 0.0:
            new_vel = 0.0
            new_accel = 0.0

        new_pos = s.position + vel * dt + 0.5 * new_accel * dt * dt

        return new_accel, new_vel, new_pos


# ── Convenience helpers ───────────────────────────────────────────────

def dps_to_rads(dps: float) -> float:
    """Degrees/s to radians/s."""
    return math.radians(dps)


def dps2_to_rads2(dps2: float) -> float:
    """Degrees/s² to radians/s²."""
    return math.radians(dps2)


def dps3_to_rads3(dps3: float) -> float:
    """Degrees/s³ to radians/s³."""
    return math.radians(dps3)


def profile_from_dps(
    max_velocity_dps: float,
    max_acceleration_dps2: float,
    max_jerk_dps3: float,
    settling_radius: float = 0.04,
    settling_stiffness: float = 25.0,
    short_move_radius: float = 0.15,
    short_move_min_scale: float = 0.15,
) -> SCurveProfile:
    """Create an SCurveProfile from degree-per-second units."""
    return SCurveProfile(
        max_velocity=dps_to_rads(max_velocity_dps),
        max_acceleration=dps2_to_rads2(max_acceleration_dps2),
        max_jerk=dps3_to_rads3(max_jerk_dps3),
        settling_radius=settling_radius,
        settling_stiffness=settling_stiffness,
        short_move_radius=short_move_radius,
        short_move_min_scale=short_move_min_scale,
    )
