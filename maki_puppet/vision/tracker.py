"""FaceTracker — the face-following control law, ported from face_tracker_node.py.

Pure: no camera, no servos, no clock of its own beyond an injectable ``now``.
:meth:`FaceTracker.update` takes one face detection plus the current servo
pose and returns command radians for head_pan/head_tilt/eyes_pan/eyes_tilt.

The control structure (all guards preserved from the ROS node):

  * **Anchor-point head control.** The face's *body-centric* bearing is
    computed from head pose + pixel offset.  The head only re-anchors once
    that bearing has drifted past ``head_reanchor_threshold_rad`` for
    ``anchor_hold_frames`` consecutive frames, then drives to the full face
    bearing — not an EMA-smoothed partial anchor, which would stall the head
    with the residual error parked in the eyes.
  * **Eyes track pixel error directly**, with engage/release hysteresis per
    axis so they don't jitter around the deadband.
  * **VOR (eye-lead).** While the head is still travelling to the anchor, the
    eyes counter-rotate by the *remaining* head displacement so gaze stays
    fixed on the face. Remaining displacement is measured against actual
    servo feedback, never against the commanded value: MotionService's S-curve
    means the command jumps to the anchor instantly while the servo lags, and
    using the command here would both under-compensate the VOR and create a
    positive-feedback loop that drives the head past the face into its limit.
  * **Micro-saccades.** Small triangle-wave eye offsets, gated to only appear
    when the face is stable in both position and velocity.

Sign conventions match :mod:`maki_puppet.motion.joints`: positive head_tilt
and eyes_tilt are DOWN, positive head_pan/eyes_pan follow the body-centric
convention already used by the servo tables.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Callable, Dict, Mapping, Optional

from ..motion.joints import JOINTS, JointInfo
from .detector import FaceDetection

TRACKING_JOINTS = ("head_pan", "head_tilt", "eyes_pan", "eyes_tilt")


def _symmetric_limit(info: JointInfo) -> float:
    """Largest magnitude the joint can reach in BOTH directions from neutral."""
    return min(abs(info.rad_min - info.neutral_rad), abs(info.rad_max - info.neutral_rad))


@dataclass(frozen=True)
class TrackerConfig:
    """Tuning for :class:`FaceTracker` (``puppet.yaml`` → ``vision.tracking``).

    Defaults are the ROS stack's DEPLOYED values (``maki_face_tracker.yaml``),
    not the node's declared parameter defaults — those differ in a dozen places
    and were never what actually ran on the robot. ``puppet.yaml`` restates
    them explicitly for tuning; the two are kept in agreement.
    """

    # Camera geometry — needed to turn pixels into body-centric angles.
    camera_hfov_deg: float = 62.0
    camera_vfov_deg: float = 49.0
    image_center_offset_x: float = 0.0
    image_center_offset_y: float = 0.0

    # Head: anchor commit
    head_reanchor_threshold_rad: float = 0.08
    anchor_hold_frames: int = 2
    head_pan_hold_threshold_rad: float = 0.008
    head_hold_frames: int = 1
    head_settle_time_s: float = 0.10
    head_settle_min_delta_rad: float = 0.008

    # Eyes: gains and hysteresis
    kp_eye_pan: float = 0.65
    kp_eye_tilt: float = 0.5
    deadband: float = 0.09
    eye_only_error_threshold: float = 0.03
    eye_release_error_threshold: float = 0.015
    tracking_mode_eye_clamp_rad: float = 0.20
    center_mode_eye_clamp_rad: float = 0.14
    eye_lead_strength: float = 0.35
    eye_recenter_feedforward: float = 0.85

    # Step limits (only used when use_planner_smoothing is False)
    use_planner_smoothing: bool = True
    max_step_rad: float = 0.025
    max_step_head_fast_rad: float = 0.04
    max_step_head_large_error_rad: float = 0.07
    max_step_eye_rad: float = 0.06
    max_step_eye_fast_rad: float = 0.14

    # Micro-saccades
    enable_eye_saccades: bool = True
    saccade_error_threshold: float = 0.06
    saccade_speed_threshold: float = 0.7
    saccade_amplitude_rad: float = 0.04
    saccade_period_s: float = 0.35
    saccade_tilt_fraction: float = 0.6

    # Detection conditioning
    face_smoothing_alpha: float = 0.24
    control_min_dt_s: float = 0.05

    @classmethod
    def from_mapping(cls, cfg: Optional[Mapping]) -> "TrackerConfig":
        """Build from a config mapping, ignoring keys we don't know."""
        cfg = dict(cfg or {})
        fields = {f for f in cls.__dataclass_fields__}
        known = {k: v for k, v in cfg.items() if k in fields}
        typed = {
            k: (bool(v) if isinstance(cls.__dataclass_fields__[k].default, bool)
                else int(v) if isinstance(cls.__dataclass_fields__[k].default, int)
                else float(v))
            for k, v in known.items()
        }
        return cls(**typed)


@dataclass(frozen=True)
class TrackerOutput:
    """One control tick's command radians."""

    head_pan: float
    head_tilt: float
    eyes_pan: float
    eyes_tilt: float

    def as_targets(self, *, eyes_only: bool = False) -> Dict[str, float]:
        """Motion-layer targets. ``eyes_only`` omits the head joints entirely,
        leaving them to whatever lower-priority layer holds them."""
        targets = {"eyes_pan": self.eyes_pan, "eyes_tilt": self.eyes_tilt}
        if not eyes_only:
            targets["head_pan"] = self.head_pan
            targets["head_tilt"] = self.head_tilt
        return targets


class FaceTracker:
    """Stateful face-following controller. One instance per tracking session."""

    def __init__(
        self,
        config: Optional[TrackerConfig] = None,
        *,
        joints: Optional[Mapping[str, JointInfo]] = None,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self.cfg = config or TrackerConfig()
        self._clock = clock
        table = joints or JOINTS

        self._hfov = math.radians(self.cfg.camera_hfov_deg)
        self._vfov = math.radians(self.cfg.camera_vfov_deg)

        # Mechanical limits, read from the canonical joint table rather than
        # duplicating the tick constants the ROS node carried.
        pan, tilt = table["head_pan"], table["head_tilt"]
        self._max_pan_rad = _symmetric_limit(pan)
        self._max_tilt_up_rad = abs(tilt.rad_min - tilt.neutral_rad)
        self._max_tilt_down_rad = abs(tilt.rad_max - tilt.neutral_rad)
        self._eyes_max_rad = min(
            _symmetric_limit(table["eyes_pan"]), _symmetric_limit(table["eyes_tilt"])
        )

        self.reset()

    # ── Lifecycle ──────────────────────────────────────────────────────

    def reset(self) -> None:
        """Clear all control state. Call whenever tracking (re)starts, so a
        new session never inherits a stale anchor from the last one."""
        self.current_pan = 0.0
        self.current_tilt = 0.0
        self.current_eye_pan = 0.0
        self.current_eye_tilt = 0.0

        self._anchor_bearing = 0.0
        self._anchor_elevation = 0.0
        self._anchor_above_thresh_count = 0
        self._head_pan_above_thresh_count = 0
        self._head_settle_until = 0.0

        self._eye_pan_active = False
        self._eye_tilt_active = False

        self._smooth_cx: Optional[float] = None
        self._smooth_cy: Optional[float] = None

        self._last_err_pan = 0.0
        self._last_err_tilt = 0.0
        self._last_err_ts = 0.0
        self._last_control_time = 0.0

        self._prev_feedback_pan = 0.0
        self._prev_feedback_tilt = 0.0
        self._seeded = False

    # ── Control ────────────────────────────────────────────────────────

    def update(
        self,
        face: Optional[FaceDetection],
        pose_rad: Mapping[str, float],
        now: Optional[float] = None,
    ) -> Optional[TrackerOutput]:
        """One control tick.

        Returns ``None`` when there is nothing to command — no face, or the
        tick arrived sooner than ``control_min_dt_s`` after the last one.
        The caller keeps holding its previous targets in that case.
        """
        now = self._clock() if now is None else now
        if face is None:
            return None
        if now - self._last_control_time < self.cfg.control_min_dt_s:
            return None

        feedback_pan = pose_rad.get("head_pan")
        feedback_tilt = pose_rad.get("head_tilt")
        if not self._seeded:
            # Start from where the head actually is, not from 0.0 — otherwise
            # the very first bearing is computed against a fictional pose.
            self.current_pan = float(feedback_pan or 0.0)
            self.current_tilt = float(feedback_tilt or 0.0)
            self.current_eye_pan = float(pose_rad.get("eyes_pan") or 0.0)
            self.current_eye_tilt = float(pose_rad.get("eyes_tilt") or 0.0)
            self._anchor_bearing = self.current_pan
            self._anchor_elevation = self.current_tilt
            self._prev_feedback_pan = self.current_pan
            self._prev_feedback_tilt = self.current_tilt
            self._seeded = True

        cx, cy = self._smooth_center(face)
        w, h = float(face.frame_w), float(face.frame_h)

        # ── Anchor-point head control ──────────────────────────────────
        bearing_pan, bearing_tilt = self._motion_reference(feedback_pan, feedback_tilt)
        face_bearing, face_elevation = self.pixel_to_body_angles(
            cx, cy, w, h, bearing_pan, bearing_tilt
        )

        exceeds = (
            abs(face_bearing - self._anchor_bearing) > self.cfg.head_reanchor_threshold_rad
            or abs(face_elevation - self._anchor_elevation) > self.cfg.head_reanchor_threshold_rad
        )
        head_wants_move = False
        if exceeds:
            self._anchor_above_thresh_count += 1
            if self._anchor_above_thresh_count >= self.cfg.anchor_hold_frames:
                # Commit the re-anchor to the FULL face bearing (see module doc).
                self._anchor_bearing = face_bearing
                self._anchor_elevation = face_elevation
                head_wants_move = True
                self._anchor_above_thresh_count = self.cfg.anchor_hold_frames
        else:
            self._anchor_above_thresh_count = 0

        # ── Pixel-space error (drives the eyes) ────────────────────────
        nx = (cx - w / 2.0) / (w / 2.0) - self.cfg.image_center_offset_x
        ny = (cy - h / 2.0) / (h / 2.0) - self.cfg.image_center_offset_y
        err_pan, err_tilt = -nx, ny
        dt_err = max(now - self._last_err_ts, 1e-3)
        err_pan_rate = (err_pan - self._last_err_pan) / dt_err
        err_tilt_rate = (err_tilt - self._last_err_tilt) / dt_err
        within_deadband = abs(nx) < self.cfg.deadband and abs(ny) < self.cfg.deadband

        self._eye_pan_active = self._update_axis_activity(
            abs(err_pan), self._eye_pan_active,
            self.cfg.eye_only_error_threshold, self.cfg.eye_release_error_threshold,
        )
        self._eye_tilt_active = self._update_axis_activity(
            abs(err_tilt), self._eye_tilt_active,
            self.cfg.eye_only_error_threshold, self.cfg.eye_release_error_threshold,
        )

        # ── Head target ────────────────────────────────────────────────
        prev_pan, prev_tilt = self.current_pan, self.current_tilt
        arrival = min(0.02, max(0.01, self.cfg.head_reanchor_threshold_rad * 0.25))
        ref_pan, ref_tilt = self._motion_reference(feedback_pan, feedback_tilt)
        anchor_pan_error = abs(self._anchor_bearing - ref_pan)
        anchor_tilt_error = abs(self._anchor_elevation - ref_tilt)
        head_target_active = anchor_pan_error > arrival or anchor_tilt_error > arrival

        if (head_wants_move or head_target_active) and now >= self._head_settle_until:
            raw_head_pan, raw_head_tilt = self._anchor_bearing, self._anchor_elevation
        else:
            # Head stays put — the eyes carry the residual.
            raw_head_pan, raw_head_tilt = self.current_pan, self.current_tilt

        pan_scale = self._error_speed_scale(
            anchor_pan_error, self.cfg.head_reanchor_threshold_rad
        )
        tilt_scale = self._error_speed_scale(
            anchor_tilt_error, self.cfg.head_reanchor_threshold_rad
        )
        max_scale = max(pan_scale, tilt_scale)

        if self.cfg.use_planner_smoothing:
            target_head_pan, target_head_tilt = raw_head_pan, raw_head_tilt
        else:
            target_head_pan = self._limit_step(
                self.current_pan, raw_head_pan,
                self._three_tier_step(
                    self.cfg.max_step_rad, self.cfg.max_step_head_fast_rad,
                    self.cfg.max_step_head_large_error_rad, pan_scale,
                ),
            )
            target_head_tilt = self._limit_step(
                self.current_tilt, raw_head_tilt,
                self._three_tier_step(
                    self.cfg.max_step_rad, self.cfg.max_step_head_fast_rad,
                    self.cfg.max_step_head_large_error_rad, tilt_scale,
                ),
            )

        target_head_pan = max(-self._max_pan_rad, min(self._max_pan_rad, target_head_pan))
        target_head_tilt = max(
            -self._max_tilt_up_rad, min(self._max_tilt_down_rad, target_head_tilt)
        )

        # Head-pan hold threshold: a secondary net against micro-jitter, on top
        # of the anchor commit. Requires the move to persist head_hold_frames.
        if abs(target_head_pan - ref_pan) < self.cfg.head_pan_hold_threshold_rad:
            target_head_pan = prev_pan
            self._head_pan_above_thresh_count = 0
        else:
            self._head_pan_above_thresh_count += 1
            if self._head_pan_above_thresh_count < self.cfg.head_hold_frames:
                target_head_pan = prev_pan
            else:
                self._head_pan_above_thresh_count = self.cfg.head_hold_frames

        # ── Eye target ─────────────────────────────────────────────────
        pan_eye_err = err_pan if self._eye_pan_active and not within_deadband else 0.0
        tilt_eye_err = err_tilt if self._eye_tilt_active and not within_deadband else 0.0
        raw_eye_pan = self.cfg.kp_eye_pan * pan_eye_err
        raw_eye_tilt = self.cfg.kp_eye_tilt * tilt_eye_err

        # VOR: counter-rotate by the head's REMAINING displacement, measured
        # against feedback (see module docstring on why not the command).
        if self.cfg.eye_lead_strength > 0.0:
            head_err_pan = raw_head_pan - self._reference_or(feedback_pan, self.current_pan)
            head_err_tilt = raw_head_tilt - self._reference_or(feedback_tilt, self.current_tilt)
            if abs(head_err_pan) > 0.03:
                raw_eye_pan += head_err_pan * self.cfg.eye_lead_strength
            if abs(head_err_tilt) > 0.03:
                raw_eye_tilt += head_err_tilt * self.cfg.eye_lead_strength

        if self.cfg.enable_eye_saccades:
            gain = self._saccade_activation_gain(
                err_pan, err_tilt, err_pan_rate, err_tilt_rate
            )
            if gain > 0.0:
                s_pan, s_tilt = self._saccade_offsets(now)
                raw_eye_pan += gain * s_pan
                raw_eye_tilt += gain * s_tilt

        # Feed-forward recentering, per tick. Skipped under planner smoothing:
        # eye-lead above already supplies feedback-based VOR from the total
        # remaining displacement, so adding feedforward double-compensates.
        if (
            self.cfg.eye_recenter_feedforward > 0.0
            and not self.cfg.use_planner_smoothing
            and (head_wants_move or head_target_active)
        ):
            raw_eye_pan -= (target_head_pan - prev_pan) * self.cfg.eye_recenter_feedforward
            raw_eye_tilt -= (target_head_tilt - prev_tilt) * self.cfg.eye_recenter_feedforward
        self._prev_feedback_pan = self._reference_or(feedback_pan, self.current_pan)
        self._prev_feedback_tilt = self._reference_or(feedback_tilt, self.current_tilt)

        clamp = min(
            self.cfg.center_mode_eye_clamp_rad if within_deadband
            else self.cfg.tracking_mode_eye_clamp_rad,
            self._eyes_max_rad,
        )
        raw_eye_pan = max(-clamp, min(clamp, raw_eye_pan))
        raw_eye_tilt = max(-clamp, min(clamp, raw_eye_tilt))

        if self.cfg.use_planner_smoothing:
            target_eye_pan, target_eye_tilt = raw_eye_pan, raw_eye_tilt
        else:
            target_eye_pan = self._limit_step(
                self.current_eye_pan, raw_eye_pan,
                self._scaled_step(
                    self.cfg.max_step_eye_rad, self.cfg.max_step_eye_fast_rad, pan_scale
                ),
            )
            target_eye_tilt = self._limit_step(
                self.current_eye_tilt, raw_eye_tilt,
                self._scaled_step(
                    self.cfg.max_step_eye_rad, self.cfg.max_step_eye_fast_rad, tilt_scale
                ),
            )

        self.current_pan = target_head_pan
        self.current_tilt = target_head_tilt
        self.current_eye_pan = target_eye_pan
        self.current_eye_tilt = target_eye_tilt

        # Settle window after a meaningful head move, so the head isn't
        # re-commanded while it is still swinging.
        head_step = max(abs(self.current_pan - prev_pan), abs(self.current_tilt - prev_tilt))
        if head_step >= self.cfg.head_settle_min_delta_rad:
            self._head_settle_until = now + max(
                0.01, self.cfg.head_settle_time_s * (1.0 - 0.7 * max_scale)
            )

        self._last_control_time = now
        self._last_err_pan = err_pan
        self._last_err_tilt = err_tilt
        self._last_err_ts = now

        return TrackerOutput(
            head_pan=self.current_pan,
            head_tilt=self.current_tilt,
            eyes_pan=self.current_eye_pan,
            eyes_tilt=self.current_eye_tilt,
        )

    # ── Geometry ───────────────────────────────────────────────────────

    def pixel_to_body_angles(
        self, px: float, py: float, img_w: float, img_h: float,
        head_pan: float, head_tilt: float,
    ) -> tuple:
        """Pixel position → body-centric (bearing, elevation) in radians."""
        nx = (px / img_w) - 0.5
        ny = (py / img_h) - 0.5
        return head_pan + (-nx * self._hfov), head_tilt + (ny * self._vfov)

    def _smooth_center(self, face: FaceDetection) -> tuple:
        a = self.cfg.face_smoothing_alpha
        if self._smooth_cx is None or a <= 0.0:
            self._smooth_cx, self._smooth_cy = face.cx, face.cy
        else:
            self._smooth_cx += a * (face.cx - self._smooth_cx)
            self._smooth_cy += a * (face.cy - self._smooth_cy)
        return self._smooth_cx, self._smooth_cy

    def _motion_reference(
        self, feedback_pan: Optional[float], feedback_tilt: Optional[float]
    ) -> tuple:
        """Head pose to reason against: actual feedback when available."""
        return (
            self._reference_or(feedback_pan, self.current_pan),
            self._reference_or(feedback_tilt, self.current_tilt),
        )

    @staticmethod
    def _reference_or(feedback: Optional[float], fallback: float) -> float:
        return float(feedback) if feedback is not None else fallback

    # ── Pure control helpers (ported verbatim) ─────────────────────────

    @staticmethod
    def _limit_step(current: float, target: float, max_step: float) -> float:
        delta = target - current
        if abs(delta) > max_step:
            delta = math.copysign(max_step, delta)
        return current + delta

    @staticmethod
    def _update_axis_activity(
        abs_err: float, active: bool, engage_threshold: float, release_threshold: float
    ) -> bool:
        return abs_err >= (release_threshold if active else engage_threshold)

    @staticmethod
    def _scaled_step(base_step: float, fast_step: float, scale: float) -> float:
        s = min(max(scale, 0.0), 1.0)
        return base_step + (fast_step - base_step) * s

    @staticmethod
    def _three_tier_step(
        base_step: float, fast_step: float, large_step: float, scale: float
    ) -> float:
        """Piecewise-linear blend across three speed tiers:
        scale 0.0–0.5 blends base→fast, 0.5–1.0 blends fast→large."""
        s = min(max(scale, 0.0), 1.0)
        if s <= 0.5:
            return base_step + (fast_step - base_step) * (s * 2.0)
        return fast_step + (large_step - fast_step) * ((s - 0.5) * 2.0)

    @staticmethod
    def _error_speed_scale(abs_err: float, engage_threshold: float) -> float:
        """0 near the engage threshold, 1 near full-frame error."""
        denom = max(1e-6, 1.0 - engage_threshold)
        return min(max((abs_err - engage_threshold) / denom, 0.0), 1.0)

    @staticmethod
    def _falloff_gain(value: float, full_threshold: float, near_threshold: float) -> float:
        if value <= full_threshold:
            return 1.0
        if value >= near_threshold:
            return 0.0
        return 1.0 - ((value - full_threshold) / max(near_threshold - full_threshold, 1e-6))

    def _saccade_activation_gain(
        self, err_pan: float, err_tilt: float, err_pan_rate: float, err_tilt_rate: float
    ) -> float:
        """Gain in [0, 1]: 1.0 when the face is stable, tapering to 0.0 once it
        is clearly moving — micro-saccades must never fight a real pursuit."""
        err_mag = max(abs(err_pan), abs(err_tilt))
        rate_mag = max(abs(err_pan_rate), abs(err_tilt_rate))
        e_full = self.cfg.saccade_error_threshold
        r_full = self.cfg.saccade_speed_threshold
        return min(
            self._falloff_gain(err_mag, e_full, 1.8 * e_full),
            self._falloff_gain(rate_mag, r_full, 1.8 * r_full),
        )

    def _saccade_offsets(self, now: float) -> tuple:
        period = max(self.cfg.saccade_period_s, 0.2)
        pan = self.cfg.saccade_amplitude_rad * self._triangle_wave(now, period)
        tilt = (
            self.cfg.saccade_amplitude_rad
            * self.cfg.saccade_tilt_fraction
            * self._triangle_wave(now + 0.37 * period, period)
        )
        return pan, tilt

    @staticmethod
    def _triangle_wave(t: float, period: float) -> float:
        """Triangular wave in [-1, 1]."""
        phase = (t / max(period, 1e-6)) % 1.0
        return 1.0 - 4.0 * abs(phase - 0.5)
