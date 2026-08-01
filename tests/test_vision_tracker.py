"""FaceTracker tests: the ported control law.

Covers the behaviours the ROS node's guard comments call out as the ones that
actually break in the field — anchor hysteresis, VOR against feedback rather
than command, saccade gating, and clamping to mechanical limits.
"""

from __future__ import annotations

import math

import pytest

from maki_puppet.motion.joints import JOINTS
from maki_puppet.vision.detector import FaceDetection
from maki_puppet.vision.tracker import FaceTracker, TrackerConfig

FRAME_W, FRAME_H = 640, 480


def face_at(cx: float, cy: float, size: float = 100.0) -> FaceDetection:
    return FaceDetection(
        bbox_x=cx - size / 2, bbox_y=cy - size / 2,
        bbox_w=size, bbox_h=size, confidence=1.0,
        frame_w=FRAME_W, frame_h=FRAME_H,
    )


def centred() -> FaceDetection:
    return face_at(FRAME_W / 2, FRAME_H / 2)


def make_tracker(**overrides) -> FaceTracker:
    # No face-centre EMA and no saccades by default: both add lag/noise that
    # would obscure what each test is actually asserting.
    base = {"face_smoothing_alpha": 0.0, "enable_eye_saccades": False}
    base.update(overrides)
    return FaceTracker(TrackerConfig(**base))


LEVEL = {"head_pan": 0.0, "head_tilt": 0.0, "eyes_pan": 0.0, "eyes_tilt": 0.0}


def run(tracker, face, pose=None, *, ticks=1, t0=100.0, dt=0.1):
    """Drive N control ticks, returning the last non-None output."""
    out = None
    for i in range(ticks):
        result = tracker.update(face, pose or LEVEL, t0 + i * dt)
        if result is not None:
            out = result
    return out


# ── Rate limiting ────────────────────────────────────────────────────────


def test_update_rejects_ticks_faster_than_control_min_dt():
    tracker = make_tracker(control_min_dt_s=0.05)
    assert tracker.update(centred(), LEVEL, 100.0) is not None
    assert tracker.update(centred(), LEVEL, 100.02) is None
    assert tracker.update(centred(), LEVEL, 100.06) is not None


def test_no_face_yields_no_command():
    tracker = make_tracker()
    assert tracker.update(None, LEVEL, 100.0) is None


# ── Seeding ──────────────────────────────────────────────────────────────


def test_first_tick_seeds_from_actual_pose_not_zero():
    """A tracker started with the head already turned must anchor relative to
    where the head IS, or its very first bearing is computed against a
    fictional pose."""
    tracker = make_tracker()
    pose = {"head_pan": 0.4, "head_tilt": 0.1}
    out = run(tracker, centred(), pose)
    # Face dead centre → bearing equals the current head pose, so no move.
    assert out.head_pan == pytest.approx(0.4, abs=1e-6)
    assert out.head_tilt == pytest.approx(0.1, abs=1e-6)


# ── Anchor hysteresis ────────────────────────────────────────────────────


def test_head_holds_until_drift_persists_for_anchor_hold_frames():
    # head_hold_frames=1 isolates the anchor gate from the head-pan hold gate,
    # which is a second, independent hysteresis stage on top of it.
    tracker = make_tracker(
        anchor_hold_frames=3, head_reanchor_threshold_rad=0.08, head_hold_frames=1
    )
    run(tracker, centred())                       # seed + anchor at 0
    far = face_at(FRAME_W * 0.9, FRAME_H / 2)     # well off to one side

    first = tracker.update(far, LEVEL, 100.1)
    assert first.head_pan == pytest.approx(0.0, abs=1e-9), "moved on frame 1 of 3"
    second = tracker.update(far, LEVEL, 100.2)
    assert second.head_pan == pytest.approx(0.0, abs=1e-9), "moved on frame 2 of 3"
    third = tracker.update(far, LEVEL, 100.3)
    assert abs(third.head_pan) > 0.1, "should re-anchor once drift has persisted"


def test_small_drift_never_moves_the_head():
    """Sub-threshold jitter must leave the head completely still — this is the
    difference between a calm robot and one that fidgets constantly."""
    tracker = make_tracker(head_reanchor_threshold_rad=0.08)
    run(tracker, centred())
    nudged = face_at(FRAME_W / 2 + 6, FRAME_H / 2)   # ~0.01 rad of bearing
    for i in range(20):
        out = tracker.update(nudged, LEVEL, 100.1 + i * 0.1)
        assert out.head_pan == pytest.approx(0.0, abs=1e-9)


def test_head_turns_toward_the_face_side():
    """Sign check: a face to the left of frame must not send the head right."""
    tracker = make_tracker(anchor_hold_frames=1)
    run(tracker, centred())
    left = face_at(FRAME_W * 0.1, FRAME_H / 2)
    left_out = run(tracker, left, ticks=3, t0=100.1)

    tracker2 = make_tracker(anchor_hold_frames=1)
    run(tracker2, centred())
    right = face_at(FRAME_W * 0.9, FRAME_H / 2)
    right_out = run(tracker2, right, ticks=3, t0=100.1)

    assert left_out.head_pan * right_out.head_pan < 0, "opposite sides, opposite directions"


# ── Eyes ─────────────────────────────────────────────────────────────────


def test_eyes_stay_still_inside_the_deadband():
    tracker = make_tracker(deadband=0.05, eye_lead_strength=0.0)
    out = run(tracker, face_at(FRAME_W / 2 + 4, FRAME_H / 2))
    assert out.eyes_pan == pytest.approx(0.0, abs=1e-9)
    assert out.eyes_tilt == pytest.approx(0.0, abs=1e-9)


def test_eye_activity_hysteresis_engages_high_releases_low():
    """Once engaged, the eyes keep tracking below the engage threshold — the
    band between engage and release is what stops them buzzing."""
    tracker = make_tracker(
        eye_only_error_threshold=0.10,
        eye_release_error_threshold=0.02,
        deadband=0.005,
        eye_lead_strength=0.0,
    )
    run(tracker, centred())
    assert tracker._eye_pan_active is False

    # Well past engage.
    run(tracker, face_at(FRAME_W / 2 + 0.30 * FRAME_W / 2, FRAME_H / 2), t0=100.1)
    assert tracker._eye_pan_active is True

    # Between release and engage: stays active.
    run(tracker, face_at(FRAME_W / 2 + 0.05 * FRAME_W / 2, FRAME_H / 2), t0=100.2)
    assert tracker._eye_pan_active is True

    # Below release: disengages.
    run(tracker, face_at(FRAME_W / 2 + 0.01 * FRAME_W / 2, FRAME_H / 2), t0=100.3)
    assert tracker._eye_pan_active is False


def test_eyes_are_clamped_to_the_mechanical_limit():
    tracker = make_tracker(
        kp_eye_pan=50.0,                    # absurd gain, to force the clamp
        tracking_mode_eye_clamp_rad=10.0,   # config clamp out of the way
        deadband=0.0,
        eye_lead_strength=0.0,
    )
    out = run(tracker, face_at(FRAME_W - 1, FRAME_H / 2), ticks=3)
    eyes_pan = JOINTS["eyes_pan"]
    limit = min(abs(eyes_pan.rad_min), abs(eyes_pan.rad_max))
    assert abs(out.eyes_pan) <= limit + 1e-9


# ── VOR / eye-lead ───────────────────────────────────────────────────────


def test_motion_reference_prefers_feedback_over_command():
    """Everything downstream — bearings, arrival, VOR — reasons against actual
    servo feedback. Using the command instead over-estimates how far the head
    has already turned and drives it past the face into its limit."""
    tracker = make_tracker()
    tracker.current_pan, tracker.current_tilt = 0.5, 0.2

    assert tracker._motion_reference(0.1, -0.05) == (0.1, -0.05)
    # No feedback available yet → fall back to the last command.
    assert tracker._motion_reference(None, None) == (0.5, 0.2)


def test_eye_lead_counter_rotates_by_the_remaining_head_travel():
    """VOR: while the head is still swinging, the eyes take up the slack so
    gaze stays on the face."""
    with_vor = make_tracker(
        eye_lead_strength=1.0, kp_eye_pan=0.0, deadband=0.0,
        anchor_hold_frames=1, head_hold_frames=1,
    )
    without = make_tracker(
        eye_lead_strength=0.0, kp_eye_pan=0.0, deadband=0.0,
        anchor_hold_frames=1, head_hold_frames=1,
    )
    off_centre = face_at(FRAME_W * 0.58, FRAME_H / 2)
    a = run(with_vor, off_centre, ticks=3, t0=100.0)
    b = run(without, off_centre, ticks=3, t0=100.0)

    assert abs(a.eyes_pan) > 0.05, "eyes should absorb the head's remaining travel"
    assert b.eyes_pan == pytest.approx(0.0), "with kp=0 and no VOR, eyes stay put"
    # The eyes lead in the direction the head is going.
    assert a.eyes_pan * a.head_pan > 0


def test_eye_lead_disabled_leaves_eyes_on_pixel_error_only():
    tracker = make_tracker(eye_lead_strength=0.0, kp_eye_pan=0.5, deadband=0.0)
    out = run(tracker, face_at(FRAME_W / 2 + 64, FRAME_H / 2))
    nx = 64 / (FRAME_W / 2)
    assert out.eyes_pan == pytest.approx(0.5 * -nx, abs=1e-9)


# ── Saccades ─────────────────────────────────────────────────────────────


def test_saccades_are_gated_off_while_the_face_is_moving():
    """Micro-saccades must never fight a real pursuit."""
    stable = make_tracker(enable_eye_saccades=True, saccade_amplitude_rad=0.05)
    run(stable, centred(), ticks=4)
    assert stable._saccade_activation_gain(0.0, 0.0, 0.0, 0.0) == pytest.approx(1.0)
    # Error and rate both well past the "clearly moving" thresholds.
    assert stable._saccade_activation_gain(0.5, 0.5, 5.0, 5.0) == pytest.approx(0.0)


def test_saccades_move_the_eyes_when_the_face_is_perfectly_still():
    with_sac = make_tracker(enable_eye_saccades=True, saccade_amplitude_rad=0.02)
    positions = {
        run(with_sac, centred(), t0=100.0 + i * 0.3).eyes_pan for i in range(6)
    }
    assert len(positions) > 1, "a locked gaze should still drift a little"


def test_triangle_wave_stays_in_range_and_is_periodic():
    tri = FaceTracker._triangle_wave
    for t in [0.0, 0.1, 0.37, 0.5, 0.9, 1.3, 7.7]:
        assert -1.0 <= tri(t, 1.2) <= 1.0
    assert tri(0.4, 1.2) == pytest.approx(tri(0.4 + 1.2, 1.2), abs=1e-9)


# ── Geometry ─────────────────────────────────────────────────────────────


def test_pixel_to_body_angles_is_inverted_by_the_sim_projection():
    tracker = make_tracker(camera_hfov_deg=62.0, camera_vfov_deg=49.0)
    hfov, vfov = math.radians(62.0), math.radians(49.0)
    head_pan, head_tilt = 0.2, -0.05
    bearing, elevation = 0.31, 0.04

    # Project world → pixels the way SimFaceDetector does...
    cx = ((head_pan - bearing) / hfov + 0.5) * FRAME_W
    cy = ((elevation - head_tilt) / vfov + 0.5) * FRAME_H
    # ...and back again.
    got_b, got_e = tracker.pixel_to_body_angles(
        cx, cy, FRAME_W, FRAME_H, head_pan, head_tilt
    )
    assert got_b == pytest.approx(bearing, abs=1e-9)
    assert got_e == pytest.approx(elevation, abs=1e-9)


def test_centred_face_reports_the_current_head_bearing():
    tracker = make_tracker()
    b, e = tracker.pixel_to_body_angles(
        FRAME_W / 2, FRAME_H / 2, FRAME_W, FRAME_H, 0.3, -0.1
    )
    assert (b, e) == pytest.approx((0.3, -0.1))


# ── Limits ───────────────────────────────────────────────────────────────


def test_head_never_exceeds_its_mechanical_range():
    """A face parked at the frame edge drives the anchor outward every frame;
    the clamp is the only thing stopping it."""
    tracker = make_tracker(anchor_hold_frames=1, head_reanchor_threshold_rad=0.01)
    pan_info, tilt_info = JOINTS["head_pan"], JOINTS["head_tilt"]
    max_pan = min(abs(pan_info.rad_min), abs(pan_info.rad_max))
    edge = face_at(FRAME_W - 1, FRAME_H - 1)
    for i in range(200):
        out = tracker.update(edge, LEVEL, 100.0 + i * 0.1)
        if out is None:
            continue
        assert abs(out.head_pan) <= max_pan + 1e-9
        assert tilt_info.rad_min - 1e-9 <= out.head_tilt <= tilt_info.rad_max + 1e-9


# ── Output shaping ───────────────────────────────────────────────────────


def test_eyes_only_output_omits_the_head_joints():
    tracker = make_tracker()
    out = run(tracker, face_at(FRAME_W * 0.8, FRAME_H / 2), ticks=3)
    assert set(out.as_targets(eyes_only=True)) == {"eyes_pan", "eyes_tilt"}
    assert set(out.as_targets()) == {"eyes_pan", "eyes_tilt", "head_pan", "head_tilt"}


# ── Config ───────────────────────────────────────────────────────────────


def test_config_from_mapping_ignores_unknown_keys_and_coerces_types():
    cfg = TrackerConfig.from_mapping(
        {"kp_eye_pan": "0.9", "anchor_hold_frames": 4,
         "enable_eye_saccades": False, "not_a_real_key": 1}
    )
    assert cfg.kp_eye_pan == pytest.approx(0.9)
    assert cfg.anchor_hold_frames == 4
    assert cfg.enable_eye_saccades is False
    assert cfg.camera_hfov_deg == pytest.approx(62.0)   # untouched default


def test_reset_clears_the_anchor_between_sessions():
    """A new track action must not inherit the previous one's anchor."""
    tracker = make_tracker(anchor_hold_frames=1)
    run(tracker, face_at(FRAME_W * 0.9, FRAME_H / 2), ticks=3)
    assert tracker._seeded is True
    tracker.reset()
    assert tracker._seeded is False
    assert tracker._anchor_bearing == 0.0
    assert tracker._smooth_cx is None
