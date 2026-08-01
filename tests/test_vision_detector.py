"""Detector tests. The real backends need OpenCV, so these cover the pure
parts: primary-face selection, the backend registry, and the closed-loop sim
detector's geometry."""

from __future__ import annotations

import math
import os

import pytest

from maki_puppet.vision.detector import (
    FaceDetection,
    HaarDetector,
    SimFaceDetector,
    YuNetDetector,
    build_detector,
    default_yunet_model_path,
    select_primary,
)


def det(cx, cy, conf=1.0, size=100.0, w=640, h=480) -> FaceDetection:
    return FaceDetection(
        bbox_x=cx - size / 2, bbox_y=cy - size / 2,
        bbox_w=size, bbox_h=size, confidence=conf, frame_w=w, frame_h=h,
    )


# ── FaceDetection ────────────────────────────────────────────────────────


def test_centre_is_derived_from_the_bounding_box():
    face = FaceDetection(
        bbox_x=100, bbox_y=50, bbox_w=80, bbox_h=60,
        confidence=0.9, frame_w=640, frame_h=480,
    )
    assert (face.cx, face.cy) == (140, 80)


# ── Primary selection ────────────────────────────────────────────────────


def test_primary_face_is_the_most_confident():
    faces = [det(100, 100, conf=0.5), det(500, 300, conf=0.95), det(300, 200, conf=0.7)]
    assert select_primary(faces).confidence == 0.95


def test_primary_of_nothing_is_none():
    assert select_primary([]) is None


# ── Registry ─────────────────────────────────────────────────────────────


def test_build_detector_selects_the_configured_backend():
    assert isinstance(build_detector({"face_detector": "haar"}), HaarDetector)
    assert isinstance(build_detector({"face_detector": "yunet"}), YuNetDetector)
    assert isinstance(build_detector({}), YuNetDetector)   # documented default


def test_sim_flag_overrides_the_configured_backend():
    """--sim must never try to open a real model, whatever the config says."""
    assert isinstance(build_detector({"face_detector": "yunet"}, sim=True), SimFaceDetector)


def test_unknown_backend_is_rejected_by_name():
    with pytest.raises(KeyError, match="mediapipe"):
        build_detector({"face_detector": "mediapipe"})


def test_yunet_without_a_model_fails_with_an_actionable_message():
    detector = build_detector({"face_detector": "yunet", "yunet_model_path": "/nope.onnx"})
    with pytest.raises(FileNotFoundError, match="yunet_model_path"):
        detector._ensure()


def test_the_yunet_model_is_vendored_in_the_repo():
    """A fresh deploy must work with no model download step."""
    path = default_yunet_model_path()
    assert path is not None, "models/face_detection_yunet_2023mar.onnx is missing"
    assert os.path.getsize(path) > 100_000


def test_empty_model_path_resolves_to_the_vendored_model():
    detector = build_detector({"face_detector": "yunet", "yunet_model_path": ""})
    assert detector.model_path == default_yunet_model_path()


def test_an_explicit_model_path_wins_over_the_vendored_one():
    detector = build_detector(
        {"face_detector": "yunet", "yunet_model_path": "/custom/model.onnx"}
    )
    assert detector.model_path == "/custom/model.onnx"


# ── Sim detector geometry ────────────────────────────────────────────────

SIM_CFG = {"width": 640, "height": 480, "camera_hfov_deg": 62.0, "camera_vfov_deg": 49.0}


def test_sim_face_is_centred_when_the_head_points_straight_at_it():
    """The closed loop: aim the head at the scripted bearing and the face
    lands dead centre of frame."""
    sim = SimFaceDetector(SIM_CFG)
    bearing, elevation = sim.world_position(3.0)
    sim.pose_provider = lambda: {"head_pan": bearing, "head_tilt": -elevation}
    face = sim.detect(None, now=3.0)[0]
    assert face.cx == pytest.approx(320.0, abs=1e-6)
    assert face.cy == pytest.approx(240.0, abs=1e-6)


def test_turning_the_head_moves_the_face_toward_the_centre():
    """This is what makes --sim converge instead of running the head to its
    limit: following the face actually reduces the error."""
    sim = SimFaceDetector(SIM_CFG)
    bearing, _ = sim.world_position(1.0)

    sim.pose_provider = lambda: {"head_pan": 0.0, "head_tilt": 0.0}
    before = sim.detect(None, now=1.0)[0]
    sim.pose_provider = lambda: {"head_pan": bearing * 0.5, "head_tilt": 0.0}
    after = sim.detect(None, now=1.0)[0]

    assert abs(after.cx - 320.0) < abs(before.cx - 320.0)


def test_sim_reports_no_face_when_the_head_looks_away():
    sim = SimFaceDetector(SIM_CFG)
    sim.pose_provider = lambda: {"head_pan": 1.0, "head_tilt": 0.0}   # way off
    assert sim.detect(None, now=0.0) == []


def test_sim_without_a_pose_provider_still_produces_faces():
    """Degrades to open loop rather than crashing."""
    sim = SimFaceDetector(SIM_CFG)
    assert len(sim.detect(None, now=0.0)) == 1


def test_sim_pose_provider_failure_does_not_break_detection():
    def boom():
        raise RuntimeError("no motion service")

    sim = SimFaceDetector(SIM_CFG, pose_provider=boom)
    assert len(sim.detect(None, now=0.0)) == 1   # falls back to a level head


def test_sim_bearing_stays_inside_the_scripted_amplitude():
    sim = SimFaceDetector({**SIM_CFG, "sim_face_bearing_rad": 0.45})
    for i in range(50):
        bearing, _ = sim.world_position(i * 0.37)
        assert abs(bearing) <= 0.45 + 1e-9
