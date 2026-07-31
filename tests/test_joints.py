"""Unit tests for joint metadata and unit conversions."""

import math

import pytest

from maki_puppet.motion import joints as J


TICK = math.pi / 2048.0  # radians per tick


# ── Metadata ──────────────────────────────────────────────────────────


def test_all_seven_joints_present():
    assert list(J.JOINTS) == [
        "head_pan",
        "head_tilt",
        "eyes_tilt",
        "eyes_pan",
        "left_eyelid",
        "right_eyelid",
        "mouth",
    ]
    ids = [info.servo_id for info in J.JOINTS.values()]
    assert sorted(ids) == [1, 2, 3, 4, 5, 6, 7]


def test_signs_match_vendored_hardware_map():
    assert J.JOINTS["head_pan"].sign == -1
    assert J.JOINTS["right_eyelid"].sign == -1  # mirrored lid
    for name in ("head_tilt", "eyes_pan", "eyes_tilt", "left_eyelid", "mouth"):
        assert J.JOINTS[name].sign == 1


def test_rad_ranges_from_tick_tables():
    # head_pan: sign -1, ticks 1348..2748 → symmetric ±700 ticks
    hp = J.JOINTS["head_pan"]
    assert hp.rad_min == pytest.approx(-700 * TICK)
    assert hp.rad_max == pytest.approx(700 * TICK)

    # head_tilt: sign +1, ticks 1800..2150 → asymmetric around 2048
    ht = J.JOINTS["head_tilt"]
    assert ht.rad_min == pytest.approx(-248 * TICK)
    assert ht.rad_max == pytest.approx(252 * TICK)   # down end widened to 2300

    # right_eyelid: sign -1 folds the range: 2000..2600 → [-552, +48] ticks
    re = J.JOINTS["right_eyelid"]
    assert re.rad_min == pytest.approx(-552 * TICK)
    assert re.rad_max == pytest.approx(48 * TICK)


def test_neutral_defaults_to_clamped_center_tick():
    # 2048 is inside every MAKI joint range → neutral_rad == 0.0 everywhere
    for info in J.JOINTS.values():
        assert info.neutral_tick == 2048
        assert info.neutral_rad == pytest.approx(0.0)


def test_neutral_tick_override_via_config():
    table = J.joints_from_config({
        "head_tilt": {"id": 2, "model": "XL430", "sign": 1,
                      "ticks": [1800, 2150], "neutral_tick": 1975},
    })
    info = table["head_tilt"]
    assert info.neutral_tick == 1975
    assert info.neutral_rad == pytest.approx((1975 - 2048) * TICK)
    # norm 0 hits the configured neutral, ±1 still hit the range ends
    assert J.norm_to_rad("head_tilt", 0.0, table) == pytest.approx(info.neutral_rad)
    assert J.norm_to_rad("head_tilt", 1.0, table) == pytest.approx(info.rad_max)
    assert J.norm_to_rad("head_tilt", -1.0, table) == pytest.approx(info.rad_min)


def test_config_neutral_outside_range_is_clamped():
    # A joint whose range excludes 2048 gets the clamped-center policy
    table = J.joints_from_config({
        "fake": {"id": 9, "sign": 1, "ticks": [1000, 1500]},
    })
    assert table["fake"].neutral_tick == 1500
    assert table["fake"].neutral_rad == pytest.approx((1500 - 2048) * TICK)


# ── norm <-> rad ──────────────────────────────────────────────────────


def test_norm_to_rad_endpoints_every_joint():
    for name, info in J.JOINTS.items():
        assert J.norm_to_rad(name, 0.0) == pytest.approx(info.neutral_rad)
        assert J.norm_to_rad(name, 1.0) == pytest.approx(info.rad_max)
        assert J.norm_to_rad(name, -1.0) == pytest.approx(info.rad_min)


@pytest.mark.parametrize("v", [-1.0, -0.75, -0.3, 0.0, 0.4, 0.9, 1.0])
def test_norm_rad_round_trip(v):
    for name in J.JOINTS:
        rad = J.norm_to_rad(name, v)
        assert J.rad_to_norm(name, rad) == pytest.approx(v, abs=1e-9)


def test_norm_asymmetric_head_tilt_slopes_differ():
    # neutral 2048 is NOT the tick-range midpoint (1800..2300), so equal norm
    # steps map to different rad steps per side. Sign convention: positive rad
    # is DOWN (see eye_gaze_node.py:110), so -0.5 is up and +0.5 is down.
    up = J.norm_to_rad("head_tilt", -0.5)
    down = J.norm_to_rad("head_tilt", 0.5)
    assert abs(up) == pytest.approx(0.5 * 248 * TICK)
    assert abs(down) == pytest.approx(0.5 * 252 * TICK)
    assert abs(up) != pytest.approx(abs(down))


def test_norm_input_clamped():
    assert J.norm_to_rad("head_pan", 5.0) == pytest.approx(J.JOINTS["head_pan"].rad_max)
    assert J.norm_to_rad("head_pan", -5.0) == pytest.approx(J.JOINTS["head_pan"].rad_min)


def test_rad_to_norm_clamps_out_of_range():
    assert J.rad_to_norm("head_tilt", 10.0) == pytest.approx(1.0)
    assert J.rad_to_norm("head_tilt", -10.0) == pytest.approx(-1.0)


def test_unknown_joint_raises_keyerror():
    with pytest.raises(KeyError):
        J.norm_to_rad("elbow", 0.5)
    with pytest.raises(KeyError):
        J.rad_to_norm("elbow", 0.0)
    with pytest.raises(KeyError):
        J.clamp_rad("elbow", 0.0)


# ── clamp_rad ─────────────────────────────────────────────────────────


def test_clamp_rad():
    hp = J.JOINTS["head_pan"]
    assert J.clamp_rad("head_pan", 99.0) == pytest.approx(hp.rad_max)
    assert J.clamp_rad("head_pan", -99.0) == pytest.approx(hp.rad_min)
    assert J.clamp_rad("head_pan", 0.1) == pytest.approx(0.1)


# ── neutral pose ──────────────────────────────────────────────────────


def test_neutral_pose_rad():
    pose = J.neutral_pose_rad()
    assert set(pose) == set(J.JOINTS)
    for name, rad in pose.items():
        info = J.JOINTS[name]
        assert info.rad_min <= rad <= info.rad_max
        assert rad == pytest.approx(0.0)  # clamped-2048 policy


# ── Eyelid openness mapping ───────────────────────────────────────────


def test_eyelid_sync_constants_match_face_expression_node():
    # Both lids open at +48 ticks from center; closed uses the SAFER (right)
    # limit: -552 ticks — never the deeper left limit of -600.
    assert J.EYELID_SYNC_OPEN_RAD == pytest.approx(48 * TICK)
    assert J.EYELID_SYNC_CLOSED_RAD == pytest.approx(-552 * TICK)


def test_eyelids_openness_both_lids_same_command_rad():
    for v in (0.0, 0.25, 0.5, 1.0):
        out = J.eyelids_openness_to_rad(v)
        assert set(out) == {"left_eyelid", "right_eyelid"}
        assert out["left_eyelid"] == out["right_eyelid"]


def test_eyelids_openness_endpoints_and_direction():
    closed = J.eyelids_openness_to_rad(0.0)["left_eyelid"]
    opened = J.eyelids_openness_to_rad(1.0)["left_eyelid"]
    assert closed == pytest.approx(J.EYELID_SYNC_CLOSED_RAD)
    assert opened == pytest.approx(J.EYELID_SYNC_OPEN_RAD)
    assert opened > closed  # more open = more positive command rad
    # Monotonic in openness
    half = J.eyelids_openness_to_rad(0.5)["left_eyelid"]
    assert closed < half < opened


def test_eyelids_openness_clamped_and_within_both_safe_ranges():
    over = J.eyelids_openness_to_rad(2.0)
    under = J.eyelids_openness_to_rad(-1.0)
    assert over["left_eyelid"] == pytest.approx(J.EYELID_SYNC_OPEN_RAD)
    assert under["left_eyelid"] == pytest.approx(J.EYELID_SYNC_CLOSED_RAD)
    # The shared stroke never overdrives either lid
    for v in (0.0, 0.5, 1.0):
        rad = J.eyelids_openness_to_rad(v)["left_eyelid"]
        assert J.clamp_rad("left_eyelid", rad) == pytest.approx(rad)
        assert J.clamp_rad("right_eyelid", rad) == pytest.approx(rad)


def test_eyelids_openness_round_trip():
    for v in (0.0, 0.3, 0.7, 1.0):
        rad = J.eyelids_openness_to_rad(v)["left_eyelid"]
        assert J.eyelids_rad_to_openness(rad) == pytest.approx(v)


# ── Mouth openness mapping ────────────────────────────────────────────


def test_mouth_openness_endpoints():
    # Constants from vendored explore_node.py
    assert J.mouth_openness_to_rad(0.0) == pytest.approx(-0.10)
    assert J.mouth_openness_to_rad(1.0) == pytest.approx(0.50)
    assert J.mouth_openness_to_rad(0.5) == pytest.approx(0.20)


def test_mouth_openness_clamped_and_safe():
    assert J.mouth_openness_to_rad(9.0) == pytest.approx(0.50)
    assert J.mouth_openness_to_rad(-9.0) == pytest.approx(-0.10)
    for v in (0.0, 0.5, 1.0):
        rad = J.mouth_openness_to_rad(v)
        assert J.clamp_rad("mouth", rad) == pytest.approx(rad)


def test_mouth_openness_round_trip():
    for v in (0.0, 0.4, 1.0):
        assert J.mouth_rad_to_openness(J.mouth_openness_to_rad(v)) == pytest.approx(v)


# ── tick conversion helpers ───────────────────────────────────────────


def test_tick_round_not_truncate():
    # round() guard: eyelid sync depends on symmetric rounding, not int().
    rad = J.tick_to_command_rad(1, 2048 + 1) / 2  # half a tick
    assert J.command_rad_to_tick(1, rad) == 2048  # 0.5 rounds to even (2048)
    rad_075 = 0.75 * TICK
    assert J.command_rad_to_tick(1, rad_075) == 2049


def test_tick_rad_round_trip_with_sign():
    for sign in (1, -1):
        for tick in (1348, 1800, 2048, 2150, 2748):
            rad = J.tick_to_command_rad(sign, tick)
            assert J.command_rad_to_tick(sign, rad) == tick


def test_neutral_pose_rad_exclude_drops_joint():
    full = J.neutral_pose_rad()
    assert "mouth" in full
    partial = J.neutral_pose_rad(exclude=("mouth",))
    assert "mouth" not in partial
    assert set(partial) == set(full) - {"mouth"}
