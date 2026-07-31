"""Pure-logic tests for maki_puppet.hw.servo_bus.

These exercise the module-level conversion/clamping helpers WITHOUT
dynamixel_sdk — the module must import and the choke-point math must work on
a machine with no hardware SDK installed.
"""

import importlib.util
import math
import sys

import maki_puppet.hw.servo_bus as sb

PI = math.pi
TICKS_PER_RAD = sb.DXL_CENTER_TICK / PI  # 2048 / pi


# ----------------------------------------------------------------------
# Import safety
# ----------------------------------------------------------------------

def test_module_imports_without_dynamixel_sdk():
    # On dev machines the SDK is absent; importing servo_bus (done above)
    # must not have pulled it in. On the robot (SDK installed) this check is
    # vacuous for sys.modules, so only assert when the SDK is unavailable.
    if importlib.util.find_spec("dynamixel_sdk") is None:
        assert "dynamixel_sdk" not in sys.modules


# ----------------------------------------------------------------------
# Vendored constants survived the fork
# ----------------------------------------------------------------------

def test_joint_limits_match_vendored():
    assert sb.JOINT_LIMITS_TICKS == {
        "head_pan": (1348, 2748),
        "head_tilt": (1800, 2300),
        "eyes_tilt": (1600, 2200),
        "eyes_pan": (1846, 2250),
        "left_eyelid": (1448, 2096),
        "right_eyelid": (2000, 2600),
        "mouth": (1960, 2500),
    }


def test_hardware_sign_map_matches_vendored():
    assert sb.DEFAULT_HARDWARE_SIGN["head_pan"] == -1
    assert sb.DEFAULT_HARDWARE_SIGN["right_eyelid"] == -1  # mirrored eyelid
    for joint in ("head_tilt", "eyes_pan", "eyes_tilt", "left_eyelid", "mouth"):
        assert sb.DEFAULT_HARDWARE_SIGN[joint] == 1


def test_id_map_matches_vendored():
    assert sb.ID_TO_JOINT_NAME == {
        1: "head_pan",
        2: "head_tilt",
        3: "eyes_tilt",
        4: "eyes_pan",
        5: "left_eyelid",
        6: "right_eyelid",
        7: "mouth",
    }


def test_profile_velocity_defaults_match_vendored_yaml():
    assert sb.DEFAULT_PROFILE_VELOCITY["head_pan"] == 130
    assert sb.DEFAULT_PROFILE_VELOCITY["head_tilt"] == 110
    for joint in ("eyes_pan", "eyes_tilt", "left_eyelid", "right_eyelid", "mouth"):
        assert sb.DEFAULT_PROFILE_VELOCITY[joint] == 0


def test_position_pid_defaults_match_vendored_yaml():
    assert sb.DEFAULT_POSITION_PID["head_pan"] == {"p": 300, "i": 0, "d": 4000}
    assert sb.DEFAULT_POSITION_PID["head_tilt"] == {"p": 600, "i": 0, "d": 4200}
    for joint in ("eyes_pan", "eyes_tilt", "left_eyelid", "right_eyelid", "mouth"):
        assert sb.DEFAULT_POSITION_PID[joint] == {"p": 0, "i": 0, "d": 0}


# ----------------------------------------------------------------------
# rad -> tick choke point: sign map, round(), clamping
# ----------------------------------------------------------------------

def test_neutral_rad_is_center_tick():
    for joint in sb.ID_TO_JOINT_NAME.values():
        assert sb.rad_to_tick(joint, 0.0) == sb.DXL_CENTER_TICK


def test_sign_map_applied():
    # head_pan has hardware sign -1: positive body-centric rad -> tick < 2048
    assert sb.rad_to_tick("head_pan", 0.3) < sb.DXL_CENTER_TICK
    # head_tilt has sign +1: positive rad -> tick > 2048
    assert sb.rad_to_tick("head_tilt", 0.05) > sb.DXL_CENTER_TICK


def test_round_not_int_truncation():
    # offset of 10.7 ticks must round to 11, not truncate to 10
    rad = 10.7 / TICKS_PER_RAD
    assert sb.rad_to_tick("left_eyelid", rad) == 2048 + 11
    # ...and -10.7 must round to -11 symmetrically
    assert sb.rad_to_tick("left_eyelid", -rad) == 2048 - 11


def test_mirrored_eyelids_stay_tick_synchronized():
    # The same closure command must move both eyelids by the same magnitude
    # in opposite tick directions (right_eyelid sign is -1). round() (not
    # int()) is what makes the two servos cross tick boundaries at the same
    # command value. Sample rads inside both joints' clamp windows.
    for rad in (-0.8, -0.5123, -0.25, -0.1, -0.0123456, 0.0, 0.0123456, 0.05, 0.07):
        left = sb.rad_to_tick("left_eyelid", rad)
        right = sb.rad_to_tick("right_eyelid", rad)
        assert left - sb.DXL_CENTER_TICK == -(right - sb.DXL_CENTER_TICK), (
            f"eyelids desynchronized at rad={rad}: left={left} right={right}"
        )


def test_clamps_to_per_joint_limits():
    # Way beyond range: head_pan sign -1, so +10 rad slams into the LOW limit
    assert sb.rad_to_tick("head_pan", 10.0) == 1348
    assert sb.rad_to_tick("head_pan", -10.0) == 2748
    # head_tilt (sign +1), tightened limits
    assert sb.rad_to_tick("head_tilt", 1.0) == 2300
    assert sb.rad_to_tick("head_tilt", -1.0) == 1800
    for joint, (lo, hi) in sb.JOINT_LIMITS_TICKS.items():
        assert lo <= sb.rad_to_tick(joint, 100.0) <= hi
        assert lo <= sb.rad_to_tick(joint, -100.0) <= hi


def test_global_clamp_for_unknown_joint():
    # Unknown joints fall back to the global 0..4095 clamp
    assert sb.rad_to_tick("mystery", 100.0, {"mystery": 1}, {}) == sb.DXL_MAX_TICK
    assert sb.rad_to_tick("mystery", -100.0, {"mystery": 1}, {}) == sb.DXL_MIN_TICK


def test_offset_trim_applied_before_conversion():
    off = math.radians(5.0)
    assert sb.rad_to_tick("head_tilt", 0.0, offset_rad=off) == round(
        2048 + off * TICKS_PER_RAD
    )


# ----------------------------------------------------------------------
# tick -> rad feedback path
# ----------------------------------------------------------------------

def test_tick_to_rad_round_trip():
    one_tick_rad = PI / sb.DXL_CENTER_TICK
    cases = {
        "head_pan": (-1.0, 1.0),
        "head_tilt": (-0.35, 0.15),
        "eyes_pan": (-0.3, 0.3),
        "mouth": (-0.13, 0.65),
    }
    for joint, (lo, hi) in cases.items():
        for i in range(11):
            rad = lo + (hi - lo) * i / 10
            back = sb.tick_to_rad(joint, sb.rad_to_tick(joint, rad))
            assert abs(back - rad) <= one_tick_rad, (joint, rad, back)


def test_tick_to_rad_applies_sign():
    # head_pan sign -1: tick below center is POSITIVE body-centric rad
    assert sb.tick_to_rad("head_pan", 1548) > 0
    assert sb.tick_to_rad("head_tilt", 1948) < 0


# ----------------------------------------------------------------------
# Present-load decoding (signed 16-bit, -1000..1000 = -100%..100%)
# ----------------------------------------------------------------------

def test_decode_present_load():
    assert sb.decode_present_load(0) == 0.0
    assert sb.decode_present_load(500) == 0.5
    assert sb.decode_present_load(1000) == 1.0
    assert sb.decode_present_load(2000) == 1.0            # capped
    assert sb.decode_present_load(0x10000 - 500) == 0.5   # negative load
    assert sb.decode_present_load(0xFFFF) == 0.001        # -1 raw


# ----------------------------------------------------------------------
# Goal-position byte packing (DXL_LOBYTE/HIBYTE macro equivalence)
# ----------------------------------------------------------------------

def test_goal_position_bytes_little_endian():
    assert sb.goal_position_bytes(2048) == [0x00, 0x08, 0x00, 0x00]
    assert sb.goal_position_bytes(0) == [0, 0, 0, 0]
    assert sb.goal_position_bytes(4095) == [0xFF, 0x0F, 0x00, 0x00]


# ----------------------------------------------------------------------
# Config merge: defaults survive partial/absent config
# ----------------------------------------------------------------------

def test_build_joint_tables_defaults():
    tables = sb.build_joint_tables({})
    assert list(tables.keys()) == list(sb.ID_TO_JOINT_NAME.values())
    assert tables["right_eyelid"]["sign"] == -1
    assert tables["head_pan"]["id"] == 1
    assert tables["head_pan"]["ticks"] == (1348, 2748)
    assert tables["head_pan"]["profile_velocity"] == 130
    assert tables["head_tilt"]["pid"] == {"p": 600, "i": 0, "d": 4200}
    assert tables["mouth"]["offset_rad"] == 0.0


def test_build_joint_tables_config_overrides():
    cfg = {
        "joints": {
            "head_pan": {
                "id": 1,
                "sign": 1,
                "ticks": [1400, 2700],
                "profile_velocity": 90,
                "pid": {"p": 200, "i": 0, "d": 3000},
                "offset_deg": 2.0,
            },
        }
    }
    tables = sb.build_joint_tables(cfg)
    hp = tables["head_pan"]
    assert hp["sign"] == 1
    assert hp["ticks"] == (1400, 2700)
    assert hp["profile_velocity"] == 90
    assert hp["pid"] == {"p": 200, "i": 0, "d": 3000}
    assert abs(hp["offset_rad"] - math.radians(2.0)) < 1e-12
    # Unconfigured joints keep vendored defaults
    assert tables["right_eyelid"]["sign"] == -1
    assert tables["right_eyelid"]["ticks"] == (2000, 2600)


def test_servo_bus_constructs_without_hardware():
    # Constructor must not touch the SDK, the port, or the lock file.
    bus = sb.ServoBus({"port": "/dev/ttyUSB0", "baud": 1000000})
    assert bus.joint_names == list(sb.ID_TO_JOINT_NAME.values())
    bus.close()  # idempotent no-op before open()
