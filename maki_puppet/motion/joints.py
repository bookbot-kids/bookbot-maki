"""Joint metadata and unit conversions for the MAKI puppet stack.

Forked from maki-rpi5-v0.2.13-rc.4/src/maki_servos/maki_servos/servo_node.py
(ID_TO_JOINT_NAME, DEFAULT_HARDWARE_SIGN, JOINT_LIMITS_TICKS, tick<->rad math)
and maki-rpi5-v0.2.13-rc.4/src/maki_behavior/maki_behavior/face_expression_node.py
(eyelid open/closed rad constants, shared eyelid stroke); guards preserved — see
plan checklist.  Mouth open/closed radians come from
maki-rpi5-v0.2.13-rc.4/src/maki_behavior/maki_behavior/explore_node.py.

Unit model
----------
* **Command radians** (used everywhere inside maki_puppet, including
  ``MotionService`` and ``ServoBus.write_targets_rad``) are body-centric:
  ``tick = round(2048 + sign * rad * 2048 / pi)``.  The hardware sign (incl.
  the mirrored right eyelid) is folded in at the ServoBus tick boundary only.
* **Normalized units** (the wire protocol) map each joint's safe range to
  [-1, +1] with 0 at the configured neutral.  The mapping is piecewise linear
  around neutral so that -1 and +1 always reach the range ends even when the
  neutral is not the range midpoint (e.g. head_tilt: ticks 1800–2150 with
  neutral 2048).
* **Openness** [0, 1] (0 = closed, 1 = open) is the wire unit for the logical
  ``eyelids`` and ``mouth`` joints.

Neutral policy
--------------
Default neutral is the vendored stack's effective home: behavior nodes command
rad 0.0, which servo_node converts to tick 2048 and clamps into
``JOINT_LIMITS_TICKS``.  We reproduce that as
``neutral_tick = clamp(2048, tick_min, tick_max)`` (for every current MAKI
joint 2048 lies inside the safe range, so neutral_rad == 0.0).  A per-joint
``neutral_tick`` in ``puppet.yaml["servo"]["joints"]`` overrides the default
(see :func:`joints_from_config`).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, Iterable, Mapping, Optional, Tuple

# Center position for all servos (ticks) — from servo_node.py
DXL_CENTER_TICK = 2048
DXL_MIN_TICK = 0
DXL_MAX_TICK = 4095


def tick_to_command_rad(sign: int, tick: int) -> float:
    """Convert a Dynamixel tick to body-centric command radians."""
    return sign * (tick - DXL_CENTER_TICK) * math.pi / DXL_CENTER_TICK


def command_rad_to_tick(sign: int, rad: float) -> int:
    """Convert body-centric command radians to a Dynamixel tick.

    Uses ``round()`` (not ``int()``) — guard preserved from servo_node.py:
    truncation introduces a one-tick left/right mismatch on the mirrored
    eyelids, visibly desynchronizing blinks.
    """
    return round(DXL_CENTER_TICK + (sign * rad * DXL_CENTER_TICK / math.pi))


@dataclass(frozen=True)
class JointInfo:
    """Static metadata for one physical joint."""

    name: str
    servo_id: int
    model: str
    sign: int                 # hardware direction (body-centric +rad convention)
    tick_min: int
    tick_max: int
    neutral_tick: int
    rad_min: float            # command-rad range (sign folded in, so min<=max)
    rad_max: float
    neutral_rad: float


def _make_joint(
    name: str,
    servo_id: int,
    model: str,
    sign: int,
    ticks: Tuple[int, int],
    neutral_tick: Optional[int] = None,
) -> JointInfo:
    tick_min, tick_max = int(ticks[0]), int(ticks[1])
    if neutral_tick is None:
        # Default neutral: the vendored clamped-2048 home (see module docstring).
        neutral_tick = min(max(DXL_CENTER_TICK, tick_min), tick_max)
    a = tick_to_command_rad(sign, tick_min)
    b = tick_to_command_rad(sign, tick_max)
    return JointInfo(
        name=name,
        servo_id=servo_id,
        model=model,
        sign=sign,
        tick_min=tick_min,
        tick_max=tick_max,
        neutral_tick=int(neutral_tick),
        rad_min=min(a, b),
        rad_max=max(a, b),
        neutral_rad=tick_to_command_rad(sign, int(neutral_tick)),
    )


# Canonical joint table — values from servo_node.py (ID_TO_JOINT_NAME,
# DEFAULT_HARDWARE_SIGN, JOINT_LIMITS_TICKS).  Guard comments preserved.
_JOINT_TABLE = (
    #  name           id  model    sign  ticks
    ("head_pan",       1, "XL430", -1, (1348, 2748)),
    # Down end at the vendored design limit (2300) so the sleep pose can droop
    # fully; up end stays tightened at 1800. positive rad = DOWN.
    ("head_tilt",      2, "XL430",  1, (1800, 2300)),
    ("eyes_tilt",      3, "XL430",  1, (1600, 2200)),
    ("eyes_pan",       4, "XL430",  1, (1846, 2250)),
    ("left_eyelid",    5, "XL430",  1, (1448, 2096)),
    # Eyelid servos face each other, so mirror the right eyelid to keep blinks in sync.
    ("right_eyelid",   6, "XL430", -1, (2000, 2600)),
    ("mouth",          7, "XL330",  1, (1960, 2500)),
)

JOINTS: Dict[str, JointInfo] = {
    row[0]: _make_joint(*row) for row in _JOINT_TABLE
}

# Order matches the vendored planner's ALL_JOINTS / servo ID order.
JOINT_NAMES = list(JOINTS)

# Logical wire-protocol joints resolved by the gateway via the helpers below.
LOGICAL_JOINTS = ("eyelids", "mouth")


def joints_from_config(servo_joints_cfg: Mapping[str, Mapping]) -> Dict[str, JointInfo]:
    """Build a JOINTS table from ``puppet.yaml["servo"]["joints"]``.

    Each entry needs ``id``, ``sign``, ``ticks: [min, max]`` and may carry
    ``model`` and ``neutral_tick`` (defaults to the clamped-2048 policy).
    """
    out: Dict[str, JointInfo] = {}
    for name, cfg in servo_joints_cfg.items():
        out[name] = _make_joint(
            name=name,
            servo_id=int(cfg["id"]),
            model=str(cfg.get("model", "XL430")),
            sign=int(cfg.get("sign", 1)),
            ticks=(int(cfg["ticks"][0]), int(cfg["ticks"][1])),
            neutral_tick=cfg.get("neutral_tick"),
        )
    return out


# ── Normalized [-1, +1] <-> command radians ───────────────────────────────


def clamp_rad(joint: str, rad: float, joints: Optional[Mapping[str, JointInfo]] = None) -> float:
    """Clamp a command-rad value into the joint's safe range.

    NOTE: this mirrors — but does not replace — the tick clamp in ServoBus,
    which remains the single hardware choke point (limits are NOT in servo
    EEPROM).
    """
    info = (joints or JOINTS)[joint]
    return min(max(rad, info.rad_min), info.rad_max)


def norm_to_rad(joint: str, v: float, joints: Optional[Mapping[str, JointInfo]] = None) -> float:
    """Normalized [-1, +1] → command radians (piecewise linear around neutral).

    +1 maps to rad_max, -1 to rad_min, 0 to the configured neutral.  Input is
    clamped to [-1, +1].  Raises ``KeyError`` for unknown joints.
    """
    info = (joints or JOINTS)[joint]
    v = min(max(float(v), -1.0), 1.0)
    if v >= 0.0:
        span = info.rad_max - info.neutral_rad
    else:
        span = info.neutral_rad - info.rad_min
    return info.neutral_rad + v * span


def rad_to_norm(joint: str, rad: float, joints: Optional[Mapping[str, JointInfo]] = None) -> float:
    """Command radians → normalized [-1, +1] (inverse of :func:`norm_to_rad`).

    Out-of-range radians clamp to ±1.
    """
    info = (joints or JOINTS)[joint]
    rad = min(max(float(rad), info.rad_min), info.rad_max)
    if rad >= info.neutral_rad:
        span = info.rad_max - info.neutral_rad
    else:
        span = info.neutral_rad - info.rad_min
    if span <= 1e-12:
        return 0.0
    return (rad - info.neutral_rad) / span


def neutral_pose_rad(
    joints: Optional[Mapping[str, JointInfo]] = None,
    *,
    exclude: Iterable[str] = (),
) -> Dict[str, float]:
    """Neutral command-rad pose for every physical joint.

    ``exclude`` drops joints the caller must not command — used to leave the
    mouth alone, since it is externally driven by the viseme stream (§5.13)
    and homing it mid-speech would snap it shut.
    """
    table = joints or JOINTS
    skip = set(exclude)
    return {
        name: info.neutral_rad
        for name, info in table.items()
        if name not in skip
    }


# ── Eyelid openness mapping ───────────────────────────────────────────────
# Ported from face_expression_node.py (~lines 90–112).  In command space the
# positive direction is "open" for both lids (the right lid's hardware mirror
# is folded into its sign), so open == rad_max and closed == rad_min.
#
# Guard preserved: both eyelids use ONE shared command-space stroke so
# synchronized motions do not introduce a tiny left/right mismatch from the
# two servo calibration spans.  The closed end uses the safer (less deep) of
# the two calibrated limits so neither lid is overdriven.

_LEFT_OPEN_RAD = JOINTS["left_eyelid"].rad_max      # tick 2096 → +48π/2048
_LEFT_CLOSED_RAD = JOINTS["left_eyelid"].rad_min    # tick 1448 → -600π/2048
_RIGHT_OPEN_RAD = JOINTS["right_eyelid"].rad_max    # tick 2000 → +48π/2048
_RIGHT_CLOSED_RAD = JOINTS["right_eyelid"].rad_min  # tick 2600 → -552π/2048

EYELID_SYNC_OPEN_RAD = (_LEFT_OPEN_RAD + _RIGHT_OPEN_RAD) / 2.0
EYELID_SYNC_CLOSED_RAD = max(_LEFT_CLOSED_RAD, _RIGHT_CLOSED_RAD)


def eyelids_openness_to_rad(openness: float) -> Dict[str, float]:
    """Openness (0 = closed, 1 = open) → command radians for BOTH eyelids.

    Note the direction flip vs. the vendored face_expression fraction, which
    is 0 = open, 1 = closed.  Both lids receive the same command rad; the
    right lid's physical mirroring is applied by ServoBus via its sign.
    """
    v = min(max(float(openness), 0.0), 1.0)
    rad = EYELID_SYNC_OPEN_RAD + (EYELID_SYNC_CLOSED_RAD - EYELID_SYNC_OPEN_RAD) * (1.0 - v)
    return {"left_eyelid": rad, "right_eyelid": rad}


def eyelids_rad_to_openness(rad: float) -> float:
    """Command radians (either lid) → openness [0, 1]. Inverse of the above."""
    span = EYELID_SYNC_OPEN_RAD - EYELID_SYNC_CLOSED_RAD
    if span <= 1e-12:
        return 1.0
    v = (rad - EYELID_SYNC_CLOSED_RAD) / span
    return min(max(v, 0.0), 1.0)


# ── Mouth openness mapping ────────────────────────────────────────────────
# From explore_node.py: "These radians map to safe ticks within
# JOINT_LIMITS_TICKS['mouth']" — closed -0.10 rad (tick ~1983),
# open +0.50 rad (tick ~2374), both inside (1960, 2500).

MOUTH_CLOSED_RAD = -0.10
MOUTH_OPEN_RAD = 0.50


def mouth_openness_to_rad(openness: float) -> float:
    """Openness (0 = closed, 1 = open) → mouth command radians."""
    v = min(max(float(openness), 0.0), 1.0)
    return MOUTH_CLOSED_RAD + (MOUTH_OPEN_RAD - MOUTH_CLOSED_RAD) * v


def mouth_rad_to_openness(rad: float) -> float:
    """Mouth command radians → openness [0, 1]."""
    span = MOUTH_OPEN_RAD - MOUTH_CLOSED_RAD
    v = (float(rad) - MOUTH_CLOSED_RAD) / span
    return min(max(v, 0.0), 1.0)
