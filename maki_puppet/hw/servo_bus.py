"""Dynamixel servo bus driver for MAKI puppet mode.

Forked from maki-rpi5-v0.2.13-rc.4/src/maki_servos/maki_servos/servo_node.py; guards preserved — see plan checklist.

Preserved guards (each exists for an observed hardware failure):
  1. JOINT_LIMITS_TICKS clamping in ONE choke point (``rad_to_tick``, called
     only from ``write_targets_rad``) — limits are NOT stored in servo EEPROM.
  2. Hardware sign map incl. mirrored right eyelid.
  3. ``round()`` (not ``int()``) tick conversion — keeps mirrored eyelids
     crossing tick boundaries at the same command value (synchronized blinks).
  4. ``flock(LOCK_EX | LOCK_NB)`` on the same lock file the ROS servo node
     uses → fail-fast mutual exclusion in both directions.
  5. Torque-off on ``atexit`` AND ``SIGTERM`` (atexit does not fire on SIGTERM
     without an explicit handler).
  6. ``port_lock`` (threading.Lock) serializing the half-duplex bus.
  7. GroupBulkRead feedback with out-of-range tick rejection, partial-packet
     IndexError handling, and hold-last-known-value on bus failure
     (``fresh=False``).
  8. Exponential-backoff + port reopen after consecutive read failures.
     (Fork fix: the vendored ``_reopen_port_and_rebuild_bulkread`` re-acquired
     the non-reentrant ``port_lock`` while it was already held by the caller —
     a latent deadlock on the Nth consecutive failure.  Here the reopen runs
     with the lock already held.)
  9. Profile-velocity + PID init writes at ``open()``.

``dynamixel_sdk`` is imported inside ``open()`` so this module (and its pure
conversion helpers) imports cleanly on machines without the SDK (``--sim``).
"""

from __future__ import annotations

import atexit
import logging
import math
import os
import signal
import sys
import threading
import time
from dataclasses import dataclass
from typing import Any, Optional

log = logging.getLogger(__name__)

# --- Dynamixel control table constants (XL430/XL330, protocol 2.0) ---
# (servo_node.py lines 46-75)

ADDR_TORQUE_ENABLE = 64
ADDR_HARDWARE_ERROR_STATUS = 70

# Position PID gains (RAM area — writable with torque on/off)
ADDR_POSITION_D_GAIN = 80   # 2 bytes, default 0
ADDR_POSITION_I_GAIN = 82   # 2 bytes, default 0
ADDR_POSITION_P_GAIN = 84   # 2 bytes, default 800

ADDR_PROFILE_VELOCITY = 112
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_LOAD = 126
ADDR_PRESENT_VELOCITY = 128
ADDR_PRESENT_POSITION = 132

LEN_GOAL_POSITION = 4
LEN_PROFILE_VELOCITY = 4
LEN_PRESENT_POSITION = 4
LEN_PRESENT_LOAD = 2
LEN_PID_GAIN = 2

TORQUE_ENABLE = 1
TORQUE_DISABLE = 0

# --- MAKI joint mapping (servo_node.py lines 78-123) ---

# Center position for all servos (ticks)
DXL_CENTER_TICK = 2048
DXL_MIN_TICK = 0
DXL_MAX_TICK = 4095

# Canonical mapping from Dynamixel ID → joint base name
ID_TO_JOINT_NAME = {
    1: "head_pan",
    2: "head_tilt",
    3: "eyes_tilt",
    4: "eyes_pan",
    5: "left_eyelid",
    6: "right_eyelid",
    7: "mouth",
}

JOINT_NAME_TO_ID = {name: dxl_id for dxl_id, name in ID_TO_JOINT_NAME.items()}

# Per-joint hardware direction (body-centric +rad convention)
DEFAULT_HARDWARE_SIGN = {
    "head_pan": -1,
    "head_tilt": 1,
    "eyes_pan": 1,
    "eyes_tilt": 1,
    "left_eyelid": 1,
    # Eyelid servos face each other, so mirror the right eyelid to keep blinks in sync.
    "right_eyelid": -1,
    "mouth": 1,
}

# Per-joint allowed tick ranges (from user specification).
# These limits are NOT in servo EEPROM — software clamping is the only guard.
JOINT_LIMITS_TICKS = {
    "head_pan": (1348, 2748),       # ID 1 XL430
    "head_tilt": (1800, 2300),      # ID 2 XL430 — down end at vendored design limit
    "eyes_tilt": (1600, 2200),      # ID 3 XL430
    "eyes_pan": (1846, 2250),       # ID 4 XL430
    "left_eyelid": (1448, 2096),    # ID 5 XL430
    "right_eyelid": (2000, 2600),   # ID 6 XL430
    "mouth": (1960, 2500),          # ID 7 XL330
}

JOINT_MODELS = {
    "head_pan": "XL430",
    "head_tilt": "XL430",
    "eyes_tilt": "XL430",
    "eyes_pan": "XL430",
    "left_eyelid": "XL430",
    "right_eyelid": "XL430",
    "mouth": "XL330",
}

# Hardware trap-profile smoothing (ticks/s, 0 = leave hardware default).
# From maki_servos_defaults.yaml: reduces oscillation on servos under heavy
# load (especially head_pan carrying the full head assembly).
DEFAULT_PROFILE_VELOCITY = {
    "head_pan": 130,     # XL430 under heavy head load — raised for tracking
    "head_tilt": 110,    # XL430 with head mass — raised for tracking
    "eyes_pan": 0,       # eyes should remain fast/unlimited
    "eyes_tilt": 0,
    "left_eyelid": 0,
    "right_eyelid": 0,
    "mouth": 0,
}

# Position PID gains (0 = leave factory default, no write issued).
# XL430 factory: P=640, I=0, D=3600.  XL330 factory: P=400, I=0, D=0.
# Lowering P reduces oscillation/hunting at rest; raising D adds damping.
DEFAULT_POSITION_PID = {
    "head_pan": {"p": 300, "i": 0, "d": 4000},
    "head_tilt": {"p": 600, "i": 0, "d": 4200},
    "eyes_pan": {"p": 0, "i": 0, "d": 0},
    "eyes_tilt": {"p": 0, "i": 0, "d": 0},
    "left_eyelid": {"p": 0, "i": 0, "d": 0},
    "right_eyelid": {"p": 0, "i": 0, "d": 0},
    "mouth": {"p": 0, "i": 0, "d": 0},
}

# Bulk-read resiliency defaults (maki_servos_defaults.yaml)
DEFAULT_READ_RETRY_BACKOFF_INITIAL_MS = 20.0
DEFAULT_READ_RETRY_BACKOFF_MAX_MS = 200.0
DEFAULT_MAX_CONSECUTIVE_FAILURES_BEFORE_REOPEN = 5


# ----------------------------------------------------------------------
# Pure conversion helpers (module-level, testable without dynamixel_sdk).
# rad_to_tick is the single clamping choke point used by write_targets_rad.
# ----------------------------------------------------------------------

def rad_to_tick(
    joint: str,
    rad: float,
    sign_map: Optional[dict[str, int]] = None,
    limits_map: Optional[dict[str, tuple[int, int]]] = None,
    offset_rad: float = 0.0,
) -> int:
    """Convert a body-centric radian command to a clamped goal tick.

    Ported from servo_node.py _joint_state_cb (lines 893-915): sign map,
    per-joint zero-trim, round() conversion, global clamp, then per-joint
    JOINT_LIMITS_TICKS clamp.  This is the ONE choke point — every goal
    position must pass through here.
    """
    if sign_map is None:
        sign_map = DEFAULT_HARDWARE_SIGN
    if limits_map is None:
        limits_map = JOINT_LIMITS_TICKS
    sign = sign_map.get(joint, 1)

    # Apply per-joint zero-trim in body-centric space, then mirror
    rad_cmd = float(rad) + offset_rad

    # Map radians → Dynamixel ticks around center.
    # Use round() (not int()) so that left/right mirrored joints cross tick
    # boundaries at the same command value, keeping eyelid blinks visually
    # synchronised.
    tick = round(DXL_CENTER_TICK + (sign * rad_cmd * DXL_CENTER_TICK / math.pi))
    tick = max(DXL_MIN_TICK, min(DXL_MAX_TICK, tick))

    # Clamp to joint-specific limits (NOT enforced in servo EEPROM)
    joint_min, joint_max = limits_map.get(joint, (DXL_MIN_TICK, DXL_MAX_TICK))
    tick = max(joint_min, min(joint_max, tick))
    return int(tick)


def tick_to_rad(
    joint: str,
    tick: int,
    sign_map: Optional[dict[str, int]] = None,
) -> float:
    """Convert a present-position tick to body-centric radians (feedback path)."""
    if sign_map is None:
        sign_map = DEFAULT_HARDWARE_SIGN
    sign = sign_map.get(joint, 1)
    return sign * (tick - DXL_CENTER_TICK) * math.pi / DXL_CENTER_TICK


def decode_present_load(raw: int) -> float:
    """Decode Present Load: signed 16-bit, -1000..1000 = -100%..100%.

    Returns a normalized absolute value (0.0 = no load, 1.0 = max load).
    Ported from servo_node.py lines 544-551.
    """
    if raw > 0x7FFF:
        raw -= 0x10000
    return min(abs(raw) / 1000.0, 1.0)


def goal_position_bytes(tick: int) -> list[int]:
    """Little-endian 4-byte goal position payload (DXL_LOBYTE/HIBYTE macros)."""
    t = int(tick) & 0xFFFFFFFF
    return [t & 0xFF, (t >> 8) & 0xFF, (t >> 16) & 0xFF, (t >> 24) & 0xFF]


def build_joint_tables(config: Optional[dict]) -> dict[str, dict[str, Any]]:
    """Merge puppet.yaml ``servo.joints`` config over vendored defaults.

    Returns ``{joint_name: {id, sign, ticks, offset_rad, profile_velocity,
    pid}}`` in canonical Dynamixel-ID order.  Starting from the vendored
    defaults means a partial (or absent) config can never silently drop the
    sign map or tick limits.
    """
    joints_cfg = (config or {}).get("joints") or {}
    tables: dict[str, dict[str, Any]] = {}
    names = list(ID_TO_JOINT_NAME.values())
    for extra in joints_cfg:
        if extra not in names:
            names.append(extra)
    for name in names:
        jc = joints_cfg.get(name) or {}
        dxl_id = int(jc.get("id", JOINT_NAME_TO_ID.get(name, 0)))
        if dxl_id <= 0:
            log.warning("Joint '%s' has no Dynamixel ID; skipping.", name)
            continue
        sign_raw = jc.get("sign", DEFAULT_HARDWARE_SIGN.get(name, 1))
        sign = 1 if int(float(sign_raw)) >= 0 else -1
        ticks_raw = jc.get("ticks", JOINT_LIMITS_TICKS.get(name, (DXL_MIN_TICK, DXL_MAX_TICK)))
        ticks = (int(ticks_raw[0]), int(ticks_raw[1]))
        pid_raw = jc.get("pid", DEFAULT_POSITION_PID.get(name, {}))
        tables[name] = {
            "id": dxl_id,
            "sign": sign,
            "ticks": ticks,
            "offset_rad": math.radians(float(jc.get("offset_deg", 0.0))),
            "profile_velocity": int(
                jc.get("profile_velocity", DEFAULT_PROFILE_VELOCITY.get(name, 0))
            ),
            "pid": {
                "p": int(pid_raw.get("p", pid_raw.get("p_gain", 0))),
                "i": int(pid_raw.get("i", pid_raw.get("i_gain", 0))),
                "d": int(pid_raw.get("d", pid_raw.get("d_gain", 0))),
            },
        }
    return tables


@dataclass
class JointFeedback:
    position_rad: float
    load: float          # normalized present-load (0.0 = none, 1.0 = max)
    fresh: bool          # False if last bulk read failed for this joint


class ServoBus:
    """Direct Dynamixel driver for MAKI's 7 servos (U2D2, protocol 2.0).

    ``config`` is ``puppet.yaml["servo"]``; any missing key falls back to the
    vendored defaults above, so the guard values cannot be lost by omission.
    """

    def __init__(self, config: dict) -> None:
        config = config or {}
        self.port_name = str(config.get("port", "/dev/ttyUSB0"))
        self.baud = int(config.get("baud", 1000000))
        self.protocol_version = float(config.get("protocol_version", 2.0))
        # Same lock file the ROS servo node derives (servo_node.py line 368)
        self.lock_file_path = str(
            config.get(
                "lock_file",
                f"/tmp/maki_servo_{os.path.basename(self.port_name)}.lock",
            )
        )

        # Bulk-read resiliency (servo_node.py lines 221-230)
        self._backoff_initial = float(
            config.get("read_retry_backoff_initial_ms", DEFAULT_READ_RETRY_BACKOFF_INITIAL_MS)
        )
        self._backoff_max = float(
            config.get("read_retry_backoff_max_ms", DEFAULT_READ_RETRY_BACKOFF_MAX_MS)
        )
        self._max_failures = int(
            config.get(
                "max_consecutive_failures_before_reopen",
                DEFAULT_MAX_CONSECUTIVE_FAILURES_BEFORE_REOPEN,
            )
        )
        self._backoff_ms = self._backoff_initial
        self._read_fail_count = 0

        self._joints = build_joint_tables(config)
        self._sign = {n: j["sign"] for n, j in self._joints.items()}
        self._limits = {n: j["ticks"] for n, j in self._joints.items()}
        self._id_for = {n: j["id"] for n, j in self._joints.items()}

        # Guard 6: port access lock serializing the half-duplex bus
        # (servo_node.py line 399)
        self.port_lock = threading.Lock()

        # Hold-last-known feedback state (guard 7). Seeded at neutral (0 rad)
        # and fresh=False until the first successful bulk read.
        self._fb_pos_rad: dict[str, float] = {n: 0.0 for n in self._joints}
        self._fb_load: dict[str, float] = {n: 0.0 for n in self._joints}
        self._fb_fresh: dict[str, bool] = {n: False for n in self._joints}

        # SDK handles (populated by open())
        self._sdk: Any = None
        self._port: Any = None
        self._packet: Any = None
        self._bulkread: Any = None
        self._lockfile: Any = None
        self._open = False
        self._torque_off_done = False

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def open(self) -> None:
        """Acquire the port lock, open the bus, init servos, arm shutdown hooks."""
        if self._open:
            return

        # Guard 4: advisory lock file — fail fast if another process (e.g. the
        # ROS servo node) is using the port (servo_node.py lines 366-377).
        try:
            self._lockfile = open(self.lock_file_path, "w")
            import fcntl  # POSIX-only; deferred so module imports anywhere
            fcntl.flock(self._lockfile, fcntl.LOCK_EX | fcntl.LOCK_NB)
            self._lockfile.write(str(os.getpid()))
            self._lockfile.flush()
        except OSError as e:
            if self._lockfile is not None:
                try:
                    self._lockfile.close()
                except OSError:
                    pass
                self._lockfile = None
            log.critical(
                "Could not acquire servo port lock %s; is another process "
                "(e.g. the ROS servo node) using %s? %s",
                self.lock_file_path, self.port_name, e,
            )
            raise SystemExit(1)

        # Pi-only import, deferred so --sim works without dynamixel_sdk.
        import dynamixel_sdk as sdk
        self._sdk = sdk

        self._port = sdk.PortHandler(self.port_name)
        self._packet = sdk.PacketHandler(self.protocol_version)
        if not self._port.openPort() or not self._port.setBaudRate(self.baud):
            self._release_lockfile()
            raise ConnectionError(
                f"Failed to open servo port '{self.port_name}' at {self.baud} bps. "
                "Check device connection, permissions (dialout group), "
                "and that no other process is using the port."
            )
        log.info("Servo port %s opened at %d bps.", self.port_name, self.baud)

        # Guard 5: torque-off on atexit AND SIGTERM (servo_node.py lines
        # 390-396) — atexit does NOT fire on SIGTERM without this handler.
        self._torque_off_done = False
        atexit.register(self._disable_all_servos_on_exit)
        try:
            self._prev_sigterm = signal.getsignal(signal.SIGTERM)
            signal.signal(
                signal.SIGTERM,
                lambda sig, frame: (self._disable_all_servos_on_exit(), sys.exit(0)),
            )
        except ValueError:
            # Not in the main thread — atexit hook still covers normal exits.
            log.warning("Could not install SIGTERM handler (not in main thread).")

        self._open = True
        self._enable_all_servos()
        # Guard 9: hardware trap-profile smoothing + PID damping init
        self._write_profile_velocities()
        self._write_pid_gains()
        self._build_bulkread()

    def close(self) -> None:
        """Torque off, close port, release lock. Idempotent."""
        if self._lockfile is None and not self._open:
            return
        self._disable_all_servos_on_exit()
        try:
            atexit.unregister(self._disable_all_servos_on_exit)
        except Exception:
            pass
        self._release_lockfile()
        self._open = False

    def _release_lockfile(self) -> None:
        if self._lockfile is None:
            return
        try:
            import fcntl
            fcntl.flock(self._lockfile, fcntl.LOCK_UN)
        except OSError:
            pass
        try:
            self._lockfile.close()
        except OSError:
            pass
        self._lockfile = None

    def _disable_all_servos_on_exit(self) -> None:
        """Disables torque on all servos; registered with atexit + SIGTERM.

        Ported from servo_node.py lines 605-621.  Deliberately does NOT take
        port_lock (the vendored exit path doesn't either): a signal handler
        blocking on a lock held by a wedged thread would prevent torque-off.
        """
        if self._torque_off_done or self._port is None:
            return
        self._torque_off_done = True
        log.info("Disabling torque for all servos (shutdown)...")
        try:
            gsw = self._sdk.GroupSyncWrite(
                self._port, self._packet, ADDR_TORQUE_ENABLE, 1
            )
            for dxl_id in self._id_for.values():
                gsw.addParam(dxl_id, [TORQUE_DISABLE])
            result = gsw.txPacket()
            if result == self._sdk.COMM_SUCCESS:
                log.info("Successfully sent torque disable command to all servos.")
            else:
                log.error(
                    "Failed to send torque disable command: %s",
                    self._packet.getTxRxResult(result),
                )
        except Exception as e:  # never let shutdown raise
            log.error("Exception during torque-off: %s", e)
        try:
            self._port.closePort()
        except Exception:
            pass
        log.info("Servo port closed.")

    # ------------------------------------------------------------------
    # Hardware init helpers (ported from servo_node.py)
    # ------------------------------------------------------------------

    def _enable_all_servos(self) -> None:
        """Enable torque on all servos with one GroupSyncWrite packet
        (servo_node.py lines 716-734)."""
        with self.port_lock:
            gsw = self._sdk.GroupSyncWrite(
                self._port, self._packet, ADDR_TORQUE_ENABLE, 1
            )
            for dxl_id in self._id_for.values():
                gsw.addParam(dxl_id, [TORQUE_ENABLE])
            result = gsw.txPacket()
            if result != self._sdk.COMM_SUCCESS:
                log.error(
                    "Failed to send torque enable command to all servos: %s",
                    self._packet.getTxRxResult(result),
                )
            else:
                log.info("Successfully sent torque enable command to all servos.")
            gsw.clearParam()

    def _write_profile_velocities(self) -> None:
        """Write per-joint profile velocities (ADDR 112) to cap motion speed.

        A non-zero profile velocity causes the XL430/XL330 to use a
        trapezoidal velocity profile for every position command, which smooths
        motion and significantly reduces oscillation under heavy load
        (e.g. head_pan carrying the full head assembly).
        0 = leave hardware default.  (servo_node.py lines 623-663)
        """
        with self.port_lock:
            for name, joint in self._joints.items():
                vel_ticks = joint["profile_velocity"]
                if vel_ticks == 0:
                    continue  # 0 = leave hardware default unchanged
                result, dxl_error = self._packet.write4ByteTxRx(
                    self._port, joint["id"], ADDR_PROFILE_VELOCITY, vel_ticks
                )
                if result != self._sdk.COMM_SUCCESS:
                    log.warning(
                        "Failed to set profile_velocity=%d for '%s' (ID %d): %s",
                        vel_ticks, name, joint["id"],
                        self._packet.getTxRxResult(result),
                    )
                elif dxl_error != 0:
                    log.warning(
                        "profile_velocity write servo error for '%s' (ID %d): %s",
                        name, joint["id"],
                        self._packet.getRxPacketError(dxl_error),
                    )
                else:
                    log.info(
                        "Profile velocity set: '%s' (ID %d) = %d ticks/s",
                        name, joint["id"], vel_ticks,
                    )

    def _write_pid_gains(self) -> None:
        """Write per-joint position PID gains (RAM addrs 80/82/84).

        Values of 0 mean "leave hardware default" — no write is issued.
        (servo_node.py lines 665-714)
        """
        with self.port_lock:
            for name, joint in self._joints.items():
                gains = joint["pid"]
                gain_map = [
                    ("P", ADDR_POSITION_P_GAIN, gains.get("p", 0)),
                    ("I", ADDR_POSITION_I_GAIN, gains.get("i", 0)),
                    ("D", ADDR_POSITION_D_GAIN, gains.get("d", 0)),
                ]
                for label, addr, val in gain_map:
                    if val <= 0:
                        continue
                    result, dxl_error = self._packet.write2ByteTxRx(
                        self._port, joint["id"], addr, val
                    )
                    if result != self._sdk.COMM_SUCCESS:
                        log.warning(
                            "Failed to set Position %s Gain=%d for '%s' (ID %d): %s",
                            label, val, name, joint["id"],
                            self._packet.getTxRxResult(result),
                        )
                    elif dxl_error != 0:
                        log.warning(
                            "Position %s Gain write error for '%s' (ID %d): %s",
                            label, name, joint["id"],
                            self._packet.getRxPacketError(dxl_error),
                        )
                    else:
                        log.info(
                            "Position %s Gain set: '%s' (ID %d) = %d",
                            label, name, joint["id"], val,
                        )

    def _build_bulkread(self) -> None:
        """Build a single GroupBulkRead for position + load feedback.

        Reads a contiguous block from Present Load (126) through Present
        Position (132-135) = 10 bytes per servo.  GroupBulkRead only allows
        one addParam per servo ID.  (servo_node.py lines 480-492)
        """
        self._bulkread = self._sdk.GroupBulkRead(self._port, self._packet)
        start = ADDR_PRESENT_LOAD  # 126
        length = (ADDR_PRESENT_POSITION + LEN_PRESENT_POSITION) - ADDR_PRESENT_LOAD  # 10
        for dxl_id in self._id_for.values():
            self._bulkread.addParam(dxl_id, start, length)

    def _reopen_port_and_rebuild_bulkread_locked(self) -> None:
        """Reopen the serial port after consecutive read failures.

        Ported from servo_node.py lines 587-603, EXCEPT it must be called with
        port_lock already held (the vendored version re-acquired the
        non-reentrant lock — a latent self-deadlock).
        """
        try:
            log.warning("Reopening servo port after consecutive read failures...")
            self._port.closePort()
            time.sleep(0.05)
            if not self._port.openPort() or not self._port.setBaudRate(self.baud):
                log.error(
                    "Reopen failed for '%s' at %d bps.", self.port_name, self.baud
                )
                return
            self._build_bulkread()
            self._read_fail_count = 0
            self._backoff_ms = self._backoff_initial
            log.info("Servo port reopened and bulk-read rebuilt successfully.")
        except Exception as e:
            log.error("Exception during servo port reopen: %s", e)

    # ------------------------------------------------------------------
    # Runtime interface
    # ------------------------------------------------------------------

    @property
    def joint_names(self) -> list[str]:
        return list(self._joints.keys())

    def write_targets_rad(self, targets: dict[str, float]) -> None:
        """Clamp → tick → GroupSyncWrite. The single goal-position choke point.

        Ported from servo_node.py _joint_state_cb (lines 882-936): every
        target passes through rad_to_tick (guards 1-3) and one SyncWrite.
        """
        if not self._open:
            raise RuntimeError("ServoBus is not open")
        with self.port_lock:
            gsw = self._sdk.GroupSyncWrite(
                self._port, self._packet, ADDR_GOAL_POSITION, LEN_GOAL_POSITION
            )
            params_added = False
            for name, rad in targets.items():
                joint = self._joints.get(name)
                if joint is None:
                    log.debug("write_targets_rad: unknown joint '%s', skipping.", name)
                    continue
                tick = rad_to_tick(
                    name, float(rad), self._sign, self._limits, joint["offset_rad"]
                )
                gsw.addParam(joint["id"], goal_position_bytes(tick))
                params_added = True
            if params_added:
                result = gsw.txPacket()
                if result != self._sdk.COMM_SUCCESS:
                    log.error(
                        "GroupSyncWrite failed: %s",
                        self._packet.getTxRxResult(result),
                    )

    def read_feedback(self) -> dict[str, JointFeedback]:
        """Bulk-read position + load for all joints.

        On bus failure the last-known values are returned with fresh=False
        (guard 7) and the exponential-backoff / port-reopen path runs
        (guard 8).  Per-joint misses (partial packets, out-of-range ticks)
        also hold last-known values with fresh=False.
        Ported from servo_node.py _fast_position_publish (lines 494-577) and
        publish_servo_metrics failure handling (lines 949-972, 1013-1020).
        """
        if not self._open:
            raise RuntimeError("ServoBus is not open")
        start_addr = ADDR_PRESENT_LOAD  # 126
        block_len = (ADDR_PRESENT_POSITION + LEN_PRESENT_POSITION) - ADDR_PRESENT_LOAD  # 10

        with self.port_lock:
            result = self._bulkread.txRxPacket()
            if result != self._sdk.COMM_SUCCESS:
                self._read_fail_count += 1
                reason = self._packet.getTxRxResult(result)
                if self._read_fail_count < self._max_failures:
                    log.warning(
                        "GroupBulkRead failed (%d/%d): %s — backing off %.0fms",
                        self._read_fail_count, self._max_failures, reason,
                        self._backoff_ms,
                    )
                    # Sleep inside the lock: keeps the bus quiet during
                    # recovery (matches vendored behavior).
                    time.sleep(self._backoff_ms / 1000.0)
                    self._backoff_ms = min(
                        self._backoff_max,
                        max(self._backoff_initial, self._backoff_ms * 2.0),
                    )
                else:
                    log.error(
                        "GroupBulkRead failed repeatedly (%d); reason: %s. "
                        "Attempting port reopen.",
                        self._read_fail_count, reason,
                    )
                    self._read_fail_count = 0
                    self._backoff_ms = self._backoff_initial
                    self._reopen_port_and_rebuild_bulkread_locked()
                # Guard 7: hold last-known values so the motion planner
                # doesn't see feedback jumps after bus glitches.
                for name in self._fb_fresh:
                    self._fb_fresh[name] = False
                return self._snapshot()

            for name, joint in self._joints.items():
                dxl_id = joint["id"]
                fresh = False
                if self._bulkread.isAvailable(dxl_id, start_addr, block_len):
                    # isAvailable() validates the *registered* address range
                    # but not the actual received data length — partial
                    # packets pass isAvailable yet raise IndexError in
                    # getData().
                    try:
                        position = self._bulkread.getData(
                            dxl_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION
                        )
                        # Reject out-of-range ticks (bus corruption)
                        if DXL_MIN_TICK <= position <= DXL_MAX_TICK:
                            self._fb_pos_rad[name] = tick_to_rad(
                                name, position, self._sign
                            )
                            try:
                                raw = self._bulkread.getData(
                                    dxl_id, ADDR_PRESENT_LOAD, LEN_PRESENT_LOAD
                                )
                                self._fb_load[name] = decode_present_load(raw)
                            except (IndexError, KeyError):
                                pass  # keep cached load
                            fresh = True
                    except (IndexError, KeyError):
                        pass  # corrupted/partial packet — hold last for this servo
                self._fb_fresh[name] = fresh

        # Successful transaction — reset backoff state (outside the lock,
        # matching vendored ordering).
        self._read_fail_count = 0
        self._backoff_ms = self._backoff_initial
        return self._snapshot()

    def _snapshot(self) -> dict[str, JointFeedback]:
        return {
            name: JointFeedback(
                position_rad=self._fb_pos_rad[name],
                load=self._fb_load[name],
                fresh=self._fb_fresh[name],
            )
            for name in self._joints
        }
