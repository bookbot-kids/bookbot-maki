"""Unit tests for MotionService against a local fake servo bus.

The fake implements the pinned ServoBus interface (joint_names,
read_feedback, write_targets_rad, open/close) with zero hardware deps, so
these tests do not depend on maki_puppet.hw existing.
"""

import time
from dataclasses import dataclass

import pytest

from maki_puppet.motion import joints as joints_mod
from maki_puppet.motion.service import MotionService


# ── Fake bus (pinned ServoBus interface) ──────────────────────────────


@dataclass
class FakeFeedback:
    position_rad: float
    load: float
    fresh: bool = True


class FakeBus:
    """Ideal servo bus: feedback tracks the last written targets instantly."""

    def __init__(self, joints=("head_pan", "head_tilt"), initial=0.0, load=0.0):
        self._joints = list(joints)
        self._pos = {j: float(initial) for j in self._joints}
        self.load = float(load)
        self.writes: list[dict] = []

    @property
    def joint_names(self) -> list[str]:
        return list(self._joints)

    def open(self) -> None:
        pass

    def close(self) -> None:
        pass

    def write_targets_rad(self, targets: dict) -> None:
        self.writes.append(dict(targets))
        self._pos.update(targets)

    def read_feedback(self) -> dict:
        return {
            j: FakeFeedback(position_rad=self._pos[j], load=self.load)
            for j in self._joints
        }


# ── Config / driver helpers ───────────────────────────────────────────


def make_config(**overrides):
    cfg = {
        "rate_hz": 50,
        "load_ema_alpha": 0.40,
        "layers": {
            "idle":       {"priority": 30, "blend_in_time_s": 0.2, "blend_out_time_s": 0.3, "claim_timeout_s": 5.0},
            "expression": {"priority": 40, "blend_in_time_s": 0.2, "blend_out_time_s": 0.3, "claim_timeout_s": 1.5, "merge_targets": True},
            "tracking":   {"priority": 50, "blend_in_time_s": 0.3, "blend_out_time_s": 0.5, "claim_timeout_s": 1.0},
            "gesture":    {"priority": 55, "blend_in_time_s": 0.15, "blend_out_time_s": 0.2, "claim_timeout_s": 1.0},
            "safety":     {"priority": 100, "blend_in_time_s": 0.0, "blend_out_time_s": 0.0, "claim_timeout_s": 5.0},
        },
        "profiles": {
            "head_pan": {
                "planning_enabled": True,
                "max_velocity_dps": 140.0, "max_acceleration_dps2": 260.0,
                "max_jerk_dps3": 500.0, "deadband_rad": 0.025,
                "target_smoothing_alpha": 0.0, "settling_radius": 0.04,
                "settling_stiffness": 25.0, "short_move_radius": 0.15,
                "short_move_min_scale": 0.15,
                "load_dampening_threshold": 0.15, "load_dampening_floor": 0.12,
            },
            "head_tilt": {
                "planning_enabled": True,
                "max_velocity_dps": 118.0, "max_acceleration_dps2": 230.0,
                "max_jerk_dps3": 400.0, "deadband_rad": 0.025,
                "target_smoothing_alpha": 0.0, "settling_radius": 0.04,
                "settling_stiffness": 25.0, "short_move_radius": 0.15,
                "short_move_min_scale": 0.15,
                "load_dampening_threshold": 0.12, "load_dampening_floor": 0.12,
            },
        },
        # Deterministic tests: no additive breathing sine
        "idle_breathing": {"enabled": False},
    }
    cfg.update(overrides)
    return cfg


class Driver:
    """Drives _tick_once with a synthetic monotonic clock (dt = 20 ms)."""

    def __init__(self, svc: MotionService):
        self.svc = svc
        self.t = 0.0
        svc._last_tick_time = 0.0

    def run(self, n_ticks: int, dt: float = 0.02, refeed=()):
        """refeed: iterable of (layer, targets, every_n_ticks)."""
        for i in range(n_ticks):
            for layer, targets, every in refeed:
                if i % every == 0:
                    self.svc.set_layer(layer, targets)
            self.t += dt
            self.svc._tick_once(self.t)


# ── Lifecycle ─────────────────────────────────────────────────────────


def test_start_stop_thread_produces_writes():
    bus = FakeBus()
    svc = MotionService(bus, make_config())
    svc.set_layer("idle", {"head_pan": 0.3})
    svc.start()
    try:
        time.sleep(0.25)
    finally:
        svc.stop()
    assert len(bus.writes) >= 3  # ~50 Hz for 0.25 s
    count = len(bus.writes)
    time.sleep(0.1)
    assert len(bus.writes) == count  # thread actually stopped
    # stop/start are idempotent
    svc.stop()
    svc.start()
    svc.stop()


def test_no_writes_without_any_layer_claim():
    bus = FakeBus()
    svc = MotionService(bus, make_config())
    Driver(svc).run(20)
    assert bus.writes == []


# ── set_layer / convergence ───────────────────────────────────────────


def test_set_layer_produces_converging_writes():
    bus = FakeBus()
    svc = MotionService(bus, make_config())
    d = Driver(svc)
    d.run(150, refeed=[("gesture", {"head_pan": 0.5}, 20)])

    assert bus.writes, "expected servo writes"
    final = bus.writes[-1]["head_pan"]
    # Converges to within one deadband width (0.025 rad) of the target —
    # the deadband intentionally suppresses the final sub-threshold delta.
    assert final == pytest.approx(0.5, abs=0.03)
    # S-curve smoothing: some intermediate write is strictly between the
    # endpoints (no single-tick jump to target).
    mids = [w["head_pan"] for w in bus.writes if 0.05 < w["head_pan"] < 0.45]
    assert mids, "expected smooth intermediate positions"
    # pose_rad reflects the fake bus feedback (tracks last write)
    assert svc.pose_rad()["head_pan"] == pytest.approx(final, abs=1e-9)


def test_set_layer_unknown_layer_raises():
    svc = MotionService(FakeBus(), make_config())
    with pytest.raises(KeyError):
        svc.set_layer("nope", {"head_pan": 0.1})
    with pytest.raises(KeyError):
        svc.release_layer("nope")


def test_set_layer_ignores_unknown_joints():
    bus = FakeBus()
    svc = MotionService(bus, make_config())
    d = Driver(svc)
    svc.set_layer("gesture", {"head_pan": 0.2, "elbow": 1.0})
    d.run(50, refeed=[("gesture", {"head_pan": 0.2, "elbow": 1.0}, 20)])
    assert all("elbow" not in w for w in bus.writes)
    assert bus.writes[-1]["head_pan"] == pytest.approx(0.2, abs=0.03)


def test_targets_clamped_to_safe_range():
    bus = FakeBus()
    svc = MotionService(bus, make_config())
    d = Driver(svc)
    # head_tilt safe range is [-0.3804, +0.3866] rad — command far beyond it
    d.run(200, refeed=[("gesture", {"head_tilt": 5.0}, 20)])
    top = max(w["head_tilt"] for w in bus.writes)
    assert top <= 0.3867
    assert bus.writes[-1]["head_tilt"] == pytest.approx(0.3866, abs=0.04)


# ── Priority arbitration ──────────────────────────────────────────────


def test_higher_priority_layer_wins():
    bus = FakeBus()
    svc = MotionService(bus, make_config())
    d = Driver(svc)
    d.run(150, refeed=[("idle", {"head_pan": -0.5}, 20)])
    assert bus.writes[-1]["head_pan"] == pytest.approx(-0.5, abs=0.03)

    # gesture (55) preempts idle (30) while both stay claimed
    d.run(200, refeed=[
        ("idle", {"head_pan": -0.5}, 20),
        ("gesture", {"head_pan": 0.5}, 20),
    ])
    assert bus.writes[-1]["head_pan"] == pytest.approx(0.5, abs=0.03)


def test_release_layer_decays_claim_back_to_lower_priority():
    bus = FakeBus()
    svc = MotionService(bus, make_config())
    d = Driver(svc)
    d.run(200, refeed=[
        ("idle", {"head_pan": -0.5}, 20),
        ("gesture", {"head_pan": 0.5}, 20),
    ])
    assert bus.writes[-1]["head_pan"] == pytest.approx(0.5, abs=0.03)

    svc.release_layer("gesture")
    d.run(200, refeed=[("idle", {"head_pan": -0.5}, 20)])
    assert bus.writes[-1]["head_pan"] == pytest.approx(-0.5, abs=0.03)


def test_claim_expires_without_refeed_and_writes_stop():
    bus = FakeBus()
    svc = MotionService(bus, make_config())
    d = Driver(svc)
    svc.set_layer("gesture", {"head_pan": 0.3})
    # gesture claim_timeout 1.0 s + blend_out 0.2 s << 3 s
    d.run(150)
    count = len(bus.writes)
    d.run(20)
    assert len(bus.writes) == count  # fully faded out — nothing to write


# ── E-stop ────────────────────────────────────────────────────────────


def test_estop_freezes_and_resume_continues():
    bus = FakeBus()
    svc = MotionService(bus, make_config())
    d = Driver(svc)
    # Long move, interrupt mid-flight
    d.run(15, refeed=[("gesture", {"head_pan": 1.0}, 10)])
    assert not svc.estopped

    svc.estop(True)
    assert svc.estopped
    frozen = bus.writes[-1]["head_pan"]
    assert frozen < 0.9  # genuinely mid-motion

    d.run(25, refeed=[("gesture", {"head_pan": 1.0}, 10)])
    held = [w["head_pan"] for w in bus.writes[-25:]]
    assert all(h == pytest.approx(frozen, abs=1e-12) for h in held)

    svc.estop(False)
    assert not svc.estopped
    d.run(200, refeed=[("gesture", {"head_pan": 1.0}, 20)])
    assert bus.writes[-1]["head_pan"] == pytest.approx(1.0, abs=0.04)


def test_estop_engage_is_idempotent():
    bus = FakeBus()
    svc = MotionService(bus, make_config())
    d = Driver(svc)
    d.run(10, refeed=[("gesture", {"head_pan": 0.5}, 5)])
    svc.estop(True)
    frozen = bus.writes[-1]["head_pan"]
    svc.estop(True)  # second engage must not re-freeze at a new position
    d.run(10)
    assert bus.writes[-1]["head_pan"] == pytest.approx(frozen, abs=1e-12)


# ── Load-reactive dampening ───────────────────────────────────────────


def test_load_dampening_scales_planner_velocity():
    cfg = make_config()
    bus_free = FakeBus(load=0.0)
    bus_loaded = FakeBus(load=0.9)
    svc_free = MotionService(bus_free, cfg)
    svc_loaded = MotionService(bus_loaded, cfg)

    refeed = [("gesture", {"head_pan": 1.0}, 20)]
    Driver(svc_free).run(40, refeed=refeed)
    Driver(svc_loaded).run(40, refeed=refeed)

    # The load-dampen wiring must reach the planner scale factors...
    assert svc_free._planners["head_pan"].velocity_scale == pytest.approx(1.0)
    assert svc_loaded._planners["head_pan"].velocity_scale < 0.35
    assert svc_loaded._planners["head_pan"].acceleration_scale < 0.35

    # ...and visibly slow the motion under load.
    progress_free = bus_free.writes[-1]["head_pan"]
    progress_loaded = bus_loaded.writes[-1]["head_pan"]
    assert progress_loaded < progress_free
    assert (progress_free - progress_loaded) > 0.03


def test_load_below_threshold_leaves_full_speed():
    bus = FakeBus(load=0.10)  # below head_pan threshold 0.15
    svc = MotionService(bus, make_config())
    Driver(svc).run(40, refeed=[("gesture", {"head_pan": 1.0}, 20)])
    assert svc._planners["head_pan"].velocity_scale == pytest.approx(1.0)


# ── Feedback seeding ──────────────────────────────────────────────────


def test_planners_seed_from_feedback_not_zero():
    # Servo boots away from zero; the first commands must plan from there.
    bus = FakeBus(initial=0.8)
    svc = MotionService(bus, make_config())
    d = Driver(svc)
    d.run(5, refeed=[("gesture", {"head_pan": 0.9}, 4)])
    first = bus.writes[0]["head_pan"]
    assert first == pytest.approx(0.8, abs=0.05)  # no jump from 0.0


def test_pose_rad_snapshot():
    bus = FakeBus(initial=0.25)
    svc = MotionService(bus, make_config())
    Driver(svc).run(2)
    pose = svc.pose_rad()
    assert pose["head_pan"] == pytest.approx(0.25)
    assert pose["head_tilt"] == pytest.approx(0.25)


# ── Idle micro-breathing ──────────────────────────────────────────────


def test_breathing_offsets_settled_joint_only():
    cfg = make_config(idle_breathing={
        "enabled": True,
        "amplitude_deg": 2.0,   # exaggerated so the sine is visible
        "frequency_hz": 1.0,
        "phase_offset_s": 0.0,
        "joints": ["head_tilt"],
    })
    bus = FakeBus()
    svc = MotionService(bus, cfg)
    d = Driver(svc)
    # Converge to a settled hold, then watch the output wiggle around it.
    # The hold center may sit up to one deadband width (0.025 rad) short of
    # the commanded 0.1, so measure the sine around the observed center.
    d.run(300, refeed=[("idle", {"head_tilt": 0.1}, 20)])
    settled = [w["head_tilt"] for w in bus.writes[-100:]]
    center = sum(settled) / len(settled)
    assert center == pytest.approx(0.1, abs=0.03)
    amp = 0.0349  # 2 degrees in rad
    assert max(settled) > center + 0.5 * amp
    assert min(settled) < center - 0.5 * amp
    # Amplitude bounded by the configured 2 degrees
    assert max(settled) < center + 1.1 * amp
    assert min(settled) > center - 1.1 * amp


# ── Sustained posture ─────────────────────────────────────────────────


def test_posture_holds_indefinitely_without_refeed():
    """The whole point of the posture layer: unlike every other layer, its
    claim must never expire, so a held pose does not drift away on its own."""
    bus = FakeBus()
    svc = MotionService(bus, make_config())
    d = Driver(svc)
    svc.set_posture({"head_tilt": 0.3})
    # Far longer than any claim_timeout in the config (max 5 s).
    d.run(600)
    assert bus.writes[-1]["head_tilt"] == pytest.approx(0.3, abs=0.03)


def test_posture_outlasts_an_expiring_gesture_and_head_returns_to_it():
    """A gesture plays over the posture, then settles back down to it rather
    than to neutral — the 'head stays down while the book is open' case."""
    bus = FakeBus()
    svc = MotionService(bus, make_config())
    d = Driver(svc)
    svc.set_posture({"head_tilt": 0.3})
    d.run(100)
    assert bus.writes[-1]["head_tilt"] == pytest.approx(0.3, abs=0.03)

    # Gesture (priority 55) beats posture (45) while it is being fed.
    d.run(100, refeed=[("gesture", {"head_tilt": -0.3}, 10)])
    assert bus.writes[-1]["head_tilt"] == pytest.approx(-0.3, abs=0.03)

    # Stop feeding it: the claim expires and we fall back to the posture.
    d.run(300)
    assert bus.writes[-1]["head_tilt"] == pytest.approx(0.3, abs=0.03)


def test_posture_beats_idle_but_loses_to_gesture():
    bus = FakeBus()
    svc = MotionService(bus, make_config())
    d = Driver(svc)
    svc.set_posture({"head_tilt": 0.3})
    d.run(200, refeed=[("idle", {"head_tilt": -0.25}, 20)])
    assert bus.writes[-1]["head_tilt"] == pytest.approx(0.3, abs=0.03)


def test_clear_posture_releases_back_to_idle():
    bus = FakeBus()
    svc = MotionService(bus, make_config())
    d = Driver(svc)
    svc.set_posture({"head_tilt": 0.3})
    d.run(150, refeed=[("idle", {"head_tilt": -0.25}, 20)])
    assert bus.writes[-1]["head_tilt"] == pytest.approx(0.3, abs=0.03)

    svc.clear_posture()
    d.run(300, refeed=[("idle", {"head_tilt": -0.25}, 20)])
    assert bus.writes[-1]["head_tilt"] == pytest.approx(-0.25, abs=0.03)


def test_set_posture_clamps_and_ignores_unknown_joints():
    bus = FakeBus()
    svc = MotionService(bus, make_config())
    d = Driver(svc)
    svc.set_posture({"head_tilt": 99.0, "not_a_joint": 0.5})
    d.run(200)
    lo, hi = joints_mod.JOINTS["head_tilt"].rad_min, joints_mod.JOINTS["head_tilt"].rad_max
    assert lo <= bus.writes[-1]["head_tilt"] <= hi
    assert "not_a_joint" not in bus.writes[-1]
