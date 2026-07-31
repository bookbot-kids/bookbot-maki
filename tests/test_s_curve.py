"""Unit tests for the S-curve planner."""

import math
import pytest

from maki_puppet.motion.s_curve import (
    SCurvePlanner,
    SCurveProfile,
    profile_from_dps,
)


# ── Helpers ───────────────────────────────────────────────────────────

def run_planner(planner: SCurvePlanner, dt: float, max_ticks: int = 10000) -> list[float]:
    """Advance planner until it arrives (or max_ticks). Return position trace."""
    trace = []
    for _ in range(max_ticks):
        pos = planner.update(dt)
        trace.append(pos)
        if planner.arrived:
            break
    return trace


def velocity_trace(trace: list[float], dt: float) -> list[float]:
    """Numerical differentiation of position trace → velocity."""
    return [(trace[i] - trace[i - 1]) / dt for i in range(1, len(trace))]


# ── SCurveProfile ─────────────────────────────────────────────────────

class TestSCurveProfile:
    def test_default_profile_valid(self):
        p = SCurveProfile()
        assert p.max_velocity > 0
        assert p.max_acceleration > 0
        assert p.max_jerk > 0

    def test_zero_velocity_raises(self):
        with pytest.raises(ValueError):
            SCurveProfile(max_velocity=0.0)

    def test_negative_jerk_raises(self):
        with pytest.raises(ValueError):
            SCurveProfile(max_jerk=-1.0)


# ── profile_from_dps ─────────────────────────────────────────────────

class TestProfileFromDps:
    def test_conversion_180dps(self):
        p = profile_from_dps(180.0, 360.0, 1200.0)
        assert abs(p.max_velocity - math.radians(180.0)) < 1e-9
        assert abs(p.max_acceleration - math.radians(360.0)) < 1e-9
        assert abs(p.max_jerk - math.radians(1200.0)) < 1e-9


# ── SCurvePlanner — basic motion ─────────────────────────────────────

class TestSCurvePlannerBasic:
    @pytest.fixture
    def planner(self):
        profile = profile_from_dps(180.0, 360.0, 1200.0)
        return SCurvePlanner(profile=profile)

    def test_starts_at_origin(self, planner):
        assert planner.state.position == 0.0
        assert planner.state.velocity == 0.0

    def test_update_zero_dt_returns_position(self, planner):
        planner.set_target(1.0)
        pos = planner.update(0.0)
        assert pos == 0.0

    def test_arrives_at_target(self, planner):
        target = 1.0  # ~57 degrees
        planner.set_target(target)
        trace = run_planner(planner, dt=0.02)
        assert planner.arrived
        assert abs(trace[-1] - target) < 1e-5

    def test_arrives_at_negative_target(self, planner):
        target = -0.5
        planner.set_target(target)
        trace = run_planner(planner, dt=0.02)
        assert planner.arrived
        assert abs(trace[-1] - target) < 1e-5

    def test_small_move(self, planner):
        target = 0.01  # ~0.6 degrees
        planner.set_target(target)
        trace = run_planner(planner, dt=0.02)
        assert planner.arrived
        assert abs(trace[-1] - target) < 1e-5

    def test_large_move(self, planner):
        target = math.radians(150)  # ~2.62 rad
        planner.set_target(target)
        trace = run_planner(planner, dt=0.02)
        assert planner.arrived
        assert abs(trace[-1] - target) < 1e-5

    def test_already_at_target_is_noop(self, planner):
        planner.set_target(0.0)
        pos = planner.update(0.02)
        assert pos == 0.0


# ── SCurvePlanner — velocity / jerk limits ────────────────────────────

class TestSCurvePlannerLimits:
    @pytest.fixture
    def planner(self):
        profile = profile_from_dps(180.0, 360.0, 1200.0)
        return SCurvePlanner(profile=profile)

    def test_velocity_never_exceeds_max(self, planner):
        planner.set_target(math.radians(180))  # big move
        trace = run_planner(planner, dt=0.02)
        dt = 0.02
        vels = velocity_trace(trace, dt)
        max_vel = max(abs(v) for v in vels) if vels else 0.0
        # Allow small numerical overshoot (< 1%)
        assert max_vel <= planner.profile.max_velocity * 1.01

    def test_monotonic_approach_positive(self, planner):
        """Position should monotonically increase for a positive target
        (no overshoot during approach, before direction reversal is needed)."""
        planner.set_target(1.0)
        trace = run_planner(planner, dt=0.02)
        for i in range(1, len(trace)):
            assert trace[i] >= trace[i - 1] - 1e-9  # allow tiny float error


# ── SCurvePlanner — direction reversal ────────────────────────────────

class TestSCurvePlannerReversal:
    @pytest.fixture
    def planner(self):
        profile = profile_from_dps(180.0, 360.0, 1200.0)
        return SCurvePlanner(profile=profile)

    def test_reversal_arrives_at_new_target(self, planner):
        planner.set_target(1.0)
        # Run for a bit to build up velocity
        for _ in range(15):
            planner.update(0.02)
        # Now reverse
        planner.set_target(-0.5)
        trace = run_planner(planner, dt=0.02)
        assert planner.arrived
        assert abs(trace[-1] - (-0.5)) < 1e-5

    def test_reversal_decelerates_first(self, planner):
        """After reversal request, velocity should eventually decrease
        to zero before going negative.  Due to jerk-limited accel ramp-down
        the velocity may momentarily increase if the starting acceleration
        is positive."""
        planner.set_target(1.0)
        for _ in range(20):
            planner.update(0.02)
        old_vel = planner.state.velocity
        assert old_vel > 0  # should be moving positive

        planner.set_target(-1.0)
        # Run enough ticks for jerk-limited decel to bring velocity to zero.
        # With jerk ~20.9 rad/s³ and starting accel ~2.09 rad/s², it takes
        # ~5 ticks to zero accel, then more to zero velocity.
        reached_lower = False
        for _ in range(200):
            planner.update(0.02)
            if planner.state.velocity <= 0.0:
                reached_lower = True
                break
        assert reached_lower, (
            f"velocity never reached zero after reversal: {planner.state.velocity}"
        )


# ── SCurvePlanner — reset ─────────────────────────────────────────────

class TestSCurvePlannerReset:
    def test_reset_sets_position(self):
        profile = profile_from_dps(180.0, 360.0, 1200.0)
        planner = SCurvePlanner(profile=profile)
        planner.reset(2.0)
        assert planner.state.position == 2.0
        assert planner.state.velocity == 0.0

    def test_reset_clears_arrived(self):
        profile = profile_from_dps(180.0, 360.0, 1200.0)
        planner = SCurvePlanner(profile=profile)
        planner.set_target(0.0)
        planner.update(0.02)
        planner.reset(1.0)
        planner.set_target(2.0)
        assert not planner.arrived


# ── SCurvePlanner — fast profile (eyes) ───────────────────────────────

class TestSCurvePlannerFastProfile:
    def test_eye_profile_arrives(self):
        profile = profile_from_dps(400.0, 1600.0, 8000.0)
        planner = SCurvePlanner(profile=profile)
        planner.set_target(0.3)  # ~17 degrees
        trace = run_planner(planner, dt=0.02)
        assert planner.arrived
        assert abs(trace[-1] - 0.3) < 1e-5

    def test_eye_profile_faster_than_head(self):
        head = SCurvePlanner(profile=profile_from_dps(180.0, 360.0, 1200.0))
        eye = SCurvePlanner(profile=profile_from_dps(400.0, 1600.0, 8000.0))
        target = 0.5
        head.set_target(target)
        eye.set_target(target)

        ht = run_planner(head, dt=0.02)
        et = run_planner(eye, dt=0.02)
        assert len(et) < len(ht)  # eyes arrive faster


# ── SCurvePlanner — settling zone ─────────────────────────────────────

class TestSCurvePlannerSettling:
    """Verify the exponential-decay settling zone produces snap-free arrival."""

    @pytest.fixture
    def planner(self):
        profile = profile_from_dps(150.0, 300.0, 800.0)
        return SCurvePlanner(profile=profile)

    def test_settling_still_arrives(self, planner):
        planner.set_target(0.5)
        trace = run_planner(planner, dt=0.02)
        assert planner.arrived
        assert abs(trace[-1] - 0.5) < 1e-4

    def test_settling_no_velocity_discontinuity(self, planner):
        """Velocity trace should not have sudden jumps >50% of max_velocity
        in a single tick — that would indicate a snapping stop."""
        planner.set_target(0.8)
        trace = run_planner(planner, dt=0.02)
        vels = velocity_trace(trace, 0.02)
        max_vel = planner.profile.max_velocity
        for i in range(1, len(vels)):
            delta = abs(vels[i] - vels[i - 1])
            # Allow max 25% of max_velocity change per tick
            assert delta < max_vel * 0.25, (
                f"Velocity jump of {delta:.4f} rad/s at tick {i} "
                f"(max_vel={max_vel:.3f})"
            )

    def test_settling_disabled_still_works(self):
        """With settling_radius=0, planner should still arrive (old behavior)."""
        profile = SCurveProfile(
            max_velocity=3.14, max_acceleration=6.28, max_jerk=20.94,
            settling_radius=0.0,
        )
        planner = SCurvePlanner(profile=profile)
        planner.set_target(1.0)
        trace = run_planner(planner, dt=0.02)
        assert planner.arrived
        assert abs(trace[-1] - 1.0) < 1e-5

    def test_settling_small_move(self, planner):
        """A move smaller than the settling radius should still converge."""
        planner.set_target(0.003)  # within settling_radius (0.005)
        trace = run_planner(planner, dt=0.02, max_ticks=500)
        assert planner.arrived
        assert abs(trace[-1] - 0.003) < 1e-4


# ── SCurvePlanner — distance-adaptive scaling ─────────────────────────

class TestSCurvePlannerDistanceAdaptive:
    """Verify that short moves use proportionally lower velocity caps."""

    @pytest.fixture
    def planner(self):
        profile = profile_from_dps(
            150.0, 300.0, 800.0,
            short_move_radius=0.15,
            short_move_min_scale=0.15,
        )
        return SCurvePlanner(profile=profile)

    def test_short_move_slower_peak_velocity(self, planner):
        """A move within short_move_radius should have lower peak velocity
        than a move well beyond it."""
        # Short move: 0.05 rad (~2.9°) — within short_move_radius (0.15)
        planner.set_target(0.05)
        short_trace = run_planner(planner, dt=0.02)
        short_vels = velocity_trace(short_trace, 0.02)
        short_peak = max(abs(v) for v in short_vels) if short_vels else 0.0

        # Long move: 1.0 rad (~57°) — well beyond short_move_radius
        planner2 = SCurvePlanner(profile=planner.profile)
        planner2.set_target(1.0)
        long_trace = run_planner(planner2, dt=0.02)
        long_vels = velocity_trace(long_trace, 0.02)
        long_peak = max(abs(v) for v in long_vels) if long_vels else 0.0

        # Short move's peak should be significantly less than long move's
        assert short_peak < long_peak * 0.7, (
            f"Short move peak {short_peak:.4f} not significantly lower "
            f"than long move peak {long_peak:.4f}"
        )

    def test_short_move_still_arrives(self, planner):
        planner.set_target(0.03)
        trace = run_planner(planner, dt=0.02)
        assert planner.arrived
        assert abs(trace[-1] - 0.03) < 1e-4

    def test_disabled_when_radius_zero(self):
        """No distance scaling when short_move_radius=0."""
        profile = profile_from_dps(
            150.0, 300.0, 800.0,
            short_move_radius=0.0,
        )
        planner = SCurvePlanner(profile=profile)
        planner.set_target(0.05)
        trace = run_planner(planner, dt=0.02)
        assert planner.arrived

    def test_long_move_full_speed(self, planner):
        """Moves beyond short_move_radius should hit near-full profile velocity."""
        planner.set_target(1.5)  # well beyond 0.15 radius
        trace = run_planner(planner, dt=0.02)
        vels = velocity_trace(trace, 0.02)
        peak = max(abs(v) for v in vels) if vels else 0.0
        # Should reach at least 80% of max_velocity
        assert peak > planner.profile.max_velocity * 0.8, (
            f"Long move peak {peak:.4f} too low "
            f"(max_vel={planner.profile.max_velocity:.3f})"
        )


# ── SCurvePlanner — external scaling ─────────────────────────────────

class TestSCurvePlannerExternalScaling:
    """Verify that velocity_scale / acceleration_scale affect motion."""

    def test_reduced_velocity_scale_lowers_peak(self):
        profile = profile_from_dps(150.0, 300.0, 800.0, short_move_radius=0.0)
        # Full speed — use a large move so planner hits cruise velocity
        p1 = SCurvePlanner(profile=profile)
        p1.set_target(3.0)
        t1 = run_planner(p1, dt=0.02)
        v1 = velocity_trace(t1, 0.02)
        peak1 = max(abs(v) for v in v1) if v1 else 0.0

        # Half speed
        p2 = SCurvePlanner(profile=profile)
        p2.velocity_scale = 0.5
        p2.acceleration_scale = 0.5
        p2.set_target(3.0)
        t2 = run_planner(p2, dt=0.02)
        v2 = velocity_trace(t2, 0.02)
        peak2 = max(abs(v) for v in v2) if v2 else 0.0

        assert peak2 < peak1 * 0.75, (
            f"Scaled peak {peak2:.4f} not lower than unscaled {peak1:.4f}"
        )

    def test_scaled_still_arrives(self):
        profile = profile_from_dps(150.0, 300.0, 800.0, short_move_radius=0.0)
        p = SCurvePlanner(profile=profile)
        p.velocity_scale = 0.3
        p.acceleration_scale = 0.3
        p.set_target(0.5)
        trace = run_planner(p, dt=0.02, max_ticks=5000)
        assert p.arrived
        assert abs(trace[-1] - 0.5) < 1e-4

    def test_dynamic_scale_change_mid_move(self):
        """Changing scale mid-move should not crash or diverge."""
        profile = profile_from_dps(150.0, 300.0, 800.0, short_move_radius=0.0)
        p = SCurvePlanner(profile=profile)
        p.set_target(1.0)
        # Run at full speed for a bit
        for _ in range(10):
            p.update(0.02)
        # Then reduce scale dramatically (simulates high load detection)
        p.velocity_scale = 0.2
        p.acceleration_scale = 0.2
        trace = run_planner(p, dt=0.02)
        assert p.arrived
        assert abs(trace[-1] - 1.0) < 1e-4
