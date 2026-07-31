"""Unit tests for the deadband filter."""


from maki_puppet.motion.deadband import DeadbandFilter


class TestDeadbandFilter:
    def test_first_target_always_passes(self):
        f = DeadbandFilter(deadband_rad=0.025)
        result = f.update(1.0)
        assert result == 1.0
        assert f.committed == 1.0

    def test_within_deadband_is_suppressed(self):
        f = DeadbandFilter(deadband_rad=0.025)
        f.update(1.0)
        result = f.update(1.01)  # 0.01 < 0.025
        assert result is None
        assert f.committed == 1.0

    def test_exceeds_deadband_passes(self):
        f = DeadbandFilter(deadband_rad=0.025)
        f.update(1.0)
        result = f.update(1.03)  # 0.03 > 0.025
        assert result == 1.03
        assert f.committed == 1.03

    def test_just_above_boundary_passes(self):
        f = DeadbandFilter(deadband_rad=0.025)
        f.update(1.0)
        # NOTE: 1.025 - 1.0 = 0.02499... in float64 (< 0.025), so use 1.026
        result = f.update(1.026)  # just above threshold
        assert result == 1.026

    def test_negative_direction(self):
        f = DeadbandFilter(deadband_rad=0.025)
        f.update(1.0)
        result = f.update(0.97)  # -0.03 > 0.025
        assert result == 0.97

    def test_deadband_recenters_on_commit(self):
        f = DeadbandFilter(deadband_rad=0.025)
        f.update(0.0)
        f.update(0.03)   # passes — recenters at 0.03
        result = f.update(0.04)  # 0.01 < 0.025 from new center
        assert result is None

    def test_zero_deadband_passes_everything(self):
        f = DeadbandFilter(deadband_rad=0.0)
        f.update(1.0)
        result = f.update(1.001)
        assert result == 1.001

    def test_reset_clears_committed(self):
        f = DeadbandFilter(deadband_rad=0.025)
        f.update(1.0)
        f.reset()
        assert f.committed is None
        result = f.update(1.0)
        assert result == 1.0  # first target after reset

    def test_reset_with_position(self):
        f = DeadbandFilter(deadband_rad=0.025)
        f.reset(2.0)
        assert f.committed == 2.0
        result = f.update(2.01)  # within deadband of 2.0
        assert result is None

    def test_many_small_steps_suppressed(self):
        f = DeadbandFilter(deadband_rad=0.025)
        f.update(0.0)
        # 10 tiny steps of 0.002 each → all suppressed
        for i in range(1, 11):
            result = f.update(i * 0.002)
            assert result is None

    def test_large_step_after_suppression(self):
        f = DeadbandFilter(deadband_rad=0.025)
        f.update(0.0)
        f.update(0.01)   # suppressed
        f.update(0.02)   # suppressed
        result = f.update(0.05)  # 0.05 > 0.025 → passes
        assert result == 0.05


class TestDeadbandHysteresis:
    """Tests for the release_factor hysteresis behavior."""

    def test_hysteresis_prevents_boundary_oscillation(self):
        """After a commit, jitter back across the *normal* boundary should
        be suppressed — the widened threshold is in effect."""
        f = DeadbandFilter(deadband_rad=0.025, release_factor=0.5)
        f.update(0.0)
        # Move past deadband → commit at 0.03
        r = f.update(0.03)
        assert r == 0.03
        # Jitter back to 0.004 (delta=0.026 from 0.03)
        # Normal deadband: 0.026 >= 0.025 → would pass
        # Widened deadband: 0.025 * 1.5 = 0.0375, 0.026 < 0.0375 → suppressed
        r = f.update(0.004)
        assert r is None

    def test_hysteresis_allows_large_move_after_commit(self):
        """A large move that exceeds the widened threshold should commit."""
        f = DeadbandFilter(deadband_rad=0.025, release_factor=0.5)
        f.update(0.0)
        f.update(0.03)  # commit, widened threshold = 0.0375
        r = f.update(0.07)  # delta=0.04 > 0.0375 → passes
        assert r == 0.07

    def test_hysteresis_reverts_after_suppression(self):
        """After one suppression, threshold reverts to normal."""
        f = DeadbandFilter(deadband_rad=0.025, release_factor=0.5)
        f.update(0.0)
        f.update(0.03)  # commit, _widened=True
        f.update(0.035)  # suppressed (0.005 < 0.0375), _widened=False
        # Now normal deadband is back — 0.03 from 0.03 → exceeds 0.025
        r = f.update(0.06)
        assert r == 0.06

    def test_hysteresis_zero_factor_is_no_hysteresis(self):
        """release_factor=0.0 means widened threshold = 1.0x = same as normal."""
        f = DeadbandFilter(deadband_rad=0.025, release_factor=0.0)
        f.update(0.0)
        f.update(0.03)  # commit, widened threshold = 0.025 * 1.0 = 0.025
        # delta=0.026 >= 0.025 → passes (no extra hysteresis)
        r = f.update(0.004)
        assert r == 0.004
