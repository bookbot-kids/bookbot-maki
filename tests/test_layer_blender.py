"""Unit tests for the layer blender."""



from maki_puppet.motion.layer_blender import LayerBlender, LayerConfig, LayerState


# ── Helpers ───────────────────────────────────────────────────────────

def make_blender(*configs: LayerConfig) -> LayerBlender:
    b = LayerBlender()
    for cfg in configs:
        b.add_layer(cfg)
    return b


# Standard configs
IDLE = LayerConfig(name="idle", default_priority=30,
                   blend_in_time_s=0.5, blend_out_time_s=0.8, claim_timeout_s=5.0)
TRACKING = LayerConfig(name="tracking", default_priority=50,
                       blend_in_time_s=0.3, blend_out_time_s=0.5, claim_timeout_s=2.0)
EXPRESSION = LayerConfig(name="expression", default_priority=40,
                         blend_in_time_s=0.2, blend_out_time_s=0.3, claim_timeout_s=1.5)
SAFETY = LayerConfig(name="safety", default_priority=80,
                     blend_in_time_s=0.0, blend_out_time_s=0.0, claim_timeout_s=5.0)
PASSTHROUGH = LayerConfig(name="passthrough", default_priority=50,
                          blend_in_time_s=0.0, blend_out_time_s=0.5,
                          claim_timeout_s=2.0, merge_targets=True)


# ── LayerConfig ───────────────────────────────────────────────────────

class TestLayerConfig:
    def test_defaults(self):
        c = LayerConfig(name="test")
        assert c.default_priority == 40
        assert c.blend_in_time_s == 0.3
        assert c.claim_timeout_s == 2.0


# ── LayerState ────────────────────────────────────────────────────────

class TestLayerState:
    def test_feed_marks_has_data(self):
        s = LayerState(config=IDLE)
        assert not s._has_data
        s.feed({"head_pan": 0.5}, now=1.0)
        assert s._has_data
        assert s.joint_targets == {"head_pan": 0.5}

    def test_feed_replaces_targets_by_default(self):
        """Without merge_targets, feed() replaces the full dict."""
        s = LayerState(config=IDLE)
        s.feed({"head_pan": 0.5, "eyes_pan": 0.1}, now=1.0)
        s.feed({"head_tilt": 0.3}, now=1.1)
        assert s.joint_targets == {"head_tilt": 0.3}

    def test_feed_merges_targets_when_enabled(self):
        """With merge_targets, feed() merges new joints into existing."""
        s = LayerState(config=PASSTHROUGH)
        s.feed({"head_pan": 0.5, "eyes_pan": 0.1}, now=1.0)
        s.feed({"mouth": 0.3}, now=1.1)
        assert s.joint_targets == {"head_pan": 0.5, "eyes_pan": 0.1, "mouth": 0.3}

    def test_feed_merge_overwrites_existing_joint(self):
        """Merge updates existing joints when a new value arrives."""
        s = LayerState(config=PASSTHROUGH)
        s.feed({"head_pan": 0.5}, now=1.0)
        s.feed({"head_pan": 0.8}, now=1.1)
        assert s.joint_targets["head_pan"] == 0.8

    def test_feed_merge_preserves_priorities(self):
        """Merge accumulates per-joint priorities from different messages."""
        s = LayerState(config=PASSTHROUGH)
        s.feed({"head_pan": 0.5}, {"head_pan": 50}, now=1.0)
        s.feed({"mouth": 0.3}, {"mouth": 40}, now=1.1)
        assert s.priority_for("head_pan") == 50
        assert s.priority_for("mouth") == 40

    def test_priority_for_default(self):
        s = LayerState(config=IDLE)
        assert s.priority_for("head_pan") == 30

    def test_priority_for_override(self):
        s = LayerState(config=IDLE)
        s.feed({"head_pan": 0.5}, {"head_pan": 60}, now=1.0)
        assert s.priority_for("head_pan") == 60

    def test_active_when_has_data_and_weight(self):
        s = LayerState(config=IDLE)
        assert not s.active
        s.feed({"head_pan": 0.5}, now=1.0)
        s.weight = 0.5
        assert s.active


# ── LayerBlender — single layer ───────────────────────────────────────

class TestBlenderSingleLayer:
    def test_single_feed_ramps_in(self):
        b = make_blender(IDLE)
        b.feed_layer("idle", {"head_pan": 1.0}, now=0.0)
        # Run enough ticks to fully blend in (0.5s at 50Hz = 25 ticks)
        for i in range(30):
            result = b.update(dt=0.02, now=0.02 * (i + 1))
        assert "head_pan" in result
        assert abs(result["head_pan"] - 1.0) < 0.05

    def test_instant_blend_in(self):
        s = LayerConfig(name="instant", default_priority=50,
                        blend_in_time_s=0.0, blend_out_time_s=0.0, claim_timeout_s=5.0)
        b = make_blender(s)
        b.feed_layer("instant", {"head_pan": 1.0}, now=0.0)
        result = b.update(dt=0.02, now=0.02)
        assert abs(result.get("head_pan", 0) - 1.0) < 1e-6

    def test_no_data_returns_empty(self):
        b = make_blender(IDLE)
        result = b.update(dt=0.02, now=0.02)
        assert result == {}

    def test_unknown_layer_ignored(self):
        b = make_blender(IDLE)
        b.feed_layer("nonexistent", {"x": 1.0}, now=0.0)
        result = b.update(dt=0.02, now=0.02)
        assert result == {}


# ── LayerBlender — priority ───────────────────────────────────────────

class TestBlenderPriority:
    def test_higher_priority_wins(self):
        b = make_blender(IDLE, TRACKING)
        # Both claim head_pan
        b.feed_layer("idle", {"head_pan": 0.0}, now=0.0)
        b.feed_layer("tracking", {"head_pan": 1.0}, now=0.0)
        # Run enough for full blend-in of both
        for i in range(50):
            result = b.update(dt=0.02, now=0.02 * (i + 1))
        # Tracking (priority 50) should win over idle (30)
        assert abs(result["head_pan"] - 1.0) < 0.05

    def test_different_joints_from_different_layers(self):
        b = make_blender(IDLE, EXPRESSION)
        # Idle claims head_pan, expression claims eyes_pan
        b.feed_layer("idle", {"head_pan": 0.5}, now=0.0)
        b.feed_layer("expression", {"eyes_pan": 0.3}, now=0.0)
        for i in range(50):
            result = b.update(dt=0.02, now=0.02 * (i + 1))
        assert abs(result["head_pan"] - 0.5) < 0.05
        assert abs(result["eyes_pan"] - 0.3) < 0.05


# ── LayerBlender — blend out (claim expiry) ───────────────────────────

class TestBlenderBlendOut:
    def test_layer_fades_out_after_timeout(self):
        b = make_blender(TRACKING)  # timeout=2.0s, blend_out=0.5s
        b.feed_layer("tracking", {"head_pan": 1.0}, now=0.0)
        # Blend in fully
        t = 0.0
        for i in range(50):
            t += 0.02
            b.update(dt=0.02, now=t)
        # Now wait for timeout (2.0s)
        t += 2.5
        # After timeout, blend-out starts (0.5s)
        for i in range(50):
            t += 0.02
            b.update(dt=0.02, now=t)
        # Layer should be fully blended out (empty or zero weight)
        layer = b.layers["tracking"]
        assert layer.weight == 0.0

    def test_re_feed_restarts_blend_in(self):
        b = make_blender(TRACKING)
        b.feed_layer("tracking", {"head_pan": 1.0}, now=0.0)
        t = 0.0
        for i in range(20):
            t += 0.02
            b.update(dt=0.02, now=t)
        # Timeout it
        t += 3.0
        for i in range(40):
            t += 0.02
            b.update(dt=0.02, now=t)
        # Should have blended out
        assert b.layers["tracking"].weight == 0.0
        # Re-feed
        b.feed_layer("tracking", {"head_pan": 0.5}, now=t)
        for i in range(50):
            t += 0.02
            b.update(dt=0.02, now=t)
        assert b.layers["tracking"].weight > 0.9


# ── LayerBlender — fallback positions ─────────────────────────────────

class TestBlenderFallback:
    def test_fallback_used_during_blend_in(self):
        b = make_blender(TRACKING)  # blend_in=0.3s
        b.set_fallback_positions({"head_pan": 0.0})
        b.feed_layer("tracking", {"head_pan": 1.0}, now=0.0)
        # First tick — weight near 0, should interpolate toward fallback
        result = b.update(dt=0.02, now=0.02)
        assert "head_pan" in result
        # Should be between fallback (0.0) and target (1.0), closer to fallback
        assert result["head_pan"] < 0.5


# ── LayerBlender — safety override ────────────────────────────────────

class TestBlenderSafety:
    def test_safety_instant_override(self):
        b = make_blender(TRACKING, SAFETY)
        b.feed_layer("tracking", {"head_pan": 1.0}, now=0.0)
        # Fully blend in tracking
        for i in range(30):
            b.update(dt=0.02, now=0.02 * (i + 1))

        # Now feed safety with instant blend-in
        t = 0.62
        b.feed_layer("safety", {"head_pan": 0.0}, now=t)
        result = b.update(dt=0.02, now=t + 0.02)
        # Safety (pri=80, blend_in=0.0) should immediately dominate
        assert abs(result["head_pan"] - 0.0) < 1e-6


# ── LayerBlender — diagnostics ────────────────────────────────────────

class TestBlenderDiagnostics:
    def test_diagnostics_has_all_layers(self):
        b = make_blender(IDLE, TRACKING)
        diag = b.get_diagnostics(now=1.0)
        assert "idle" in diag
        assert "tracking" in diag
        assert "weight" in diag["idle"]
        assert "has_data" in diag["idle"]

    def test_diagnostics_age_after_feed(self):
        b = make_blender(IDLE)
        b.feed_layer("idle", {"head_pan": 0.5}, now=1.0)
        diag = b.get_diagnostics(now=1.5)
        assert diag["idle"]["age_ms"] == 500


# ── LayerBlender — passthrough merge scenario ─────────────────────────

class TestBlenderPassthroughMerge:
    """Simulate the real passthrough scenario: multiple behavior nodes
    publish different joint subsets to the same shared topic."""

    def test_multiple_publishers_all_joints_survive(self):
        """Simulates gaze_expression + TTS + face_tracker sharing one layer."""
        b = make_blender(PASSTHROUGH)

        # gaze_expression publishes eyes + eyelids
        b.feed_layer("passthrough",
                      {"eyes_pan": 0.1, "eyes_tilt": 0.2,
                       "left_eyelid": 0.0, "right_eyelid": 0.0},
                      now=0.0)
        # TTS publishes mouth only
        b.feed_layer("passthrough",
                      {"mouth": 0.3},
                      {"mouth": 40},
                      now=0.001)
        # face_tracker publishes head + eyes
        b.feed_layer("passthrough",
                      {"head_pan": 0.5, "head_tilt": -0.1,
                       "eyes_pan": 0.15, "eyes_tilt": 0.25},
                      {"head_pan": 50, "head_tilt": 50},
                      now=0.002)

        result = b.update(dt=0.02, now=0.02)
        # ALL joints should be present in output
        assert "head_pan" in result
        assert "head_tilt" in result
        assert "eyes_pan" in result
        assert "eyes_tilt" in result
        assert "left_eyelid" in result
        assert "right_eyelid" in result
        assert "mouth" in result
        # eyes_pan should have the face_tracker value (last writer wins)
        assert abs(result["eyes_pan"] - 0.15) < 0.01
        assert abs(result["mouth"] - 0.3) < 0.01
        assert abs(result["head_pan"] - 0.5) < 0.01

    def test_mouth_survives_after_many_gaze_updates(self):
        """Mouth at 15 Hz should not be erased by 50 Hz gaze_expression."""
        b = make_blender(PASSTHROUGH)

        # TTS sets mouth
        b.feed_layer("passthrough", {"mouth": 0.4}, now=0.0)
        # 50 gaze_expression updates (simulating 50 Hz for 1s)
        t = 0.0
        for i in range(50):
            t += 0.02
            b.feed_layer("passthrough",
                          {"eyes_pan": 0.1, "left_eyelid": 0.0, "right_eyelid": 0.0},
                          now=t)
            result = b.update(dt=0.02, now=t)
        # Mouth should still be present and at the TTS value
        assert "mouth" in result
        assert abs(result["mouth"] - 0.4) < 0.01

    def test_head_survives_between_face_tracker_updates(self):
        """Head at 20 Hz should not be erased by 50 Hz gaze_expression."""
        b = make_blender(PASSTHROUGH)

        # face_tracker sets head
        b.feed_layer("passthrough",
                      {"head_pan": 0.3, "head_tilt": -0.2},
                      now=0.0)
        # Several gaze_expression updates at higher rate
        for i in range(5):
            t = 0.004 * (i + 1)
            b.feed_layer("passthrough",
                          {"eyes_pan": 0.1, "left_eyelid": 0.0},
                          now=t)
        result = b.update(dt=0.02, now=0.02)
        assert abs(result["head_pan"] - 0.3) < 0.01
        assert abs(result["head_tilt"] - (-0.2)) < 0.01


# ── LayerBlender — clear_layer (explicit release) ─────────────────────

class TestBlenderClearLayer:
    """Tests for 3.0.4: explicit layer release via clear_layer()."""

    def test_clear_starts_blend_out(self):
        """clear_layer() immediately starts the blend-out ramp."""
        b = make_blender(TRACKING)
        b.feed_layer("tracking", {"head_pan": 1.0}, now=0.0)
        # Fully blend in
        t = 0.0
        for i in range(30):
            t += 0.02
            b.update(dt=0.02, now=t)
        assert b.layers["tracking"].weight > 0.95

        # Clear the layer
        b.clear_layer("tracking")
        assert b.layers["tracking"]._ramp_direction == -1

    def test_clear_blends_out_over_configured_time(self):
        """After clear, the layer ramps down over blend_out_time_s."""
        b = make_blender(TRACKING)  # blend_out=0.5s
        b.feed_layer("tracking", {"head_pan": 1.0}, now=0.0)
        t = 0.0
        for i in range(30):
            t += 0.02
            b.update(dt=0.02, now=t)
        assert b.layers["tracking"].weight > 0.95

        b.clear_layer("tracking")
        # Run blend-out for 0.5s
        for i in range(30):
            t += 0.02
            b.update(dt=0.02, now=t)
        assert b.layers["tracking"].weight == 0.0
        assert not b.layers["tracking"]._has_data

    def test_clear_faster_than_timeout(self):
        """clear_layer() triggers immediate blend-out, not waiting for timeout."""
        b = make_blender(TRACKING)  # timeout=2.0s
        b.feed_layer("tracking", {"head_pan": 1.0}, now=0.0)
        t = 0.0
        for i in range(30):
            t += 0.02
            b.update(dt=0.02, now=t)
        assert b.layers["tracking"].weight > 0.95

        # Clear at t=0.6 (well before the 2.0s timeout)
        b.clear_layer("tracking")
        # After 0.6s of blend-out (>0.5s configured), should be fully out
        for i in range(35):
            t += 0.02
            b.update(dt=0.02, now=t)
        assert b.layers["tracking"].weight == 0.0

    def test_clear_then_refeed_restarts(self):
        """After clear + full blend-out, a new feed restarts blend-in."""
        b = make_blender(TRACKING)
        b.feed_layer("tracking", {"head_pan": 1.0}, now=0.0)
        t = 0.0
        for i in range(30):
            t += 0.02
            b.update(dt=0.02, now=t)
        b.clear_layer("tracking")
        for i in range(30):
            t += 0.02
            b.update(dt=0.02, now=t)
        assert b.layers["tracking"].weight == 0.0

        # Re-feed
        b.feed_layer("tracking", {"head_pan": 0.5}, now=t)
        for i in range(30):
            t += 0.02
            b.update(dt=0.02, now=t)
        assert b.layers["tracking"].weight > 0.9

    def test_clear_unknown_layer_no_error(self):
        """clear_layer() on non-existent layer is a no-op."""
        b = make_blender(IDLE)
        b.clear_layer("nonexistent")  # should not raise

    def test_clear_idle_layer_no_data(self):
        """clear_layer() on a never-fed layer is a no-op."""
        b = make_blender(IDLE)
        b.clear_layer("idle")  # no data yet — should not raise
        assert b.layers["idle"].weight == 0.0

    def test_clear_during_blend_in(self):
        """clear_layer() during an active blend-in reverses direction."""
        b = make_blender(TRACKING)  # blend_in=0.3s
        b.feed_layer("tracking", {"head_pan": 1.0}, now=0.0)
        # Partially blend in (3 ticks = 0.06s out of 0.3s)
        t = 0.0
        for i in range(3):
            t += 0.02
            b.update(dt=0.02, now=t)
        mid_weight = b.layers["tracking"].weight
        assert 0.0 < mid_weight < 1.0

        # Clear during blend-in
        b.clear_layer("tracking")
        assert b.layers["tracking"]._ramp_direction == -1
        # Continue — weight should decrease
        t += 0.02
        b.update(dt=0.02, now=t)
        assert b.layers["tracking"].weight < mid_weight


# ── LayerBlender — per-joint ownership diagnostics ────────────────────

class TestBlenderJointOwnership:
    """Tests for 3.0.5: per-joint ownership diagnostics."""

    def test_single_layer_ownership(self):
        """Single active layer owns all its joints."""
        b = make_blender(IDLE)
        b.feed_layer("idle", {"head_pan": 0.5, "eyes_pan": 0.2}, now=0.0)
        # Fully blend in
        for i in range(30):
            b.update(dt=0.02, now=0.02 * (i + 1))

        own = b.get_joint_ownership(now=1.0)
        assert "head_pan" in own
        assert own["head_pan"]["owner"] == "idle"
        assert own["head_pan"]["priority"] == 30
        assert own["head_pan"]["transition"] == "stable"
        assert own["head_pan"]["weight"] == 1.0

    def test_priority_determines_owner(self):
        """Higher-priority layer is reported as owner."""
        b = make_blender(IDLE, TRACKING)
        b.feed_layer("idle", {"head_pan": 0.0}, now=0.0)
        b.feed_layer("tracking", {"head_pan": 1.0}, now=0.0)
        for i in range(50):
            b.update(dt=0.02, now=0.02 * (i + 1))

        own = b.get_joint_ownership(now=1.5)
        assert own["head_pan"]["owner"] == "tracking"
        assert own["head_pan"]["priority"] == 50

    def test_per_joint_priority_override(self):
        """effort[] override changes the effective owner for specific joints."""
        b = make_blender(TRACKING, EXPRESSION)
        # Tracking has head at default p50
        b.feed_layer("tracking", {"head_pan": 0.5}, now=0.0)
        # Expression sends head_pan with gesture priority 51
        b.feed_layer("expression", {"head_pan": 0.3}, {"head_pan": 51}, now=0.0)
        for i in range(50):
            b.update(dt=0.02, now=0.02 * (i + 1))

        own = b.get_joint_ownership(now=1.5)
        assert own["head_pan"]["owner"] == "expression"
        assert own["head_pan"]["priority"] == 51

    def test_blending_in_transition(self):
        """A layer mid-blend-in shows transition='blending_in'."""
        b = make_blender(TRACKING)  # blend_in=0.3s
        b.feed_layer("tracking", {"head_pan": 1.0}, now=0.0)
        # 2 ticks — not yet fully in
        b.update(dt=0.02, now=0.02)
        b.update(dt=0.02, now=0.04)
        own = b.get_joint_ownership(now=0.04)
        assert own["head_pan"]["transition"] == "blending_in"

    def test_blending_out_transition(self):
        """After clear_layer, transition shows 'blending_out'."""
        b = make_blender(TRACKING)
        b.feed_layer("tracking", {"head_pan": 1.0}, now=0.0)
        for i in range(30):
            b.update(dt=0.02, now=0.02 * (i + 1))
        b.clear_layer("tracking")
        b.update(dt=0.02, now=1.0)
        own = b.get_joint_ownership(now=1.0)
        assert own["head_pan"]["transition"] == "blending_out"

    def test_no_active_layers_empty(self):
        """No active layers → empty ownership dict."""
        b = make_blender(IDLE)
        own = b.get_joint_ownership(now=1.0)
        assert own == {}

    def test_different_owners_per_joint(self):
        """Different joints can have different owners."""
        b = make_blender(IDLE, TRACKING)
        b.feed_layer("idle", {"head_pan": 0.1, "eyes_pan": 0.2}, now=0.0)
        b.feed_layer("tracking", {"head_pan": 0.5}, now=0.0)
        for i in range(50):
            b.update(dt=0.02, now=0.02 * (i + 1))

        own = b.get_joint_ownership(now=1.5)
        # head_pan: tracking wins (p50 > p30)
        assert own["head_pan"]["owner"] == "tracking"
        # eyes_pan: only idle owns it
        assert own["eyes_pan"]["owner"] == "idle"
