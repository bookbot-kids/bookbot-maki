"""Tests for the simulated hardware drivers (SimServoBus / SimLedRing) and
the pure LED waveform math shared with the real LedRing."""

import math
from pathlib import Path

import pytest
import yaml

from maki_puppet.hw import JointFeedback, SimLedRing, SimServoBus
from maki_puppet.hw.led_ring import Anim, LedRing, blend_rgb, hue_to_rgb, sample_animation
from maki_puppet.hw.servo_bus import JOINT_LIMITS_TICKS, DXL_CENTER_TICK

ANIMATIONS_FILE = Path(__file__).resolve().parents[1] / "config" / "animations.yaml"

ALL_JOINTS = [
    "head_pan", "head_tilt", "eyes_tilt", "eyes_pan",
    "left_eyelid", "right_eyelid", "mouth",
]


def load_animations() -> dict:
    with open(ANIMATIONS_FILE) as f:
        return yaml.safe_load(f)


class FakeClock:
    def __init__(self, t: float = 0.0):
        self.t = t

    def __call__(self) -> float:
        return self.t

    def advance(self, dt: float) -> None:
        self.t += dt


# ----------------------------------------------------------------------
# SimServoBus
# ----------------------------------------------------------------------

def make_bus(**kwargs):
    clock = FakeClock()
    bus = SimServoBus(kwargs.pop("config", None), time_fn=clock, **kwargs)
    bus.open()
    return bus, clock


def test_sim_bus_joint_names():
    bus, _ = make_bus()
    assert bus.joint_names == ALL_JOINTS


def test_sim_bus_initial_feedback_is_neutral_fresh_no_load():
    bus, _ = make_bus()
    fb = bus.read_feedback()
    assert set(fb.keys()) == set(ALL_JOINTS)
    for jf in fb.values():
        assert isinstance(jf, JointFeedback)
        assert jf.position_rad == 0.0
        assert jf.load == 0.0
        assert jf.fresh is True


def test_sim_bus_feedback_slews_toward_target():
    bus, clock = make_bus()
    bus.write_targets_rad({"head_pan": 0.5})
    clock.advance(0.1)  # default slew 2.0 rad/s -> 0.2 rad max step
    pos1 = bus.read_feedback()["head_pan"].position_rad
    assert 0.0 < pos1 < 0.5
    assert pos1 == pytest.approx(0.2)
    clock.advance(0.1)
    pos2 = bus.read_feedback()["head_pan"].position_rad
    assert pos1 < pos2 < 0.5
    clock.advance(10.0)
    pos3 = bus.read_feedback()["head_pan"].position_rad
    assert pos3 == pytest.approx(0.5, abs=1e-3)
    # Settled: no overshoot on further reads
    clock.advance(1.0)
    assert bus.read_feedback()["head_pan"].position_rad == pytest.approx(pos3)


def test_sim_bus_clamps_targets_to_joint_limits():
    bus, clock = make_bus()
    bus.write_targets_rad({"head_tilt": 5.0})  # way beyond mechanical range
    clock.advance(60.0)
    pos = bus.read_feedback()["head_tilt"].position_rad
    lo, hi = JOINT_LIMITS_TICKS["head_tilt"]
    max_rad = (hi - DXL_CENTER_TICK) * math.pi / DXL_CENTER_TICK  # sign +1
    assert pos == pytest.approx(max_rad, abs=1e-6)
    assert pos < 5.0


def test_sim_bus_mirrored_right_eyelid_sign():
    # Same body-centric command produces the same body-centric feedback for
    # both eyelids (the sign flip is hidden below the rad interface).
    bus, clock = make_bus()
    bus.write_targets_rad({"left_eyelid": -0.4, "right_eyelid": -0.4})
    clock.advance(60.0)
    fb = bus.read_feedback()
    assert fb["left_eyelid"].position_rad == pytest.approx(
        fb["right_eyelid"].position_rad, abs=1e-3
    )
    assert fb["left_eyelid"].position_rad == pytest.approx(-0.4, abs=1e-3)


def test_sim_bus_unknown_joint_ignored():
    bus, clock = make_bus()
    bus.write_targets_rad({"antenna": 1.0, "head_pan": 0.1})
    clock.advance(60.0)
    fb = bus.read_feedback()
    assert "antenna" not in fb
    assert fb["head_pan"].position_rad == pytest.approx(0.1, abs=1e-3)


def test_sim_bus_requires_open():
    bus = SimServoBus()
    with pytest.raises(RuntimeError):
        bus.write_targets_rad({"head_pan": 0.0})
    with pytest.raises(RuntimeError):
        bus.read_feedback()


def test_sim_bus_close_idempotent():
    bus, _ = make_bus()
    bus.close()
    bus.close()
    bus.open()  # can reopen
    assert bus.read_feedback()["head_pan"].fresh is True


# ----------------------------------------------------------------------
# SimLedRing
# ----------------------------------------------------------------------

def make_ring(config=None):
    ring = SimLedRing(config or {"default_animation": "breathing_cyan"},
                      load_animations())
    ring.start()
    return ring


def test_sim_ring_default_current():
    ring = make_ring()
    assert ring.current == {"animation": "breathing_cyan"}


def test_sim_ring_set_animation():
    ring = make_ring()
    ring.set_animation("alarm_red")
    assert ring.current == {"animation": "alarm_red"}


def test_sim_ring_unknown_animation_raises_keyerror():
    ring = make_ring()
    with pytest.raises(KeyError):
        ring.set_animation("does_not_exist")
    assert ring.current == {"animation": "breathing_cyan"}  # unchanged


def test_sim_ring_off_redirects_to_default():
    # never-fully-dark guard: "off" maps to the ambient default animation
    ring = make_ring()
    ring.set_animation("alarm_red")
    ring.set_animation("off")
    assert ring.current == {"animation": "breathing_cyan"}


def test_sim_ring_set_color():
    ring = make_ring()
    ring.set_color(10, 300, -5)  # clamped to 0..255
    assert ring.current == {"color": {"r": 10, "g": 255, "b": 0}}


def test_sim_ring_current_returns_copy():
    ring = make_ring()
    ring.set_color(1, 2, 3)
    snap = ring.current
    snap["color"]["r"] = 99
    assert ring.current == {"color": {"r": 1, "g": 2, "b": 3}}


def test_sim_ring_stop_idempotent():
    ring = make_ring()
    ring.stop()
    ring.stop()
    ring.start()
    assert "animation" in ring.current


# ----------------------------------------------------------------------
# Real LedRing: hardware-free surface (constructor + catalog validation)
# ----------------------------------------------------------------------

def test_led_ring_constructs_and_validates_without_hardware():
    ring = LedRing({"pixel_count": 48}, load_animations())
    with pytest.raises(KeyError):
        ring.set_animation("nope")
    ring.set_animation("breathing_cyan")  # no hardware touched before start()
    assert ring.current == {"animation": "breathing_cyan"}
    ring.stop()  # idempotent even when never started


def test_led_ring_accepts_unwrapped_animations_dict():
    inner = load_animations()["animations"]
    ring = LedRing({}, inner)
    ring.set_animation("alarm_red")
    assert ring.current == {"animation": "alarm_red"}


# ----------------------------------------------------------------------
# Pure waveform math (shared by LedRing; guards the no-strobe requirement)
# ----------------------------------------------------------------------

def sample_over_period(name: str, period: float = 2.0, steps: int = 200):
    anim = Anim(name, (100, 50, 25), period)
    return [
        sample_animation(anim, anim.start + period * i / steps)
        for i in range(steps + 1)
    ]


def test_breathing_waveform_is_16_to_60_percent():
    scales = [s for _, s in sample_over_period("breathing")]
    assert min(scales) == pytest.approx(0.16, abs=1e-9)
    assert max(scales) == pytest.approx(0.60, abs=1e-3)


def test_pulse_waveform_bounds():
    scales = [s for _, s in sample_over_period("pulse")]
    assert min(scales) == pytest.approx(0.20, abs=1e-9)
    assert max(scales) == pytest.approx(0.75, abs=1e-3)


def test_flash_decays_exponentially_to_floor_never_dark():
    samples = sample_over_period("flash")
    scales = [s for _, s in samples]
    # Peak at cycle start, decaying monotonically within the cycle
    assert scales[0] == pytest.approx(0.70, abs=1e-9)
    assert all(a >= b for a, b in zip(scales[:-1], scales[1:-1]))
    # No photosensitive strobe: never below the 0.15 floor
    assert all(s >= 0.15 for s in scales)


def test_static_returns_full_scale_rgb():
    anim = Anim("static", (1, 2, 3), 1.0)
    assert sample_animation(anim, anim.start + 0.37) == ((1, 2, 3), 1.0)


def test_rainbow_cycles_hue_at_fixed_scale():
    samples = sample_over_period("rainbow", period=6.0, steps=6)
    assert all(s == 0.45 for _, s in samples)
    rgbs = {rgb for rgb, _ in samples}
    assert len(rgbs) > 1  # hue actually moves


def test_hue_to_rgb_primaries():
    assert hue_to_rgb(0.0) == (255, 0, 0)
    assert hue_to_rgb(1.0 / 3.0) == (0, 255, 0)
    assert hue_to_rgb(2.0 / 3.0) == (0, 0, 255)


def test_blend_rgb_endpoints_and_midpoint():
    assert blend_rgb((0, 0, 0), (255, 100, 50), 0.0) == (0, 0, 0)
    assert blend_rgb((0, 0, 0), (255, 100, 50), 1.0) == (255, 100, 50)
    assert blend_rgb((0, 0, 0), (200, 100, 50), 0.5) == (100, 50, 25)


# ── LedRing SPI write deduplication ────────────────────────────────────────


class _FakePixels:
    """Stand-in for NeoPixel_SPI that counts frames clocked to the strip."""

    def __init__(self, n=48):
        self.brightness = 0.0
        self.shows = 0
        self._n = n
        self._px = [(0, 0, 0)] * n

    def fill(self, rgb):
        self._px = [tuple(rgb)] * self._n

    def show(self):
        self.shows += 1

    def __setitem__(self, i, rgb):
        self._px[i] = tuple(rgb)

    def __len__(self):
        return self._n


def _ring_with_fake_pixels():
    from maki_puppet.hw.led_ring import LedRing

    ring = LedRing({"pixel_count": 48}, {"animations": {}})
    ring.pixels = _FakePixels()
    return ring


def test_static_frame_is_written_once_not_every_tick():
    """A still ring must not re-clock identical SPI frames.

    Each redundant frame is a chance for a NeoPixel-over-SPI timing glitch to
    latch a wrong colour — the stray red/green flash this guards against.
    """
    ring = _ring_with_fake_pixels()
    for _ in range(50):
        ring._fill_and_show((0, 80, 200), 1.0)
    assert ring.pixels.shows == 1


def test_changed_colour_or_brightness_still_writes():
    ring = _ring_with_fake_pixels()
    ring._fill_and_show((0, 80, 200), 1.0)
    ring._fill_and_show((200, 0, 0), 1.0)      # colour changed
    ring._fill_and_show((200, 0, 0), 0.5)      # brightness changed
    assert ring.pixels.shows == 3


def test_force_bypasses_the_cache():
    """Clear-on-shutdown must land even if it matches the cached frame."""
    ring = _ring_with_fake_pixels()
    ring._fill_and_show((0, 0, 0), 1.0)
    ring._fill_and_show((0, 0, 0), 1.0, force=True)
    assert ring.pixels.shows == 2


def test_chase_rainbow_invalidates_the_cache():
    """Per-pixel writes leave the strip unlike any cached whole-ring frame."""
    from maki_puppet.hw.led_ring import Anim

    ring = _ring_with_fake_pixels()
    ring._fill_and_show((0, 80, 200), 1.0)
    before = ring.pixels.shows
    ring._render_chase_rainbow(Anim("chase_rainbow", (0, 0, 0), 1.0), 0.0)
    ring._fill_and_show((0, 80, 200), 1.0)     # same colour as before the chase
    assert ring.pixels.shows > before + 1, "static repaint was wrongly skipped"
