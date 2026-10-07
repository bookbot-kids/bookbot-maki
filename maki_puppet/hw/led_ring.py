"""NeoPixel LED ring driver for MAKI puppet mode.

Forked from maki-rpi5-v0.2.13-rc.4/src/maki_led/maki_led/led_ring_node.py; guards preserved — see plan checklist.

Preserved behavior:
  - 50 Hz animation daemon thread with a pixel threading.Lock guarding all
    pixel writes.
  - Animation types: static / pulse / breathing / flash / rainbow /
    chase_rainbow with the exact vendored waveform math (breathing sinusoid
    16-60%, flash exponential decay to a 0.15 floor — never a photosensitive
    strobe, chase_rainbow spatial per-pixel hue).
  - GRB pixel order, 48 pixels (2x24 daisy-chained rings) over hardware SPI.
  - Brightness floor (animation_brightness_floor * max_brightness) so LEDs
    never appear fully off; "off" animation requests redirect to the ambient
    default animation.
  - Crossfade between animations (animation_transition_s), with the
    chase_rainbow snap exception (spatial patterns have no single-RGB sample
    to crossfade from).
  - Optional min-dwell deferral of animation changes (default 0 s in puppet
    mode; the vendored node used 2 s).

New vs vendored: advisory flock on the LED lock file (fail-fast if a second
process tries to drive the same SPI strip).

Dropped vs vendored (ROS companion features with no puppet source):
engagement-driven brightness lerp, affect valence/arousal tint, lip-sync
parameters.

``board``/``neopixel_spi`` are imported inside ``start()`` so this module
imports cleanly on machines without Blinka (``--sim``).
"""

from __future__ import annotations

import colorsys
import logging
import math
import os
import threading
import time
from typing import Any, Optional

log = logging.getLogger(__name__)

# Temporary diagnostic switch (MAKI_LED_TRACE=1) — see _trace_near_white.
_LED_TRACE = bool(os.environ.get("MAKI_LED_TRACE"))

VALID_PIXEL_ORDERS = {"GRB", "RGB", "GRBW", "RGBW"}

DEFAULT_LOCK_FILE = "/tmp/maki_led_spi.lock"
DEFAULT_PIXEL_COUNT = 48          # 2x24 daisy-chained rings
DEFAULT_PIXEL_ORDER = "GRB"
DEFAULT_BRIGHTNESS = 0.05
DEFAULT_MAX_BRIGHTNESS = 0.10
DEFAULT_ANIMATION_BRIGHTNESS_CAP = 0.9
DEFAULT_ANIMATION_BRIGHTNESS_FLOOR = 0.15
DEFAULT_ANIMATION_TRANSITION_S = 0.25
DEFAULT_ANIMATION_MIN_DWELL_S = 0.0   # vendored node used 2.0
DEFAULT_ANIMATION = "breathing_cyan"


class Anim:
    """Container for a running animation (vendored ``_Anim``)."""

    def __init__(self, name: str, rgb: tuple, period: float,
                 brightness_scale: float = 1.0):
        self.name = name
        self.rgb = tuple(rgb)
        self.period = max(0.1, float(period))
        # Multiplier on the ring's base brightness for this animation only
        # (animations.yaml `brightness_scale`). Still clamped to
        # max_brightness, so it can lift one signal above the rest without
        # ever exceeding the ring's hard cap.
        self.brightness_scale = max(0.0, float(brightness_scale))
        self.start = time.time()


def hue_to_rgb(h: float) -> tuple[int, int, int]:
    """Vendored ``_hue_to_rgb`` (led_ring_node.py lines 587-604).

    Kept byte-identical to the ROS stack's wheel: one channel held at full
    while a second ramps, so the secondaries (yellow, cyan, magenta) put out
    r+g+b = 510 against 255 at the primaries. That uneven output was suspected
    of causing the rainbow's white flashes, but the ROS stack ran this exact
    wheel and looked clean — the difference was brightness (0.03) and speed
    (2.5 s per revolution), not the hue maths.

    The one deviation is the ``h % 1.0`` wrap. Vendored, h=1.0 gives
    int(6) % 6 = 0 with f = 6, i.e. g = 1530 — out of range and byte-truncated
    to a near-white pixel. It is unreachable from both callers (each already
    takes % 1.0), so the wrap changes no observable behaviour; it just stops
    the bug being reachable at all.
    """
    h = h % 1.0
    i = int(h * 6) % 6
    f = h * 6 - i
    q = 1 - f
    if i == 0:
        r, g, b = 1, f, 0
    elif i == 1:
        r, g, b = q, 1, 0
    elif i == 2:
        r, g, b = 0, 1, f
    elif i == 3:
        r, g, b = 0, q, 1
    elif i == 4:
        r, g, b = f, 0, 1
    else:
        r, g, b = 1, 0, q  # i == 5
    return (int(r * 255), int(g * 255), int(b * 255))


def blend_rgb(base: tuple, tint: tuple, strength: float) -> tuple[int, int, int]:
    """Linearly blend *base* RGB toward *tint* by *strength* in [0..1].

    This is the second (surviving) ``_blend_rgb`` definition in the vendored
    class (lines 486-494) — the one the animation loop actually used.
    """
    s = max(0.0, min(1.0, strength))
    return (
        int(base[0] * (1 - s) + tint[0] * s),
        int(base[1] * (1 - s) + tint[1] * s),
        int(base[2] * (1 - s) + tint[2] * s),
    )


def blend_hue(base: tuple, tint: tuple, strength: float) -> tuple[int, int, int]:
    """Blend *base* toward *tint* through hue, keeping the colour saturated.

    A straight RGB lerp (:func:`blend_rgb`) walks in a straight line through
    the colour cube, and between two opposing hues that line passes through
    the neutral axis: yellow→blue crosses (128,140,100), orange→blue crosses
    (128,100,100). Those are grey, and a grey ring at low brightness reads as
    a white flash — which is exactly what the crossfade was producing on every
    colour change.

    Interpolating hue (shortest way round the wheel) with saturation and value
    lerped separately keeps every intermediate on the saturated surface, so the
    transition sweeps through colour instead of washing out through white.
    """
    s = max(0.0, min(1.0, strength))
    h1, s1, v1 = colorsys.rgb_to_hsv(*(c / 255.0 for c in base[:3]))
    h2, s2, v2 = colorsys.rgb_to_hsv(*(c / 255.0 for c in tint[:3]))

    # A fully desaturated endpoint (black/white/grey) has no meaningful hue —
    # borrow the other end's so the blend doesn't swing through an arbitrary one.
    if s1 <= 1e-6:
        h1 = h2
    if s2 <= 1e-6:
        h2 = h1

    delta = h2 - h1
    if delta > 0.5:
        delta -= 1.0
    elif delta < -0.5:
        delta += 1.0

    h = (h1 + delta * s) % 1.0
    r, g, b = colorsys.hsv_to_rgb(h, s1 + (s2 - s1) * s, v1 + (v2 - v1) * s)
    return (int(r * 255), int(g * 255), int(b * 255))


def sample_animation(anim: Anim, now_s: float) -> tuple[tuple, float]:
    """Return (rgb, brightness_scale) for a whole-ring animation at *now_s*.

    Exact vendored waveform math (led_ring_node.py lines 377-396):
      - pulse:     0.20..0.75 raised cosine
      - breathing: 0.16..0.60 raised cosine (16-60%)
      - flash:     fast bright pulse decaying exponentially to the 0.15
                   brightness floor — never goes fully dark; replaces the old
                   strobe to avoid photosensitive effects
      - rainbow:   whole-ring hue cycle at fixed 0.45 scale
    """
    t = (now_s - anim.start) % anim.period
    frac = t / anim.period
    if anim.name == "static":
        return anim.rgb, 1.0
    if anim.name == "pulse":
        bright = 0.20 + 0.55 * (0.5 - 0.5 * math.cos(2 * math.pi * frac))
        return anim.rgb, bright
    if anim.name == "breathing":
        bright = 0.16 + 0.44 * (0.5 - 0.5 * math.cos(2 * math.pi * frac))
        return anim.rgb, bright
    if anim.name == "flash":
        # Fast bright pulse that decays to the brightness floor (0.15) —
        # never goes fully dark. Replaces the old strobe to avoid
        # photosensitive effects.
        bright = 0.15 + 0.55 * math.exp(-frac * 4.5)
        return anim.rgb, bright
    if anim.name == "rainbow":
        return hue_to_rgb(frac), 0.45
    log.warning("Unknown animation type in loop: %s", anim.name)
    return anim.rgb, 1.0


def _unwrap_animations(animations: Optional[dict]) -> dict:
    """Accept either the parsed animations.yaml ({"animations": {...}}) or
    the inner name→definition mapping directly."""
    animations = animations or {}
    inner = animations.get("animations")
    if isinstance(inner, dict):
        return inner
    return animations


class LedRing:
    """Dual daisy-chained NeoPixel rings over hardware SPI, 50 Hz animations.

    ``config`` is ``puppet.yaml["led"]``; ``animations`` is the parsed
    animations.yaml (or its inner ``animations:`` mapping).
    """

    def __init__(self, config: dict, animations: dict) -> None:
        config = config or {}
        self._defs = _unwrap_animations(animations)

        self.pixel_count = int(config.get("pixel_count", DEFAULT_PIXEL_COUNT))
        self.pixel_order = str(config.get("pixel_order", DEFAULT_PIXEL_ORDER)).upper()
        if self.pixel_order not in VALID_PIXEL_ORDERS:
            log.warning(
                "Unsupported pixel_order '%s'. Falling back to GRB.", self.pixel_order
            )
            self.pixel_order = "GRB"
        self.lock_file_path = str(config.get("lock_file", DEFAULT_LOCK_FILE))
        self.default_animation = str(
            config.get("default_animation", DEFAULT_ANIMATION)
        )

        self.brightness = float(config.get("default_brightness", DEFAULT_BRIGHTNESS))
        self.max_brightness = float(config.get("max_brightness", DEFAULT_MAX_BRIGHTNESS))
        self.animation_brightness_cap = max(
            0.0, min(1.0, float(config.get("animation_brightness_cap",
                                           DEFAULT_ANIMATION_BRIGHTNESS_CAP)))
        )
        self.animation_brightness_floor = max(
            0.0, min(1.0, float(config.get("animation_brightness_floor",
                                           DEFAULT_ANIMATION_BRIGHTNESS_FLOOR)))
        )
        # Hardware brightness floor: fraction of max so LEDs never appear
        # fully off (vendored line 150).
        self.min_brightness = self.animation_brightness_floor * self.max_brightness
        self.animation_transition_s = max(
            0.0, float(config.get("animation_transition_s",
                                  DEFAULT_ANIMATION_TRANSITION_S)))
        self.animation_min_dwell_s = max(
            0.0, float(config.get("animation_min_dwell_s",
                                  DEFAULT_ANIMATION_MIN_DWELL_S)))

        self.pixels: Any = None
        # Last brightness pushed to the driver. Assigning `pixels.brightness`
        # rescales the entire output buffer, so only write it when it changes.
        self._last_brightness: Optional[float] = None
        # Last (rgb, brightness) actually clocked out — see _fill_and_show.
        self._last_frame: Optional[tuple] = None
        self._active_anim: Optional[Anim] = None
        self._previous_anim: Optional[Anim] = None
        self._transition_started_at: float = 0.0
        self._last_anim_change_at: float = 0.0
        self._pending_anim: Optional[Anim] = None
        self._is_shutting_down = False
        self._anim_lock = threading.Lock()
        self._pixel_lock = threading.Lock()  # guards all pixel writes
        self._anim_thread: Optional[threading.Thread] = None
        self._lockfile: Any = None
        self._running = False
        self._current: dict = {"animation": self.default_animation}

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def start(self) -> None:
        """Acquire the SPI lock, init NeoPixel hardware, start the 50 Hz thread."""
        if self._running:
            return

        # Advisory flock (new vs vendored) — fail fast if a second process is
        # driving the same SPI strip.
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
                "Could not acquire LED SPI lock %s; is another process driving "
                "the LED ring? %s",
                self.lock_file_path, e,
            )
            raise SystemExit(1)

        # Pi-only imports, deferred so --sim works without Blinka.
        try:
            import board
            import neopixel_spi
        except Exception as e:
            self._release_lockfile()
            raise RuntimeError(
                "LED hardware dependencies missing (board/neopixel_spi). "
                "Install with 'pip install adafruit-blinka neopixel-spi' and "
                "ensure SPI is enabled (sudo raspi-config -> Interface "
                f"Options -> SPI). Details: {e}"
            ) from e

        try:
            spi_bus = board.SPI()
            self.pixels = neopixel_spi.NeoPixel_SPI(
                spi_bus,
                self.pixel_count,
                brightness=min(self.brightness, self.max_brightness),
                auto_write=False,
                pixel_order=self.pixel_order,
            )
        except Exception as e:
            self._release_lockfile()
            raise RuntimeError(
                f"Failed to initialize NeoPixel_SPI hardware: {e}"
            ) from e
        log.info(
            "NeoPixel_SPI initialised (%d pixels, order %s).",
            self.pixel_count, self.pixel_order,
        )
        self._fill_and_show((0, 0, 0), force=True)  # Clear ring on startup

        try:
            self.set_animation(self.default_animation)
        except KeyError:
            log.warning(
                "Default animation '%s' not defined; falling back to static "
                "light blue.", self.default_animation,
            )
            # Vendored startup default: static light blue.
            self._fill_and_show((0, 90, 180), 1.0)

        self._is_shutting_down = False
        self._anim_thread = threading.Thread(
            target=self._anim_loop, daemon=True, name="maki-led-anim"
        )
        self._anim_thread.start()
        self._running = True
        log.info("LED ring started (default animation: %s).", self.default_animation)

    def stop(self) -> None:
        """Blank pixels, join the animation thread, release the lock. Idempotent."""
        self._is_shutting_down = True
        if self._anim_thread is not None and self._anim_thread.is_alive():
            self._anim_thread.join(timeout=0.5)
        self._anim_thread = None
        if self.pixels is not None:
            self._fill_and_show((0, 0, 0), force=True)
            log.info("LED ring cleared")
        self._release_lockfile()
        self._running = False

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

    # ------------------------------------------------------------------
    # Public command interface
    # ------------------------------------------------------------------

    def set_animation(self, name: str) -> None:
        """Switch to a named animation from animations.yaml.

        Raises KeyError on unknown names.  "off" is redirected to the ambient
        default so the ring never goes fully dark (vendored guard, lines
        444-449).
        """
        anim_name = str(name).lower()
        if anim_name == "off" and self.default_animation != "off":
            # LEDs must stay within the configured brightness floor — never go
            # fully dark. Redirect "off" to the ambient fallback.
            log.info("LED 'off' redirected → %s (ambient floor)", self.default_animation)
            anim_name = self.default_animation
        details = self._defs.get(anim_name)
        if not details:
            raise KeyError(f"Animation '{anim_name}' not defined in animations.yaml")
        anim_type = details.get("type", "static")
        color = tuple(details.get("color", [0, 0, 0]))
        period = float(details.get("period", 2.0))
        brightness_scale = float(details.get("brightness_scale", 1.0))
        if self._request_animation(Anim(anim_type, color, period, brightness_scale)):
            log.info(
                "Animation requested: %s with %s P:%ss", anim_type, color, period
            )
        with self._anim_lock:
            self._current = {"animation": anim_name}

    def set_color(self, r: int, g: int, b: int) -> None:
        """Show a static color, routed through the crossfade path so
        color→animation transitions blend smoothly instead of snapping
        (vendored _color_cb, lines 421-439)."""
        r = max(0, min(255, int(r)))
        g = max(0, min(255, int(g)))
        b = max(0, min(255, int(b)))
        queued = self._request_animation(Anim("static", (r, g, b), 1.0))
        if not queued:
            with self._anim_lock:
                active = self._active_anim
            if active is not None and active.name == "static":
                # Same static already active: update immediately so the anim
                # loop's 20 ms sleep doesn't delay the colour change.
                self._fill_and_show((r, g, b))
        with self._anim_lock:
            self._current = {"color": {"r": r, "g": g, "b": b}}

    @property
    def current(self) -> dict:
        with self._anim_lock:
            cur = self._current
            if "color" in cur:
                return {"color": dict(cur["color"])}
            return dict(cur)

    # ------------------------------------------------------------------
    # Animation engine (ported from led_ring_node.py)
    # ------------------------------------------------------------------

    @staticmethod
    def _anim_matches(left: Optional[Anim], right: Optional[Anim]) -> bool:
        if left is None or right is None:
            return False
        return (
            left.name == right.name
            and left.rgb == right.rgb
            and abs(left.period - right.period) < 1e-6
            and abs(left.brightness_scale - right.brightness_scale) < 1e-6
        )

    def _commit_animation_locked(self, anim: Anim, now_s: float) -> bool:
        previous = self._active_anim
        if self._anim_matches(previous, anim):
            self._pending_anim = None
            return False
        self._active_anim = anim
        self._last_anim_change_at = now_s
        # Spatial patterns (e.g. chase_rainbow) have no single-RGB sample to
        # crossfade from — sample_animation doesn't know how to represent
        # them, so skip the crossfade and snap straight to the new animation.
        if (
            previous is not None
            and previous.name != "chase_rainbow"
            and self.animation_transition_s > 0.0
        ):
            self._previous_anim = previous
            self._transition_started_at = now_s
        else:
            self._previous_anim = None
            self._transition_started_at = 0.0
        self._pending_anim = None
        return True

    def _request_animation(self, anim: Anim) -> bool:
        now_s = time.time()
        with self._anim_lock:
            if self._anim_matches(self._active_anim, anim):
                self._pending_anim = None
                return False
            if (
                self._active_anim is not None
                and self.animation_min_dwell_s > 0.0
                and (now_s - self._last_anim_change_at) < self.animation_min_dwell_s
            ):
                self._pending_anim = anim
                remaining = self.animation_min_dwell_s - (now_s - self._last_anim_change_at)
                log.info(
                    "LED dwell active — deferring %s %s for %.2fs",
                    anim.name, anim.rgb, remaining,
                )
                return True
            return self._commit_animation_locked(anim, now_s)

    def _fill_and_show(
        self, rgb_tuple: tuple, scale: float = 1.0, *, force: bool = False
    ) -> None:
        """Push one whole-ring colour, skipping writes that change nothing.

        The animation thread runs at 50 Hz regardless of animation type, so a
        `static` animation would otherwise re-clock an identical frame down the
        SPI bus 50 times a second. NeoPixel-over-SPI has no error detection, so
        every one of those frames is a chance for a timing glitch to latch a
        wrong colour into a pixel — visible as a stray red/green flash on an
        otherwise still ring. Deduplicating makes a static animation cost
        exactly one write.

        ``force`` bypasses the cache for writes that must land (clear on
        start/stop).
        """
        if self.pixels is None:
            return
        with self._pixel_lock:
            current_brightness = max(
                self.min_brightness,
                min(self.max_brightness, self.brightness * scale),
            )
            frame = (tuple(rgb_tuple), round(current_brightness, 4))
            if not force and frame == self._last_frame:
                return
            self._trace_near_white(rgb_tuple, "fill")
            self._set_brightness_locked(current_brightness)
            self.pixels.fill(rgb_tuple)
            self.pixels.show()
            self._last_frame = frame

    def _trace_near_white(self, rgb: tuple, source: str) -> None:
        """Diagnostic (MAKI_LED_TRACE=1): log any washed-out frame, with the
        stack of whoever wrote it.

        Flags on *greyness* — how close the channels are to each other —
        rather than on absolute level, because the ring runs at ~5% brightness
        where a dim grey like (60,70,65) still reads as a white flash but sits
        far below any sensible absolute threshold.
        """
        if not _LED_TRACE:
            return
        active = self._active_anim
        if active is not None and active.name == "static" and tuple(rgb[:3]) == active.rgb:
            return  # the exact colour asked for (e.g. the neutral white), not a wash-out
        r, g, b = rgb[:3]
        if max(r, g, b) < 12:
            return  # essentially off; nothing visible to wash out
        greyness = (min(r, g, b) + 1) / (max(r, g, b) + 1)
        if greyness > 0.6:
            log.warning(
                "LED washed-out frame %s (greyness %.2f) from %s (brightness=%s)",
                (r, g, b), greyness, source, self._last_brightness, stack_info=True,
            )


    def _set_brightness_locked(self, value: float) -> None:
        """Assign ``pixels.brightness`` only when it actually changes.

        The driver's brightness setter rescales every byte of the output
        buffer, so writing it on every frame doubles the per-frame work of the
        spatial rainbow for no visible effect. Caller must hold _pixel_lock.
        """
        rounded = round(value, 4)
        if rounded == self._last_brightness:
            return
        self.pixels.brightness = value
        self._last_brightness = rounded

    def _render_chase_rainbow(self, anim: Anim, now_s: float) -> None:
        """Spatial animation: each pixel gets its own hue, rotating around the
        strip. Writes a distinct color per pixel directly (lines 398-415)."""
        if self.pixels is None:
            return
        offset = (now_s - anim.start) / anim.period
        with self._pixel_lock:
            self._set_brightness_locked(max(
                self.min_brightness,
                min(self.max_brightness, self.brightness * anim.brightness_scale),
            ))
            n = max(1, self.pixel_count)
            for i in range(n):
                hue = ((i / n) + offset) % 1.0
                px = hue_to_rgb(hue)
                self._trace_near_white(px, "chase_rainbow")
                self.pixels[i] = px
            self.pixels.show()
            # Per-pixel write: the whole-ring cache no longer describes the
            # strip, so the next fill must repaint even if its colour matches.
            self._last_frame = None

    def _anim_loop(self) -> None:
        """Main animation loop, ~50 Hz. Runs in a daemon thread.

        Ported from led_ring_node.py _anim_loop (lines 507-585) minus the ROS
        engagement/affect modulation.
        """
        while not self._is_shutting_down:
            active_anim_copy: Optional[Anim] = None
            previous_anim_copy: Optional[Anim] = None
            transition_started_at = 0.0
            with self._anim_lock:
                now_s = time.time()
                if self._pending_anim is not None and (
                    self._active_anim is None
                    or (now_s - self._last_anim_change_at) >= self.animation_min_dwell_s
                ):
                    log.info(
                        "LED dwell elapsed — applying deferred %s %s",
                        self._pending_anim.name, self._pending_anim.rgb,
                    )
                    self._commit_animation_locked(self._pending_anim, now_s)
                if self._active_anim:
                    active_anim_copy = self._active_anim
                if self._previous_anim:
                    previous_anim_copy = self._previous_anim
                    transition_started_at = self._transition_started_at

            if active_anim_copy is None or self.pixels is None:
                time.sleep(0.05)
                continue

            now_s = time.time()

            if active_anim_copy.name == "chase_rainbow":
                # Spatial pattern — bypasses the whole-ring crossfade path
                # below, which only knows how to blend a single RGB value.
                self._render_chase_rainbow(active_anim_copy, now_s)
                # Full 50 Hz. This used to be throttled to 25 Hz on the theory
                # that per-pixel writes were the heaviest SPI load and so the
                # most likely source of the white flashes; they turned out to
                # come from the crossfade desaturating (see blend_hue), not
                # from the wire. At a 0.5 s revolution 25 Hz is only ~12 frames
                # per spin, which reads as a 4-pixel-per-step stutter.
                time.sleep(0.02)
                continue

            active_rgb, active_scale = sample_animation(active_anim_copy, now_s)

            if previous_anim_copy is not None and self.animation_transition_s > 0.0:
                transition_t = (now_s - transition_started_at) / self.animation_transition_s
                if transition_t >= 1.0:
                    with self._anim_lock:
                        self._previous_anim = None
                        self._transition_started_at = 0.0
                    output_rgb = active_rgb
                    output_scale = active_scale
                    output_gain = active_anim_copy.brightness_scale
                else:
                    prev_rgb, prev_scale = sample_animation(previous_anim_copy, now_s)
                    output_rgb = blend_hue(prev_rgb, active_rgb, transition_t)
                    mix = max(0.0, min(1.0, transition_t))
                    output_scale = prev_scale + (active_scale - prev_scale) * mix
                    prev_gain = previous_anim_copy.brightness_scale
                    output_gain = prev_gain + (
                        active_anim_copy.brightness_scale - prev_gain
                    ) * mix
            else:
                output_rgb = active_rgb
                output_scale = active_scale
                output_gain = active_anim_copy.brightness_scale

            # Clamp scale to [animation_brightness_floor, 1.0] before applying
            # the cap. This keeps LEDs within 15-90% of base brightness. The
            # per-animation brightness_scale applies after the clamp, so it is
            # the one way to go above that; _fill_and_show still caps the
            # result at max_brightness.
            clamped_scale = max(self.animation_brightness_floor, min(1.0, output_scale))
            self._fill_and_show(
                output_rgb,
                clamped_scale * self.animation_brightness_cap * output_gain,
            )

            time.sleep(0.02)
