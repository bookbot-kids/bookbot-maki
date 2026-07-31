"""Simulated hardware drivers for MAKI puppet mode (Mac dev, no hardware).

SimServoBus and SimLedRing implement the exact interfaces of
:class:`maki_puppet.hw.servo_bus.ServoBus` and
:class:`maki_puppet.hw.led_ring.LedRing` with zero hardware dependencies.
They reuse the same pure conversion/clamping math and animation definitions,
so limit clamping and catalog validation behave identically to the real
drivers.
"""

from __future__ import annotations

import logging
import time
from typing import Any, Callable, Optional

from .led_ring import _unwrap_animations
from .servo_bus import (
    JointFeedback,
    build_joint_tables,
    rad_to_tick,
    tick_to_rad,
)

log = logging.getLogger(__name__)

# Full-range head moves complete in ~0.5 s — plausible servo speed, and fast
# enough that tests observing successive read_feedback() calls see motion.
DEFAULT_SLEW_RAD_S = 2.0


class SimServoBus:
    """Drop-in ServoBus replacement: feedback slews toward the last written
    (clamped) targets at a plausible rate; load is always 0.0, fresh always
    True.

    ``time_fn`` is injectable so tests can drive the slew deterministically.
    """

    def __init__(
        self,
        config: Optional[dict] = None,
        *,
        time_fn: Callable[[], float] = time.monotonic,
    ) -> None:
        config = config or {}
        self._time_fn = time_fn
        self.slew_rad_s = float(config.get("sim_slew_rad_s", DEFAULT_SLEW_RAD_S))

        self._joints = build_joint_tables(config)
        self._sign = {n: j["sign"] for n, j in self._joints.items()}
        self._limits = {n: j["ticks"] for n, j in self._joints.items()}

        # Positions and targets in body-centric radians, seeded at neutral.
        self._positions: dict[str, float] = {n: 0.0 for n in self._joints}
        self._targets: dict[str, float] = {n: 0.0 for n in self._joints}
        self._last_step_t: float = self._time_fn()
        self._open = False

    # -- lifecycle ------------------------------------------------------

    def open(self) -> None:
        if self._open:
            return
        self._open = True
        self._last_step_t = self._time_fn()
        log.debug("SimServoBus open (joints: %s)", ", ".join(self._joints))

    def close(self) -> None:
        if not self._open:
            return
        self._open = False
        log.debug("SimServoBus closed (torque off).")

    # -- runtime interface ---------------------------------------------

    @property
    def joint_names(self) -> list[str]:
        return list(self._joints.keys())

    def write_targets_rad(self, targets: dict[str, float]) -> None:
        if not self._open:
            raise RuntimeError("SimServoBus is not open")
        for name, rad in targets.items():
            joint = self._joints.get(name)
            if joint is None:
                log.debug("write_targets_rad: unknown joint '%s', skipping.", name)
                continue
            # Same choke-point math as the real bus: the effective target is
            # the clamped tick converted back to radians.
            tick = rad_to_tick(
                name, float(rad), self._sign, self._limits, joint["offset_rad"]
            )
            self._targets[name] = tick_to_rad(name, tick, self._sign)

    def read_feedback(self) -> dict[str, JointFeedback]:
        if not self._open:
            raise RuntimeError("SimServoBus is not open")
        now = self._time_fn()
        dt = max(0.0, now - self._last_step_t)
        self._last_step_t = now
        max_step = self.slew_rad_s * dt
        for name in self._joints:
            delta = self._targets[name] - self._positions[name]
            if abs(delta) <= max_step:
                self._positions[name] = self._targets[name]
            else:
                self._positions[name] += max_step if delta > 0 else -max_step
        return {
            name: JointFeedback(position_rad=self._positions[name], load=0.0, fresh=True)
            for name in self._joints
        }


class SimLedRing:
    """Drop-in LedRing replacement: records current state, logs transitions
    at DEBUG, touches no SPI hardware."""

    def __init__(
        self,
        config: Optional[dict] = None,
        animations: Optional[dict] = None,
    ) -> None:
        config = config or {}
        self._defs = _unwrap_animations(animations)
        self.default_animation = str(config.get("default_animation", "breathing_cyan"))
        self._current: dict = {"animation": self.default_animation}
        self._running = False

    # -- lifecycle ------------------------------------------------------

    def start(self) -> None:
        if self._running:
            return
        self._running = True
        if self.default_animation in self._defs:
            self._current = {"animation": self.default_animation}
        log.debug(
            "SimLedRing started (default animation: %s)", self.default_animation
        )

    def stop(self) -> None:
        if not self._running:
            return
        self._running = False
        log.debug("SimLedRing stopped (pixels blanked).")

    # -- command interface -----------------------------------------------

    def set_animation(self, name: str) -> None:
        anim_name = str(name).lower()
        if anim_name == "off" and self.default_animation != "off":
            # Mirror the real ring's never-fully-dark guard.
            log.debug(
                "SimLedRing 'off' redirected → %s (ambient floor)",
                self.default_animation,
            )
            anim_name = self.default_animation
        if not self._defs.get(anim_name):
            raise KeyError(f"Animation '{anim_name}' not defined in animations.yaml")
        log.debug("SimLedRing animation: %s -> %s", self._current, anim_name)
        self._current = {"animation": anim_name}

    def set_color(self, r: int, g: int, b: int) -> None:
        r = max(0, min(255, int(r)))
        g = max(0, min(255, int(g)))
        b = max(0, min(255, int(b)))
        log.debug("SimLedRing color: %s -> (%d, %d, %d)", self._current, r, g, b)
        self._current = {"color": {"r": r, "g": g, "b": b}}

    @property
    def current(self) -> dict:
        if "color" in self._current:
            return {"color": dict(self._current["color"])}
        return dict(self._current)
