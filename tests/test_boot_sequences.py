"""Wake-up / go-to-sleep sequences around the gateway's lifetime.

These run outside the action engine (they bracket its lifetime), so they are
exercised against a real PuppetApp on the sim stack rather than the engine.
"""

from __future__ import annotations

import asyncio

import pytest

from maki_puppet.app import PuppetApp
from maki_puppet.motion import joints as joints_mod

from .test_server import CONFIG

pytestmark = pytest.mark.asyncio

# Short but non-zero, so the hold loop actually runs more than one pass.
BOOT = {
    "server": {"host": "127.0.0.1", "port": 0},
    "idle": {"delay_s": 300},
    "boot": {
        "startup": {
            "led_animation": "chase_rainbow",
            "head_tilt": 0.0,
            "eyelids": 1.0,
            "duration_s": 0.5,
        },
        "shutdown": {
            "led_animation": "sleep_deep_breathe",
            "head_tilt": 0.45,
            "eyelids": 0.0,
            # Long enough for the S-curve to actually arrive at the droop
            # target — head_tilt has ~22 deg of down travel to cover.
            "duration_s": 1.5,
            "timeout_s": 5.0,
        },
    },
}


async def test_startup_opens_eyes_levels_head_and_spins_rainbow():
    app = PuppetApp(CONFIG, sim=True, config_overrides=BOOT)
    await app.start()
    try:
        assert app.led.current == {"animation": "chase_rainbow"}
        pose = app.motion.pose_rad()
        assert pose["head_tilt"] == pytest.approx(
            joints_mod.norm_to_rad("head_tilt", 0.0), abs=0.05
        )
        # Eyes open == eyelids openness 1.0
        assert joints_mod.eyelids_rad_to_openness(
            pose["left_eyelid"]
        ) == pytest.approx(1.0, abs=0.1)
    finally:
        await app.stop()


async def test_shutdown_closes_eyes_drops_head_and_glows_blue():
    app = PuppetApp(CONFIG, sim=True, config_overrides=BOOT)
    await app.start()
    # Capture the final servo write before torque-off, since pose_rad() is
    # unavailable once the motion loop has stopped.
    final = {}
    original_close = app.bus.close

    def capture_then_close():
        final.update(app.motion.pose_rad())
        final["led"] = dict(app.led.current)
        return original_close()

    app.bus.close = capture_then_close
    await app.stop()

    assert final["led"] == {"animation": "sleep_deep_breathe"}
    assert final["head_tilt"] == pytest.approx(
        joints_mod.norm_to_rad("head_tilt", 0.45), abs=0.05
    )
    assert joints_mod.eyelids_rad_to_openness(
        final["left_eyelid"]
    ) == pytest.approx(0.0, abs=0.1)


async def test_shutdown_sequence_runs_before_torque_off():
    """Ordering guard: the head must settle while torque is still on."""
    app = PuppetApp(CONFIG, sim=True, config_overrides=BOOT)
    await app.start()

    order = []
    original_close = app.bus.close
    original_set = app.motion.set_layer

    def track_set(layer, targets):
        if layer == "gesture" and "left_eyelid" in targets:
            order.append("park")
        return original_set(layer, targets)

    def track_close():
        order.append("torque_off")
        return original_close()

    app.motion.set_layer = track_set
    app.bus.close = track_close
    await app.stop()

    assert "park" in order and "torque_off" in order
    assert order.index("park") < order.index("torque_off")


async def test_shutdown_sequence_is_bounded_by_timeout():
    """A wedged sequence must not hang shutdown."""
    cfg = {**BOOT, "boot": {**BOOT["boot"],
                            "shutdown": {**BOOT["boot"]["shutdown"],
                                         "duration_s": 60.0,
                                         "timeout_s": 0.5}}}
    app = PuppetApp(CONFIG, sim=True, config_overrides=cfg)
    await app.start()
    started = asyncio.get_event_loop().time()
    await app.stop()
    elapsed = asyncio.get_event_loop().time() - started
    assert elapsed < 5.0, f"shutdown took {elapsed:.1f}s; timeout did not fire"


async def test_boot_sequence_failure_does_not_block_startup():
    """A bad animation name must not stop the gateway coming up."""
    cfg = {**BOOT, "boot": {**BOOT["boot"],
                            "startup": {**BOOT["boot"]["startup"],
                                        "led_animation": "no_such_animation"}}}
    app = PuppetApp(CONFIG, sim=True, config_overrides=cfg)
    await app.start()
    try:
        assert app.server.port != 0   # came up anyway
    finally:
        await app.stop()
