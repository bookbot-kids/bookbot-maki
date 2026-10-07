"""Application wiring: config → hardware → motion → engine → server.

``PuppetApp`` owns the whole stack and its startup/shutdown order.  With
``sim`` (config flag or --sim) the Sim drivers are wired instead of real
hardware, so the full gateway runs on a dev machine.
"""

from __future__ import annotations

import asyncio
import logging
import signal
import time
from pathlib import Path
from typing import Any, Mapping, Optional

import yaml

from .bridge import AppEventBridge, RobotAPI
from .choreography import Choreographer
from .engine import ActionEngine
from .hw import Camera, LedRing, ServoBus, SimCamera, SimLedRing, SimServoBus
from .idle import IdleBehavior
from .motion import MotionService
from .motion import joints as joints_mod
from .server import GatewayServer
from .vision import FaceTrackingBehavior, TrackerConfig, VisionService

log = logging.getLogger(__name__)

ESTOP_ALARM_ANIMATION = "alarm_red"

# How often the startup/shutdown sequences re-feed their motion layer.
# Must stay well under the gesture layer's claim_timeout_s (1.0 s).
BOOT_REFEED_S = 0.2


def _deep_merge(base: dict, override: Mapping) -> dict:
    out = dict(base)
    for key, value in override.items():
        if isinstance(value, Mapping) and isinstance(out.get(key), dict):
            out[key] = _deep_merge(out[key], value)
        else:
            out[key] = value
    return out


class PuppetApp:
    def __init__(
        self,
        config_path: Path,
        *,
        sim: Optional[bool] = None,
        config_overrides: Optional[Mapping[str, Any]] = None,
    ) -> None:
        self.config_path = Path(config_path)
        with open(self.config_path, "r", encoding="utf-8") as fh:
            cfg = yaml.safe_load(fh) or {}
        if config_overrides:
            cfg = _deep_merge(cfg, config_overrides)
        self.config = cfg
        # --sim forces sim; otherwise the config flag decides.
        self.sim = bool(cfg.get("sim", False)) if sim is None else bool(sim)

        config_dir = self.config_path.parent
        led_cfg = cfg.get("led") or {}
        animations_path = config_dir / str(led_cfg.get("animations_file", "animations.yaml"))
        with open(animations_path, "r", encoding="utf-8") as fh:
            animations_doc = yaml.safe_load(fh) or {}
        animation_names = list((animations_doc.get("animations") or animations_doc).keys())
        choreo_path = config_dir / str(cfg.get("choreographies_file", "choreographies.yaml"))

        # ── Hardware ───────────────────────────────────────────────────
        servo_cfg = cfg.get("servo") or {}
        if self.sim:
            self.bus: Any = SimServoBus(servo_cfg)
            self.led: Any = SimLedRing(led_cfg, animations_doc)
        else:
            self.bus = ServoBus(servo_cfg)
            self.led = LedRing(led_cfg, animations_doc)

        # ── Vision ─────────────────────────────────────────────────────
        # Optional: absent or `enabled: false` leaves self.vision None, and
        # the `track` action then rejects with vision_unavailable rather than
        # pretending to follow a face.
        vision_cfg = dict(cfg.get("vision") or {})
        self.vision: Optional[Any] = None
        self.tracker_config = TrackerConfig.from_mapping(vision_cfg.get("tracking"))
        if vision_cfg.get("enabled", False):
            camera_cfg = dict(vision_cfg.get("camera") or {})
            camera = SimCamera(camera_cfg) if self.sim else Camera(camera_cfg)
            self.vision = VisionService(
                camera,
                vision_cfg,
                sim=self.sim,
                # Closed-loop sim: the scripted face is positioned relative to
                # where the head actually is, so tracking converges in --sim
                # the way it does on the robot.
                pose_provider=lambda: self.motion.pose_rad(),
            )

        # ── Motion / choreography / engine / idle / server ─────────────
        motion_cfg = cfg.get("motion") or {}
        self.motion = MotionService(self.bus, motion_cfg)
        self.choreographer = Choreographer(choreo_path, animation_names)
        self.choreographer.load()
        self.engine = ActionEngine(
            self.motion,
            self.led,
            get_gesture=self.choreographer.get_gesture,
            vision=self.vision,
            tracker_config=self.tracker_config,
            alarm_animation=ESTOP_ALARM_ANIMATION,
            default_animation=str(led_cfg.get("default_animation", "breathing_cyan")),
        )
        self.idle = IdleBehavior(
            self.engine, self.motion, self.led, cfg.get("idle") or {},
            # Late-bound: the bridge is built below, and tests swap it out.
            led_hold=lambda: self.bridge.holds_ring,
        )
        # Autonomous face following. Armed after the wake sequence so MAKI
        # looks at whoever is in front of it without being asked.
        self.face_tracking = FaceTrackingBehavior(
            self.engine,
            self.motion,
            self.vision,
            self.tracker_config,
            vision_cfg.get("auto_track") or {},
        )
        self.bridge = AppEventBridge(
            RobotAPI(self.engine, self.choreographer, animation_names, self.led)
        )
        self.server = GatewayServer(
            cfg.get("server") or {},
            engine=self.engine,
            motion=self.motion,
            led=self.led,
            choreographer=self.choreographer,
            animations=animation_names,
            # `vision` is advertised in `welcome.capabilities` only when a
            # camera is actually configured, so a client can tell before it
            # sends a `track` act.
            capabilities=(
                ("motion", "mouth", "led", "vision")
                if self.vision is not None
                else ("motion", "mouth", "led")
            ),
            sim=self.sim,
            motion_rate_hz=float(motion_cfg.get("rate_hz", 50.0)),
            bridge=self.bridge,
        )

        self._boot_cfg = dict(cfg.get("boot") or {})
        self._stop_event: Optional[asyncio.Event] = None
        self._started = False

    # ── Lifecycle ──────────────────────────────────────────────────────

    async def _hold_pose(
        self,
        *,
        head_tilt: float,
        eyelids: float,
        duration_s: float,
    ) -> None:
        """Drive head_tilt/eyelids and hold them for ``duration_s``.

        Fed on the ``gesture`` layer, which times out after ~1 s if it stops
        being refreshed — so the targets are re-fed on a short cadence rather
        than set once. Runs outside the action engine on purpose: these
        sequences bracket the engine's own lifetime.
        """
        targets = {
            "head_tilt": joints_mod.norm_to_rad("head_tilt", head_tilt),
            **joints_mod.eyelids_openness_to_rad(eyelids),
        }
        deadline = time.monotonic() + duration_s
        while time.monotonic() < deadline:
            self.motion.set_layer("gesture", targets)
            await asyncio.sleep(BOOT_REFEED_S)
        self.motion.release_layer("gesture")

    async def _run_boot_sequence(self, phase: str) -> None:
        """Run the configured ``startup`` / ``shutdown`` sequence.

        Never raises: a failed flourish must not stop the gateway coming up,
        nor block it going down.
        """
        cfg = dict((self._boot_cfg.get(phase) or {}))
        if not cfg:
            return
        try:
            animation = cfg.get("led_animation")
            if animation:
                self.led.set_animation(str(animation))
            await self._hold_pose(
                head_tilt=float(cfg.get("head_tilt", 0.0)),
                eyelids=float(cfg.get("eyelids", 1.0)),
                duration_s=float(cfg.get("duration_s", 2.0)),
            )
        except Exception:
            log.exception("%s sequence failed", phase)

    async def start(self) -> None:
        """Bring the whole stack up (hardware first, network last)."""
        if self._started:
            return
        self.bus.open()
        self.led.start()
        self.motion.start()
        if self.vision is not None:
            # Never fatal: a missing or busy camera must not stop the gateway
            # coming up. `track` then rejects with vision_unavailable.
            try:
                self.vision.start()
            except Exception:
                log.exception("vision service failed to start; tracking disabled")
                self.vision = None
                self.engine.vision = None
        # Wake up before accepting clients, so the first connection sees a
        # robot that is already upright and looking at them.
        await self._run_boot_sequence("startup")
        await self.server.start()
        self.choreographer.start_watch()
        self.idle.start()
        # After the wake sequence: MAKI is upright with its eyes open, so this
        # is the moment it should start looking around for someone.
        self.face_tracking.start()
        self._started = True
        log.info(
            "maki_puppet up (%s) on ws://%s:%d/ws",
            "sim" if self.sim else "hardware",
            self.server.host,
            self.server.port,
        )

    async def stop(self) -> None:
        """Orderly shutdown: idle → vision → engine → server → motion → led → bus."""
        if not self._started:
            return
        self._started = False
        self.idle.stop()
        self.face_tracking.stop()
        if self.vision is not None:
            # Stop seeing before we stop moving, so no track performance picks
            # up a fresh face while the engine is being torn down.
            try:
                self.vision.stop()
            except Exception:
                log.exception("vision stop failed")
        self.choreographer.stop_watch()
        try:
            await self.engine.stop_all()
        except Exception:
            log.exception("engine stop_all failed")
        await self.server.stop()
        # Go to sleep while the motion loop and LED ring are still running —
        # this must land before motion.stop()/bus.close() cut torque, or the
        # head drops instead of settling. Bounded so a wedged bus can't hang
        # shutdown.
        try:
            await asyncio.wait_for(
                self._run_boot_sequence("shutdown"),
                timeout=float(
                    (self._boot_cfg.get("shutdown") or {}).get("timeout_s", 5.0)
                ),
            )
        except asyncio.TimeoutError:
            log.warning("shutdown sequence timed out; continuing")
        self.motion.stop()
        self.led.stop()
        self.bus.close()   # torque off + release lock (idempotent)
        log.info("maki_puppet stopped")

    def request_stop(self) -> None:
        if self._stop_event is not None:
            self._stop_event.set()

    async def run(self) -> None:
        """start() → wait for SIGINT/SIGTERM → stop().

        Signal handlers are installed AFTER bus.open() on purpose: the real
        ServoBus installs its own SIGTERM torque-off handler at open(); ours
        replaces it with a graceful shutdown whose bus.close() performs the
        same torque-off.
        """
        self._stop_event = asyncio.Event()
        await self.start()
        loop = asyncio.get_running_loop()
        for sig in (signal.SIGINT, signal.SIGTERM):
            try:
                loop.add_signal_handler(sig, self.request_stop)
            except (NotImplementedError, RuntimeError):  # pragma: no cover
                pass
        try:
            await self._stop_event.wait()
        finally:
            await self.stop()
