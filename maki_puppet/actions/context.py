"""ActionContext — the execution environment handed to every action runner."""

from __future__ import annotations

import asyncio
import time
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, Optional, Set


@dataclass
class ActionContext:
    """Everything an action needs to touch the robot.

    ``motion``/``led`` follow the pinned MotionService/LedRing interfaces.
    ``vision`` is the VisionService (or None when no camera is configured —
    the `track` action rejects rather than silently standing still), and
    ``tracker_config`` is the tuning it builds each FaceTracker from.
    ``get_gesture(name)`` returns choreography keyframes (raises KeyError).
    ``claim_layer`` is the engine's layer-ownership hook: it records which
    performance most recently fed a motion layer so a superseded performance
    never releases a layer its replacement is actively using.
    ``touched_layers`` is what the engine releases when the performance ends
    (completed, cancelled, superseded or error) — nothing else is restored.
    """

    motion: Any
    led: Any
    vision: Optional[Any] = None
    tracker_config: Optional[Any] = None
    get_gesture: Callable[[str], list] = lambda name: (_ for _ in ()).throw(KeyError(name))
    clock: Callable[[], float] = time.monotonic
    sleep: Callable[[float], Any] = asyncio.sleep
    claim_layer: Optional[Callable[[str], None]] = None
    touched_layers: Set[str] = field(default_factory=set)

    def set_motion_layer(self, layer: str, targets_rad: Dict[str, float]) -> None:
        """Feed radian targets to a motion layer, tracking it for release."""
        if self.claim_layer is not None:
            self.claim_layer(layer)
        self.touched_layers.add(layer)
        self.motion.set_layer(layer, targets_rad)
