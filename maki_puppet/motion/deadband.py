"""
Deadband filter for servo target suppression.

Prevents micro-oscillations by ignoring target updates that fall within a
configurable deadband of the last-committed position.  Each servo axis gets
its own ``DeadbandFilter`` instance with an independent threshold.

The deadband is a simple **position-only hysteresis band**: if the new target
is within ±deadband_rad of the last committed position, the update is
suppressed.  When a target finally exceeds the deadband, it is committed and
the band re-centers.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional


@dataclass
class DeadbandFilter:
    """Per-servo deadband filter with hysteresis.

    Parameters
    ----------
    deadband_rad : float
        Half-width of the deadband in radians.  If a new target is within
        ±deadband_rad of the last committed position, it is suppressed.
        Set to 0.0 to disable filtering.
    release_factor : float
        Hysteresis multiplier (>= 0).  After a target passes through the
        deadband, the *next* update uses a widened threshold of
        ``deadband_rad * (1 + release_factor)`` to prevent immediate
        re-triggering by jitter swinging back across the boundary.
        Once any update is suppressed (target close to committed),
        the threshold reverts to the normal ``deadband_rad``.
        0.0 = no hysteresis (same as original behavior).
    """

    deadband_rad: float = 0.025  # ~1.4 degrees
    release_factor: float = 0.5  # widen by 50% after commit

    # The last position that passed through the filter.
    _committed: Optional[float] = None
    # True immediately after a commit — next update uses wider threshold
    _widened: bool = False

    def update(self, new_target: float) -> Optional[float]:
        """Filter a new target position.

        Returns
        -------
        Optional[float]
            The target if it exceeds the deadband (i.e. should be forwarded
            to the S-curve planner), or ``None`` if it was suppressed.
        """
        if self.deadband_rad <= 0.0:
            # Filtering disabled — pass everything through.
            self._committed = new_target
            self._widened = False
            return new_target

        if self._committed is None:
            # First target — always commit.  Don't widen: there's no
            # prior movement to protect against jitter reversal.
            self._committed = new_target
            self._widened = False
            return new_target

        delta = abs(new_target - self._committed)

        # After a commit, temporarily widen the threshold to prevent
        # jitter from immediately swinging back past the boundary.
        if self._widened:
            threshold = self.deadband_rad * (1.0 + self.release_factor)
        else:
            threshold = self.deadband_rad

        if delta >= threshold:
            self._committed = new_target
            self._widened = True
            return new_target

        # Suppressed — revert to normal deadband for next update
        self._widened = False
        return None

    def reset(self, position: Optional[float] = None) -> None:
        """Reset the filter.  Next target will always pass through."""
        self._committed = position
        self._widened = False

    @property
    def committed(self) -> Optional[float]:
        """Last committed (passed-through) position, or None."""
        return self._committed
