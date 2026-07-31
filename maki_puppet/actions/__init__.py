"""Action implementations for the MPP/1 gateway.

``ACTIONS`` maps action kind → :class:`ActionSpec` (name, factory, channels,
param validation); ``execute_step`` runs a validated Step tree against an
:class:`ActionContext`.
"""

from .context import ActionContext
from .library import ACTIONS, ActionSpec, execute_step

__all__ = ["ACTIONS", "ActionContext", "ActionSpec", "execute_step"]
