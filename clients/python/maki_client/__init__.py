"""maki_client — Python SDK for the MAKI puppet gateway (MPP/1).

Pure ``websockets`` + stdlib; no dependency on ``maki_puppet``. The wire
contract is ``maki_puppet/PROTOCOL.md``. Also usable as a CLI:
``python -m maki_client --host <robot> blink``.
"""

from .client import (
    DEFAULT_URL,
    PROTOCOL_VERSION,
    TERMINAL_STATUSES,
    ActionHandle,
    Maki,
    MakiActionError,
    MakiCancelled,
    MakiConnectionError,
    MakiError,
    MakiProtocolError,
    RobotState,
    par,
    seq,
)

__version__ = "0.1.0"

__all__ = [
    "DEFAULT_URL",
    "PROTOCOL_VERSION",
    "TERMINAL_STATUSES",
    "ActionHandle",
    "Maki",
    "MakiActionError",
    "MakiCancelled",
    "MakiConnectionError",
    "MakiError",
    "MakiProtocolError",
    "RobotState",
    "par",
    "seq",
]
