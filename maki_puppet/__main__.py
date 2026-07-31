"""CLI entry point: ``python -m maki_puppet`` / ``maki-puppet``."""

from __future__ import annotations

import argparse
import asyncio
import logging
from pathlib import Path

from .app import PuppetApp

DEFAULT_CONFIG = Path(__file__).resolve().parent.parent / "config" / "puppet.yaml"


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        prog="maki-puppet",
        description="ROS-free puppet-mode gateway for the MAKI robot (MPP/1)",
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=DEFAULT_CONFIG,
        help=f"path to puppet.yaml (default: {DEFAULT_CONFIG})",
    )
    parser.add_argument(
        "--sim",
        action="store_true",
        help="use simulated hardware (no servos/LEDs required)",
    )
    parser.add_argument(
        "--log-level",
        default="INFO",
        choices=["DEBUG", "INFO", "WARNING", "ERROR"],
        help="logging level (default: INFO)",
    )
    args = parser.parse_args(argv)

    logging.basicConfig(
        level=getattr(logging, args.log_level),
        format="%(asctime)s %(levelname)-7s %(name)s: %(message)s",
    )

    app = PuppetApp(args.config, sim=True if args.sim else None)
    try:
        asyncio.run(app.run())
    except KeyboardInterrupt:  # pragma: no cover
        pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
