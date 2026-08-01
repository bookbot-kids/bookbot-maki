#!/usr/bin/env bash
# Robot-side runner for the MAKI puppet gateway.
#
# Activates the robot venv and execs the gateway. maki_puppet must never run
# alongside the ROS 2 stack, but the shared venv's own activate script does
# source /opt/ros/jazzy/setup.bash directly — that file isn't `set -u` clean
# (AMENT_TRACE_SETUP_FILES is unbound), so nounset is relaxed just for the
# source below. Extra arguments are passed straight through, e.g.:
#
#   scripts/run_puppet.sh            # normal run
#   scripts/run_puppet.sh --sim      # no-hardware simulation
#
# The config is resolved RELATIVE TO THIS SCRIPT, not from a hardcoded path, so
# the runner always uses the checkout it was started from. It used to default to
# ~/maki_puppet/config/puppet.yaml, which meant a boot script that pulled one
# checkout could silently run another one's config — the two drifted and the
# `git pull` at boot updated a tree nothing executed.
#
# Overrides:
#   MAKI_VENV           venv to activate   (default: ~/lux_robot_venv)
#   MAKI_PUPPET_CONFIG  config file        (default: <this repo>/config/puppet.yaml)
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENV="${MAKI_VENV:-$HOME/lux_robot_venv}"
CONFIG="${MAKI_PUPPET_CONFIG:-$REPO_ROOT/config/puppet.yaml}"

if [[ ! -f "$VENV/bin/activate" ]]; then
  echo "error: venv not found at $VENV" >&2
  exit 1
fi

set +u
source "$VENV/bin/activate"
set -u

# Run from the repo root so `python -m maki_puppet` imports THIS checkout's
# package (sys.path[0] is the cwd, which outranks the venv's site-packages).
# Without this the code could come from a stale editable install while the
# config came from here — the same split this script's CONFIG default fixes.
cd "$REPO_ROOT"
exec python -m maki_puppet --config "$CONFIG" "$@"
