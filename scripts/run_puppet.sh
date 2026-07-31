#!/usr/bin/env bash
# Robot-side runner for the MAKI puppet gateway.
#
# Activates the robot venv and execs the gateway. maki_puppet must never run
# alongside the ROS 2 stack, but the shared venv's own activate script does
# source /opt/ros/jazzy/setup.bash directly — that file isn't `set -u` clean
# (AMENT_TRACE_SETUP_FILES is unbound), so nounset is relaxed just for the
# source below. Extra arguments are passed straight through, e.g.:
#
#   ~/maki_puppet/scripts/run_puppet.sh            # normal run
#   ~/maki_puppet/scripts/run_puppet.sh --sim      # no-hardware simulation
#
# Overrides:
#   MAKI_VENV           venv to activate   (default: ~/lux_robot_venv)
#   MAKI_PUPPET_CONFIG  config file        (default: ~/maki_puppet/config/puppet.yaml)
set -euo pipefail

VENV="${MAKI_VENV:-$HOME/lux_robot_venv}"
CONFIG="${MAKI_PUPPET_CONFIG:-$HOME/maki_puppet/config/puppet.yaml}"

if [[ ! -f "$VENV/bin/activate" ]]; then
  echo "error: venv not found at $VENV" >&2
  exit 1
fi

set +u
source "$VENV/bin/activate"
set -u
exec python -m maki_puppet --config "$CONFIG" "$@"
