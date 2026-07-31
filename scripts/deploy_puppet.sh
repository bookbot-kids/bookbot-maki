#!/usr/bin/env bash
# Sync this package to the MAKI robot and (re)install it into the robot's
# Python venv. No ROS anywhere: maki_puppet replaces the vendored ROS 2 stack
# at runtime and must never run alongside it (the shared servo lock file makes
# the second stack fail fast).
#
# Usage:
#   scripts/deploy_puppet.sh user@host
#   ROBOT_HOST=user@host scripts/deploy_puppet.sh
#
# The robot venv is expected at ~/lux_robot_venv; override with ROBOT_VENV.
set -euo pipefail

ROBOT="${1:-${ROBOT_HOST:-}}"
if [[ -z "$ROBOT" ]]; then
  echo "usage: $0 <user@host>   (or set ROBOT_HOST=user@host)" >&2
  exit 1
fi

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
REMOTE_DIR="maki_puppet"
REMOTE_VENV="${ROBOT_VENV:-\$HOME/lux_robot_venv}"

echo "Syncing $REPO_ROOT -> $ROBOT:~/$REMOTE_DIR"
rsync -avz --delete \
  --exclude '.git' \
  --exclude '__pycache__' \
  --exclude '.venv' \
  --exclude '*.egg-info' \
  --exclude '.pytest_cache' \
  --exclude '.DS_Store' \
  "$REPO_ROOT/" "$ROBOT:~/$REMOTE_DIR/"

echo "Installing into the robot venv..."
ssh "$ROBOT" ROBOT_VENV="$REMOTE_VENV" bash -s <<'REMOTE'
set -euo pipefail

VENV="$(eval echo "${ROBOT_VENV:-$HOME/lux_robot_venv}")"
if [[ ! -f "$VENV/bin/activate" ]]; then
  echo "error: venv not found at $VENV" >&2
  exit 1
fi

# On the reference robot this venv is shared with a ROS workspace, so its
# activate script sources /opt/ros/jazzy/setup.bash directly — and that file
# isn't `set -u` clean (AMENT_TRACE_SETUP_FILES is unbound). Relax nounset
# just for the source.
set +u
source "$VENV/bin/activate"
set -u
pip install -e "$HOME/maki_puppet[robot]"

# Stop any previously running puppet so the next start picks up this deploy
# (its atexit/SIGTERM handler torques servos off and releases the locks).
pkill -f 'maki_puppet' || true
REMOTE

cat <<EOF

Deployed. Next steps on the robot:

  ssh $ROBOT
  ~/maki_puppet/scripts/run_puppet.sh

Make sure no other stack is driving the servos — anything holding
/tmp/maki_servo_ttyUSB0.lock makes the second process exit immediately.

Bench smoke test from this machine:

  python scripts/smoke.py --host ${ROBOT#*@}
EOF
