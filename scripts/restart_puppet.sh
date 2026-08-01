#!/usr/bin/env bash
# Restart just the gateway, leaving the Bookbot app (and the desktop session)
# alone. Use this instead of rebooting to pick up a new deploy — a reboot takes
# ~90 s and drops the app; this takes ~10 s and the app simply reconnects.
#
#   ssh lux@robot '~/bookbot-maki/scripts/restart_puppet.sh'
#
# The boot script (start_bookbot_linux.sh) launches the gateway with nohup, so
# there is no service to restart — this reproduces that launch after stopping
# whatever is currently running.
#
# Overrides:
#   MAKI_PUPPET_LOG   gateway log (default: ~/run_puppet_log.txt)
#   MAKI_STOP_TIMEOUT seconds to wait for a clean stop (default: 10)
set -uo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PUPPET_LOG="${MAKI_PUPPET_LOG:-$HOME/run_puppet_log.txt}"
STOP_TIMEOUT="${MAKI_STOP_TIMEOUT:-10}"

# NOTE the bracket: `[m]aki_puppet` matches the string "maki_puppet" but NOT
# this script's own command line (which contains the literal brackets). Without
# it, pgrep/pkill match themselves and the shell running them — which has
# bitten this before, killing the ssh command mid-way and leaving the robot
# with no gateway at all.
PATTERN='python -m [m]aki_puppet'

pids() { pgrep -f "$PATTERN" || true; }

running="$(pids)"
if [ -n "$running" ]; then
  echo "stopping gateway (pid $(echo "$running" | tr '\n' ' '))"
  # SIGTERM, not SIGKILL: the gateway's handler torques the servos off and
  # releases the servo/LED lock files on the way out. SIGKILL leaves the head
  # under power in whatever pose it was holding.
  kill $running 2>/dev/null || true
  waited=0
  while [ -n "$(pids)" ] && [ "$waited" -lt "$STOP_TIMEOUT" ]; do
    sleep 1
    waited=$((waited + 1))
  done
  if [ -n "$(pids)" ]; then
    echo "did not stop after ${STOP_TIMEOUT}s, sending SIGKILL"
    kill -9 $(pids) 2>/dev/null || true
    sleep 1
  fi
else
  echo "no gateway running"
fi

echo "starting gateway (log: $PUPPET_LOG)"
# Same launch shape as the boot script: the subshell redirects its own stdio so
# the gateway doesn't hold our stdout open and hang the calling ssh.
(cd "$REPO_ROOT" && nohup bash "$REPO_ROOT/scripts/run_puppet.sh" \
  >"$PUPPET_LOG" 2>&1 </dev/null &) >/dev/null 2>&1

# Wait for it to actually serve, rather than reporting success on a launch that
# then died on a busy serial port or a bad config.
for _ in $(seq 1 30); do
  sleep 1
  if grep -q "maki_puppet up" "$PUPPET_LOG" 2>/dev/null; then
    grep -m1 "maki_puppet up" "$PUPPET_LOG"
    exit 0
  fi
  if ! pgrep -f "$PATTERN" >/dev/null && [ -s "$PUPPET_LOG" ]; then
    echo "gateway exited during startup:" >&2
    tail -15 "$PUPPET_LOG" >&2
    exit 1
  fi
done

echo "gateway did not report ready within 30s:" >&2
tail -15 "$PUPPET_LOG" >&2
exit 1
