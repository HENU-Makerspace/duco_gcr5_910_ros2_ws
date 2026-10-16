#!/usr/bin/env bash
#
# Stop the processes started by the verification.
#
# SAFETY: this must never kill a real-robot stack that may run on the same
# machine.  It only targets
#
#   1. the process groups recorded by verify_all.sh (the twin launch and the
#      node under test, including every child: gz sim, move_group,
#      robot_state_publisher, controllers, ...), and
#   2. two identifiers that only this harness produces, for launches started by
#      hand: the twin launch file name, and `ros2 run ... DualTrajectoryAction`
#      (the real stack is started from a launch file, not via ros2 run).
#
# It deliberately does NOT match generic names such as "move_group",
# "robot_state_publisher", "ruby" or a bare "DualTrajectoryAction", because the
# real stack uses those too.
set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TWIN_LOG_DIR="${TWIN_LOG_DIR:-/tmp/duco_dual_trajectory_twin_logs}"

pgids=()
for pgid_file in "$TWIN_LOG_DIR"/launch.pgid "$TWIN_LOG_DIR"/node.pgid; do
  [ -f "$pgid_file" ] || continue
  pgid="$(cat "$pgid_file" 2>/dev/null || true)"
  [ -n "$pgid" ] && pgids+=("$pgid")
done

# Polite shutdown first.  A negative PID means "the whole process group";
# verify_all starts both background jobs with setsid, so their PGID equals the
# recorded PID.
for pgid in "${pgids[@]:-}"; do
  [ -n "$pgid" ] && kill -- "-$pgid" 2>/dev/null
done

# Fallbacks, for runs started outside verify_all.sh.
pkill -f 'digital_twin_real.launch.py' 2>/dev/null
pkill -f 'ros2 run duco_ros_driver DualTrajectoryAction' 2>/dev/null

sleep 3

# move_group in particular can ignore SIGTERM, so escalate.
for pgid in "${pgids[@]:-}"; do
  [ -n "$pgid" ] && kill -9 -- "-$pgid" 2>/dev/null
done
pkill -9 -f 'digital_twin_real.launch.py' 2>/dev/null
pkill -9 -f 'ros2 run duco_ros_driver DualTrajectoryAction' 2>/dev/null

sleep 2
rm -f "$TWIN_LOG_DIR"/launch.pgid "$TWIN_LOG_DIR"/node.pgid

if pgrep -f 'digital_twin_real.launch.py' >/dev/null 2>&1 \
  || pgrep -f 'ros2 run duco_ros_driver DualTrajectoryAction' >/dev/null 2>&1; then
  echo "warning: some twin processes may still be running" >&2
  exit 1
fi

echo "twin environment stopped"
