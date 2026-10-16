#!/usr/bin/env bash
#
# Run the whole digital-twin reliability verification (Phase A, B, C) and
# report a summary.  Everything is torn down on exit.
set -o pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=env.sh
source "$HERE/env.sh"
mkdir -p "$TWIN_LOG_DIR"

cleanup() {
  bash "$HERE/stop_all.sh" >/dev/null 2>&1
}
trap cleanup EXIT
bash "$HERE/stop_all.sh" >/dev/null 2>&1 || true

# Refuse to start on top of a previous run: leftovers would silently corrupt the
# results (a stale latch makes the very first goal fail).
for action in /arm_1_controller/follow_joint_trajectory \
              /sim_arm_1_controller/follow_joint_trajectory \
              /twin_front/follow_joint_trajectory; do
  if ros2 action list 2>/dev/null | grep -qx "$action"; then
    echo "FAIL: $action is already present; a previous run was not cleaned up." >&2
    echo "      Run scripts/dual_twin_verification/stop_all.sh first." >&2
    exit 1
  fi
done

wait_for_action() {
  local name="$1" tries="${2:-90}"
  for _ in $(seq 1 "$tries"); do
    if ros2 action list 2>/dev/null | grep -qx "$name"; then
      return 0
    fi
    sleep 1
  done
  return 1
}

echo "== Phase A: Gazebo + MoveIt digital twin =="
setsid bash "$HERE/phase_a_moveit_gazebo.sh" > "$TWIN_LOG_DIR/launch.log" 2>&1 &
echo $! > "$TWIN_LOG_DIR/launch.pgid"
if ! wait_for_action /arm_1_controller/follow_joint_trajectory; then
  echo "FAIL: the digital twin did not come up; see $TWIN_LOG_DIR/launch.log" >&2
  exit 1
fi
# The launch starts the controllers via a 5 s timer, and the node refuses goals
# until the Gazebo action server is ready, so wait for that too.
if ! wait_for_action /sim_arm_1_controller/follow_joint_trajectory; then
  echo "FAIL: the Gazebo controller did not come up; see $TWIN_LOG_DIR/launch.log" >&2
  exit 1
fi
sleep 2
"$TWIN_PYTHON" -u "$HERE/phase_a_check.py" | tee "$TWIN_LOG_DIR/phase_a.log"
A=${PIPESTATUS[0]}

echo
echo "== Phase B/C: isolated node instance against the twin =="
setsid bash "$HERE/phase_b_node.sh" > "$TWIN_LOG_DIR/node.log" 2>&1 &
echo $! > "$TWIN_LOG_DIR/node.pgid"
if ! wait_for_action /twin_front/follow_joint_trajectory; then
  echo "FAIL: the node under test did not come up; see $TWIN_LOG_DIR/node.log" >&2
  exit 1
fi
sleep 5

"$TWIN_PYTHON" -u "$HERE/phase_b_checks.py" | tee "$TWIN_LOG_DIR/phase_b.log"
B=${PIPESTATUS[0]}
"$TWIN_PYTHON" -u "$HERE/phase_c_check.py" | tee "$TWIN_LOG_DIR/phase_c.log"
C=${PIPESTATUS[0]}

echo
echo "== summary =="
printf 'Phase A: %s\n' "$([ "$A" -eq 0 ] && echo PASS || echo FAIL)"
printf 'Phase B: %s\n' "$([ "$B" -eq 0 ] && echo PASS || echo FAIL)"
printf 'Phase C: %s\n' "$([ "$C" -eq 0 ] && echo PASS || echo FAIL)"
printf 'logs:    %s\n' "$TWIN_LOG_DIR"

[ "$A" -eq 0 ] && [ "$B" -eq 0 ] && [ "$C" -eq 0 ]
