#!/usr/bin/env bash
#
# Start the DualTrajectoryAction instance used by Phase B/C.
#
# publish_real:=true is safe here: every real-* topic and service is renamed to
# a test-only name (see twin_common.py), no real-robot node is running, and the
# ROS domain is isolated.  The "real command" therefore goes to a topic that
# nobody consumes.
set -eo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=env.sh
source "$HERE/env.sh"

if [ -f "$TWIN_LATCH" ]; then
  echo "removing stale fault marker $TWIN_LATCH left by a previous run" >&2
fi
rm -f "$TWIN_LATCH"

exec ros2 run duco_ros_driver DualTrajectoryAction --ros-args \
  -r __node:=DualTwinTest \
  -p front_action_name:=twin_front/follow_joint_trajectory \
  -p sim_action_name:=sim_arm_1_controller/follow_joint_trajectory \
  -p real_command_topic:=twin_joint_path_command \
  -p publish_real:=true \
  -p wait_for_real:=true \
  -p publish_real_stop_request:=true \
  -p real_feedback_topic:=twin_real_feedback \
  -p real_status_topic:=/twin_robot_state \
  -p clear_fault_service:=twin_clear_fault \
  -p fault_latch_file:="$TWIN_LATCH" \
  -p real_goal_tolerance:=0.02 \
  -p real_velocity_tolerance:=0.05 \
  -p real_feedback_timeout:=1.0 \
  -p real_status_timeout:=1.0 \
  -p min_real_trajectory_points:=2 \
  -p cancel_timeout:=5.0
