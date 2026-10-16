#!/usr/bin/env bash
#
# Launch the MoveIt + Gazebo digital twin (Phase A) with no real-robot nodes.
#
# use_real_robot:=false => publish_real is false and DucoDriver / DucoRobotStatus
# / DucoRobotControl are NOT launched, so nothing can command the physical arm.
set -eo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=env.sh
source "$HERE/env.sh"

exec ros2 launch duco_gcr5_910_moveit_config digital_twin_real.launch.py \
  use_real_robot:=false \
  use_rviz:=false \
  wait_for_real:=false \
  fault_latch_file:="$TWIN_LAUNCH_LATCH" \
  gz_args:="-r -s -v 3 $TWIN_WORLD"
