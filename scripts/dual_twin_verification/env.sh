#!/usr/bin/env bash
#
# Shared environment for the DUCO digital-twin reliability verification.
#
# Everything runs on an isolated ROS domain with local-only discovery so the
# test can never talk to the lab's real-robot session.
#
# Source this file, do not execute it.

export TWIN_WS="${TWIN_WS:-$HOME/duco_ros2_ws}"

# Isolated domain: keep the verification away from the real-robot network.
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-77}"
export ROS_LOCALHOST_ONLY=1

# ROS 2 Jazzy is built for the system Python; the conda python on PATH cannot
# import rclpy.  Always use this interpreter for the check scripts.
export TWIN_PYTHON="${TWIN_PYTHON:-/usr/bin/python3}"

# The ROS setup scripts are not `set -u` / `set -e` clean, and the caller may
# have enabled those.  Relax them while sourcing; callers rely on explicit
# error handling afterwards.
set +u
set +e

source /opt/ros/jazzy/setup.bash
source "$TWIN_WS/install/setup.bash"

export TWIN_WORLD="$TWIN_WS/install/duco_gcr5_910_moveit_config/share/duco_gcr5_910_moveit_config/worlds/gcr5_910_empty.sdf"

# Marker file used by the node under test (Phase B/C).
export TWIN_LATCH="${TWIN_LATCH:-/tmp/duco_dual_trajectory_twin_fault.state}"

# The node started by the launch file never receives goals (it has
# publish_real:=false) so it gets a marker path that is never written.
export TWIN_LAUNCH_LATCH="/tmp/duco_dual_trajectory_twin_launch_unused.state"

export TWIN_LOG_DIR="${TWIN_LOG_DIR:-/tmp/duco_dual_trajectory_twin_logs}"
