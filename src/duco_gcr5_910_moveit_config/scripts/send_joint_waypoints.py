#!/usr/bin/env python3
import argparse
import math
import sys

import rclpy
import yaml
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint


class JointWaypointSender(Node):
    def __init__(self, action_name, wait_sec):
        super().__init__("joint_waypoint_sender")
        self._client = ActionClient(self, FollowJointTrajectory, action_name)
        self._wait_sec = wait_sec

    def send(self, trajectory):
        if not self._client.wait_for_server(timeout_sec=self._wait_sec):
            raise RuntimeError(
                "FollowJointTrajectory action server is not available. "
                "Start digital_twin_real.launch.py first and keep it running."
            )

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            raise RuntimeError("Trajectory goal was rejected")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        return result_future.result().result


def load_trajectory(path):
    with open(path, "r", encoding="utf-8") as file:
        data = yaml.safe_load(file)

    names = data["joint_names"]
    points = data["waypoints"]
    if not isinstance(names, list) or not names:
        raise ValueError("joint_names must be a non-empty list")
    if any(not isinstance(name, str) or not name for name in names):
        raise ValueError("joint_names must contain non-empty strings")
    if len(set(names)) != len(names):
        raise ValueError("joint_names must not contain duplicates")
    if not isinstance(points, list) or not points:
        raise ValueError("waypoints must be a non-empty list")
    trajectory = FollowJointTrajectory.Goal().trajectory
    trajectory.joint_names = names

    previous_duration = 0.0
    for index, waypoint in enumerate(points):
        positions = waypoint["positions"]
        if len(positions) != len(names):
            raise ValueError(f"Waypoint {index} has {len(positions)} positions, expected {len(names)}")
        position_values = [float(value) for value in positions]
        if any(not math.isfinite(value) for value in position_values):
            raise ValueError(f"Waypoint {index} positions contain NaN or Inf")

        duration = float(waypoint["duration"])
        if not math.isfinite(duration) or duration <= previous_duration:
            raise ValueError(f"Waypoint {index} duration must be greater than the previous duration")
        previous_duration = duration

        point = JointTrajectoryPoint()
        point.positions = position_values
        for field in ("velocities", "accelerations", "effort"):
            if field in waypoint:
                values = [float(value) for value in waypoint[field]]
                if len(values) != len(names):
                    raise ValueError(
                        f"Waypoint {index} {field} has {len(values)} values, expected {len(names)}")
                if any(not math.isfinite(value) for value in values):
                    raise ValueError(f"Waypoint {index} {field} contains NaN or Inf")
                setattr(point, field, values)
        seconds = int(math.floor(duration))
        nanoseconds = int(round((duration - seconds) * 1_000_000_000))
        if nanoseconds >= 1_000_000_000:
            seconds += 1
            nanoseconds -= 1_000_000_000
        point.time_from_start.sec = seconds
        point.time_from_start.nanosec = nanoseconds
        trajectory.points.append(point)

    return trajectory


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("waypoints_file")
    parser.add_argument(
        "--action",
        default="/arm_1_controller/follow_joint_trajectory",
        help="FollowJointTrajectory action name",
    )
    parser.add_argument(
        "--wait-sec",
        type=float,
        default=60.0,
        help="Seconds to wait for the FollowJointTrajectory action server",
    )
    args = parser.parse_args(argv)

    rclpy.init()
    node = JointWaypointSender(args.action, args.wait_sec)
    try:
        trajectory = load_trajectory(args.waypoints_file)
        result = node.send(trajectory)
        node.get_logger().info(f"Trajectory finished with error_code={result.error_code}")
    except Exception as exc:
        node.get_logger().error(str(exc))
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
