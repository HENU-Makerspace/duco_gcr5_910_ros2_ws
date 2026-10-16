#!/usr/bin/python3
"""Phase A: MoveIt + Gazebo digital twin, no real-robot nodes.

Sends one well-formed FollowJointTrajectory goal to the action server provided
by the launch file and confirms from /joint_states that the Gazebo model really
moved.  ``use_real_robot:=false`` means ``publish_real`` is false, so the node
never publishes a real command.
"""

import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import rclpy  # noqa: E402
from twin_common import (  # noqa: E402
    ARM_ACTION,
    GOAL_STATUS,
    Reporter,
    TwinHarness,
)


def main():
    reporter = Reporter("A")
    rclpy.init()
    harness = TwinHarness("twin_phase_a", synthetic=False).start()

    if not harness.arm_client.wait_for_server(timeout_sec=90.0):
        reporter.check("action server available", False, ARM_ACTION)
        harness.stop()
        rclpy.shutdown()
        return 1
    reporter.check("action server available", True, ARM_ACTION)

    if not harness.wait_joint_states():
        reporter.check("Gazebo /joint_states available", False)
        harness.stop()
        rclpy.shutdown()
        return 1
    reporter.check("Gazebo /joint_states available", True)

    start = harness.positions()
    print("[A] start positions:", [round(p, 4) for p in start])

    target = list(start)
    target[0] = start[0] + 0.35
    target[4] = start[4] - 0.25

    # The node refuses goals until the Gazebo action server is ready, so retry
    # briefly instead of failing on a start-up race.
    goal = harness.make_goal(start, target)
    handle = None
    status = None
    error_code = None
    for attempt in range(1, 5):
        handle, status, error_code = harness.send(harness.arm_client, goal)
        if status is not None:
            break
        accepted = handle is not None and handle.accepted
        print("[A] attempt %d: goal did not complete (accepted=%s); retrying"
              % (attempt, accepted))
        time.sleep(3.0)

    reporter.check("goal reached the terminal state", status is not None,
                   "status=%s error_code=%s" % (status, error_code))
    reporter.check("goal SUCCEEDED", status == 4, GOAL_STATUS.get(status, "?"))

    time.sleep(1.5)
    end = harness.positions()
    deltas = [round(e - s, 4) for s, e in zip(start, end)]
    print("[A] end positions:  ", [round(p, 4) for p in end])
    print("[A] joint deltas:   ", deltas)
    reporter.check("Gazebo reached the commanded joints",
                   abs(end[0] - target[0]) < 0.05 and abs(end[4] - target[4]) < 0.05)

    ok = reporter.summary()
    harness.stop()
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
