#!/usr/bin/python3
"""Phase C: pin down the recovery gate.

Same synthetic-real setup as Phase B, but after the fault is latched we first
wait for the outstanding Gazebo goal to reach a terminal state.  The *only*
remaining blocker is then the unhealthy robot status, so the refusal message
must name the robot fault rather than an outstanding simulation goal.
"""

import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import rclpy  # noqa: E402
from twin_common import MARKER, Reporter, TwinHarness, wait_future  # noqa: E402


def main():
    reporter = Reporter("C")
    rclpy.init()
    harness = TwinHarness("twin_phase_c", synthetic=True).start()

    if not harness.twin_client.wait_for_server(timeout_sec=90.0):
        reporter.check("action server available", False)
        harness.stop()
        rclpy.shutdown()
        return 1
    if not harness.wait_joint_states():
        reporter.check("digital twin /joint_states available", False)
        harness.stop()
        rclpy.shutdown()
        return 1

    # Start from a known-unlatched state.
    response = harness.clear_with_retry(tries=8, delay=0.6)
    reporter.check("starts from an unlatched state",
                   response is not None and response.success,
                   "msg=%r" % (response.message if response else None))
    time.sleep(0.5)

    start = harness.positions()
    target = list(start)
    target[2] = start[2] + 0.30
    before = harness.cmd_count
    handle = harness.send_goal(
        harness.twin_client, harness.make_goal(start, target, 8000))
    reporter.check("goal accepted", handle is not None and handle.accepted)
    reporter.check("real command dispatched", harness.wait_for_command(before))

    harness.emc = True
    wrapped = wait_future(handle.get_result_async(), 60.0)
    reporter.check("fault aborts the goal", wrapped is not None and wrapped.status == 6,
                   "status=%s" % (wrapped.status if wrapped else None))
    reporter.check("fault latched", os.path.exists(MARKER))

    # Let the orphaned Gazebo goal finish so it is no longer the blocker.
    time.sleep(4.0)

    response = harness.call_clear()
    message = (response.message if response else "") or ""
    reporter.check("recovery refused while the robot reports a fault",
                   response is not None and not response.success and "reports a fault" in message,
                   "msg=%r" % message)

    harness.emc = False
    time.sleep(1.0)
    response = harness.call_clear()
    message = (response.message if response else "") or ""
    reporter.check("recovery accepted once healthy",
                   response is not None and response.success, "msg=%r" % message)
    reporter.check("marker removed", not os.path.exists(MARKER))

    ok = reporter.summary()
    harness.stop()
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
