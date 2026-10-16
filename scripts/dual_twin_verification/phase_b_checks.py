#!/usr/bin/python3
"""Phase B: fault-latch and recovery semantics on the live digital twin.

The node under test runs with ``publish_real:=true`` against a *synthetic* real
robot: this script publishes the two vendor topics the node consumes and records
the real command the node emits.  See ``twin_common.py`` for why this cannot
reach the physical arm.

Scenarios
  S1  healthy synchronized goal                             -> SUCCEEDED
  S2  cancel a goal that already commanded the real arm      -> CANCELED + latch
  S3  new goals refused while latched; operator recovery     -> usable again
  S4  unhealthy robot status while executing                 -> ABORTED + latch
  S5  recovery accepted once the robot reports healthy again
"""

import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import rclpy  # noqa: E402
from twin_common import (  # noqa: E402
    MARKER,
    Reporter,
    TwinHarness,
    wait_future,
)


def main():
    reporter = Reporter("B")
    print("[B] marker file:", MARKER, "exists:", os.path.exists(MARKER))

    rclpy.init()
    harness = TwinHarness("twin_phase_b", synthetic=True).start()

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
    reporter.check("digital twin /joint_states available", True)

    # An interrupted previous run can leave the node latched.  Clear it first so
    # the first scenario is not invalidated by stale state.
    response = harness.clear_with_retry(tries=8, delay=0.6)
    reporter.check("node starts unlatched (or is clearable)",
                   response is not None and response.success,
                   "msg=%r" % (response.message if response else None))
    time.sleep(1.0)  # let healthy synthetic status/feedback accumulate

    # ---- S1: healthy synchronized goal -------------------------------------
    start = harness.positions()
    target = list(start)
    target[0] = start[0] + 0.30
    target[4] = start[4] - 0.20
    before = harness.cmd_count
    _, status, error_code = harness.send(harness.twin_client, harness.make_goal(start, target))
    reporter.check("S1 healthy goal -> SUCCEEDED", status == 4,
                   "status=%s error_code=%s" % (status, error_code))
    reporter.check("S1 node published the real command", harness.cmd_count > before,
                   "cmd_count=%d" % harness.cmd_count)
    reporter.check("S1 marker removed after confirmed success", not os.path.exists(MARKER))

    # ---- S2: cancel a goal that already commanded the real arm -------------
    start = harness.positions()
    target = list(start)
    target[1] = start[1] + 0.40
    before = harness.cmd_count
    handle = harness.send_goal(
        harness.twin_client, harness.make_goal(start, target, 6000))
    reporter.check("S2 goal accepted", handle is not None and handle.accepted)
    reporter.check("S2 real command dispatched before cancel",
                   harness.wait_for_command(before))
    cancel_response = wait_future(handle.cancel_goal_async())
    reporter.check("S2 cancel accepted by the front server",
                   cancel_response is not None and len(cancel_response.goals_canceling) > 0)
    status = goal_status(handle)
    reporter.check("S2 canceled goal -> CANCELED", status == 5, "status=%s" % status)
    reporter.check("S2 unconfirmed cancel latched the fault", os.path.exists(MARKER))

    # ---- S3: refusal while latched, then operator recovery -----------------
    start = harness.positions()
    target = list(start)
    target[2] = start[2] + 0.25
    refused, _, _ = harness.send(harness.twin_client, harness.make_goal(start, target))
    reporter.check("S3 new goal refused while latched",
                   refused is not None and not refused.accepted)

    response = harness.clear_with_retry()
    reporter.check("S3 operator recovery succeeds once the cell is healthy",
                   response is not None and response.success,
                   "msg=%r" % (response.message if response else None))
    reporter.check("S3 marker removed after recovery", not os.path.exists(MARKER))

    start = harness.positions()
    target = list(start)
    target[3] = start[3] + 0.20
    _, status, error_code = harness.send(harness.twin_client, harness.make_goal(start, target))
    reporter.check("S3 goal accepted again after recovery", status == 4,
                   "status=%s error_code=%s" % (status, error_code))

    # ---- S4: unhealthy status while executing ------------------------------
    start = harness.positions()
    target = list(start)
    target[1] = start[1] - 0.35
    before = harness.cmd_count
    handle = harness.send_goal(
        harness.twin_client, harness.make_goal(start, target, 6000))
    reporter.check("S4 goal accepted", handle is not None and handle.accepted)
    reporter.check("S4 real command dispatched before fault injection",
                   harness.wait_for_command(before))
    harness.emc = True
    status = goal_status(handle)
    reporter.check("S4 unhealthy status -> ABORTED", status == 6, "status=%s" % status)
    reporter.check("S4 fault latched", os.path.exists(MARKER))

    # ---- S5: recovery after the fault clears -------------------------------
    response = harness.call_clear()
    reporter.check("S5 recovery refused while latched and unresolved",
                   response is not None and not response.success,
                   "msg=%r" % (response.message if response else None))

    harness.emc = False
    time.sleep(0.8)
    response = harness.clear_with_retry()
    reporter.check("S5 recovery accepted once the status is healthy again",
                   response is not None and response.success,
                   "msg=%r" % (response.message if response else None))
    reporter.check("S5 marker removed", not os.path.exists(MARKER))

    start = harness.positions()
    target = list(start)
    target[4] = start[4] + 0.15
    _, status, error_code = harness.send(harness.twin_client, harness.make_goal(start, target))
    reporter.check("S5 node ready for new goals after recovery", status == 4,
                   "status=%s error_code=%s" % (status, error_code))

    if os.path.exists(MARKER):
        with open(MARKER, encoding="utf-8") as handle:
            print("[B] marker content:", handle.read().strip())

    ok = reporter.summary()
    harness.stop()
    rclpy.shutdown()
    return 0 if ok else 1


def goal_status(handle, timeout=60.0):
    """Terminal status of an already-sent goal."""
    wrapped = wait_future(handle.get_result_async(), timeout)
    return wrapped.status if wrapped else None


if __name__ == "__main__":
    sys.exit(main())
