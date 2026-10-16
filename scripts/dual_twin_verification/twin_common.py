"""Shared helpers for the DUCO digital-twin reliability verification.

The verification drives the real ``DualTrajectoryAction`` node against the
Gazebo digital twin while a *synthetic* real robot publishes the two vendor
topics the node consumes (joint feedback + robot status).

It cannot move the physical arm:

* the ROS domain is isolated (see ``env.sh``) with local-only discovery,
* no real driver (``DucoDriver`` / ``DucoRobotStatus`` / ``DucoRobotControl``)
  is launched,
* in Phase B/C every real-* name is renamed to a test-only name
  (``/twin_joint_path_command``, ``/twin_real_feedback``, ``/twin_robot_state``,
  ``/twin_clear_fault``) that nobody consumes.

Run the check scripts with ``$TWIN_PYTHON`` (see ``env.sh``): ROS 2 Jazzy is
built for the system Python, not for the conda interpreter that is first on
PATH.
"""

import os
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from std_srvs.srv import Trigger
from duco_msg.msg import DucoRobotState

JOINTS = [
    "arm_1_joint_1", "arm_1_joint_2", "arm_1_joint_3",
    "arm_1_joint_4", "arm_1_joint_5", "arm_1_joint_6",
]

# Phase A drives the node started by the launch file; Phase B/C drive the
# isolated node instance with the test-only name.
ARM_ACTION = "/arm_1_controller/follow_joint_trajectory"
TWIN_ACTION = "/twin_front/follow_joint_trajectory"
CMD_TOPIC = "/twin_joint_path_command"
FB_TOPIC = "/twin_real_feedback"
ST_TOPIC = "/twin_robot_state"
CLEAR_SRV = "/twin_clear_fault"

MARKER = os.environ.get("TWIN_LATCH", "/tmp/duco_dual_trajectory_twin_fault.state")

GOAL_STATUS = {4: "SUCCEEDED", 5: "CANCELED", 6: "ABORTED"}

# FollowJointTrajectory::Result error codes.
PATH_TOLERANCE_VIOLATED = -4
GOAL_TOLERANCE_VIOLATED = -5


class Reporter:
    """Collects PASS/FAIL checks and prints a summary."""

    def __init__(self, tag):
        self.tag = tag
        self.results = []

    def check(self, name, ok, detail=""):
        ok = bool(ok)
        self.results.append((name, ok))
        print("[%s] %-58s %s %s" % (self.tag, name, "PASS" if ok else "FAIL", detail))
        return ok

    def summary(self):
        failed = [name for name, ok in self.results if not ok]
        print("[%s] ---- %d/%d checks passed ----" % (
            self.tag, len(self.results) - len(failed), len(self.results)))
        if failed:
            print("[%s] FAILURES: %s" % (self.tag, failed))
        return not failed


def wait_future(fut, timeout=20.0):
    """Wait for an already-spinning future without stealing the executor."""
    end = time.time() + timeout
    while time.time() < end and rclpy.ok():
        if fut.done():
            return fut.result()
        time.sleep(0.02)
    return None


class TwinHarness(Node):
    """Node under test driver: synthetic real I/O plus goal/result helpers."""

    def __init__(self, name, synthetic=True):
        super().__init__(name)
        self.latest_js = None
        self.cmd_count = 0
        self.last_cmd_final = None
        self.fb_positions = None
        self.emc = False
        self.collision = False
        self.robot_error = 0
        self._executor = None
        self._thread = None

        self.create_subscription(JointState, "/joint_states", self._on_js, 20)
        if synthetic:
            self.create_subscription(JointTrajectory, CMD_TOPIC, self._on_cmd, 20)
            self.pub_fb = self.create_publisher(FollowJointTrajectory.Feedback, FB_TOPIC, 50)
            self.pub_st = self.create_publisher(DucoRobotState, ST_TOPIC, 50)
            self.create_timer(0.02, self._tick)

        self.arm_client = ActionClient(self, FollowJointTrajectory, ARM_ACTION)
        self.twin_client = ActionClient(self, FollowJointTrajectory, TWIN_ACTION)
        self.clear_cli = self.create_client(Trigger, CLEAR_SRV)

    # ---- lifecycle ---------------------------------------------------------
    def start(self):
        self._executor = MultiThreadedExecutor()
        self._executor.add_node(self)
        self._thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._thread.start()
        return self

    def stop(self):
        """Shut the executor down and join before destroying, which avoids the
        rclpy 'terminate called without an active exception' race."""
        if self._executor is not None:
            self._executor.shutdown()
        if self._thread is not None:
            self._thread.join(timeout=5.0)
        self.destroy_node()

    # ---- inbound -----------------------------------------------------------
    def _on_js(self, msg):
        self.latest_js = msg

    def _on_cmd(self, msg):
        # The node publishes the original trajectory once it commands the real
        # arm, and an empty stop request when canceling.
        if msg.points:
            self.cmd_count += 1
            self.last_cmd_final = list(msg.points[-1].positions)
            self.fb_positions = list(msg.points[-1].positions)

    # ---- outbound (synthetic real robot) -----------------------------------
    def _tick(self):
        feedback = FollowJointTrajectory.Feedback()
        feedback.joint_names = JOINTS
        positions = self.fb_positions
        if positions is None:
            by_name = dict(zip(self.latest_js.name, self.latest_js.position)) \
                if self.latest_js is not None else {}
            positions = [by_name.get(j, 0.0) for j in JOINTS]
        feedback.actual.positions = list(positions)
        feedback.actual.velocities = [0.0] * 6
        self.pub_fb.publish(feedback)

        status = DucoRobotState()
        status.emc_stop_signal = self.emc
        status.collision = self.collision
        status.robot_error = int(self.robot_error)
        self.pub_st.publish(status)

    # ---- helpers -----------------------------------------------------------
    def wait_joint_states(self, timeout=60.0):
        end = time.time() + timeout
        while time.time() < end and rclpy.ok():
            if self.latest_js is not None:
                return True
            time.sleep(0.2)
        return False

    def positions(self):
        if self.latest_js is None:
            return [0.0] * 6
        by_name = dict(zip(self.latest_js.name, self.latest_js.position))
        return [by_name.get(j, 0.0) for j in JOINTS]

    def make_goal(self, start, target, millis=3000):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS
        for frac, ms in [(0.0, 0), (0.5, millis // 2), (1.0, millis)]:
            point = JointTrajectoryPoint()
            point.positions = [s + (t - s) * frac for s, t in zip(start, target)]
            point.velocities = [0.0] * 6
            point.time_from_start = Duration(sec=ms // 1000, nanosec=(ms % 1000) * 1000000)
            goal.trajectory.points.append(point)
        return goal

    def send_goal(self, client, goal):
        """Send a goal and return the handle *without* waiting for the result.

        Needed when the caller must act while the goal is still executing
        (cancel it, or inject a robot fault)."""
        return wait_future(client.send_goal_async(goal), 20.0)

    def send(self, client, goal, result_timeout=120.0):
        """Returns (goal_handle, status, error_code); status is None on refusal
        or timeout."""
        handle = self.send_goal(client, goal)
        if handle is None or not handle.accepted:
            return handle, None, None
        wrapped = wait_future(handle.get_result_async(), result_timeout)
        if wrapped is None:
            return handle, None, None
        return handle, wrapped.status, wrapped.result.error_code

    def wait_for_command(self, previous_count, timeout=20.0):
        end = time.time() + timeout
        while time.time() < end and self.cmd_count == previous_count:
            time.sleep(0.05)
        return self.cmd_count > previous_count

    def call_clear(self, timeout=20.0):
        if not self.clear_cli.wait_for_service(timeout_sec=10.0):
            return None
        return wait_future(self.clear_cli.call_async(Trigger.Request()), timeout)

    def clear_with_retry(self, tries=6, delay=0.6):
        last = None
        for _ in range(tries):
            last = self.call_clear()
            if last is not None and last.success:
                return last
            time.sleep(delay)
        return last


def add_script_dir_to_path():
    """Allow the check scripts to be run from anywhere."""
    here = os.path.dirname(os.path.abspath(__file__))
    if here not in sys.path:
        sys.path.insert(0, here)
