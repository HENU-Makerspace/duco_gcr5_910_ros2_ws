// Integration tests for DualTrajectoryAction.
//
// A mock FollowJointTrajectory action server stands in for the Gazebo
// controller, and the node under test is driven through its public ROS
// interfaces only (action client, real feedback/status topics and the recovery
// service).  All timeouts are shrunk to a few hundred milliseconds so the tests
// exercise the timeout paths without long waits.
//
// Every harness uses unique action, topic, service and node names.  Sharing
// names across cases makes the action client receive responses for a previous
// test's goal ("unknown goal response"), which produces flaky failures that have
// nothing to do with the node under test.
//
// These tests never touch a real robot: publish_real is used only to make the
// node believe a real command was dispatched, which is what the fault latch and
// the feedback gate reason about.

#include <atomic>
#include <algorithm>
#include <chrono>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include <cstdint>
#include <cstdio>
#include <fstream>
#include <sys/stat.h>
#include <unistd.h>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "duco_msg/msg/duco_robot_state.hpp"
#include "duco_ros_driver/DualTrajectoryAction.hpp"

using namespace std::chrono_literals;

namespace {

using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using SimGoalHandle = rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;
using ClientGoalHandle = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

const std::vector<std::string> kJointNames = {
  "arm_1_joint_1", "arm_1_joint_2", "arm_1_joint_3",
  "arm_1_joint_4", "arm_1_joint_5", "arm_1_joint_6"};

const std::vector<double> kGoalPositions = {0.1, -0.1, 0.1, -0.1, 0.1, -0.1};

/// Monotonic id so every harness gets its own names.
std::atomic<int> g_harness_id{0};

// ---------------------------------------------------------------------------
// Mock Gazebo controller
// ---------------------------------------------------------------------------

class MockSimServer : public rclcpp::Node
{
public:
  MockSimServer(const std::string & action_name, const std::string & node_name)
  : Node(node_name)
  {
    server_ = rclcpp_action::create_server<FollowJointTrajectory>(
      this, action_name,
      [this](
        const rclcpp_action::GoalUUID &, std::shared_ptr<const FollowJointTrajectory::Goal>) {
        if (accept_delay.count() > 0) {
          std::this_thread::sleep_for(accept_delay);
        }
        return accept_goal.load() ?
               rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE :
               rclcpp_action::GoalResponse::REJECT;
      },
      [this](const std::shared_ptr<SimGoalHandle>) {
        ++cancel_requests;
        return honor_cancel.load() ?
               rclcpp_action::CancelResponse::ACCEPT :
               rclcpp_action::CancelResponse::REJECT;
      },
      [this](const std::shared_ptr<SimGoalHandle> handle) { onAccepted(handle); });
  }

  /// Delay inserted inside the goal callback, to simulate a late acceptance.
  std::chrono::milliseconds accept_delay{0};
  std::chrono::milliseconds execute_duration{200};
  std::atomic<bool> accept_goal{true};
  std::atomic<bool> honor_cancel{true};
  /// Whether an accepted goal eventually succeeds or aborts.
  std::atomic<bool> succeed{true};
  /// When false, a cancelled goal never reports a result (cancel timeout).
  std::atomic<bool> answer_cancel_with_result{true};
  /// When true the controller ignores cancel requests entirely and always
  /// reports the outcome of the trajectory (used to make a cancel lose the race).
  std::atomic<bool> ignore_cancel{false};
  std::atomic<int> accepted_count{0};
  std::atomic<int> cancel_requests{0};

private:
  /// Goal handles kept alive to model a controller that accepted a cancel
  /// request but never sent a terminal state.
  std::mutex held_mutex_;
  std::vector<std::shared_ptr<SimGoalHandle>> held_;

  void onAccepted(const std::shared_ptr<SimGoalHandle> handle)
  {
    ++accepted_count;
    std::thread([this, handle]() {
      const auto deadline = std::chrono::steady_clock::now() + execute_duration;
      while (std::chrono::steady_clock::now() < deadline) {
        if (!ignore_cancel.load() && handle->is_canceling()) {
          if (answer_cancel_with_result.load()) {
            handle->canceled(std::make_shared<FollowJointTrajectory::Result>());
          } else {
            // Model "Gazebo accepted the cancel and never reported a terminal
            // state" faithfully.  The server-side goal handle must stay alive:
            // releasing the last reference lets rclcpp_action answer the pending
            // GetResult request with the CANCELING status, which would look like
            // a cancelled result the controller never sent.
            std::lock_guard<std::mutex> lock(held_mutex_);
            held_.push_back(handle);
          }
          return;
        }
        std::this_thread::sleep_for(10ms);
      }
      auto result = std::make_shared<FollowJointTrajectory::Result>();
      if (succeed.load()) {
        handle->succeed(result);
      } else {
        result->error_code = FollowJointTrajectory::Result::PATH_TOLERANCE_VIOLATED;
        handle->abort(result);
      }
    }).detach();
  }

  rclcpp_action::Server<FollowJointTrajectory>::SharedPtr server_;
};

// ---------------------------------------------------------------------------
// Harness
// ---------------------------------------------------------------------------

/**
 * \\brief Owns the node under test, the mock controller and a helper node.
 */
/// Optional harness settings that most tests do not need.
struct HarnessExtras {
  bool fault_recovery_enabled{true};
  /// Empty selects a unique path owned by this harness, so tests never share
  /// interlock state.  Set to a deliberately unusable path to exercise the
  /// durability failure path.
  std::string fault_latch_file;
  /// When false the node publishes real trajectories without a durable record.
  bool real_persistence_required{true};
  bool latch_on_real_fault{true};
  std::vector<int64_t> unhealthy_robot_states;
  bool require_velocity{true};
  double feedback_max_age{0.3};
  double status_max_age{0.3};
  double real_execution_timeout{3.0};
  double cancel_timeout{0.5};
};

/// A directory no user can create files in, used to prove that an unusable
/// interlock state file refuses real motion instead of being ignored.
constexpr char kUnusableLatchPath[] = "/proc/self/duco_test_marker/marker.state";

class Harness
{
public:
  using Extras = HarnessExtras;

  Harness(
    bool publish_real, bool wait_for_real,
    std::chrono::milliseconds accept_delay = 0ms,
    std::chrono::milliseconds execute_duration = 200ms,
    const Extras & extras = Extras())
  {
    const int id = ++g_harness_id;
    const std::string suffix = "_t" + std::to_string(id);
    front_action_ = "arm_1_controller/follow_joint_trajectory" + suffix;
    sim_action_ = "sim_arm_1_controller/follow_joint_trajectory" + suffix;
    real_command_topic_ = "joint_path_command" + suffix;
    motion_state_topic_ = "real_motion_state" + suffix;
    // Each harness owns its interlock state file so that one test can never
    // inherit a latch from another.
    marker_path_ = extras.fault_latch_file.empty() ?
      "/tmp/duco_dual_action_test_" + std::to_string(::getpid()) + "/marker" + suffix + ".state" :
      extras.fault_latch_file;

    sim_ = std::make_shared<MockSimServer>(sim_action_, "mock_sim_server" + suffix);
    sim_->accept_delay = accept_delay;
    sim_->execute_duration = execute_duration;

    auto options = rclcpp::NodeOptions();
    options.arguments({"--ros-args", "-r", "__node:=dual_action_proxy" + suffix});
    options.parameter_overrides({
      rclcpp::Parameter("front_action_name", front_action_),
      rclcpp::Parameter("sim_action_name", sim_action_),
      rclcpp::Parameter("real_command_topic", real_command_topic_),
      rclcpp::Parameter("publish_real", publish_real),
      rclcpp::Parameter("wait_for_real", wait_for_real),
      rclcpp::Parameter("real_feedback_topic", std::string("real_feedback_states") + suffix),
      rclcpp::Parameter("real_status_topic", std::string("/duco_cobot/robot_state") + suffix),
      rclcpp::Parameter("real_goal_tolerance", 0.02),
      rclcpp::Parameter("real_velocity_tolerance", 0.05),
      rclcpp::Parameter("acceptance_timeout", 0.6),
      rclcpp::Parameter("sim_execution_timeout", 3.0),
      rclcpp::Parameter("cancel_timeout", extras.cancel_timeout),
      rclcpp::Parameter("real_execution_timeout", extras.real_execution_timeout),
      rclcpp::Parameter("real_feedback_max_age", extras.feedback_max_age),
      rclcpp::Parameter("real_status_max_age", extras.status_max_age),
      rclcpp::Parameter("real_require_velocity", extras.require_velocity),
      rclcpp::Parameter("real_motion_state_topic", motion_state_topic_),
      rclcpp::Parameter("fault_recovery_enabled", extras.fault_recovery_enabled),
      rclcpp::Parameter("latch_on_real_fault", extras.latch_on_real_fault),
      rclcpp::Parameter("real_unhealthy_robot_states", extras.unhealthy_robot_states),
      rclcpp::Parameter("clear_fault_service", std::string("clear_fault") + suffix),
      rclcpp::Parameter("fault_latch_file", marker_path_),
      rclcpp::Parameter("real_persistence_required", extras.real_persistence_required),
      rclcpp::Parameter("sim_time_scale", 1.0),
    });

    proxy_ = std::make_shared<DualTrajectoryAction>(options);
    helper_ = std::make_shared<rclcpp::Node>("dual_action_test_helper" + suffix);

    client_ = rclcpp_action::create_client<FollowJointTrajectory>(helper_, front_action_);
    sim_client_ = rclcpp_action::create_client<FollowJointTrajectory>(helper_, sim_action_);
    status_pub_ = helper_->create_publisher<duco_msg::msg::DucoRobotState>(
      "/duco_cobot/robot_state" + suffix, 10);
    feedback_pub_ = helper_->create_publisher<FollowJointTrajectory::Feedback>(
      std::string("real_feedback_states") + suffix, 10);
    recovery_ = helper_->create_client<std_srvs::srv::Trigger>(std::string("clear_fault") + suffix);

    // Records the order of real motion and stop commands so a test can prove
    // that a stop was never overtaken by the motion it belongs to.
    real_command_sub_ = helper_->create_subscription<trajectory_msgs::msg::JointTrajectory>(
      real_command_topic_, 10,
      [this](trajectory_msgs::msg::JointTrajectory::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(command_mutex_);
        real_commands_.push_back(msg->points.empty() ? "stop" : "motion");
      });
    motion_state_sub_ = helper_->create_subscription<std_msgs::msg::String>(
      motion_state_topic_, 10,
      [this](std_msgs::msg::String::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(command_mutex_);
        latest_motion_state_ = msg->data;
      });

    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor_->add_node(proxy_);
    executor_->add_node(sim_);
    executor_->add_node(helper_);
    spin_thread_ = std::thread([this]() { executor_->spin(); });

    ready_ = client_->wait_for_action_server(10s) && sim_client_->wait_for_action_server(10s) &&
      recovery_->wait_for_service(10s);
    // Discovery of the proxy's own internal action client is independent of the
    // test client's, so give the graph a moment to settle.
    std::this_thread::sleep_for(500ms);

    startRealPumps();
  }

  ~Harness()
  {
    stopRealPumps();
    executor_->cancel();
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
    executor_->remove_node(proxy_);
    executor_->remove_node(sim_);
    executor_->remove_node(helper_);

    // The node owns the marker; drop it so a later test run starts clean.
    ::unlink(marker_path_.c_str());
    ::unlink((marker_path_ + ".tmp").c_str());
    ::rmdir(("/tmp/duco_dual_action_test_" + std::to_string(::getpid())).c_str());
  }

  bool ready() const { return ready_; }

  /// The interlock state file this harness configured.
  const std::string & markerPath() const { return marker_path_; }

  /// True when the durable interlock record exists right now.
  bool markerExists() const
  {
    struct stat info {};
    return ::stat(marker_path_.c_str(), &info) == 0;
  }

  /// Publish a healthy status and the current feedback from a background thread.
  void startRealPumps()
  {
    pumping_.store(true);
    pump_thread_ = std::thread([this]() {
      while (pumping_.load()) {
        publishStatus();
        publishFeedback(currentFeedback());
        std::this_thread::sleep_for(20ms);
      }
    });
  }

  void stopRealPumps()
  {
    if (pumping_.exchange(false) && pump_thread_.joinable()) {
      pump_thread_.join();
    }
  }

  /// Fault reported by the robot status: 0 none, 1 E-stop, 2 collision,
  /// 3 error code, 4 controller state.
  enum FaultMode { kNoFault = 0, kEmc = 1, kCollision = 2, kError = 3, kControllerState = 4 };
  void setHealthy(bool healthy) { fault_mode_.store(healthy ? kNoFault : kError); }
  void setFaultMode(int mode) { fault_mode_.store(mode); }

  void setFeedbackPositions(const std::vector<double> & positions)
  {
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    feedback_positions_ = positions;
  }

  std::vector<double> currentFeedback()
  {
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    return feedback_positions_;
  }

  void publishStatus()
  {
    duco_msg::msg::DucoRobotState msg;
    msg.emc_stop_signal = false;
    msg.collision = false;
    msg.robot_error = 0;
    msg.robot_state = 0;
    switch (fault_mode_.load()) {
      case kEmc:
        msg.emc_stop_signal = true;
        break;
      case kCollision:
        msg.collision = true;
        break;
      case kError:
        msg.robot_error = 42;
        break;
      case kControllerState:
        msg.robot_state = 5;
        break;
      default:
        break;
    }
    status_pub_->publish(msg);
  }

  void publishFeedback(const std::vector<double> & positions)
  {
    FollowJointTrajectory::Feedback msg;
    msg.joint_names = kJointNames;
    msg.actual.positions = positions;
    msg.actual.velocities.assign(positions.size(), 0.0);
    feedback_pub_->publish(msg);
  }

  FollowJointTrajectory::Goal makeGoal() const
  {
    FollowJointTrajectory::Goal goal;
    goal.trajectory.joint_names = kJointNames;
    trajectory_msgs::msg::JointTrajectoryPoint first;
    first.positions = std::vector<double>(kJointNames.size(), 0.0);
    first.time_from_start.sec = 0;
    first.time_from_start.nanosec = 500000000u;
    trajectory_msgs::msg::JointTrajectoryPoint last;
    last.positions = kGoalPositions;
    last.time_from_start.sec = 1;
    goal.trajectory.points = {first, last};
    return goal;
  }

  /// Send a goal and return the goal handle, or nullptr when rejected.
  std::shared_ptr<ClientGoalHandle> sendGoal(std::chrono::milliseconds timeout = 5s)
  {
    auto future = client_->async_send_goal(makeGoal());
    if (future.wait_for(timeout) != std::future_status::ready) {
      return nullptr;
    }
    return future.get();
  }

  /**
   * \\brief Wait for a terminal result.
   *
   * Returns false when the goal is still pending after \\p timeout.
   */
  bool waitForResult(
    const std::shared_ptr<ClientGoalHandle> & handle,
    rclcpp_action::ResultCode & code_out,
    std::chrono::milliseconds timeout = 5s)
  {
    if (!handle) {
      return false;
    }
    auto future = client_->async_get_result(handle);
    if (future.wait_for(timeout) != std::future_status::ready) {
      return false;
    }
    code_out = future.get().code;
    return true;
  }

  bool cancelGoal(const std::shared_ptr<ClientGoalHandle> & handle)
  {
    if (!handle) {
      return false;
    }
    auto future = client_->async_cancel_goal(handle);
    return future.wait_for(5s) == std::future_status::ready;
  }

  bool callRecovery(std::string & message)
  {
    auto future = recovery_->async_send_request(
      std::make_shared<std_srvs::srv::Trigger::Request>());
    if (future.wait_for(5s) != std::future_status::ready) {
      return false;
    }
    auto response = future.get();
    message = response->message;
    return response->success;
  }

  /// Like waitForResult(), but also returns the FollowJointTrajectory error code
  /// so a test can distinguish a clean cancellation from a failed one.
  bool waitForResultWithError(
    const std::shared_ptr<ClientGoalHandle> & handle,
    rclcpp_action::ResultCode & code_out, int32_t & error_out,
    std::chrono::milliseconds timeout = 5s)
  {
    if (!handle) {
      return false;
    }
    auto future = client_->async_get_result(handle);
    if (future.wait_for(timeout) != std::future_status::ready) {
      return false;
    }
    const auto wrapped = future.get();
    code_out = wrapped.code;
    error_out = wrapped.result ? wrapped.result->error_code : 0;
    return true;
  }

  /// The order in which real motion and stop commands were published.
  std::vector<std::string> realCommands()
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    return real_commands_;
  }

  std::string latestMotionState()
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    return latest_motion_state_;
  }

  bool waitForMotionStateContaining(
    const std::string & needle, std::chrono::milliseconds timeout = 5s)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      if (latestMotionState().find(needle) != std::string::npos) {
        return true;
      }
      std::this_thread::sleep_for(20ms);
    }
    return false;
  }

  std::shared_ptr<DualTrajectoryAction> proxy_;
  std::shared_ptr<MockSimServer> sim_;

private:
  std::string front_action_;
  std::string sim_action_;
  std::string real_command_topic_;
  std::string motion_state_topic_;
  std::string marker_path_;
  rclcpp::Node::SharedPtr helper_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr client_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr sim_client_;
  rclcpp::Publisher<duco_msg::msg::DucoRobotState>::SharedPtr status_pub_;
  rclcpp::Publisher<FollowJointTrajectory::Feedback>::SharedPtr feedback_pub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr recovery_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr real_command_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr motion_state_sub_;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread spin_thread_;
  std::thread pump_thread_;
  std::atomic<bool> pumping_{false};
  std::atomic<int> fault_mode_{kNoFault};
  std::mutex feedback_mutex_;
  std::vector<double> feedback_positions_{kGoalPositions};
  std::mutex command_mutex_;
  std::vector<std::string> real_commands_;
  std::string latest_motion_state_;
  bool ready_{false};
};

/**
 * \\brief Retry the recovery service until the node has seen real robot status.
 *
 * The status publisher is created with the helper node, so the first samples can
 * arrive after the test has started.  The node correctly refuses a recovery
 * before any status exists, so the tests wait for that first sample instead of
 * assuming it.
 */
bool waitForRecovery(Harness & harness, std::string & message, int attempts = 40)
{
  for (int attempt = 0; attempt < attempts; ++attempt) {
    if (harness.callRecovery(message)) {
      return true;
    }
    std::this_thread::sleep_for(50ms);
  }
  return false;
}

}  // namespace

class DualActionTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }
};

// ---------------------------------------------------------------------------
// Simulation-only paths
// ---------------------------------------------------------------------------

TEST_F(DualActionTest, SimulationOnlyGoalSucceeds)
{
  Harness h(false, false);
  ASSERT_TRUE(h.ready());
  auto handle = h.sendGoal();
  ASSERT_NE(handle, nullptr);

  rclcpp_action::ResultCode code{};
  ASSERT_TRUE(h.waitForResult(handle, code));
  EXPECT_EQ(code, rclcpp_action::ResultCode::SUCCEEDED);
}

TEST_F(DualActionTest, RejectedGoalIsAborted)
{
  Harness h(false, false);
  ASSERT_TRUE(h.ready());
  h.sim_->accept_goal = false;

  auto handle = h.sendGoal();
  ASSERT_NE(handle, nullptr);

  rclcpp_action::ResultCode code{};
  ASSERT_TRUE(h.waitForResult(handle, code));
  EXPECT_EQ(code, rclcpp_action::ResultCode::ABORTED);
}

TEST_F(DualActionTest, FailedSimulationIsAborted)
{
  Harness h(false, false);
  ASSERT_TRUE(h.ready());
  h.sim_->succeed = false;

  auto handle = h.sendGoal();
  ASSERT_NE(handle, nullptr);

  rclcpp_action::ResultCode code{};
  ASSERT_TRUE(h.waitForResult(handle, code));
  EXPECT_EQ(code, rclcpp_action::ResultCode::ABORTED);
}

// ---------------------------------------------------------------------------
// Cancellation
// ---------------------------------------------------------------------------

TEST_F(DualActionTest, CancelWhileExecutingIsForwardedToGazebo)
{
  Harness h(false, false, 0ms, 2s);
  ASSERT_TRUE(h.ready());
  auto handle = h.sendGoal();
  ASSERT_NE(handle, nullptr);
  std::this_thread::sleep_for(300ms);

  ASSERT_TRUE(h.cancelGoal(handle));
  EXPECT_GT(h.sim_->cancel_requests.load(), 0)
    << "the cancel must be forwarded to the Gazebo controller";

  rclcpp_action::ResultCode code{};
  ASSERT_TRUE(h.waitForResult(handle, code, 5s));
  EXPECT_EQ(code, rclcpp_action::ResultCode::CANCELED);
}

TEST_F(DualActionTest, CancelTimeoutIsReportedAsAbortedNotCanceled)
{
  // Gazebo accepts the cancellation but never reports a terminal state, so the
  // proxy has to fall back on its cancel timeout.  "The cancel request was
  // accepted" is not evidence that the trajectory stopped, and the ROS 2 action
  // state machine uses CANCELED for "the goal was canceled", so the honest
  // terminal state here is ABORTED.
  Harness h(false, false, 0ms, 10s);
  ASSERT_TRUE(h.ready());
  h.sim_->answer_cancel_with_result = false;

  auto handle = h.sendGoal();
  ASSERT_NE(handle, nullptr);
  std::this_thread::sleep_for(300ms);
  ASSERT_TRUE(h.cancelGoal(handle));

  rclcpp_action::ResultCode code{};
  int32_t error_code{};
  ASSERT_TRUE(h.waitForResultWithError(handle, code, error_code, 8s));
  EXPECT_EQ(code, rclcpp_action::ResultCode::ABORTED)
    << "a cancel that never produced a result must not be reported as a cancellation";
  EXPECT_EQ(error_code, FollowJointTrajectory::Result::GOAL_TOLERANCE_VIOLATED)
    << "the trajectory outcome was never established, so it is not a success";
}

// ---------------------------------------------------------------------------
// Real feedback gate
// ---------------------------------------------------------------------------

TEST_F(DualActionTest, RealCompletionIsRequiredWhenWaitForRealIsSet)
{
  // Gazebo finishes, but the real arm never reports that it reached the goal,
  // so the goal must not be reported as successful.
  Harness h(true, true);
  ASSERT_TRUE(h.ready());
  h.setFeedbackPositions(std::vector<double>(kJointNames.size(), 0.0));
  std::this_thread::sleep_for(100ms);

  auto handle = h.sendGoal();
  ASSERT_NE(handle, nullptr);

  rclcpp_action::ResultCode code{};
  const bool finished = h.waitForResult(handle, code, 2000ms);
  EXPECT_FALSE(finished)
    << "the goal completed as " << static_cast<int>(code)
    << " even though real feedback stayed outside tolerance";
}

TEST_F(DualActionTest, RealCompletionSucceedsWhenFeedbackReachesTheGoal)
{
  Harness h(true, true);
  ASSERT_TRUE(h.ready());
  h.setFeedbackPositions(kGoalPositions);

  auto handle = h.sendGoal();
  ASSERT_NE(handle, nullptr);

  rclcpp_action::ResultCode code{};
  ASSERT_TRUE(h.waitForResult(handle, code, 5s));
  EXPECT_EQ(code, rclcpp_action::ResultCode::SUCCEEDED);
}

TEST_F(DualActionTest, FaultReportedWhileExecutingAbortsTheGoalAndLatches)
{
  Harness h(true, true, 0ms, 2s);
  ASSERT_TRUE(h.ready());
  h.setFeedbackPositions(kGoalPositions);

  auto handle = h.sendGoal();
  ASSERT_NE(handle, nullptr);
  std::this_thread::sleep_for(200ms);
  h.setFaultMode(Harness::kEmc);

  rclcpp_action::ResultCode code{};
  ASSERT_TRUE(h.waitForResult(handle, code, 5s));
  EXPECT_EQ(code, rclcpp_action::ResultCode::ABORTED);

  // Even though the action goal is gone, the real channel stays locked.
  h.setFaultMode(Harness::kNoFault);
  std::this_thread::sleep_for(200ms);
  EXPECT_EQ(h.sendGoal(), nullptr)
    << "the interlock must survive the failed action goal";
}

TEST_F(DualActionTest, FaultReportedWhileIdleRejectsRealGoals)
{
  Harness h(true, true);
  ASSERT_TRUE(h.ready());
  h.setFaultMode(Harness::kEmc);
  std::this_thread::sleep_for(250ms);

  EXPECT_EQ(h.sendGoal(), nullptr)
    << "a fault reported before any goal must already forbid real motion";
}

TEST_F(DualActionTest, ConfiguredControllerStateIsTreatedAsAFault)
{
  Harness::Extras extras;
  extras.unhealthy_robot_states = {5};
  Harness h(true, true, 0ms, 200ms, extras);
  ASSERT_TRUE(h.ready());
  h.setFaultMode(Harness::kControllerState);
  std::this_thread::sleep_for(250ms);

  EXPECT_EQ(h.sendGoal(), nullptr)
    << "a controller state declared unsafe by the operator must latch the interlock";
}

TEST_F(DualActionTest, UndeclaredControllerStateIsNotInterpreted)
{
  // The vendor does not document the controller state enum, so it must not be
  // guessed to be a fault.
  Harness h(true, true);
  ASSERT_TRUE(h.ready());
  h.setFaultMode(Harness::kControllerState);
  std::this_thread::sleep_for(250ms);

  auto handle = h.sendGoal();
  EXPECT_NE(handle, nullptr) << "an undocumented controller state must not be invented into a fault";
}

TEST_F(DualActionTest, SimulationOnlyModeIgnoresRealRobotFaults)
{
  Harness h(false, false);
  ASSERT_TRUE(h.ready());
  h.setFaultMode(Harness::kEmc);
  std::this_thread::sleep_for(250ms);

  auto handle = h.sendGoal();
  ASSERT_NE(handle, nullptr) << "the simulation channel is isolated from the real channel";
  rclcpp_action::ResultCode code{};
  ASSERT_TRUE(h.waitForResult(handle, code, 5s));
  EXPECT_EQ(code, rclcpp_action::ResultCode::SUCCEEDED);
}

// ---------------------------------------------------------------------------
// Fault latch and recovery
// ---------------------------------------------------------------------------

TEST_F(DualActionTest, CancelTimeoutWithRealCommandLatchesAndBlocksNewGoals)
{
  // A real command is dispatched, then the cancellation can never be confirmed.
  Harness h(true, true, 0ms, 10s);
  ASSERT_TRUE(h.ready());
  h.sim_->answer_cancel_with_result = false;

  auto handle = h.sendGoal();
  ASSERT_NE(handle, nullptr);
  std::this_thread::sleep_for(400ms);
  ASSERT_TRUE(h.cancelGoal(handle));

  rclcpp_action::ResultCode code{};
  ASSERT_TRUE(h.waitForResult(handle, code, 8s));
  EXPECT_EQ(code, rclcpp_action::ResultCode::ABORTED)
    << "an unconfirmed cancellation is a failure, not a cancellation";

  // The real command was published and its stop was never confirmed, so the
  // next goal must be refused rather than silently accepted.
  auto second = h.sendGoal();
  EXPECT_EQ(second, nullptr)
    << "a goal was accepted after an unconfirmed real stop";
  EXPECT_TRUE(h.markerExists())
    << "the unconfirmed real motion must be recorded durably before the goal ends";
}

TEST_F(DualActionTest, RecoveryIsRefusedWhileTheRobotIsUnhealthy)
{
  Harness h(true, true);
  ASSERT_TRUE(h.ready());

  // The real command is dispatched cleanly, then Gazebo fails, which latches
  // the fault because the arm's state is no longer known.
  h.sim_->succeed = false;
  auto handle = h.sendGoal();
  ASSERT_NE(handle, nullptr);
  rclcpp_action::ResultCode code{};
  ASSERT_TRUE(h.waitForResult(handle, code, 5s));
  ASSERT_EQ(code, rclcpp_action::ResultCode::ABORTED);

  h.setHealthy(false);
  std::this_thread::sleep_for(300ms);

  std::string message;
  EXPECT_FALSE(h.callRecovery(message));
  EXPECT_NE(message.find("refusing"), std::string::npos) << message;
}

TEST_F(DualActionTest, RecoverySucceedsOnceTheRobotIsHealthyAgain)
{
  Harness h(true, true);
  ASSERT_TRUE(h.ready());

  h.sim_->succeed = false;
  auto handle = h.sendGoal();
  ASSERT_NE(handle, nullptr);
  rclcpp_action::ResultCode code{};
  ASSERT_TRUE(h.waitForResult(handle, code, 5s));
  ASSERT_EQ(code, rclcpp_action::ResultCode::ABORTED);

  h.setHealthy(false);
  std::this_thread::sleep_for(300ms);
  std::string message;
  ASSERT_FALSE(h.callRecovery(message));

  h.setHealthy(true);
  std::this_thread::sleep_for(300ms);
  EXPECT_TRUE(h.callRecovery(message)) << message;
  EXPECT_NE(message.find("cleared"), std::string::npos) << message;

  // The node accepts trajectories again after an explicit recovery.
  h.sim_->succeed = true;
  auto second = h.sendGoal();
  ASSERT_NE(second, nullptr) << "the node should accept goals after recovery";
  rclcpp_action::ResultCode second_code{};
  ASSERT_TRUE(h.waitForResult(second, second_code, 5s));
  EXPECT_EQ(second_code, rclcpp_action::ResultCode::SUCCEEDED);
}

// ---------------------------------------------------------------------------
// Late Gazebo responses
// ---------------------------------------------------------------------------

TEST_F(DualActionTest, LateAcceptanceAfterAcceptanceTimeoutIsCancelled)
{
  // Gazebo answers well after acceptance_timeout (0.6 s).  The response must
  // not be dropped, and the stale simulation goal must be cancelled rather than
  // left running.
  Harness h(false, false, 1200ms, 10s);
  ASSERT_TRUE(h.ready());

  auto handle = h.sendGoal();
  ASSERT_NE(handle, nullptr);

  rclcpp_action::ResultCode code{};
  ASSERT_TRUE(h.waitForResult(handle, code, 8s));
  EXPECT_EQ(code, rclcpp_action::ResultCode::ABORTED);

  // The delayed acceptance arrives after the front goal is gone.  It must be
  // cancelled rather than dropped, which the mock can observe.
  const auto deadline = std::chrono::steady_clock::now() + 5s;
  while (std::chrono::steady_clock::now() < deadline && h.sim_->cancel_requests.load() == 0) {
    std::this_thread::sleep_for(50ms);
  }
  EXPECT_GT(h.sim_->accepted_count.load(), 0)
    << "the delayed goal should have been accepted by the mock controller";
  EXPECT_GT(h.sim_->cancel_requests.load(), 0)
    << "a late acceptance must trigger a cancel instead of being silently dropped";
}

// ---------------------------------------------------------------------------
// Real motion interlock: unconfirmed motion must not be overwritten
// ---------------------------------------------------------------------------

TEST_F(DualActionTest, UnconfirmedRealMotionBlocksTheNextRealTrajectory)
{
  // wait_for_real=false still reports the simulation result, but the real arm's
  // completion was never confirmed, so the next real trajectory must not be
  // allowed to overwrite an unaccounted motion.
  Harness h(true, false);
  ASSERT_TRUE(h.ready());
  h.setFeedbackPositions(std::vector<double>(kJointNames.size(), 0.0));

  auto handle = h.sendGoal();
  ASSERT_NE(handle, nullptr);
  rclcpp_action::ResultCode code{};
  ASSERT_TRUE(h.waitForResult(handle, code, 5s));
  EXPECT_EQ(code, rclcpp_action::ResultCode::SUCCEEDED)
    << "the simulation result is still reported";

  EXPECT_EQ(h.sendGoal(), nullptr)
    << "an unconfirmed real motion must not be overwritten by the next trajectory";

  std::string message;
  ASSERT_TRUE(h.callRecovery(message)) << message;
  EXPECT_NE(h.sendGoal(), nullptr) << "a real goal is accepted again after an explicit recovery";
}

TEST_F(DualActionTest, ConfirmedCompletionReleasesTheChannelForTheNextGoal)
{
  Harness h(true, true);
  ASSERT_TRUE(h.ready());
  h.setFeedbackPositions(kGoalPositions);

  for (int i = 0; i < 2; ++i) {
    auto handle = h.sendGoal();
    ASSERT_NE(handle, nullptr) << "goal " << i << " was refused";
    rclcpp_action::ResultCode code{};
    ASSERT_TRUE(h.waitForResult(handle, code, 5s));
    EXPECT_EQ(code, rclcpp_action::ResultCode::SUCCEEDED) << "goal " << i;
  }
}

TEST_F(DualActionTest, RealExecutionTimeoutLatchesAndBlocksNewGoals)
{
  Harness::Extras extras;
  extras.real_execution_timeout = 1.0;
  Harness h(true, true, 0ms, 200ms, extras);
  ASSERT_TRUE(h.ready());
  // The arm never reports the goal, so the real execution timeout decides.
  h.setFeedbackPositions(std::vector<double>(kJointNames.size(), 0.0));

  auto handle = h.sendGoal();
  ASSERT_NE(handle, nullptr);
  rclcpp_action::ResultCode code{};
  ASSERT_TRUE(h.waitForResult(handle, code, 8s));
  EXPECT_EQ(code, rclcpp_action::ResultCode::ABORTED);

  EXPECT_EQ(h.sendGoal(), nullptr)
    << "a timed out real execution must forbid the next real trajectory";
}

TEST_F(DualActionTest, InterlockIsRestoredFromTheMarkerFile)
{
  const std::string marker = "/tmp/duco_dual_trajectory_test_marker.state";
  {
    std::ofstream out(marker, std::ios::trunc);
    out << "goal-uuid uncertain: interrupted run\n";
  }

  Harness::Extras extras;
  extras.fault_latch_file = marker;
  Harness h(true, true, 0ms, 200ms, extras);
  ASSERT_TRUE(h.ready());
  EXPECT_EQ(h.sendGoal(), nullptr)
    << "a restart must not silently clear an unconfirmed real motion";

  // Simulation-only mode never publishes a real trajectory, so the marker must
  // not block a pure simulation session.
  Harness sim_only(false, false, 0ms, 200ms, extras);
  ASSERT_TRUE(sim_only.ready());
  EXPECT_NE(sim_only.sendGoal(), nullptr)
    << "a stale real-robot marker must not block simulation-only operation";

  std::remove(marker.c_str());
}

// ---------------------------------------------------------------------------
// Cancellation result mapping
// ---------------------------------------------------------------------------

TEST_F(DualActionTest, RejectedCancelIsNotReportedAsACleanCancellation)
{
  // Gazebo refuses the cancel and never reports a terminal state, so the proxy
  // falls back on its cancel timeout and must not claim a clean cancellation.
  Harness h(false, false, 0ms, 10s);
  ASSERT_TRUE(h.ready());
  h.sim_->honor_cancel = false;
  h.sim_->answer_cancel_with_result = false;

  auto handle = h.sendGoal();
  ASSERT_NE(handle, nullptr);
  std::this_thread::sleep_for(200ms);
  ASSERT_TRUE(h.cancelGoal(handle));

  rclcpp_action::ResultCode code{};
  int32_t error_code = 0;
  ASSERT_TRUE(h.waitForResultWithError(handle, code, error_code, 8s));
  EXPECT_EQ(code, rclcpp_action::ResultCode::ABORTED)
    << "a rejected cancel must not be reported as a cancellation";
  EXPECT_EQ(error_code, FollowJointTrajectory::Result::GOAL_TOLERANCE_VIOLATED)
    << "a rejected cancel must not be reported as a successful cancellation";
}

TEST_F(DualActionTest, CancelLosingTheRaceReportsTheRealOutcome)
{
  Harness::Extras extras;
  extras.cancel_timeout = 2.0;
  Harness h(false, false, 0ms, 300ms, extras);
  ASSERT_TRUE(h.ready());
  h.sim_->ignore_cancel = true;

  auto handle = h.sendGoal();
  ASSERT_NE(handle, nullptr);
  std::this_thread::sleep_for(50ms);
  ASSERT_TRUE(h.cancelGoal(handle));

  rclcpp_action::ResultCode code{};
  ASSERT_TRUE(h.waitForResult(handle, code, 8s));
  EXPECT_EQ(code, rclcpp_action::ResultCode::SUCCEEDED)
    << "a trajectory that actually completed must not be relabelled as canceled";
}

TEST_F(DualActionTest, RepeatedCancelRequestsTerminateTheGoalOnce)
{
  Harness::Extras extras;
  extras.cancel_timeout = 1.0;
  Harness h(false, false, 0ms, 10s, extras);
  ASSERT_TRUE(h.ready());
  h.sim_->answer_cancel_with_result = false;

  auto handle = h.sendGoal();
  ASSERT_NE(handle, nullptr);
  std::this_thread::sleep_for(200ms);
  ASSERT_TRUE(h.cancelGoal(handle));
  ASSERT_TRUE(h.cancelGoal(handle));
  ASSERT_TRUE(h.cancelGoal(handle));

  rclcpp_action::ResultCode code{};
  ASSERT_TRUE(h.waitForResult(handle, code, 8s));
  EXPECT_EQ(code, rclcpp_action::ResultCode::ABORTED)
    << "repeated cancels still never proved that the trajectory stopped";
}

// ---------------------------------------------------------------------------
// Command ordering and concurrency
// ---------------------------------------------------------------------------

TEST_F(DualActionTest, SecondGoalWhileOneIsActiveIsRejected)
{
  Harness h(false, false, 0ms, 3s);
  ASSERT_TRUE(h.ready());
  auto first = h.sendGoal();
  ASSERT_NE(first, nullptr);

  EXPECT_EQ(h.sendGoal(), nullptr)
    << "a second trajectory must be refused while one is active";

  ASSERT_TRUE(h.cancelGoal(first));
  rclcpp_action::ResultCode code{};
  ASSERT_TRUE(h.waitForResult(first, code, 5s));
}

TEST_F(DualActionTest, AStopIsNeverOvertakenByTheMotionItBelongsTo)
{
  // Cancel as early as possible: whatever the interleaving between the Gazebo
  // acceptance and the cancel, a real motion command must never be published
  // after the stop request that belongs to the same goal.
  Harness h(true, true, 0ms, 2s);
  ASSERT_TRUE(h.ready());
  h.setFeedbackPositions(kGoalPositions);

  auto handle = h.sendGoal();
  ASSERT_NE(handle, nullptr);
  ASSERT_TRUE(h.cancelGoal(handle));
  rclcpp_action::ResultCode code{};
  ASSERT_TRUE(h.waitForResult(handle, code, 5s));

  const auto commands = h.realCommands();
  std::size_t first_stop = commands.size();
  std::size_t last_motion = commands.size();
  for (std::size_t i = 0; i < commands.size(); ++i) {
    if (commands[i] == "stop" && first_stop == commands.size()) {
      first_stop = i;
    }
    if (commands[i] == "motion") {
      last_motion = i;
    }
  }
  ASSERT_NE(first_stop, commands.size())
    << "the cancel must publish a real stop request";
  if (last_motion != commands.size()) {
    EXPECT_LT(last_motion, first_stop)
      << "a real motion command was published after the stop request for the same goal";
  }
}

// ---------------------------------------------------------------------------
// Recovery authorization and diagnostics
// ---------------------------------------------------------------------------

TEST_F(DualActionTest, RecoveryIsDisabledUnlessExplicitlyEnabled)
{
  Harness::Extras extras;
  extras.fault_recovery_enabled = false;
  Harness h(true, true, 0ms, 200ms, extras);
  ASSERT_TRUE(h.ready());
  h.setFaultMode(Harness::kEmc);
  std::this_thread::sleep_for(250ms);

  std::string message;
  EXPECT_FALSE(h.callRecovery(message));
  EXPECT_NE(message.find("disabled"), std::string::npos) << message;
}

TEST_F(DualActionTest, DiagnosticStateSeparatesSimulationAndRealOutcome)
{
  Harness h(true, false);
  ASSERT_TRUE(h.ready());
  h.setFeedbackPositions(kGoalPositions);

  auto handle = h.sendGoal();
  ASSERT_NE(handle, nullptr);
  rclcpp_action::ResultCode code{};
  ASSERT_TRUE(h.waitForResult(handle, code, 5s));
  ASSERT_EQ(code, rclcpp_action::ResultCode::SUCCEEDED);

  EXPECT_TRUE(h.waitForMotionStateContaining("sim=succeeded"))
    << h.latestMotionState();
  EXPECT_TRUE(h.waitForMotionStateContaining("real=unconfirmed"))
    << h.latestMotionState();
  EXPECT_TRUE(h.waitForMotionStateContaining("state=awaiting_confirmation"))
    << h.latestMotionState();
  EXPECT_TRUE(h.waitForMotionStateContaining("latched=1"))
    << h.latestMotionState();
}

// ---------------------------------------------------------------------------
// Durable interlock state
// ---------------------------------------------------------------------------

TEST_F(DualActionTest, AUsableStateFileIsReportedAsReady)
{
  Harness h(true, false);
  ASSERT_TRUE(h.ready());
  h.setFaultMode(Harness::kError);
  std::this_thread::sleep_for(250ms);

  EXPECT_TRUE(h.waitForMotionStateContaining("persistence=ok", 2s))
    << "the durable record must be reported as usable: " << h.latestMotionState();
}

TEST_F(DualActionTest, RequiredPersistenceRefusesRealMotionWhenTheStateFileIsUnusable)
{
  // real_persistence_required is true, but the state file cannot be written.
  // Publishing a real trajectory here would create a motion that a restart
  // could not remember, so it must be refused.
  Harness::Extras extras;
  extras.fault_latch_file = kUnusableLatchPath;
  extras.real_persistence_required = true;
  Harness h(true, false, 0ms, 200ms, extras);
  ASSERT_TRUE(h.ready());
  h.setFeedbackPositions(kGoalPositions);

  EXPECT_EQ(h.sendGoal(), nullptr)
    << "a real goal was accepted although its durable record cannot be written";
  EXPECT_TRUE(h.realCommands().empty())
    << "a real trajectory was published without a durable interlock record";

  // The failure must be visible, not silent.
  h.setFaultMode(Harness::kError);
  std::this_thread::sleep_for(250ms);
  EXPECT_TRUE(h.waitForMotionStateContaining("persistence=unavailable", 2s))
    << h.latestMotionState();
}

TEST_F(DualActionTest, OptionalPersistenceStillAllowsRealMotion)
{
  // The same unusable path, but durability was explicitly declared optional, so
  // the real channel keeps working (with a loud error) instead of stopping.
  Harness::Extras extras;
  extras.fault_latch_file = kUnusableLatchPath;
  extras.real_persistence_required = false;
  Harness h(true, false, 0ms, 200ms, extras);
  ASSERT_TRUE(h.ready());
  h.setFeedbackPositions(kGoalPositions);

  auto handle = h.sendGoal();
  ASSERT_NE(handle, nullptr)
    << "durability was declared optional, so the goal must still be accepted";
  rclcpp_action::ResultCode code{};
  ASSERT_TRUE(h.waitForResult(handle, code, 5s));
  EXPECT_EQ(code, rclcpp_action::ResultCode::SUCCEEDED);

  const auto commands = h.realCommands();
  EXPECT_NE(
    std::find(commands.begin(), commands.end(), std::string("motion")), commands.end())
    << "the real trajectory should have been published";
  EXPECT_FALSE(h.markerExists())
    << "no durable record can exist on an unusable path";
}

TEST_F(DualActionTest, ARuntimeWriteFailureBlocksRealMotion)
{
  if (::geteuid() == 0) {
    GTEST_SKIP() << "root bypasses directory permissions, so this case cannot be exercised";
  }
  // The record works at startup, then the directory becomes unwritable.  A latch
  // that cannot be recorded must stop the real channel instead of continuing
  // with a promise it cannot keep.
  const std::string directory =
    "/tmp/duco_dual_action_test_ro_" + std::to_string(::getpid());
  Harness::Extras extras;
  extras.fault_latch_file = directory + "/marker.state";
  Harness h(true, false, 0ms, 200ms, extras);
  ASSERT_TRUE(h.ready());

  ASSERT_EQ(::chmod(directory.c_str(), 0500), 0);
  h.setFaultMode(Harness::kError);
  EXPECT_TRUE(h.waitForMotionStateContaining("persistence=unavailable", 3s))
    << "the failed write must be reported, not swallowed: " << h.latestMotionState();

  // While the record cannot be written, real motion stays refused.
  EXPECT_EQ(h.sendGoal(), nullptr)
    << "a real goal was accepted although the durable record cannot be written";
  EXPECT_TRUE(h.realCommands().empty()) << "a real trajectory was still published";

  // Once the operator restores the directory, the channel becomes usable again.
  ASSERT_EQ(::chmod(directory.c_str(), 0700), 0);
  h.setFaultMode(Harness::kNoFault);
  std::this_thread::sleep_for(250ms);
  std::string message;
  ASSERT_TRUE(waitForRecovery(h, message)) << message;

  h.setFeedbackPositions(kGoalPositions);
  auto handle = h.sendGoal();
  ASSERT_NE(handle, nullptr) << "the channel did not recover after the directory was restored";
  rclcpp_action::ResultCode code{};
  ASSERT_TRUE(h.waitForResult(handle, code, 5s));
  ASSERT_TRUE(h.callRecovery(message)) << message;

  ::unlink((directory + "/marker.state").c_str());
  ::unlink((directory + "/marker.state.tmp").c_str());
  ::rmdir(directory.c_str());
}

TEST_F(DualActionTest, TheDurableRecordExistsWhenTheResultIsDelivered)
{
  // The task slot release and the durable record belong to one transaction: once
  // the client sees the result, the record must already be on disk, otherwise a
  // crash in between would erase the memory of an unconfirmed real motion.
  Harness h(true, false, 0ms, 200ms);
  ASSERT_TRUE(h.ready());
  h.setFeedbackPositions(kGoalPositions);
  ASSERT_FALSE(h.markerExists()) << "the harness must start without a record";

  auto handle = h.sendGoal();
  ASSERT_NE(handle, nullptr);
  rclcpp_action::ResultCode code{};
  ASSERT_TRUE(h.waitForResult(handle, code, 5s));
  ASSERT_EQ(code, rclcpp_action::ResultCode::SUCCEEDED)
    << "the simulation result is still reported (wait_for_real is false)";

  EXPECT_TRUE(h.markerExists())
    << "the result was delivered before the unconfirmed motion was recorded";

  std::string message;
  ASSERT_TRUE(h.callRecovery(message)) << message;
  EXPECT_FALSE(h.markerExists()) << "the operator recovery must drop the record";
}

TEST_F(DualActionTest, UnconfirmedFinishAndANewGoalAreNeverInconsistent)
{
  // "The task ended" and "the next task arrives" interleave as fast as the test
  // can make them.  The interlock must never be observable as released while the
  // durable record of the previous motion still stands.
  Harness h(true, false, 0ms, 200ms);
  ASSERT_TRUE(h.ready());
  h.setFeedbackPositions(kGoalPositions);

  for (int iteration = 0; iteration < 3; ++iteration) {
    auto handle = h.sendGoal();
    ASSERT_NE(handle, nullptr) << "iteration " << iteration;
    rclcpp_action::ResultCode code{};
    ASSERT_TRUE(h.waitForResult(handle, code, 5s)) << "iteration " << iteration;
    ASSERT_EQ(code, rclcpp_action::ResultCode::SUCCEEDED);

    // Immediately, without any sleep: the slot is free but the channel is not.
    EXPECT_EQ(h.sendGoal(), nullptr)
      << "iteration " << iteration << ": a goal was accepted while the previous real motion was "
      << "still unconfirmed";
    EXPECT_TRUE(h.markerExists()) << "iteration " << iteration;

    std::string message;
    ASSERT_TRUE(h.callRecovery(message)) << "iteration " << iteration << ": " << message;
    EXPECT_FALSE(h.markerExists()) << "iteration " << iteration;

    auto after_recovery = h.sendGoal();
    ASSERT_NE(after_recovery, nullptr)
      << "iteration " << iteration << ": the channel stayed blocked after a recovery";
    rclcpp_action::ResultCode second_code{};
    ASSERT_TRUE(h.waitForResult(after_recovery, second_code, 5s));
    ASSERT_TRUE(h.callRecovery(message)) << message;
  }
}

TEST_F(DualActionTest, ARestartWithARecordLatchesUntilAnOperatorRecovers)
{
  Harness::Extras extras;
  extras.fault_latch_file =
    "/tmp/duco_dual_action_test_restart_" + std::to_string(::getpid()) + ".state";
  {
    std::ofstream record(extras.fault_latch_file, std::ios::trunc);
    record << "goal-previous executing: interrupted run\n";
  }

  Harness h(true, false, 0ms, 200ms, extras);
  ASSERT_TRUE(h.ready());
  h.setFeedbackPositions(kGoalPositions);

  EXPECT_EQ(h.sendGoal(), nullptr)
    << "a restart must not silently clear an unconfirmed real motion";

  std::string message;
  ASSERT_TRUE(h.callRecovery(message)) << message;
  EXPECT_FALSE(h.markerExists());

  auto handle = h.sendGoal();
  ASSERT_NE(handle, nullptr) << "the channel must be usable after a recovery";
}

TEST_F(DualActionTest, AnEmptyRecordStillLatchesOnRestart)
{
  // A record that exists at all means "not confirmed"; an empty one must not be
  // read as a clean state.
  Harness::Extras extras;
  extras.fault_latch_file =
    "/tmp/duco_dual_action_test_empty_" + std::to_string(::getpid()) + ".state";
  {
    std::ofstream record(extras.fault_latch_file, std::ios::trunc);
  }

  Harness h(true, false, 0ms, 200ms, extras);
  ASSERT_TRUE(h.ready());
  EXPECT_EQ(h.sendGoal(), nullptr) << "an empty record must still block real motion";
}

TEST_F(DualActionTest, ASuccessfulRecoveryNeverErasesAConcurrentFault)
{
  // The fault is put on the wire *before* the recovery request, so whichever of
  // the two callbacks wins the lock, the fault must be latched afterwards.  A
  // recovery that cleared the state it never observed would leave the channel
  // looking healthy while the robot reported a fault.
  Harness h(true, false, 0ms, 200ms);
  ASSERT_TRUE(h.ready());

  std::string message;
  ASSERT_TRUE(waitForRecovery(h, message)) << message;

  for (int iteration = 0; iteration < 8; ++iteration) {
    ASSERT_TRUE(waitForRecovery(h, message)) << "iteration " << iteration << ": " << message;

    h.setFaultMode(Harness::kError);
    h.publishStatus();  // a fault is definitely in flight now
    h.callRecovery(message);
    h.setFaultMode(Harness::kNoFault);
    std::this_thread::sleep_for(400ms);

    EXPECT_TRUE(h.waitForMotionStateContaining("latched=1", 500ms))
      << "iteration " << iteration
      << ": the fault reported before the recovery was erased (" << h.latestMotionState() << ")";

    // Re-arm for the next iteration.  With the cell healthy again a recovery
    // must be accepted.
    bool cleared = false;
    for (int attempt = 0; attempt < 40 && !cleared; ++attempt) {
      cleared = waitForRecovery(h, message);
    }
    ASSERT_TRUE(cleared)
      << "iteration " << iteration << ": the channel never became recoverable again: " << message;
  }
}

