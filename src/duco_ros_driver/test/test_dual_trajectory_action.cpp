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
#include <chrono>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
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
  std::atomic<int> accepted_count{0};
  std::atomic<int> cancel_requests{0};

private:
  void onAccepted(const std::shared_ptr<SimGoalHandle> handle)
  {
    ++accepted_count;
    std::thread([this, handle]() {
      const auto deadline = std::chrono::steady_clock::now() + execute_duration;
      while (std::chrono::steady_clock::now() < deadline) {
        if (handle->is_canceling()) {
          if (answer_cancel_with_result.load()) {
            handle->canceled(std::make_shared<FollowJointTrajectory::Result>());
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
class Harness
{
public:
  Harness(
    bool publish_real, bool wait_for_real,
    std::chrono::milliseconds accept_delay = 0ms,
    std::chrono::milliseconds execute_duration = 200ms)
  {
    const int id = ++g_harness_id;
    const std::string suffix = "_t" + std::to_string(id);
    front_action_ = "arm_1_controller/follow_joint_trajectory" + suffix;
    sim_action_ = "sim_arm_1_controller/follow_joint_trajectory" + suffix;

    sim_ = std::make_shared<MockSimServer>(sim_action_, "mock_sim_server" + suffix);
    sim_->accept_delay = accept_delay;
    sim_->execute_duration = execute_duration;

    auto options = rclcpp::NodeOptions();
    options.arguments({"--ros-args", "-r", "__node:=dual_action_proxy" + suffix});
    options.parameter_overrides({
      rclcpp::Parameter("front_action_name", front_action_),
      rclcpp::Parameter("sim_action_name", sim_action_),
      rclcpp::Parameter("real_command_topic", std::string("joint_path_command") + suffix),
      rclcpp::Parameter("publish_real", publish_real),
      rclcpp::Parameter("wait_for_real", wait_for_real),
      rclcpp::Parameter("real_feedback_topic", std::string("real_feedback_states") + suffix),
      rclcpp::Parameter("real_status_topic", std::string("/duco_cobot/robot_state") + suffix),
      rclcpp::Parameter("real_goal_tolerance", 0.02),
      rclcpp::Parameter("real_velocity_tolerance", 0.05),
      rclcpp::Parameter("acceptance_timeout", 0.6),
      rclcpp::Parameter("sim_execution_timeout", 3.0),
      rclcpp::Parameter("cancel_timeout", 0.5),
      rclcpp::Parameter("real_execution_timeout", 3.0),
      rclcpp::Parameter("real_feedback_timeout", 0.3),
      rclcpp::Parameter("real_status_timeout", 0.3),
      rclcpp::Parameter("clear_fault_service", std::string("clear_fault") + suffix),
      // Keep the tests self-contained: no marker file across runs.
      rclcpp::Parameter("fault_latch_file", std::string("")),
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
  }

  bool ready() const { return ready_; }

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

  void setHealthy(bool healthy) { healthy_.store(healthy); }

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
    msg.robot_error = healthy_.load() ? 0 : 42;
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

  std::shared_ptr<DualTrajectoryAction> proxy_;
  std::shared_ptr<MockSimServer> sim_;

private:
  std::string front_action_;
  std::string sim_action_;
  rclcpp::Node::SharedPtr helper_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr client_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr sim_client_;
  rclcpp::Publisher<duco_msg::msg::DucoRobotState>::SharedPtr status_pub_;
  rclcpp::Publisher<FollowJointTrajectory::Feedback>::SharedPtr feedback_pub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr recovery_;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread spin_thread_;
  std::thread pump_thread_;
  std::atomic<bool> pumping_{false};
  std::atomic<bool> healthy_{true};
  std::mutex feedback_mutex_;
  std::vector<double> feedback_positions_{kGoalPositions};
  bool ready_{false};
};

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

TEST_F(DualActionTest, CancelTimeoutStillReportsCanceled)
{
  // Gazebo accepts the cancellation but never reports a terminal state, so the
  // proxy has to fall back on its cancel timeout.
  Harness h(false, false, 0ms, 10s);
  ASSERT_TRUE(h.ready());
  h.sim_->answer_cancel_with_result = false;

  auto handle = h.sendGoal();
  ASSERT_NE(handle, nullptr);
  std::this_thread::sleep_for(300ms);
  ASSERT_TRUE(h.cancelGoal(handle));

  rclcpp_action::ResultCode code{};
  ASSERT_TRUE(h.waitForResult(handle, code, 8s));
  EXPECT_EQ(code, rclcpp_action::ResultCode::CANCELED);
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

TEST_F(DualActionTest, UnhealthyRealRobotAbortsTheGoal)
{
  Harness h(true, true);
  ASSERT_TRUE(h.ready());
  h.setHealthy(false);
  std::this_thread::sleep_for(100ms);

  auto handle = h.sendGoal();
  ASSERT_NE(handle, nullptr);

  rclcpp_action::ResultCode code{};
  ASSERT_TRUE(h.waitForResult(handle, code, 5s));
  EXPECT_EQ(code, rclcpp_action::ResultCode::ABORTED);
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
  EXPECT_EQ(code, rclcpp_action::ResultCode::CANCELED);

  // The real command was published and its stop was never confirmed, so the
  // next goal must be refused rather than silently accepted.
  auto second = h.sendGoal();
  EXPECT_EQ(second, nullptr)
    << "a goal was accepted after an unconfirmed real stop";
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
