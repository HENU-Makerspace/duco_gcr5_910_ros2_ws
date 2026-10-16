// DualTrajectoryAction.hpp
//
// Proxy action server that forwards a MoveIt trajectory to Gazebo and, once
// Gazebo accepts it, to the real DUCO arm.  Split out of the .cpp so that
// integration tests can instantiate the node directly.

#ifndef DUCO_ROS_DRIVER_DUAL_TRAJECTORY_ACTION_HPP
#define DUCO_ROS_DRIVER_DUAL_TRAJECTORY_ACTION_HPP

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <functional>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <vector>

#include <builtin_interfaces/msg/duration.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "duco_msg/msg/duco_robot_state.hpp"

#include "duco_ros_driver/RealFeedbackGate.hpp"
#include "duco_ros_driver/RealMotionInterlock.hpp"
#include "duco_ros_driver/TrajectoryValidation.hpp"

class DualTrajectoryAction : public rclcpp::Node
{
public:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using GoalHandle = rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;
  using SimGoalHandle = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;
  using GoalUUID = rclcpp_action::GoalUUID;

  explicit DualTrajectoryAction(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("DualTrajectoryAction", options)
  {
    front_action_name_ = declare_parameter<std::string>(
      "front_action_name", "arm_1_controller/follow_joint_trajectory");
    sim_action_name_ = declare_parameter<std::string>(
      "sim_action_name", "sim_arm_1_controller/follow_joint_trajectory");
    real_command_topic_ = declare_parameter<std::string>("real_command_topic", "joint_path_command");
    // Defaults to false: the real channel must be enabled explicitly, so a
    // plain `ros2 run` can never command the physical arm by accident.
    publish_real_ = declare_parameter<bool>("publish_real", false);
    wait_for_real_ = declare_parameter<bool>("wait_for_real", false);
    publish_real_stop_request_ = declare_parameter<bool>("publish_real_stop_request", true);
    real_feedback_topic_ = declare_parameter<std::string>(
      "real_feedback_topic", "real_feedback_states");
    real_status_topic_ = declare_parameter<std::string>(
      "real_status_topic", "/duco_cobot/robot_state");
    real_goal_tolerance_ = declare_parameter<double>("real_goal_tolerance", 0.02);
    real_velocity_tolerance_ = declare_parameter<double>("real_velocity_tolerance", 0.05);
    // The vendor status and feedback topics are published from the same 20 ms
    // loop (~50 Hz), so 1.0 s tolerates ~50 consecutive missed samples before
    // the data is treated as stale.  Tune these down for stricter monitoring.
    real_feedback_max_age_ = declare_parameter<double>("real_feedback_max_age", 1.0);
    real_status_max_age_ = declare_parameter<double>("real_status_max_age", 1.0);
    // When true (recommended) an absent velocity array fails completion instead
    // of being read as "the arm is not moving".
    real_require_velocity_ = declare_parameter<bool>("real_require_velocity", true);
    // Diagnostics for the upper layer: separates the simulation result from the
    // real motion result.  Empty disables the publisher.
    real_motion_state_topic_ = declare_parameter<std::string>(
      "real_motion_state_topic", "dual_trajectory/real_motion_state");
    // The recovery service is disabled by default.  Enabling it is an explicit
    // operator action (a ROS 2 service cannot be authenticated on its own), and
    // it is the only way to release the real motion interlock.
    fault_recovery_enabled_ = declare_parameter<bool>("fault_recovery_enabled", false);
    // A fault reported by the robot (E-stop, collision, error code) forbids real
    // motion until an operator recovers it, even when no goal is active.
    latch_on_real_fault_ = declare_parameter<bool>("latch_on_real_fault", true);
    // Controller state values that must be treated as a fault.  Empty by
    // default: the vendor does not document the enum, so nothing is assumed.
    // DucoRobotStatus publishes a "stop" event when the controller drops to
    // state 5 while moving, so `[5]` is a reasonable starting point after
    // confirming it on the cell.
    real_unhealthy_robot_states_ = declare_parameter<std::vector<int64_t>>(
      "real_unhealthy_robot_states", std::vector<int64_t>{});
    // DucoDriver::setRobotJointsByMoveIt() unconditionally drops the first path
    // point, so a trajectory with fewer than two points is enqueued as an empty
    // path.  Refuse to forward such a command to the real controller.
    min_real_trajectory_points_ = declare_parameter<int>("min_real_trajectory_points", 2);
    clear_fault_service_ = declare_parameter<std::string>("clear_fault_service", "clear_fault");
    // When non-empty, a marker file records that a real trajectory was published
    // without confirmed completion.  A restart with the marker present starts
    // latched, because a process-local flag cannot prove that an unconfirmed
    // real motion has gone away.
    fault_latch_file_ = declare_parameter<std::string>(
      "fault_latch_file", "/tmp/duco_dual_trajectory_fault.state");
    real_execution_timeout_ = declare_parameter<double>("real_execution_timeout", 300.0);
    wait_for_sim_seconds_ = declare_parameter<double>("wait_for_sim_seconds", 5.0);
    acceptance_timeout_ = declare_parameter<double>("acceptance_timeout", wait_for_sim_seconds_);
    sim_execution_timeout_ = declare_parameter<double>("sim_execution_timeout", 300.0);
    cancel_timeout_ = declare_parameter<double>("cancel_timeout", 5.0);
    sim_time_scale_ = declare_parameter<double>("sim_time_scale", 1.0);
    enforce_expected_joint_names_ = declare_parameter<bool>("enforce_expected_joint_names", true);
    expected_joint_names_ = declare_parameter<std::vector<std::string>>(
      "expected_joint_names",
      {"arm_1_joint_1", "arm_1_joint_2", "arm_1_joint_3", "arm_1_joint_4",
       "arm_1_joint_5", "arm_1_joint_6"});
    joint_position_min_ = declare_parameter<std::vector<double>>(
      "joint_position_min", std::vector<double>{});
    joint_position_max_ = declare_parameter<std::vector<double>>(
      "joint_position_max", std::vector<double>{});

    validateConfiguration();

    real_command_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
      real_command_topic_, 10);
    if (!real_motion_state_topic_.empty()) {
      // Diagnostics only; nothing consumes it for control decisions.
      real_motion_state_pub_ = create_publisher<std_msgs::msg::String>(
        real_motion_state_topic_, 10);
    }
    sim_action_client_ = rclcpp_action::create_client<FollowJointTrajectory>(this, sim_action_name_);
    real_feedback_sub_ = create_subscription<FollowJointTrajectory::Feedback>(
      real_feedback_topic_, 100,
      std::bind(&DualTrajectoryAction::realFeedbackCallback, this, std::placeholders::_1));
    real_status_sub_ = create_subscription<duco_msg::msg::DucoRobotState>(
      real_status_topic_, 30,
      std::bind(&DualTrajectoryAction::realStatusCallback, this, std::placeholders::_1));
    watchdog_timer_ = create_wall_timer(
      std::chrono::milliseconds(100), std::bind(&DualTrajectoryAction::watchdogCallback, this));

    action_server_ = rclcpp_action::create_server<FollowJointTrajectory>(
      this,
      front_action_name_,
      std::bind(&DualTrajectoryAction::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&DualTrajectoryAction::handle_cancel, this, std::placeholders::_1),
      std::bind(&DualTrajectoryAction::handle_accepted, this, std::placeholders::_1));

    clear_fault_srv_ = create_service<std_srvs::srv::Trigger>(
      clear_fault_service_,
      std::bind(
        &DualTrajectoryAction::clearFaultCallback, this,
        std::placeholders::_1, std::placeholders::_2));

    restoreLatchedFault();

    RCLCPP_INFO(
      get_logger(),
      "Dual trajectory action ready: MoveIt [%s] -> Gazebo [%s] + real topic [%s]",
      front_action_name_.c_str(), sim_action_name_.c_str(), real_command_topic_.c_str());
    RCLCPP_INFO(
      get_logger(),
      "Real completion wait: %s, feedback topic: [%s], tolerance: %.4f rad, timeout: %.1f s",
      (wait_for_real_ && publish_real_) ? "enabled" : "disabled",
      real_feedback_topic_.c_str(), real_goal_tolerance_, real_execution_timeout_);
  }

private:
  enum class TaskState { Idle, WaitingForSim, Executing, Canceling, Succeeded, Failed, Canceled };

  static const char * stateName(TaskState state)
  {
    switch (state) {
      case TaskState::Idle: return "idle";
      case TaskState::WaitingForSim: return "waiting_for_sim";
      case TaskState::Executing: return "executing";
      case TaskState::Canceling: return "canceling";
      case TaskState::Succeeded: return "succeeded";
      case TaskState::Failed: return "failed";
      case TaskState::Canceled: return "canceled";
    }
    return "unknown";
  }

  static std::string goalIdText(const GoalUUID & id)
  {
    std::ostringstream stream;
    stream << std::hex << std::setfill('0');
    for (const auto byte : id) {
      stream << std::setw(2) << static_cast<unsigned int>(byte);
    }
    return stream.str();
  }

  void setStateLocked(TaskState state, const std::string & goal_id)
  {
    if (task_state_ != state) {
      RCLCPP_INFO(
        get_logger(), "Task %s state: %s -> %s", goal_id.c_str(),
        stateName(task_state_), stateName(state));
      task_state_ = state;
    }
  }

  /**
   * \brief The real motion channel is modelled by RealMotionInterlock.
   *
   * The interlock lives *outside* the action goal on purpose: finishing a goal
   * is not evidence that the arm stopped, so the permission to send the next
   * real trajectory is tracked separately from active_goal_ and only an explicit
   * operator recovery releases it.
   */

  /**
   * \brief One outstanding Gazebo goal request.
   *
   * The record is created when the goal is sent, before Gazebo answers, so a
   * response that arrives after the front goal already terminated can still be
   * correlated by UUID.  Previously such a response was dropped by an early
   * return, leaving a simulation goal running with no handle to cancel it.
   */
  struct SimRequest {
    GoalUUID front_goal_id{};
    std::shared_ptr<SimGoalHandle> handle;
    bool accepted{false};
    bool abandoned{false};
    bool cancel_requested{false};
    /// Set when Gazebo answered the cancel with an empty goals_canceling list.
    bool cancel_rejected{false};
    std::chrono::steady_clock::time_point created_at{};
    std::chrono::steady_clock::time_point abandoned_at{};
  };

  void validateConfiguration()
  {
    const auto requirePositiveFinite = [](double value, const char * name) {
      if (!std::isfinite(value) || value <= 0.0) {
        throw std::invalid_argument(std::string(name) + " must be finite and > 0");
      }
    };
    const auto requireNonNegativeFinite = [](double value, const char * name) {
      if (!std::isfinite(value) || value < 0.0) {
        throw std::invalid_argument(std::string(name) + " must be finite and >= 0");
      }
    };

    requirePositiveFinite(sim_time_scale_, "sim_time_scale");
    requirePositiveFinite(acceptance_timeout_, "acceptance_timeout");
    requirePositiveFinite(sim_execution_timeout_, "sim_execution_timeout");
    requirePositiveFinite(cancel_timeout_, "cancel_timeout");
    requirePositiveFinite(real_execution_timeout_, "real_execution_timeout");
    requireNonNegativeFinite(real_goal_tolerance_, "real_goal_tolerance");
    requireNonNegativeFinite(real_velocity_tolerance_, "real_velocity_tolerance");

    if (enforce_expected_joint_names_ && expected_joint_names_.empty()) {
      throw std::invalid_argument("expected_joint_names cannot be empty when enforcement is enabled");
    }
    std::unordered_set<std::string> expected_set;
    for (const auto & name : expected_joint_names_) {
      if (name.empty() || !expected_set.insert(name).second) {
        throw std::invalid_argument("expected_joint_names must contain unique non-empty names");
      }
    }
    if (joint_position_min_.size() != joint_position_max_.size() &&
      (!joint_position_min_.empty() || !joint_position_max_.empty()))
    {
      throw std::invalid_argument("joint_position_min and joint_position_max must have equal sizes");
    }
    if (!joint_position_min_.empty() && joint_position_min_.size() != expected_joint_names_.size()) {
      throw std::invalid_argument("joint position limits must match expected_joint_names size");
    }
    for (size_t i = 0; i < joint_position_min_.size(); ++i) {
      if (!std::isfinite(joint_position_min_[i]) || !std::isfinite(joint_position_max_[i]) ||
        joint_position_min_[i] > joint_position_max_[i])
      {
        throw std::invalid_argument("joint position limits must be finite and min <= max");
      }
    }

    requirePositiveFinite(real_feedback_max_age_, "real_feedback_max_age");
    requirePositiveFinite(real_status_max_age_, "real_status_max_age");
    if (min_real_trajectory_points_ < 1) {
      throw std::invalid_argument("min_real_trajectory_points must be >= 1");
    }
    // Waiting for a real completion that can never be commanded would let the
    // goal hang until the simulation timeout instead of failing loudly.
    if (wait_for_real_ && !publish_real_) {
      throw std::invalid_argument(
              "wait_for_real=true requires publish_real=true: there is nothing to wait for "
              "when the real channel is disabled");
    }
    {
      std::unordered_set<int64_t> unhealthy_states;
      for (const auto state : real_unhealthy_robot_states_) {
        if (!unhealthy_states.insert(state).second) {
          throw std::invalid_argument("real_unhealthy_robot_states contains a duplicate value");
        }
      }
    }

    trajectory_policy_.enforce_expected_joint_names = enforce_expected_joint_names_;
    trajectory_policy_.expected_joint_names = expected_joint_names_;
    trajectory_policy_.joint_position_min = joint_position_min_;
    trajectory_policy_.joint_position_max = joint_position_max_;
  }

  // ---------------------------------------------------------------------------
  // Simulation goal ledger.  All helpers require state_mutex_ to be held.
  // ---------------------------------------------------------------------------

  SimRequest * findSimRequestLocked(const GoalUUID & front_goal_id)
  {
    for (auto & request : sim_requests_) {
      if (request.front_goal_id == front_goal_id) {
        return &request;
      }
    }
    return nullptr;
  }

  void eraseSimRequestLocked(const GoalUUID & front_goal_id)
  {
    sim_requests_.erase(
      std::remove_if(
        sim_requests_.begin(), sim_requests_.end(),
        [&front_goal_id](const SimRequest & request) {
          return request.front_goal_id == front_goal_id;
        }),
      sim_requests_.end());
  }

  // ---------------------------------------------------------------------------
  // Fault latch and recovery
  // ---------------------------------------------------------------------------

  /**
   * \brief Restore the interlock from a marker left by a previous run.
   *
   * Only the real channel cares: in simulation-only mode no real trajectory can
   * be published, so a stale marker must not block a pure simulation session.
   */
  void restoreLatchedFault()
  {
    if (!publish_real_ || fault_latch_file_.empty()) {
      return;
    }
    std::ifstream marker(fault_latch_file_);
    if (!marker.good()) {
      return;
    }
    std::string detail;
    std::getline(marker, detail);
    std::lock_guard<std::mutex> lock(state_mutex_);
    real_interlock_.onUncertain(
      0, "startup",
      "a previous run published a real trajectory whose completion was never confirmed"
      " (marker: " + (detail.empty() ? std::string("unknown") : detail) + ")");
    RCLCPP_ERROR(
      get_logger(),
      "Real motion interlock restored from [%s]: %s. New real trajectories stay refused "
      "until an operator recovers the channel.", fault_latch_file_.c_str(),
      real_interlock_.reason().c_str());
    publishInterlockStateLocked();
  }

  /// Latch because the real motion state cannot be established.
  void latchUncertainLocked(const std::string & reason, const std::string & goal_id)
  {
    if (!real_interlock_.onUncertain(command_generation_, goal_id, reason)) {
      // Something at least as severe is already latched; never downgrade it.
      return;
    }
    RCLCPP_ERROR(
      get_logger(),
      "Real motion state is uncertain (goal %s): %s. New real trajectories are refused until "
      "an operator calls the '%s' service.", goal_id.c_str(), reason.c_str(),
      clear_fault_service_.c_str());
    writeFaultMarkerLocked();
    publishInterlockStateLocked();
  }

  /// Latch because the robot itself reported a fault.
  void latchRobotFaultLocked(const std::string & reason, const std::string & goal_id)
  {
    if (!real_interlock_.onFault(goal_id, reason)) {
      return;
    }
    RCLCPP_ERROR(
      get_logger(),
      "Real robot fault (goal %s): %s. New real trajectories are refused until an operator "
      "calls the '%s' service.", goal_id.c_str(), reason.c_str(), clear_fault_service_.c_str());
    writeFaultMarkerLocked();
    publishInterlockStateLocked();
  }

  void writeFaultMarkerLocked() const
  {
    if (fault_latch_file_.empty()) {
      return;
    }
    std::ofstream marker(fault_latch_file_, std::ios::trunc);
    if (!marker.good()) {
      RCLCPP_ERROR(get_logger(), "Unable to write fault marker [%s]", fault_latch_file_.c_str());
      return;
    }
    marker << (real_interlock_.goalId().empty() ? "-" : real_interlock_.goalId())
           << " "
           << duco_reliability::realMotionStateName(real_interlock_.state())
           << ": "
           << (real_interlock_.reason().empty() ? "-" : real_interlock_.reason())
           << "\n";
  }

  void removeFaultMarker() const
  {
    if (fault_latch_file_.empty()) {
      return;
    }
    std::remove(fault_latch_file_.c_str());
  }

  /**
   * \brief Publish the diagnostics that separate the simulation result from the
   * real motion result.  Caller must hold state_mutex_.
   */
  void publishInterlockStateLocked()
  {
    if (!real_motion_state_pub_) {
      return;
    }
    std::ostringstream out;
    out << "state=" << duco_reliability::realMotionStateName(real_interlock_.state())
        << ";latched=" << (real_interlock_.latched() ? 1 : 0)
        << ";goal=" << (real_interlock_.goalId().empty() ? "-" : real_interlock_.goalId())
        << ";sim=" << sim_outcome_
        << ";real=" << real_outcome_
        << ";robot_state=" << latest_real_robot_state_
        << ";reason=" << (real_interlock_.reason().empty() ? "-" : real_interlock_.reason());
    std_msgs::msg::String message;
    message.data = out.str();
    real_motion_state_pub_->publish(message);
  }

  /**
   * \brief Operator-driven recovery.
   *
   * Calling the service is the operator's attestation that the cell was
   * inspected.  Every observable condition must also hold.  Zero-velocity
   * feedback alone is deliberately not sufficient: the vendor driver keeps
   * re-publishing cached values after a disconnect.
   */
  void clearFaultCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    // A ROS 2 service cannot be authenticated, so the explicit enable flag is
    // the operator's authorization to release the real motion interlock.
    if (!fault_recovery_enabled_) {
      response->success = false;
      response->message =
        "fault recovery is disabled on this node; set fault_recovery_enabled:=true on the "
        "operator instance after inspecting the cell";
      RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
      return;
    }

    const auto now = std::chrono::steady_clock::now();
    std::string blocker;
    std::string cleared_from;
    std::string cleared_goal;
    std::string cleared_reason;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (!real_interlock_.latched() && !real_interlock_.blocksRealMotion()) {
        response->success = true;
        response->message = "no latched real robot fault";
        return;
      }
      if (active_goal_ || slot_reserved_) {
        blocker = "a trajectory goal is still active";
      } else if (!sim_requests_.empty()) {
        blocker = "a Gazebo goal is still outstanding";
      } else if (real_command_sent_ ||
        real_interlock_.state() == duco_reliability::RealMotionState::Executing)
      {
        blocker = "a real trajectory command is still outstanding";
      } else if (latest_real_status_time_ == std::chrono::steady_clock::time_point{}) {
        blocker = "no real robot status has been received";
      } else if (now < latest_real_status_time_ ||
        now - latest_real_status_time_ > secondsToSteady(real_status_max_age_))
      {
        blocker = "real robot status is stale";
      } else if (!latest_real_status_healthy_) {
        blocker = "real robot status reports a fault";
      }
      cleared_from = duco_reliability::realMotionStateName(real_interlock_.state());
      cleared_goal = real_interlock_.goalId();
      cleared_reason = real_interlock_.reason();
    }

    if (!blocker.empty()) {
      response->success = false;
      response->message = "refusing to clear the latched fault: " + blocker;
      RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
      return;
    }

    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      RCLCPP_WARN(
        get_logger(), "Operator cleared the real motion interlock (was %s, goal %s): %s",
        cleared_from.c_str(), cleared_goal.c_str(), cleared_reason.c_str());
      real_interlock_.onOperatorRecovery();
      real_faulted_ = false;
      real_fault_reason_.clear();
      real_completion_confirmed_ = false;
      real_outcome_ = "recovered";
      publishInterlockStateLocked();
      setStateLocked(TaskState::Idle, "recovery");
    }
    removeFaultMarker();

    response->success = true;
    response->message =
      "latched fault cleared. This confirms that the operator inspected the cell and that the "
      "observable status was healthy; it is not a vendor-confirmed safe stop.";
    RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
  }

  rclcpp_action::GoalResponse handle_goal(
    const GoalUUID & goal_id, std::shared_ptr<const FollowJointTrajectory::Goal> goal)
  {
    std::string error;
    if (!duco_reliability::validateTrajectory(goal->trajectory, trajectory_policy_, error)) {
      RCLCPP_ERROR(
        get_logger(), "Rejecting goal %s: invalid trajectory: %s",
        goalIdText(goal_id).c_str(), error.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (!std::isfinite(sim_time_scale_) || sim_time_scale_ <= 0.0) {
      RCLCPP_ERROR(get_logger(), "Rejecting goal %s: invalid sim_time_scale", goalIdText(goal_id).c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (!sim_action_client_->action_server_is_ready()) {
      RCLCPP_WARN(
        get_logger(), "Rejecting goal %s: Gazebo action server [%s] is not ready",
        goalIdText(goal_id).c_str(), sim_action_name_.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }

    std::lock_guard<std::mutex> lock(state_mutex_);
    // The real motion channel is gated independently of the action slot: a goal
    // that ended without a confirmed real completion keeps refusing the next
    // real trajectory even though active_goal_ has already been released.
    if (publish_real_ && real_interlock_.blocksRealMotion()) {
      RCLCPP_ERROR(
        get_logger(),
        "Rejecting goal %s: the real motion interlock is '%s' (%s). Confirm the cell is safe "
        "and call the '%s' service.",
        goalIdText(goal_id).c_str(),
        duco_reliability::realMotionStateName(real_interlock_.state()),
        real_interlock_.reason().c_str(), clear_fault_service_.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (!sim_requests_.empty()) {
      // A Gazebo goal from a previous request is still unaccounted for.  Its
      // execution state is unknown, so a new goal could conflict with it.
      RCLCPP_ERROR(
        get_logger(),
        "Rejecting goal %s: %zu stale Gazebo goal request(s) are still outstanding",
        goalIdText(goal_id).c_str(), sim_requests_.size());
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (slot_reserved_ || active_goal_) {
      RCLCPP_WARN(
        get_logger(), "Rejecting goal %s: another trajectory is active", goalIdText(goal_id).c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }
    slot_reserved_ = true;
    pending_goal_id_ = goal_id;
    reservation_time_ = std::chrono::steady_clock::now();
    RCLCPP_INFO(
      get_logger(), "Accepted goal request %s after validation (%zu points)",
      goalIdText(goal_id).c_str(), goal->trajectory.points.size());
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle> goal_handle)
  {
    std::shared_ptr<SimGoalHandle> sim_goal;
    bool stop_published = false;
    const GoalUUID front_id = goal_handle->get_goal_id();
    const std::string id = goalIdText(front_id);
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (active_goal_ != goal_handle || terminal_state_) {
        RCLCPP_DEBUG(get_logger(), "Ignoring cancel for inactive goal %s", id.c_str());
        return rclcpp_action::CancelResponse::ACCEPT;
      }
      if (task_state_ == TaskState::Canceling) {
        return rclcpp_action::CancelResponse::ACCEPT;
      }
      setStateLocked(TaskState::Canceling, id);
      cancel_deadline_ = std::chrono::steady_clock::now() + secondsToSteady(cancel_timeout_);
      auto * request = findSimRequestLocked(front_id);
      if (request != nullptr && request->handle && !request->cancel_requested) {
        request->cancel_requested = true;
        sim_goal = request->handle;
      }
      // The stop goes out under the same lock that guards the command
      // generation, so a motion command that races with this cancel can never
      // overtake the stop.
      stop_published = publishRealStopLocked(
        front_id, command_generation_, active_real_trajectory_.joint_names);
    }

    RCLCPP_WARN(
      get_logger(),
      "Cancel requested for goal %s: accepting a cancel request does not mean the robot has "
      "stopped, and Gazebo cancellation is asynchronous", id.c_str());
    if (sim_goal) {
      requestSimulationCancel(front_id, sim_goal);
    }
    if (stop_published) {
      RCLCPP_WARN(
        get_logger(),
        "Published empty real trajectory as the existing driver stop request for goal %s; "
        "this is not a vendor-confirmed emergency stop", id.c_str());
    }
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    const std::string id = goalIdText(goal_handle->get_goal_id());
    // Identity of this goal's real command.  Every publish is tagged with it so
    // that a late callback from a previous goal can never command the arm.
    uint64_t generation = 0;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (!slot_reserved_ || pending_goal_id_ != goal_handle->get_goal_id()) {
        RCLCPP_ERROR(get_logger(), "Accepted goal %s has no valid reservation; aborting", id.c_str());
        auto result = std::make_shared<FollowJointTrajectory::Result>();
        result->error_code = FollowJointTrajectory::Result::INVALID_GOAL;
        goal_handle->abort(result);
        return;
      }
      slot_reserved_ = false;
      active_goal_ = goal_handle;
      active_real_trajectory_ = goal->trajectory;
      sim_done_ = false;
      real_command_sent_ = false;
      real_completion_confirmed_ = false;
      ++command_generation_;
      real_stopped_generation_ = 0;
      generation = command_generation_;
      real_feedback_sequence_at_command_ = real_feedback_sequence_;
      real_status_sequence_at_command_ = real_status_sequence_;
      real_faulted_ = false;
      real_fault_reason_.clear();
      terminal_state_ = false;
      sim_outcome_ = "pending";
      real_outcome_ = publish_real_ ? "not_dispatched" : "disabled";
      setStateLocked(TaskState::WaitingForSim, id);
      acceptance_deadline_ = std::chrono::steady_clock::now() + secondsToSteady(acceptance_timeout_);
    }

    FollowJointTrajectory::Goal sim_goal;
    sim_goal.trajectory = goal->trajectory;
    std::string error;
    if (!duco_reliability::scaleTrajectoryTime(sim_goal.trajectory, sim_time_scale_, error)) {
      finishGoal(
        goal_handle, rclcpp_action::ResultCode::ABORTED,
        FollowJointTrajectory::Result::INVALID_GOAL,
        "Cannot scale Gazebo trajectory: " + error);
      return;
    }
    sim_goal.path_tolerance = goal->path_tolerance;
    sim_goal.goal_tolerance = goal->goal_tolerance;
    sim_goal.goal_time_tolerance = goal->goal_time_tolerance;
    const auto real_trajectory = goal->trajectory;

    rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions options;
    options.goal_response_callback =
      [this, goal_handle, real_trajectory, generation](
        const SimGoalHandle::SharedPtr & sim_goal_handle) {
        const GoalUUID front_id = goal_handle->get_goal_id();
        const std::string id = goalIdText(front_id);
        bool request_unknown = false;
        bool front_gone = false;
        bool late_rejection = false;
        bool cancel_immediately = false;
        bool publish_real = false;
        bool real_precheck_failed = false;
        std::string real_precheck_error;
        {
          std::lock_guard<std::mutex> lock(state_mutex_);
          auto * request = findSimRequestLocked(front_id);
          if (request == nullptr) {
            // The request was already closed out, yet a response arrived.  It
            // must never be left running.
            request_unknown = true;
          } else if (request->abandoned || active_goal_ != goal_handle || terminal_state_ ||
            generation != command_generation_)
          {
            // The front goal already finished, or a newer goal took over: this
            // response belongs to the past and must never command the real arm.
            front_gone = true;
            late_rejection = sim_goal_handle == nullptr;
            if (!late_rejection) {
              request->handle = sim_goal_handle;
              request->accepted = true;
              request->cancel_requested = true;
              request->abandoned = true;
              request->abandoned_at = std::chrono::steady_clock::now();
            }
          } else if (!sim_goal_handle) {
            cancel_immediately = task_state_ == TaskState::Canceling;
          } else {
            request->handle = sim_goal_handle;
            request->accepted = true;
            if (task_state_ == TaskState::Canceling) {
              cancel_immediately = true;
            } else {
              setStateLocked(TaskState::Executing, id);
              sim_deadline_ = std::chrono::steady_clock::now() +
                secondsToSteady(sim_execution_timeout_);
              sim_outcome_ = "accepted";
              if (publish_real_) {
                // Last chance to refuse before the real arm is commanded.
                if (!validateRealCommandLocked(real_trajectory, real_precheck_error)) {
                  real_precheck_failed = true;
                  real_outcome_ = "refused";
                } else {
                  // The generation check and the publish happen inside the same
                  // critical section, so a cancel or a fault that acquired the
                  // mutex first is always observed and this motion is refused
                  // instead of overtaking the stop request.
                  publish_real = publishRealMotionLocked(front_id, generation, real_trajectory);
                }
              }
            }
          }
        }

        if (request_unknown) {
          if (sim_goal_handle) {
            RCLCPP_ERROR(
              get_logger(),
              "Untracked Gazebo goal response for %s; cancelling it instead of dropping it",
              id.c_str());
            requestSimulationCancel(front_id, sim_goal_handle);
          }
          return;
        }

        if (front_gone) {
          if (late_rejection) {
            std::lock_guard<std::mutex> lock(state_mutex_);
            eraseSimRequestLocked(front_id);
            return;
          }
          // Late acceptance for a goal that already terminated.  Keep tracking
          // it until it reports a terminal state, and cancel it now.
          RCLCPP_ERROR(
            get_logger(),
            "Late Gazebo acceptance for goal %s which already terminated; cancelling the "
            "stale simulation goal instead of dropping the response", id.c_str());
          requestSimulationCancel(front_id, sim_goal_handle);
          return;
        }

        if (!sim_goal_handle) {
          if (cancel_immediately) {
            finishGoal(
              goal_handle, rclcpp_action::ResultCode::CANCELED,
              FollowJointTrajectory::Result::SUCCESSFUL,
              "Goal canceled before Gazebo accepted it");
          } else {
            finishGoal(
              goal_handle, rclcpp_action::ResultCode::ABORTED,
              FollowJointTrajectory::Result::INVALID_GOAL,
              "Gazebo controller rejected or failed to accept trajectory");
          }
          return;
        }
        if (cancel_immediately) {
          requestSimulationCancel(front_id, sim_goal_handle);
          return;
        }
        if (real_precheck_failed) {
          // The simulation has already accepted the goal but the real command
          // is not safe to send, so the two arms would diverge.  Cancel the
          // simulation and report the refusal rather than silently continuing.
          RCLCPP_ERROR(
            get_logger(), "Goal %s: refusing to command the real robot: %s",
            id.c_str(), real_precheck_error.c_str());
          requestSimulationCancel(front_id, sim_goal_handle);
          finishGoal(
            goal_handle, rclcpp_action::ResultCode::ABORTED,
            FollowJointTrajectory::Result::INVALID_GOAL,
            "Refused to command the real robot: " + real_precheck_error);
          return;
        }
        if (publish_real) {
          RCLCPP_INFO(
            get_logger(), "Gazebo accepted goal %s; publishing %zu original points to real DUCO driver",
            id.c_str(), real_trajectory.points.size());
        }
      };
    options.feedback_callback =
      [this, goal_handle](SimGoalHandle::SharedPtr,
      const std::shared_ptr<const FollowJointTrajectory::Feedback> feedback) {
        bool active = false;
        {
          std::lock_guard<std::mutex> lock(state_mutex_);
          active = active_goal_ == goal_handle && !terminal_state_;
        }
        if (active) {
          goal_handle->publish_feedback(std::make_shared<FollowJointTrajectory::Feedback>(*feedback));
        }
      };
    options.result_callback =
      [this, goal_handle, generation](const SimGoalHandle::WrappedResult & wrapped) {
        const GoalUUID front_id = goal_handle->get_goal_id();
        const std::string id = goalIdText(front_id);
        auto result = std::make_shared<FollowJointTrajectory::Result>();
        if (wrapped.result) {
          *result = *wrapped.result;
        }
        bool abandoned = false;
        bool active = false;
        bool canceling = false;
        bool cancel_lost_race = false;
        bool real_commanded = false;
        {
          std::lock_guard<std::mutex> lock(state_mutex_);
          auto * request = findSimRequestLocked(front_id);
          abandoned = request != nullptr && request->abandoned;
          eraseSimRequestLocked(front_id);

          active = active_goal_ == goal_handle && !terminal_state_ &&
            generation == command_generation_;
          if (active) {
            canceling = task_state_ == TaskState::Canceling;
            if (canceling && wrapped.code == rclcpp_action::ResultCode::SUCCEEDED) {
              // The cancel request lost the race: the trajectory really did
              // finish.  Resume the normal completion logic rather than
              // pretending a cancellation happened, because a cancel request
              // has never been evidence that the robot stopped.
              cancel_lost_race = true;
              canceling = false;
              setStateLocked(TaskState::Executing, id);
            }
            sim_done_ = wrapped.code == rclcpp_action::ResultCode::SUCCEEDED;
            real_commanded = real_command_sent_;
            if (wrapped.code == rclcpp_action::ResultCode::SUCCEEDED) {
              sim_outcome_ = "succeeded";
            } else if (wrapped.code == rclcpp_action::ResultCode::CANCELED) {
              sim_outcome_ = "canceled";
            } else {
              sim_outcome_ = "aborted";
            }
          }
        }
        if (!active) {
          if (abandoned) {
            RCLCPP_INFO(
              get_logger(),
              "Stale Gazebo goal %s reached a terminal state after the front goal had finished; "
              "the real robot state is still not inferred from it", id.c_str());
          }
          return;
        }
        if (cancel_lost_race) {
          RCLCPP_WARN(
            get_logger(),
            "Goal %s: a cancellation was requested but Gazebo completed the trajectory; "
            "reporting the real outcome instead of a clean cancellation", id.c_str());
          tryFinishGoal(goal_handle);
        } else if (canceling) {
          // Only CANCELED from Gazebo is a cancellation.  What Gazebo actually
          // reported is preserved, and a real stop is never inferred from it.
          if (wrapped.code == rclcpp_action::ResultCode::CANCELED) {
            finishGoal(
              goal_handle, rclcpp_action::ResultCode::CANCELED,
              real_commanded ?
              FollowJointTrajectory::Result::GOAL_TOLERANCE_VIOLATED : result->error_code,
              real_commanded ?
              "Gazebo canceled the trajectory but the real robot stop is not confirmed" :
              "Gazebo canceled the trajectory");
          } else {
            finishGoal(
              goal_handle, rclcpp_action::ResultCode::ABORTED,
              result->error_code,
              "Gazebo aborted the trajectory while a cancellation was pending; the real robot "
              "state is not inferred");
          }
        } else if (wrapped.code == rclcpp_action::ResultCode::SUCCEEDED) {
          tryFinishGoal(goal_handle);
        } else if (wrapped.code == rclcpp_action::ResultCode::CANCELED) {
          finishGoal(
            goal_handle, rclcpp_action::ResultCode::CANCELED,
            real_commanded ?
            FollowJointTrajectory::Result::GOAL_TOLERANCE_VIOLATED : result->error_code,
            real_commanded ?
            "Gazebo trajectory canceled but the real robot stop is not confirmed" :
            "Gazebo trajectory canceled");
        } else {
          finishGoal(
            goal_handle, rclcpp_action::ResultCode::ABORTED,
            result->error_code, "Gazebo trajectory failed or aborted; real robot state is not inferred");
        }
      };

    try {
      {
        // Register the request before it is sent: a response can arrive after
        // the front goal has already terminated, and it must still be
        // correlatable so the simulation goal can be cancelled.
        std::lock_guard<std::mutex> lock(state_mutex_);
        SimRequest request;
        request.front_goal_id = goal_handle->get_goal_id();
        request.created_at = std::chrono::steady_clock::now();
        sim_requests_.push_back(request);
      }
      sim_action_client_->async_send_goal(sim_goal, options);
    } catch (const std::exception & exception) {
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        eraseSimRequestLocked(goal_handle->get_goal_id());
      }
      finishGoal(
        goal_handle, rclcpp_action::ResultCode::ABORTED,
        FollowJointTrajectory::Result::INVALID_GOAL,
        std::string("Failed to send Gazebo trajectory: ") + exception.what());
    }
  }

  /**
   * \brief Ask Gazebo to cancel a goal.
   *
   * Takes the front goal UUID rather than the goal handle so that an abandoned
   * simulation goal, whose front action has already terminated, can still be
   * cancelled and logged.
   */
  void requestSimulationCancel(
    const GoalUUID & front_id, const std::shared_ptr<SimGoalHandle> & sim_goal)
  {
    const std::string id = goalIdText(front_id);
    try {
      sim_action_client_->async_cancel_goal(
        sim_goal,
        [this, front_id, id](
          const rclcpp_action::Client<FollowJointTrajectory>::CancelResponse::SharedPtr response) {
          // An empty goals_canceling list means Gazebo refused the cancel.  That
          // is recorded on the request so the timeout path can refuse to report
          // a clean cancellation.
          const bool rejected = !response || response->goals_canceling.empty();
          {
            std::lock_guard<std::mutex> lock(state_mutex_);
            auto * request = findSimRequestLocked(front_id);
            if (request != nullptr) {
              request->cancel_rejected = rejected;
            }
          }
          if (rejected) {
            RCLCPP_ERROR(
              get_logger(),
              "Gazebo rejected cancellation for goal %s; the simulation goal state is unknown "
              "and a clean cancellation is never reported", id.c_str());
          } else {
            RCLCPP_INFO(
              get_logger(), "Gazebo accepted cancellation for goal %s", id.c_str());
          }
        });
    } catch (const std::exception & exception) {
      RCLCPP_ERROR(
        get_logger(), "Unable to request Gazebo cancellation for goal %s: %s",
        id.c_str(), exception.what());
    }
  }

  void realFeedbackCallback(const FollowJointTrajectory::Feedback::SharedPtr msg)
  {
    std::shared_ptr<GoalHandle> goal;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      latest_real_joint_names_ = msg->joint_names;
      latest_real_positions_ = msg->actual.positions;
      latest_real_velocities_ = msg->actual.velocities;
      // The vendor message carries no timestamp, so freshness is measured from
      // local arrival.  These values are re-published after the controller
      // connection drops, which is why arrival alone proves nothing.
      latest_real_feedback_time_ = std::chrono::steady_clock::now();
      ++real_feedback_sequence_;
      goal = active_goal_;
    }
    tryFinishGoal(goal);
  }

  void realStatusCallback(const duco_msg::msg::DucoRobotState::SharedPtr msg)
  {
    std::string reason;
    if (msg->emc_stop_signal) {
      reason = "emergency stop is active";
    } else if (msg->collision) {
      reason = "robot collision status is active";
    } else if (msg->robot_error != 0) {
      reason = "robot error code " + std::to_string(msg->robot_error);
    } else if (!real_unhealthy_robot_states_.empty() &&
      std::find(
        real_unhealthy_robot_states_.begin(), real_unhealthy_robot_states_.end(),
        static_cast<int64_t>(msg->robot_state)) != real_unhealthy_robot_states_.end())
    {
      // The vendor does not document the controller state enum, so nothing is
      // assumed: only the values the operator declared unsafe are treated as a
      // fault.  DucoRobotStatus itself reports a stop when the state drops to 5
      // while moving.
      reason = "controller reported state " + std::to_string(msg->robot_state) +
        ", which is configured as unsafe";
    }

    std::shared_ptr<GoalHandle> goal;
    std::string fault_goal_id = "-";
    bool log_fault = false;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      // Unlike the feedback topic, the status topic stops publishing when the
      // controller connection drops (DucoRobotStatus::getRobotStatus only
      // publishes inside the connected branch).  This counter is therefore the
      // liveness signal the feedback gate depends on.
      ++real_status_sequence_;
      latest_real_status_time_ = std::chrono::steady_clock::now();
      latest_real_status_healthy_ = reason.empty();
      latest_real_robot_state_ = static_cast<int64_t>(msg->robot_state);
      if (!reason.empty()) {
        log_fault = !real_faulted_ || real_fault_reason_ != reason;
        real_faulted_ = true;
        real_fault_reason_ = reason;
      }
      goal = active_goal_;
      if (goal) {
        fault_goal_id = goalIdText(goal->get_goal_id());
      }
    }
    if (log_fault) {
      RCLCPP_ERROR(
        get_logger(), "Real robot fault detected: %s (emc_stop=%s collision=%s error=%d state=%d)",
        reason.c_str(), msg->emc_stop_signal ? "true" : "false",
        msg->collision ? "true" : "false", msg->robot_error,
        static_cast<int>(msg->robot_state));
    }
    if (log_fault && latch_on_real_fault_) {
      // A fault reported by the robot forbids real motion even when no goal is
      // active: the arm must be inspected before it is commanded again.
      std::lock_guard<std::mutex> lock(state_mutex_);
      latchRobotFaultLocked(reason, fault_goal_id);
    }
    tryFinishGoal(goal);
  }

  void watchdogCallback()
  {
    const auto now = std::chrono::steady_clock::now();
    std::shared_ptr<GoalHandle> goal;
    std::string failure;
    std::string failure_goal_id;
    bool uncertain = false;
    rclcpp_action::ResultCode result_code = rclcpp_action::ResultCode::ABORTED;
    int32_t error_code = FollowJointTrajectory::Result::GOAL_TOLERANCE_VIOLATED;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (slot_reserved_ && !active_goal_ &&
        now - reservation_time_ > secondsToSteady(acceptance_timeout_))
      {
        RCLCPP_WARN(get_logger(), "Clearing stale goal reservation after %.2f seconds", acceptance_timeout_);
        slot_reserved_ = false;
        task_state_ = TaskState::Idle;
      }
      goal = active_goal_;

      // Bounded cleanup for Gazebo requests abandoned before Gazebo ever
      // answered.  Without this the node would refuse every new goal forever if
      // the simulation action server stopped responding, with no way back.
      for (auto it = sim_requests_.begin(); it != sim_requests_.end(); ) {
        const bool never_answered =
          it->abandoned && !it->handle &&
          it->abandoned_at != std::chrono::steady_clock::time_point{};
        if (never_answered &&
          now - it->abandoned_at > secondsToSteady(acceptance_timeout_))
        {
          RCLCPP_ERROR(
            get_logger(),
            "Dropping Gazebo goal request %s that was never answered after being abandoned; its "
            "execution state stays unknown", goalIdText(it->front_goal_id).c_str());
          it = sim_requests_.erase(it);
          continue;
        }
        ++it;
      }

      if (!goal || terminal_state_) {
        return;
      }
      failure_goal_id = goalIdText(goal->get_goal_id());
      bool cancel_rejected = false;
      if (auto * request = findSimRequestLocked(goal->get_goal_id())) {
        cancel_rejected = request->cancel_rejected;
      }
      if (task_state_ == TaskState::Canceling && now >= cancel_deadline_) {
        failure = cancel_rejected ?
          "cancellation timed out and Gazebo had rejected the cancel request; the robot may "
          "still be executing" :
          "cancellation timed out; the robot may still be executing";
        result_code = rclcpp_action::ResultCode::CANCELED;
        // A cancel request being accepted never meant the arm stopped, and a
        // rejected cancel is even less evidence.  Only a cancel that had no real
        // command outstanding may be reported as a clean cancellation.
        error_code = (real_command_sent_ || cancel_rejected) ?
          FollowJointTrajectory::Result::GOAL_TOLERANCE_VIOLATED :
          FollowJointTrajectory::Result::SUCCESSFUL;
        uncertain = real_command_sent_;
      } else if (task_state_ == TaskState::WaitingForSim && now >= acceptance_deadline_) {
        failure = "Gazebo goal acceptance timed out";
        error_code = FollowJointTrajectory::Result::INVALID_GOAL;
      } else if (task_state_ == TaskState::Executing && !sim_done_ && now >= sim_deadline_) {
        failure = "Gazebo trajectory execution timed out; real robot state is not inferred";
        uncertain = real_command_sent_;
      } else if (wait_for_real_ && publish_real_ && real_command_sent_ &&
        now - real_command_time_ >= secondsToSteady(real_execution_timeout_))
      {
        failure = "real robot execution timed out; no vendor stop interface was assumed";
        uncertain = true;
      }
    }
    if (!failure.empty()) {
      if (uncertain) {
        // A timeout means the real motion state cannot be established, which is
        // more serious than a plain unconfirmed completion.
        std::lock_guard<std::mutex> lock(state_mutex_);
        latchUncertainLocked(failure, failure_goal_id);
      }
      finishGoal(goal, result_code, error_code, failure);
      return;
    }
    tryFinishGoal(goal);
  }

  static std::chrono::steady_clock::duration secondsToSteady(double seconds)
  {
    return std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(seconds));
  }

  /**
   * \brief The single exit for real *motion* commands.
   *
   * Caller must hold state_mutex_.  Everything that decides whether the arm may
   * move is checked here, inside the same critical section as the publish, so no
   * other callback can interleave between the decision and the command:
   *   * the caller must still be the current goal (generation and handle),
   *   * the goal must not have terminated,
   *   * no real stop may have been requested for this command, and
   *   * the real motion interlock must allow motion.
   */
  bool publishRealMotionLocked(
    const GoalUUID & goal_id, uint64_t generation,
    const trajectory_msgs::msg::JointTrajectory & trajectory)
  {
    const std::string id = goalIdText(goal_id);
    if (!publish_real_) {
      return false;
    }
    if (generation == 0 || generation != command_generation_) {
      RCLCPP_ERROR(
        get_logger(),
        "Refusing to publish a real trajectory for stale goal %s (generation %llu, current %llu)",
        id.c_str(), static_cast<unsigned long long>(generation),
        static_cast<unsigned long long>(command_generation_));
      return false;
    }
    if (!active_goal_ || active_goal_->get_goal_id() != goal_id || terminal_state_) {
      return false;
    }
    if (real_stopped_generation_ == generation) {
      RCLCPP_ERROR(
        get_logger(),
        "Refusing to publish a real trajectory for goal %s: a real stop request was already sent "
        "for this command", id.c_str());
      return false;
    }
    if (real_command_sent_) {
      return false;
    }
    if (real_interlock_.blocksRealMotion()) {
      RCLCPP_ERROR(
        get_logger(), "Refusing to publish a real trajectory for goal %s: interlock is '%s' (%s)",
        id.c_str(), duco_reliability::realMotionStateName(real_interlock_.state()),
        real_interlock_.reason().c_str());
      return false;
    }

    real_command_sent_ = true;
    real_completion_confirmed_ = false;
    active_goal_generation_ = generation;
    real_feedback_sequence_at_command_ = real_feedback_sequence_;
    real_status_sequence_at_command_ = real_status_sequence_;
    real_command_time_ = std::chrono::steady_clock::now();
    real_interlock_.onCommandPublished(generation, id);
    // Record that a real trajectory is outstanding: if this process dies before
    // the completion is confirmed, a restart must not look like a safe state.
    writeFaultMarkerLocked();
    real_command_pub_->publish(trajectory);
    real_outcome_ = "dispatched";
    publishInterlockStateLocked();
    return true;
  }

  /**
   * \brief The single exit for real *stop* requests.
   *
   * Caller must hold state_mutex_.  Publishing a stop is not a confirmation that
   * the robot stopped: no vendor interface provides one, so this only asks the
   * existing driver to stop and records that the command was stopped, which in
   * turn forbids the matching motion command from being published afterwards.
   * The two can therefore never be reordered.
   */
  bool publishRealStopLocked(
    const GoalUUID & goal_id, uint64_t generation,
    const std::vector<std::string> & joint_names)
  {
    if (!publish_real_ || !publish_real_stop_request_) {
      return false;
    }
    if (generation == 0 || generation != command_generation_) {
      return false;
    }
    if (real_stopped_generation_ == generation) {
      return false;
    }
    trajectory_msgs::msg::JointTrajectory stop_command;
    stop_command.joint_names = joint_names;
    real_command_pub_->publish(stop_command);
    real_stopped_generation_ = generation;
    real_outcome_ = "stop_requested";
    publishInterlockStateLocked();
    RCLCPP_WARN(
      get_logger(), "Published the real stop request for goal %s (generation %llu)",
      goalIdText(goal_id).c_str(), static_cast<unsigned long long>(generation));
    return true;
  }

  /**
   * \brief Final checks performed immediately before the real trajectory is
   * published.  Caller must hold state_mutex_.
   *
   * Returns false and fills \p error when the command must not be sent.  This
   * function never attempts to recover or clear a robot fault; recovery is an
   * explicit operator action.
   */
  bool validateRealCommandLocked(
    const trajectory_msgs::msg::JointTrajectory & trajectory, std::string & error) const
  {
    if (real_faulted_) {
      error = "real robot fault is active: " + real_fault_reason_;
      return false;
    }
    if (real_command_sent_) {
      error = "a previously published real trajectory has not been confirmed";
      return false;
    }
    if (real_interlock_.blocksRealMotion()) {
      error = std::string("the real motion interlock is '") +
        duco_reliability::realMotionStateName(real_interlock_.state()) + "': " +
        real_interlock_.reason();
      return false;
    }

    const auto now = std::chrono::steady_clock::now();
    if (latest_real_status_time_ == std::chrono::steady_clock::time_point{}) {
      error = "no real robot status has been received yet";
      return false;
    }
    if (now < latest_real_status_time_ ||
      now - latest_real_status_time_ > secondsToSteady(real_status_max_age_))
    {
      error = "real robot status is stale";
      return false;
    }
    if (!latest_real_status_healthy_) {
      error = "real robot status reports a fault";
      return false;
    }

    if (trajectory.points.size() < static_cast<size_t>(min_real_trajectory_points_)) {
      error = "trajectory has " + std::to_string(trajectory.points.size()) +
        " point(s) but the real driver needs at least " +
        std::to_string(min_real_trajectory_points_) +
        " because it drops the first point";
      return false;
    }
    return true;
  }

  /**
   * \brief Build the feedback-gate inputs from the cached state and evaluate
   * them.  Caller must hold state_mutex_.
   *
   * Replaces the previous value-difference check, which accepted NaN because
   * every comparison against NaN is false, and skipped the velocity test
   * entirely when the velocity array was empty.
   */
  duco_reliability::FeedbackVerdict evaluateRealAtGoalLocked(
    std::chrono::steady_clock::time_point now) const
  {
    if (!real_command_sent_ || active_real_trajectory_.points.empty()) {
      return duco_reliability::FeedbackVerdict::InvalidTarget;
    }

    duco_reliability::GoalTarget target;
    target.joint_names = active_real_trajectory_.joint_names;
    target.final_positions = active_real_trajectory_.points.back().positions;
    target.feedback_sequence_at_command = real_feedback_sequence_at_command_;
    target.status_sequence_at_command = real_status_sequence_at_command_;

    duco_reliability::FeedbackSnapshot feedback;
    feedback.joint_names = latest_real_joint_names_;
    feedback.positions = latest_real_positions_;
    feedback.velocities = latest_real_velocities_;
    feedback.received_at = latest_real_feedback_time_;
    feedback.sequence = real_feedback_sequence_;

    duco_reliability::StatusSnapshot status;
    status.healthy = latest_real_status_healthy_;
    status.received_at = latest_real_status_time_;
    status.sequence = real_status_sequence_;

    duco_reliability::FeedbackPolicy policy;
    policy.position_tolerance = real_goal_tolerance_;
    policy.velocity_tolerance = real_velocity_tolerance_;
    policy.require_velocity = real_require_velocity_;
    policy.feedback_max_age = std::chrono::duration_cast<std::chrono::milliseconds>(
      secondsToSteady(real_feedback_max_age_));
    policy.status_max_age = std::chrono::duration_cast<std::chrono::milliseconds>(
      secondsToSteady(real_status_max_age_));

    return duco_reliability::evaluateRealFeedback(target, feedback, status, policy, now);
  }

  void tryFinishGoal(const std::shared_ptr<GoalHandle> & goal)
  {
    if (!goal) {
      return;
    }
    bool success = false;
    std::string fault_reason;
    auto verdict = duco_reliability::FeedbackVerdict::Reached;
    const auto now = std::chrono::steady_clock::now();
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (active_goal_ != goal || terminal_state_ || task_state_ == TaskState::Canceling) {
        return;
      }
      // A real-robot fault only has authority over the real channel.  In pure
      // simulation mode no real trajectory can be published, so the simulated
      // motion stays isolated from the cell's state instead of being aborted by
      // a fault it cannot act on.
      if (real_faulted_ && publish_real_) {
        fault_reason = real_fault_reason_;
      } else if (!sim_done_) {
        return;
      } else if (!wait_for_real_ || !publish_real_) {
        // Pure simulation, or the operator asked not to wait for the real arm.
        // In both cases the simulation alone decides the action result; a real
        // completion is never claimed, and finishGoal() keeps the interlock
        // latched when a real command was actually dispatched.
        success = true;
        real_completion_confirmed_ = false;
        real_outcome_ = publish_real_ ? "unconfirmed" : "disabled";
      } else {
        verdict = evaluateRealAtGoalLocked(now);
        success = verdict == duco_reliability::FeedbackVerdict::Reached;
        if (success) {
          real_completion_confirmed_ = true;
        } else {
          real_outcome_ = std::string("unconfirmed:") +
            duco_reliability::feedbackVerdictName(verdict);
        }
      }
    }
    if (success) {
      finishGoal(
        goal, rclcpp_action::ResultCode::SUCCEEDED,
        FollowJointTrajectory::Result::SUCCESSFUL, "");
      return;
    }
    if (fault_reason.empty()) {
      // The simulation finished but the real arm is not confirmed at the goal.
      // Record why, throttled, because this runs on every real feedback and
      // status message (~100 Hz combined).
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Goal %s: waiting for real completion, feedback verdict is '%s'",
        goalIdText(goal->get_goal_id()).c_str(),
        duco_reliability::feedbackVerdictName(verdict));
      return;
    }
    finishGoal(
      goal, rclcpp_action::ResultCode::ABORTED,
      FollowJointTrajectory::Result::PATH_TOLERANCE_VIOLATED,
      "Aborting synchronized trajectory: " + fault_reason);
  }

  void finishGoal(
    const std::shared_ptr<GoalHandle> & goal,
    rclcpp_action::ResultCode code,
    int32_t error_code,
    const std::string & reason)
  {
    const GoalUUID goal_id = goal->get_goal_id();
    const std::string id = goalIdText(goal_id);
    std::vector<std::shared_ptr<SimGoalHandle>> stale_sim_goals;
    std::string latched_reason;
    std::string post_latch_state;
    bool real_was_commanded = false;
    bool real_motion_settled = false;
    bool remove_marker = false;
    uint64_t this_generation = 0;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (active_goal_ != goal || terminal_state_) {
        return;
      }
      terminal_state_ = true;
      if (code == rclcpp_action::ResultCode::SUCCEEDED) {
        setStateLocked(TaskState::Succeeded, id);
      } else if (code == rclcpp_action::ResultCode::CANCELED) {
        setStateLocked(TaskState::Canceled, id);
      } else {
        setStateLocked(TaskState::Failed, id);
      }

      real_was_commanded = real_command_sent_;
      this_generation = active_goal_generation_;
      // A real trajectory was published and this goal did not end in a
      // confirmed real completion, so the arm's state is unknown.  Ending the
      // action must never be taken as evidence that it stopped.
      if (real_was_commanded) {
        if (code == rclcpp_action::ResultCode::SUCCEEDED && real_completion_confirmed_) {
          real_motion_settled = true;
        } else if (code == rclcpp_action::ResultCode::SUCCEEDED) {
          // wait_for_real=false was used: the simulation result is reported, but
          // it says nothing about the arm, so the interlock stays latched.
          latched_reason =
            "the simulation succeeded but the real robot completion was never confirmed";
        } else {
          latched_reason = reason.empty() ?
            std::string("trajectory ended without a confirmed real completion") : reason;
        }
      }

      // Any Gazebo goal belonging to this front goal that has not reported a
      // terminal state is now orphaned.  Keep tracking it so a late result is
      // not dropped, and cancel it.
      for (auto & request : sim_requests_) {
        if (request.front_goal_id == goal_id && !request.abandoned) {
          request.abandoned = true;
          request.abandoned_at = std::chrono::steady_clock::now();
          if (request.handle && !request.cancel_requested) {
            request.cancel_requested = true;
            stale_sim_goals.push_back(request.handle);
          }
        }
      }

      active_goal_.reset();
      slot_reserved_ = false;
      real_command_sent_ = false;
      real_completion_confirmed_ = false;
    }

    for (const auto & handle : stale_sim_goals) {
      RCLCPP_ERROR(
        get_logger(),
        "Goal %s finished while its Gazebo goal was still running; cancelling it. New goals stay "
        "refused until it reports a terminal state.", id.c_str());
      requestSimulationCancel(goal_id, handle);
    }

    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (real_motion_settled) {
        // The feedback gate observed a valid, fresh, healthy, in-tolerance
        // sample, so this command is accounted for.
        real_interlock_.onConfirmedCompletion(this_generation);
        real_outcome_ = "confirmed";
      } else if (!latched_reason.empty()) {
        // The action is finished, the arm is not.  This is the state the whole
        // fix is about: the goal is gone but the real motion is unaccounted for.
        real_interlock_.onGoalFinishedUnconfirmed(this_generation, id, latched_reason);
      }
      if (real_motion_settled || (!real_was_commanded && !real_interlock_.latched())) {
        remove_marker = true;
      }
      post_latch_state = duco_reliability::realMotionStateName(real_interlock_.state());
      publishInterlockStateLocked();
    }

    if (remove_marker) {
      removeFaultMarker();
    }
    if (!latched_reason.empty()) {
      RCLCPP_ERROR(
        get_logger(),
        "Goal %s ended with status %s but the real motion is not confirmed: %s. The real motion "
        "interlock is now '%s', so the next real trajectory is refused until an operator calls "
        "the '%s' service.",
        id.c_str(), code == rclcpp_action::ResultCode::CANCELED ? "canceled" : "failed",
        latched_reason.c_str(), post_latch_state.c_str(), clear_fault_service_.c_str());
    } else if (real_motion_settled && real_was_commanded && !wait_for_real_) {
      RCLCPP_WARN(
        get_logger(),
        "Goal %s reported success from the simulation alone (wait_for_real is false); that is "
        "not a confirmation that the real arm reached the goal", id.c_str());
    }

    auto result = std::make_shared<FollowJointTrajectory::Result>();
    result->error_code = error_code;
    if (!reason.empty()) {
      if (code == rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_INFO(get_logger(), "Goal %s completed: %s", id.c_str(), reason.c_str());
      } else {
        RCLCPP_WARN(
          get_logger(), "Goal %s completed with status %s: %s", id.c_str(),
          code == rclcpp_action::ResultCode::CANCELED ? "canceled" : "failed", reason.c_str());
      }
    } else {
      RCLCPP_INFO(get_logger(), "Goal %s completed successfully", id.c_str());
    }
    if (code == rclcpp_action::ResultCode::SUCCEEDED) {
      goal->succeed(result);
    } else if (code == rclcpp_action::ResultCode::CANCELED) {
      goal->canceled(result);
    } else {
      goal->abort(result);
    }
  }

  std::string front_action_name_;
  std::string sim_action_name_;
  std::string real_command_topic_;
  bool publish_real_{false};
  bool wait_for_real_{false};
  bool publish_real_stop_request_{true};
  std::string real_feedback_topic_;
  std::string real_status_topic_;
  double real_goal_tolerance_{0.02};
  double real_velocity_tolerance_{0.05};
  double real_feedback_max_age_{1.0};
  double real_status_max_age_{1.0};
  bool real_require_velocity_{true};
  std::string real_motion_state_topic_;
  bool fault_recovery_enabled_{false};
  bool latch_on_real_fault_{true};
  std::vector<int64_t> real_unhealthy_robot_states_;
  int min_real_trajectory_points_{2};
  std::string clear_fault_service_;
  std::string fault_latch_file_;
  double real_execution_timeout_{300.0};
  double wait_for_sim_seconds_{5.0};
  double acceptance_timeout_{5.0};
  double sim_execution_timeout_{300.0};
  double cancel_timeout_{5.0};
  double sim_time_scale_{1.0};
  bool enforce_expected_joint_names_{true};
  std::vector<std::string> expected_joint_names_;
  std::vector<double> joint_position_min_;
  std::vector<double> joint_position_max_;
  /// Derived once from the parameters above; used by the trajectory rules.
  duco_reliability::TrajectoryValidationPolicy trajectory_policy_;

  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr real_command_pub_;
  /// Diagnostics only: separates the simulation result from the real motion
  /// result for the upper layer.  Never used for a control decision.
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr real_motion_state_pub_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr sim_action_client_;
  rclcpp_action::Server<FollowJointTrajectory>::SharedPtr action_server_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_fault_srv_;
  rclcpp::Subscription<FollowJointTrajectory::Feedback>::SharedPtr real_feedback_sub_;
  rclcpp::Subscription<duco_msg::msg::DucoRobotState>::SharedPtr real_status_sub_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;

  std::mutex state_mutex_;
  TaskState task_state_{TaskState::Idle};
  std::shared_ptr<GoalHandle> active_goal_;
  /// Gazebo goal requests that have not yet reported a terminal state.  Kept
  /// independently of the front goal so late responses stay correlatable.
  std::vector<SimRequest> sim_requests_;
  /// Real motion permission, independent of the action goal.  This is the
  /// authority for whether the next real trajectory may be sent.
  duco_reliability::RealMotionInterlock real_interlock_;
  /// Identity of the goal that currently owns the real channel.  Incremented on
  /// every accepted goal, so a late callback can prove which goal it belongs to.
  uint64_t command_generation_{0};
  /// Generation of the goal that is active right now (set when its command is
  /// published).
  uint64_t active_goal_generation_{0};
  /// Generation a real stop request was published for.  A motion command for the
  /// same generation is refused afterwards, so a stop can never be overtaken.
  uint64_t real_stopped_generation_{0};
  /// Last known simulation outcome and real motion outcome, for diagnostics.
  std::string sim_outcome_{"idle"};
  std::string real_outcome_{"disabled"};
  trajectory_msgs::msg::JointTrajectory active_real_trajectory_;
  std::vector<std::string> latest_real_joint_names_;
  std::vector<double> latest_real_positions_;
  std::vector<double> latest_real_velocities_;
  GoalUUID pending_goal_id_{};
  bool slot_reserved_{false};
  bool sim_done_{false};
  bool real_command_sent_{false};
  /// True only when the feedback gate confirmed this goal's real completion.
  bool real_completion_confirmed_{false};
  bool terminal_state_{false};
  bool real_faulted_{false};
  std::string real_fault_reason_;
  /// Last controller state value seen in the robot status.  The vendor does not
  /// document the enum, so it is only recorded/logged, never interpreted.
  int64_t latest_real_robot_state_{-1};
  uint64_t real_feedback_sequence_{0};
  uint64_t real_feedback_sequence_at_command_{0};
  /// Counter over status messages.  DucoRobotStatus stops publishing status
  /// when the controller connection drops, while the feedback topic keeps
  /// re-publishing cached values, so this is the real liveness signal.
  uint64_t real_status_sequence_{0};
  uint64_t real_status_sequence_at_command_{0};
  std::chrono::steady_clock::time_point latest_real_feedback_time_{};
  std::chrono::steady_clock::time_point latest_real_status_time_{};
  bool latest_real_status_healthy_{false};
  std::chrono::steady_clock::time_point reservation_time_{};
  std::chrono::steady_clock::time_point acceptance_deadline_{};
  std::chrono::steady_clock::time_point sim_deadline_{};
  std::chrono::steady_clock::time_point cancel_deadline_{};
  std::chrono::steady_clock::time_point real_command_time_{};
};

#endif  // DUCO_ROS_DRIVER_DUAL_TRAJECTORY_ACTION_HPP
