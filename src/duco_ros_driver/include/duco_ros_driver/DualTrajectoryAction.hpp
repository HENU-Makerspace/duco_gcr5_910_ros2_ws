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
#include <std_srvs/srv/trigger.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "duco_msg/msg/duco_robot_state.hpp"

#include "duco_ros_driver/RealFeedbackGate.hpp"
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
    publish_real_ = declare_parameter<bool>("publish_real", true);
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
    real_feedback_timeout_ = declare_parameter<double>("real_feedback_timeout", 1.0);
    real_status_timeout_ = declare_parameter<double>("real_status_timeout", 1.0);
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
   * \brief A real-robot fault that outlives the action goal.
   *
   * Once a real trajectory has been published, ending the action, timing out or
   * accepting the next goal is not evidence that the arm stopped.  The latch is
   * cleared only by an explicit operator request through clear_fault_service_.
   */
  struct RealFault {
    bool latched{false};
    std::string reason;
    std::string goal_id;
  };

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

    requirePositiveFinite(real_feedback_timeout_, "real_feedback_timeout");
    requirePositiveFinite(real_status_timeout_, "real_status_timeout");
    if (min_real_trajectory_points_ < 1) {
      throw std::invalid_argument("min_real_trajectory_points must be >= 1");
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
   * \brief Restore a latched fault from a previous run.
   *
   * A process-local flag cannot prove that an unconfirmed real motion has gone
   * away, so the marker file survives restarts and forces an explicit operator
   * acknowledgement.
   */
  void restoreLatchedFault()
  {
    if (fault_latch_file_.empty()) {
      return;
    }
    std::ifstream marker(fault_latch_file_);
    if (!marker.good()) {
      return;
    }
    std::string detail;
    std::getline(marker, detail);
    std::lock_guard<std::mutex> lock(state_mutex_);
    latchFaultLocked(
      "a previous run published a real trajectory whose completion was never confirmed"
      " (marker: " + (detail.empty() ? std::string("unknown") : detail) + ")",
      "startup");
  }

  void latchFaultLocked(const std::string & reason, const std::string & goal_id)
  {
    if (!latched_fault_.latched) {
      latched_fault_.latched = true;
      latched_fault_.reason = reason;
      latched_fault_.goal_id = goal_id;
    }
    RCLCPP_ERROR(
      get_logger(),
      "Real robot fault latched (goal %s): %s. New trajectories are refused until an "
      "operator calls the '%s' service.", goal_id.c_str(), reason.c_str(),
      clear_fault_service_.c_str());
    writeFaultMarkerLocked();
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
    marker << latched_fault_.goal_id << " " << latched_fault_.reason << "\n";
  }

  void removeFaultMarker() const
  {
    if (fault_latch_file_.empty()) {
      return;
    }
    std::remove(fault_latch_file_.c_str());
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
    const auto now = std::chrono::steady_clock::now();
    std::string blocker;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (!latched_fault_.latched) {
        response->success = true;
        response->message = "no latched real robot fault";
        return;
      }
      if (active_goal_ || slot_reserved_) {
        blocker = "a trajectory goal is still active";
      } else if (!sim_requests_.empty()) {
        blocker = "a Gazebo goal is still outstanding";
      } else if (real_command_sent_) {
        blocker = "a real trajectory command is still outstanding";
      } else if (latest_real_status_time_ == std::chrono::steady_clock::time_point{}) {
        blocker = "no real robot status has been received";
      } else if (now < latest_real_status_time_ ||
        now - latest_real_status_time_ > secondsToSteady(real_status_timeout_))
      {
        blocker = "real robot status is stale";
      } else if (!latest_real_status_healthy_) {
        blocker = "real robot status reports a fault";
      }
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
        get_logger(), "Operator cleared the latched fault raised by goal %s: %s",
        latched_fault_.goal_id.c_str(), latched_fault_.reason.c_str());
      latched_fault_ = RealFault{};
      real_faulted_ = false;
      real_fault_reason_.clear();
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
    if (latched_fault_.latched) {
      RCLCPP_ERROR(
        get_logger(),
        "Rejecting goal %s: a real robot fault is latched (%s). Confirm the cell is safe and "
        "call the '%s' service.", goalIdText(goal_id).c_str(), latched_fault_.reason.c_str(),
        clear_fault_service_.c_str());
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
    bool publish_stop = false;
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
      publish_stop = publish_real_ && publish_real_stop_request_ && real_command_sent_;
    }

    RCLCPP_WARN(
      get_logger(),
      "Cancel requested for goal %s: accepting a cancel request does not mean the robot has "
      "stopped, and Gazebo cancellation is asynchronous", id.c_str());
    if (sim_goal) {
      requestSimulationCancel(front_id, sim_goal);
    }
    if (publish_stop) {
      trajectory_msgs::msg::JointTrajectory stop_command;
      stop_command.joint_names = goal_handle->get_goal()->trajectory.joint_names;
      real_command_pub_->publish(stop_command);
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
      real_feedback_sequence_at_command_ = real_feedback_sequence_;
      real_status_sequence_at_command_ = real_status_sequence_;
      real_faulted_ = false;
      real_fault_reason_.clear();
      terminal_state_ = false;
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
      [this, goal_handle, real_trajectory](const SimGoalHandle::SharedPtr & sim_goal_handle) {
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
          } else if (request->abandoned || active_goal_ != goal_handle || terminal_state_) {
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
              sim_deadline_ = std::chrono::steady_clock::now() + secondsToSteady(sim_execution_timeout_);
              publish_real = publish_real_;
              if (publish_real) {
                // Last chance to refuse before the real arm is commanded.
                if (!validateRealCommandLocked(real_trajectory, real_precheck_error)) {
                  real_precheck_failed = true;
                  publish_real = false;
                } else {
                  real_command_sent_ = true;
                  real_feedback_sequence_at_command_ = real_feedback_sequence_;
                  real_status_sequence_at_command_ = real_status_sequence_;
                  real_command_time_ = std::chrono::steady_clock::now();
                  writeFaultMarkerLocked();
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
          real_command_pub_->publish(real_trajectory);
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
      [this, goal_handle](const SimGoalHandle::WrappedResult & wrapped) {
        const GoalUUID front_id = goal_handle->get_goal_id();
        const std::string id = goalIdText(front_id);
        auto result = std::make_shared<FollowJointTrajectory::Result>();
        if (wrapped.result) {
          *result = *wrapped.result;
        }
        bool abandoned = false;
        bool active = false;
        bool canceling = false;
        {
          std::lock_guard<std::mutex> lock(state_mutex_);
          auto * request = findSimRequestLocked(front_id);
          abandoned = request != nullptr && request->abandoned;
          eraseSimRequestLocked(front_id);

          active = active_goal_ == goal_handle && !terminal_state_;
          if (active) {
            canceling = task_state_ == TaskState::Canceling;
            sim_done_ = wrapped.code == rclcpp_action::ResultCode::SUCCEEDED;
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
        if (canceling) {
          finishGoal(
            goal_handle, rclcpp_action::ResultCode::CANCELED,
            FollowJointTrajectory::Result::SUCCESSFUL,
            "Gazebo returned after cancellation request");
        } else if (wrapped.code == rclcpp_action::ResultCode::SUCCEEDED) {
          tryFinishGoal(goal_handle);
        } else if (wrapped.code == rclcpp_action::ResultCode::CANCELED) {
          finishGoal(
            goal_handle, rclcpp_action::ResultCode::CANCELED,
            result->error_code, "Gazebo trajectory canceled");
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
        [this, id](const rclcpp_action::Client<FollowJointTrajectory>::CancelResponse::SharedPtr response) {
          if (!response || response->goals_canceling.empty()) {
            RCLCPP_ERROR(
              get_logger(),
              "Gazebo rejected cancellation for goal %s; the simulation goal state is unknown "
              "and new goals stay refused until it is resolved", id.c_str());
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
    }

    std::shared_ptr<GoalHandle> goal;
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
      if (!reason.empty()) {
        log_fault = !real_faulted_ || real_fault_reason_ != reason;
        real_faulted_ = true;
        real_fault_reason_ = reason;
      }
      goal = active_goal_;
    }
    if (log_fault) {
      RCLCPP_ERROR(
        get_logger(), "Real robot fault detected: %s (emc_stop=%s collision=%s error=%d)",
        reason.c_str(), msg->emc_stop_signal ? "true" : "false",
        msg->collision ? "true" : "false", msg->robot_error);
    }
    tryFinishGoal(goal);
  }

  void watchdogCallback()
  {
    const auto now = std::chrono::steady_clock::now();
    std::shared_ptr<GoalHandle> goal;
    std::string failure;
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
      if (task_state_ == TaskState::Canceling && now >= cancel_deadline_) {
        failure = "cancellation timed out; the real robot may still be executing";
        result_code = rclcpp_action::ResultCode::CANCELED;
        // A cancel request being accepted never meant the arm stopped.  Only
        // report a clean cancellation when no real command was outstanding.
        error_code = real_command_sent_ ?
          FollowJointTrajectory::Result::GOAL_TOLERANCE_VIOLATED :
          FollowJointTrajectory::Result::SUCCESSFUL;
      } else if (task_state_ == TaskState::WaitingForSim && now >= acceptance_deadline_) {
        failure = "Gazebo goal acceptance timed out";
        error_code = FollowJointTrajectory::Result::INVALID_GOAL;
      } else if (task_state_ == TaskState::Executing && now >= sim_deadline_) {
        failure = "Gazebo trajectory execution timed out; real robot state is not inferred";
      } else if (wait_for_real_ && publish_real_ && real_command_sent_ &&
        now - real_command_time_ >= secondsToSteady(real_execution_timeout_))
      {
        failure = "real robot execution timed out; no vendor stop interface was assumed";
      }
    }
    if (!failure.empty()) {
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

    const auto now = std::chrono::steady_clock::now();
    if (latest_real_status_time_ == std::chrono::steady_clock::time_point{}) {
      error = "no real robot status has been received yet";
      return false;
    }
    if (now < latest_real_status_time_ ||
      now - latest_real_status_time_ > secondsToSteady(real_status_timeout_))
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
    policy.feedback_max_age = std::chrono::duration_cast<std::chrono::milliseconds>(
      secondsToSteady(real_feedback_timeout_));
    policy.status_max_age = std::chrono::duration_cast<std::chrono::milliseconds>(
      secondsToSteady(real_status_timeout_));

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
      if (real_faulted_) {
        fault_reason = real_fault_reason_;
      } else if (!sim_done_) {
        return;
      } else if (!wait_for_real_ || !publish_real_) {
        // Pure simulation, or the operator asked not to wait for the real arm.
        success = true;
      } else {
        verdict = evaluateRealAtGoalLocked(now);
        success = verdict == duco_reliability::FeedbackVerdict::Reached;
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
    bool real_was_commanded = false;
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
      // A real trajectory was published and this goal did not end in a
      // confirmed real completion, so the arm's state is unknown.  Ending the
      // action must never be taken as evidence that it stopped.
      if (real_was_commanded && code != rclcpp_action::ResultCode::SUCCEEDED) {
        latched_reason = reason.empty() ?
          std::string("trajectory ended without a confirmed real completion") : reason;
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
    }

    for (const auto & handle : stale_sim_goals) {
      RCLCPP_ERROR(
        get_logger(),
        "Goal %s finished while its Gazebo goal was still running; cancelling it. New goals stay "
        "refused until it reports a terminal state.", id.c_str());
      requestSimulationCancel(goal_id, handle);
    }

    if (!latched_reason.empty()) {
      std::lock_guard<std::mutex> lock(state_mutex_);
      latchFaultLocked(latched_reason, id);
    } else if (code == rclcpp_action::ResultCode::SUCCEEDED) {
      removeFaultMarker();
      if (real_was_commanded && !wait_for_real_) {
        RCLCPP_WARN(
          get_logger(),
          "Goal %s reported success from the simulation alone (wait_for_real is false); that is "
          "not a confirmation that the real arm reached the goal", id.c_str());
      }
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
  bool publish_real_{true};
  bool wait_for_real_{false};
  bool publish_real_stop_request_{true};
  std::string real_feedback_topic_;
  std::string real_status_topic_;
  double real_goal_tolerance_{0.02};
  double real_velocity_tolerance_{0.05};
  double real_feedback_timeout_{1.0};
  double real_status_timeout_{1.0};
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
  /// Real robot fault that survives the action goal until an operator clears it.
  RealFault latched_fault_;
  trajectory_msgs::msg::JointTrajectory active_real_trajectory_;
  std::vector<std::string> latest_real_joint_names_;
  std::vector<double> latest_real_positions_;
  std::vector<double> latest_real_velocities_;
  GoalUUID pending_goal_id_{};
  bool slot_reserved_{false};
  bool sim_done_{false};
  bool real_command_sent_{false};
  bool terminal_state_{false};
  bool real_faulted_{false};
  std::string real_fault_reason_;
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
