#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
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
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "duco_msg/msg/duco_robot_state.hpp"

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
  }

  bool durationNanoseconds(
    const builtin_interfaces::msg::Duration & duration, int64_t & nanoseconds,
    std::string & error) const
  {
    if (duration.sec < 0) {
      error = "duration has a negative seconds field";
      return false;
    }
    if (duration.nanosec >= 1000000000U) {
      error = "duration nanoseconds field is outside [0, 1e9)";
      return false;
    }
    constexpr int64_t billion = 1000000000LL;
    if (static_cast<int64_t>(duration.sec) >
      (std::numeric_limits<int64_t>::max() - duration.nanosec) / billion)
    {
      error = "duration overflows int64 nanoseconds";
      return false;
    }
    nanoseconds = static_cast<int64_t>(duration.sec) * billion + duration.nanosec;
    return true;
  }

  bool validateTrajectory(
    const trajectory_msgs::msg::JointTrajectory & trajectory, std::string & error) const
  {
    if (trajectory.joint_names.empty()) {
      error = "joint_names is empty";
      return false;
    }
    std::unordered_set<std::string> names;
    for (const auto & name : trajectory.joint_names) {
      if (name.empty()) {
        error = "joint_names contains an empty name";
        return false;
      }
      if (!names.insert(name).second) {
        error = "joint_names contains duplicate name: " + name;
        return false;
      }
    }
    if (enforce_expected_joint_names_ && trajectory.joint_names != expected_joint_names_) {
      error = "joint_names do not exactly match expected_joint_names";
      return false;
    }
    if (trajectory.points.empty()) {
      error = "trajectory points is empty";
      return false;
    }

    int64_t previous_time = -1;
    for (size_t point_index = 0; point_index < trajectory.points.size(); ++point_index) {
      const auto & point = trajectory.points[point_index];
      const size_t joint_count = trajectory.joint_names.size();
      if (point.positions.size() != joint_count) {
        error = "point " + std::to_string(point_index) + " positions dimension is " +
          std::to_string(point.positions.size()) + ", expected " + std::to_string(joint_count);
        return false;
      }
      if ((!point.velocities.empty() && point.velocities.size() != joint_count) ||
        (!point.accelerations.empty() && point.accelerations.size() != joint_count) ||
        (!point.effort.empty() && point.effort.size() != joint_count))
      {
        error = "optional point array dimension does not match joint count at point " +
          std::to_string(point_index);
        return false;
      }
      const auto checkFinite = [&](const std::vector<double> & values, const char * field) {
        for (const double value : values) {
          if (!std::isfinite(value)) {
            error = std::string(field) + " contains NaN or Inf at point " +
              std::to_string(point_index);
            return false;
          }
        }
        return true;
      };
      if (!checkFinite(point.positions, "positions") ||
        !checkFinite(point.velocities, "velocities") ||
        !checkFinite(point.accelerations, "accelerations") ||
        !checkFinite(point.effort, "effort"))
      {
        return false;
      }

      int64_t current_time = 0;
      if (!durationNanoseconds(point.time_from_start, current_time, error)) {
        error = "point " + std::to_string(point_index) + ": " + error;
        return false;
      }
      if (previous_time >= 0 && current_time <= previous_time) {
        error = "time_from_start must be strictly increasing at point " +
          std::to_string(point_index);
        return false;
      }
      previous_time = current_time;

      if (!joint_position_min_.empty()) {
        for (size_t joint_index = 0; joint_index < joint_count; ++joint_index) {
          if (point.positions[joint_index] < joint_position_min_[joint_index] ||
            point.positions[joint_index] > joint_position_max_[joint_index])
          {
            error = "position limit violated at point " + std::to_string(point_index) +
              ", joint " + trajectory.joint_names[joint_index];
            return false;
          }
        }
      }
    }
    return true;
  }

  bool scaleTrajectoryTime(
    trajectory_msgs::msg::JointTrajectory & trajectory, double scale, std::string & error) const
  {
    if (!std::isfinite(scale) || scale <= 0.0) {
      error = "sim_time_scale must be finite and > 0";
      return false;
    }
    constexpr long double billion = 1000000000.0L;
    constexpr long double max_duration =
      static_cast<long double>(std::numeric_limits<int32_t>::max()) * billion + (billion - 1.0L);
    int64_t previous_scaled = -1;
    const long double scale_ld = static_cast<long double>(scale);
    const long double velocity_scale = 1.0L / scale_ld;
    const long double acceleration_scale = velocity_scale * velocity_scale;

    for (size_t point_index = 0; point_index < trajectory.points.size(); ++point_index) {
      auto & point = trajectory.points[point_index];
      int64_t original = 0;
      if (!durationNanoseconds(point.time_from_start, original, error)) {
        return false;
      }
      const long double scaled = static_cast<long double>(original) * scale_ld;
      if (!std::isfinite(scaled) || scaled < 0.0L || scaled > max_duration) {
        error = "scaled trajectory duration overflows at point " + std::to_string(point_index);
        return false;
      }
      const int64_t scaled_ns = static_cast<int64_t>(std::llround(scaled));
      if (previous_scaled >= 0 && scaled_ns <= previous_scaled) {
        error = "scaled time_from_start is not strictly increasing at point " +
          std::to_string(point_index);
        return false;
      }
      previous_scaled = scaled_ns;
      point.time_from_start.sec = static_cast<int32_t>(scaled_ns / 1000000000LL);
      point.time_from_start.nanosec = static_cast<uint32_t>(scaled_ns % 1000000000LL);

      const auto scaleValues = [&](std::vector<double> & values, long double factor, const char * field) {
        for (double & value : values) {
          const long double scaled_value = static_cast<long double>(value) * factor;
          if (!std::isfinite(scaled_value) ||
            scaled_value > std::numeric_limits<double>::max() ||
            scaled_value < -std::numeric_limits<double>::max())
          {
            error = std::string("scaled ") + field + " is not finite at point " +
              std::to_string(point_index);
            return false;
          }
          value = static_cast<double>(scaled_value);
        }
        return true;
      };
      if (!scaleValues(point.velocities, velocity_scale, "velocities") ||
        !scaleValues(point.accelerations, acceleration_scale, "accelerations"))
      {
        return false;
      }
    }
    return true;
  }

  rclcpp_action::GoalResponse handle_goal(
    const GoalUUID & goal_id, std::shared_ptr<const FollowJointTrajectory::Goal> goal)
  {
    std::string error;
    if (!validateTrajectory(goal->trajectory, error)) {
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
    const std::string id = goalIdText(goal_handle->get_goal_id());
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
      sim_goal = sim_goal_handle_;
      publish_stop = publish_real_ && publish_real_stop_request_ && real_command_sent_;
    }

    RCLCPP_WARN(
      get_logger(), "Cancel requested for goal %s; Gazebo cancellation is asynchronous", id.c_str());
    if (sim_goal) {
      requestSimulationCancel(goal_handle, sim_goal);
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
      real_faulted_ = false;
      real_fault_reason_.clear();
      terminal_state_ = false;
      setStateLocked(TaskState::WaitingForSim, id);
      acceptance_deadline_ = std::chrono::steady_clock::now() + secondsToSteady(acceptance_timeout_);
    }

    FollowJointTrajectory::Goal sim_goal;
    sim_goal.trajectory = goal->trajectory;
    std::string error;
    if (!scaleTrajectoryTime(sim_goal.trajectory, sim_time_scale_, error)) {
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
        bool cancel_immediately = false;
        bool publish_real = false;
        {
          std::lock_guard<std::mutex> lock(state_mutex_);
          if (active_goal_ != goal_handle || terminal_state_) {
            return;
          }
          if (!sim_goal_handle) {
            cancel_immediately = task_state_ == TaskState::Canceling;
          } else {
            sim_goal_handle_ = sim_goal_handle;
            if (task_state_ == TaskState::Canceling) {
              cancel_immediately = true;
            } else {
              setStateLocked(TaskState::Executing, goalIdText(goal_handle->get_goal_id()));
              sim_deadline_ = std::chrono::steady_clock::now() + secondsToSteady(sim_execution_timeout_);
              publish_real = publish_real_;
              if (publish_real) {
                real_command_sent_ = true;
                real_feedback_sequence_at_command_ = real_feedback_sequence_;
                real_command_time_ = std::chrono::steady_clock::now();
              }
            }
          }
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
          requestSimulationCancel(goal_handle, sim_goal_handle);
          return;
        }
        if (publish_real) {
          RCLCPP_INFO(
            get_logger(), "Gazebo accepted goal %s; publishing %zu original points to real DUCO driver",
            goalIdText(goal_handle->get_goal_id()).c_str(), real_trajectory.points.size());
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
        auto result = std::make_shared<FollowJointTrajectory::Result>();
        if (wrapped.result) {
          *result = *wrapped.result;
        }
        bool canceling = false;
        {
          std::lock_guard<std::mutex> lock(state_mutex_);
          if (active_goal_ != goal_handle || terminal_state_) {
            return;
          }
          canceling = task_state_ == TaskState::Canceling;
          sim_goal_handle_.reset();
          sim_done_ = wrapped.code == rclcpp_action::ResultCode::SUCCEEDED;
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
      sim_action_client_->async_send_goal(sim_goal, options);
    } catch (const std::exception & exception) {
      finishGoal(
        goal_handle, rclcpp_action::ResultCode::ABORTED,
        FollowJointTrajectory::Result::INVALID_GOAL,
        std::string("Failed to send Gazebo trajectory: ") + exception.what());
    }
  }

  void requestSimulationCancel(
    const std::shared_ptr<GoalHandle> & goal, const std::shared_ptr<SimGoalHandle> & sim_goal)
  {
    try {
      sim_action_client_->async_cancel_goal(
        sim_goal,
        [this, goal](const rclcpp_action::Client<FollowJointTrajectory>::CancelResponse::SharedPtr response) {
          if (!response || response->goals_canceling.empty()) {
            RCLCPP_ERROR(
              get_logger(), "Gazebo rejected cancellation for goal %s; waiting for bounded timeout",
              goalIdText(goal->get_goal_id()).c_str());
          } else {
            RCLCPP_INFO(
              get_logger(), "Gazebo accepted cancellation for goal %s",
              goalIdText(goal->get_goal_id()).c_str());
          }
        });
    } catch (const std::exception & exception) {
      RCLCPP_ERROR(
        get_logger(), "Unable to request Gazebo cancellation for goal %s: %s",
        goalIdText(goal->get_goal_id()).c_str(), exception.what());
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
      if (!goal || terminal_state_) {
        return;
      }
      if (task_state_ == TaskState::Canceling && now >= cancel_deadline_) {
        failure = "cancellation timed out; real robot may still be executing";
        result_code = rclcpp_action::ResultCode::CANCELED;
        error_code = FollowJointTrajectory::Result::SUCCESSFUL;
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

  bool realAtGoalLocked() const
  {
    if (!real_command_sent_ || real_feedback_sequence_ <= real_feedback_sequence_at_command_ ||
      active_real_trajectory_.points.empty() || latest_real_joint_names_.empty())
    {
      return false;
    }
    const auto & target = active_real_trajectory_.points.back().positions;
    if (target.size() != active_real_trajectory_.joint_names.size()) {
      return false;
    }
    for (size_t target_index = 0; target_index < target.size(); ++target_index) {
      const auto it = std::find(
        latest_real_joint_names_.begin(), latest_real_joint_names_.end(),
        active_real_trajectory_.joint_names[target_index]);
      if (it == latest_real_joint_names_.end()) {
        return false;
      }
      const size_t actual_index = static_cast<size_t>(
        std::distance(latest_real_joint_names_.begin(), it));
      if (actual_index >= latest_real_positions_.size() ||
        std::abs(latest_real_positions_[actual_index] - target[target_index]) > real_goal_tolerance_)
      {
        return false;
      }
      if (actual_index < latest_real_velocities_.size() &&
        std::abs(latest_real_velocities_[actual_index]) > real_velocity_tolerance_)
      {
        return false;
      }
    }
    return true;
  }

  void tryFinishGoal(const std::shared_ptr<GoalHandle> & goal)
  {
    if (!goal) {
      return;
    }
    bool success = false;
    std::string fault_reason;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (active_goal_ != goal || terminal_state_ || task_state_ == TaskState::Canceling) {
        return;
      }
      if (real_faulted_) {
        fault_reason = real_fault_reason_;
      } else if (sim_done_ && (!wait_for_real_ || !publish_real_ || realAtGoalLocked())) {
        success = true;
      } else {
        return;
      }
    }
    if (success) {
      finishGoal(
        goal, rclcpp_action::ResultCode::SUCCEEDED,
        FollowJointTrajectory::Result::SUCCESSFUL, "");
    } else {
      finishGoal(
        goal, rclcpp_action::ResultCode::ABORTED,
        FollowJointTrajectory::Result::PATH_TOLERANCE_VIOLATED,
        "Aborting synchronized trajectory: " + fault_reason);
    }
  }

  void finishGoal(
    const std::shared_ptr<GoalHandle> & goal,
    rclcpp_action::ResultCode code,
    int32_t error_code,
    const std::string & reason)
  {
    const std::string id = goalIdText(goal->get_goal_id());
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
      active_goal_.reset();
      sim_goal_handle_.reset();
      slot_reserved_ = false;
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

  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr real_command_pub_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr sim_action_client_;
  rclcpp_action::Server<FollowJointTrajectory>::SharedPtr action_server_;
  SimGoalHandle::SharedPtr sim_goal_handle_;
  rclcpp::Subscription<FollowJointTrajectory::Feedback>::SharedPtr real_feedback_sub_;
  rclcpp::Subscription<duco_msg::msg::DucoRobotState>::SharedPtr real_status_sub_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;

  std::mutex state_mutex_;
  TaskState task_state_{TaskState::Idle};
  std::shared_ptr<GoalHandle> active_goal_;
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
  std::chrono::steady_clock::time_point reservation_time_{};
  std::chrono::steady_clock::time_point acceptance_deadline_{};
  std::chrono::steady_clock::time_point sim_deadline_{};
  std::chrono::steady_clock::time_point cancel_deadline_{};
  std::chrono::steady_clock::time_point real_command_time_{};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DualTrajectoryAction>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
