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
        bool cancel_immediately = false;
        bool publish_real = false;
        bool real_precheck_failed = false;
        std::string real_precheck_error;
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
                // Last chance to refuse before the real arm is commanded.
                if (!validateRealCommandLocked(real_trajectory, real_precheck_error)) {
                  real_precheck_failed = true;
                  publish_real = false;
                } else {
                  real_command_sent_ = true;
                  real_feedback_sequence_at_command_ = real_feedback_sequence_;
                  real_status_sequence_at_command_ = real_status_sequence_;
                  real_command_time_ = std::chrono::steady_clock::now();
                }
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
        if (real_precheck_failed) {
          // The simulation has already accepted the goal but the real command
          // is not safe to send, so the two arms would diverge.  Cancel the
          // simulation and report the refusal rather than silently continuing.
          RCLCPP_ERROR(
            get_logger(), "Goal %s: refusing to command the real robot: %s",
            goalIdText(goal_handle->get_goal_id()).c_str(), real_precheck_error.c_str());
          requestSimulationCancel(goal_handle, sim_goal_handle);
          finishGoal(
            goal_handle, rclcpp_action::ResultCode::ABORTED,
            FollowJointTrajectory::Result::INVALID_GOAL,
            "Refused to command the real robot: " + real_precheck_error);
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
  double real_feedback_timeout_{1.0};
  double real_status_timeout_{1.0};
  int min_real_trajectory_points_{2};
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
