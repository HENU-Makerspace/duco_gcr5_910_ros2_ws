#include <memory>
#include <string>
#include <cmath>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

class DualTrajectoryAction : public rclcpp::Node
{
public:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using GoalHandle = rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;

  explicit DualTrajectoryAction(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("DualTrajectoryAction", options)
  {
    front_action_name_ = declare_parameter<std::string>(
      "front_action_name", "arm_1_controller/follow_joint_trajectory");
    sim_action_name_ = declare_parameter<std::string>(
      "sim_action_name", "sim_arm_1_controller/follow_joint_trajectory");
    real_command_topic_ = declare_parameter<std::string>(
      "real_command_topic", "joint_path_command");
    publish_real_ = declare_parameter<bool>("publish_real", true);
    wait_for_sim_seconds_ = declare_parameter<double>("wait_for_sim_seconds", 5.0);
    sim_time_scale_ = declare_parameter<double>("sim_time_scale", 1.0);

    real_command_pub_ =
      create_publisher<trajectory_msgs::msg::JointTrajectory>(real_command_topic_, 10);
    sim_action_client_ =
      rclcpp_action::create_client<FollowJointTrajectory>(this, sim_action_name_);
    action_server_ = rclcpp_action::create_server<FollowJointTrajectory>(
      this,
      front_action_name_,
      std::bind(&DualTrajectoryAction::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&DualTrajectoryAction::handle_cancel, this, std::placeholders::_1),
      std::bind(&DualTrajectoryAction::handle_accepted, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Dual trajectory action ready: MoveIt [%s] -> Gazebo [%s] + real topic [%s]",
      front_action_name_.c_str(),
      sim_action_name_.c_str(),
      real_command_topic_.c_str());
  }

private:
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const FollowJointTrajectory::Goal> goal)
  {
    if (goal->trajectory.points.empty()) {
      RCLCPP_ERROR(get_logger(), "Rejecting empty trajectory");
      return rclcpp_action::GoalResponse::REJECT;
    }

    const auto timeout =
      std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(wait_for_sim_seconds_));
    if (!sim_action_client_->wait_for_action_server(timeout)) {
      RCLCPP_ERROR(
        get_logger(), "Rejecting trajectory because Gazebo action server [%s] is not ready",
        sim_action_name_.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle> goal_handle)
  {
    (void)goal_handle;
    if (sim_goal_handle_) {
      sim_action_client_->async_cancel_goal(sim_goal_handle_);
    }
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
  {
    const auto goal = goal_handle->get_goal();

    FollowJointTrajectory::Goal sim_goal;
    sim_goal.trajectory = goal->trajectory;
    scale_trajectory_time(sim_goal.trajectory, sim_time_scale_);
    sim_goal.path_tolerance = goal->path_tolerance;
    sim_goal.goal_tolerance = goal->goal_tolerance;
    sim_goal.goal_time_tolerance = goal->goal_time_tolerance;
    auto real_trajectory = goal->trajectory;

    rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions options;
    options.goal_response_callback =
      [this, goal_handle, real_trajectory](const rclcpp_action::ClientGoalHandle<FollowJointTrajectory>::SharedPtr & gh) {
        sim_goal_handle_ = gh;
        if (!gh) {
          RCLCPP_ERROR(get_logger(), "Gazebo controller rejected trajectory");
          auto result = std::make_shared<FollowJointTrajectory::Result>();
          result->error_code = FollowJointTrajectory::Result::INVALID_GOAL;
          goal_handle->abort(result);
          return;
        }

        if (publish_real_) {
          RCLCPP_INFO(
            get_logger(), "Gazebo accepted trajectory; publishing %zu points to real DUCO driver",
            real_trajectory.points.size());
          real_command_pub_->publish(real_trajectory);
        }
      };
    options.feedback_callback =
      [goal_handle](
      rclcpp_action::ClientGoalHandle<FollowJointTrajectory>::SharedPtr,
      const std::shared_ptr<const FollowJointTrajectory::Feedback> feedback) {
        goal_handle->publish_feedback(std::make_shared<FollowJointTrajectory::Feedback>(*feedback));
      };
    options.result_callback =
      [this, goal_handle](const rclcpp_action::ClientGoalHandle<FollowJointTrajectory>::WrappedResult & wrapped) {
        sim_goal_handle_.reset();
        auto result = std::make_shared<FollowJointTrajectory::Result>();
        if (wrapped.result) {
          *result = *wrapped.result;
        }

        if (wrapped.code == rclcpp_action::ResultCode::SUCCEEDED) {
          RCLCPP_INFO(get_logger(), "Gazebo trajectory finished; reporting success to MoveIt");
          goal_handle->succeed(result);
        } else if (wrapped.code == rclcpp_action::ResultCode::CANCELED) {
          RCLCPP_WARN(get_logger(), "Gazebo trajectory canceled");
          goal_handle->canceled(result);
        } else {
          RCLCPP_ERROR(get_logger(), "Gazebo trajectory failed or aborted");
          goal_handle->abort(result);
        }
      };

    sim_action_client_->async_send_goal(sim_goal, options);
  }

  void scale_trajectory_time(trajectory_msgs::msg::JointTrajectory & trajectory, double scale)
  {
    if (scale <= 0.0 || std::abs(scale - 1.0) < 1e-9) {
      return;
    }

    for (auto & point : trajectory.points) {
      rclcpp::Duration duration(point.time_from_start);
      const auto scaled_nanoseconds =
        static_cast<int64_t>(std::llround(static_cast<double>(duration.nanoseconds()) * scale));
      point.time_from_start.sec = static_cast<int32_t>(scaled_nanoseconds / 1000000000LL);
      point.time_from_start.nanosec = static_cast<uint32_t>(scaled_nanoseconds % 1000000000LL);
    }
    RCLCPP_INFO(get_logger(), "Scaled Gazebo trajectory time by %.3f", scale);
  }

  std::string front_action_name_;
  std::string sim_action_name_;
  std::string real_command_topic_;
  bool publish_real_{true};
  double wait_for_sim_seconds_{5.0};
  double sim_time_scale_{1.0};

  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr real_command_pub_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr sim_action_client_;
  rclcpp_action::Server<FollowJointTrajectory>::SharedPtr action_server_;
  rclcpp_action::ClientGoalHandle<FollowJointTrajectory>::SharedPtr sim_goal_handle_;
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
