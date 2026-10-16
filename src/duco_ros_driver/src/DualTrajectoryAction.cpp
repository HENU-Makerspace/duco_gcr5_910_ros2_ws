// Entry point for the dual trajectory proxy node.

#include <exception>
#include <iostream>
#include <memory>

#include "duco_ros_driver/DualTrajectoryAction.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  // A rejected configuration must fail loudly with a readable message instead of
  // aborting inside the constructor, and it must never silently fall back to a
  // different (possibly unsafe) behaviour.
  std::shared_ptr<DualTrajectoryAction> node;
  try {
    node = std::make_shared<DualTrajectoryAction>();
  } catch (const std::exception & exception) {
    std::cerr << "DualTrajectoryAction: invalid configuration: " << exception.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
