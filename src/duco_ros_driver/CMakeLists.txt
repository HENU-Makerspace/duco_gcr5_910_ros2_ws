cmake_minimum_required(VERSION 3.8)
project(duco_ros_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp)
find_package(rclcpp_action)
find_package(builtin_interfaces REQUIRED)
find_package(control_msgs)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(duco_msg)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  # Pure-logic unit tests.  These deliberately avoid ROS nodes, executors and
  # simulated action servers so they stay fast and deterministic.
  ament_add_gtest(test_real_feedback_gate test/test_real_feedback_gate.cpp)
  target_include_directories(test_real_feedback_gate PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)

  ament_add_gtest(test_trajectory_validation test/test_trajectory_validation.cpp)
  target_include_directories(test_trajectory_validation PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
  ament_target_dependencies(test_trajectory_validation
    builtin_interfaces trajectory_msgs)
endif()
add_executable(DucoDriver src/DucoDriver.cpp src/DucoDriverNode.cpp src/RobotNode.cpp)
add_executable(DucoRobotStatus src/DucoRobotStatus.cpp src/DucoRobotStatusNode.cpp src/RobotNode.cpp)
add_executable(DucoTrajectoryAction src/DucoTrajectoryAction.cpp)
add_executable(DualTrajectoryAction src/DualTrajectoryAction.cpp)
add_executable(DucoRobotControl src/DucoRobotControl.cpp src/RobotNode.cpp)


ament_target_dependencies(DucoDriver rclcpp rclcpp_action control_msgs sensor_msgs trajectory_msgs std_msgs duco_msg)
ament_target_dependencies(DucoRobotStatus rclcpp rclcpp_action control_msgs sensor_msgs trajectory_msgs std_msgs duco_msg)
ament_target_dependencies(DucoTrajectoryAction rclcpp rclcpp_action control_msgs sensor_msgs trajectory_msgs std_msgs duco_msg)
ament_target_dependencies(DualTrajectoryAction rclcpp rclcpp_action builtin_interfaces control_msgs sensor_msgs trajectory_msgs std_msgs duco_msg)
ament_target_dependencies(DucoRobotControl rclcpp rclcpp_action control_msgs sensor_msgs trajectory_msgs std_msgs duco_msg)

install (TARGETS DucoDriver DESTINATION lib/${PROJECT_NAME}
    )
install (TARGETS DucoRobotStatus DESTINATION lib/${PROJECT_NAME}
    )
install (TARGETS DucoTrajectoryAction DESTINATION lib/${PROJECT_NAME}
    )
install (TARGETS DualTrajectoryAction DESTINATION lib/${PROJECT_NAME}
    )
install (TARGETS DucoRobotControl DESTINATION lib/${PROJECT_NAME}
    )
target_link_libraries(DucoDriver ${CMAKE_CURRENT_SOURCE_DIR}/lib/libDucoCobotAPI.a)
target_link_libraries(DucoRobotStatus ${CMAKE_CURRENT_SOURCE_DIR}/lib/libDucoCobotAPI.a)
target_link_libraries(DucoRobotControl ${CMAKE_CURRENT_SOURCE_DIR}/lib/libDucoCobotAPI.a)

ament_package()
