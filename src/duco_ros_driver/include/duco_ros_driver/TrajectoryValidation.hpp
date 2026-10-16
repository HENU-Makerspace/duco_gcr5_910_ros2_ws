// TrajectoryValidation.hpp
//
// ROS-independent trajectory validation and time scaling extracted from
// DualTrajectoryAction so that the rules can be unit tested directly.
//
// Only the message types are reused; no node, executor or parameter server is
// required.  The behaviour is identical to the previous in-class
// implementation.

#ifndef DUCO_ROS_DRIVER_TRAJECTORY_VALIDATION_HPP
#define DUCO_ROS_DRIVER_TRAJECTORY_VALIDATION_HPP

#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <unordered_set>
#include <vector>

#include <builtin_interfaces/msg/duration.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace duco_reliability {

/**
 * \brief Configuration the trajectory rules depend on.
 */
struct TrajectoryValidationPolicy {
  bool enforce_expected_joint_names{true};
  std::vector<std::string> expected_joint_names;
  std::vector<double> joint_position_min;
  std::vector<double> joint_position_max;
};

/**
 * \brief Convert a builtin duration to nanoseconds, rejecting negative or
 * out-of-range fields.
 */
inline bool durationNanoseconds(
  const builtin_interfaces::msg::Duration & duration, int64_t & nanoseconds,
  std::string & error)
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

/**
 * \brief Validate joint names, dimensions, finiteness, timing and limits.
 */
inline bool validateTrajectory(
  const trajectory_msgs::msg::JointTrajectory & trajectory,
  const TrajectoryValidationPolicy & policy, std::string & error)
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
  if (policy.enforce_expected_joint_names &&
    trajectory.joint_names != policy.expected_joint_names)
  {
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

    if (!policy.joint_position_min.empty()) {
      for (size_t joint_index = 0; joint_index < joint_count; ++joint_index) {
        if (point.positions[joint_index] < policy.joint_position_min[joint_index] ||
          point.positions[joint_index] > policy.joint_position_max[joint_index])
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

/**
 * \brief Scale point timestamps, velocities and accelerations in place.
 *
 * Velocities scale by 1/scale and accelerations by 1/scale^2 so that the motion
 * profile keeps its shape.  Strictly increasing timestamps and 32-bit duration
 * limits are re-checked after scaling.
 */
inline bool scaleTrajectoryTime(
  trajectory_msgs::msg::JointTrajectory & trajectory, double scale, std::string & error)
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

}  // namespace duco_reliability

#endif  // DUCO_ROS_DRIVER_TRAJECTORY_VALIDATION_HPP
