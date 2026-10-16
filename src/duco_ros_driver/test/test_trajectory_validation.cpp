// Unit tests for TrajectoryValidation.hpp
//
// The rules previously lived inside DualTrajectoryAction as private members and
// could not be tested at all.  They are pure functions now, so the whole matrix
// runs without a node or an executor.  Only the message types are ROS-derived.

#include <limits>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <builtin_interfaces/msg/duration.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "duco_ros_driver/TrajectoryValidation.hpp"

using duco_reliability::durationNanoseconds;
using duco_reliability::scaleTrajectoryTime;
using duco_reliability::TrajectoryValidationPolicy;
using duco_reliability::validateTrajectory;

using JointTrajectory = trajectory_msgs::msg::JointTrajectory;
using JointTrajectoryPoint = trajectory_msgs::msg::JointTrajectoryPoint;

namespace {

const double kNaN = std::numeric_limits<double>::quiet_NaN();
const double kInf = std::numeric_limits<double>::infinity();

const std::vector<std::string> kNames = {"arm_1_joint_1", "arm_1_joint_2"};

builtin_interfaces::msg::Duration makeDuration(int32_t sec, uint32_t nanosec = 0u)
{
  builtin_interfaces::msg::Duration d;
  d.sec = sec;
  d.nanosec = nanosec;
  return d;
}

JointTrajectoryPoint makePoint(
  double a, double b, int32_t sec, uint32_t nanosec = 0u)
{
  JointTrajectoryPoint point;
  point.positions = {a, b};
  point.time_from_start = makeDuration(sec, nanosec);
  return point;
}

/**
 * \brief A two-point trajectory that passes every rule.
 */
JointTrajectory makeValidTrajectory()
{
  JointTrajectory trajectory;
  trajectory.joint_names = kNames;
  trajectory.points.push_back(makePoint(0.1, -0.1, 1));
  trajectory.points.push_back(makePoint(0.2, -0.2, 2));
  return trajectory;
}

TrajectoryValidationPolicy makeDefaultPolicy()
{
  TrajectoryValidationPolicy policy;
  policy.enforce_expected_joint_names = false;
  return policy;
}

}  // namespace

// ---------------------------------------------------------------------------
// durationNanoseconds
// ---------------------------------------------------------------------------

TEST(TrajectoryValidation, DurationConvertsToNanoseconds)
{
  int64_t ns = 0;
  std::string error;
  ASSERT_TRUE(durationNanoseconds(makeDuration(2, 500000000u), ns, error));
  EXPECT_EQ(ns, 2500000000LL);
  EXPECT_TRUE(error.empty());
}

TEST(TrajectoryValidation, NegativeDurationSecondsAreRejected)
{
  int64_t ns = 0;
  std::string error;
  EXPECT_FALSE(durationNanoseconds(makeDuration(-1, 0u), ns, error));
  EXPECT_FALSE(error.empty());
}

TEST(TrajectoryValidation, OutOfRangeNanosecondsAreRejected)
{
  int64_t ns = 0;
  std::string error;
  EXPECT_FALSE(durationNanoseconds(makeDuration(0, 1000000000u), ns, error));
  EXPECT_FALSE(error.empty());
}

// ---------------------------------------------------------------------------
// validateTrajectory: accepted input
// ---------------------------------------------------------------------------

TEST(TrajectoryValidation, ValidTrajectoryPasses)
{
  std::string error;
  EXPECT_TRUE(validateTrajectory(makeValidTrajectory(), makeDefaultPolicy(), error));
  EXPECT_TRUE(error.empty());
}

TEST(TrajectoryValidation, SinglePointTrajectoryIsStructurallyValid)
{
  // A single point is a legal FollowJointTrajectory; the proxy layer is what
  // refuses to forward it to the real driver, because DucoDriver erases the
  // first point and would enqueue an empty path.
  JointTrajectory trajectory;
  trajectory.joint_names = kNames;
  trajectory.points.push_back(makePoint(0.1, -0.1, 1));
  std::string error;
  EXPECT_TRUE(validateTrajectory(trajectory, makeDefaultPolicy(), error));
}

TEST(TrajectoryValidation, OptionalArraysMatchingJointCountPass)
{
  auto trajectory = makeValidTrajectory();
  trajectory.points[0].velocities = {0.1, -0.1};
  trajectory.points[0].accelerations = {0.01, -0.01};
  trajectory.points[0].effort = {1.0, 1.0};
  std::string error;
  EXPECT_TRUE(validateTrajectory(trajectory, makeDefaultPolicy(), error));
}

// ---------------------------------------------------------------------------
// validateTrajectory: joint names
// ---------------------------------------------------------------------------

TEST(TrajectoryValidation, EmptyJointNamesAreRejected)
{
  auto trajectory = makeValidTrajectory();
  trajectory.joint_names.clear();
  std::string error;
  EXPECT_FALSE(validateTrajectory(trajectory, makeDefaultPolicy(), error));
  EXPECT_EQ(error, "joint_names is empty");
}

TEST(TrajectoryValidation, EmptyJointNameEntryIsRejected)
{
  auto trajectory = makeValidTrajectory();
  trajectory.joint_names[1] = "";
  std::string error;
  EXPECT_FALSE(validateTrajectory(trajectory, makeDefaultPolicy(), error));
  EXPECT_EQ(error, "joint_names contains an empty name");
}

TEST(TrajectoryValidation, DuplicateJointNamesAreRejected)
{
  auto trajectory = makeValidTrajectory();
  trajectory.joint_names[1] = trajectory.joint_names[0];
  std::string error;
  EXPECT_FALSE(validateTrajectory(trajectory, makeDefaultPolicy(), error));
  EXPECT_EQ(error, "joint_names contains duplicate name: arm_1_joint_1");
}

TEST(TrajectoryValidation, ExpectedJointNamesAreEnforcedWhenEnabled)
{
  auto trajectory = makeValidTrajectory();
  TrajectoryValidationPolicy policy;
  policy.enforce_expected_joint_names = true;
  policy.expected_joint_names = {"arm_1_joint_1", "arm_1_joint_3"};
  std::string error;
  EXPECT_FALSE(validateTrajectory(trajectory, policy, error));
  EXPECT_EQ(error, "joint_names do not exactly match expected_joint_names");
}

TEST(TrajectoryValidation, ExpectedJointNamesAreIgnoredWhenDisabled)
{
  auto trajectory = makeValidTrajectory();
  TrajectoryValidationPolicy policy;
  policy.enforce_expected_joint_names = false;
  policy.expected_joint_names = {"something_else"};
  std::string error;
  EXPECT_TRUE(validateTrajectory(trajectory, policy, error));
}

// ---------------------------------------------------------------------------
// validateTrajectory: points and dimensions
// ---------------------------------------------------------------------------

TEST(TrajectoryValidation, EmptyPointListIsRejected)
{
  auto trajectory = makeValidTrajectory();
  trajectory.points.clear();
  std::string error;
  EXPECT_FALSE(validateTrajectory(trajectory, makeDefaultPolicy(), error));
  EXPECT_EQ(error, "trajectory points is empty");
}

TEST(TrajectoryValidation, PositionDimensionMismatchIsRejected)
{
  auto trajectory = makeValidTrajectory();
  trajectory.points[0].positions.push_back(0.0);
  std::string error;
  EXPECT_FALSE(validateTrajectory(trajectory, makeDefaultPolicy(), error));
  EXPECT_NE(error.find("positions dimension is 3, expected 2"), std::string::npos);
}

TEST(TrajectoryValidation, OptionalArrayDimensionMismatchIsRejected)
{
  auto trajectory = makeValidTrajectory();
  trajectory.points[1].velocities = {0.1};
  std::string error;
  EXPECT_FALSE(validateTrajectory(trajectory, makeDefaultPolicy(), error));
  EXPECT_NE(error.find("optional point array dimension does not match"), std::string::npos);
}

// ---------------------------------------------------------------------------
// validateTrajectory: non-finite values
// ---------------------------------------------------------------------------

TEST(TrajectoryValidation, NaNAndInfValuesAreRejected)
{
  {
    auto trajectory = makeValidTrajectory();
    trajectory.points[0].positions[0] = kNaN;
    std::string error;
    EXPECT_FALSE(validateTrajectory(trajectory, makeDefaultPolicy(), error));
    EXPECT_NE(error.find("positions contains NaN or Inf"), std::string::npos);
  }
  {
    auto trajectory = makeValidTrajectory();
    trajectory.points[1].velocities = {kInf, 0.0};
    std::string error;
    EXPECT_FALSE(validateTrajectory(trajectory, makeDefaultPolicy(), error));
    EXPECT_NE(error.find("velocities contains NaN or Inf"), std::string::npos);
  }
  {
    auto trajectory = makeValidTrajectory();
    trajectory.points[1].accelerations = {0.0, -kInf};
    std::string error;
    EXPECT_FALSE(validateTrajectory(trajectory, makeDefaultPolicy(), error));
    EXPECT_NE(error.find("accelerations contains NaN or Inf"), std::string::npos);
  }
}

// ---------------------------------------------------------------------------
// validateTrajectory: timing
// ---------------------------------------------------------------------------

TEST(TrajectoryValidation, NonIncreasingTimestampsAreRejected)
{
  {
    auto trajectory = makeValidTrajectory();
    trajectory.points[1].time_from_start = makeDuration(1);
    std::string error;
    EXPECT_FALSE(validateTrajectory(trajectory, makeDefaultPolicy(), error));
    EXPECT_NE(error.find("strictly increasing"), std::string::npos);
  }
  {
    auto trajectory = makeValidTrajectory();
    trajectory.points[1].time_from_start = makeDuration(0, 500000000u);
    std::string error;
    EXPECT_FALSE(validateTrajectory(trajectory, makeDefaultPolicy(), error));
    EXPECT_NE(error.find("strictly increasing"), std::string::npos);
  }
}

TEST(TrajectoryValidation, InvalidPointDurationIsRejectedWithPointContext)
{
  auto trajectory = makeValidTrajectory();
  trajectory.points[1].time_from_start = makeDuration(-5);
  std::string error;
  EXPECT_FALSE(validateTrajectory(trajectory, makeDefaultPolicy(), error));
  EXPECT_EQ(error.find("point 1:"), 0u);
}

// ---------------------------------------------------------------------------
// validateTrajectory: joint limits
// ---------------------------------------------------------------------------

TEST(TrajectoryValidation, PositionLimitViolationIsRejected)
{
  auto trajectory = makeValidTrajectory();
  TrajectoryValidationPolicy policy = makeDefaultPolicy();
  policy.joint_position_min = {-0.15, -0.15};
  policy.joint_position_max = {0.15, 0.15};
  std::string error;
  EXPECT_FALSE(validateTrajectory(trajectory, policy, error));
  EXPECT_NE(error.find("position limit violated at point 1"), std::string::npos);
}

TEST(TrajectoryValidation, PositionInsideLimitsIsAccepted)
{
  auto trajectory = makeValidTrajectory();
  TrajectoryValidationPolicy policy = makeDefaultPolicy();
  policy.joint_position_min = {-1.0, -1.0};
  policy.joint_position_max = {1.0, 1.0};
  std::string error;
  EXPECT_TRUE(validateTrajectory(trajectory, policy, error));
}

TEST(TrajectoryValidation, LimitsAreIgnoredWhenEmpty)
{
  auto trajectory = makeValidTrajectory();
  TrajectoryValidationPolicy policy = makeDefaultPolicy();
  policy.joint_position_min.clear();
  policy.joint_position_max.clear();
  std::string error;
  EXPECT_TRUE(validateTrajectory(trajectory, policy, error));
}

// ---------------------------------------------------------------------------
// scaleTrajectoryTime
// ---------------------------------------------------------------------------

TEST(TrajectoryValidation, ScalingSlowsTimingAndScalesDerivatives)
{
  auto trajectory = makeValidTrajectory();
  trajectory.points[0].velocities = {0.4, -0.4};
  trajectory.points[0].accelerations = {0.8, -0.8};

  std::string error;
  ASSERT_TRUE(scaleTrajectoryTime(trajectory, 2.0, error));

  // Times double.
  EXPECT_EQ(trajectory.points[0].time_from_start.sec, 2);
  EXPECT_EQ(trajectory.points[1].time_from_start.sec, 4);
  // Velocities halve, accelerations quarter.
  EXPECT_DOUBLE_EQ(trajectory.points[0].velocities[0], 0.2);
  EXPECT_DOUBLE_EQ(trajectory.points[0].accelerations[0], 0.2);
}

TEST(TrajectoryValidation, ScalingPreservesStrictlyIncreasingTimestamps)
{
  auto trajectory = makeValidTrajectory();
  std::string error;
  ASSERT_TRUE(scaleTrajectoryTime(trajectory, 0.5, error));
  int64_t first = 0;
  int64_t second = 0;
  ASSERT_TRUE(durationNanoseconds(trajectory.points[0].time_from_start, first, error));
  ASSERT_TRUE(durationNanoseconds(trajectory.points[1].time_from_start, second, error));
  EXPECT_LT(first, second);
}

TEST(TrajectoryValidation, InvalidScaleFactorsAreRejected)
{
  const double bad_scales[] = {0.0, -1.0, kNaN, kInf, -kInf};
  for (const double scale : bad_scales) {
    auto trajectory = makeValidTrajectory();
    std::string error;
    EXPECT_FALSE(scaleTrajectoryTime(trajectory, scale, error)) << "scale=" << scale;
    EXPECT_EQ(error, "sim_time_scale must be finite and > 0");
  }
}

TEST(TrajectoryValidation, ScalingOverflowIsRejected)
{
  JointTrajectory trajectory;
  trajectory.joint_names = kNames;
  trajectory.points.push_back(makePoint(0.1, -0.1, 2000000000));

  std::string error;
  EXPECT_FALSE(scaleTrajectoryTime(trajectory, 2.0, error));
  EXPECT_NE(error.find("overflows"), std::string::npos);
}

TEST(TrajectoryValidation, ScalingThatCollapsesTimestampsIsRejected)
{
  // Two distinct times round to the same nanosecond value after scaling, so the
  // post-scaling ordering check must fail rather than emit a degenerate
  // trajectory.
  auto trajectory = makeValidTrajectory();
  std::string error;
  EXPECT_FALSE(scaleTrajectoryTime(trajectory, 1e-12, error));
  EXPECT_NE(error.find("not strictly increasing"), std::string::npos);
}

TEST(TrajectoryValidation, ScalingNonFiniteDerivativesIsRejected)
{
  auto trajectory = makeValidTrajectory();
  trajectory.points[0].velocities = {std::numeric_limits<double>::max(), 0.0};
  std::string error;
  // Scaling by 1/1e-300 would overflow; use a large inverse factor instead of a
  // tiny scale so the original duration does not collapse first.
  EXPECT_FALSE(scaleTrajectoryTime(trajectory, 1e-300, error));
  EXPECT_FALSE(error.empty());
}
