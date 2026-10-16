// Unit tests for RealFeedbackGate.hpp
//
// This file is intentionally ROS-free apart from the gtest framework: the gate
// is a pure function so the rules can be exercised without a node, an executor
// or a simulated action server.

#include <chrono>
#include <limits>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "duco_ros_driver/RealFeedbackGate.hpp"

using duco_reliability::evaluateRealFeedback;
using duco_reliability::FeedbackPolicy;
using duco_reliability::FeedbackSnapshot;
using duco_reliability::FeedbackVerdict;
using duco_reliability::GoalTarget;
using duco_reliability::StatusSnapshot;
using duco_reliability::SteadyClock;

using namespace std::chrono_literals;

namespace {

// A fixed "now" so every expectation is deterministic.
const SteadyClock::time_point kNow = SteadyClock::time_point{} + 3600s;

const double kNaN = std::numeric_limits<double>::quiet_NaN();
const double kInf = std::numeric_limits<double>::infinity();

/**
 * \brief Builds a scenario where every condition holds, then lets each test
 * break exactly one thing.
 */
struct Scenario {
  GoalTarget goal;
  FeedbackSnapshot feedback;
  StatusSnapshot status;
  FeedbackPolicy policy;

  Scenario()
  {
    goal.joint_names = {"arm_1_joint_1", "arm_1_joint_2"};
    goal.final_positions = {0.5, -0.5};
    goal.feedback_sequence_at_command = 10;
    goal.status_sequence_at_command = 20;

    feedback.joint_names = {"arm_1_joint_1", "arm_1_joint_2"};
    feedback.positions = {0.5, -0.5};
    feedback.velocities = {0.0, 0.0};
    feedback.received_at = kNow - 10ms;
    feedback.sequence = 11;

    status.healthy = true;
    status.received_at = kNow - 10ms;
    status.sequence = 21;
  }

  FeedbackVerdict evaluate() const
  {
    return evaluateRealFeedback(goal, feedback, status, policy, kNow);
  }
};

}  // namespace

// ---------------------------------------------------------------------------
// Baseline
// ---------------------------------------------------------------------------

TEST(RealFeedbackGate, ReachedWhenEverythingIsValid)
{
  EXPECT_EQ(Scenario().evaluate(), FeedbackVerdict::Reached);
}

TEST(RealFeedbackGate, VerdictNamesAreStable)
{
  // The names end up in logs and in test assertions; keep them stable.
  EXPECT_STREQ(duco_reliability::feedbackVerdictName(FeedbackVerdict::Reached), "reached");
  EXPECT_STREQ(
    duco_reliability::feedbackVerdictName(FeedbackVerdict::NoNewStatus), "no_new_status");
  EXPECT_STREQ(
    duco_reliability::feedbackVerdictName(FeedbackVerdict::StaleFeedback), "stale_feedback");
  EXPECT_STREQ(
    duco_reliability::feedbackVerdictName(FeedbackVerdict::UnhealthyStatus), "unhealthy_status");
}

// ---------------------------------------------------------------------------
// Tolerance behaviour, including the boundary and the NaN regression
// ---------------------------------------------------------------------------

// Boundary values must be exactly representable in binary, otherwise
// 0.5 + 0.02 is already slightly larger than 0.02 and the strict ">" comparison
// correctly reports a violation.  0.25 is exact, so the boundary is exact too.
TEST(RealFeedbackGate, PositionExactlyOnToleranceIsReached)
{
  Scenario s;
  s.policy.position_tolerance = 0.25;
  s.feedback.positions[0] = 0.5 + 0.25;
  EXPECT_EQ(s.evaluate(), FeedbackVerdict::Reached);
}

TEST(RealFeedbackGate, PositionOutsideToleranceIsRejected)
{
  Scenario s;
  s.policy.position_tolerance = 0.25;
  s.feedback.positions[0] = 0.5 + 0.25 + 1e-9;
  EXPECT_EQ(s.evaluate(), FeedbackVerdict::PositionOutsideTolerance);
}

TEST(RealFeedbackGate, ToleranceIsStrictlyExclusive)
{
  // Guards the floating-point subtlety above: a tolerance that is not exactly
  // representable makes the nominal boundary land on the rejecting side.
  Scenario s;
  s.feedback.positions[0] = 0.5 + s.policy.position_tolerance;
  EXPECT_EQ(s.evaluate(), FeedbackVerdict::PositionOutsideTolerance);
}

TEST(RealFeedbackGate, VelocityExactlyOnToleranceIsReached)
{
  Scenario s;
  s.policy.velocity_tolerance = 0.25;
  s.feedback.velocities[1] = -0.25;
  EXPECT_EQ(s.evaluate(), FeedbackVerdict::Reached);
}

TEST(RealFeedbackGate, VelocityOutsideToleranceIsRejected)
{
  Scenario s;
  s.policy.velocity_tolerance = 0.25;
  s.feedback.velocities[1] = -0.25 - 1e-9;
  EXPECT_EQ(s.evaluate(), FeedbackVerdict::VelocityOutsideTolerance);
}

// Regression: the previous std::abs(pos - target) > tol test let NaN through,
// because every comparison with NaN is false.  NaN must never count as reached.
TEST(RealFeedbackGate, NaNAndInfPositionsAreRejected)
{
  {
    Scenario s;
    s.feedback.positions[0] = kNaN;
    EXPECT_EQ(s.evaluate(), FeedbackVerdict::InvalidFeedback);
  }
  {
    Scenario s;
    s.feedback.positions[1] = kInf;
    EXPECT_EQ(s.evaluate(), FeedbackVerdict::InvalidFeedback);
  }
}

// Regression: the previous velocity check was skipped entirely when the
// velocity array was empty, so missing velocities silently passed.
TEST(RealFeedbackGate, NaNAndInfVelocitiesAreRejected)
{
  {
    Scenario s;
    s.feedback.velocities[0] = kNaN;
    EXPECT_EQ(s.evaluate(), FeedbackVerdict::InvalidFeedback);
  }
  {
    Scenario s;
    s.feedback.velocities[1] = -kInf;
    EXPECT_EQ(s.evaluate(), FeedbackVerdict::InvalidFeedback);
  }
}

// ---------------------------------------------------------------------------
// Feedback payload shape
// ---------------------------------------------------------------------------

TEST(RealFeedbackGate, MissingVelocitiesAreRejected)
{
  Scenario s;
  s.feedback.velocities.clear();
  EXPECT_EQ(s.evaluate(), FeedbackVerdict::InvalidFeedback);
}

TEST(RealFeedbackGate, EmptyJointNamesAreRejected)
{
  Scenario s;
  s.feedback.joint_names.clear();
  EXPECT_EQ(s.evaluate(), FeedbackVerdict::InvalidFeedback);
}

TEST(RealFeedbackGate, DuplicateFeedbackJointNamesAreRejected)
{
  Scenario s;
  s.feedback.joint_names[1] = s.feedback.joint_names[0];
  EXPECT_EQ(s.evaluate(), FeedbackVerdict::InvalidFeedback);
}

TEST(RealFeedbackGate, EmptyFeedbackJointNameIsRejected)
{
  Scenario s;
  s.feedback.joint_names[1] = "";
  EXPECT_EQ(s.evaluate(), FeedbackVerdict::InvalidFeedback);
}

TEST(RealFeedbackGate, PositionAndNameSizeMismatchIsRejected)
{
  Scenario s;
  s.feedback.positions.push_back(0.0);
  EXPECT_EQ(s.evaluate(), FeedbackVerdict::InvalidFeedback);
}

TEST(RealFeedbackGate, ExtraUnrelatedJointsAreTolerated)
{
  // The vendor publishes a fixed-width array; joints outside the goal are fine
  // as long as they are finite and uniquely named.
  Scenario s;
  s.feedback.joint_names.push_back("arm_1_joint_7");
  s.feedback.positions.push_back(0.0);
  s.feedback.velocities.push_back(0.0);
  EXPECT_EQ(s.evaluate(), FeedbackVerdict::Reached);
}

TEST(RealFeedbackGate, MissingGoalJointIsRejected)
{
  Scenario s;
  s.feedback.joint_names = {"arm_1_joint_1", "arm_1_joint_3"};
  EXPECT_EQ(s.evaluate(), FeedbackVerdict::MissingJoint);
}

// ---------------------------------------------------------------------------
// Goal target shape
// ---------------------------------------------------------------------------

TEST(RealFeedbackGate, EmptyTargetIsRejected)
{
  Scenario s;
  s.goal.joint_names.clear();
  s.goal.final_positions.clear();
  EXPECT_EQ(s.evaluate(), FeedbackVerdict::InvalidTarget);
}

TEST(RealFeedbackGate, TargetDimensionMismatchIsRejected)
{
  Scenario s;
  s.goal.final_positions.pop_back();
  EXPECT_EQ(s.evaluate(), FeedbackVerdict::InvalidTarget);
}

TEST(RealFeedbackGate, NonFiniteTargetIsRejected)
{
  Scenario s;
  s.goal.final_positions[0] = kNaN;
  EXPECT_EQ(s.evaluate(), FeedbackVerdict::InvalidTarget);
}

TEST(RealFeedbackGate, DuplicateTargetJointNamesAreRejected)
{
  Scenario s;
  s.goal.joint_names[1] = s.goal.joint_names[0];
  EXPECT_EQ(s.evaluate(), FeedbackVerdict::InvalidTarget);
}

// ---------------------------------------------------------------------------
// Sequence ordering: data that predates the command
// ---------------------------------------------------------------------------

TEST(RealFeedbackGate, FeedbackFromBeforeTheCommandIsRejected)
{
  Scenario s;
  s.feedback.sequence = s.goal.feedback_sequence_at_command;
  EXPECT_EQ(s.evaluate(), FeedbackVerdict::NoNewFeedback);
}

TEST(RealFeedbackGate, StatusFromBeforeTheCommandIsRejected)
{
  Scenario s;
  s.status.sequence = s.goal.status_sequence_at_command;
  EXPECT_EQ(s.evaluate(), FeedbackVerdict::NoNewStatus);
}

TEST(RealFeedbackGate, NeverReceivedFeedbackIsRejected)
{
  Scenario s;
  s.feedback.received_at = SteadyClock::time_point{};
  EXPECT_EQ(s.evaluate(), FeedbackVerdict::NoNewFeedback);
}

TEST(RealFeedbackGate, NeverReceivedStatusIsRejected)
{
  Scenario s;
  s.status.received_at = SteadyClock::time_point{};
  EXPECT_EQ(s.evaluate(), FeedbackVerdict::NoNewStatus);
}

// ---------------------------------------------------------------------------
// Freshness
// ---------------------------------------------------------------------------

TEST(RealFeedbackGate, StaleFeedbackIsRejected)
{
  Scenario s;
  s.feedback.received_at = kNow - s.policy.feedback_max_age - 1ms;
  EXPECT_EQ(s.evaluate(), FeedbackVerdict::StaleFeedback);
}

TEST(RealFeedbackGate, StaleStatusIsRejected)
{
  Scenario s;
  s.status.received_at = kNow - s.policy.status_max_age - 1ms;
  EXPECT_EQ(s.evaluate(), FeedbackVerdict::StaleStatus);
}

TEST(RealFeedbackGate, FeedbackExactlyAtMaxAgeIsReached)
{
  Scenario s;
  s.feedback.received_at = kNow - s.policy.feedback_max_age;
  s.status.received_at = kNow - s.policy.status_max_age;
  EXPECT_EQ(s.evaluate(), FeedbackVerdict::Reached);
}

TEST(RealFeedbackGate, TimestampInTheFutureIsRejected)
{
  Scenario s;
  s.feedback.received_at = kNow + 1s;
  EXPECT_EQ(s.evaluate(), FeedbackVerdict::StaleFeedback);
}

// This is the exact failure mode reported against DucoRobotStatus: after the
// controller disconnects the feedback topic keeps publishing cached values, so
// the feedback sequence advances while the status topic goes silent.
TEST(RealFeedbackGate, CachedFeedbackWhileStatusIsSilentIsRejected)
{
  Scenario s;
  s.feedback.sequence = s.goal.feedback_sequence_at_command + 500;  // still streaming
  s.feedback.received_at = kNow - 1ms;                              // looks brand new
  s.status.sequence = s.goal.status_sequence_at_command;            // status never advanced
  s.status.received_at = kNow - 30s;                                // last status is old
  EXPECT_EQ(s.evaluate(), FeedbackVerdict::NoNewStatus);
}

TEST(RealFeedbackGate, FreshFeedbackButAgedOutStatusIsRejected)
{
  Scenario s;
  s.feedback.sequence = s.goal.feedback_sequence_at_command + 500;
  s.feedback.received_at = kNow - 1ms;
  s.status.sequence = s.goal.status_sequence_at_command + 500;
  s.status.received_at = kNow - s.policy.status_max_age - 1ms;
  EXPECT_EQ(s.evaluate(), FeedbackVerdict::StaleStatus);
}

// ---------------------------------------------------------------------------
// Robot health
// ---------------------------------------------------------------------------

TEST(RealFeedbackGate, EmergencyStopIsRejected)
{
  Scenario s;
  s.status.healthy = false;
  EXPECT_EQ(s.evaluate(), FeedbackVerdict::UnhealthyStatus);
}

TEST(RealFeedbackGate, UnhealthyStatusWinsOverPositionTolerance)
{
  // Health is checked before geometry so a fault is never masked by a joint
  // that happens to sit close to the goal.
  Scenario s;
  s.status.healthy = false;
  s.feedback.positions[0] = 99.0;
  EXPECT_EQ(s.evaluate(), FeedbackVerdict::UnhealthyStatus);
}

// ---------------------------------------------------------------------------
// Policy validation
// ---------------------------------------------------------------------------

TEST(RealFeedbackGate, NonFiniteToleranceIsInvalidPolicy)
{
  {
    Scenario s;
    s.policy.position_tolerance = kNaN;
    EXPECT_EQ(s.evaluate(), FeedbackVerdict::InvalidPolicy);
  }
  {
    Scenario s;
    s.policy.velocity_tolerance = kInf;
    EXPECT_EQ(s.evaluate(), FeedbackVerdict::InvalidPolicy);
  }
}

TEST(RealFeedbackGate, NegativeToleranceIsInvalidPolicy)
{
  Scenario s;
  s.policy.position_tolerance = -0.01;
  EXPECT_EQ(s.evaluate(), FeedbackVerdict::InvalidPolicy);
}

TEST(RealFeedbackGate, ZeroMaxAgeIsInvalidPolicy)
{
  {
    Scenario s;
    s.policy.feedback_max_age = 0ms;
    EXPECT_EQ(s.evaluate(), FeedbackVerdict::InvalidPolicy);
  }
  {
    Scenario s;
    s.policy.status_max_age = 0ms;
    EXPECT_EQ(s.evaluate(), FeedbackVerdict::InvalidPolicy);
  }
}
