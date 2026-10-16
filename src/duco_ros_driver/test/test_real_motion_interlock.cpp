// Unit tests for RealMotionInterlock.hpp
//
// ROS-free: the interlock is a pure state machine, so the rules that decide
// whether a new *real* trajectory may be sent can be exercised directly.

#include <string>

#include <gtest/gtest.h>

#include "duco_ros_driver/RealMotionInterlock.hpp"

using duco_reliability::RealMotionInterlock;
using duco_reliability::RealMotionState;

TEST(RealMotionInterlock, StartsAllowed)
{
  RealMotionInterlock interlock;
  EXPECT_EQ(interlock.state(), RealMotionState::Allowed);
  EXPECT_FALSE(interlock.latched());
  EXPECT_FALSE(interlock.blocksRealMotion());
  EXPECT_TRUE(interlock.reason().empty());
}

TEST(RealMotionInterlock, StateNamesAreStable)
{
  EXPECT_STREQ(
    duco_reliability::realMotionStateName(RealMotionState::Allowed), "allowed");
  EXPECT_STREQ(
    duco_reliability::realMotionStateName(RealMotionState::Executing), "executing");
  EXPECT_STREQ(
    duco_reliability::realMotionStateName(RealMotionState::AwaitingConfirmation),
    "awaiting_confirmation");
  EXPECT_STREQ(
    duco_reliability::realMotionStateName(RealMotionState::Uncertain), "uncertain");
  EXPECT_STREQ(
    duco_reliability::realMotionStateName(RealMotionState::Forbidden), "forbidden");
}

TEST(RealMotionInterlock, PublishedCommandIsExecutingAndNotLatched)
{
  RealMotionInterlock interlock;
  ASSERT_TRUE(interlock.onCommandPublished(7, "goal-a"));
  EXPECT_EQ(interlock.state(), RealMotionState::Executing);
  EXPECT_EQ(interlock.generation(), 7u);
  EXPECT_EQ(interlock.goalId(), "goal-a");
  EXPECT_FALSE(interlock.latched());
  // Executing must not block: the running goal still has to publish its stop.
  EXPECT_FALSE(interlock.blocksRealMotion());
}

TEST(RealMotionInterlock, ConfirmedCompletionOfTheCurrentGenerationClearsIt)
{
  RealMotionInterlock interlock;
  ASSERT_TRUE(interlock.onCommandPublished(7, "goal-a"));
  EXPECT_TRUE(interlock.onConfirmedCompletion(7));
  EXPECT_EQ(interlock.state(), RealMotionState::Allowed);
  EXPECT_FALSE(interlock.latched());
}

TEST(RealMotionInterlock, AStaleGenerationCannotClearTheInterlock)
{
  RealMotionInterlock interlock;
  ASSERT_TRUE(interlock.onCommandPublished(7, "goal-a"));
  // A late callback from an older goal must not release the current motion.
  EXPECT_FALSE(interlock.onConfirmedCompletion(6));
  EXPECT_EQ(interlock.state(), RealMotionState::Executing);
  EXPECT_EQ(interlock.generation(), 7u);
}

TEST(RealMotionInterlock, GoalEndingWithoutConfirmationLatches)
{
  RealMotionInterlock interlock;
  ASSERT_TRUE(interlock.onCommandPublished(1, "goal-a"));
  ASSERT_TRUE(interlock.onGoalFinishedUnconfirmed(1, "goal-a", "trajectory ended early"));
  EXPECT_EQ(interlock.state(), RealMotionState::AwaitingConfirmation);
  EXPECT_TRUE(interlock.latched());
  EXPECT_TRUE(interlock.blocksRealMotion());
  EXPECT_EQ(interlock.reason(), "trajectory ended early");
}

TEST(RealMotionInterlock, LatchedInterlockRefusesNewCommands)
{
  RealMotionInterlock interlock;
  ASSERT_TRUE(interlock.onCommandPublished(1, "goal-a"));
  ASSERT_TRUE(interlock.onGoalFinishedUnconfirmed(1, "goal-a", "unconfirmed"));
  EXPECT_FALSE(interlock.onCommandPublished(2, "goal-b"))
    << "a latched interlock must not accept a new real trajectory";
  EXPECT_EQ(interlock.state(), RealMotionState::AwaitingConfirmation);
  EXPECT_EQ(interlock.goalId(), "goal-a");
}

TEST(RealMotionInterlock, UncertaintyEscalatesFromAwaitingConfirmation)
{
  RealMotionInterlock interlock;
  ASSERT_TRUE(interlock.onCommandPublished(1, "goal-a"));
  ASSERT_TRUE(interlock.onGoalFinishedUnconfirmed(1, "goal-a", "ended early"));
  ASSERT_TRUE(interlock.onUncertain(1, "goal-a", "real feedback was lost"));
  EXPECT_EQ(interlock.state(), RealMotionState::Uncertain);
  EXPECT_EQ(interlock.reason(), "real feedback was lost");
}

TEST(RealMotionInterlock, FaultIsTheMostSevereAndIsSticky)
{
  RealMotionInterlock interlock;
  ASSERT_TRUE(interlock.onCommandPublished(1, "goal-a"));
  ASSERT_TRUE(interlock.onFault("goal-a", "emergency stop is active"));
  EXPECT_EQ(interlock.state(), RealMotionState::Forbidden);
  EXPECT_TRUE(interlock.blocksRealMotion());

  // A later, milder observation must not downgrade the fault.
  EXPECT_FALSE(interlock.onGoalFinishedUnconfirmed(1, "goal-a", "ended early"));
  EXPECT_FALSE(interlock.onUncertain(1, "goal-a", "feedback lost"));
  EXPECT_FALSE(interlock.onConfirmedCompletion(1));
  EXPECT_EQ(interlock.state(), RealMotionState::Forbidden);
  EXPECT_EQ(interlock.reason(), "emergency stop is active");
}

TEST(RealMotionInterlock, FaultKeepsTheFirstReasonWhenAlreadyForbidden)
{
  RealMotionInterlock interlock;
  ASSERT_TRUE(interlock.onFault("goal-a", "emergency stop is active"));
  EXPECT_FALSE(interlock.onFault("goal-a", "collision"));
  EXPECT_EQ(interlock.reason(), "emergency stop is active");
}

TEST(RealMotionInterlock, FaultCanBeRaisedWhileIdle)
{
  // A fault reported before any command still forbids real motion.
  RealMotionInterlock interlock;
  ASSERT_TRUE(interlock.onFault("-", "robot error code 42"));
  EXPECT_EQ(interlock.state(), RealMotionState::Forbidden);
  EXPECT_TRUE(interlock.blocksRealMotion());
  EXPECT_FALSE(interlock.onCommandPublished(1, "goal-a"));
}

TEST(RealMotionInterlock, OnlyOperatorRecoveryClearsALatchedState)
{
  RealMotionInterlock interlock;
  ASSERT_TRUE(interlock.onFault("goal-a", "collision"));
  ASSERT_TRUE(interlock.latched());

  EXPECT_TRUE(interlock.onOperatorRecovery());
  EXPECT_EQ(interlock.state(), RealMotionState::Allowed);
  EXPECT_FALSE(interlock.latched());
  EXPECT_FALSE(interlock.blocksRealMotion());
  EXPECT_TRUE(interlock.reason().empty());
  EXPECT_TRUE(interlock.goalId().empty());
}

TEST(RealMotionInterlock, RecoveryOnAnAllowedInterlockReportsNoChange)
{
  RealMotionInterlock interlock;
  EXPECT_FALSE(interlock.onOperatorRecovery());
}

TEST(RealMotionInterlock, RecoveryAlsoReleasesAnExecutingInterlock)
{
  // The operator explicitly resolves a command whose state can no longer be
  // established, so a command that never reported a result must not wedge the
  // channel forever either.
  RealMotionInterlock interlock;
  ASSERT_TRUE(interlock.onCommandPublished(3, "goal-a"));
  EXPECT_TRUE(interlock.onOperatorRecovery());
  EXPECT_EQ(interlock.state(), RealMotionState::Allowed);
}
