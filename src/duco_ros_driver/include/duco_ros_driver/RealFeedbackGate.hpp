// RealFeedbackGate.hpp
//
// Pure (ROS-independent) validation for the feedback used by
// DualTrajectoryAction to decide whether the physical DUCO arm has reached a
// commanded goal.
//
// The vendor driver (DucoRobotStatus) keeps publishing the last cached joint
// positions and velocities after the controller connection drops, so
// "a feedback message arrived" is NOT evidence that the data is current or
// trustworthy.  This gate therefore requires:
//
//   * a robot status message received *after* the command was published,
//   * fresh timestamps for both feedback and status,
//   * a healthy status (no E-stop, no collision, no robot error),
//   * complete, unique, correctly sized and finite joint data,
//   * all joints inside the position and velocity tolerances.
//
// A Reached verdict only means that valid feedback satisfies the numeric
// tolerances.  It is NOT a vendor confirmation that the controller finished the
// trajectory or that the robot performed a safe stop.
//
// Every check returns a distinct verdict so that "why was completion refused"
// can be answered without guessing, and so that a caller can never average a
// NaN, a missing array or a duplicated joint name into a "reached" answer.

#ifndef DUCO_ROS_DRIVER_REAL_FEEDBACK_GATE_HPP
#define DUCO_ROS_DRIVER_REAL_FEEDBACK_GATE_HPP

#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace duco_reliability
{

using SteadyClock = std::chrono::steady_clock;

/**
 * \brief One received real-robot joint feedback sample.
 *
 * \p sequence is a locally maintained counter that increments for every
 * received message, and \p received_at is the local steady_clock time of
 * arrival.  Neither is provided by the vendor message itself.
 */
struct FeedbackSnapshot {
  std::vector<std::string> joint_names;
  std::vector<double> positions;
  std::vector<double> velocities;
  SteadyClock::time_point received_at{};
  std::uint64_t sequence{0};
};

/**
 * \brief Robot status reduced to what the gate needs.
 *
 * \p healthy is false when the robot reports an E-stop, a collision or a
 * non-zero error code.
 */
struct StatusSnapshot {
  bool healthy{false};
  SteadyClock::time_point received_at{};
  std::uint64_t sequence{0};
};

/**
 * \brief Goal target plus the counters observed when the command was published.
 */
struct GoalTarget {
  std::vector<std::string> joint_names;
  std::vector<double> final_positions;
  std::uint64_t feedback_sequence_at_command{0};
  std::uint64_t status_sequence_at_command{0};
};

struct FeedbackPolicy {
  double position_tolerance{0.02};
  double velocity_tolerance{0.05};
  std::chrono::milliseconds feedback_max_age{1000};
  std::chrono::milliseconds status_max_age{1000};
  /**
   * \brief Whether the velocity array is part of the completion decision.
   *
   * When true a missing or incomplete velocity array is a hard failure: an
   * absent array must never be treated as "velocity is zero".  When false the
   * velocity test is skipped entirely for an absent array (it is still applied
   * to any array that is present, and a partial array is still rejected).
   */
  bool require_velocity{true};
};

enum class FeedbackVerdict {
  Reached,
  InvalidPolicy,
  NoNewStatus,
  StaleStatus,
  UnhealthyStatus,
  NoNewFeedback,
  StaleFeedback,
  InvalidTarget,
  InvalidFeedbackDimensions,
  EmptyJointName,
  DuplicateJointName,
  NonFinitePosition,
  NonFiniteVelocity,
  MissingJoint,
  PositionOutsideTolerance,
  VelocityOutsideTolerance
};

inline const char * feedbackVerdictName(FeedbackVerdict verdict) noexcept
{
  switch (verdict) {
    case FeedbackVerdict::Reached: return "reached";
    case FeedbackVerdict::InvalidPolicy: return "invalid_policy";
    case FeedbackVerdict::NoNewStatus: return "no_new_status";
    case FeedbackVerdict::StaleStatus: return "stale_status";
    case FeedbackVerdict::UnhealthyStatus: return "unhealthy_status";
    case FeedbackVerdict::NoNewFeedback: return "no_new_feedback";
    case FeedbackVerdict::StaleFeedback: return "stale_feedback";
    case FeedbackVerdict::InvalidTarget: return "invalid_target";
    case FeedbackVerdict::InvalidFeedbackDimensions: return "invalid_feedback_dimensions";
    case FeedbackVerdict::EmptyJointName: return "empty_joint_name";
    case FeedbackVerdict::DuplicateJointName: return "duplicate_joint_name";
    case FeedbackVerdict::NonFinitePosition: return "non_finite_position";
    case FeedbackVerdict::NonFiniteVelocity: return "non_finite_velocity";
    case FeedbackVerdict::MissingJoint: return "missing_joint";
    case FeedbackVerdict::PositionOutsideTolerance: return "position_outside_tolerance";
    case FeedbackVerdict::VelocityOutsideTolerance: return "velocity_outside_tolerance";
  }
  return "unknown";
}

/**
 * \brief Decide whether valid real-robot feedback supports a "reached" claim.
 *
 * Status is evaluated before feedback on purpose: the vendor status topic stops
 * publishing when the controller connection drops, while the feedback topic
 * keeps publishing cached values, so status freshness is the cheaper and more
 * reliable liveness signal.
 */
inline FeedbackVerdict evaluateRealFeedback(
  const GoalTarget & goal,
  const FeedbackSnapshot & feedback,
  const StatusSnapshot & status,
  const FeedbackPolicy & policy,
  SteadyClock::time_point now)
{
  if (!std::isfinite(policy.position_tolerance) || policy.position_tolerance < 0.0 ||
    !std::isfinite(policy.velocity_tolerance) || policy.velocity_tolerance < 0.0 ||
    policy.feedback_max_age.count() <= 0 || policy.status_max_age.count() <= 0)
  {
    return FeedbackVerdict::InvalidPolicy;
  }

  if (status.sequence <= goal.status_sequence_at_command ||
    status.received_at == SteadyClock::time_point{})
  {
    return FeedbackVerdict::NoNewStatus;
  }
  if (now < status.received_at || now - status.received_at > policy.status_max_age) {
    return FeedbackVerdict::StaleStatus;
  }
  if (!status.healthy) {
    return FeedbackVerdict::UnhealthyStatus;
  }

  if (feedback.sequence <= goal.feedback_sequence_at_command ||
    feedback.received_at == SteadyClock::time_point{})
  {
    return FeedbackVerdict::NoNewFeedback;
  }
  if (now < feedback.received_at || now - feedback.received_at > policy.feedback_max_age) {
    return FeedbackVerdict::StaleFeedback;
  }

  if (goal.joint_names.empty() || goal.joint_names.size() != goal.final_positions.size()) {
    return FeedbackVerdict::InvalidTarget;
  }

  std::unordered_set<std::string> target_names;
  for (std::size_t i = 0; i < goal.joint_names.size(); ++i) {
    if (goal.joint_names[i].empty() ||
      !target_names.insert(goal.joint_names[i]).second ||
      !std::isfinite(goal.final_positions[i]))
    {
      return FeedbackVerdict::InvalidTarget;
    }
  }

  // Dimensions first: the arrays must be self-consistent before any element is
  // inspected, otherwise an index computed from one array could address another.
  if (feedback.joint_names.empty() ||
    feedback.positions.size() != feedback.joint_names.size())
  {
    return FeedbackVerdict::InvalidFeedbackDimensions;
  }
  if (policy.require_velocity) {
    if (feedback.velocities.size() != feedback.joint_names.size()) {
      return FeedbackVerdict::InvalidFeedbackDimensions;
    }
  } else if (!feedback.velocities.empty() &&
    feedback.velocities.size() != feedback.joint_names.size())
  {
    return FeedbackVerdict::InvalidFeedbackDimensions;
  }
  const bool use_velocity = policy.require_velocity || !feedback.velocities.empty();

  std::unordered_map<std::string, std::size_t> by_name;
  by_name.reserve(feedback.joint_names.size());

  for (std::size_t i = 0; i < feedback.joint_names.size(); ++i) {
    if (feedback.joint_names[i].empty()) {
      return FeedbackVerdict::EmptyJointName;
    }
    if (!by_name.emplace(feedback.joint_names[i], i).second) {
      return FeedbackVerdict::DuplicateJointName;
    }
    if (!std::isfinite(feedback.positions[i])) {
      return FeedbackVerdict::NonFinitePosition;
    }
    if (use_velocity && !std::isfinite(feedback.velocities[i])) {
      return FeedbackVerdict::NonFiniteVelocity;
    }
  }

  for (std::size_t i = 0; i < goal.joint_names.size(); ++i) {
    const auto found = by_name.find(goal.joint_names[i]);
    if (found == by_name.end()) {
      return FeedbackVerdict::MissingJoint;
    }

    const std::size_t index = found->second;
    if (std::abs(feedback.positions[index] - goal.final_positions[i]) >
      policy.position_tolerance)
    {
      return FeedbackVerdict::PositionOutsideTolerance;
    }
    if (use_velocity && std::abs(feedback.velocities[index]) > policy.velocity_tolerance) {
      return FeedbackVerdict::VelocityOutsideTolerance;
    }
  }

  return FeedbackVerdict::Reached;
}

}  // namespace duco_reliability

#endif  // DUCO_ROS_DRIVER_REAL_FEEDBACK_GATE_HPP
