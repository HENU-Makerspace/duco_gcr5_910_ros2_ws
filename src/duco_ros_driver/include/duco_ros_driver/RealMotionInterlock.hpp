// RealMotionInterlock.hpp
//
// ROS-independent model of the *real robot motion channel* state.
//
// The action goal and the physical motion are two different things.  A goal can
// terminate (succeed, fail, be canceled or time out) while the arm is still
// moving or while nothing can prove that it stopped.  This class keeps that
// knowledge independently of the action slot, so "the software task ended" is
// never mistaken for "the arm is safe to overwrite with the next trajectory".
//
// States
//   Allowed               no real motion outstanding, a new command may be sent
//   Executing             a real trajectory was published and its goal is active
//   AwaitingConfirmation  the goal ended but the real motion was never confirmed
//   Uncertain             the real motion state could not be established
//   Forbidden             the robot reported a fault, or a previous run left an
//                         unconfirmed motion behind
//
// The last three are *sticky*: they survive the action goal, they are never
// cleared by a timer, by a late callback, by a fresh simulation success or by a
// single healthy feedback message, and they survive a process restart through
// the marker file.  Only an explicit operator recovery clears them.
//
// There is no vendor interface that confirms a stop or a connection, so a
// recovery clears the interlock as an *operator attestation*; it is not a
// vendor-confirmed safe stop.

#ifndef DUCO_ROS_DRIVER_REAL_MOTION_INTERLOCK_HPP
#define DUCO_ROS_DRIVER_REAL_MOTION_INTERLOCK_HPP

#include <cstdint>
#include <string>

namespace duco_reliability
{

enum class RealMotionState {
  Allowed,
  Executing,
  AwaitingConfirmation,
  Uncertain,
  Forbidden
};

inline const char * realMotionStateName(RealMotionState state) noexcept
{
  switch (state) {
    case RealMotionState::Allowed: return "allowed";
    case RealMotionState::Executing: return "executing";
    case RealMotionState::AwaitingConfirmation: return "awaiting_confirmation";
    case RealMotionState::Uncertain: return "uncertain";
    case RealMotionState::Forbidden: return "forbidden";
  }
  return "unknown";
}

/**
 * \brief Severity order used to keep the most serious reason.
 *
 * A less severe transition never downgrades a more severe state, so a late
 * callback cannot talk the interlock out of a fault.
 */
inline int realMotionSeverity(RealMotionState state) noexcept
{
  switch (state) {
    case RealMotionState::Allowed: return 0;
    case RealMotionState::Executing: return 1;
    case RealMotionState::AwaitingConfirmation: return 2;
    case RealMotionState::Uncertain: return 3;
    case RealMotionState::Forbidden: return 4;
  }
  return 0;
}

class RealMotionInterlock
{
public:
  RealMotionState state() const noexcept {return state_;}
  const std::string & reason() const noexcept {return reason_;}
  const std::string & goalId() const noexcept {return goal_id_;}
  std::uint64_t generation() const noexcept {return generation_;}

  /// True while a new real trajectory must be refused.
  bool blocksRealMotion() const noexcept
  {
    return state_ != RealMotionState::Allowed && state_ != RealMotionState::Executing;
  }

  /// True when the state outlives the action goal and needs an operator.
  bool latched() const noexcept
  {
    return state_ == RealMotionState::AwaitingConfirmation ||
           state_ == RealMotionState::Uncertain ||
           state_ == RealMotionState::Forbidden;
  }

  /**
   * \brief Record that a real trajectory was published for \p generation.
   *
   * Refused while latched: the operator must resolve the previous motion first.
   */
  bool onCommandPublished(std::uint64_t generation, const std::string & goal_id)
  {
    if (latched()) {
      return false;
    }
    state_ = RealMotionState::Executing;
    generation_ = generation;
    goal_id_ = goal_id;
    reason_.clear();
    return true;
  }

  /**
   * \brief Record a *confirmed* real completion for \p generation.
   *
   * Only the generation that is currently executing may clear the interlock,
   * and only when nothing more severe was recorded meanwhile.
   */
  bool onConfirmedCompletion(std::uint64_t generation)
  {
    if (state_ != RealMotionState::Executing || generation != generation_) {
      return false;
    }
    reset();
    return true;
  }

  /// The goal ended without confirming that the arm reached the goal / stopped.
  bool onGoalFinishedUnconfirmed(
    std::uint64_t generation, const std::string & goal_id, const std::string & reason)
  {
    return escalate(RealMotionState::AwaitingConfirmation, generation, goal_id, reason);
  }

  /// The real motion state could not be established (lost feedback, no status,
  /// rejected cancellation, unknown Gazebo outcome, ...).
  bool onUncertain(
    std::uint64_t generation, const std::string & goal_id, const std::string & reason)
  {
    return escalate(RealMotionState::Uncertain, generation, goal_id, reason);
  }

  /// The robot reported a fault (E-stop, collision, error code).
  bool onFault(const std::string & goal_id, const std::string & reason)
  {
    if (state_ == RealMotionState::Forbidden) {
      return false;
    }
    state_ = RealMotionState::Forbidden;
    goal_id_ = goal_id;
    reason_ = reason;
    return true;
  }

  /// Explicit operator recovery.  Returns true when something was cleared.
  bool onOperatorRecovery()
  {
    if (state_ == RealMotionState::Allowed) {
      return false;
    }
    reset();
    return true;
  }

  void reset()
  {
    state_ = RealMotionState::Allowed;
    generation_ = 0;
    goal_id_.clear();
    reason_.clear();
  }

private:
  bool escalate(
    RealMotionState target, std::uint64_t generation, const std::string & goal_id,
    const std::string & reason)
  {
    if (realMotionSeverity(target) <= realMotionSeverity(state_)) {
      // Already at least this serious: keep the first (more severe) explanation
      // and never downgrade.
      return false;
    }
    state_ = target;
    generation_ = generation;
    goal_id_ = goal_id;
    reason_ = reason;
    return true;
  }

  RealMotionState state_{RealMotionState::Allowed};
  std::uint64_t generation_{0};
  std::string goal_id_;
  std::string reason_;
};

}  // namespace duco_reliability

#endif  // DUCO_ROS_DRIVER_REAL_MOTION_INTERLOCK_HPP
