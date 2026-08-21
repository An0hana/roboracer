// Copyright 2026 RoboRacer Team

#include "state_machine/race_state_machine.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace state_machine
{
namespace
{

bool finite(double value)
{
  return std::isfinite(value);
}

void validateConfig(const StateMachineConfig & config)
{
  if (!finite(config.follow_distance) || config.follow_distance <= 0.0 ||
    !finite(config.follow_time_headway) || config.follow_time_headway < 0.0 ||
    !finite(config.maximum_follow_distance) ||
    config.maximum_follow_distance < config.follow_distance ||
    !finite(config.opponent_corridor_half_width) ||
    config.opponent_corridor_half_width <= 0.0 ||
    !finite(config.pass_margin) || config.pass_margin < 0.0 ||
    !finite(config.transition_confirmation) ||
    config.transition_confirmation < 0.0 ||
    !finite(config.minimum_state_duration) ||
    config.minimum_state_duration < 0.0 ||
    !finite(config.recovery_confirmation) ||
    config.recovery_confirmation < 0.0 ||
    !finite(config.opponent_lost_timeout) ||
    config.opponent_lost_timeout < 0.0 ||
    !finite(config.return_blend_duration) ||
    config.return_blend_duration <= 0.0 ||
    !finite(config.overtake_lateral_offset) ||
    config.overtake_lateral_offset <= 0.0 ||
    !finite(config.cruise_speed_scale) ||
    config.cruise_speed_scale <= 0.0 ||
    config.cruise_speed_scale > 1.0 ||
    !finite(config.trailing_speed_scale) ||
    config.trailing_speed_scale <= 0.0 ||
    config.trailing_speed_scale > 1.0 ||
    !finite(config.overtake_speed_scale) ||
    config.overtake_speed_scale <= 0.0 ||
    config.overtake_speed_scale > 1.0 ||
    !finite(config.degraded_speed_scale) ||
    config.degraded_speed_scale <= 0.0 ||
    config.degraded_speed_scale > config.cruise_speed_scale ||
    !finite(config.minimum_raceline_weight_scale) ||
    config.minimum_raceline_weight_scale <= 0.0 ||
    config.minimum_raceline_weight_scale > 1.0 ||
    !finite(config.maximum_safety_weight_scale) ||
    config.maximum_safety_weight_scale < 1.0 ||
    !finite(config.stuck_command_speed_threshold) ||
    config.stuck_command_speed_threshold <= 0.0 ||
    !finite(config.stuck_speed_threshold) || config.stuck_speed_threshold <= 0.0 ||
    !finite(config.stuck_confirmation) || config.stuck_confirmation <= 0.0 ||
    !finite(config.recovery_reverse_distance) || config.recovery_reverse_distance <= 0.0 ||
    !finite(config.recovery_minimum_success_distance) ||
    config.recovery_minimum_success_distance <= 0.0 ||
    config.recovery_minimum_success_distance > config.recovery_reverse_distance ||
    !finite(config.recovery_max_reverse_time) || config.recovery_max_reverse_time <= 0.0 ||
    !finite(config.recovery_settle_confirmation) ||
    config.recovery_settle_confirmation < 0.0 ||
    !finite(config.recovery_cooldown) || config.recovery_cooldown < 0.0)
  {
    throw std::invalid_argument("invalid race state machine configuration");
  }
}

}  // namespace

TrackConfidenceFilter::TrackConfidenceFilter(
  double rise_time, double fall_time, double initial)
: rise_time_(rise_time), fall_time_(fall_time), value_(initial)
{
  if (!finite(rise_time_) || rise_time_ <= 0.0 ||
    !finite(fall_time_) || fall_time_ <= 0.0 ||
    !finite(value_) || value_ < 0.0 || value_ > 1.0)
  {
    throw std::invalid_argument("invalid track confidence filter configuration");
  }
}

double TrackConfidenceFilter::update(double raw_confidence, double dt)
{
  if (!finite(raw_confidence) || !finite(dt) || dt < 0.0) {
    throw std::invalid_argument("invalid track confidence filter input");
  }
  raw_confidence = std::clamp(raw_confidence, 0.0, 1.0);
  const double time_constant =
    raw_confidence >= value_ ? rise_time_ : fall_time_;
  const double alpha = dt <= 0.0 ? 0.0 : 1.0 - std::exp(-dt / time_constant);
  value_ = std::clamp(value_ + alpha * (raw_confidence - value_), 0.0, 1.0);
  return value_;
}

double TrackConfidenceFilter::value() const noexcept
{
  return value_;
}

RaceStateMachine::RaceStateMachine(StateMachineConfig config)
: config_(std::move(config))
{
  validateConfig(config_);
}

StateCommand RaceStateMachine::update(const StateObservation & observation)
{
  if (!finite(observation.time) ||
    !finite(observation.opponent_longitudinal) ||
    !finite(observation.opponent_lateral) ||
    !finite(observation.ego_speed) ||
    !finite(observation.opponent_longitudinal_speed) ||
    !finite(observation.left_clearance_score) ||
    !finite(observation.right_clearance_score) ||
    !finite(observation.track_confidence) ||
    !finite(observation.commanded_speed))
  {
    throw std::invalid_argument("state observation must be finite");
  }
  if (initialized_ && observation.time < last_time_) {
    throw std::invalid_argument("state observation time must be monotonic");
  }
  const double dt = initialized_ ? observation.time - last_time_ : 0.0;
  if (!initialized_) {
    initialized_ = true;
    state_enter_time_ = observation.time;
    last_opponent_seen_time_ = observation.time;
  }
  last_time_ = observation.time;
  latest_track_confidence_ =
    std::clamp(observation.track_confidence, 0.0, 1.0);
  if (observation.opponent_detected) {
    last_opponent_seen_time_ = observation.time;
  }

  if (safety_state_ == SafetyState::READY && !observation.inputs_ready) {
    transitionSafety(SafetyState::FAULT, "stale_or_missing_input", observation.time);
    return command(observation.time);
  }
  if (safety_state_ == SafetyState::READY && observation.emergency_stop) {
    transitionSafety(SafetyState::STOP, "emergency_clearance", observation.time);
    return command(observation.time);
  }

  switch (safety_state_) {
    case SafetyState::INIT:
      if (!observation.inputs_ready) {
        reason_ = "waiting_for_inputs";
        clearPendingTransition();
      } else if (transitionConfirmed(
          100 + static_cast<int>(SafetyState::READY), "inputs_ready",
          observation.time, config_.recovery_confirmation))
      {
        transitionSafety(SafetyState::READY, "inputs_ready", observation.time);
      }
      break;
    case SafetyState::FAULT:
    case SafetyState::STOP:
      if (observation.inputs_ready && !observation.emergency_stop) {
        if (transitionConfirmed(
            100 + static_cast<int>(SafetyState::READY), "fault_recovered",
            observation.time, config_.recovery_confirmation))
        {
          transitionSafety(SafetyState::READY, "fault_recovered", observation.time);
        }
      } else {
        clearPendingTransition();
      }
      break;
    case SafetyState::READY:
      break;
  }

  if (safety_state_ != SafetyState::READY) {
    return command(observation.time);
  }

  if (behavior_state_ == BehaviorState::RECOVERY) {
    if (!observation.command_available) {
      transitionSafety(
        SafetyState::FAULT, "recovery_command_unavailable", observation.time);
      return command(observation.time);
    }

    if (recovery_phase_ == RecoveryPhase::REVERSE) {
      recovery_distance_ += std::max(0.0, -observation.ego_speed) * dt;
      const bool distance_reached =
        recovery_distance_ >= config_.recovery_reverse_distance;
      const bool time_reached =
        observation.time - recovery_start_time_ >= config_.recovery_max_reverse_time;
      if (!observation.reverse_path_clear) {
        if (recovery_distance_ < config_.recovery_minimum_success_distance) {
          transitionSafety(
            SafetyState::STOP, "recovery_reverse_blocked", observation.time);
          return command(observation.time);
        }
        recovery_phase_ = RecoveryPhase::SETTLE;
        recovery_settle_since_.reset();
        reason_ = "recovery_reverse_blocked_after_progress";
      } else if (distance_reached) {
        recovery_phase_ = RecoveryPhase::SETTLE;
        recovery_settle_since_.reset();
        reason_ = "recovery_distance_reached";
      } else if (time_reached) {
        // Never report recovery_complete after a reverse attempt that barely
        // moved. That old transition restored normal MPPI steering, then
        // re-entered recovery and centred it again, producing the observed
        // steering oscillation without escaping the wall.
        if (recovery_distance_ < config_.recovery_minimum_success_distance) {
          transitionSafety(
            SafetyState::STOP, "recovery_insufficient_progress", observation.time);
          return command(observation.time);
        }
        recovery_phase_ = RecoveryPhase::SETTLE;
        recovery_settle_since_.reset();
        reason_ = "recovery_time_limit_after_progress";
      }
    }

    if (recovery_phase_ == RecoveryPhase::SETTLE) {
      if (std::abs(observation.ego_speed) <= config_.stuck_speed_threshold) {
        if (!recovery_settle_since_.has_value()) {
          recovery_settle_since_ = observation.time;
        }
        if (observation.time - *recovery_settle_since_ >=
          config_.recovery_settle_confirmation)
        {
          transitionBehavior(
            BehaviorState::RACING, PreferredSide::NONE,
            "recovery_complete", observation.time);
        }
      } else {
        recovery_settle_since_.reset();
      }
      if (behavior_state_ == BehaviorState::RECOVERY &&
        observation.time - recovery_start_time_ >=
        config_.recovery_max_reverse_time + 1.0)
      {
        transitionSafety(SafetyState::STOP, "recovery_settle_timeout", observation.time);
      }
    }
    return command(observation.time);
  }

  const bool cooldown_elapsed =
    !last_recovery_exit_time_.has_value() ||
    observation.time - *last_recovery_exit_time_ >= config_.recovery_cooldown;
  // A solver that cannot find a safe forward rollout deliberately publishes a
  // zero-speed braking command.  Requiring a positive output command here
  // creates a deadlock: the condition that should request recovery can never
  // become true.  RACING and OVERTAKE both imply forward-motion intent, while
  // TRAILING may legitimately hold position behind a stopped opponent and
  // therefore still requires an explicit positive command.
  const bool forward_motion_expected =
    behavior_state_ == BehaviorState::RACING ||
    behavior_state_ == BehaviorState::OVERTAKE ||
    (behavior_state_ == BehaviorState::TRAILING &&
    observation.commanded_speed >= config_.stuck_command_speed_threshold);
  const bool stuck_candidate =
    observation.command_available && cooldown_elapsed &&
    forward_motion_expected &&
    std::abs(observation.ego_speed) <= config_.stuck_speed_threshold;
  if (stuck_candidate) {
    if (!stuck_since_.has_value()) {
      stuck_since_ = observation.time;
    }
    if (observation.time - *stuck_since_ >= config_.stuck_confirmation &&
      observation.reverse_path_clear)
    {
      transitionBehavior(
        BehaviorState::RECOVERY, PreferredSide::NONE,
        "vehicle_stuck", observation.time);
      return command(observation.time);
    }
  } else {
    stuck_since_.reset();
  }

  const bool behavior_can_change =
    observation.time - state_enter_time_ >= config_.minimum_state_duration;
  switch (behavior_state_) {
    case BehaviorState::RACING:
      if (opponentAhead(observation) && behavior_can_change) {
        const PreferredSide side = config_.allow_direct_overtake ?
          preferredOvertakeSide(observation) : PreferredSide::NONE;
        const BehaviorState target =
          side == PreferredSide::NONE ?
          BehaviorState::TRAILING : BehaviorState::OVERTAKE;
        const std::string reason =
          side == PreferredSide::NONE ?
          "opponent_ahead" : "clear_corridor_ahead";
        if (transitionConfirmed(
            static_cast<int>(target), reason,
            observation.time, config_.transition_confirmation))
        {
          transitionBehavior(target, side, reason, observation.time);
        }
      } else {
        clearPendingTransition();
      }
      break;
    case BehaviorState::TRAILING:
      if (!opponentAhead(observation) && behavior_can_change) {
        if (transitionConfirmed(
            static_cast<int>(BehaviorState::RACING), "path_clear",
            observation.time, config_.transition_confirmation))
        {
          transitionBehavior(
            BehaviorState::RACING, PreferredSide::NONE,
            "path_clear", observation.time);
        }
      } else if (behavior_can_change &&
        (observation.left_available || observation.right_available))
      {
        const PreferredSide side = preferredOvertakeSide(observation);
        const std::string reason =
          side == PreferredSide::LEFT ? "left_corridor_clear" : "right_corridor_clear";
        if (transitionConfirmed(
            static_cast<int>(BehaviorState::OVERTAKE), reason,
            observation.time, config_.transition_confirmation))
        {
          transitionBehavior(BehaviorState::OVERTAKE, side, reason, observation.time);
        }
      } else {
        clearPendingTransition();
      }
      break;
    case BehaviorState::OVERTAKE:
      if (behavior_can_change) {
        const bool passed =
          observation.opponent_detected &&
          observation.opponent_longitudinal < -config_.pass_margin;
        const bool opponent_lost =
          !observation.opponent_detected &&
          observation.time - last_opponent_seen_time_ >=
          config_.opponent_lost_timeout;
        if (passed || opponent_lost) {
          const std::string reason =
            passed ? "opponent_behind" : "opponent_lost_after_overtake";
          if (transitionConfirmed(
              static_cast<int>(BehaviorState::RACING), reason,
              observation.time, config_.transition_confirmation))
          {
            transitionBehavior(
              BehaviorState::RACING, PreferredSide::NONE, reason, observation.time);
          }
        } else {
          clearPendingTransition();
        }
      }
      break;
    case BehaviorState::RECOVERY:
      break;
  }
  return command(observation.time);
}

SafetyState RaceStateMachine::safetyState() const noexcept
{
  return safety_state_;
}

BehaviorState RaceStateMachine::behaviorState() const noexcept
{
  return behavior_state_;
}

bool RaceStateMachine::opponentAhead(const StateObservation & observation) const
{
  return observation.opponent_detected &&
         observation.opponent_longitudinal > 0.0 &&
         observation.opponent_longitudinal <= activeFollowDistance(observation) &&
         std::abs(observation.opponent_lateral) <=
         config_.opponent_corridor_half_width;
}

double RaceStateMachine::activeFollowDistance(
  const StateObservation & observation) const
{
  const double closing_speed = std::max(
    0.0, observation.ego_speed - observation.opponent_longitudinal_speed);
  return std::clamp(
    config_.follow_distance + config_.follow_time_headway * closing_speed,
    config_.follow_distance, config_.maximum_follow_distance);
}

PreferredSide RaceStateMachine::preferredOvertakeSide(
  const StateObservation & observation) const
{
  if (observation.left_available && observation.right_available) {
    return observation.left_clearance_score >= observation.right_clearance_score ?
           PreferredSide::LEFT : PreferredSide::RIGHT;
  }
  if (observation.left_available) {
    return PreferredSide::LEFT;
  }
  if (observation.right_available) {
    return PreferredSide::RIGHT;
  }
  return PreferredSide::NONE;
}

bool RaceStateMachine::transitionConfirmed(
  int target, const std::string & reason, double now, double confirmation)
{
  if (!pending_state_.has_value() || *pending_state_ != target ||
    pending_reason_ != reason)
  {
    pending_state_ = target;
    pending_reason_ = reason;
    pending_since_ = now;
  }
  return now - pending_since_ >= confirmation;
}

void RaceStateMachine::transitionSafety(
  SafetyState target, const std::string & reason, double now)
{
  if (behavior_state_ == BehaviorState::RECOVERY) {
    last_recovery_exit_time_ = now;
  }
  safety_state_ = target;
  behavior_state_ = BehaviorState::RACING;
  preferred_side_ = PreferredSide::NONE;
  reason_ = reason;
  state_enter_time_ = now;
  return_blend_initial_offset_ = 0.0;
  resetRecovery();
  clearPendingTransition();
}

void RaceStateMachine::transitionBehavior(
  BehaviorState target, PreferredSide side, const std::string & reason,
  double now)
{
  if (behavior_state_ == BehaviorState::OVERTAKE &&
    target == BehaviorState::RACING)
  {
    return_blend_initial_offset_ =
      preferred_side_ == PreferredSide::LEFT ?
      config_.overtake_lateral_offset : -config_.overtake_lateral_offset;
    return_blend_start_time_ = now;
  }
  if (target == BehaviorState::RECOVERY) {
    recovery_phase_ = RecoveryPhase::REVERSE;
    recovery_distance_ = 0.0;
    recovery_start_time_ = now;
    recovery_settle_since_.reset();
    stuck_since_.reset();
    return_blend_initial_offset_ = 0.0;
  } else if (behavior_state_ == BehaviorState::RECOVERY) {
    last_recovery_exit_time_ = now;
    resetRecovery();
  }
  behavior_state_ = target;
  preferred_side_ = side;
  reason_ = reason;
  state_enter_time_ = now;
  clearPendingTransition();
}

void RaceStateMachine::clearPendingTransition()
{
  pending_state_.reset();
  pending_reason_.clear();
}

void RaceStateMachine::resetRecovery()
{
  recovery_phase_ = RecoveryPhase::NONE;
  recovery_distance_ = 0.0;
  recovery_start_time_ = 0.0;
  recovery_settle_since_.reset();
  stuck_since_.reset();
}

StateCommand RaceStateMachine::command(double now) const
{
  StateCommand output;
  output.safety_state = safety_state_;
  output.behavior_state = behavior_state_;
  output.recovery_phase = recovery_phase_;
  output.preferred_side = preferred_side_;
  output.reason = reason_;
  output.track_confidence = latest_track_confidence_;
  output.raceline_weight_scale =
    config_.minimum_raceline_weight_scale +
    (1.0 - config_.minimum_raceline_weight_scale) * latest_track_confidence_;
  output.safety_weight_scale =
    config_.maximum_safety_weight_scale -
    (config_.maximum_safety_weight_scale - 1.0) * latest_track_confidence_;
  const double environment_speed =
    config_.degraded_speed_scale +
    (config_.cruise_speed_scale - config_.degraded_speed_scale) *
    latest_track_confidence_;
  output.stop_requested = safety_state_ != SafetyState::READY;
  if (output.stop_requested) {
    return output;
  }

  switch (behavior_state_) {
    case BehaviorState::RACING:
      output.speed_scale = environment_speed;
      if (return_blend_initial_offset_ != 0.0) {
        const double fraction = std::clamp(
          (now - return_blend_start_time_) / config_.return_blend_duration,
          0.0, 1.0);
        output.lateral_reference_offset =
          return_blend_initial_offset_ * (1.0 - fraction);
      }
      break;
    case BehaviorState::TRAILING:
      output.speed_scale = std::min(environment_speed, config_.trailing_speed_scale);
      break;
    case BehaviorState::OVERTAKE:
      output.speed_scale = std::min(environment_speed, config_.overtake_speed_scale);
      output.lateral_reference_offset =
        preferred_side_ == PreferredSide::LEFT ?
        config_.overtake_lateral_offset : -config_.overtake_lateral_offset;
      break;
    case BehaviorState::RECOVERY:
      // The MPPI node interprets recovery_phase and bypasses normal sampling.
      // Keeping the scale at zero prevents a stale consumer from driving forward.
      output.speed_scale = 0.0;
      break;
  }
  return output;
}

}  // namespace state_machine
