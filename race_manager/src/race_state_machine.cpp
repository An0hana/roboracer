// Copyright 2026 RoboRacer Team

#include "race_manager/race_state_machine.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace race_manager
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
    !finite(config.opponent_corridor_half_width) ||
    config.opponent_corridor_half_width <= 0.0 ||
    !finite(config.pass_margin) || config.pass_margin < 0.0 ||
    !finite(config.overtake_lateral_threshold) ||
    config.overtake_lateral_threshold <= 0.0 ||
    !finite(config.return_lateral_threshold) ||
    config.return_lateral_threshold < 0.0 ||
    config.return_lateral_threshold >= config.overtake_lateral_threshold ||
    !finite(config.transition_confirmation) ||
    config.transition_confirmation < 0.0 ||
    !finite(config.minimum_state_duration) ||
    config.minimum_state_duration < 0.0 ||
    !finite(config.recovery_confirmation) ||
    config.recovery_confirmation < 0.0 ||
    !finite(config.opponent_lost_timeout) ||
    config.opponent_lost_timeout < 0.0 ||
    !finite(config.cruise_speed_scale) ||
    config.cruise_speed_scale <= 0.0 ||
    config.cruise_speed_scale > 1.0 ||
    !finite(config.trailing_speed_scale) ||
    config.trailing_speed_scale <= 0.0 ||
    config.trailing_speed_scale > 1.0 ||
    !finite(config.overtake_speed_scale) ||
    config.overtake_speed_scale <= 0.0 ||
    config.overtake_speed_scale > 1.0 ||
    !finite(config.return_speed_scale) ||
    config.return_speed_scale <= 0.0 ||
    config.return_speed_scale > 1.0)
  {
    throw std::invalid_argument("invalid race state machine configuration");
  }
}

}  // namespace

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
    !finite(observation.selected_lateral))
  {
    throw std::invalid_argument("state observation must be finite");
  }
  if (initialized_ && observation.time < last_time_) {
    throw std::invalid_argument("state observation time must be monotonic");
  }
  if (!initialized_) {
    initialized_ = true;
    state_enter_time_ = observation.time;
    last_opponent_seen_time_ = observation.time;
  }
  last_time_ = observation.time;
  if (observation.opponent_detected) {
    last_opponent_seen_time_ = observation.time;
  }

  const bool state_can_change =
    observation.time - state_enter_time_ >= config_.minimum_state_duration;
  const auto enter_fault = [this, &observation](const std::string & reason) {
      transitionTo(
        BehaviorState::FAULT, PreferredSide::NONE, reason, observation.time);
    };

  if (state_ != BehaviorState::INIT &&
    state_ != BehaviorState::FAULT &&
    (!observation.inputs_ready || !observation.trajectory_valid))
  {
    enter_fault(
      observation.inputs_ready ? "invalid_trajectory" : "stale_or_missing_input");
    return command();
  }

  switch (state_) {
    case BehaviorState::INIT:
      if (!observation.inputs_ready) {
        reason_ = "waiting_for_inputs";
        clearPendingTransition();
      } else {
        if (!observation.trajectory_valid) {
          reason_ = "waiting_for_valid_trajectory";
          if (transitionConfirmed(
              BehaviorState::FAULT, "initial_trajectory_invalid",
              observation.time, config_.recovery_confirmation))
          {
            enter_fault("initial_trajectory_invalid");
          }
        } else {
          if (transitionConfirmed(
              BehaviorState::GLOBAL_TRACK, "inputs_ready", observation.time,
              config_.recovery_confirmation))
          {
            transitionTo(
              BehaviorState::GLOBAL_TRACK, PreferredSide::NONE,
              "inputs_ready", observation.time);
          }
        }
      }
      break;

    case BehaviorState::GLOBAL_TRACK:
      if (opponentAhead(observation) && state_can_change) {
        if (transitionConfirmed(
            BehaviorState::TRAILING, "opponent_ahead", observation.time,
            config_.transition_confirmation))
        {
          transitionTo(
            BehaviorState::TRAILING, PreferredSide::NONE,
            "opponent_ahead", observation.time);
        }
      } else {
        clearPendingTransition();
      }
      break;

    case BehaviorState::TRAILING:
      if (!opponentAhead(observation) && state_can_change) {
        if (transitionConfirmed(
            BehaviorState::GLOBAL_TRACK, "path_clear", observation.time,
            config_.transition_confirmation))
        {
          transitionTo(
            BehaviorState::GLOBAL_TRACK, PreferredSide::NONE,
            "path_clear", observation.time);
        }
      } else {
        if (
          state_can_change &&
          std::abs(observation.selected_lateral) >=
          config_.overtake_lateral_threshold)
        {
          const PreferredSide side = observation.selected_lateral > 0.0 ?
            PreferredSide::LEFT : PreferredSide::RIGHT;
          const std::string reason = side == PreferredSide::LEFT ?
            "safe_left_trajectory" : "safe_right_trajectory";
          if (transitionConfirmed(
              BehaviorState::OVERTAKE, reason, observation.time,
              config_.transition_confirmation))
          {
            transitionTo(
              BehaviorState::OVERTAKE, side, reason, observation.time);
          }
        } else {
          clearPendingTransition();
        }
      }
      break;

    case BehaviorState::OVERTAKE:
      if (state_can_change) {
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
              BehaviorState::RETURN, reason, observation.time,
              config_.transition_confirmation))
          {
            transitionTo(
              BehaviorState::RETURN, PreferredSide::NONE,
              reason, observation.time);
          }
        } else {
          clearPendingTransition();
        }
      }
      break;

    case BehaviorState::RETURN:
      if (state_can_change &&
        std::abs(observation.selected_lateral) <=
        config_.return_lateral_threshold &&
        !opponentAhead(observation))
      {
        if (transitionConfirmed(
            BehaviorState::GLOBAL_TRACK, "raceline_reacquired",
            observation.time, config_.transition_confirmation))
        {
          transitionTo(
            BehaviorState::GLOBAL_TRACK, PreferredSide::NONE,
            "raceline_reacquired", observation.time);
        }
      } else {
        clearPendingTransition();
      }
      break;

    case BehaviorState::FAULT:
      if (observation.inputs_ready && observation.trajectory_valid) {
        if (transitionConfirmed(
            BehaviorState::GLOBAL_TRACK, "fault_recovered", observation.time,
            config_.recovery_confirmation))
        {
          transitionTo(
            BehaviorState::GLOBAL_TRACK, PreferredSide::NONE,
            "fault_recovered", observation.time);
        }
      } else {
        clearPendingTransition();
      }
      break;

    case BehaviorState::READY:
    case BehaviorState::STOP:
      break;
  }
  return command();
}

BehaviorState RaceStateMachine::state() const noexcept
{
  return state_;
}

bool RaceStateMachine::opponentAhead(
  const StateObservation & observation) const
{
  return observation.opponent_detected &&
         observation.opponent_longitudinal > 0.0 &&
         observation.opponent_longitudinal <= config_.follow_distance &&
         std::abs(observation.opponent_lateral) <=
         config_.opponent_corridor_half_width;
}

bool RaceStateMachine::transitionConfirmed(
  BehaviorState target, const std::string & reason, double now,
  double confirmation)
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

void RaceStateMachine::transitionTo(
  BehaviorState target, PreferredSide side, const std::string & reason,
  double now)
{
  state_ = target;
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

StateCommand RaceStateMachine::command() const
{
  StateCommand output;
  output.state = state_;
  output.preferred_side = preferred_side_;
  output.reason = reason_;
  switch (state_) {
    case BehaviorState::GLOBAL_TRACK:
      output.speed_scale = config_.cruise_speed_scale;
      output.use_local_trajectory = true;
      break;
    case BehaviorState::TRAILING:
      output.speed_scale = config_.trailing_speed_scale;
      output.use_local_trajectory = true;
      break;
    case BehaviorState::OVERTAKE:
      output.speed_scale = config_.overtake_speed_scale;
      output.use_local_trajectory = true;
      break;
    case BehaviorState::RETURN:
      output.speed_scale = config_.return_speed_scale;
      output.use_local_trajectory = true;
      break;
    case BehaviorState::FAULT:
      output.fallback_ftg = true;
      break;
    case BehaviorState::INIT:
    case BehaviorState::READY:
    case BehaviorState::STOP:
      break;
  }
  return output;
}

}  // namespace race_manager
