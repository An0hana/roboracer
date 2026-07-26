// Copyright 2026 RoboRacer Team

#ifndef RACE_MANAGER__RACE_STATE_MACHINE_HPP_
#define RACE_MANAGER__RACE_STATE_MACHINE_HPP_

#include <cstdint>
#include <optional>
#include <string>

namespace race_manager
{

enum class BehaviorState : std::uint8_t
{
  INIT = 0,
  READY = 1,
  GLOBAL_TRACK = 2,
  TRAILING = 3,
  OVERTAKE = 4,
  FAULT = 5,
  STOP = 6,
  RETURN = 7
};

enum class PreferredSide : std::int8_t
{
  RIGHT = -1,
  NONE = 0,
  LEFT = 1
};

struct StateMachineConfig
{
  double follow_distance{4.0};
  double opponent_corridor_half_width{0.80};
  double pass_margin{0.30};
  double overtake_lateral_threshold{0.08};
  double return_lateral_threshold{0.05};
  double transition_confirmation{0.20};
  double minimum_state_duration{0.40};
  double recovery_confirmation{0.50};
  double opponent_lost_timeout{0.50};

  double cruise_speed_scale{1.0};
  double trailing_speed_scale{0.60};
  double overtake_speed_scale{1.0};
  double return_speed_scale{0.80};
};

struct StateObservation
{
  double time{0.0};
  bool inputs_ready{false};
  bool trajectory_valid{false};
  bool opponent_detected{false};
  double opponent_longitudinal{0.0};
  double opponent_lateral{0.0};
  // Signed lateral change of the selected trajectory from its first point.
  double selected_lateral{0.0};
};

struct StateCommand
{
  BehaviorState state{BehaviorState::INIT};
  PreferredSide preferred_side{PreferredSide::NONE};
  double speed_scale{0.0};
  bool use_local_trajectory{false};
  bool fallback_ftg{false};
  std::string reason{"waiting_for_inputs"};
};

class RaceStateMachine
{
public:
  explicit RaceStateMachine(StateMachineConfig config);

  [[nodiscard]] StateCommand update(const StateObservation & observation);
  [[nodiscard]] BehaviorState state() const noexcept;

private:
  [[nodiscard]] bool opponentAhead(const StateObservation & observation) const;
  [[nodiscard]] bool transitionConfirmed(
    BehaviorState target, const std::string & reason, double now,
    double confirmation);
  void transitionTo(
    BehaviorState target, PreferredSide side, const std::string & reason,
    double now);
  void clearPendingTransition();
  [[nodiscard]] StateCommand command() const;

  StateMachineConfig config_;
  BehaviorState state_{BehaviorState::INIT};
  PreferredSide preferred_side_{PreferredSide::NONE};
  std::string reason_{"waiting_for_inputs"};
  double state_enter_time_{0.0};
  double last_time_{0.0};
  double last_opponent_seen_time_{0.0};
  bool initialized_{false};
  std::optional<BehaviorState> pending_state_;
  std::string pending_reason_;
  double pending_since_{0.0};
};

}  // namespace race_manager

#endif  // RACE_MANAGER__RACE_STATE_MACHINE_HPP_
