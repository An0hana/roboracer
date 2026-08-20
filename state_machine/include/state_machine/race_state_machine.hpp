// Copyright 2026 RoboRacer Team

#ifndef STATE_MACHINE__RACE_STATE_MACHINE_HPP_
#define STATE_MACHINE__RACE_STATE_MACHINE_HPP_

#include <cstdint>
#include <optional>
#include <string>

namespace state_machine
{

enum class SafetyState : std::uint8_t
{
  INIT = 0,
  READY = 1,
  FAULT = 2,
  STOP = 3
};

enum class BehaviorState : std::uint8_t
{
  RACING = 0,
  TRAILING = 1,
  OVERTAKE = 2,
  RECOVERY = 3
};

enum class RecoveryPhase : std::uint8_t
{
  NONE = 0,
  REVERSE = 1,
  SETTLE = 2
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
  double follow_time_headway{1.50};
  double maximum_follow_distance{12.0};
  double opponent_corridor_half_width{0.80};
  double pass_margin{0.30};
  double transition_confirmation{0.20};
  double minimum_state_duration{0.40};
  double recovery_confirmation{0.50};
  double opponent_lost_timeout{0.50};
  double return_blend_duration{1.50};
  double overtake_lateral_offset{0.45};
  bool allow_direct_overtake{true};

  double cruise_speed_scale{1.0};
  double trailing_speed_scale{0.60};
  double overtake_speed_scale{1.0};
  double degraded_speed_scale{0.55};
  double minimum_raceline_weight_scale{0.25};
  double maximum_safety_weight_scale{2.0};

  double stuck_command_speed_threshold{0.20};
  double stuck_speed_threshold{0.05};
  double stuck_confirmation{1.50};
  double recovery_reverse_distance{0.40};
  double recovery_max_reverse_time{3.0};
  double recovery_settle_confirmation{0.25};
  double recovery_cooldown{2.0};
};

struct StateObservation
{
  double time{0.0};
  bool inputs_ready{false};
  bool emergency_stop{false};
  bool opponent_detected{false};
  double opponent_longitudinal{0.0};
  double opponent_lateral{0.0};
  double ego_speed{0.0};
  double opponent_longitudinal_speed{0.0};
  bool left_available{false};
  bool right_available{false};
  double left_clearance_score{0.0};
  double right_clearance_score{0.0};
  double track_confidence{1.0};
  bool command_available{false};
  double commanded_speed{0.0};
  bool reverse_path_clear{false};
};

struct StateCommand
{
  SafetyState safety_state{SafetyState::INIT};
  BehaviorState behavior_state{BehaviorState::RACING};
  RecoveryPhase recovery_phase{RecoveryPhase::NONE};
  PreferredSide preferred_side{PreferredSide::NONE};
  double track_confidence{1.0};
  double speed_scale{0.0};
  double raceline_weight_scale{1.0};
  double safety_weight_scale{1.0};
  double lateral_reference_offset{0.0};
  bool stop_requested{true};
  std::string reason{"waiting_for_inputs"};
};

class TrackConfidenceFilter
{
public:
  TrackConfidenceFilter(double rise_time, double fall_time, double initial = 1.0);
  [[nodiscard]] double update(double raw_confidence, double dt);
  [[nodiscard]] double value() const noexcept;

private:
  double rise_time_;
  double fall_time_;
  double value_;
};

class RaceStateMachine
{
public:
  explicit RaceStateMachine(StateMachineConfig config);

  [[nodiscard]] StateCommand update(const StateObservation & observation);
  [[nodiscard]] SafetyState safetyState() const noexcept;
  [[nodiscard]] BehaviorState behaviorState() const noexcept;

private:
  [[nodiscard]] double activeFollowDistance(
    const StateObservation & observation) const;
  [[nodiscard]] bool opponentAhead(const StateObservation & observation) const;
  [[nodiscard]] PreferredSide preferredOvertakeSide(
    const StateObservation & observation) const;
  [[nodiscard]] bool transitionConfirmed(
    int target, const std::string & reason, double now, double confirmation);
  void transitionSafety(
    SafetyState target, const std::string & reason, double now);
  void transitionBehavior(
    BehaviorState target, PreferredSide side, const std::string & reason,
    double now);
  void clearPendingTransition();
  void resetRecovery();
  [[nodiscard]] StateCommand command(double now) const;

  StateMachineConfig config_;
  SafetyState safety_state_{SafetyState::INIT};
  BehaviorState behavior_state_{BehaviorState::RACING};
  PreferredSide preferred_side_{PreferredSide::NONE};
  std::string reason_{"waiting_for_inputs"};
  double state_enter_time_{0.0};
  double last_time_{0.0};
  double last_opponent_seen_time_{0.0};
  double latest_track_confidence_{1.0};
  double return_blend_start_time_{0.0};
  double return_blend_initial_offset_{0.0};
  RecoveryPhase recovery_phase_{RecoveryPhase::NONE};
  double recovery_distance_{0.0};
  double recovery_start_time_{0.0};
  std::optional<double> recovery_settle_since_;
  std::optional<double> stuck_since_;
  std::optional<double> last_recovery_exit_time_;
  bool initialized_{false};
  std::optional<int> pending_state_;
  std::string pending_reason_;
  double pending_since_{0.0};
};

}  // namespace state_machine

#endif  // STATE_MACHINE__RACE_STATE_MACHINE_HPP_
