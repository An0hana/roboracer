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
  STOP = 3,
  RECOVERY = 4
};

enum class BehaviorState : std::uint8_t
{
  RACING = 0,
  TRAILING = 1,
  OVERTAKE = 2
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

  // Reverse-recovery gate. Failure evidence is held briefly across AEB
  // debounce/diagnostic transitions, then combined with low progress and
  // either a genuinely tight turn or a saturated planner steering command.
  double stuck_speed_threshold{0.08};
  double recovery_entry_time{0.75};
  double failure_evidence_hold_time{1.0};
  double tight_curve_steering_threshold{0.18};
  double tight_curve_exit_threshold{0.14};
  double steering_saturation_fraction{0.85};
  double min_steering{-0.404};
  double max_steering{0.381};
  double wheelbase{0.324};

  // Recovery is phased: settle the steering at zero speed, reverse, then
  // brake to a confirmed stop before handing control back to MPPI.
  double recovery_settle_time{0.30};
  double reverse_speed{0.30};
  double reverse_steer_sign{-1.0};
  double reverse_steer_fraction{0.90};

  // Reverse until the turn has eased or the target distance is reached.
  // Maximum limits are backstops; all exits still pass through BRAKING.
  double recovery_min_reverse_distance{0.3};
  double recovery_target_reverse_distance{0.6};
  double reverse_max_duration{6.0};
  double reverse_max_distance{1.8};
  double recovery_stop_speed_threshold{0.05};
  double recovery_stop_hold_time{0.30};
  double reentry_debounce{4.0};
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

  // Planner / safety failure evidence consumed by the recovery gate.
  bool mppi_solver_failed{false};
  bool aeb_emergency{false};
  bool aeb_latched{false};
  double mppi_steering_command{0.0};
  // Steering angle (rad) the raceline needs at the current pose, computed by
  // the node as atan(nearest_curvature * wheelbase).
  double required_steering{0.0};
};

struct StateCommand
{
  SafetyState safety_state{SafetyState::INIT};
  BehaviorState behavior_state{BehaviorState::RACING};
  PreferredSide preferred_side{PreferredSide::NONE};
  double track_confidence{1.0};
  double speed_scale{0.0};
  double raceline_weight_scale{1.0};
  double safety_weight_scale{1.0};
  double lateral_reference_offset{0.0};
  bool stop_requested{true};
  bool recovery_active{false};
  double recovery_speed{0.0};
  double recovery_steering{0.0};
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
  enum class RecoveryPhase : std::uint8_t
  {
    IDLE = 0,
    SETTLING = 1,
    REVERSING = 2,
    BRAKING = 3
  };

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
  [[nodiscard]] StateCommand command(double now) const;
  [[nodiscard]] bool stuckAtTightTurn(
    const StateObservation & observation) const;
  [[nodiscard]] bool tightTurnEvidence(
    const StateObservation & observation) const;
  [[nodiscard]] double steeringLimitForSign(double steering) const;

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
  bool initialized_{false};
  std::optional<int> pending_state_;
  std::string pending_reason_;
  double pending_since_{0.0};

  // Reverse-recovery bookkeeping.
  double required_steering_{0.0};
  std::optional<double> last_failure_evidence_time_;
  std::optional<double> stuck_since_;
  double recovery_enter_time_{0.0};
  double recovery_phase_enter_time_{0.0};
  double reverse_distance_{0.0};
  std::optional<double> recovery_stopped_since_;
  double reentry_available_until_{0.0};
  double recovery_steering_{0.0};
  double recovery_command_speed_{0.0};
  RecoveryPhase recovery_phase_{RecoveryPhase::IDLE};
};

}  // namespace state_machine

#endif  // STATE_MACHINE__RACE_STATE_MACHINE_HPP_
