// Copyright 2026 RoboRacer Team

#include <cmath>
#include <stdexcept>

#include "gtest/gtest.h"
#include "state_machine/race_state_machine.hpp"

namespace state_machine
{
namespace
{

StateMachineConfig config()
{
  StateMachineConfig value;
  value.transition_confirmation = 0.20;
  value.minimum_state_duration = 0.40;
  value.recovery_confirmation = 0.50;
  value.opponent_lost_timeout = 0.50;
  value.return_blend_duration = 1.0;
  return value;
}

StateCommand runFor(
  RaceStateMachine & machine, StateObservation & observation,
  double duration, double step = 0.05)
{
  StateCommand result;
  const int count = static_cast<int>(std::ceil(duration / step));
  for (int index = 0; index < count; ++index) {
    observation.time += step;
    result = machine.update(observation);
  }
  return result;
}

void enterReady(RaceStateMachine & machine, StateObservation & observation)
{
  observation.inputs_ready = true;
  static_cast<void>(machine.update(observation));
  const auto result = runFor(machine, observation, 0.60);
  ASSERT_EQ(result.safety_state, SafetyState::READY);
  ASSERT_EQ(result.behavior_state, BehaviorState::RACING);
}

void enterTrailing(RaceStateMachine & machine, StateObservation & observation)
{
  observation.opponent_detected = true;
  observation.opponent_longitudinal = 2.0;
  observation.opponent_lateral = 0.10;
  const auto result = runFor(machine, observation, 0.70);
  ASSERT_EQ(result.behavior_state, BehaviorState::TRAILING);
}

TEST(RaceStateMachine, RequiresFreshInputsBeforeReady)
{
  RaceStateMachine machine(config());
  StateObservation observation;
  auto result = machine.update(observation);
  EXPECT_EQ(result.safety_state, SafetyState::INIT);
  EXPECT_TRUE(result.stop_requested);

  enterReady(machine, observation);
  result = machine.update(observation);
  EXPECT_FALSE(result.stop_requested);
  EXPECT_DOUBLE_EQ(result.speed_scale, 1.0);
}

TEST(RaceStateMachine, FaultsImmediatelyAndRecoversWithDebounce)
{
  RaceStateMachine machine(config());
  StateObservation observation;
  enterReady(machine, observation);

  observation.inputs_ready = false;
  auto result = machine.update(observation);
  EXPECT_EQ(result.safety_state, SafetyState::FAULT);
  EXPECT_TRUE(result.stop_requested);

  observation.inputs_ready = true;
  result = runFor(machine, observation, 0.30);
  EXPECT_EQ(result.safety_state, SafetyState::FAULT);
  result = runFor(machine, observation, 0.30);
  EXPECT_EQ(result.safety_state, SafetyState::READY);
}

TEST(RaceStateMachine, EmergencyStopRequiresConfirmedRecovery)
{
  RaceStateMachine machine(config());
  StateObservation observation;
  enterReady(machine, observation);
  observation.emergency_stop = true;
  auto result = machine.update(observation);
  EXPECT_EQ(result.safety_state, SafetyState::STOP);
  EXPECT_TRUE(result.stop_requested);
  observation.emergency_stop = false;
  result = runFor(machine, observation, 0.60);
  EXPECT_EQ(result.safety_state, SafetyState::READY);
}

TEST(RaceStateMachine, OvertakesAndBlendsDirectlyBackToRacing)
{
  RaceStateMachine machine(config());
  StateObservation observation;
  enterReady(machine, observation);
  enterTrailing(machine, observation);

  observation.left_available = true;
  observation.left_clearance_score = 1.0;
  auto result = runFor(machine, observation, 0.70);
  ASSERT_EQ(result.behavior_state, BehaviorState::OVERTAKE);
  EXPECT_EQ(result.preferred_side, PreferredSide::LEFT);
  EXPECT_DOUBLE_EQ(result.lateral_reference_offset, 0.45);

  observation.opponent_longitudinal = -0.5;
  result = runFor(machine, observation, 0.70);
  ASSERT_EQ(result.behavior_state, BehaviorState::RACING);
  EXPECT_GT(result.lateral_reference_offset, 0.0);
  result = runFor(machine, observation, 1.1);
  EXPECT_NEAR(result.lateral_reference_offset, 0.0, 1.0e-9);
}

TEST(RaceStateMachine, EntersOvertakeDirectlyWhenAheadCorridorIsClear)
{
  RaceStateMachine machine(config());
  StateObservation observation;
  enterReady(machine, observation);

  observation.opponent_detected = true;
  observation.opponent_longitudinal = 5.0;
  observation.opponent_lateral = 0.10;
  observation.ego_speed = 2.0;
  observation.opponent_longitudinal_speed = 1.0;
  observation.left_available = true;
  observation.left_clearance_score = 1.0;

  const auto result = runFor(machine, observation, 0.70);
  EXPECT_EQ(result.behavior_state, BehaviorState::OVERTAKE);
  EXPECT_EQ(result.preferred_side, PreferredSide::LEFT);
  EXPECT_EQ(result.reason, "clear_corridor_ahead");
}

TEST(RaceStateMachine, UsesClosingSpeedToTriggerBehaviorEarlier)
{
  RaceStateMachine machine(config());
  StateObservation observation;
  enterReady(machine, observation);

  observation.opponent_detected = true;
  observation.opponent_longitudinal = 5.0;
  observation.opponent_lateral = 0.10;
  observation.ego_speed = 2.0;
  observation.opponent_longitudinal_speed = 1.0;

  auto result = runFor(machine, observation, 0.70);
  EXPECT_EQ(result.behavior_state, BehaviorState::TRAILING);

  RaceStateMachine receding_machine(config());
  StateObservation receding_observation;
  enterReady(receding_machine, receding_observation);
  receding_observation.opponent_detected = true;
  receding_observation.opponent_longitudinal = 5.0;
  receding_observation.opponent_lateral = 0.10;
  receding_observation.ego_speed = 1.0;
  receding_observation.opponent_longitudinal_speed = 2.0;

  result = runFor(receding_machine, receding_observation, 0.70);
  EXPECT_EQ(result.behavior_state, BehaviorState::RACING);
}

TEST(RaceStateMachine, ChoosesCorridorWithHigherClearance)
{
  RaceStateMachine machine(config());
  StateObservation observation;
  enterReady(machine, observation);
  enterTrailing(machine, observation);
  observation.left_available = true;
  observation.right_available = true;
  observation.left_clearance_score = 0.96;
  observation.right_clearance_score = 1.0;
  const auto result = runFor(machine, observation, 0.70);
  EXPECT_EQ(result.behavior_state, BehaviorState::OVERTAKE);
  EXPECT_EQ(result.preferred_side, PreferredSide::RIGHT);
}

TEST(RaceStateMachine, DeformationScalesOneBaseParameterSetContinuously)
{
  RaceStateMachine machine(config());
  StateObservation observation;
  observation.track_confidence = 0.4;
  enterReady(machine, observation);
  const auto result = machine.update(observation);
  EXPECT_NEAR(result.speed_scale, 0.73, 1.0e-9);
  EXPECT_NEAR(result.raceline_weight_scale, 0.55, 1.0e-9);
  EXPECT_NEAR(result.safety_weight_scale, 1.6, 1.0e-9);
}

TEST(TrackConfidenceFilter, FallsFastAndRecoversSlowly)
{
  TrackConfidenceFilter filter(1.5, 0.35);
  const double fallen = filter.update(0.0, 0.35);
  EXPECT_LT(fallen, 0.40);
  const double recovered = filter.update(1.0, 0.35);
  EXPECT_LT(recovered, 0.60);
  EXPECT_GT(recovered, fallen);
}

StateMachineConfig recoveryConfig()
{
  StateMachineConfig value = config();
  value.recovery_entry_time = 1.5;
  value.exit_clear_time = 1.0;
  value.recovery_min_reverse_distance = 0.3;
  value.reverse_max_duration = 2.0;
  value.reverse_max_distance = 1.8;
  value.reentry_debounce = 1.0;
  return value;
}

void enterRecovery(RaceStateMachine & machine, StateObservation & observation)
{
  observation.ego_speed = 0.0;
  observation.mppi_solver_failed = true;
  observation.aeb_latched = false;
  observation.required_steering = 0.45;
  const StateCommand result = runFor(machine, observation, 1.6);
  ASSERT_EQ(result.safety_state, SafetyState::RECOVERY);
}

TEST(RaceStateMachine, EntersRecoveryWhenStuckAtCurvatureLimit)
{
  RaceStateMachine machine(config());
  StateObservation observation;
  enterReady(machine, observation);

  // Stationary + planner failing, but the raceline is within capability.
  observation.ego_speed = 0.0;
  observation.mppi_solver_failed = true;
  observation.required_steering = 0.10;
  const auto safe = runFor(machine, observation, 1.6);
  EXPECT_EQ(safe.safety_state, SafetyState::READY);

  // Stationary + infeasible curvature, but the planner is healthy.
  observation.mppi_solver_failed = false;
  observation.required_steering = 0.45;
  const auto healthy = runFor(machine, observation, 1.6);
  EXPECT_EQ(healthy.safety_state, SafetyState::READY);

  // Stationary + planner failing + curvature beyond the mechanical limit.
  observation.mppi_solver_failed = true;
  const auto result = runFor(machine, observation, 1.6);
  EXPECT_EQ(result.safety_state, SafetyState::RECOVERY);
  EXPECT_TRUE(result.recovery_active);
  EXPECT_TRUE(result.stop_requested);
  EXPECT_DOUBLE_EQ(result.recovery_speed, 0.5);
  // required_steering > 0 with reverse_steer_sign = -1.0 -> reverse hard left.
  EXPECT_DOUBLE_EQ(result.recovery_steering, -0.32);
}

TEST(RaceStateMachine, ExitsRecoveryAfterReverseWhenPlannerClears)
{
  RaceStateMachine machine(recoveryConfig());
  StateObservation observation;
  enterReady(machine, observation);
  enterRecovery(machine, observation);

  // Reverse 0.8 s at 0.5 m/s -> reverse_distance = 0.4 >= 0.3 minimum.
  observation.ego_speed = -0.5;
  static_cast<void>(runFor(machine, observation, 0.8));
  EXPECT_EQ(machine.update(observation).safety_state, SafetyState::RECOVERY);

  // Planner recovers; after the clear window and minimum reverse distance the
  // machine must return to READY and drop the reverse command.
  observation.mppi_solver_failed = false;
  const auto result = runFor(machine, observation, 1.2);
  EXPECT_EQ(result.safety_state, SafetyState::READY);
  EXPECT_FALSE(result.recovery_active);
  EXPECT_EQ(result.reason, "recovery_complete");
}

TEST(RaceStateMachine, RecoveryForcedExitOnMaxDuration)
{
  StateMachineConfig cfg = recoveryConfig();
  cfg.recovery_min_reverse_distance = 0.3;
  RaceStateMachine machine(cfg);
  StateObservation observation;
  enterReady(machine, observation);
  enterRecovery(machine, observation);

  // Planner keeps failing and the car cannot move (e.g. rear blocked): the
  // clear window never opens and reverse_distance stays below the minimum, so
  // the maximum-duration backstop must force the exit.
  observation.ego_speed = 0.0;
  observation.mppi_solver_failed = true;
  const auto result = runFor(machine, observation, 2.2);
  EXPECT_EQ(result.safety_state, SafetyState::READY);
  EXPECT_EQ(result.reason, "recovery_limit");
}

TEST(RaceStateMachine, RecoveryFaultsOnStaleInput)
{
  RaceStateMachine machine(config());
  StateObservation observation;
  enterReady(machine, observation);
  enterRecovery(machine, observation);

  observation.inputs_ready = false;
  const auto result = machine.update(observation);
  EXPECT_EQ(result.safety_state, SafetyState::FAULT);
  EXPECT_TRUE(result.stop_requested);
  EXPECT_FALSE(result.recovery_active);
}

TEST(RaceStateMachine, RecoveryReentryIsDebounced)
{
  StateMachineConfig cfg = recoveryConfig();  // reentry_debounce = 1.0
  RaceStateMachine machine(cfg);
  StateObservation observation;
  enterReady(machine, observation);
  enterRecovery(machine, observation);

  // Reverse a little, let the planner clear, exit to READY.
  observation.ego_speed = -0.5;
  static_cast<void>(runFor(machine, observation, 0.8));
  observation.mppi_solver_failed = false;
  const auto exited = runFor(machine, observation, 1.2);
  ASSERT_EQ(exited.safety_state, SafetyState::READY);

  // Immediately stuck again: reentry is gated by reentry_debounce (1.0 s).
  observation.ego_speed = 0.0;
  observation.mppi_solver_failed = true;
  observation.required_steering = 0.45;
  const auto debounced = runFor(machine, observation, 1.5);
  EXPECT_EQ(debounced.safety_state, SafetyState::READY);

  // Once the debounce elapses the stuck accumulator runs again and re-enters.
  const auto reentered = runFor(machine, observation, 1.5);
  EXPECT_EQ(reentered.safety_state, SafetyState::RECOVERY);
}

TEST(RaceStateMachine, RejectsInvalidInputAndConfiguration)
{
  auto invalid = config();
  invalid.return_blend_duration = 0.0;
  EXPECT_THROW(RaceStateMachine machine(invalid), std::invalid_argument);
  invalid = config();
  invalid.maximum_follow_distance = invalid.follow_distance - 0.1;
  EXPECT_THROW(RaceStateMachine machine(invalid), std::invalid_argument);

  RaceStateMachine machine(config());
  StateObservation observation;
  observation.time = 1.0;
  static_cast<void>(machine.update(observation));
  observation.time = 0.5;
  EXPECT_THROW(machine.update(observation), std::invalid_argument);
}

}  // namespace
}  // namespace state_machine
