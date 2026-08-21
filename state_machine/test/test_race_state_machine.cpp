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
  value.stuck_confirmation = 0.30;
  value.recovery_reverse_distance = 0.10;
  value.recovery_minimum_success_distance = 0.08;
  value.recovery_max_reverse_time = 1.0;
  value.recovery_settle_confirmation = 0.10;
  value.recovery_cooldown = 0.50;
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

TEST(RaceStateMachine, RecoversAConfirmedStationaryForwardCommand)
{
  RaceStateMachine machine(config());
  StateObservation observation;
  enterReady(machine, observation);
  observation.command_available = true;
  observation.commanded_speed = 0.40;
  observation.ego_speed = 0.0;
  observation.reverse_path_clear = true;

  auto result = runFor(machine, observation, 0.40);
  ASSERT_EQ(result.behavior_state, BehaviorState::RECOVERY);
  EXPECT_EQ(result.recovery_phase, RecoveryPhase::REVERSE);
  EXPECT_DOUBLE_EQ(result.speed_scale, 0.0);
  EXPECT_FALSE(result.stop_requested);

  observation.commanded_speed = -0.25;
  observation.ego_speed = -0.20;
  result = runFor(machine, observation, 0.60);
  ASSERT_EQ(result.behavior_state, BehaviorState::RECOVERY);
  EXPECT_EQ(result.recovery_phase, RecoveryPhase::SETTLE);

  observation.commanded_speed = 0.0;
  observation.ego_speed = 0.0;
  result = runFor(machine, observation, 0.20);
  EXPECT_EQ(result.behavior_state, BehaviorState::RACING);
  EXPECT_EQ(result.recovery_phase, RecoveryPhase::NONE);
  EXPECT_EQ(result.reason, "recovery_complete");
}

TEST(RaceStateMachine, RecoversFromZeroSpeedBrakingFallbackWhileRacing)
{
  RaceStateMachine machine(config());
  StateObservation observation;
  enterReady(machine, observation);
  observation.command_available = true;
  observation.commanded_speed = 0.0;
  observation.ego_speed = 0.0;
  observation.reverse_path_clear = true;

  const auto result = runFor(machine, observation, 0.40);
  EXPECT_EQ(result.behavior_state, BehaviorState::RECOVERY);
  EXPECT_EQ(result.recovery_phase, RecoveryPhase::REVERSE);
  EXPECT_EQ(result.reason, "vehicle_stuck");
}

TEST(RaceStateMachine, DoesNotClaimSuccessAfterAReverseTimeoutWithoutProgress)
{
  RaceStateMachine machine(config());
  StateObservation observation;
  enterReady(machine, observation);
  observation.command_available = true;
  observation.commanded_speed = 0.40;
  observation.ego_speed = 0.0;
  observation.reverse_path_clear = true;

  auto result = runFor(machine, observation, 0.40);
  ASSERT_EQ(result.behavior_state, BehaviorState::RECOVERY);
  result = runFor(machine, observation, 1.10);
  EXPECT_TRUE(result.stop_requested);
  EXPECT_EQ(result.safety_state, SafetyState::STOP);
  EXPECT_EQ(result.reason, "recovery_insufficient_progress");
}

TEST(RaceStateMachine, StopsIfRearPathBlocksBeforeUsefulProgress)
{
  RaceStateMachine machine(config());
  StateObservation observation;
  enterReady(machine, observation);
  observation.command_available = true;
  observation.commanded_speed = 0.40;
  observation.ego_speed = 0.0;
  observation.reverse_path_clear = true;

  auto result = runFor(machine, observation, 0.40);
  ASSERT_EQ(result.behavior_state, BehaviorState::RECOVERY);
  observation.reverse_path_clear = false;
  result = runFor(machine, observation, 0.05);
  EXPECT_TRUE(result.stop_requested);
  EXPECT_EQ(result.safety_state, SafetyState::STOP);
  EXPECT_EQ(result.reason, "recovery_reverse_blocked");
}

TEST(RaceStateMachine, DoesNotReverseIntoABlockedRearCorridor)
{
  RaceStateMachine machine(config());
  StateObservation observation;
  enterReady(machine, observation);
  observation.command_available = true;
  observation.commanded_speed = 0.40;
  observation.ego_speed = 0.0;
  observation.reverse_path_clear = false;

  const auto result = runFor(machine, observation, 1.0);
  EXPECT_EQ(result.behavior_state, BehaviorState::RACING);
  EXPECT_EQ(result.recovery_phase, RecoveryPhase::NONE);
}

TEST(RaceStateMachine, MissingCommandStopsAnActiveRecovery)
{
  RaceStateMachine machine(config());
  StateObservation observation;
  enterReady(machine, observation);
  observation.command_available = true;
  observation.commanded_speed = 0.40;
  observation.reverse_path_clear = true;
  auto result = runFor(machine, observation, 0.40);
  ASSERT_EQ(result.behavior_state, BehaviorState::RECOVERY);

  observation.command_available = false;
  result = machine.update(observation);
  EXPECT_EQ(result.safety_state, SafetyState::FAULT);
  EXPECT_TRUE(result.stop_requested);
  EXPECT_EQ(result.reason, "recovery_command_unavailable");
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

TEST(RaceStateMachine, RejectsInvalidInputAndConfiguration)
{
  auto invalid = config();
  invalid.return_blend_duration = 0.0;
  EXPECT_THROW(RaceStateMachine machine(invalid), std::invalid_argument);
  invalid = config();
  invalid.maximum_follow_distance = invalid.follow_distance - 0.1;
  EXPECT_THROW(RaceStateMachine machine(invalid), std::invalid_argument);
  invalid = config();
  invalid.recovery_reverse_distance = 0.0;
  EXPECT_THROW(RaceStateMachine machine(invalid), std::invalid_argument);
  invalid = config();
  invalid.recovery_minimum_success_distance =
    invalid.recovery_reverse_distance + 0.01;
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
