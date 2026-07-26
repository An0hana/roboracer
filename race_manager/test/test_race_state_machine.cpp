// Copyright 2026 RoboRacer Team

#include <cmath>
#include <stdexcept>

#include "gtest/gtest.h"
#include "race_manager/race_state_machine.hpp"

namespace race_manager
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
  return value;
}

StateObservation readyObservation()
{
  StateObservation observation;
  observation.inputs_ready = true;
  observation.trajectory_valid = true;
  return observation;
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

void enterGlobal(
  RaceStateMachine & machine, StateObservation & observation)
{
  static_cast<void>(machine.update(observation));
  const auto result = runFor(machine, observation, 0.60);
  ASSERT_EQ(result.state, BehaviorState::GLOBAL_TRACK);
}

void enterTrailing(
  RaceStateMachine & machine, StateObservation & observation)
{
  observation.opponent_detected = true;
  observation.opponent_longitudinal = 2.0;
  observation.opponent_lateral = 0.10;
  const auto result = runFor(machine, observation, 0.70);
  ASSERT_EQ(result.state, BehaviorState::TRAILING);
}

void enterOvertake(
  RaceStateMachine & machine, StateObservation & observation,
  double selected_lateral)
{
  observation.selected_lateral = selected_lateral;
  const auto result = runFor(machine, observation, 0.70);
  ASSERT_EQ(result.state, BehaviorState::OVERTAKE);
}

TEST(RaceStateMachine, WaitsForInputsThenCruises)
{
  RaceStateMachine machine(config());
  StateObservation observation;
  auto result = machine.update(observation);
  EXPECT_EQ(result.state, BehaviorState::INIT);
  EXPECT_FALSE(result.use_local_trajectory);

  observation = readyObservation();
  result = runFor(machine, observation, 0.60);
  EXPECT_EQ(result.state, BehaviorState::GLOBAL_TRACK);
  EXPECT_TRUE(result.use_local_trajectory);
  EXPECT_DOUBLE_EQ(result.speed_scale, 1.0);
}

TEST(RaceStateMachine, ToleratesTransientInvalidTrajectoryAtStartup)
{
  RaceStateMachine machine(config());
  auto observation = readyObservation();
  observation.trajectory_valid = false;
  auto result = runFor(machine, observation, 0.30);
  EXPECT_EQ(result.state, BehaviorState::INIT);
  EXPECT_EQ(result.reason, "waiting_for_valid_trajectory");

  observation.trajectory_valid = true;
  result = runFor(machine, observation, 0.60);
  EXPECT_EQ(result.state, BehaviorState::GLOBAL_TRACK);
}

TEST(RaceStateMachine, FaultsWhenInitialTrajectoryRemainsInvalid)
{
  RaceStateMachine machine(config());
  auto observation = readyObservation();
  observation.trajectory_valid = false;
  const auto result = runFor(machine, observation, 0.60);
  EXPECT_EQ(result.state, BehaviorState::FAULT);
  EXPECT_EQ(result.reason, "initial_trajectory_invalid");
}

TEST(RaceStateMachine, CompletesOvertakeStateSequence)
{
  RaceStateMachine machine(config());
  auto observation = readyObservation();
  enterGlobal(machine, observation);
  enterTrailing(machine, observation);

  auto result = machine.update(observation);
  EXPECT_EQ(result.state, BehaviorState::TRAILING);
  EXPECT_DOUBLE_EQ(result.speed_scale, 0.60);

  enterOvertake(machine, observation, 0.25);
  result = machine.update(observation);
  EXPECT_EQ(result.preferred_side, PreferredSide::LEFT);
  EXPECT_DOUBLE_EQ(result.speed_scale, 1.0);

  observation.opponent_longitudinal = -0.50;
  result = runFor(machine, observation, 0.70);
  EXPECT_EQ(result.state, BehaviorState::RETURN);
  EXPECT_EQ(result.preferred_side, PreferredSide::NONE);
  EXPECT_DOUBLE_EQ(result.speed_scale, 0.80);

  observation.opponent_detected = false;
  observation.selected_lateral = 0.0;
  result = runFor(machine, observation, 0.70);
  EXPECT_EQ(result.state, BehaviorState::GLOBAL_TRACK);
}

TEST(RaceStateMachine, SelectsRightOvertakeSide)
{
  RaceStateMachine machine(config());
  auto observation = readyObservation();
  enterGlobal(machine, observation);
  enterTrailing(machine, observation);
  enterOvertake(machine, observation, -0.20);
  const auto result = machine.update(observation);
  EXPECT_EQ(result.preferred_side, PreferredSide::RIGHT);
  EXPECT_EQ(result.reason, "safe_right_trajectory");
}

TEST(RaceStateMachine, KeepsTrailingWhenOnlyStoppingTrajectoryIsAvailable)
{
  RaceStateMachine machine(config());
  auto observation = readyObservation();
  enterGlobal(machine, observation);
  enterTrailing(machine, observation);

  observation.selected_lateral = 0.0;
  observation.trajectory_valid = true;
  const auto result = runFor(machine, observation, 1.0);
  EXPECT_EQ(result.state, BehaviorState::TRAILING);
  EXPECT_FALSE(result.fallback_ftg);
  EXPECT_TRUE(result.use_local_trajectory);
  EXPECT_DOUBLE_EQ(result.speed_scale, 0.60);
}

TEST(RaceStateMachine, RejectsShortOpponentDetectionSpike)
{
  RaceStateMachine machine(config());
  auto observation = readyObservation();
  enterGlobal(machine, observation);

  observation.opponent_detected = true;
  observation.opponent_longitudinal = 2.0;
  runFor(machine, observation, 0.10);
  observation.opponent_detected = false;
  const auto result = runFor(machine, observation, 0.30);
  EXPECT_EQ(result.state, BehaviorState::GLOBAL_TRACK);
}

TEST(RaceStateMachine, FaultsImmediatelyAndRecoversWithDebounce)
{
  RaceStateMachine machine(config());
  auto observation = readyObservation();
  enterGlobal(machine, observation);

  observation.trajectory_valid = false;
  auto result = machine.update(observation);
  EXPECT_EQ(result.state, BehaviorState::FAULT);
  EXPECT_TRUE(result.fallback_ftg);
  EXPECT_FALSE(result.use_local_trajectory);

  observation.trajectory_valid = true;
  result = runFor(machine, observation, 0.30);
  EXPECT_EQ(result.state, BehaviorState::FAULT);
  result = runFor(machine, observation, 0.30);
  EXPECT_EQ(result.state, BehaviorState::GLOBAL_TRACK);
}

TEST(RaceStateMachine, ReturnsWhenOpponentIsLostDuringOvertake)
{
  RaceStateMachine machine(config());
  auto observation = readyObservation();
  enterGlobal(machine, observation);
  enterTrailing(machine, observation);
  enterOvertake(machine, observation, 0.25);

  observation.opponent_detected = false;
  const auto result = runFor(machine, observation, 0.80);
  EXPECT_EQ(result.state, BehaviorState::RETURN);
  EXPECT_EQ(result.reason, "opponent_lost_after_overtake");
}

TEST(RaceStateMachine, RejectsNonMonotonicTime)
{
  RaceStateMachine machine(config());
  auto observation = readyObservation();
  observation.time = 1.0;
  static_cast<void>(machine.update(observation));
  observation.time = 0.5;
  EXPECT_THROW(machine.update(observation), std::invalid_argument);
}

TEST(RaceStateMachine, RejectsInvalidConfiguration)
{
  auto invalid = config();
  invalid.return_lateral_threshold = invalid.overtake_lateral_threshold;
  EXPECT_THROW(RaceStateMachine machine(invalid), std::invalid_argument);
}

}  // namespace
}  // namespace race_manager
