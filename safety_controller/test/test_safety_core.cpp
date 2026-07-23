#include <cmath>
#include <limits>

#include "gtest/gtest.h"
#include "safety_controller/safety_core.hpp"

namespace safety_controller
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

ScanData clearScan()
{
  ScanData scan;
  scan.ranges.assign(181U, std::numeric_limits<double>::infinity());
  scan.angle_min = -0.5 * kPi;
  scan.angle_increment = kPi / 180.0;
  scan.range_min = 0.05;
  scan.range_max = 10.0;
  return scan;
}

ScanData singlePointScan(double base_x, double base_y)
{
  ScanData scan;
  const double lidar_x = base_x - 0.275;
  scan.ranges = {std::hypot(lidar_x, base_y)};
  scan.angle_min = std::atan2(base_y, lidar_x);
  scan.angle_increment = 0.01;
  scan.range_min = 0.05;
  scan.range_max = 10.0;
  return scan;
}

void updateFreshInputs(
  SafetyCore & core, double now, double speed = 1.0, double steering = 0.10)
{
  core.updateState(speed, steering, now);
  core.updateScan(clearScan(), now);
  core.updateCommand(ControllerMode::kMppi, DriveCommand{1.5, 0.12}, now);
  core.updateCommand(ControllerMode::kFtg, DriveCommand{0.8, -0.08}, now);
}

TEST(SafetyCore, SelectedControllerIsForwardedWithoutAutomaticSwitch)
{
  SafetyCore core;
  updateFreshInputs(core, 1.0);

  const ArbitrationResult result = core.evaluate(1.05);
  EXPECT_FALSE(result.stopped());
  EXPECT_EQ(result.selected_mode, ControllerMode::kMppi);
  EXPECT_DOUBLE_EQ(result.command.speed, 1.5);
  EXPECT_NEAR(result.command.steering_angle, 0.12, 1e-12);
}

TEST(SafetyCore, AebOverridesFreshSelectedCommandAndRetainsSteering)
{
  SafetyCore core;
  updateFreshInputs(core, 1.0, 1.0, 0.10);
  core.updateScan(singlePointScan(0.80, 0.0), 1.04);

  const ArbitrationResult result = core.evaluate(1.05);
  EXPECT_TRUE(result.stopped());
  EXPECT_EQ(result.stop_reason, StopReason::kAeb);
  EXPECT_TRUE(result.aeb.emergency);
  EXPECT_DOUBLE_EQ(result.command.speed, 0.0);
  EXPECT_NEAR(result.command.steering_angle, 0.10, 1e-12);
}

TEST(SafetyCore, ObstacleOutsideSweptVehicleDoesNotStop)
{
  SafetyCore core;
  updateFreshInputs(core, 1.0, 1.0, 0.0);
  core.updateScan(singlePointScan(0.80, 0.60), 1.04);

  const ArbitrationResult result = core.evaluate(1.05);
  EXPECT_FALSE(result.stopped());
  EXPECT_FALSE(result.aeb.emergency);
  EXPECT_DOUBLE_EQ(result.command.speed, 1.5);
}

TEST(SafetyCore, BelowMinimumRangeReturnIsIgnoredRatherThanClampedIntoAebPath)
{
  SafetyCore core;
  updateFreshInputs(core, 1.0, 1.0, 0.0);
  ScanData scan = clearScan();
  scan.ranges[scan.ranges.size() / 2U] = 0.0;
  core.updateScan(scan, 1.01);

  const ArbitrationResult result = core.evaluate(1.02);
  EXPECT_FALSE(result.stopped());
  EXPECT_FALSE(result.aeb.emergency);
  EXPECT_EQ(result.aeb.valid_beams, scan.ranges.size() - 1U);
}

TEST(SafetyCore, SteeringChangesTheAebSweptPath)
{
  const ScanData obstacle = singlePointScan(0.81, 0.22);

  SafetyCore straight_core;
  updateFreshInputs(straight_core, 1.0, 1.0, 0.0);
  straight_core.updateScan(obstacle, 1.01);
  EXPECT_FALSE(straight_core.evaluate(1.02).aeb.emergency);

  SafetyCore turning_core;
  updateFreshInputs(turning_core, 1.0, 1.0, 0.30);
  turning_core.updateScan(obstacle, 1.01);
  const ArbitrationResult turning_result = turning_core.evaluate(1.02);
  EXPECT_TRUE(turning_result.aeb.emergency);
  EXPECT_EQ(turning_result.stop_reason, StopReason::kAeb);
}

TEST(SafetyCore, ModeChangeRequiresFreshLowSpeedState)
{
  SafetyCore core;
  updateFreshInputs(core, 1.0, 0.25, 0.0);
  EXPECT_FALSE(core.requestMode(ControllerMode::kFtg, 1.01));
  EXPECT_EQ(core.selectedMode(), ControllerMode::kMppi);

  core.updateState(0.19, 0.0, 1.02);
  EXPECT_TRUE(core.requestMode(ControllerMode::kFtg, 1.03));
  EXPECT_EQ(core.selectedMode(), ControllerMode::kFtg);
  const ArbitrationResult result = core.evaluate(1.04);
  EXPECT_FALSE(result.stopped());
  EXPECT_DOUBLE_EQ(result.command.speed, 0.8);
}

TEST(SafetyCore, ModeChangeRejectsStaleStateAndExactThreshold)
{
  SafetyCore core;
  core.updateState(0.10, 0.0, 0.0);
  EXPECT_FALSE(core.requestMode(ControllerMode::kFtg, 0.101));

  core.updateState(0.20, 0.0, 1.0);
  EXPECT_FALSE(core.requestMode(ControllerMode::kFtg, 1.01));
}

TEST(SafetyCore, StateTimeoutStopsBeforeUsingFreshCommand)
{
  SafetyCore core;
  core.updateState(1.0, 0.17, 0.0);
  core.updateScan(clearScan(), 0.101);
  core.updateCommand(ControllerMode::kMppi, DriveCommand{1.0, -0.1}, 0.101);

  const ArbitrationResult result = core.evaluate(0.101);
  EXPECT_EQ(result.stop_reason, StopReason::kStateTimeout);
  EXPECT_DOUBLE_EQ(result.command.speed, 0.0);
  EXPECT_NEAR(result.command.steering_angle, 0.17, 1e-12);
}

TEST(SafetyCore, ScanTimeoutStops)
{
  SafetyCore core;
  core.updateScan(clearScan(), 0.0);
  core.updateState(1.0, 0.0, 0.151);
  core.updateCommand(ControllerMode::kMppi, DriveCommand{1.0, 0.0}, 0.151);

  EXPECT_EQ(core.evaluate(0.151).stop_reason, StopReason::kScanTimeout);
}

TEST(SafetyCore, SelectedCommandTimeoutStopsEvenWhenOtherModeIsFresh)
{
  SafetyCore core;
  core.updateCommand(ControllerMode::kMppi, DriveCommand{1.0, 0.1}, 0.0);
  core.updateCommand(ControllerMode::kFtg, DriveCommand{1.0, -0.1}, 0.101);
  core.updateState(1.0, 0.1, 0.101);
  core.updateScan(clearScan(), 0.101);

  const ArbitrationResult result = core.evaluate(0.101);
  EXPECT_EQ(result.stop_reason, StopReason::kCommandTimeout);
  EXPECT_EQ(result.selected_mode, ControllerMode::kMppi);
}

TEST(SafetyCore, ReplayedOldInputsRemainTimedOut)
{
  SafetyCore core;
  updateFreshInputs(core, 1.0);

  // The caller supplies source timestamps, so receiving these messages later
  // must not make replayed data fresh.
  const ArbitrationResult result = core.evaluate(1.20);
  EXPECT_TRUE(result.stopped());
  EXPECT_EQ(result.stop_reason, StopReason::kStateTimeout);
  EXPECT_NEAR(result.state_age, 0.20, 1e-12);
  EXPECT_NEAR(result.scan_age, 0.20, 1e-12);
  EXPECT_NEAR(result.command_age, 0.20, 1e-12);
}

TEST(SafetyCore, InvalidScanStops)
{
  SafetyCore core;
  core.updateState(1.0, 0.0, 1.0);
  core.updateCommand(ControllerMode::kMppi, DriveCommand{1.0, 0.0}, 1.0);
  ScanData invalid;
  core.updateScan(invalid, 1.0);

  EXPECT_EQ(core.evaluate(1.01).stop_reason, StopReason::kInvalidScan);

  ScanData nan_scan = clearScan();
  nan_scan.ranges.assign(20U, std::numeric_limits<double>::quiet_NaN());
  core.updateScan(nan_scan, 1.02);
  EXPECT_EQ(core.evaluate(1.03).stop_reason, StopReason::kInvalidScan);
}

TEST(SafetyCore, InvalidSelectedCommandStops)
{
  SafetyCore core;
  core.updateState(1.0, 0.0, 1.0);
  core.updateScan(clearScan(), 1.0);
  core.updateCommand(
    ControllerMode::kMppi,
    DriveCommand{std::numeric_limits<double>::quiet_NaN(), 0.0}, 1.0);

  EXPECT_EQ(core.evaluate(1.01).stop_reason, StopReason::kInvalidCommand);
}

TEST(SafetyCore, FiniteCommandOutsideIndependentEnvelopeStops)
{
  SafetyCore core;
  core.updateState(1.0, 0.0, 1.0);
  core.updateScan(clearScan(), 1.0);
  core.updateCommand(ControllerMode::kMppi, DriveCommand{2.01, 0.0}, 1.0);
  EXPECT_EQ(core.evaluate(1.01).stop_reason, StopReason::kInvalidCommand);

  core.updateCommand(ControllerMode::kMppi, DriveCommand{1.0, -0.201}, 1.02);
  EXPECT_EQ(core.evaluate(1.03).stop_reason, StopReason::kInvalidCommand);
}

TEST(SafetyCore, Float32EnvelopeRoundoffIsAcceptedAndClamped)
{
  SafetyConfig config;
  config.max_command_speed = 0.20;
  SafetyCore core(config);
  core.updateState(0.10, 0.0, 1.0);
  core.updateScan(clearScan(), 1.0);

  const double float32_upper = static_cast<double>(static_cast<float>(0.20));
  const double float32_lower = static_cast<double>(static_cast<float>(-0.20));
  ASSERT_GT(float32_upper, config.max_command_speed);
  ASSERT_LT(float32_lower, config.min_command_steering);

  core.updateCommand(
    ControllerMode::kMppi, DriveCommand{float32_upper, float32_upper}, 1.0);
  ArbitrationResult result = core.evaluate(1.01);
  EXPECT_EQ(result.stop_reason, StopReason::kNone);
  EXPECT_DOUBLE_EQ(result.command.speed, config.max_command_speed);
  EXPECT_DOUBLE_EQ(result.command.steering_angle, config.max_command_steering);

  core.updateCommand(
    ControllerMode::kMppi, DriveCommand{0.10, float32_lower}, 1.02);
  result = core.evaluate(1.03);
  EXPECT_EQ(result.stop_reason, StopReason::kNone);
  EXPECT_DOUBLE_EQ(result.command.steering_angle, config.min_command_steering);
}

}  // namespace
}  // namespace safety_controller
