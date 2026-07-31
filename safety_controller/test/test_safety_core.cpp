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
  const double lidar_x = base_x - 0.250;
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

TEST(SafetyCore, AebReleaseRequiresContinuousClearHoldAndRampsResume)
{
  SafetyConfig config;
  config.aeb_clear_hold_time = 0.20;
  config.aeb_release_check_speed = 1.50;
  config.aeb_resume_acceleration = 1.00;
  config.aeb_steering_recovery_enabled = false;
  SafetyCore core(config);
  updateFreshInputs(core, 1.00, 1.0, 0.0);
  core.updateScan(singlePointScan(0.80, 0.0), 1.00);

  const ArbitrationResult triggered = core.evaluate(1.01);
  ASSERT_EQ(triggered.stop_reason, StopReason::kAeb);
  EXPECT_TRUE(triggered.aeb_latched);

  updateFreshInputs(core, 1.02, 0.8, 0.0);
  const ArbitrationResult first_clear = core.evaluate(1.02);
  EXPECT_EQ(first_clear.stop_reason, StopReason::kAeb);
  EXPECT_TRUE(first_clear.aeb_latched);

  updateFreshInputs(core, 1.15, 0.5, 0.0);
  const ArbitrationResult held = core.evaluate(1.15);
  EXPECT_EQ(held.stop_reason, StopReason::kAeb);
  EXPECT_TRUE(held.aeb_latched);
  EXPECT_NEAR(held.aeb_clear_duration, 0.13, 1e-9);

  updateFreshInputs(core, 1.23, 0.2, 0.0);
  const ArbitrationResult released = core.evaluate(1.23);
  EXPECT_EQ(released.stop_reason, StopReason::kNone);
  EXPECT_FALSE(released.aeb_latched);
  EXPECT_TRUE(released.aeb_resume_active);
  EXPECT_GT(released.command.speed, 0.0);
  EXPECT_LT(released.command.speed, 0.10);

  updateFreshInputs(core, 1.33, 0.2, 0.0);
  const ArbitrationResult ramping = core.evaluate(1.33);
  EXPECT_EQ(ramping.stop_reason, StopReason::kNone);
  EXPECT_TRUE(ramping.aeb_resume_active);
  EXPECT_NEAR(ramping.command.speed, 0.18, 1e-9);
}

TEST(SafetyCore, AebObstacleReappearanceResetsClearHold)
{
  SafetyConfig config;
  config.aeb_clear_hold_time = 0.20;
  config.aeb_steering_recovery_enabled = false;
  SafetyCore core(config);
  updateFreshInputs(core, 1.00, 1.0, 0.0);
  core.updateScan(singlePointScan(0.80, 0.0), 1.00);
  ASSERT_EQ(core.evaluate(1.01).stop_reason, StopReason::kAeb);

  updateFreshInputs(core, 1.02, 0.8, 0.0);
  ASSERT_EQ(core.evaluate(1.02).stop_reason, StopReason::kAeb);

  updateFreshInputs(core, 1.12, 0.7, 0.0);
  core.updateScan(singlePointScan(0.70, 0.0), 1.12);
  const ArbitrationResult retriggered = core.evaluate(1.12);
  EXPECT_TRUE(retriggered.aeb.emergency);
  EXPECT_DOUBLE_EQ(retriggered.aeb_clear_duration, 0.0);

  updateFreshInputs(core, 1.13, 0.5, 0.0);
  EXPECT_EQ(core.evaluate(1.13).stop_reason, StopReason::kAeb);
  updateFreshInputs(core, 1.25, 0.2, 0.0);
  EXPECT_EQ(core.evaluate(1.25).stop_reason, StopReason::kAeb);
  updateFreshInputs(core, 1.34, 0.1, 0.0);
  EXPECT_EQ(core.evaluate(1.34).stop_reason, StopReason::kNone);
}

TEST(SafetyCore, ZeroUpstreamCommandDoesNotCancelAebResumeRamp)
{
  SafetyConfig config;
  config.aeb_clear_hold_time = 0.10;
  config.aeb_resume_acceleration = 1.00;
  SafetyCore core(config);
  updateFreshInputs(core, 1.00, 1.0, 0.0);
  core.updateScan(singlePointScan(0.70, 0.0), 1.00);
  ASSERT_EQ(core.evaluate(1.01).stop_reason, StopReason::kAeb);

  updateFreshInputs(core, 1.02, 0.5, 0.0);
  core.updateCommand(ControllerMode::kMppi, DriveCommand{0.0, 0.0}, 1.02);
  ASSERT_EQ(core.evaluate(1.02).stop_reason, StopReason::kAeb);
  updateFreshInputs(core, 1.13, 0.2, 0.0);
  core.updateCommand(ControllerMode::kMppi, DriveCommand{0.0, 0.0}, 1.13);
  const ArbitrationResult released_at_zero = core.evaluate(1.13);
  EXPECT_EQ(released_at_zero.stop_reason, StopReason::kNone);
  EXPECT_TRUE(released_at_zero.aeb_resume_active);
  EXPECT_DOUBLE_EQ(released_at_zero.command.speed, 0.0);

  updateFreshInputs(core, 1.18, 0.2, 0.0);
  const ArbitrationResult resumed = core.evaluate(1.18);
  EXPECT_TRUE(resumed.aeb_resume_active);
  EXPECT_GT(resumed.command.speed, 0.0);
  EXPECT_LT(resumed.command.speed, 0.20);
}

TEST(SafetyCore, MovingEmergencyStopClampsRetainedSteering)
{
  SafetyConfig config;
  config.min_command_steering = -0.32;
  config.max_command_steering = 0.32;
  SafetyCore core(config);
  core.updateState(1.0, 0.50, 1.0);
  core.updateScan(singlePointScan(0.31, 0.0), 1.0);
  core.updateCommand(ControllerMode::kMppi, DriveCommand{1.0, 0.0}, 1.0);

  const ArbitrationResult result = core.evaluate(1.01);
  EXPECT_EQ(result.stop_reason, StopReason::kAeb);
  EXPECT_DOUBLE_EQ(result.command.speed, 0.0);
  EXPECT_DOUBLE_EQ(result.command.steering_angle, config.max_command_steering);
  EXPECT_DOUBLE_EQ(core.currentSteeringAngle(), 0.50);
}

TEST(SafetyCore, StationaryEmergencyStopCentersAndClearsSteeringLatch)
{
  SafetyConfig config;
  config.min_command_steering = -0.32;
  config.max_command_steering = 0.32;
  config.aeb_steering_recovery_enabled = false;
  SafetyCore core(config);
  core.updateState(0.0, 0.30, 1.0);
  core.updateScan(singlePointScan(0.31, 0.0), 1.0);
  core.updateCommand(ControllerMode::kMppi, DriveCommand{1.0, -0.10}, 1.0);

  const ArbitrationResult result = core.evaluate(1.01);
  EXPECT_EQ(result.stop_reason, StopReason::kAeb);
  EXPECT_DOUBLE_EQ(result.command.speed, 0.0);
  EXPECT_DOUBLE_EQ(result.command.steering_angle, 0.0);
  EXPECT_DOUBLE_EQ(core.currentSteeringAngle(), 0.30);
}

TEST(SafetyCore, AebSteeringRecoveryTurnsAtZeroSpeedBeforeRelease)
{
  SafetyConfig config;
  config.min_command_steering = -0.32;
  config.max_command_steering = 0.32;
  config.aeb_clear_hold_time = 0.20;
  config.aeb_steering_recovery_rate = 0.80;
  config.aeb_steering_recovery_tolerance = 0.01;
  SafetyCore core(config);
  core.updateState(1.0, 0.0, 1.00);
  core.updateScan(singlePointScan(0.70, 0.0), 1.00);
  core.updateCommand(ControllerMode::kMppi, DriveCommand{1.0, 0.20}, 1.00);
  ASSERT_EQ(core.evaluate(1.01).stop_reason, StopReason::kAeb);

  core.updateState(0.0, 0.0, 1.02, false);
  core.updateScan(clearScan(), 1.02);
  core.updateCommand(ControllerMode::kMppi, DriveCommand{1.0, 0.20}, 1.02);
  const ArbitrationResult started = core.evaluate(1.02);
  EXPECT_EQ(started.stop_reason, StopReason::kAeb);
  EXPECT_TRUE(started.aeb_steering_recovery_active);
  EXPECT_FALSE(started.aeb_steering_recovery_ready);
  EXPECT_GT(started.command.steering_angle, 0.0);
  EXPECT_DOUBLE_EQ(started.command.speed, 0.0);

  core.updateState(0.0, 0.0, 1.30, false);
  core.updateScan(clearScan(), 1.30);
  core.updateCommand(ControllerMode::kMppi, DriveCommand{1.0, 0.20}, 1.30);
  const ArbitrationResult at_target = core.evaluate(1.30);
  EXPECT_EQ(at_target.stop_reason, StopReason::kAeb);
  // The command has reached 0.20 rad, but the effective wheel angle still
  // follows the actuator response model and must not release in this cycle.
  EXPECT_FALSE(at_target.aeb_steering_recovery_ready);
  EXPECT_NEAR(at_target.command.steering_angle, 0.20, 1e-12);

  core.updateState(0.0, 0.0, 1.51, false);
  core.updateScan(clearScan(), 1.51);
  core.updateCommand(ControllerMode::kMppi, DriveCommand{1.0, 0.20}, 1.51);
  const ArbitrationResult effective_at_target = core.evaluate(1.51);
  EXPECT_EQ(effective_at_target.stop_reason, StopReason::kAeb);
  EXPECT_TRUE(effective_at_target.aeb_steering_recovery_ready);

  core.updateState(0.0, 0.0, 1.72, false);
  core.updateScan(clearScan(), 1.72);
  core.updateCommand(ControllerMode::kMppi, DriveCommand{1.0, 0.20}, 1.72);
  const ArbitrationResult released = core.evaluate(1.72);
  EXPECT_EQ(released.stop_reason, StopReason::kNone);
  EXPECT_TRUE(released.aeb_resume_active);
  EXPECT_GT(released.command.speed, 0.0);
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

TEST(SafetyCore, ConfiguredVehicleSelfReturnDoesNotTriggerAeb)
{
  SafetyConfig config;
  config.self_filter_enabled = true;
  config.self_filter_min_x = 0.05;
  config.self_filter_max_x = 0.35;
  config.self_filter_min_y = -0.22;
  config.self_filter_max_y = 0.22;
  SafetyCore core(config);
  updateFreshInputs(core, 1.0, 0.0, 0.0);

  // Reproduces the dominant real-vehicle return: 3.5 cm from the lidar at
  // about +12 degrees, or approximately (0.284, 0.007) in base_link.
  ScanData self_return = singlePointScan(0.284, 0.007);
  self_return.range_min = 0.023;
  core.updateScan(self_return, 1.01);
  const ArbitrationResult result = core.evaluate(1.02);

  EXPECT_FALSE(result.aeb.emergency);
  EXPECT_EQ(result.aeb.self_filtered_beams, 1U);
  EXPECT_EQ(result.stop_reason, StopReason::kNone);
}

TEST(SafetyCore, SelfFilterDoesNotHideObstacleAheadOfVehicle)
{
  SafetyConfig config;
  config.self_filter_enabled = true;
  config.self_filter_min_x = 0.05;
  config.self_filter_max_x = 0.35;
  config.self_filter_min_y = -0.22;
  config.self_filter_max_y = 0.22;
  SafetyCore core(config);
  updateFreshInputs(core, 1.0, 1.0, 0.0);

  core.updateScan(singlePointScan(0.70, 0.0), 1.01);
  const ArbitrationResult result = core.evaluate(1.02);

  EXPECT_TRUE(result.aeb.emergency);
  EXPECT_EQ(result.aeb.self_filtered_beams, 0U);
  EXPECT_EQ(result.stop_reason, StopReason::kAeb);
}

TEST(SafetyCore, SteeringChangesTheAebSweptPath)
{
  const ScanData obstacle = singlePointScan(0.81, 0.22);

  SafetyCore straight_core;
  updateFreshInputs(straight_core, 1.0, 1.0, 0.0);
  straight_core.updateCommand(ControllerMode::kMppi, DriveCommand{1.5, 0.0}, 1.0);
  straight_core.updateScan(obstacle, 1.01);
  EXPECT_FALSE(straight_core.evaluate(1.02).aeb.emergency);

  SafetyCore turning_core;
  updateFreshInputs(turning_core, 1.0, 1.0, 0.30);
  turning_core.updateScan(obstacle, 1.01);
  const ArbitrationResult turning_result = turning_core.evaluate(1.02);
  EXPECT_TRUE(turning_result.aeb.emergency);
  EXPECT_EQ(turning_result.stop_reason, StopReason::kAeb);
}

TEST(SafetyCore, ServoTargetAndEffectiveSteeringRemainDecoupledAtSpeed)
{
  SafetyConfig config;
  config.state_timeout = 2.0;
  config.scan_timeout = 2.0;
  config.command_timeout = 2.0;
  SafetyCore core(config);
  core.updateState(2.0, 0.0, 1.00);
  core.updateScan(clearScan(), 1.00);
  core.updateCommand(ControllerMode::kMppi, DriveCommand{2.0, 0.20}, 1.00);

  const ArbitrationResult commanded = core.evaluate(1.00);
  ASSERT_EQ(commanded.stop_reason, StopReason::kNone);
  EXPECT_NEAR(commanded.command.steering_angle, 0.20, 1e-12);
  EXPECT_NEAR(commanded.estimated_effective_steering, 0.0, 1e-12);

  core.updateState(2.0, 0.0, 1.05, false);
  const ArbitrationResult responding = core.evaluate(1.05);
  EXPECT_GT(responding.estimated_effective_steering, 0.0);
  EXPECT_LT(responding.estimated_effective_steering, 0.20);

  core.updateState(2.0, 0.0, 2.00, false);
  const ArbitrationResult settled = core.evaluate(2.00);
  // effectiveness = 1 - 0.05 * speed^2 = 0.8 at 2 m/s.
  EXPECT_NEAR(settled.estimated_effective_steering, 0.16, 1e-12);
  EXPECT_NEAR(settled.command.steering_angle, 0.20, 1e-12);
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
  SafetyConfig config;
  SafetyCore core(config);
  core.updateState(1.0, 0.0, 1.0);
  core.updateScan(clearScan(), 1.0);
  core.updateCommand(ControllerMode::kMppi, DriveCommand{2.01, 0.0}, 1.0);
  EXPECT_EQ(core.evaluate(1.01).stop_reason, StopReason::kInvalidCommand);

  core.updateCommand(
    ControllerMode::kMppi,
    DriveCommand{1.0, config.min_command_steering - 0.001}, 1.02);
  EXPECT_EQ(core.evaluate(1.03).stop_reason, StopReason::kInvalidCommand);
}

TEST(SafetyCore, Float32EnvelopeRoundoffIsAcceptedAndClamped)
{
  SafetyConfig config;
  config.max_command_speed = 0.20;
  config.min_command_steering = -0.20;
  config.max_command_steering = 0.20;
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
