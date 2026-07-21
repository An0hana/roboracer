#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <numeric>
#include <utility>
#include <vector>

#include "ftg_controller/ftg_core.hpp"

namespace ftg_controller
{
namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kOneDegree = kPi / 180.0;

ProcessedScan makeScan(std::size_t count = 181U, double default_range = 0.0)
{
  ProcessedScan scan;
  scan.angle_increment = kOneDegree;
  scan.ranges.assign(count, default_range);
  scan.angles.reserve(count);
  const double middle = static_cast<double>(count - 1U) * 0.5;
  for (std::size_t index = 0; index < count; ++index) {
    scan.angles.push_back((static_cast<double>(index) - middle) * scan.angle_increment);
  }
  return scan;
}

void openGap(ProcessedScan & scan, std::size_t begin, std::size_t end, double range)
{
  ASSERT_LE(begin, end);
  ASSERT_LE(end, scan.ranges.size());
  std::fill(scan.ranges.begin() + static_cast<std::ptrdiff_t>(begin),
    scan.ranges.begin() + static_cast<std::ptrdiff_t>(end), range);
}

ProcessedScan mirrored(const ProcessedScan & input)
{
  ProcessedScan output = input;
  std::reverse(output.ranges.begin(), output.ranges.end());
  for (std::size_t index = 0; index < output.angles.size(); ++index) {
    output.angles[index] = -input.angles[input.angles.size() - 1U - index];
  }
  return output;
}

TEST(PreprocessScan, SanitizesNonFiniteRangesAndNeverSmoothsThroughAnObstacle)
{
  FTGConfig config;
  config.smoothing_window = 3;
  config.max_lidar_range = 10.0;
  config.min_lidar_range = 0.05;
  FollowTheGapPlanner planner(config);

  const std::vector<float> input{
    std::numeric_limits<float>::infinity(), 8.0F, 1.0F, 8.0F,
    std::numeric_limits<float>::quiet_NaN(),
    -std::numeric_limits<float>::infinity(), 0.01F};
  const auto scan = planner.preprocessScan(input, -3.0 * kOneDegree, kOneDegree, 0.02, 12.0);

  ASSERT_TRUE(scan.valid());
  ASSERT_EQ(scan.ranges.size(), input.size());
  EXPECT_GT(scan.ranges[0], 0.0);
  EXPECT_LE(scan.ranges[0], 10.0);
  EXPECT_LE(scan.ranges[1], 8.0);
  EXPECT_DOUBLE_EQ(scan.ranges[2], 1.0);
  EXPECT_LE(scan.ranges[3], 8.0);
  EXPECT_DOUBLE_EQ(scan.ranges[4], 0.0);
  EXPECT_DOUBLE_EQ(scan.ranges[5], 0.0);
  EXPECT_DOUBLE_EQ(scan.ranges[6], 0.0);
  for (const double range : scan.ranges) {
    EXPECT_TRUE(std::isfinite(range));
    EXPECT_GE(range, 0.0);
    EXPECT_LE(range, config.max_lidar_range);
  }
}

TEST(TraversabilityMask, ExpandsEveryDisparityButHasNoGlobalClosestPointBubble)
{
  FTGConfig config;
  config.smoothing_window = 1;
  config.disparity_threshold = 0.4;
  config.safety_radius = 0.35;
  config.min_clearance = 0.45;
  FollowTheGapPlanner planner(config);

  std::vector<double> ranges(30U, 5.0);
  ranges[6] = 1.0;
  ranges[22] = 1.5;
  const auto mask = planner.createTraversabilityMask(ranges, 0.1);

  ASSERT_EQ(mask.size(), ranges.size());
  // Both sides of both discontinuities must be expanded into the farther ranges.
  EXPECT_EQ(mask[5], 0U);
  EXPECT_EQ(mask[7], 0U);
  EXPECT_EQ(mask[21], 0U);
  EXPECT_EQ(mask[23], 0U);

  FTGConfig no_disparity_config = config;
  no_disparity_config.disparity_threshold = 100.0;
  FollowTheGapPlanner no_disparity_planner(no_disparity_config);
  const auto no_bubble_mask =
    no_disparity_planner.createTraversabilityMask(ranges, 0.1);
  ASSERT_EQ(no_bubble_mask.size(), ranges.size());
  EXPECT_TRUE(std::all_of(
      no_bubble_mask.begin(), no_bubble_mask.end(),
      [](std::uint8_t value) {return value != 0U;}));
}

TEST(GapDetection, RejectsPhysicalGapsNarrowerThanVehicleRequirement)
{
  FTGConfig config;
  config.min_clearance = 0.45;
  config.min_gap_width = 0.65;
  FollowTheGapPlanner planner(config);

  const std::vector<double> ranges(100U, 2.0);
  std::vector<std::uint8_t> mask(100U, 0U);
  std::fill(mask.begin() + 5, mask.begin() + 15, 1U);    // About 0.35 m wide.
  std::fill(mask.begin() + 40, mask.begin() + 70, 1U);  // About 1.0 m wide.

  const auto gaps = planner.findGaps(ranges, mask, kOneDegree);

  ASSERT_EQ(gaps.size(), 1U);
  EXPECT_EQ(gaps.front().begin, 40U);
  EXPECT_EQ(gaps.front().end, 70U);
  EXPECT_GE(gaps.front().physical_width, config.min_gap_width);
}

TEST(TargetSelection, SymmetricStraightRemainsStableUnderDeterministicNoise)
{
  FTGConfig config;
  config.smoothing_window = 1;
  FollowTheGapPlanner planner(config);

  std::vector<double> steering_history;
  steering_history.reserve(80U);
  int meaningful_sign_changes = 0;
  int previous_sign = 0;
  for (int frame = 0; frame < 80; ++frame) {
    auto scan = makeScan(181U, 8.0);
    for (std::size_t index = 0; index < scan.ranges.size(); ++index) {
      const int pattern = static_cast<int>(
        (index * 17U + static_cast<std::size_t>(frame) * 13U) % 11U);
      scan.ranges[index] += 0.01 * static_cast<double>(pattern - 5);
    }
    // Alternate which side owns the single largest raw return. The target should not chase it.
    scan.ranges[frame % 2 == 0 ? 65U : 115U] += 0.08;

    const PlanResult result = planner.plan(scan);
    ASSERT_TRUE(result.ok()) << "frame " << frame << ", stop=" << toString(result.stop_reason);
    steering_history.push_back(result.steering_angle);
    const int sign = result.steering_angle > 1e-3 ? 1 : (result.steering_angle < -1e-3 ? -1 : 0);
    if (sign != 0 && previous_sign != 0 && sign != previous_sign) {
      ++meaningful_sign_changes;
    }
    if (sign != 0) {
      previous_sign = sign;
    }
  }

  const double squared_sum = std::inner_product(
    steering_history.begin(), steering_history.end(), steering_history.begin(), 0.0);
  const double rms = std::sqrt(squared_sum / static_cast<double>(steering_history.size()));
  EXPECT_LT(rms, 0.04);
  EXPECT_LE(meaningful_sign_changes, 1);
}

TEST(TargetSelection, HysteresisRetainsSafeTargetAndReleasesItWhenGapDisappears)
{
  FTGConfig config;
  config.smoothing_window = 1;
  config.disparity_threshold = 100.0;
  FollowTheGapPlanner planner(config);

  auto first_scan = makeScan();
  openGap(first_scan, 45U, 76U, 5.00);
  openGap(first_scan, 105U, 136U, 5.08);
  const PlanResult first = planner.plan(first_scan);
  ASSERT_TRUE(first.ok());
  ASSERT_GT(first.target_angle, 0.0);

  auto small_challenge = makeScan();
  openGap(small_challenge, 45U, 76U, 5.08);
  openGap(small_challenge, 105U, 136U, 5.00);
  const PlanResult retained = planner.plan(small_challenge);
  ASSERT_TRUE(retained.ok());
  EXPECT_TRUE(retained.retained_previous_target);
  EXPECT_GT(retained.target_angle, 0.0);

  auto previous_gap_removed = makeScan();
  openGap(previous_gap_removed, 45U, 76U, 5.0);
  const PlanResult switched = planner.plan(previous_gap_removed);
  ASSERT_TRUE(switched.ok());
  EXPECT_FALSE(switched.retained_previous_target);
  EXPECT_LT(switched.target_angle, 0.0);
}

TEST(PathGeometry, MirroredBendsProduceOppositeSteeringAndEqualClearanceAndSpeed)
{
  FTGConfig config;
  config.smoothing_window = 1;
  config.disparity_threshold = 100.0;
  auto left_scan = makeScan();
  openGap(left_scan, 105U, 151U, 5.0);
  const auto right_scan = mirrored(left_scan);

  FollowTheGapPlanner left_planner(config);
  FollowTheGapPlanner right_planner(config);
  const PlanResult left = left_planner.plan(left_scan);
  const PlanResult right = right_planner.plan(right_scan);

  ASSERT_TRUE(left.ok());
  ASSERT_TRUE(right.ok());
  EXPECT_NEAR(left.steering_angle, -right.steering_angle, 1e-9);
  EXPECT_NEAR(left.curvature, -right.curvature, 1e-9);
  EXPECT_DOUBLE_EQ(left.path.collision_distance, right.path.collision_distance);
  EXPECT_DOUBLE_EQ(left.path.lateral_margin, right.path.lateral_margin);

  CommandController left_controller;
  CommandController right_controller;
  const CommandResult left_command =
    left_controller.update(left.curvature, left.path.collision_distance, 0.01);
  const CommandResult right_command =
    right_controller.update(right.curvature, right.path.collision_distance, 0.01);
  EXPECT_NEAR(left_command.steering_angle, -right_command.steering_angle, 1e-12);
  EXPECT_NEAR(left_command.speed, right_command.speed, 1e-12);
  EXPECT_FALSE(left_command.emergency);
  EXPECT_FALSE(right_command.emergency);
}

TEST(TargetSelection, SharpTurnWithWideSweptPathDoesNotBecomeNoGap)
{
  FTGConfig config;
  config.smoothing_window = 1;
  config.disparity_threshold = 100.0;
  FollowTheGapPlanner planner(config);

  for (const std::pair<std::size_t, std::size_t> & indices :
    {std::pair<std::size_t, std::size_t>{125U, 166U}, {145U, 181U}})
  {
    planner.reset();
    auto scan = makeScan();
    openGap(scan, indices.first, indices.second, 5.0);
    const PlanResult plan = planner.plan(scan);
    ASSERT_TRUE(plan.ok()) << "stop=" << toString(plan.stop_reason);
    EXPECT_NE(plan.stop_reason, StopReason::kNoGap);
    EXPECT_GT(std::abs(plan.curvature), 0.1);

    CommandController controller;
    const CommandResult command =
      controller.update(plan.curvature, plan.path.collision_distance, 0.01);
    EXPECT_FALSE(command.emergency);
    EXPECT_GT(command.speed, 0.0);
  }
}

TEST(PathGeometry, ObstacleInsideSweptPathTriggersEmergencyStop)
{
  FTGConfig config;
  config.smoothing_window = 1;
  FollowTheGapPlanner planner(config);
  auto scan = makeScan(181U, 8.0);
  scan.ranges[90] = 0.10;

  const PathMetrics path = planner.evaluatePath(scan.ranges, scan.angles, 0.0);
  EXPECT_LE(path.collision_distance, config.emergency_distance);

  CommandController controller;
  controller.reset(2.0, 0.0);
  const CommandResult command = controller.update(0.0, path.collision_distance, 0.01);
  EXPECT_TRUE(command.emergency);
  EXPECT_DOUBLE_EQ(command.speed, 0.0);
  EXPECT_DOUBLE_EQ(command.acceleration, 0.0);
}

TEST(CommandController, ControlledStopRespectsDecelerationJerkAndKeepsSteering)
{
  CommandConfig config;
  config.max_decel = 3.0;
  config.max_jerk = 15.0;
  CommandController controller(config);
  controller.reset(3.0, 0.2);

  constexpr double dt = 0.1;
  double previous_speed = controller.currentSpeed();
  double previous_acceleration = controller.currentAcceleration();
  for (int step = 0; step < 20; ++step) {
    const CommandResult command = controller.controlledStop(dt);
    EXPECT_DOUBLE_EQ(command.steering_angle, 0.2);
    EXPECT_FALSE(command.emergency);
    EXPECT_GE(command.acceleration, -config.max_decel - 1e-12);
    EXPECT_LE(
      std::abs(command.acceleration - previous_acceleration), config.max_jerk * dt + 1e-12);
    EXPECT_LE(previous_speed - command.speed, config.max_decel * dt + 1e-12);
    EXPECT_LE(command.speed, previous_speed + 1e-12);
    EXPECT_GE(command.speed, 0.0);
    previous_speed = command.speed;
    previous_acceleration = command.acceleration;
  }
  EXPECT_DOUBLE_EQ(controller.currentSpeed(), 0.0);
}

TEST(CommandController, OneSecondResponseIsNearlyIndependentOfScanFrequency)
{
  struct Snapshot
  {
    double steering;
    double speed;
  };
  const auto simulate = [](int frequency) {
      CommandController controller;
      const double dt = 1.0 / static_cast<double>(frequency);
      CommandResult command;
      for (int step = 0; step < frequency; ++step) {
        command = controller.update(0.4, std::numeric_limits<double>::infinity(), dt);
      }
      return Snapshot{command.steering_angle, command.speed};
    };

  const Snapshot at_40_hz = simulate(40);
  const Snapshot at_100_hz = simulate(100);
  const Snapshot at_250_hz = simulate(250);
  EXPECT_NEAR(at_40_hz.steering, at_100_hz.steering, 0.02);
  EXPECT_NEAR(at_40_hz.steering, at_250_hz.steering, 0.02);
  EXPECT_NEAR(at_40_hz.speed, at_100_hz.speed, 0.1);
  EXPECT_NEAR(at_40_hz.speed, at_250_hz.speed, 0.1);
}

TEST(CommandController, NonpositiveTimeDoesNotAdvanceFilteredState)
{
  CommandController controller;
  controller.reset(2.0, 0.1);

  const CommandResult zero_dt =
    controller.update(0.8, std::numeric_limits<double>::infinity(), 0.0);
  EXPECT_DOUBLE_EQ(zero_dt.steering_angle, 0.1);
  EXPECT_DOUBLE_EQ(zero_dt.speed, 2.0);
  EXPECT_DOUBLE_EQ(zero_dt.acceleration, 0.0);

  const CommandResult backwards_time = controller.controlledStop(-0.01);
  EXPECT_DOUBLE_EQ(backwards_time.steering_angle, 0.1);
  EXPECT_DOUBLE_EQ(backwards_time.speed, 2.0);
  EXPECT_DOUBLE_EQ(backwards_time.acceleration, 0.0);
}

}  // namespace
}  // namespace ftg_controller
