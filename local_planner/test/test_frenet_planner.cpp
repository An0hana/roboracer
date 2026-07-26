// Copyright 2026 RoboRacer Team

#include <algorithm>
#include <cmath>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

#include "gtest/gtest.h"
#include "local_planner/frenet_planner.hpp"

namespace local_planner
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

ReferenceLine circularLine(double width = 2.0)
{
  constexpr std::size_t count = 200U;
  constexpr double radius = 5.0;
  std::vector<ReferencePoint> points;
  points.reserve(count);
  for (std::size_t index = 0U; index < count; ++index) {
    const double angle =
      2.0 * kPi * static_cast<double>(index) / static_cast<double>(count);
    ReferencePoint point;
    point.s = radius * angle;
    point.x = radius * std::cos(angle);
    point.y = radius * std::sin(angle);
    point.yaw = angle + 0.5 * kPi;
    point.curvature = 1.0 / radius;
    point.speed = 2.0;
    point.width_left = width;
    point.width_right = width;
    points.push_back(point);
  }
  return ReferenceLine::fromPoints(std::move(points));
}

FrenetPlannerConfig config()
{
  FrenetPlannerConfig value;
  value.planning_horizons = {3.0, 4.0, 5.0};
  value.speed_scales = {0.60, 0.80, 1.00};
  value.lateral_samples_per_side = 3U;
  value.sample_spacing = 0.10;
  value.overtake_offset = 0.45;
  value.projection_search_radius = 30U;
  value.max_projection_distance = 1.0;
  value.max_projection_heading_error = 60.0 * kPi / 180.0;
  value.minimum_frenet_jacobian = 0.20;
  value.max_curvature = 1.0;
  value.weight_switch = 3.0;
  return value;
}

EgoState egoOnLine()
{
  EgoState ego;
  ego.x = 5.0;
  ego.y = 0.0;
  ego.yaw = 0.5 * kPi;
  ego.speed = 2.0;
  return ego;
}

Obstacle obstacleAtProgress(double progress)
{
  constexpr double radius = 5.0;
  const double angle = progress / radius;
  Obstacle obstacle;
  obstacle.id = 7;
  obstacle.classification = 1U;
  obstacle.x = radius * std::cos(angle);
  obstacle.y = radius * std::sin(angle);
  obstacle.yaw = angle + 0.5 * kPi;
  obstacle.length = 0.552;
  obstacle.width = 0.320;
  obstacle.confidence = 1.0;
  obstacle.visible = true;
  return obstacle;
}

const CandidateTrajectory * findCandidate(
  const PlanResult & result, double target_d, double horizon,
  double speed_scale)
{
  const auto candidate = std::find_if(
    result.candidates.begin(), result.candidates.end(),
    [target_d, horizon, speed_scale](const CandidateTrajectory & value) {
      return std::abs(value.target_d - target_d) < 1.0e-6 &&
      std::abs(value.horizon - horizon) < 1.0e-6 &&
      std::abs(value.speed_scale - speed_scale) < 1.0e-6;
    });
  return candidate == result.candidates.end() ? nullptr : &(*candidate);
}

TEST(ReferenceLine, ProjectsWithSignedLateralOffset)
{
  const auto line = circularLine();
  const auto outside = line.project(5.20, 0.0);
  const auto inside = line.project(4.80, 0.0);
  EXPECT_NEAR(outside.s, 0.0, 0.02);
  EXPECT_NEAR(outside.d, -0.20, 0.01);
  EXPECT_NEAR(inside.d, 0.20, 0.01);
}

TEST(ReferenceLine, RejectsProjectionAgainstTravelDirection)
{
  const auto line = circularLine();
  const auto projection = line.project(
    5.0, 0.0, std::nullopt, 30U, -0.5 * kPi, 60.0 * kPi / 180.0);
  EXPECT_GT(projection.distance, 1.0);
}

TEST(FrenetPlanner, SelectsNominalWithoutObstacle)
{
  FrenetPlanner planner(circularLine(), config());
  const auto result = planner.plan(egoOnLine(), {});
  ASSERT_TRUE(result.valid);
  ASSERT_EQ(result.candidates.size(), 63U);
  const auto & selected = result.candidates[result.selected_index];
  EXPECT_NEAR(selected.target_d, 0.0, 1.0e-9);
  EXPECT_NEAR(selected.horizon, 5.0, 1.0e-9);
  EXPECT_NEAR(selected.speed_scale, 1.0, 1.0e-9);
  EXPECT_GT(selected.points.size(), 40U);
}

TEST(FrenetPlanner, StartsAtMeasuredSpeedAndRespectsAccelerationLimits)
{
  FrenetPlanner planner(circularLine(), config());
  auto ego = egoOnLine();
  ego.speed = 1.0;
  const auto result = planner.plan(ego, {});
  ASSERT_TRUE(result.valid);
  const auto & selected = result.candidates[result.selected_index];
  ASSERT_GT(selected.points.size(), 2U);
  EXPECT_NEAR(selected.points.front().speed, ego.speed, 1.0e-9);
  for (std::size_t index = 1U; index < selected.points.size(); ++index) {
    EXPECT_LE(
      selected.points[index].acceleration,
      config().max_acceleration + 1.0e-6);
    EXPECT_GE(
      selected.points[index].acceleration,
      config().min_acceleration - 1.0e-6);
  }
}

TEST(FrenetPlanner, GeneratesQuinticBoundaryTrajectory)
{
  FrenetPlanner planner(circularLine(), config());
  auto ego = egoOnLine();
  ego.yaw += 0.10;
  ego.curvature = 0.25;
  const auto result = planner.plan(ego, {});
  const auto * left = findCandidate(result, 0.45, 5.0, 1.0);
  ASSERT_NE(left, nullptr);
  ASSERT_TRUE(left->valid);
  ASSERT_GT(left->points.size(), 40U);
  EXPECT_NEAR(left->points.front().x, ego.x, 1.0e-9);
  EXPECT_NEAR(left->points.front().y, ego.y, 1.0e-9);
  EXPECT_NEAR(left->points.front().yaw, ego.yaw, 1.0e-9);
  EXPECT_NEAR(left->points.front().curvature, 0.25, 1.0e-9);
  EXPECT_NEAR(left->points.front().d, 0.0, 1.0e-6);
  EXPECT_NEAR(left->points.back().d, 0.45, 1.0e-6);
  const auto terminal_reference = planner.referenceLine().sample(
    left->points.back().s);
  EXPECT_NEAR(
    std::sin(left->points.back().yaw - terminal_reference.yaw),
    0.0, 1.0e-6);
  const double expected_terminal_curvature =
    terminal_reference.curvature /
    (1.0 - terminal_reference.curvature * left->points.back().d);
  EXPECT_NEAR(
    left->points.back().curvature, expected_terminal_curvature, 1.0e-6);
}

TEST(FrenetPlanner, PrunesLateralSamplesUsingTrackWidth)
{
  FrenetPlanner planner(circularLine(0.50), config());
  const auto result = planner.plan(egoOnLine(), {});
  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.candidates.size(), 45U);
  EXPECT_TRUE(
    std::all_of(
      result.candidates.begin(), result.candidates.end(),
      [](const CandidateTrajectory & candidate) {
        return std::abs(candidate.target_d) <= 0.28 + 1.0e-6;
      }));
}

TEST(FrenetPlanner, AvoidsObstacleUsingSideCandidate)
{
  FrenetPlanner planner(circularLine(), config());
  const auto obstacle = obstacleAtProgress(3.0);
  const auto result = planner.plan(
    egoOnLine(), {obstacle}, std::nullopt, 0.45);
  ASSERT_TRUE(result.valid);
  EXPECT_TRUE(
    std::all_of(
      result.candidates.begin(), result.candidates.end(),
      [](const CandidateTrajectory & candidate) {
        return std::abs(candidate.target_d) >= 1.0e-9 ||
        !candidate.valid || candidate.stopping;
      }));
  EXPECT_GT(result.candidates[result.selected_index].target_d, 0.0);
  EXPECT_FALSE(result.candidates[result.selected_index].stopping);
  EXPECT_GE(
    result.candidates[result.selected_index].target_d,
    0.5 * config().vehicle_width +
    0.5 * obstacle.width + config().collision_margin);
}

TEST(FrenetPlanner, TrailsWhenAvailableOffsetCannotClearOpponent)
{
  auto planner_config = config();
  planner_config.overtake_offset = 0.30;
  FrenetPlanner planner(circularLine(), planner_config);
  const auto result = planner.plan(
    egoOnLine(), {obstacleAtProgress(3.0)});
  ASSERT_TRUE(result.valid);
  const auto & selected = result.candidates[result.selected_index];
  EXPECT_TRUE(selected.stopping);
  EXPECT_NEAR(selected.target_d, 0.0, 1.0e-9);
  EXPECT_TRUE(
    std::none_of(
      result.candidates.begin(), result.candidates.end(),
      [](const CandidateTrajectory & candidate) {
        return candidate.valid &&
        std::abs(candidate.target_d) > 1.0e-9;
      }));
}

TEST(FrenetPlanner, ShortTransitionCannotIgnoreObstacleBeyondEndpoint)
{
  FrenetPlanner planner(circularLine(), config());
  const auto result = planner.plan(
    egoOnLine(), {obstacleAtProgress(4.0)}, std::nullopt, 0.45);
  ASSERT_TRUE(result.valid);
  EXPECT_TRUE(
    std::all_of(
      result.candidates.begin(), result.candidates.end(),
      [](const CandidateTrajectory & candidate) {
        return std::abs(candidate.target_d) >= 1.0e-9 ||
        !candidate.valid || candidate.stopping;
      }));
  EXPECT_GT(
    std::abs(result.candidates[result.selected_index].target_d), 0.10);
  EXPECT_FALSE(result.candidates[result.selected_index].stopping);
}

TEST(FrenetPlanner, StopsOnRacelineWhenEveryPassingCandidateIsBlocked)
{
  FrenetPlanner planner(circularLine(0.35), config());
  const auto result = planner.plan(
    egoOnLine(), {obstacleAtProgress(3.0)});
  ASSERT_TRUE(result.valid);
  const auto & selected = result.candidates[result.selected_index];
  EXPECT_TRUE(selected.stopping);
  EXPECT_EQ(selected.reason, "trailing_stop");
  EXPECT_NEAR(selected.target_d, 0.0, 1.0e-9);
  EXPECT_GT(selected.stop_distance, 0.0);
  ASSERT_FALSE(selected.points.empty());
  EXPECT_LE(selected.points.front().speed, egoOnLine().speed);
  EXPECT_NEAR(selected.points.back().speed, 0.0, 1.0e-9);
  EXPECT_TRUE(
    std::any_of(
      result.candidates.begin(), result.candidates.end(),
      [](const CandidateTrajectory & candidate) {
        return !candidate.valid && candidate.reason == "collision";
      }));
}

TEST(FrenetPlanner, RejectsStopWhenObstacleIsInsideBrakingDistance)
{
  FrenetPlanner planner(circularLine(0.35), config());
  auto ego = egoOnLine();
  ego.speed = 4.0;
  const auto result = planner.plan(
    ego, {obstacleAtProgress(3.0)});
  EXPECT_FALSE(result.valid);
  EXPECT_TRUE(
    std::none_of(
      result.candidates.begin(), result.candidates.end(),
      [](const CandidateTrajectory & candidate) {
        return candidate.stopping;
      }));
}

TEST(FrenetPlanner, UsesRearAxleReferenceForVehicleFootprint)
{
  FrenetPlanner planner(circularLine(), config());
  Obstacle obstacle;
  obstacle.id = 9;
  obstacle.x = 5.0;
  obstacle.y = -0.30;
  obstacle.yaw = 0.5 * kPi;
  obstacle.length = 0.05;
  obstacle.width = 0.10;
  obstacle.confidence = 1.0;
  obstacle.visible = true;
  const auto result = planner.plan(egoOnLine(), {obstacle});
  ASSERT_TRUE(result.valid);
  const auto * nominal = findCandidate(result, 0.0, 5.0, 1.0);
  ASSERT_NE(nominal, nullptr);
  EXPECT_TRUE(nominal->valid);
}

TEST(FrenetPlanner, RejectsInvalidConfiguration)
{
  auto invalid = config();
  invalid.speed_scales.clear();
  EXPECT_THROW(
    FrenetPlanner planner(circularLine(), invalid),
    std::invalid_argument);
}

TEST(FrenetPlanner, RestrictsCandidatesToPreferredSide)
{
  FrenetPlanner planner(circularLine(), config());
  const auto result = planner.plan(
    egoOnLine(), {}, std::nullopt, std::nullopt, -1);
  ASSERT_TRUE(result.valid);
  ASSERT_FALSE(result.candidates.empty());
  EXPECT_TRUE(
    std::all_of(
      result.candidates.begin(), result.candidates.end(),
      [](const CandidateTrajectory & candidate) {
        return candidate.target_d < 0.0;
      }));
}

TEST(FrenetPlanner, PreferredSideNeverCommandsReverseLateralMotion)
{
  FrenetPlanner planner(circularLine(), config());
  auto ego = egoOnLine();
  ego.x = 4.80;
  const auto result = planner.plan(
    ego, {}, std::nullopt, std::nullopt, 1);
  ASSERT_TRUE(result.valid);
  ASSERT_FALSE(result.candidates.empty());
  EXPECT_TRUE(
    std::all_of(
      result.candidates.begin(), result.candidates.end(),
      [&result](const CandidateTrajectory & candidate) {
        return candidate.target_d + 1.0e-9 >= result.ego_d;
      }));
}

TEST(FrenetPlanner, ReturnModeOnlySamplesRaceline)
{
  FrenetPlanner planner(circularLine(), config());
  auto ego = egoOnLine();
  ego.x = 4.70;
  const auto result = planner.plan(
    ego, {}, std::nullopt, std::nullopt, std::nullopt, true, 0.8);
  ASSERT_TRUE(result.valid);
  ASSERT_EQ(result.candidates.size(), 9U);
  EXPECT_TRUE(
    std::all_of(
      result.candidates.begin(), result.candidates.end(),
      [](const CandidateTrajectory & candidate) {
        return std::abs(candidate.target_d) < 1.0e-9 &&
        candidate.speed_scale <= 0.8;
      }));
  const auto & selected = result.candidates[result.selected_index];
  EXPECT_NEAR(selected.points.front().d, 0.30, 0.01);
  EXPECT_NEAR(selected.points.back().d, 0.0, 1.0e-6);
}

TEST(FrenetPlanner, RejectsOppositeHeading)
{
  FrenetPlanner planner(circularLine(), config());
  auto ego = egoOnLine();
  ego.yaw = -0.5 * kPi;
  const auto result = planner.plan(ego, {});
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reason, "projection_too_far");
  EXPECT_TRUE(result.candidates.empty());
}

}  // namespace
}  // namespace local_planner
