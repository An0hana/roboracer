#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <limits>
#include <string>
#include <vector>

#include "mppi_controller/mppi_core.hpp"

namespace mppi_controller
{
namespace
{

RaceLine makeCircle(bool mirrored = false, double half_width = 1.0)
{
  constexpr std::size_t count = 160U;
  constexpr double radius = 5.0;
  std::vector<Waypoint> points;
  points.reserve(count);
  for (std::size_t index = 0U; index < count; ++index) {
    const double theta = 2.0 * kPi * static_cast<double>(index) /
      static_cast<double>(count);
    const double sign = mirrored ? -1.0 : 1.0;
    points.push_back(Waypoint{
      radius * theta,
      radius * std::cos(theta),
      sign * radius * std::sin(theta),
      normalizeAngle(sign * (theta + kPi * 0.5)),
      sign / radius,
      1.2,
      half_width,
      half_width});
  }
  return RaceLine::fromWaypoints(std::move(points));
}

MppiConfig fastConfig()
{
  MppiConfig config;
  config.rollout_count = 256U;
  config.horizon_steps = 12U;
  config.repair_steps = 4U;
  config.repair_clearance = 0.0;
  config.random_seed = 1234U;
  return config;
}

TEST(BicycleModel, Rk4IntegratesStraightAndCurvedMotion)
{
  BicycleModel model;
  const State straight = model.step(State{0.0, 0.0, 0.0, 1.0, 0.0}, Control{}, 0.1);
  EXPECT_NEAR(straight.x, 0.1, 1.0e-10);
  EXPECT_NEAR(straight.y, 0.0, 1.0e-10);
  EXPECT_NEAR(straight.yaw, 0.0, 1.0e-10);

  const State left = model.step(State{0.0, 0.0, 0.0, 1.0, 0.15}, Control{}, 0.1);
  const State right = model.step(State{0.0, 0.0, 0.0, 1.0, -0.15}, Control{}, 0.1);
  EXPECT_NEAR(left.x, right.x, 1.0e-12);
  EXPECT_NEAR(left.y, -right.y, 1.0e-12);
  EXPECT_NEAR(left.yaw, -right.yaw, 1.0e-12);
}

TEST(BicycleModel, EnforcesAsymmetricSteeringAndControlLimits)
{
  VehicleConfig config;
  config.min_steering = -0.231;
  config.max_steering = 0.309;
  BicycleModel model(config);
  const State bounded = model.clampState(State{0.0, 0.0, 0.0, 20.0, -1.0});
  EXPECT_DOUBLE_EQ(bounded.steering, -0.231);
  EXPECT_DOUBLE_EQ(bounded.speed, config.max_speed);
  const Control control = model.clampControl(Control{10.0, -10.0});
  EXPECT_DOUBLE_EQ(control.steering_rate, config.max_steering_rate);
  EXPECT_DOUBLE_EQ(control.acceleration, config.min_acceleration);
}

TEST(BicycleModel, PropagatesTimestampAgeWithSignedLongitudinalSpeed)
{
  VehicleConfig config;
  config.min_speed = -2.0;
  BicycleModel model(config);
  const State propagated = propagateState(
    model, State{0.0, 0.0, 0.0, -1.0, 0.0}, Control{}, 0.10);
  EXPECT_NEAR(propagated.x, -0.10, 1.0e-10);
  EXPECT_NEAR(propagated.y, 0.0, 1.0e-10);
  EXPECT_THROW(
    {
      const State unused = propagateState(
        model, State{0.0, 0.0, 0.0, 3.0, 0.0}, Control{}, 0.10);
      (void)unused;
    }, std::invalid_argument);
}

TEST(BicycleModel, DetectsImplausibleLocalizationJumps)
{
  VehicleConfig config;
  const State previous{0.0, 0.0, 0.0, 1.0, 0.0};
  EXPECT_FALSE(isLocalizationJump(
      previous, State{0.04, 0.0, 0.02, 1.0, 0.0}, 0.05, config, 0.20, 0.30));
  EXPECT_TRUE(isLocalizationJump(
      previous, State{1.0, 0.0, 0.02, 1.0, 0.0}, 0.05, config, 0.20, 0.30));
  EXPECT_TRUE(isLocalizationJump(
      previous, State{0.04, 0.0, 1.0, 1.0, 0.0}, 0.05, config, 0.20, 0.30));
}

TEST(RaceLine, ParsesStrictCsvAndWrapsLocalReference)
{
  const std::string path = "/tmp/mppi_controller_test_line.csv";
  {
    std::ofstream output(path);
    output << "s,x,y,yaw,curvature,v_ref,width_left,width_right\n";
    std::size_t index = 0U;
    for (std::size_t side = 0U; side < 4U; ++side) {
      for (std::size_t step = 0U; step < 4U; ++step, ++index) {
        const double along = 0.25 * static_cast<double>(step);
        double x = 0.0;
        double y = 0.0;
        double yaw = 0.0;
        if (side == 0U) {x = along; y = 0.0; yaw = 0.0;}
        if (side == 1U) {x = 1.0; y = along; yaw = kPi * 0.5;}
        if (side == 2U) {x = 1.0 - along; y = 1.0; yaw = kPi;}
        if (side == 3U) {x = 0.0; y = 1.0 - along; yaw = -kPi * 0.5;}
        output << 0.25 * static_cast<double>(index) << ',' << x << ',' << y << ',' <<
          yaw << ",0,1,0.8,0.7\n";
      }
    }
  }
  const RaceLine line = RaceLine::fromCsv(path);
  std::remove(path.c_str());
  ASSERT_TRUE(line.valid());
  EXPECT_NEAR(line.length(), 4.0, 1.0e-12);
  EXPECT_EQ(line.nearestIndex(1.0, 0.26), 5U);
  const auto local = line.localReference(line.size() - 1U, 3U);
  ASSERT_EQ(local.size(), 3U);
  EXPECT_DOUBLE_EQ(local[0].s, 3.75);
  EXPECT_DOUBLE_EQ(local[1].s, 0.0);
  EXPECT_DOUBLE_EQ(local[2].s, 0.25);
  EXPECT_NEAR(line.forwardProgress(3.8, 0.2), 0.4, 1.0e-12);
}

TEST(RaceLine, RejectsSpatialDiscontinuityAndInconsistentProgress)
{
  std::vector<Waypoint> discontinuous{
    Waypoint{0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0},
    Waypoint{1.0, 1.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0},
    Waypoint{2.0, 0.0, 0.1, 0.0, 0.0, 1.0, 1.0, 1.0}};
  EXPECT_THROW(RaceLine::fromWaypoints(discontinuous), std::invalid_argument);

  std::vector<Waypoint> inconsistent{
    Waypoint{0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0},
    Waypoint{0.20, 0.10, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0},
    Waypoint{0.40, 0.05, 0.08, 0.0, 0.0, 1.0, 1.0, 1.0}};
  EXPECT_THROW(RaceLine::fromWaypoints(inconsistent), std::invalid_argument);
}

TEST(RaceLine, RejectsWrongSchema)
{
  const std::string path = "/tmp/mppi_controller_bad_line.csv";
  {
    std::ofstream output(path);
    output << "x,y,s\n0,0,0\n1,0,1\n0,1,2\n";
  }
  EXPECT_THROW(RaceLine::fromCsv(path), std::runtime_error);
  std::remove(path.c_str());
}

TEST(DistanceField, RepresentsObstaclesUnknownCellsAndRotatedOrigins)
{
  std::vector<std::int8_t> grid(25U, 0);
  grid[2U * 5U + 2U] = 100;
  const DistanceField field = DistanceField::fromOccupancyGrid(
    5U, 5U, 1.0, 10.0, 20.0, grid, 50, true, kPi * 0.5);
  EXPECT_DOUBLE_EQ(field.clearance(7.5, 22.5), 0.0);
  // Clearance is conservative to the occupied cell boundary, not its center.
  EXPECT_NEAR(field.clearance(7.5, 20.5), 2.0 - std::sqrt(0.5), 1.0e-12);
  EXPECT_DOUBLE_EQ(field.clearance(100.0, 100.0), 0.0);
}

TEST(Cost, BoundaryViolationAndCbfArePenalized)
{
  const RaceLine track = makeCircle(false, 0.45);
  MppiConfig config = fastConfig();
  CpuMppiBackend backend(config, VehicleConfig{});
  std::vector<Control> controls(config.horizon_steps, Control{});
  const State centered{5.0, 0.0, kPi * 0.5, 0.5, 0.0};
  const State outside{4.45, 0.0, kPi * 0.5, 0.5, 0.0};
  const CostBreakdown center_cost = backend.evaluateTrajectory(
    centered, controls, track, nullptr);
  const CostBreakdown outside_cost = backend.evaluateTrajectory(
    outside, controls, track, nullptr);
  EXPECT_EQ(center_cost.collision, 0.0);
  EXPECT_GT(outside_cost.boundary + outside_cost.cbf + outside_cost.collision,
    center_cost.boundary + center_cost.cbf + center_cost.collision);

  const State opposite_outside{5.55, 0.0, kPi * 0.5, 0.5, 0.0};
  const CostBreakdown opposite_cost = backend.evaluateTrajectory(
    opposite_outside, controls, track, nullptr);
  EXPECT_GT(opposite_cost.boundary + opposite_cost.cbf + opposite_cost.collision,
    center_cost.boundary + center_cost.cbf + center_cost.collision);
}

TEST(Cost, DetectsObstacleInsideFootprintBetweenSparseSampleLocations)
{
  const RaceLine track = makeCircle(false, 2.0);
  MppiConfig config = fastConfig();
  CpuMppiBackend backend(config, VehicleConfig{});
  constexpr std::size_t width = 200U;
  constexpr std::size_t height = 200U;
  constexpr double resolution = 0.02;
  constexpr double origin_x = 3.0;
  constexpr double origin_y = -1.0;
  std::vector<std::int8_t> occupancy(width * height, 0);
  const auto grid_x = static_cast<std::size_t>((5.0 - origin_x) / resolution);
  const auto grid_y = static_cast<std::size_t>((0.32 - origin_y) / resolution);
  occupancy[grid_y * width + grid_x] = 100;
  const DistanceField field = DistanceField::fromOccupancyGrid(
    width, height, resolution, origin_x, origin_y, occupancy, 50, false);
  std::vector<Control> controls(1U, Control{});
  const CostBreakdown cost = backend.evaluateTrajectory(
    State{5.0, 0.0, kPi * 0.5, 0.0, 0.0}, controls, track, &field);
  EXPECT_GT(cost.collision, 0.0);
}

TEST(Cost, TreatsRepairClearanceAsHardRolloutBoundary)
{
  const RaceLine track = makeCircle(false, 2.0);
  MppiConfig permissive = fastConfig();
  MppiConfig guarded = permissive;
  guarded.repair_clearance = 0.05;
  CpuMppiBackend permissive_backend(permissive, VehicleConfig{});
  CpuMppiBackend guarded_backend(guarded, VehicleConfig{});

  constexpr std::size_t width = 400U;
  constexpr std::size_t height = 400U;
  constexpr double resolution = 0.01;
  constexpr double origin_x = 3.0;
  constexpr double origin_y = -2.0;
  std::vector<std::int8_t> occupancy(width * height, 0);
  const auto obstacle_x = static_cast<std::size_t>((5.25 - origin_x) / resolution);
  const auto obstacle_y = static_cast<std::size_t>((-0.055 - origin_y) / resolution);
  occupancy[obstacle_y * width + obstacle_x] = 100;
  const DistanceField field = DistanceField::fromOccupancyGrid(
    width, height, resolution, origin_x, origin_y, occupancy, 50, false);
  const State state{5.0, 0.0, kPi * 0.5, 0.0, 0.0};
  const std::vector<Control> controls(1U, Control{});

  const CostBreakdown permissive_cost = permissive_backend.evaluateTrajectory(
    state, controls, track, &field);
  const CostBreakdown guarded_cost = guarded_backend.evaluateTrajectory(
    state, controls, track, &field);
  EXPECT_EQ(permissive_cost.collision, 0.0);
  EXPECT_GT(guarded_cost.collision, 0.0);
  EXPECT_GT(guarded_cost.cbf, permissive_cost.cbf);
}

TEST(Cost, ClampsRaceLineSpeedToConfiguredVehicleMaximum)
{
  const RaceLine track = makeCircle();
  const MppiConfig config = fastConfig();
  VehicleConfig vehicle;
  vehicle.max_speed = 0.5;
  CpuMppiBackend backend(config, vehicle);
  const std::vector<Control> controls(1U, Control{});

  const CostBreakdown at_cap = backend.evaluateTrajectory(
    State{5.0, 0.0, kPi * 0.5, 0.5, 0.0}, controls, track, nullptr);
  const CostBreakdown below_cap = backend.evaluateTrajectory(
    State{5.0, 0.0, kPi * 0.5, 0.4, 0.0}, controls, track, nullptr);
  EXPECT_NEAR(at_cap.speed, 0.0, 1.0e-12);
  EXPECT_GT(below_cap.speed, at_cap.speed);
}

TEST(Cost, RejectsUturnAndReverseProgressTrajectories)
{
  const RaceLine track = makeCircle(false, 2.0);
  MppiConfig config = fastConfig();
  config.horizon_steps = 24U;
  config.repair_steps = 4U;
  CpuMppiBackend backend(config, VehicleConfig{});
  const std::vector<Control> controls(config.horizon_steps, Control{});

  const CostBreakdown forward = backend.evaluateTrajectory(
    State{5.0, 0.0, kPi * 0.5, 0.5, std::atan(0.324 / 5.0)},
    controls, track, nullptr);
  const CostBreakdown reversed = backend.evaluateTrajectory(
    State{5.0, 0.0, -kPi * 0.5, 0.5, 0.0},
    controls, track, nullptr);

  EXPECT_EQ(forward.collision, 0.0);
  EXPECT_GT(reversed.collision, 0.0);
}

TEST(Cost, TerminalProgressPreventsLowSpeedShortSightedSolution)
{
  const RaceLine track = makeCircle(false, 2.0);
  MppiConfig config = fastConfig();
  config.horizon_steps = 48U;
  config.minimum_preview_distance = 4.0;
  VehicleConfig vehicle;
  vehicle.max_speed = 4.0;
  CpuMppiBackend backend(config, vehicle);
  const State initial{
    5.0, 0.0, kPi * 0.5, 0.5, std::atan(vehicle.wheelbase / 5.0)};
  const std::vector<Control> coasting(config.horizon_steps, Control{});
  const std::vector<Control> accelerating(
    config.horizon_steps, Control{0.0, vehicle.max_acceleration});

  const CostBreakdown coast_cost = backend.evaluateTrajectory(
    initial, coasting, track, nullptr);
  const CostBreakdown acceleration_cost = backend.evaluateTrajectory(
    initial, accelerating, track, nullptr);

  EXPECT_GT(coast_cost.terminal_progress, acceleration_cost.terminal_progress);
  EXPECT_EQ(acceleration_cost.collision, 0.0);
}

TEST(Repair, AcceptsSafeSequenceAndRejectsUnsafeInitialState)
{
  const RaceLine track = makeCircle(false, 0.6);
  MppiConfig config = fastConfig();
  CpuMppiBackend backend(config, VehicleConfig{});
  constexpr std::size_t width = 80U;
  constexpr std::size_t height = 80U;
  constexpr double resolution = 0.05;
  constexpr double origin_x = 3.0;
  constexpr double origin_y = -2.0;
  std::vector<std::int8_t> occupancy(width * height, 0);
  const auto obstacle_x = static_cast<std::size_t>((4.0 - origin_x) / resolution);
  const auto obstacle_y = static_cast<std::size_t>((0.0 - origin_y) / resolution);
  occupancy[obstacle_y * width + obstacle_x] = 100;
  const DistanceField field = DistanceField::fromOccupancyGrid(
    width, height, resolution, origin_x, origin_y, occupancy, 50, false);
  std::vector<Control> controls(config.horizon_steps, Control{});
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(50);
  EXPECT_TRUE(backend.repairControls(
      State{5.0, 0.0, kPi * 0.5, 0.5, 0.0}, controls, track, &field, deadline));

  const auto second_deadline =
    std::chrono::steady_clock::now() + std::chrono::milliseconds(50);
  EXPECT_FALSE(backend.repairControls(
      State{4.0, 0.0, kPi * 0.5, 0.5, 0.0}, controls, track, &field,
      second_deadline));
}

TEST(Repair, HonorsExpiredRepairDeadline)
{
  const RaceLine track = makeCircle(false, 0.6);
  const MppiConfig config = fastConfig();
  CpuMppiBackend backend(config, VehicleConfig{});
  std::vector<Control> controls(config.horizon_steps, Control{});
  EXPECT_FALSE(backend.repairControls(
      State{5.0, 0.0, kPi * 0.5, 0.5, 0.0}, controls, track, nullptr,
      std::chrono::steady_clock::now() - std::chrono::milliseconds(1)));
}

TEST(MppiBackend, FixedSeedProducesDeterministicControl)
{
  const RaceLine track = makeCircle();
  const MppiConfig config = fastConfig();
  CpuMppiBackend first(config, VehicleConfig{});
  CpuMppiBackend second(config, VehicleConfig{});
  MppiRequest request;
  request.initial_state = State{5.0, 0.0, kPi * 0.5, 0.7, 0.0};
  request.race_line = &track;
  const MppiResult first_result = first.compute(request);
  const MppiResult second_result = second.compute(request);
  ASSERT_TRUE(first_result.valid) << first_result.reason;
  ASSERT_TRUE(second_result.valid) << second_result.reason;
  EXPECT_DOUBLE_EQ(first_result.control.steering_rate, second_result.control.steering_rate);
  EXPECT_DOUBLE_EQ(first_result.control.acceleration, second_result.control.acceleration);
  EXPECT_DOUBLE_EQ(first_result.cost.total(), second_result.cost.total());
}

TEST(MppiBackend, MirroredTrackProducesOppositeSteeringAndEqualAcceleration)
{
  const RaceLine left_track = makeCircle(false);
  const RaceLine right_track = makeCircle(true);
  const MppiConfig config = fastConfig();
  CpuMppiBackend left(config, VehicleConfig{});
  CpuMppiBackend right(config, VehicleConfig{});
  MppiRequest left_request;
  left_request.initial_state = State{5.0, 0.0, kPi * 0.5, 0.7, 0.0};
  left_request.race_line = &left_track;
  MppiRequest right_request;
  right_request.initial_state = State{5.0, 0.0, -kPi * 0.5, 0.7, 0.0};
  right_request.race_line = &right_track;
  const MppiResult left_result = left.compute(left_request);
  const MppiResult right_result = right.compute(right_request);
  ASSERT_TRUE(left_result.valid) << left_result.reason;
  ASSERT_TRUE(right_result.valid) << right_result.reason;
  EXPECT_NEAR(left_result.control.steering_rate, -right_result.control.steering_rate, 1.0e-10);
  EXPECT_NEAR(left_result.control.acceleration, right_result.control.acceleration, 1.0e-10);
}

TEST(MppiBackend, Default2048By32ConfigurationReturnsBoundedControl)
{
  const RaceLine track = makeCircle();
  const MppiConfig config;
  const VehicleConfig vehicle;
  CpuMppiBackend backend(config, vehicle);
  MppiRequest request;
  request.initial_state = State{5.0, 0.0, kPi * 0.5, 0.7, 0.0};
  request.race_line = &track;
  const MppiResult result = backend.compute(request);
  ASSERT_TRUE(result.valid) << result.reason;
  EXPECT_GE(result.control.steering_rate, vehicle.min_steering_rate);
  EXPECT_LE(result.control.steering_rate, vehicle.max_steering_rate);
  EXPECT_GE(result.control.acceleration, vehicle.min_acceleration);
  EXPECT_LE(result.control.acceleration, vehicle.max_acceleration);
  EXPECT_EQ(result.control_sequence.size(), config.horizon_steps);
  EXPECT_EQ(result.predicted_states.size(), config.horizon_steps + 1U);
  RecordProperty("solve_time_ms", result.solve_time_ms);
}

TEST(MppiBackend, DeterministicLaunchSampleEscapesZeroSpeedLocalOptimum)
{
  const RaceLine track = makeCircle();
  MppiConfig config = fastConfig();
  config.horizon_steps = 32U;
  VehicleConfig vehicle;
  vehicle.max_speed = 0.5;
  CpuMppiBackend backend(config, vehicle);
  MppiRequest request;
  request.initial_state = State{5.0, 0.0, kPi * 0.5, 0.0, 0.0};
  request.race_line = &track;

  const MppiResult result = backend.compute(request);
  ASSERT_TRUE(result.valid) << result.reason;
  EXPECT_GT(result.control.acceleration, 0.1);
}

TEST(MppiBackend, RejectsOutOfBoundsMeasuredInitialStateInsteadOfClampingIt)
{
  const RaceLine track = makeCircle();
  MppiConfig config = fastConfig();
  VehicleConfig vehicle;
  CpuMppiBackend backend(config, vehicle);
  MppiRequest request;
  request.initial_state = State{5.0, 0.0, kPi * 0.5, vehicle.max_speed + 0.1, 0.0};
  request.race_line = &track;
  const MppiResult excessive_speed = backend.compute(request);
  EXPECT_FALSE(excessive_speed.valid);
  EXPECT_EQ(excessive_speed.reason, "initial_state_out_of_bounds");

  request.initial_state.speed = 0.5;
  request.initial_state.steering = vehicle.max_steering + 0.01;
  const MppiResult excessive_steering = backend.compute(request);
  EXPECT_FALSE(excessive_steering.valid);
  EXPECT_EQ(excessive_steering.reason, "initial_state_out_of_bounds");
}

TEST(MppiBackend, RejectsInvalidExplorationScale)
{
  const RaceLine track = makeCircle();
  CpuMppiBackend backend(fastConfig(), VehicleConfig{});
  MppiRequest request;
  request.initial_state = State{5.0, 0.0, kPi * 0.5, 0.7, 0.0};
  request.race_line = &track;
  request.exploration_scale = 0.5;
  const MppiResult result = backend.compute(request);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reason, "invalid_exploration_scale");
}

TEST(MppiBackend, RejectsNonfiniteOrNegativeConfiguration)
{
  MppiConfig config = fastConfig();
  config.cbf_gamma = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(CpuMppiBackend(config, VehicleConfig{}), std::invalid_argument);
  config = fastConfig();
  config.weights.progress = -1.0;
  EXPECT_THROW(CpuMppiBackend(config, VehicleConfig{}), std::invalid_argument);
  config = fastConfig();
  config.pure_noise_fraction = 1.1;
  EXPECT_THROW(CpuMppiBackend(config, VehicleConfig{}), std::invalid_argument);
  config = fastConfig();
  config.maximum_heading_error = kPi;
  EXPECT_THROW(CpuMppiBackend(config, VehicleConfig{}), std::invalid_argument);

  VehicleConfig vehicle;
  vehicle.max_steering = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(
    {
      const BicycleModel model(vehicle);
      (void)model;
    },
    std::invalid_argument);
}

}  // namespace
}  // namespace mppi_controller
