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

TEST(BicycleModel, SeparatesServoTargetFromEffectiveSteeringResponse)
{
  VehicleConfig config;
  config.steering_response_time = 0.15;
  config.min_effective_steering_rate = -1.20;
  config.max_effective_steering_rate = 1.20;
  config.effective_steering_rate_speed_coefficient = 0.18;
  config.steering_effectiveness_speed_squared = 0.05;
  BicycleModel model(config);

  const State initial{0.0, 0.0, 0.0, 2.0, 0.0, 0.0};
  const State first = model.step(
    initial, Control{config.max_steering_rate, 0.0}, 0.05);
  EXPECT_NEAR(first.steering_command, 0.075, 1e-12);
  EXPECT_GT(first.steering, 0.0);
  EXPECT_LT(first.steering, first.steering_command);

  State settled{0.0, 0.0, 0.0, 2.0, 0.0, 0.20};
  for (int step = 0; step < 100; ++step) {
    settled = model.step(settled, Control{}, 0.02);
  }
  EXPECT_NEAR(
    settled.steering,
    effectiveSteeringTarget(config, 0.20, 2.0), 1e-5);
  EXPECT_NEAR(settled.steering_command, 0.20, 1e-12);
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

TEST(DistanceField, VehicleFootprintClearanceCoversTheWholeBody)
{
  constexpr std::size_t width = 80U;
  constexpr std::size_t height = 80U;
  constexpr double resolution = 0.05;
  constexpr double origin = -2.0;
  std::vector<std::int8_t> grid(width * height, 0);
  const auto obstacle_x = static_cast<std::size_t>((0.25 - origin) / resolution);
  const auto obstacle_y = static_cast<std::size_t>((0.0 - origin) / resolution);
  grid[obstacle_y * width + obstacle_x] = 100;
  const DistanceField field = DistanceField::fromOccupancyGrid(
    width, height, resolution, origin, origin, grid, 50, false);
  const VehicleConfig vehicle;

  EXPECT_LT(vehicleFootprintClearance(State{}, vehicle, &field), 0.0);
  EXPECT_GT(
    vehicleFootprintClearance(State{1.0, 0.0, 0.0, 0.0, 0.0}, vehicle, &field),
    0.20);
  EXPECT_TRUE(std::isinf(vehicleFootprintClearance(State{}, vehicle, nullptr)));
}

TEST(Recovery, AllowsRecordedGridQuantizationWhileRequiringAClearEndpoint)
{
  // 0822_A3_01 started 8.3-12.4 cm inside the inflated envelope. The
  // clearance generally improved while reversing, with one harmless 17.5 mm
  // regression at a 5 cm grid-cell boundary, and ended well outside it.
  const std::vector<double> recorded_like{
    -0.124, -0.101, -0.0835, -0.101, -0.060, -0.010, 0.040, 0.176};
  EXPECT_TRUE(reverseRecoveryClearanceSafe(recorded_like, 0.13, 0.025));

  // These are the two independent causes of all recorded rejections.
  EXPECT_FALSE(reverseRecoveryClearanceSafe(recorded_like, 0.08, 0.025));
  EXPECT_FALSE(reverseRecoveryClearanceSafe(recorded_like, 0.13, 0.005));
}

TEST(Recovery, StillRejectsBlockedOrNonEscapingReversePaths)
{
  EXPECT_FALSE(reverseRecoveryClearanceSafe({}, 0.13, 0.025));
  EXPECT_FALSE(reverseRecoveryClearanceSafe(
      {-0.10, -0.12, -0.15, -0.18}, 0.13, 0.025));
  EXPECT_FALSE(reverseRecoveryClearanceSafe(
      {-0.10, -0.05, -0.01}, 0.13, 0.025));
  EXPECT_FALSE(reverseRecoveryClearanceSafe(
      {-0.10, 0.02, -0.01, 0.10}, 0.13, 0.025));
  EXPECT_FALSE(reverseRecoveryClearanceSafe(
      {-0.10, std::numeric_limits<double>::quiet_NaN(), 0.10},
      0.13, 0.025));
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

TEST(Cost, TreatsRepairClearanceAsSoftBufferBeyondHardSafetyMargin)
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
  EXPECT_EQ(guarded_cost.collision, 0.0);
  EXPECT_GT(guarded_cost.cbf, permissive_cost.cbf);

  std::vector<Control> recovery_controls(guarded.horizon_steps, Control{});
  const auto deadline =
    std::chrono::steady_clock::now() + std::chrono::milliseconds(50);
  EXPECT_TRUE(guarded_backend.repairControls(
      state, recovery_controls, track, &field, deadline));
}

TEST(Cost, MovingClearanceRecoveryMustDecelerateAndRestoreHardMargin)
{
  const RaceLine track = makeCircle(false, 2.0);
  MppiConfig config = fastConfig();
  config.horizon_steps = 24U;
  config.dt = 0.05;
  config.initial_clearance_tolerance = 0.015;
  config.clearance_recovery_steps = 12U;
  config.clearance_recovery_speed_threshold = 1.50;
  config.clearance_recovery_acceleration_speed_threshold = 0.10;
  const VehicleConfig vehicle;
  CpuMppiBackend backend(config, vehicle);

  const double vehicle_segment_length = vehicle.length /
    std::ceil(vehicle.length / (vehicle.width * 0.5));
  const double vehicle_cover_radius = std::hypot(
    vehicle.width * 0.5, vehicle_segment_length * 0.5);
  Obstacle obstacle;
  obstacle.half_width = 0.02;
  obstacle.x =
    5.0 + vehicle_cover_radius + obstacle.half_width +
    vehicle.safety_margin - 0.010;
  obstacle.y = -vehicle.rear_overhang + vehicle_segment_length * 0.5;
  const std::vector<Obstacle> obstacles{obstacle};
  const State initial{5.0, 0.0, kPi * 0.5, 0.0, 0.0};
  EXPECT_NEAR(
    obstacleClearance(initial, vehicle, &obstacles, 0.0) -
    vehicle.safety_margin, -0.010, 1.0e-9);

  // Remaining stationary inside the envelope is not accepted.
  const std::vector<Control> stationary(config.horizon_steps, Control{});
  const CostBreakdown stationary_cost = backend.evaluateTrajectory(
    initial, stationary, track, nullptr, nullptr, &obstacles);
  EXPECT_GT(stationary_cost.collision, 0.0);

  // A bounded launch that monotonically separates from the obstacle and
  // restores the full margin within the configured window remains feasible.
  const std::vector<Control> departing(
    config.horizon_steps, Control{0.0, vehicle.max_acceleration});
  const CostBreakdown departing_cost = backend.evaluateTrajectory(
    initial, departing, track, nullptr, nullptr, &obstacles);
  EXPECT_EQ(departing_cost.collision, 0.0);

  // Once moving, an otherwise geometrically valid recovery may not accelerate.
  State moving = initial;
  moving.speed = 0.80;
  const CostBreakdown accelerating_moving_cost = backend.evaluateTrajectory(
    moving, departing, track, nullptr, nullptr, &obstacles);
  EXPECT_GT(accelerating_moving_cost.collision, 0.0);

  // A deterministic braking sequence is accepted only when its predicted
  // footprint monotonically leaves the soft-envelope violation.
  std::vector<Control> braking(
    config.horizon_steps, Control{0.0, vehicle.min_acceleration});
  const CostBreakdown braking_cost = backend.evaluateTrajectory(
    moving, braking, track, nullptr, nullptr, &obstacles);
  EXPECT_EQ(braking_cost.collision, 0.0);

  // The repair stage strips positive acceleration from moving recovery steps.
  std::vector<Control> repaired(
    config.horizon_steps, Control{0.0, vehicle.max_acceleration});
  const auto deadline =
    std::chrono::steady_clock::now() + std::chrono::milliseconds(50);
  EXPECT_TRUE(backend.repairControls(
      moving, repaired, track, nullptr, deadline, &obstacles));
  EXPECT_LE(repaired.front().acceleration, 0.0);
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

TEST(BicycleModel, ClampsOnlySmallMeasuredSpeedBoundaryOvershoot)
{
  VehicleConfig vehicle;
  State state{1.0, 2.0, 0.2, vehicle.max_speed + 0.24, 0.1};
  EXPECT_TRUE(clampMeasuredSpeedWithinTolerance(state, vehicle, 0.25));
  EXPECT_DOUBLE_EQ(state.speed, vehicle.max_speed);

  state.speed = vehicle.min_speed - 0.10;
  EXPECT_TRUE(clampMeasuredSpeedWithinTolerance(state, vehicle, 0.25));
  EXPECT_DOUBLE_EQ(state.speed, vehicle.min_speed);

  state.speed = vehicle.max_speed + 0.26;
  EXPECT_FALSE(clampMeasuredSpeedWithinTolerance(state, vehicle, 0.25));

  state.speed = vehicle.max_speed;
  state.steering = vehicle.max_steering + 0.01;
  EXPECT_FALSE(clampMeasuredSpeedWithinTolerance(state, vehicle, 0.25));
}

TEST(BicycleModel, RecordedSpeedOvershootCanBePropagatedAfterToleranceClamp)
{
  VehicleConfig vehicle;
  vehicle.max_speed = 1.50;
  const BicycleModel model(vehicle);
  State estimator_state{0.0, 0.0, 0.0, 1.537479, 0.02, 0.05};

  ASSERT_TRUE(clampMeasuredSpeedWithinTolerance(estimator_state, vehicle, 0.50));
  EXPECT_DOUBLE_EQ(estimator_state.speed, 1.50);
  EXPECT_NO_THROW({
    const State propagated =
      propagateState(model, estimator_state, Control{}, 0.025, 0.01);
    EXPECT_TRUE(stateWithinLimits(propagated, vehicle));
  });
}

TEST(OverspeedRecovery, UsesHysteresisAndRateLimitedVelocityReduction)
{
  OverspeedRecoveryConfig config;
  config.max_command_speed = 1.50;
  config.entry_margin = 0.10;
  config.exit_margin = 0.03;
  config.exit_hold_time = 0.20;
  config.command_reduction = 0.10;
  config.proportional_gain = 0.50;
  config.command_deceleration = 1.50;
  OverspeedRecovery recovery(config);

  const OverspeedRecoveryResult normal = recovery.update(1.55, 1.50, 0.05);
  EXPECT_FALSE(normal.active);
  EXPECT_TRUE(std::isinf(normal.command_limit));

  const OverspeedRecoveryResult entered = recovery.update(1.77, 1.50, 0.05);
  ASSERT_TRUE(entered.active);
  EXPECT_NEAR(entered.measured_excess, 0.27, 1.0e-12);
  EXPECT_NEAR(entered.command_limit, 1.425, 1.0e-12);

  const OverspeedRecoveryResult middle = recovery.update(1.56, 1.425, 0.05);
  EXPECT_TRUE(middle.active);
  EXPECT_DOUBLE_EQ(middle.clear_duration, 0.0);

  EXPECT_TRUE(recovery.update(1.52, 1.35, 0.10).active);
  const OverspeedRecoveryResult exited = recovery.update(1.51, 1.30, 0.10);
  EXPECT_FALSE(exited.active);
  EXPECT_TRUE(std::isinf(exited.command_limit));
}

TEST(OverspeedRecovery, InvalidMeasurementResetsRecovery)
{
  OverspeedRecoveryConfig config;
  config.max_command_speed = 1.50;
  OverspeedRecovery recovery(config);
  ASSERT_TRUE(recovery.update(1.70, 1.50, 0.05).active);
  EXPECT_FALSE(
    recovery.update(std::numeric_limits<double>::quiet_NaN(), 1.40, 0.05).active);
  EXPECT_FALSE(recovery.update(1.50, 1.40, 0.05).active);
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

// On the radius-5 circle the raceline starts at (5, 0) heading +y. Arc
// distance d ahead corresponds to angle d / 5.
Obstacle circleObstacle(double arc_ahead, double radial_offset)
{
  const double theta = arc_ahead / 5.0;
  const double radius = 5.0 + radial_offset;
  Obstacle obstacle;
  obstacle.x = radius * std::cos(theta);
  obstacle.y = radius * std::sin(theta);
  obstacle.yaw = normalizeAngle(theta + kPi * 0.5);
  obstacle.half_length = 0.15;
  obstacle.half_width = 0.10;
  return obstacle;
}

TEST(Obstacle, ClearanceIsConservativeAndExtrapolatesMotion)
{
  const VehicleConfig vehicle;
  const State state{0.0, 0.0, 0.0, 1.0, 0.0};
  EXPECT_TRUE(std::isinf(obstacleClearance(state, vehicle, nullptr, 0.0)));
  const std::vector<Obstacle> none;
  EXPECT_TRUE(std::isinf(obstacleClearance(state, vehicle, &none, 0.0)));

  Obstacle ahead;
  ahead.x = 2.0;
  ahead.half_length = 0.2;
  ahead.half_width = 0.1;
  const std::vector<Obstacle> static_obstacle{ahead};
  // Exact rectangle gap: vehicle front 0.428, obstacle rear 1.8.
  const double exact_gap = 1.372;
  const double reported = obstacleClearance(state, vehicle, &static_obstacle, 0.0);
  EXPECT_LE(reported, exact_gap);
  EXPECT_GE(reported, exact_gap - 0.25);

  Obstacle approaching = ahead;
  approaching.vx = -1.0;
  const std::vector<Obstacle> moving{approaching};
  Obstacle shifted = ahead;
  shifted.x = 1.5;
  const std::vector<Obstacle> equivalent{shifted};
  EXPECT_NEAR(
    obstacleClearance(state, vehicle, &moving, 0.5),
    obstacleClearance(state, vehicle, &equivalent, 0.0), 1.0e-12);
}

TEST(Cost, DistinguishesDepartingFromMergingObstacle)
{
  const RaceLine track = makeCircle(false, 2.0);
  MppiConfig config = fastConfig();
  config.horizon_steps = 20U;
  CpuMppiBackend backend(config, VehicleConfig{});
  const State initial{5.0, 0.0, kPi * 0.5, 1.2, std::atan(0.324 / 5.0)};
  const std::vector<Control> follow(config.horizon_steps, Control{});
  const double arrival_time = 1.0 / 1.2;

  // Departing: currently dead ahead on the path, but leaving it fast. A
  // static treatment would flag this as a collision.
  Obstacle departing = circleObstacle(1.0, 0.0);
  departing.vx = 2.0 * std::cos(1.0 / 5.0);
  departing.vy = 2.0 * std::sin(1.0 / 5.0);
  const std::vector<Obstacle> departing_set{departing};
  const CostBreakdown departing_cost = backend.evaluateTrajectory(
    initial, follow, track, nullptr, nullptr, &departing_set);
  EXPECT_EQ(departing_cost.collision, 0.0);

  // Merging: currently clear of the path, but timed to reach the vehicle's
  // arrival point exactly when the vehicle does. A static treatment would
  // call this safe.
  const double offset = 0.8;
  Obstacle merging = circleObstacle(1.0, offset);
  merging.vx = -(offset / arrival_time) * std::cos(1.0 / 5.0);
  merging.vy = -(offset / arrival_time) * std::sin(1.0 / 5.0);
  const std::vector<Obstacle> merging_set{merging};
  const CostBreakdown merging_cost = backend.evaluateTrajectory(
    initial, follow, track, nullptr, nullptr, &merging_set);
  EXPECT_GT(merging_cost.collision, 0.0);

  Obstacle merging_static = merging;
  merging_static.vx = 0.0;
  merging_static.vy = 0.0;
  const std::vector<Obstacle> static_set{merging_static};
  const CostBreakdown static_cost = backend.evaluateTrajectory(
    initial, follow, track, nullptr, nullptr, &static_set);
  EXPECT_EQ(static_cost.collision, 0.0);
}

TEST(Cost, ObstacleMeasurementAgeShiftsExtrapolation)
{
  const RaceLine track = makeCircle(false, 2.0);
  MppiConfig config = fastConfig();
  config.horizon_steps = 20U;
  CpuMppiBackend backend(config, VehicleConfig{});
  const State initial{5.0, 0.0, kPi * 0.5, 1.2, std::atan(0.324 / 5.0)};
  const std::vector<Control> follow(config.horizon_steps, Control{});

  Obstacle stale = circleObstacle(1.0, 0.6);
  stale.vx = -1.0 * std::cos(1.0 / 5.0);
  stale.vy = -1.0 * std::sin(1.0 / 5.0);
  stale.time_offset = 0.1;
  const std::vector<Obstacle> stale_set{stale};

  Obstacle compensated = stale;
  compensated.x += stale.vx * 0.1;
  compensated.y += stale.vy * 0.1;
  compensated.time_offset = 0.0;
  const std::vector<Obstacle> compensated_set{compensated};

  const CostBreakdown stale_cost = backend.evaluateTrajectory(
    initial, follow, track, nullptr, nullptr, &stale_set);
  const CostBreakdown compensated_cost = backend.evaluateTrajectory(
    initial, follow, track, nullptr, nullptr, &compensated_set);
  EXPECT_NEAR(stale_cost.total(), compensated_cost.total(), 1.0e-9);
  EXPECT_EQ(stale_cost.collision, compensated_cost.collision);
}

TEST(CpuMppi, SwervesAroundStaticObstacleOnRaceLine)
{
  const RaceLine track = makeCircle(false, 2.0);
  MppiConfig config = fastConfig();
  config.horizon_steps = 30U;
  config.random_seed = 42U;
  CpuMppiBackend backend(config, VehicleConfig{});

  // 1.8 m ahead keeps the deterministic braking rollouts feasible (stopping
  // needs ~1.15 m including body length and cover slack), so the sampler
  // always has a valid fallback while it discovers the swerve. Slightly
  // off-center so left/right avoidance rollouts cannot cancel each other out
  // in the importance-weighted average.
  const std::vector<Obstacle> obstacles{circleObstacle(1.8, 0.15)};
  MppiRequest request;
  request.initial_state = State{5.0, 0.0, kPi * 0.5, 1.2, std::atan(0.324 / 5.0)};
  request.race_line = &track;
  request.obstacles = &obstacles;

  // Receding-horizon warm starts from a fixed state; the sampler must
  // converge onto an avoiding sequence within a few iterations.
  MppiResult result;
  for (int iteration = 0; iteration < 5; ++iteration) {
    result = backend.compute(request);
    if (result.valid) {
      break;
    }
  }
  ASSERT_TRUE(result.valid) << result.reason;
  EXPECT_EQ(result.cost.collision, 0.0);
  EXPECT_GT(result.metrics.minimum_obstacle_clearance, 0.0);
  // The solution must keep racing, not just slam the brakes.
  EXPECT_GT(result.metrics.forward_progress, 0.3);

  // The same scenario without the obstacle must not report obstacle
  // clearance, proving the metric actually came from the obstacle input.
  backend.reset();
  MppiRequest empty_request = request;
  empty_request.obstacles = nullptr;
  const MppiResult unobstructed = backend.compute(empty_request);
  ASSERT_TRUE(unobstructed.valid) << unobstructed.reason;
  EXPECT_TRUE(std::isinf(unobstructed.metrics.minimum_obstacle_clearance));
}

TEST(CudaMppi, AvoidsObstaclesWithCpuValidatedSafety)
{
  if (!cudaBackendCompiled()) {
    GTEST_SKIP() << "CUDA backend not compiled";
  }
  if (!cudaBackendDeviceCompatible()) {
    // The vendor kernels abort the whole process on an architecture-mismatched
    // launch, so this must be a skip, not a runtime failure path.
    GTEST_SKIP() << "no CUDA device matching the compiled architecture";
  }
  MppiConfig config;  // Defaults match the compiled 2048 x 72 CUDA shape.
  config.random_seed = 42U;
  std::unique_ptr<MppiBackend> backend;
  try {
    backend = makeBackend("cuda", config, VehicleConfig{});
  } catch (const std::exception & exception) {
    GTEST_SKIP() << "CUDA backend unavailable: " << exception.what();
  }
  const RaceLine track = makeCircle(false, 2.0);
  if (!backend->warmup(track, nullptr)) {
    GTEST_SKIP() << "no usable CUDA device for warmup";
  }
  backend->reset();

  const std::vector<Obstacle> obstacles{circleObstacle(1.8, 0.15)};
  MppiRequest request;
  request.initial_state = State{5.0, 0.0, kPi * 0.5, 1.2, std::atan(0.324 / 5.0)};
  request.race_line = &track;
  request.obstacles = &obstacles;

  MppiResult result;
  for (int iteration = 0; iteration < 8; ++iteration) {
    result = backend->compute(request);
    if (result.valid && result.reason == "ok") {
      break;
    }
  }
  // Cost and metrics of the returned trajectory come from the CPU validator,
  // so these assertions certify the CUDA proposal against the reference
  // safety model — the cross-backend consistency check.
  ASSERT_TRUE(result.valid) << result.reason;
  EXPECT_EQ(result.cost.collision, 0.0);
  EXPECT_GT(result.metrics.minimum_obstacle_clearance, 0.0);
  // A GPU cost that ignored obstacles would fail CPU validation into the
  // braking fallback: near-zero progress and a fallback reason.
  EXPECT_EQ(result.reason, "ok");
  EXPECT_GT(result.metrics.forward_progress, 0.3);
  RecordProperty("solve_time_ms", result.solve_time_ms);
  EXPECT_LT(result.solve_time_ms, 25.0);
}

TEST(MppiBehavior, ScalesSpeedAndOffsetsRaceLineWithoutAnotherWeightSet)
{
  MppiConfig config = fastConfig();
  VehicleConfig vehicle;
  CpuMppiBackend backend(config, vehicle);
  const RaceLine track = makeCircle(false, 2.0);
  const State initial{
    5.0, 0.0, kPi * 0.5, 1.2, std::atan(vehicle.wheelbase / 5.0)};
  const std::vector<Control> controls(8U);

  MppiBehavior neutral;
  const CostBreakdown neutral_cost = backend.evaluateTrajectory(
    initial, controls, track, nullptr, nullptr, nullptr, &neutral);

  MppiBehavior slow = neutral;
  slow.speed_scale = 0.5;
  const CostBreakdown slow_cost = backend.evaluateTrajectory(
    initial, controls, track, nullptr, nullptr, nullptr, &slow);
  EXPECT_GT(slow_cost.speed, neutral_cost.speed);

  MppiBehavior overtake = neutral;
  overtake.lateral_reference_offset = 0.45;
  const CostBreakdown overtake_cost = backend.evaluateTrajectory(
    initial, controls, track, nullptr, nullptr, nullptr, &overtake);
  EXPECT_GT(overtake_cost.lateral, neutral_cost.lateral);

  MppiBehavior deformed = overtake;
  deformed.raceline_weight_scale = 0.25;
  const CostBreakdown deformed_cost = backend.evaluateTrajectory(
    initial, controls, track, nullptr, nullptr, nullptr, &deformed);
  EXPECT_NEAR(
    deformed_cost.lateral, 0.25 * overtake_cost.lateral, 1.0e-9);
}

}  // namespace
}  // namespace mppi_controller
