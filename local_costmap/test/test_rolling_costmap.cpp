#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include "local_costmap/rolling_costmap.hpp"

namespace
{

std::int8_t cellAt(
  const local_costmap::CostmapGrid & grid, double world_x, double world_y)
{
  const double dx = world_x - grid.origin_x;
  const double dy = world_y - grid.origin_y;
  const double cosine = std::cos(grid.origin_yaw);
  const double sine = std::sin(grid.origin_yaw);
  const auto x = static_cast<std::size_t>(
    (cosine * dx + sine * dy) / grid.resolution);
  const auto y = static_cast<std::size_t>(
    (-sine * dx + cosine * dy) / grid.resolution);
  return grid.data[y * grid.width + x];
}

TEST(RollingCostmap, MarksHitsAndKeepsShortHistory)
{
  local_costmap::RollingCostmapConfig config;
  config.size_x = 4.0;
  config.size_y = 4.0;
  config.resolution = 0.5;
  config.persistence = 0.20;
  config.forward_offset = 0.0;
  local_costmap::RollingCostmap costmap(config);

  const auto first = costmap.update(0.0, 0.0, 0.0, 1.0, {{1.0, 0.0}});
  EXPECT_EQ(first.width, 8U);
  EXPECT_EQ(first.height, 8U);
  EXPECT_EQ(cellAt(first, 1.0, 0.0), 100);

  const auto retained = costmap.update(0.0, 0.0, 0.0, 1.15, {});
  EXPECT_EQ(cellAt(retained, 1.0, 0.0), 100);
  EXPECT_EQ(costmap.retainedPointCount(), 1U);

  const auto expired = costmap.update(0.0, 0.0, 0.0, 1.21, {});
  EXPECT_EQ(cellAt(expired, 1.0, 0.0), 0);
  EXPECT_EQ(costmap.retainedPointCount(), 0U);
}

TEST(RollingCostmap, ConnectsOnlyAdjacentNearbyLaserHits)
{
  const std::vector<local_costmap::IndexedPoint2d> scan_hits{
    {0U, {0.0, 0.0}},
    {1U, {0.1, 0.0}},
    {2U, {0.5, 0.0}},
    {4U, {0.55, 0.0}},
  };
  const auto connected = local_costmap::connectAdjacentHits(
    scan_hits, 0.20, 0.025);

  ASSERT_EQ(connected.size(), 7U);
  EXPECT_NEAR(connected[1].x, 0.025, 1.0e-12);
  EXPECT_NEAR(connected[2].x, 0.050, 1.0e-12);
  EXPECT_NEAR(connected[3].x, 0.075, 1.0e-12);
  EXPECT_NEAR(connected[4].x, 0.100, 1.0e-12);
  EXPECT_NEAR(connected[5].x, 0.500, 1.0e-12);
  EXPECT_NEAR(connected[6].x, 0.550, 1.0e-12);
}

TEST(RollingCostmap, RecentersWithoutMovingWorldHits)
{
  local_costmap::RollingCostmapConfig config;
  config.size_x = 4.0;
  config.size_y = 4.0;
  config.resolution = 0.5;
  config.persistence = 1.0;
  config.forward_offset = 0.0;
  local_costmap::RollingCostmap costmap(config);

  (void)costmap.update(0.0, 0.0, 0.0, 1.0, {{1.0, 0.0}});
  const auto shifted = costmap.update(0.5, 0.0, 0.0, 1.1, {});
  EXPECT_DOUBLE_EQ(shifted.origin_x, -1.5);
  EXPECT_EQ(cellAt(shifted, 1.0, 0.0), 100);
}

TEST(RollingCostmap, ClearsHistoryWhenTimeMovesBackwards)
{
  local_costmap::RollingCostmap costmap;
  (void)costmap.update(0.0, 0.0, 0.0, 5.0, {{1.0, 0.0}});
  const auto rewound = costmap.update(0.0, 0.0, 0.0, 4.0, {});
  EXPECT_EQ(costmap.retainedPointCount(), 0U);
  EXPECT_EQ(cellAt(rewound, 1.0, 0.0), 0);
}

TEST(RollingCostmap, RejectsInvalidConfigurationAndUpdates)
{
  local_costmap::RollingCostmapConfig config;
  config.resolution = 0.0;
  EXPECT_THROW((void)local_costmap::RollingCostmap(config), std::invalid_argument);

  local_costmap::RollingCostmap costmap;
  EXPECT_THROW(
    costmap.update(
      0.0, 0.0, 0.0, std::numeric_limits<double>::quiet_NaN(), {}),
    std::invalid_argument);
}

TEST(RollingCostmap, ShiftsGridAheadAlongVehicleHeading)
{
  local_costmap::RollingCostmapConfig config;
  config.size_x = 8.0;
  config.size_y = 4.0;
  config.resolution = 0.05;
  config.forward_offset = 2.0;
  config.align_with_vehicle = true;
  local_costmap::RollingCostmap costmap(config);

  const auto facing_x = costmap.update(2.0, 3.0, 0.0, 1.0, {});
  EXPECT_NEAR(facing_x.origin_x, 0.0, 1.0e-9);
  EXPECT_NEAR(facing_x.origin_y, 1.0, 1.0e-9);
  EXPECT_NEAR(facing_x.origin_yaw, 0.0, 1.0e-9);

  const auto facing_y = costmap.update(
    2.0, 3.0, 0.5 * std::acos(-1.0), 1.1, {});
  EXPECT_NEAR(facing_y.origin_x, 4.0, 1.0e-9);
  EXPECT_NEAR(facing_y.origin_y, 1.0, 1.0e-9);
  EXPECT_NEAR(facing_y.origin_yaw, 0.5 * std::acos(-1.0), 1.0e-9);
}

TEST(RollingCostmap, RotatesHitsIntoVehicleAlignedGrid)
{
  local_costmap::RollingCostmapConfig config;
  config.size_x = 14.0;
  config.size_y = 5.0;
  config.resolution = 0.05;
  config.forward_offset = 5.0;
  config.align_with_vehicle = true;
  local_costmap::RollingCostmap costmap(config);

  const double half_pi = 0.5 * std::acos(-1.0);
  const auto grid = costmap.update(
    2.0, 3.0, half_pi, 1.0,
    {
      {2.0, 14.9},  // 11.9 m forward
      {2.0, 1.1},   // 1.9 m behind
      {-0.4, 3.0},  // 2.4 m to the left
    });

  EXPECT_EQ(grid.width, 280U);
  EXPECT_EQ(grid.height, 100U);
  EXPECT_EQ(cellAt(grid, 2.0, 14.9), 100);
  EXPECT_EQ(cellAt(grid, 2.0, 1.1), 100);
  EXPECT_EQ(cellAt(grid, -0.4, 3.0), 100);
}

TEST(RollingCostmap, KeepsDefaultGridMapAlignedWhileVehicleTurns)
{
  local_costmap::RollingCostmapConfig config;
  config.size_x = 22.0;
  config.size_y = 22.0;
  config.resolution = 0.05;
  config.forward_offset = 0.0;
  config.align_with_vehicle = false;
  local_costmap::RollingCostmap costmap(config);

  const auto grid = costmap.update(
    2.0, 3.0, 0.5 * std::acos(-1.0), 1.0, {{2.0, 9.0}});

  EXPECT_EQ(grid.width, 440U);
  EXPECT_EQ(grid.height, 440U);
  EXPECT_NEAR(grid.origin_x, -9.0, 1.0e-9);
  EXPECT_NEAR(grid.origin_y, -8.0, 1.0e-9);
  EXPECT_NEAR(grid.origin_yaw, 0.0, 1.0e-9);
  EXPECT_EQ(cellAt(grid, 2.0, 9.0), 100);
}

}  // namespace
