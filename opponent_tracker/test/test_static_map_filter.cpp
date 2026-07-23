#include <cmath>
#include <stdexcept>
#include <string>

#include "gtest/gtest.h"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "opponent_tracker/static_map_filter.hpp"

namespace opponent_tracker
{
namespace
{

nav_msgs::msg::OccupancyGrid makeMap()
{
  nav_msgs::msg::OccupancyGrid map;
  map.header.frame_id = "map";
  map.info.resolution = 0.1F;
  map.info.width = 10U;
  map.info.height = 10U;
  map.info.origin.orientation.w = 1.0;
  map.data.assign(100U, 0);
  map.data[5U * map.info.width + 4U] = 100;
  return map;
}

TEST(StaticMapFilter, RejectsInvalidConfiguration)
{
  StaticMapFilterConfig config;
  config.radius = -0.1;
  EXPECT_THROW(StaticMapFilter filter(config), std::invalid_argument);
}

TEST(StaticMapFilter, RejectsInvalidMap)
{
  StaticMapFilter filter(StaticMapFilterConfig{});
  auto map = makeMap();
  map.data.pop_back();
  filter.update(map);
  EXPECT_FALSE(filter.ready());
}

TEST(StaticMapFilter, DetectsOccupiedAndInflatedCells)
{
  StaticMapFilter filter(StaticMapFilterConfig{});
  filter.update(makeMap());

  EXPECT_TRUE(filter.ready());
  EXPECT_TRUE(filter.supportsFrame("map"));
  EXPECT_FALSE(filter.supportsFrame("odom"));
  EXPECT_TRUE(filter.nearOccupied(0.45, 0.55, "map"));
  EXPECT_TRUE(filter.nearOccupied(0.60, 0.55, "map"));
  EXPECT_FALSE(filter.nearOccupied(0.90, 0.90, "map"));
  EXPECT_FALSE(filter.nearOccupied(0.45, 0.55, "odom"));
}

TEST(StaticMapFilter, SupportsRotatedMapOrigin)
{
  StaticMapFilter filter(StaticMapFilterConfig{});
  auto map = makeMap();
  map.info.origin.position.x = 1.0;
  map.info.origin.position.y = 2.0;
  constexpr double kQuarterTurn = 1.5707963267948966;
  map.info.origin.orientation.z = std::sin(kQuarterTurn * 0.5);
  map.info.origin.orientation.w = std::cos(kQuarterTurn * 0.5);
  filter.update(map);

  // Occupied cell center is (0.45, 0.55) in map-local coordinates.
  EXPECT_TRUE(filter.nearOccupied(0.45, 2.45, "map"));
}

TEST(StaticMapFilter, CanTreatUnknownCellsAsOccupied)
{
  auto map = makeMap();
  map.data[1U * map.info.width + 1U] = -1;

  StaticMapFilterConfig free_unknown_config;
  free_unknown_config.radius = 0.0;
  free_unknown_config.unknown_is_occupied = false;
  StaticMapFilter free_unknown_filter(free_unknown_config);
  free_unknown_filter.update(map);
  EXPECT_FALSE(free_unknown_filter.nearOccupied(0.15, 0.15, "map"));

  StaticMapFilterConfig occupied_unknown_config;
  occupied_unknown_config.radius = 0.0;
  occupied_unknown_config.unknown_is_occupied = true;
  StaticMapFilter occupied_unknown_filter(occupied_unknown_config);
  occupied_unknown_filter.update(map);
  EXPECT_TRUE(occupied_unknown_filter.nearOccupied(0.15, 0.15, "map"));
}

}  // namespace
}  // namespace opponent_tracker
