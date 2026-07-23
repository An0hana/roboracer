#ifndef OPPONENT_TRACKER__STATIC_MAP_FILTER_HPP_
#define OPPONENT_TRACKER__STATIC_MAP_FILTER_HPP_

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "nav_msgs/msg/occupancy_grid.hpp"

namespace opponent_tracker
{

struct StaticMapFilterConfig
{
  bool enabled{true};
  double radius{0.12};
  int occupied_threshold{50};
  bool unknown_is_occupied{true};
};

class StaticMapFilter
{
public:
  explicit StaticMapFilter(StaticMapFilterConfig config);

  void update(const nav_msgs::msg::OccupancyGrid & map);

  bool ready() const;

  bool supportsFrame(const std::string & frame_id) const;

  bool nearOccupied(double x, double y, const std::string & frame_id) const;

private:
  bool occupied(std::int8_t value) const;

  StaticMapFilterConfig config_;
  std::string frame_id_;
  double resolution_{0.0};
  std::uint32_t width_{0U};
  std::uint32_t height_{0U};
  double origin_x_{0.0};
  double origin_y_{0.0};
  double origin_cosine_{1.0};
  double origin_sine_{0.0};
  std::vector<std::int8_t> data_;
};

}  // namespace opponent_tracker

#endif  // OPPONENT_TRACKER__STATIC_MAP_FILTER_HPP_
