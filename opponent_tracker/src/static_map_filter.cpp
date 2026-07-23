#include "opponent_tracker/static_map_filter.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace opponent_tracker
{

StaticMapFilter::StaticMapFilter(StaticMapFilterConfig config)
: config_(std::move(config))
{
  if (!std::isfinite(config_.radius) || config_.radius < 0.0 ||
    config_.occupied_threshold < 0 || config_.occupied_threshold > 100)
  {
    throw std::invalid_argument("invalid static map filter configuration");
  }
}

void StaticMapFilter::update(const nav_msgs::msg::OccupancyGrid & map)
{
  const auto expected_size =
    static_cast<std::size_t>(map.info.width) *
    static_cast<std::size_t>(map.info.height);
  if (!std::isfinite(map.info.resolution) || map.info.resolution <= 0.0 ||
    map.info.width == 0U || map.info.height == 0U ||
    map.data.size() != expected_size || map.header.frame_id.empty())
  {
    frame_id_.clear();
    data_.clear();
    return;
  }

  const auto & orientation = map.info.origin.orientation;
  const double yaw = std::atan2(
    2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
    1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z));

  frame_id_ = map.header.frame_id;
  resolution_ = static_cast<double>(map.info.resolution);
  width_ = map.info.width;
  height_ = map.info.height;
  origin_x_ = map.info.origin.position.x;
  origin_y_ = map.info.origin.position.y;
  origin_cosine_ = std::cos(yaw);
  origin_sine_ = std::sin(yaw);
  data_ = map.data;
}

bool StaticMapFilter::ready() const
{
  return !frame_id_.empty() && !data_.empty();
}

bool StaticMapFilter::supportsFrame(const std::string & frame_id) const
{
  return ready() && frame_id == frame_id_;
}

bool StaticMapFilter::nearOccupied(
  double x, double y, const std::string & frame_id) const
{
  if (!config_.enabled || !supportsFrame(frame_id) ||
    !std::isfinite(x) || !std::isfinite(y))
  {
    return false;
  }

  const double dx = x - origin_x_;
  const double dy = y - origin_y_;
  const double map_x = origin_cosine_ * dx + origin_sine_ * dy;
  const double map_y = -origin_sine_ * dx + origin_cosine_ * dy;
  const int center_column = static_cast<int>(std::floor(map_x / resolution_));
  const int center_row = static_cast<int>(std::floor(map_y / resolution_));
  if (center_column < 0 || center_row < 0 ||
    center_column >= static_cast<int>(width_) ||
    center_row >= static_cast<int>(height_))
  {
    return false;
  }

  const int radius_cells = static_cast<int>(std::ceil(config_.radius / resolution_));
  const double cell_padding = std::sqrt(2.0) * 0.5 * resolution_;
  const double maximum_distance = config_.radius + cell_padding;

  const int minimum_column = std::max(0, center_column - radius_cells - 1);
  const int maximum_column = std::min(
    static_cast<int>(width_) - 1, center_column + radius_cells + 1);
  const int minimum_row = std::max(0, center_row - radius_cells - 1);
  const int maximum_row = std::min(
    static_cast<int>(height_) - 1, center_row + radius_cells + 1);

  for (int row = minimum_row; row <= maximum_row; ++row) {
    for (int column = minimum_column; column <= maximum_column; ++column) {
      const auto index =
        static_cast<std::size_t>(row) * static_cast<std::size_t>(width_) +
        static_cast<std::size_t>(column);
      if (!occupied(data_[index])) {
        continue;
      }

      const double cell_x = (static_cast<double>(column) + 0.5) * resolution_;
      const double cell_y = (static_cast<double>(row) + 0.5) * resolution_;
      if (std::hypot(cell_x - map_x, cell_y - map_y) <= maximum_distance) {
        return true;
      }
    }
  }
  return false;
}

bool StaticMapFilter::occupied(std::int8_t value) const
{
  if (value < 0) {
    return config_.unknown_is_occupied;
  }
  return static_cast<int>(value) >= config_.occupied_threshold;
}

}  // namespace opponent_tracker
