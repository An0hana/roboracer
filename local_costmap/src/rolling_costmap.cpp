#include "local_costmap/rolling_costmap.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace local_costmap
{

namespace
{

bool positiveFinite(double value)
{
  return std::isfinite(value) && value > 0.0;
}

std::size_t cellCount(double size, double resolution)
{
  return static_cast<std::size_t>(std::ceil(size / resolution));
}

}  // namespace

std::vector<Point2d> connectAdjacentHits(
  const std::vector<IndexedPoint2d> & ordered_hits,
  double maximum_gap, double sample_spacing)
{
  if (!positiveFinite(maximum_gap) || !positiveFinite(sample_spacing) ||
    sample_spacing > maximum_gap)
  {
    throw std::invalid_argument("invalid scan wall-connection configuration");
  }
  std::vector<Point2d> connected;
  if (ordered_hits.empty()) {
    return connected;
  }
  connected.reserve(ordered_hits.size());
  for (std::size_t index = 0U; index < ordered_hits.size(); ++index) {
    const IndexedPoint2d & current = ordered_hits[index];
    if (!std::isfinite(current.point.x) || !std::isfinite(current.point.y)) {
      continue;
    }
    if (!connected.empty() && index > 0U) {
      const IndexedPoint2d & previous = ordered_hits[index - 1U];
      const double dx = current.point.x - previous.point.x;
      const double dy = current.point.y - previous.point.y;
      const double distance = std::hypot(dx, dy);
      if (current.beam_index == previous.beam_index + 1U &&
        std::isfinite(previous.point.x) && std::isfinite(previous.point.y) &&
        distance <= maximum_gap)
      {
        const std::size_t segments = std::max<std::size_t>(
          1U, static_cast<std::size_t>(std::ceil(distance / sample_spacing)));
        for (std::size_t segment = 1U; segment < segments; ++segment) {
          const double ratio =
            static_cast<double>(segment) / static_cast<double>(segments);
          connected.push_back(Point2d{
            previous.point.x + ratio * dx,
            previous.point.y + ratio * dy});
        }
      }
    }
    connected.push_back(current.point);
  }
  return connected;
}

RollingCostmap::RollingCostmap(RollingCostmapConfig config)
: config_(config)
{
  if (!positiveFinite(config_.size_x) || !positiveFinite(config_.size_y) ||
    !positiveFinite(config_.resolution) || !std::isfinite(config_.persistence) ||
    config_.persistence < 0.0 || !std::isfinite(config_.forward_offset) ||
    config_.forward_offset < 0.0 ||
    config_.forward_offset >= 0.5 * config_.size_x ||
    config_.occupied_value <= 0)
  {
    throw std::invalid_argument("invalid rolling costmap configuration");
  }
  const std::size_t width = cellCount(config_.size_x, config_.resolution);
  const std::size_t height = cellCount(config_.size_y, config_.resolution);
  if (width == 0U || height == 0U ||
    width > std::numeric_limits<std::size_t>::max() / height)
  {
    throw std::invalid_argument("rolling costmap dimensions are invalid");
  }
}

CostmapGrid RollingCostmap::update(
  double base_x, double base_y, double base_yaw, double stamp_seconds,
  const std::vector<Point2d> & hits)
{
  if (!std::isfinite(base_x) || !std::isfinite(base_y) ||
    !std::isfinite(base_yaw) ||
    !std::isfinite(stamp_seconds))
  {
    throw std::invalid_argument("rolling costmap update contains non-finite values");
  }

  if (have_stamp_ && stamp_seconds + 1.0e-9 < last_stamp_seconds_) {
    retained_points_.clear();
  }
  last_stamp_seconds_ = stamp_seconds;
  have_stamp_ = true;

  for (const Point2d & hit : hits) {
    if (std::isfinite(hit.x) && std::isfinite(hit.y)) {
      retained_points_.push_back(TimedPoint{hit, stamp_seconds});
    }
  }

  const double oldest_allowed = stamp_seconds - config_.persistence;
  while (!retained_points_.empty() &&
    retained_points_.front().stamp_seconds + 1.0e-9 < oldest_allowed)
  {
    retained_points_.pop_front();
  }

  CostmapGrid grid;
  grid.width = cellCount(config_.size_x, config_.resolution);
  grid.height = cellCount(config_.size_y, config_.resolution);
  grid.resolution = config_.resolution;
  const double center_x = base_x + config_.forward_offset * std::cos(base_yaw);
  const double center_y = base_y + config_.forward_offset * std::sin(base_yaw);
  const double grid_yaw = config_.align_with_vehicle ? base_yaw : 0.0;
  const double cosine = std::cos(grid_yaw);
  const double sine = std::sin(grid_yaw);
  const double half_size_x = 0.5 * static_cast<double>(grid.width) * grid.resolution;
  const double half_size_y = 0.5 * static_cast<double>(grid.height) * grid.resolution;
  grid.origin_x = center_x - cosine * half_size_x + sine * half_size_y;
  grid.origin_y = center_y - sine * half_size_x - cosine * half_size_y;
  grid.origin_yaw = grid_yaw;
  grid.data.assign(grid.width * grid.height, 0);

  for (const TimedPoint & timed : retained_points_) {
    const double dx = timed.point.x - grid.origin_x;
    const double dy = timed.point.y - grid.origin_y;
    const double local_x = (cosine * dx + sine * dy) / grid.resolution;
    const double local_y = (-sine * dx + cosine * dy) / grid.resolution;
    if (local_x < 0.0 || local_y < 0.0 ||
      local_x >= static_cast<double>(grid.width) ||
      local_y >= static_cast<double>(grid.height))
    {
      continue;
    }
    const auto cell_x = static_cast<std::size_t>(std::floor(local_x));
    const auto cell_y = static_cast<std::size_t>(std::floor(local_y));
    grid.data[cell_y * grid.width + cell_x] = config_.occupied_value;
  }
  return grid;
}

void RollingCostmap::reset() noexcept
{
  retained_points_.clear();
  last_stamp_seconds_ = 0.0;
  have_stamp_ = false;
}

const RollingCostmapConfig & RollingCostmap::config() const noexcept
{
  return config_;
}

std::size_t RollingCostmap::retainedPointCount() const noexcept
{
  return retained_points_.size();
}

}  // namespace local_costmap
