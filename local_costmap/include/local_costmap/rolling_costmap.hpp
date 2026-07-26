#ifndef LOCAL_COSTMAP__ROLLING_COSTMAP_HPP_
#define LOCAL_COSTMAP__ROLLING_COSTMAP_HPP_

#include <cstddef>
#include <cstdint>
#include <deque>
#include <vector>

namespace local_costmap
{

struct Point2d
{
  double x{0.0};
  double y{0.0};
};

struct IndexedPoint2d
{
  std::size_t beam_index{0U};
  Point2d point{};
};

[[nodiscard]] std::vector<Point2d> connectAdjacentHits(
  const std::vector<IndexedPoint2d> & ordered_hits,
  double maximum_gap, double sample_spacing);

struct RollingCostmapConfig
{
  double size_x{22.0};
  double size_y{22.0};
  double resolution{0.05};
  double persistence{0.20};
  double forward_offset{0.0};
  bool align_with_vehicle{false};
  std::int8_t occupied_value{100};
};

struct CostmapGrid
{
  std::size_t width{0U};
  std::size_t height{0U};
  double resolution{0.0};
  double origin_x{0.0};
  double origin_y{0.0};
  double origin_yaw{0.0};
  std::vector<std::int8_t> data;
};

class RollingCostmap
{
public:
  explicit RollingCostmap(RollingCostmapConfig config = {});

  [[nodiscard]] CostmapGrid update(
    double base_x, double base_y, double base_yaw, double stamp_seconds,
    const std::vector<Point2d> & hits);
  void reset() noexcept;

  [[nodiscard]] const RollingCostmapConfig & config() const noexcept;
  [[nodiscard]] std::size_t retainedPointCount() const noexcept;

private:
  struct TimedPoint
  {
    Point2d point;
    double stamp_seconds{0.0};
  };

  RollingCostmapConfig config_;
  std::deque<TimedPoint> retained_points_;
  double last_stamp_seconds_{0.0};
  bool have_stamp_{false};
};

}  // namespace local_costmap

#endif  // LOCAL_COSTMAP__ROLLING_COSTMAP_HPP_
