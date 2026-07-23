#ifndef OPPONENT_TRACKER__SCAN_CLUSTERER_HPP_
#define OPPONENT_TRACKER__SCAN_CLUSTERER_HPP_

#include <cstddef>
#include <vector>

namespace opponent_tracker
{

struct Detection
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  double length{0.0};
  double width{0.0};
  double confidence{0.0};
  std::size_t point_count{0U};
};

struct ScanClustererConfig
{
  double min_range{0.05};
  double max_range{8.0};
  double breakpoint_base{0.06};
  double breakpoint_scale{2.0};
  std::size_t min_cluster_points{3U};
  double min_object_length{0.05};
  double max_object_length{1.0};
  double max_object_width{0.8};
  double min_box_dimension{0.04};
  std::size_t confidence_full_points{20U};
};

class ScanClusterer
{
public:
  explicit ScanClusterer(ScanClustererConfig config);

  std::vector<Detection> detect(
    const std::vector<float> & ranges,
    double angle_min,
    double angle_increment,
    double message_range_min,
    double message_range_max) const;

private:
  ScanClustererConfig config_;
};

}  // namespace opponent_tracker

#endif  // OPPONENT_TRACKER__SCAN_CLUSTERER_HPP_
