#include "opponent_tracker/scan_clusterer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

namespace opponent_tracker
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

struct Sample
{
  double x;
  double y;
  double range;
};

double normalizeAngle(double angle)
{
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

bool finiteAndPositive(double value)
{
  return std::isfinite(value) && value > 0.0;
}

bool buildDetection(
  const std::vector<Sample> & cluster,
  const ScanClustererConfig & config,
  Detection & detection)
{
  if (cluster.size() < config.min_cluster_points) {
    return false;
  }

  double mean_x = 0.0;
  double mean_y = 0.0;
  for (const auto & sample : cluster) {
    mean_x += sample.x;
    mean_y += sample.y;
  }
  const double count = static_cast<double>(cluster.size());
  mean_x /= count;
  mean_y /= count;

  double covariance_xx = 0.0;
  double covariance_xy = 0.0;
  double covariance_yy = 0.0;
  for (const auto & sample : cluster) {
    const double dx = sample.x - mean_x;
    const double dy = sample.y - mean_y;
    covariance_xx += dx * dx;
    covariance_xy += dx * dy;
    covariance_yy += dy * dy;
  }

  const double yaw = 0.5 * std::atan2(
    2.0 * covariance_xy, covariance_xx - covariance_yy);
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);

  double min_u = std::numeric_limits<double>::infinity();
  double max_u = -std::numeric_limits<double>::infinity();
  double min_v = std::numeric_limits<double>::infinity();
  double max_v = -std::numeric_limits<double>::infinity();

  for (const auto & sample : cluster) {
    const double dx = sample.x - mean_x;
    const double dy = sample.y - mean_y;
    const double u = cos_yaw * dx + sin_yaw * dy;
    const double v = -sin_yaw * dx + cos_yaw * dy;
    min_u = std::min(min_u, u);
    max_u = std::max(max_u, u);
    min_v = std::min(min_v, v);
    max_v = std::max(max_v, v);
  }

  const double center_u = 0.5 * (min_u + max_u);
  const double center_v = 0.5 * (min_v + max_v);
  const double center_x =
    mean_x + cos_yaw * center_u - sin_yaw * center_v;
  const double center_y =
    mean_y + sin_yaw * center_u + cos_yaw * center_v;

  double box_yaw = yaw;
  double raw_length = max_u - min_u;
  double raw_width = max_v - min_v;
  if (raw_width > raw_length) {
    std::swap(raw_length, raw_width);
    box_yaw = normalizeAngle(box_yaw + 0.5 * kPi);
  }

  if (!std::isfinite(center_x) || !std::isfinite(center_y) ||
    !std::isfinite(raw_length) || !std::isfinite(raw_width) ||
    raw_length < config.min_object_length ||
    raw_length > config.max_object_length ||
    raw_width > config.max_object_width)
  {
    return false;
  }

  detection.x = center_x;
  detection.y = center_y;
  detection.yaw = normalizeAngle(box_yaw);
  detection.length = std::max(raw_length, config.min_box_dimension);
  detection.width = std::max(raw_width, config.min_box_dimension);
  detection.confidence = std::clamp(
    count / static_cast<double>(config.confidence_full_points), 0.0, 1.0);
  detection.point_count = cluster.size();
  detection.points.clear();
  detection.points.reserve(cluster.size());
  for (const auto & sample : cluster) {
    detection.points.push_back(Point2D{sample.x, sample.y});
  }
  return true;
}

}  // namespace

ScanClusterer::ScanClusterer(ScanClustererConfig config)
: config_(std::move(config))
{
  if (!finiteAndPositive(config_.min_range) ||
    !finiteAndPositive(config_.max_range) ||
    config_.max_range <= config_.min_range ||
    !std::isfinite(config_.breakpoint_base) || config_.breakpoint_base < 0.0 ||
    !std::isfinite(config_.breakpoint_scale) || config_.breakpoint_scale < 0.0 ||
    config_.min_cluster_points < 2U ||
    !finiteAndPositive(config_.min_object_length) ||
    !finiteAndPositive(config_.max_object_length) ||
    config_.max_object_length < config_.min_object_length ||
    !finiteAndPositive(config_.max_object_width) ||
    !finiteAndPositive(config_.min_box_dimension) ||
    config_.confidence_full_points < config_.min_cluster_points)
  {
    throw std::invalid_argument("invalid scan clusterer configuration");
  }
}

std::vector<Detection> ScanClusterer::detect(
  const std::vector<float> & ranges,
  double angle_min,
  double angle_increment,
  double message_range_min,
  double message_range_max) const
{
  std::vector<Detection> detections;
  if (ranges.empty() || !std::isfinite(angle_min) ||
    !std::isfinite(angle_increment) || angle_increment == 0.0)
  {
    return detections;
  }

  double effective_min_range = config_.min_range;
  if (std::isfinite(message_range_min) && message_range_min > 0.0) {
    effective_min_range = std::max(effective_min_range, message_range_min);
  }

  double effective_max_range = config_.max_range;
  if (std::isfinite(message_range_max) && message_range_max > 0.0) {
    effective_max_range = std::min(effective_max_range, message_range_max);
  }
  if (effective_max_range <= effective_min_range) {
    return detections;
  }

  std::vector<Sample> cluster;
  cluster.reserve(ranges.size());

  const auto flush_cluster = [&]() {
      Detection detection;
      if (buildDetection(cluster, config_, detection)) {
        detections.push_back(detection);
      }
      cluster.clear();
    };

  for (std::size_t index = 0U; index < ranges.size(); ++index) {
    const double range = static_cast<double>(ranges[index]);
    if (!std::isfinite(range) ||
      range < effective_min_range || range > effective_max_range)
    {
      flush_cluster();
      continue;
    }

    const double angle = angle_min + static_cast<double>(index) * angle_increment;
    const Sample sample{
      range * std::cos(angle),
      range * std::sin(angle),
      range};

    if (!cluster.empty()) {
      const auto & previous = cluster.back();
      const double point_gap = std::hypot(
        sample.x - previous.x, sample.y - previous.y);
      const double adaptive_threshold =
        config_.breakpoint_base +
        config_.breakpoint_scale * std::min(sample.range, previous.range) *
        std::abs(angle_increment);
      if (point_gap > adaptive_threshold) {
        flush_cluster();
      }
    }
    cluster.push_back(sample);
  }
  flush_cluster();

  return detections;
}

}  // namespace opponent_tracker
