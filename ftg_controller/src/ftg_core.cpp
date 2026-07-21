#include "ftg_controller/ftg_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <set>
#include <utility>

namespace ftg_controller
{
namespace
{

constexpr double kEpsilon = 1e-9;

double pi() noexcept
{
  static const double value = std::acos(-1.0);
  return value;
}

double positiveOr(double value, double fallback) noexcept
{
  return std::isfinite(value) && value > 0.0 ? value : fallback;
}

double nonnegativeOr(double value, double fallback) noexcept
{
  return std::isfinite(value) && value >= 0.0 ? value : fallback;
}

FTGConfig sanitizeConfig(FTGConfig config)
{
  const FTGConfig defaults;
  config.field_of_view_deg = positiveOr(config.field_of_view_deg, defaults.field_of_view_deg);
  config.max_lidar_range = positiveOr(config.max_lidar_range, defaults.max_lidar_range);
  config.min_lidar_range = nonnegativeOr(config.min_lidar_range, defaults.min_lidar_range);
  if (config.min_lidar_range >= config.max_lidar_range) {
    config.min_lidar_range = defaults.min_lidar_range;
  }
  config.smoothing_window = std::max(config.smoothing_window, 1);
  config.disparity_threshold =
    nonnegativeOr(config.disparity_threshold, defaults.disparity_threshold);
  config.safety_radius = nonnegativeOr(config.safety_radius, defaults.safety_radius);
  config.min_clearance = nonnegativeOr(config.min_clearance, defaults.min_clearance);
  config.min_gap_width = nonnegativeOr(config.min_gap_width, defaults.min_gap_width);
  config.best_point_window = std::max(config.best_point_window, 1);
  config.candidate_count = std::max(config.candidate_count, 1);
  config.depth_weight = nonnegativeOr(config.depth_weight, defaults.depth_weight);
  config.heading_weight = nonnegativeOr(config.heading_weight, defaults.heading_weight);
  config.gap_center_weight =
    nonnegativeOr(config.gap_center_weight, defaults.gap_center_weight);
  config.path_clearance_weight =
    nonnegativeOr(config.path_clearance_weight, defaults.path_clearance_weight);
  config.path_margin_weight =
    nonnegativeOr(config.path_margin_weight, defaults.path_margin_weight);
  config.continuity_weight =
    nonnegativeOr(config.continuity_weight, defaults.continuity_weight);
  config.gap_switch_hysteresis = std::clamp(
    nonnegativeOr(config.gap_switch_hysteresis, defaults.gap_switch_hysteresis), 0.0, 1.0);
  config.lidar_offset = nonnegativeOr(config.lidar_offset, defaults.lidar_offset);
  config.min_lookahead = positiveOr(config.min_lookahead, defaults.min_lookahead);
  config.max_lookahead = positiveOr(config.max_lookahead, defaults.max_lookahead);
  if (config.max_lookahead < config.min_lookahead) {
    config.max_lookahead = config.min_lookahead;
  }
  config.wheelbase = positiveOr(config.wheelbase, defaults.wheelbase);
  config.max_steering_angle =
    positiveOr(config.max_steering_angle, defaults.max_steering_angle);
  config.path_sweep_radius =
    nonnegativeOr(config.path_sweep_radius, defaults.path_sweep_radius);
  config.path_horizon = positiveOr(config.path_horizon, defaults.path_horizon);
  config.emergency_distance =
    nonnegativeOr(config.emergency_distance, defaults.emergency_distance);
  return config;
}

CommandConfig sanitizeConfig(CommandConfig config)
{
  const CommandConfig defaults;
  config.wheelbase = positiveOr(config.wheelbase, defaults.wheelbase);
  config.max_steering_angle =
    positiveOr(config.max_steering_angle, defaults.max_steering_angle);
  config.max_steering_rate =
    positiveOr(config.max_steering_rate, defaults.max_steering_rate);
  config.steering_time_constant =
    nonnegativeOr(config.steering_time_constant, defaults.steering_time_constant);
  config.max_speed = nonnegativeOr(config.max_speed, defaults.max_speed);
  config.max_lateral_accel =
    positiveOr(config.max_lateral_accel, defaults.max_lateral_accel);
  config.emergency_distance =
    nonnegativeOr(config.emergency_distance, defaults.emergency_distance);
  config.max_accel = positiveOr(config.max_accel, defaults.max_accel);
  config.max_decel = positiveOr(config.max_decel, defaults.max_decel);
  config.max_jerk = positiveOr(config.max_jerk, defaults.max_jerk);
  config.nominal_scan_period =
    positiveOr(config.nominal_scan_period, defaults.nominal_scan_period);
  return config;
}

std::vector<double> movingMean(const std::vector<double> & values, int requested_window)
{
  if (values.size() < 2 || requested_window <= 1) {
    return values;
  }
  int window = std::min(requested_window, static_cast<int>(values.size()));
  if (window % 2 == 0) {
    --window;
  }
  if (window <= 1) {
    return values;
  }

  const int half = window / 2;
  std::vector<double> result(values.size(), 0.0);
  for (std::size_t index = 0; index < values.size(); ++index) {
    double sum = 0.0;
    for (int offset = -half; offset <= half; ++offset) {
      const int source_index = std::clamp(
        static_cast<int>(index) + offset, 0, static_cast<int>(values.size()) - 1);
      sum += values[static_cast<std::size_t>(source_index)];
    }
    result[index] = sum / static_cast<double>(window);
  }
  return result;
}

std::vector<double> conservativeSmooth(const std::vector<double> & values, int window)
{
  const std::vector<double> mean = movingMean(values, window);
  std::vector<double> result(values.size(), 0.0);
  for (std::size_t index = 0; index < values.size(); ++index) {
    if (std::isfinite(values[index]) && values[index] > 0.0) {
      result[index] = std::min(values[index], mean[index]);
    }
  }
  return result;
}

double percentile(std::vector<double> values, double fraction)
{
  if (values.empty()) {
    return 0.0;
  }
  std::sort(values.begin(), values.end());
  const double position =
    (static_cast<double>(values.size()) - 1.0) * std::clamp(fraction, 0.0, 1.0);
  const std::size_t lower = static_cast<std::size_t>(std::floor(position));
  const std::size_t upper = static_cast<std::size_t>(std::ceil(position));
  const double blend = position - static_cast<double>(lower);
  return values[lower] + blend * (values[upper] - values[lower]);
}

struct PathProjection
{
  double progress{0.0};
  double distance{std::numeric_limits<double>::infinity()};
};

void considerProjection(
  double obstacle_x, double obstacle_y, double curvature, double theta,
  double horizon, PathProjection & best)
{
  const double progress = theta / curvature;
  if (progress < -kEpsilon || progress > horizon + kEpsilon) {
    return;
  }
  const double path_x = std::sin(theta) / curvature;
  const double path_y = (1.0 - std::cos(theta)) / curvature;
  const double distance = std::hypot(obstacle_x - path_x, obstacle_y - path_y);
  if (distance < best.distance - kEpsilon ||
    (std::abs(distance - best.distance) <= kEpsilon && progress < best.progress))
  {
    best.distance = distance;
    best.progress = std::clamp(progress, 0.0, horizon);
  }
}

PathProjection projectOntoPath(
  double obstacle_x, double obstacle_y, double curvature, double horizon)
{
  if (std::abs(curvature) < 1e-6) {
    const double progress = std::clamp(obstacle_x, 0.0, horizon);
    return {progress, std::hypot(obstacle_x - progress, obstacle_y)};
  }

  PathProjection best;
  considerProjection(obstacle_x, obstacle_y, curvature, 0.0, horizon, best);
  const double theta_end = curvature * horizon;
  considerProjection(obstacle_x, obstacle_y, curvature, theta_end, horizon, best);

  const double raw_theta = std::atan2(
    curvature * obstacle_x, 1.0 - curvature * obstacle_y);
  const double theta_min = std::min(0.0, theta_end);
  const double theta_max = std::max(0.0, theta_end);
  const int first_turn = static_cast<int>(
    std::floor((theta_min - raw_theta) / (2.0 * pi()))) - 1;
  const int last_turn = static_cast<int>(
    std::ceil((theta_max - raw_theta) / (2.0 * pi()))) + 1;
  for (int turn = first_turn; turn <= last_turn; ++turn) {
    const double theta = raw_theta + static_cast<double>(turn) * 2.0 * pi();
    considerProjection(obstacle_x, obstacle_y, curvature, theta, horizon, best);
  }
  return best;
}

struct Candidate
{
  std::size_t index{0};
  Gap gap;
  double range{0.0};
  double bearing{0.0};
  double lookahead{0.0};
  double curvature{0.0};
  double steering_angle{0.0};
  double score{0.0};
  PathMetrics path;
  bool safe{false};
};

}  // namespace

const char * toString(StopReason reason) noexcept
{
  switch (reason) {
    case StopReason::kNone:
      return "none";
    case StopReason::kInvalidScan:
      return "invalid_scan";
    case StopReason::kNoGap:
      return "no_gap";
    case StopReason::kPathEmergency:
      return "path_emergency";
  }
  return "invalid_scan";
}

bool ProcessedScan::valid() const noexcept
{
  if (ranges.empty() || ranges.size() != angles.size() ||
    !std::isfinite(angle_increment) || std::abs(angle_increment) <= kEpsilon)
  {
    return false;
  }
  for (std::size_t index = 0; index < ranges.size(); ++index) {
    if (!std::isfinite(ranges[index]) || ranges[index] < 0.0 ||
      !std::isfinite(angles[index]))
    {
      return false;
    }
  }
  return true;
}

FollowTheGapPlanner::FollowTheGapPlanner(FTGConfig config)
: config_(sanitizeConfig(std::move(config)))
{
}

ProcessedScan FollowTheGapPlanner::preprocessScan(
  const std::vector<float> & input_ranges,
  double angle_min,
  double angle_increment,
  double sensor_range_min,
  double sensor_range_max) const
{
  ProcessedScan output;
  if (input_ranges.empty() || !std::isfinite(angle_min) ||
    !std::isfinite(angle_increment) || std::abs(angle_increment) <= kEpsilon)
  {
    return output;
  }

  const double sensor_max =
    std::isfinite(sensor_range_max) && sensor_range_max > 0.0 ?
    sensor_range_max : config_.max_lidar_range;
  const double range_cap = std::min(config_.max_lidar_range, sensor_max);
  const double sensor_min =
    std::isfinite(sensor_range_min) && sensor_range_min >= 0.0 ?
    sensor_range_min : config_.min_lidar_range;
  const double range_floor = std::max(config_.min_lidar_range, sensor_min);
  if (range_floor >= range_cap) {
    return output;
  }

  const double half_fov = config_.field_of_view_deg * pi() / 360.0;
  output.ranges.reserve(input_ranges.size());
  output.angles.reserve(input_ranges.size());
  for (std::size_t index = 0; index < input_ranges.size(); ++index) {
    const double angle = angle_min + static_cast<double>(index) * angle_increment;
    if (!std::isfinite(angle) || std::abs(angle) > half_fov + kEpsilon) {
      continue;
    }

    double range = static_cast<double>(input_ranges[index]);
    if (range == std::numeric_limits<double>::infinity()) {
      range = range_cap;
    } else if (!std::isfinite(range)) {
      range = 0.0;
    }
    if (range < range_floor) {
      range = 0.0;
    } else {
      range = std::min(range, range_cap);
    }
    output.ranges.push_back(range);
    output.angles.push_back(angle);
  }

  if (output.ranges.empty()) {
    return {};
  }
  output.ranges = conservativeSmooth(output.ranges, config_.smoothing_window);
  output.angle_increment = angle_increment;
  return output;
}

std::vector<std::uint8_t> FollowTheGapPlanner::createTraversabilityMask(
  const std::vector<double> & ranges, double angle_increment) const
{
  std::vector<std::uint8_t> traversable(ranges.size(), 0U);
  if (ranges.empty() || !std::isfinite(angle_increment) ||
    std::abs(angle_increment) <= kEpsilon)
  {
    return traversable;
  }

  for (std::size_t index = 0; index < ranges.size(); ++index) {
    traversable[index] = static_cast<std::uint8_t>(
      std::isfinite(ranges[index]) && ranges[index] >= config_.min_clearance);
  }

  const double angular_resolution = std::abs(angle_increment);
  for (std::size_t left = 0; left + 1 < ranges.size(); ++left) {
    const std::size_t right = left + 1;
    const double left_range = ranges[left];
    const double right_range = ranges[right];
    if (!std::isfinite(left_range) || !std::isfinite(right_range) ||
      left_range <= 0.0 || right_range <= 0.0 ||
      std::abs(right_range - left_range) <= config_.disparity_threshold)
    {
      continue;
    }

    const bool left_is_close = left_range < right_range;
    const int close_index = static_cast<int>(left_is_close ? left : right);
    const double close_range = left_is_close ? left_range : right_range;
    const int direction = left_is_close ? 1 : -1;
    const double half_angle = std::atan2(config_.safety_radius, close_range);
    const int inflation_count = static_cast<int>(std::ceil(half_angle / angular_resolution));
    for (int offset = 0; offset <= inflation_count; ++offset) {
      const int inflated_index = close_index + direction * offset;
      if (inflated_index < 0 || inflated_index >= static_cast<int>(ranges.size())) {
        break;
      }
      traversable[static_cast<std::size_t>(inflated_index)] = 0U;
    }
  }
  return traversable;
}

std::vector<Gap> FollowTheGapPlanner::findGaps(
  const std::vector<double> & ranges,
  const std::vector<std::uint8_t> & traversable,
  double angle_increment) const
{
  std::vector<Gap> gaps;
  if (ranges.empty() || ranges.size() != traversable.size() ||
    !std::isfinite(angle_increment) || std::abs(angle_increment) <= kEpsilon)
  {
    return gaps;
  }

  std::size_t index = 0;
  while (index < ranges.size()) {
    while (index < ranges.size() && traversable[index] == 0U) {
      ++index;
    }
    const std::size_t begin = index;
    while (index < ranges.size() && traversable[index] != 0U) {
      ++index;
    }
    const std::size_t end = index;
    if (begin == end) {
      continue;
    }

    std::vector<double> gap_ranges(ranges.begin() + begin, ranges.begin() + end);
    const double angular_width =
      static_cast<double>(end - begin) * std::abs(angle_increment);
    const double representative_range = percentile(std::move(gap_ranges), 0.5);
    const double physical_width = 2.0 * representative_range *
      std::sin(std::min(angular_width, pi()) * 0.5);
    if (physical_width + kEpsilon >= config_.min_gap_width) {
      gaps.push_back({begin, end, angular_width, physical_width});
    }
  }
  return gaps;
}

PathMetrics FollowTheGapPlanner::evaluatePath(
  const std::vector<double> & ranges,
  const std::vector<double> & angles,
  double curvature) const
{
  PathMetrics metrics;
  if (ranges.empty() || ranges.size() != angles.size() || !std::isfinite(curvature)) {
    metrics.collision_distance = 0.0;
    metrics.lateral_margin = -config_.path_sweep_radius;
    return metrics;
  }

  for (std::size_t index = 0; index < ranges.size(); ++index) {
    const double range = ranges[index];
    const double angle = angles[index];
    if (!std::isfinite(range) || range <= 0.0 || !std::isfinite(angle)) {
      continue;
    }

    const double obstacle_x = config_.lidar_offset + range * std::cos(angle);
    const double obstacle_y = range * std::sin(angle);
    const PathProjection projection = projectOntoPath(
      obstacle_x, obstacle_y, curvature, config_.path_horizon);
    metrics.lateral_margin = std::min(
      metrics.lateral_margin, projection.distance - config_.path_sweep_radius);
    if (projection.distance <= config_.path_sweep_radius + kEpsilon) {
      const double entry_offset = std::sqrt(std::max(
        config_.path_sweep_radius * config_.path_sweep_radius -
        projection.distance * projection.distance, 0.0));
      const double entry_progress = std::clamp(
        projection.progress - entry_offset, 0.0, config_.path_horizon);
      metrics.collision_distance = std::min(metrics.collision_distance, entry_progress);
    }
  }
  return metrics;
}

PlanResult FollowTheGapPlanner::plan(const ProcessedScan & scan)
{
  PlanResult result;
  if (!scan.valid()) {
    previous_target_angle_.reset();
    result.stop_reason = StopReason::kInvalidScan;
    return result;
  }

  const std::vector<std::uint8_t> traversable = createTraversabilityMask(
    scan.ranges, scan.angle_increment);
  const std::vector<Gap> gaps = findGaps(scan.ranges, traversable, scan.angle_increment);
  if (gaps.empty()) {
    previous_target_angle_.reset();
    result.stop_reason = StopReason::kNoGap;
    return result;
  }

  std::optional<std::size_t> previous_index;
  if (previous_target_angle_.has_value()) {
    double smallest_difference = std::numeric_limits<double>::infinity();
    for (std::size_t index = 0; index < scan.angles.size(); ++index) {
      const double difference = std::abs(scan.angles[index] - *previous_target_angle_);
      if (difference < smallest_difference) {
        smallest_difference = difference;
        previous_index = index;
      }
    }
    if (!previous_index.has_value() || traversable[*previous_index] == 0U) {
      previous_index.reset();
      previous_target_angle_.reset();
    }
  }

  std::vector<Candidate> candidates;
  std::optional<std::size_t> previous_candidate;
  const double continuity_span = std::max(
    config_.field_of_view_deg * pi() / 180.0, std::abs(scan.angle_increment));

  for (const Gap & gap : gaps) {
    const std::size_t count = gap.end - gap.begin;
    std::vector<double> gap_ranges(
      scan.ranges.begin() + gap.begin, scan.ranges.begin() + gap.end);
    const std::vector<double> local_depth = movingMean(gap_ranges, config_.best_point_window);
    std::set<std::size_t> candidate_indices;
    candidate_indices.insert(gap.begin + (count - 1U) / 2U);
    const auto depth_peak = std::max_element(local_depth.begin(), local_depth.end());
    candidate_indices.insert(gap.begin + static_cast<std::size_t>(
      std::distance(local_depth.begin(), depth_peak)));

    const std::size_t sample_count = std::min(
      count, static_cast<std::size_t>(config_.candidate_count));
    if (sample_count == 1U) {
      candidate_indices.insert(gap.begin + (count - 1U) / 2U);
    } else {
      for (std::size_t sample = 0; sample < sample_count; ++sample) {
        const double fraction = static_cast<double>(sample) /
          static_cast<double>(sample_count - 1U);
        const std::size_t offset = static_cast<std::size_t>(std::llround(
          fraction * static_cast<double>(count - 1U)));
        candidate_indices.insert(gap.begin + offset);
      }
    }
    if (previous_index.has_value() && *previous_index >= gap.begin && *previous_index < gap.end) {
      candidate_indices.insert(*previous_index);
    }

    for (const std::size_t candidate_index : candidate_indices) {
      const std::size_t offset = candidate_index - gap.begin;
      const double range = scan.ranges[candidate_index];
      const double lidar_angle = scan.angles[candidate_index];
      const double target_x = config_.lidar_offset + range * std::cos(lidar_angle);
      const double target_y = range * std::sin(lidar_angle);
      const double target_distance = std::hypot(target_x, target_y);
      if (!std::isfinite(target_distance) || target_distance <= kEpsilon) {
        continue;
      }

      Candidate candidate;
      candidate.index = candidate_index;
      candidate.gap = gap;
      candidate.range = range;
      candidate.bearing = std::atan2(target_y, target_x);
      candidate.lookahead = std::clamp(
        target_distance, config_.min_lookahead, config_.max_lookahead);
      candidate.curvature = 2.0 * std::sin(candidate.bearing) / candidate.lookahead;
      candidate.steering_angle = std::clamp(
        std::atan(config_.wheelbase * candidate.curvature),
        -config_.max_steering_angle, config_.max_steering_angle);
      candidate.curvature = std::tan(candidate.steering_angle) / config_.wheelbase;
      candidate.path = evaluatePath(scan.ranges, scan.angles, candidate.curvature);
      candidate.safe = !std::isfinite(candidate.path.collision_distance) ||
        candidate.path.collision_distance > config_.emergency_distance;

      const double normalized_depth = std::clamp(
        local_depth[offset] / config_.max_lidar_range, 0.0, 1.0);
      const double normalized_heading = std::clamp(
        0.5 * (std::cos(candidate.bearing) + 1.0), 0.0, 1.0);
      const double half_gap = std::max(0.5 * static_cast<double>(count - 1U), 0.5);
      const double gap_center = 0.5 * static_cast<double>(gap.begin + gap.end - 1U);
      const double normalized_center = std::clamp(
        1.0 - std::abs(static_cast<double>(candidate_index) - gap_center) / half_gap,
        0.0, 1.0);
      const double normalized_path_clearance =
        std::isfinite(candidate.path.collision_distance) ?
        std::clamp(candidate.path.collision_distance / config_.path_horizon, 0.0, 1.0) :
        1.0;
      const double normalized_path_margin =
        std::isfinite(candidate.path.lateral_margin) && config_.path_sweep_radius > kEpsilon ?
        std::clamp(
          (candidate.path.lateral_margin + config_.path_sweep_radius) /
          (2.0 * config_.path_sweep_radius), 0.0, 1.0) :
        1.0;
      const double normalized_continuity = previous_target_angle_.has_value() ?
        std::clamp(
          1.0 - std::abs(lidar_angle - *previous_target_angle_) / continuity_span,
          0.0, 1.0) :
        1.0;
      const double total_weight =
        config_.depth_weight + config_.heading_weight + config_.gap_center_weight +
        config_.path_clearance_weight + config_.path_margin_weight +
        config_.continuity_weight;
      const double weighted_score =
        config_.depth_weight * normalized_depth +
        config_.heading_weight * normalized_heading +
        config_.gap_center_weight * normalized_center +
        config_.path_clearance_weight * normalized_path_clearance +
        config_.path_margin_weight * normalized_path_margin +
        config_.continuity_weight * normalized_continuity;
      candidate.score = total_weight > kEpsilon ? weighted_score / total_weight : 0.0;

      candidates.push_back(candidate);
      if (previous_index.has_value() && candidate_index == *previous_index) {
        previous_candidate = candidates.size() - 1U;
      }
    }
  }

  if (candidates.empty()) {
    previous_target_angle_.reset();
    result.stop_reason = StopReason::kNoGap;
    return result;
  }

  auto scoreLess = [&candidates](std::size_t left, std::size_t right) {
      return candidates[left].score < candidates[right].score;
    };
  std::vector<std::size_t> safe_indices;
  safe_indices.reserve(candidates.size());
  for (std::size_t index = 0; index < candidates.size(); ++index) {
    if (candidates[index].safe) {
      safe_indices.push_back(index);
    }
  }

  std::size_t selected_index = 0;
  if (!safe_indices.empty()) {
    selected_index = *std::max_element(safe_indices.begin(), safe_indices.end(), scoreLess);
    if (previous_candidate.has_value() && candidates[*previous_candidate].safe &&
      candidates[*previous_candidate].score + kEpsilon >=
      candidates[selected_index].score * (1.0 - config_.gap_switch_hysteresis))
    {
      selected_index = *previous_candidate;
      result.retained_previous_target = true;
    }
  } else {
    selected_index = static_cast<std::size_t>(std::distance(
      candidates.begin(), std::max_element(
        candidates.begin(), candidates.end(),
        [](const Candidate & left, const Candidate & right) {
          return left.score < right.score;
        })));
  }

  const Candidate & selected = candidates[selected_index];
  result.target_index = selected.index;
  result.target_angle = scan.angles[selected.index];
  result.target_range = selected.range;
  result.target_bearing = selected.bearing;
  result.lookahead = selected.lookahead;
  result.curvature = selected.curvature;
  result.steering_angle = selected.steering_angle;
  result.score = selected.score;
  result.path = selected.path;
  result.gap = selected.gap;
  if (!selected.safe) {
    previous_target_angle_.reset();
    result.stop_reason = StopReason::kPathEmergency;
    return result;
  }

  previous_target_angle_ = result.target_angle;
  result.stop_reason = StopReason::kNone;
  return result;
}

void FollowTheGapPlanner::reset() noexcept
{
  previous_target_angle_.reset();
}

CommandController::CommandController(CommandConfig config)
: config_(sanitizeConfig(std::move(config)))
{
}

double CommandController::validDt(double dt) const noexcept
{
  return std::isfinite(dt) && dt > 0.0 ? dt : 0.0;
}

double CommandController::updateSteering(double requested_curvature, double dt) noexcept
{
  if (dt <= 0.0) {
    return current_steering_angle_;
  }
  if (!std::isfinite(requested_curvature)) {
    requested_curvature = 0.0;
  }
  const double requested_steering = std::clamp(
    std::atan(config_.wheelbase * requested_curvature),
    -config_.max_steering_angle, config_.max_steering_angle);
  const double alpha = config_.steering_time_constant <= kEpsilon ?
    1.0 : 1.0 - std::exp(-dt / config_.steering_time_constant);
  const double filtered = current_steering_angle_ +
    alpha * (requested_steering - current_steering_angle_);
  const double maximum_change = config_.max_steering_rate * dt;
  current_steering_angle_ = std::clamp(
    filtered,
    current_steering_angle_ - maximum_change,
    current_steering_angle_ + maximum_change);
  current_steering_angle_ = std::clamp(
    current_steering_angle_, -config_.max_steering_angle, config_.max_steering_angle);
  return current_steering_angle_;
}

double CommandController::limitSpeed(double requested_speed, double dt) noexcept
{
  requested_speed = std::clamp(requested_speed, 0.0, config_.max_speed);
  if (dt <= 0.0) {
    return current_speed_;
  }
  if (current_speed_ <= kEpsilon && requested_speed <= kEpsilon) {
    current_speed_ = 0.0;
    const double jerk_change = config_.max_jerk * dt;
    current_acceleration_ = std::clamp(0.0,
      current_acceleration_ - jerk_change, current_acceleration_ + jerk_change);
    return current_speed_;
  }
  const double speed_error = requested_speed - current_speed_;
  const double desired_acceleration = std::clamp(
    speed_error / dt, -config_.max_decel, config_.max_accel);
  const double jerk_change = config_.max_jerk * dt;
  double next_acceleration = std::clamp(
    desired_acceleration,
    current_acceleration_ - jerk_change,
    current_acceleration_ + jerk_change);
  next_acceleration = std::clamp(
    next_acceleration, -config_.max_decel, config_.max_accel);

  double next_speed = current_speed_ + next_acceleration * dt;
  if ((speed_error >= 0.0 && next_speed > requested_speed) ||
    (speed_error <= 0.0 && next_speed < requested_speed))
  {
    next_speed = requested_speed;
    next_acceleration = (next_speed - current_speed_) / dt;
  }
  next_speed = std::clamp(next_speed, 0.0, config_.max_speed);
  next_speed = next_speed <= kEpsilon ? 0.0 : next_speed;
  current_speed_ = next_speed;
  current_acceleration_ = next_acceleration;
  return current_speed_;
}

CommandResult CommandController::makeResult(
  double curvature_limit, double stopping_limit, double requested_speed,
  bool emergency) const noexcept
{
  return {
    current_steering_angle_, current_speed_, current_acceleration_,
    curvature_limit, stopping_limit, requested_speed, emergency};
}

CommandResult CommandController::update(
  double requested_curvature, double path_collision_distance, double dt)
{
  dt = validDt(dt);
  updateSteering(requested_curvature, dt);
  if (std::isnan(path_collision_distance) || path_collision_distance < 0.0 ||
    path_collision_distance <= config_.emergency_distance)
  {
    return emergencyStop();
  }

  const double actual_curvature =
    std::tan(current_steering_angle_) / config_.wheelbase;
  const double curvature_limit = std::abs(actual_curvature) > kEpsilon ?
    std::min(
      config_.max_speed,
      std::sqrt(config_.max_lateral_accel / std::abs(actual_curvature))) :
    config_.max_speed;
  const double stopping_limit = std::isfinite(path_collision_distance) ?
    std::min(
      config_.max_speed,
      std::sqrt(2.0 * config_.max_decel * std::max(
        path_collision_distance - config_.emergency_distance, 0.0))) :
    config_.max_speed;
  const double requested_speed = std::min(
    {config_.max_speed, curvature_limit, stopping_limit});
  limitSpeed(requested_speed, dt);
  return makeResult(curvature_limit, stopping_limit, requested_speed, false);
}

CommandResult CommandController::controlledStop(double dt)
{
  dt = validDt(dt);
  limitSpeed(0.0, dt);
  return makeResult(0.0, 0.0, 0.0, false);
}

CommandResult CommandController::emergencyStop() noexcept
{
  current_speed_ = 0.0;
  current_acceleration_ = 0.0;
  return makeResult(0.0, 0.0, 0.0, true);
}

void CommandController::reset(double speed, double steering_angle) noexcept
{
  current_speed_ = std::clamp(
    std::isfinite(speed) ? speed : 0.0, 0.0, config_.max_speed);
  current_steering_angle_ = std::clamp(
    std::isfinite(steering_angle) ? steering_angle : 0.0,
    -config_.max_steering_angle, config_.max_steering_angle);
  current_acceleration_ = 0.0;
}

}  // namespace ftg_controller
