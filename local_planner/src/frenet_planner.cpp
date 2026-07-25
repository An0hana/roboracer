// Copyright 2026 RoboRacer Team

#include "local_planner/frenet_planner.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace local_planner
{
namespace
{

constexpr double kEpsilon = 1.0e-9;
constexpr double kMaximumReferenceSegment = 0.30;

bool finite(double value)
{
  return std::isfinite(value);
}

double normalizeAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

double interpolateAngle(double from, double to, double ratio)
{
  return normalizeAngle(from + ratio * normalizeAngle(to - from));
}

std::string trim(const std::string & value)
{
  const auto first = value.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) {
    return "";
  }
  const auto last = value.find_last_not_of(" \t\r\n");
  return value.substr(first, last - first + 1U);
}

std::vector<std::string> splitCsv(const std::string & line)
{
  std::vector<std::string> fields;
  std::stringstream stream(line);
  std::string field;
  while (std::getline(stream, field, ',')) {
    fields.push_back(trim(field));
  }
  return fields;
}

double parseNumber(
  const std::string & text, std::size_t line_number)
{
  std::size_t consumed = 0U;
  double result = 0.0;
  try {
    result = std::stod(text, &consumed);
  } catch (const std::exception &) {
    throw std::runtime_error(
            "invalid number in race-line CSV line " + std::to_string(line_number));
  }
  if (consumed != text.size() || !finite(result)) {
    throw std::runtime_error(
            "invalid number in race-line CSV line " + std::to_string(line_number));
  }
  return result;
}

double quinticSmoothStep(double ratio)
{
  const double value = std::clamp(ratio, 0.0, 1.0);
  const double squared = value * value;
  const double cubed = squared * value;
  return 10.0 * cubed - 15.0 * cubed * value + 6.0 * cubed * squared;
}

double signedCurvature(
  const TrajectorySample & previous, const TrajectorySample & current,
  const TrajectorySample & next)
{
  const double first_x = current.x - previous.x;
  const double first_y = current.y - previous.y;
  const double second_x = next.x - current.x;
  const double second_y = next.y - current.y;
  const double chord_x = next.x - previous.x;
  const double chord_y = next.y - previous.y;
  const double denominator =
    std::hypot(first_x, first_y) *
    std::hypot(second_x, second_y) *
    std::hypot(chord_x, chord_y);
  if (denominator <= kEpsilon) {
    return 0.0;
  }
  return 2.0 * (first_x * second_y - first_y * second_x) / denominator;
}

struct Rectangle
{
  double x;
  double y;
  double yaw;
  double half_length;
  double half_width;
};

bool rectanglesOverlap(const Rectangle & first, const Rectangle & second)
{
  const double delta_x = second.x - first.x;
  const double delta_y = second.y - first.y;
  const std::array<double, 4> axis_x{
    std::cos(first.yaw), -std::sin(first.yaw),
    std::cos(second.yaw), -std::sin(second.yaw)};
  const std::array<double, 4> axis_y{
    std::sin(first.yaw), std::cos(first.yaw),
    std::sin(second.yaw), std::cos(second.yaw)};

  const double first_cosine = std::cos(first.yaw);
  const double first_sine = std::sin(first.yaw);
  const double second_cosine = std::cos(second.yaw);
  const double second_sine = std::sin(second.yaw);
  for (std::size_t index = 0U; index < axis_x.size(); ++index) {
    const double separation =
      std::abs(delta_x * axis_x[index] + delta_y * axis_y[index]);
    const double first_projection =
      first.half_length *
      std::abs(first_cosine * axis_x[index] + first_sine * axis_y[index]) +
      first.half_width *
      std::abs(-first_sine * axis_x[index] + first_cosine * axis_y[index]);
    const double second_projection =
      second.half_length *
      std::abs(second_cosine * axis_x[index] + second_sine * axis_y[index]) +
      second.half_width *
      std::abs(-second_sine * axis_x[index] + second_cosine * axis_y[index]);
    if (separation > first_projection + second_projection) {
      return false;
    }
  }
  return true;
}

void validateConfig(const FrenetPlannerConfig & config)
{
  const bool invalid_horizon = config.planning_horizons.empty() ||
    std::any_of(
    config.planning_horizons.begin(), config.planning_horizons.end(),
    [&config](double horizon) {
      return !finite(horizon) || horizon < config.sample_spacing;
    });
  const bool invalid_speed_scale = config.speed_scales.empty() ||
    std::any_of(
    config.speed_scales.begin(), config.speed_scales.end(),
    [](double scale) {
      return !finite(scale) || scale <= 0.0 || scale > 1.0;
    });
  const std::size_t candidate_limit =
    (2U * config.lateral_samples_per_side + 1U) *
    config.planning_horizons.size() * config.speed_scales.size();
  if (!finite(config.sample_spacing) || config.sample_spacing <= 0.0 ||
    invalid_horizon || invalid_speed_scale ||
    config.lateral_samples_per_side == 0U ||
    config.lateral_samples_per_side > 10U ||
    candidate_limit > 1000U ||
    !finite(config.transition_length) || config.transition_length <= 0.0 ||
    !finite(config.overtake_offset) || config.overtake_offset <= 0.0 ||
    !finite(config.minimum_lateral_offset) ||
    config.minimum_lateral_offset <= 0.0 ||
    config.minimum_lateral_offset > config.overtake_offset ||
    !finite(config.boundary_sampling_buffer) ||
    config.boundary_sampling_buffer < 0.0 ||
    config.projection_search_radius == 0U ||
    !finite(config.max_projection_distance) || config.max_projection_distance <= 0.0 ||
    !finite(config.minimum_frenet_jacobian) ||
    config.minimum_frenet_jacobian <= 0.0 ||
    !finite(config.vehicle_length) || config.vehicle_length <= 0.0 ||
    !finite(config.vehicle_width) || config.vehicle_width <= 0.0 ||
    !finite(config.safety_margin) || config.safety_margin < 0.0 ||
    !finite(config.collision_margin) || config.collision_margin < 0.0 ||
    !finite(config.default_opponent_length) || config.default_opponent_length <= 0.0 ||
    !finite(config.default_opponent_width) || config.default_opponent_width <= 0.0 ||
    !finite(config.minimum_obstacle_confidence) ||
    config.minimum_obstacle_confidence < 0.0 ||
    !finite(config.max_speed) || config.max_speed <= 0.0 ||
    !finite(config.max_lateral_acceleration) ||
    config.max_lateral_acceleration <= 0.0 ||
    !finite(config.min_acceleration) || !finite(config.max_acceleration) ||
    config.min_acceleration > config.max_acceleration ||
    !finite(config.max_curvature) || config.max_curvature <= 0.0 ||
    !finite(config.weight_lateral_offset) || config.weight_lateral_offset < 0.0 ||
    !finite(config.weight_curvature) || config.weight_curvature < 0.0 ||
    !finite(config.weight_clearance) || config.weight_clearance < 0.0 ||
    !finite(config.weight_switch) || config.weight_switch < 0.0 ||
    !finite(config.weight_speed) || config.weight_speed < 0.0 ||
    !finite(config.weight_horizon) || config.weight_horizon < 0.0)
  {
    throw std::invalid_argument("invalid Frenet planner configuration");
  }
}

}  // namespace

ReferenceLine ReferenceLine::fromCsv(const std::string & path)
{
  std::ifstream input(path);
  if (!input.is_open()) {
    throw std::runtime_error("unable to open race-line CSV: " + path);
  }

  std::string line;
  std::size_t line_number = 1U;
  if (!std::getline(input, line)) {
    throw std::runtime_error("race-line CSV is empty: " + path);
  }
  const std::vector<std::string> expected{
    "s", "x", "y", "yaw", "curvature", "v_ref", "width_left", "width_right"};
  if (splitCsv(line) != expected) {
    throw std::runtime_error(
            "race-line CSV header must be: "
            "s,x,y,yaw,curvature,v_ref,width_left,width_right");
  }

  std::vector<ReferencePoint> points;
  while (std::getline(input, line)) {
    ++line_number;
    if (trim(line).empty()) {
      continue;
    }
    const auto fields = splitCsv(line);
    if (fields.size() != expected.size()) {
      throw std::runtime_error(
              "race-line CSV line " + std::to_string(line_number) +
              " must contain 8 fields");
    }
    ReferencePoint point;
    point.s = parseNumber(fields[0], line_number);
    point.x = parseNumber(fields[1], line_number);
    point.y = parseNumber(fields[2], line_number);
    point.yaw = normalizeAngle(parseNumber(fields[3], line_number));
    point.curvature = parseNumber(fields[4], line_number);
    point.speed = parseNumber(fields[5], line_number);
    point.width_left = parseNumber(fields[6], line_number);
    point.width_right = parseNumber(fields[7], line_number);
    points.push_back(point);
  }
  return ReferenceLine(std::move(points));
}

ReferenceLine ReferenceLine::fromPoints(std::vector<ReferencePoint> points)
{
  return ReferenceLine(std::move(points));
}

ReferenceLine::ReferenceLine(std::vector<ReferencePoint> points)
: points_(std::move(points))
{
  if (points_.size() < 3U) {
    throw std::invalid_argument("reference line needs at least three points");
  }
  for (std::size_t index = 0U; index < points_.size(); ++index) {
    const auto & point = points_[index];
    if (!finite(point.s) || !finite(point.x) || !finite(point.y) ||
      !finite(point.yaw) || !finite(point.curvature) ||
      !finite(point.speed) || !finite(point.width_left) ||
      !finite(point.width_right) || point.speed <= 0.0 ||
      point.width_left <= 0.0 || point.width_right <= 0.0)
    {
      throw std::invalid_argument("reference line contains invalid values");
    }
    if (index > 0U) {
      const auto & previous = points_[index - 1U];
      const double segment = std::hypot(
        point.x - previous.x, point.y - previous.y);
      if (point.s <= previous.s || segment <= kEpsilon ||
        segment > kMaximumReferenceSegment)
      {
        throw std::invalid_argument(
                "reference line is not strictly increasing or is discontinuous");
      }
      if (std::abs((point.s - previous.s) - segment) >
        std::max(0.02, 0.15 * segment))
      {
        throw std::invalid_argument(
                "reference line s is inconsistent with Cartesian spacing");
      }
    }
  }

  const auto & first = points_.front();
  const auto & last = points_.back();
  closure_length_ = std::hypot(last.x - first.x, last.y - first.y);
  if (closure_length_ <= kEpsilon ||
    closure_length_ > kMaximumReferenceSegment)
  {
    throw std::invalid_argument("reference line is not a continuous closed loop");
  }
  start_s_ = first.s;
  length_ = last.s - start_s_ + closure_length_;
}

bool ReferenceLine::valid() const noexcept
{
  return points_.size() >= 3U && length_ > 0.0;
}

std::size_t ReferenceLine::size() const noexcept
{
  return points_.size();
}

double ReferenceLine::length() const noexcept
{
  return length_;
}

const std::vector<ReferencePoint> & ReferenceLine::points() const noexcept
{
  return points_;
}

double ReferenceLine::wrapS(double s) const
{
  double wrapped = std::fmod(s - start_s_, length_);
  if (wrapped < 0.0) {
    wrapped += length_;
  }
  return start_s_ + wrapped;
}

ReferencePoint ReferenceLine::sample(double s) const
{
  if (!finite(s) || !valid()) {
    throw std::invalid_argument("sample requires finite s and a valid reference line");
  }
  const double wrapped = wrapS(s);
  const auto upper = std::upper_bound(
    points_.begin(), points_.end(), wrapped,
    [](double value, const ReferencePoint & point) {
      return value < point.s;
    });

  const ReferencePoint * first = nullptr;
  const ReferencePoint * second = nullptr;
  double first_s = 0.0;
  double second_s = 0.0;
  if (upper == points_.end()) {
    first = &points_.back();
    second = &points_.front();
    first_s = first->s;
    second_s = start_s_ + length_;
  } else if (upper == points_.begin()) {
    first = &points_.front();
    second = &points_[1U];
    first_s = first->s;
    second_s = second->s;
  } else {
    first = &(*(upper - 1));
    second = &(*upper);
    first_s = first->s;
    second_s = second->s;
  }
  const double ratio = std::clamp(
    (wrapped - first_s) / std::max(second_s - first_s, kEpsilon), 0.0, 1.0);

  ReferencePoint result;
  result.s = wrapped;
  result.x = first->x + ratio * (second->x - first->x);
  result.y = first->y + ratio * (second->y - first->y);
  result.yaw = interpolateAngle(first->yaw, second->yaw, ratio);
  result.curvature =
    first->curvature + ratio * (second->curvature - first->curvature);
  result.speed = first->speed + ratio * (second->speed - first->speed);
  result.width_left =
    first->width_left + ratio * (second->width_left - first->width_left);
  result.width_right =
    first->width_right + ratio * (second->width_right - first->width_right);
  return result;
}

FrenetProjection ReferenceLine::project(
  double x, double y, std::optional<std::size_t> hint,
  std::size_t search_radius) const
{
  if (!finite(x) || !finite(y) || !valid() || search_radius == 0U) {
    throw std::invalid_argument("invalid reference-line projection request");
  }

  FrenetProjection best;
  best.distance = std::numeric_limits<double>::infinity();
  const auto consider = [&](std::size_t index, FrenetProjection & output) {
      const std::size_t next_index = (index + 1U) % points_.size();
      const auto & first = points_[index];
      const auto & second = points_[next_index];
      const double segment_x = second.x - first.x;
      const double segment_y = second.y - first.y;
      const double segment_length = std::hypot(segment_x, segment_y);
      if (segment_length <= kEpsilon) {
        return;
      }
      const double tangent_x = segment_x / segment_length;
      const double tangent_y = segment_y / segment_length;
      const double ratio = std::clamp(
        ((x - first.x) * segment_x + (y - first.y) * segment_y) /
        (segment_length * segment_length), 0.0, 1.0);
      const double projected_x = first.x + ratio * segment_x;
      const double projected_y = first.y + ratio * segment_y;
      const double offset_x = x - projected_x;
      const double offset_y = y - projected_y;
      const double distance = std::hypot(offset_x, offset_y);
      if (distance >= output.distance) {
        return;
      }
      output.index = index;
      output.s = wrapS(first.s + ratio * segment_length);
      output.d = -tangent_y * offset_x + tangent_x * offset_y;
      output.distance = distance;
    };

  if (!hint.has_value() ||
    search_radius * 2U + 1U >= points_.size())
  {
    for (std::size_t index = 0U; index < points_.size(); ++index) {
      consider(index, best);
    }
  } else {
    const auto count = static_cast<std::ptrdiff_t>(points_.size());
    const auto center = static_cast<std::ptrdiff_t>(*hint % points_.size());
    const auto radius = static_cast<std::ptrdiff_t>(search_radius);
    for (std::ptrdiff_t offset = -radius; offset <= radius; ++offset) {
      std::ptrdiff_t wrapped = (center + offset) % count;
      if (wrapped < 0) {
        wrapped += count;
      }
      consider(static_cast<std::size_t>(wrapped), best);
    }
  }
  return best;
}

FrenetPlanner::FrenetPlanner(
  ReferenceLine reference_line, FrenetPlannerConfig config)
: reference_line_(std::move(reference_line)), config_(std::move(config))
{
  if (!reference_line_.valid()) {
    throw std::invalid_argument("Frenet planner requires a valid reference line");
  }
  validateConfig(config_);
}

PlanResult FrenetPlanner::plan(
  const EgoState & ego, const std::vector<Obstacle> & obstacles,
  std::optional<std::size_t> projection_hint,
  std::optional<double> previous_target_d) const
{
  if (!finite(ego.x) || !finite(ego.y) || !finite(ego.yaw) ||
    !finite(ego.speed))
  {
    throw std::invalid_argument("ego state must be finite");
  }

  FrenetProjection projection = reference_line_.project(
    ego.x, ego.y, projection_hint, config_.projection_search_radius);
  if (projection.distance > config_.max_projection_distance &&
    projection_hint.has_value())
  {
    projection = reference_line_.project(
      ego.x, ego.y, std::nullopt, config_.projection_search_radius);
  }

  PlanResult result;
  result.projection_index = projection.index;
  result.ego_s = projection.s;
  result.ego_d = projection.d;
  if (projection.distance > config_.max_projection_distance) {
    return result;
  }

  const auto lateral_offsets = lateralOffsets(projection);
  result.candidates.reserve(
    lateral_offsets.size() * config_.planning_horizons.size() *
    config_.speed_scales.size());
  for (const double target_d : lateral_offsets) {
    const std::string side =
      std::abs(target_d) < kEpsilon ? "nominal" :
      (target_d > 0.0 ? "left" : "right");
    for (const double horizon : config_.planning_horizons) {
      for (const double speed_scale : config_.speed_scales) {
        std::ostringstream name;
        name << side << "_d" << std::fixed << std::setprecision(2) << target_d
             << "_h" << std::setprecision(1) << horizon
             << "_v" << std::setprecision(0) << speed_scale * 100.0;
        result.candidates.push_back(
          generateCandidate(
            projection, target_d, horizon, speed_scale, name.str(),
            obstacles, previous_target_d));
      }
    }
  }

  double minimum_cost = std::numeric_limits<double>::infinity();
  for (std::size_t index = 0U; index < result.candidates.size(); ++index) {
    const auto & candidate = result.candidates[index];
    if (candidate.valid && candidate.cost < minimum_cost) {
      minimum_cost = candidate.cost;
      result.selected_index = index;
      result.valid = true;
    }
  }
  return result;
}

std::vector<double> FrenetPlanner::lateralOffsets(
  const FrenetProjection & projection) const
{
  const double required_half_width =
    0.5 * config_.vehicle_width + config_.safety_margin;
  const double maximum_horizon = *std::max_element(
    config_.planning_horizons.begin(), config_.planning_horizons.end());
  double available_left = config_.overtake_offset;
  double available_right = config_.overtake_offset;
  const std::size_t sample_count = static_cast<std::size_t>(
    std::ceil(maximum_horizon / config_.sample_spacing)) + 1U;
  for (std::size_t index = 0U; index < sample_count; ++index) {
    const double progress = std::min(
      maximum_horizon,
      static_cast<double>(index) * config_.sample_spacing);
    const auto reference = reference_line_.sample(projection.s + progress);
    available_left = std::min(
      available_left,
      reference.width_left - required_half_width -
      config_.boundary_sampling_buffer);
    available_right = std::min(
      available_right,
      reference.width_right - required_half_width -
      config_.boundary_sampling_buffer);
  }

  std::vector<double> offsets{0.0};
  offsets.reserve(2U * config_.lateral_samples_per_side + 1U);
  const auto append_side =
    [this, &offsets](double available, double direction) {
      if (available < config_.minimum_lateral_offset) {
        return;
      }
      for (std::size_t index = 1U;
        index <= config_.lateral_samples_per_side; ++index)
      {
        const double ratio =
          static_cast<double>(index) /
          static_cast<double>(config_.lateral_samples_per_side);
        const double offset = available * ratio;
        if (offset + kEpsilon >= config_.minimum_lateral_offset) {
          offsets.push_back(direction * offset);
        }
      }
    };
  append_side(std::max(0.0, available_left), 1.0);
  append_side(std::max(0.0, available_right), -1.0);
  return offsets;
}

CandidateTrajectory FrenetPlanner::generateCandidate(
  const FrenetProjection & projection, double target_d, double horizon,
  double speed_scale, const std::string & name,
  const std::vector<Obstacle> & obstacles,
  std::optional<double> previous_target_d) const
{
  CandidateTrajectory candidate;
  candidate.name = name;
  candidate.target_d = target_d;
  candidate.horizon = horizon;
  candidate.speed_scale = speed_scale;
  candidate.valid = true;
  candidate.reason = "ok";

  const std::size_t sample_count = static_cast<std::size_t>(
    std::ceil(horizon / config_.sample_spacing)) + 1U;
  candidate.points.reserve(sample_count);
  const double transition =
    std::min(config_.transition_length, 0.5 * horizon);
  const double required_half_width =
    0.5 * config_.vehicle_width + config_.safety_margin;

  for (std::size_t index = 0U; index < sample_count; ++index) {
    const double progress = std::min(
      horizon,
      static_cast<double>(index) * config_.sample_spacing);
    const auto reference = reference_line_.sample(projection.s + progress);
    double lateral = target_d;
    if (progress < transition) {
      const double blend = quinticSmoothStep(progress / transition);
      lateral = projection.d + blend * (target_d - projection.d);
    } else if (progress > horizon - transition) {
      const double blend = quinticSmoothStep(
        (progress - (horizon - transition)) / transition);
      lateral = target_d * (1.0 - blend);
    }
    const double jacobian = 1.0 - reference.curvature * lateral;
    if (jacobian <= config_.minimum_frenet_jacobian &&
      candidate.valid)
    {
      candidate.valid = false;
      candidate.reason = "frenet_singularity";
    }
    const double left_width = reference.width_left - lateral;
    const double right_width = reference.width_right + lateral;
    if ((left_width < required_half_width ||
      right_width < required_half_width) && candidate.valid)
    {
      candidate.valid = false;
      candidate.reason = "track_boundary";
    }

    TrajectorySample point;
    point.s = reference.s;
    point.d = lateral;
    point.x = reference.x - std::sin(reference.yaw) * lateral;
    point.y = reference.y + std::cos(reference.yaw) * lateral;
    point.yaw = reference.yaw;
    point.speed =
      std::min(reference.speed, config_.max_speed) * speed_scale;
    point.left_width = left_width;
    point.right_width = right_width;
    candidate.points.push_back(point);
  }

  if (candidate.points.size() >= 2U) {
    for (std::size_t index = 0U; index < candidate.points.size(); ++index) {
      const std::size_t previous = index == 0U ? 0U : index - 1U;
      const std::size_t next = std::min(
        index + 1U, candidate.points.size() - 1U);
      const double dx =
        candidate.points[next].x - candidate.points[previous].x;
      const double dy =
        candidate.points[next].y - candidate.points[previous].y;
      if (std::hypot(dx, dy) > kEpsilon) {
        candidate.points[index].yaw = std::atan2(dy, dx);
      }
    }
  }
  if (candidate.points.size() >= 3U) {
    for (std::size_t index = 1U; index + 1U < candidate.points.size(); ++index) {
      candidate.points[index].curvature = signedCurvature(
        candidate.points[index - 1U], candidate.points[index],
        candidate.points[index + 1U]);
    }
    candidate.points.front().curvature = candidate.points[1U].curvature;
    candidate.points.back().curvature =
      candidate.points[candidate.points.size() - 2U].curvature;
  }

  double elapsed = 0.0;
  double clearance_cost = 0.0;
  for (std::size_t index = 0U; index < candidate.points.size(); ++index) {
    auto & point = candidate.points[index];
    const double absolute_curvature = std::abs(point.curvature);
    if (absolute_curvature > config_.max_curvature && candidate.valid) {
      candidate.valid = false;
      candidate.reason = "curvature_limit";
    }
    if (absolute_curvature > kEpsilon) {
      point.speed = std::min(
        point.speed,
        std::sqrt(config_.max_lateral_acceleration / absolute_curvature));
    }
    point.speed = std::clamp(point.speed, 0.0, config_.max_speed);

    if (index > 0U) {
      const auto & previous = candidate.points[index - 1U];
      const double segment =
        std::hypot(point.x - previous.x, point.y - previous.y);
      const double average_speed =
        std::max(0.1, 0.5 * (point.speed + previous.speed));
      elapsed += segment / average_speed;
      const double acceleration =
        (point.speed * point.speed - previous.speed * previous.speed) /
        std::max(2.0 * segment, kEpsilon);
      point.acceleration = std::clamp(
        acceleration, config_.min_acceleration, config_.max_acceleration);
    }

    const Rectangle ego_rectangle{
      point.x, point.y, point.yaw,
      0.5 * config_.vehicle_length + 0.5 * config_.collision_margin,
      0.5 * config_.vehicle_width + 0.5 * config_.collision_margin};
    for (const auto & obstacle : obstacles) {
      if (!finite(obstacle.x) || !finite(obstacle.y) ||
        !finite(obstacle.yaw) || !finite(obstacle.vx) ||
        !finite(obstacle.vy) || !finite(obstacle.confidence) ||
        obstacle.confidence < config_.minimum_obstacle_confidence)
      {
        continue;
      }
      const double obstacle_length =
        finite(obstacle.length) && obstacle.length > 0.0 ?
        obstacle.length : config_.default_opponent_length;
      const double obstacle_width =
        finite(obstacle.width) && obstacle.width > 0.0 ?
        obstacle.width : config_.default_opponent_width;
      const double predicted_x = obstacle.x + elapsed * obstacle.vx;
      const double predicted_y = obstacle.y + elapsed * obstacle.vy;
      const Rectangle obstacle_rectangle{
        predicted_x, predicted_y, obstacle.yaw,
        0.5 * obstacle_length + 0.5 * config_.collision_margin,
        0.5 * obstacle_width + 0.5 * config_.collision_margin};
      if (rectanglesOverlap(ego_rectangle, obstacle_rectangle) &&
        candidate.valid)
      {
        candidate.valid = false;
        candidate.reason = "collision";
      }
      const double center_distance =
        std::hypot(point.x - predicted_x, point.y - predicted_y);
      clearance_cost += std::exp(-center_distance);
    }
  }

  const double maximum_horizon = *std::max_element(
    config_.planning_horizons.begin(), config_.planning_horizons.end());
  const double normalized_clearance =
    clearance_cost / std::max<std::size_t>(candidate.points.size(), 1U);
  candidate.cost =
    config_.weight_lateral_offset * target_d * target_d +
    config_.weight_clearance * normalized_clearance +
    config_.weight_speed * (1.0 - speed_scale) * (1.0 - speed_scale) +
    config_.weight_horizon *
    (maximum_horizon - horizon) / maximum_horizon;
  double curvature_cost = 0.0;
  for (const auto & point : candidate.points) {
    curvature_cost +=
      point.curvature * point.curvature * config_.sample_spacing;
  }
  candidate.cost +=
    config_.weight_curvature * curvature_cost / std::max(horizon, kEpsilon);
  if (previous_target_d.has_value()) {
    const double change = target_d - *previous_target_d;
    candidate.cost += config_.weight_switch * change * change;
  }
  return candidate;
}

const ReferenceLine & FrenetPlanner::referenceLine() const noexcept
{
  return reference_line_;
}

}  // namespace local_planner
