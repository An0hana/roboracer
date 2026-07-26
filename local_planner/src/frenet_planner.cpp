// Copyright 2026 RoboRacer Team

#include "local_planner/frenet_planner.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iterator>
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
constexpr std::uint8_t kOpponentClassification = 1U;

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

struct QuinticPolynomial
{
  QuinticPolynomial(
    double start_position, double start_slope, double start_second_derivative,
    double end_position, double end_slope, double end_second_derivative,
    double length)
  {
    if (!finite(length) || length <= kEpsilon) {
      throw std::invalid_argument("quintic polynomial length must be positive");
    }
    coefficients[0] = start_position;
    coefficients[1] = start_slope;
    coefficients[2] = 0.5 * start_second_derivative;

    const double length2 = length * length;
    const double length3 = length2 * length;
    const double length4 = length3 * length;
    const double length5 = length4 * length;
    const double position_residual =
      end_position -
      (coefficients[0] + coefficients[1] * length +
      coefficients[2] * length2);
    const double slope_residual =
      end_slope - (coefficients[1] + 2.0 * coefficients[2] * length);
    const double second_residual =
      end_second_derivative - 2.0 * coefficients[2];
    coefficients[3] =
      (10.0 * position_residual - 4.0 * slope_residual * length +
      0.5 * second_residual * length2) / length3;
    coefficients[4] =
      (-15.0 * position_residual + 7.0 * slope_residual * length -
      second_residual * length2) / length4;
    coefficients[5] =
      (6.0 * position_residual - 3.0 * slope_residual * length +
      0.5 * second_residual * length2) / length5;
  }

  [[nodiscard]] double position(double progress) const
  {
    return coefficients[0] + progress * (
      coefficients[1] + progress * (
        coefficients[2] + progress * (
          coefficients[3] + progress * (
            coefficients[4] + progress * coefficients[5]))));
  }

  [[nodiscard]] double slope(double progress) const
  {
    return coefficients[1] + progress * (
      2.0 * coefficients[2] + progress * (
        3.0 * coefficients[3] + progress * (
          4.0 * coefficients[4] + progress * 5.0 * coefficients[5])));
  }

  [[nodiscard]] double secondDerivative(double progress) const
  {
    return 2.0 * coefficients[2] + progress * (
      6.0 * coefficients[3] + progress * (
        12.0 * coefficients[4] + progress * 20.0 * coefficients[5]));
  }

  std::array<double, 6> coefficients{};
};

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
    !finite(config.overtake_offset) || config.overtake_offset <= 0.0 ||
    !finite(config.minimum_lateral_offset) ||
    config.minimum_lateral_offset <= 0.0 ||
    config.minimum_lateral_offset > config.overtake_offset ||
    !finite(config.boundary_sampling_buffer) ||
    config.boundary_sampling_buffer < 0.0 ||
    config.projection_search_radius == 0U ||
    !finite(config.max_projection_distance) || config.max_projection_distance <= 0.0 ||
    !finite(config.max_projection_heading_error) ||
    config.max_projection_heading_error <= 0.0 ||
    config.max_projection_heading_error > 0.5 * 3.14159265358979323846 ||
    !finite(config.minimum_frenet_jacobian) ||
    config.minimum_frenet_jacobian <= 0.0 ||
    !finite(config.vehicle_length) || config.vehicle_length <= 0.0 ||
    !finite(config.vehicle_width) || config.vehicle_width <= 0.0 ||
    !finite(config.rear_overhang) || config.rear_overhang < 0.0 ||
    config.rear_overhang >= config.vehicle_length ||
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
    config.min_acceleration >= 0.0 ||
    config.max_acceleration < 0.0 ||
    config.min_acceleration > config.max_acceleration ||
    !finite(config.max_curvature) || config.max_curvature <= 0.0 ||
    !finite(config.trailing_stop_margin) ||
    config.trailing_stop_margin < 0.0 ||
    !finite(config.weight_lateral_offset) || config.weight_lateral_offset < 0.0 ||
    !finite(config.weight_curvature) || config.weight_curvature < 0.0 ||
    !finite(config.weight_clearance) || config.weight_clearance < 0.0 ||
    !finite(config.weight_switch) || config.weight_switch < 0.0 ||
    !finite(config.weight_speed) || config.weight_speed < 0.0 ||
    !finite(config.weight_horizon) || config.weight_horizon < 0.0 ||
    !finite(config.weight_stop) || config.weight_stop < 0.0)
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
  std::size_t search_radius, std::optional<double> yaw,
  double maximum_heading_error) const
{
  if (!finite(x) || !finite(y) || !valid() || search_radius == 0U ||
    (yaw.has_value() && !finite(*yaw)) ||
    !finite(maximum_heading_error) ||
    maximum_heading_error <= 0.0 ||
    maximum_heading_error > 3.14159265358979323846)
  {
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
      const double reference_yaw = std::atan2(tangent_y, tangent_x);
      const double heading_error = yaw.has_value() ?
        normalizeAngle(*yaw - reference_yaw) : 0.0;
      if (yaw.has_value() &&
        std::abs(heading_error) > maximum_heading_error)
      {
        return;
      }
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
      output.reference_yaw = reference_yaw;
      output.heading_error = heading_error;
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
  std::optional<double> previous_target_d,
  std::optional<int> preferred_side,
  bool return_to_raceline,
  double behavior_speed_scale) const
{
  if (!finite(ego.x) || !finite(ego.y) || !finite(ego.yaw) ||
    !finite(ego.speed) ||
    (ego.curvature.has_value() && !finite(*ego.curvature)) ||
    (preferred_side.has_value() &&
    *preferred_side != -1 && *preferred_side != 0 && *preferred_side != 1) ||
    !finite(behavior_speed_scale) ||
    behavior_speed_scale <= 0.0 || behavior_speed_scale > 1.0)
  {
    throw std::invalid_argument("ego state must be finite");
  }

  FrenetProjection projection = reference_line_.project(
    ego.x, ego.y, projection_hint, config_.projection_search_radius,
    ego.yaw, config_.max_projection_heading_error);
  if ((!finite(projection.distance) ||
    projection.distance > config_.max_projection_distance) &&
    projection_hint.has_value())
  {
    projection = reference_line_.project(
      ego.x, ego.y, std::nullopt, config_.projection_search_radius,
      ego.yaw, config_.max_projection_heading_error);
  }

  PlanResult result;
  if (!finite(projection.distance)) {
    result.reason = "projection_heading_mismatch";
    return result;
  }
  result.projection_index = projection.index;
  result.ego_s = projection.s;
  result.ego_d = projection.d;
  if (projection.distance > config_.max_projection_distance) {
    result.reason = "projection_too_far";
    return result;
  }

  const auto lateral_offsets = lateralOffsets(
    projection, preferred_side, return_to_raceline);
  result.candidates.reserve(
    lateral_offsets.size() * config_.planning_horizons.size() *
    config_.speed_scales.size());
  for (const double target_d : lateral_offsets) {
    const std::string side =
      std::abs(target_d) < kEpsilon ? "nominal" :
      (target_d > 0.0 ? "left" : "right");
    for (const double horizon : config_.planning_horizons) {
      for (const double speed_scale : config_.speed_scales) {
        const double combined_speed_scale =
          speed_scale * behavior_speed_scale;
        std::ostringstream name;
        name << side << "_d" << std::fixed << std::setprecision(2) << target_d
             << "_h" << std::setprecision(1) << horizon
             << "_v" << std::setprecision(0) <<
          combined_speed_scale * 100.0;
        auto candidate = generateCandidate(
          ego, projection, target_d, horizon,
          combined_speed_scale, name.str(),
          obstacles, previous_target_d);
        if (candidate.valid &&
          std::abs(target_d) >= config_.minimum_lateral_offset &&
          !hasRequiredPassingClearance(
            projection, candidate, obstacles))
        {
          candidate.valid = false;
          candidate.reason = "pass_clearance";
        }
        result.candidates.push_back(std::move(candidate));
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
      result.reason = "ok";
    }
  }
  return result;
}

std::vector<double> FrenetPlanner::lateralOffsets(
  const FrenetProjection & projection,
  std::optional<int> preferred_side,
  bool return_to_raceline) const
{
  if (return_to_raceline) {
    return {0.0};
  }
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
  if (preferred_side.has_value() && *preferred_side != 0) {
    const bool current_offset_can_be_held =
      (*preferred_side > 0 &&
      projection.d >= config_.minimum_lateral_offset &&
      projection.d <= available_left + kEpsilon) ||
      (*preferred_side < 0 &&
      projection.d <= -config_.minimum_lateral_offset &&
      -projection.d <= available_right + kEpsilon);
    if (current_offset_can_be_held) {
      offsets.push_back(projection.d);
    }
    offsets.erase(
      std::remove_if(
        offsets.begin(), offsets.end(),
        [&projection, &preferred_side](double offset) {
          return *preferred_side > 0 ?
                 offset <= kEpsilon ||
                 offset + kEpsilon < projection.d :
                 offset >= -kEpsilon ||
                 offset - kEpsilon > projection.d;
        }),
      offsets.end());
    std::sort(offsets.begin(), offsets.end());
    offsets.erase(
      std::unique(
        offsets.begin(), offsets.end(),
        [](double first, double second) {
          return std::abs(first - second) < kEpsilon;
        }),
      offsets.end());
  }
  return offsets;
}

bool FrenetPlanner::hasRequiredPassingClearance(
  const FrenetProjection & projection,
  const CandidateTrajectory & candidate,
  const std::vector<Obstacle> & obstacles) const
{
  const double trajectory_length = *std::max_element(
    config_.planning_horizons.begin(), config_.planning_horizons.end());
  for (const auto & obstacle : obstacles) {
    if (obstacle.classification != kOpponentClassification ||
      !finite(obstacle.x) || !finite(obstacle.y) ||
      !finite(obstacle.confidence) ||
      obstacle.confidence < config_.minimum_obstacle_confidence)
    {
      continue;
    }
    const auto obstacle_projection = reference_line_.project(
      obstacle.x, obstacle.y, projection.index,
      config_.projection_search_radius);
    if (!finite(obstacle_projection.distance)) {
      continue;
    }
    double forward_distance = obstacle_projection.s - projection.s;
    if (forward_distance < 0.0) {
      forward_distance += reference_line_.length();
    }
    const double obstacle_length =
      finite(obstacle.length) && obstacle.length > 0.0 ?
      obstacle.length : config_.default_opponent_length;
    if (forward_distance <= 0.0 ||
      forward_distance >
      trajectory_length + 0.5 * obstacle_length)
    {
      continue;
    }
    const double obstacle_width =
      finite(obstacle.width) && obstacle.width > 0.0 ?
      obstacle.width : config_.default_opponent_width;
    const double required_center_separation =
      0.5 * config_.vehicle_width + 0.5 * obstacle_width +
      config_.collision_margin;
    if (candidate.target_d > 0.0 &&
      candidate.target_d <
      obstacle_projection.d + required_center_separation)
    {
      return false;
    }
    if (candidate.target_d < 0.0 &&
      candidate.target_d >
      obstacle_projection.d - required_center_separation)
    {
      return false;
    }
  }
  return true;
}

CandidateTrajectory FrenetPlanner::generateCandidate(
  const EgoState & ego, const FrenetProjection & projection,
  double target_d, double horizon,
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

  const double trajectory_length = *std::max_element(
    config_.planning_horizons.begin(), config_.planning_horizons.end());
  const std::size_t sample_count = static_cast<std::size_t>(
    std::ceil(trajectory_length / config_.sample_spacing)) + 1U;
  candidate.points.reserve(sample_count);
  const double required_half_width =
    0.5 * config_.vehicle_width + config_.safety_margin;
  const auto start_reference = reference_line_.sample(projection.s);
  const double initial_jacobian =
    1.0 - start_reference.curvature * projection.d;
  const double heading_error =
    normalizeAngle(ego.yaw - start_reference.yaw);
  const double initial_slope =
    initial_jacobian * std::tan(heading_error);
  const double derivative_step = std::max(
    config_.sample_spacing, 0.05);
  const double start_curvature_derivative =
    reference_line_.sample(projection.s + derivative_step).curvature -
    reference_line_.sample(projection.s - derivative_step).curvature;
  const double normalized_start_curvature_derivative =
    start_curvature_derivative /
    (2.0 * derivative_step);
  const double desired_initial_curvature =
    ego.curvature.value_or(start_reference.curvature);
  const double initial_norm_squared =
    initial_jacobian * initial_jacobian +
    initial_slope * initial_slope;
  const double initial_second_derivative =
    (
    desired_initial_curvature *
    std::pow(initial_norm_squared, 1.5) -
    initial_jacobian * initial_jacobian * start_reference.curvature -
    normalized_start_curvature_derivative * projection.d * initial_slope -
    2.0 * start_reference.curvature * initial_slope * initial_slope) /
    std::max(initial_jacobian, config_.minimum_frenet_jacobian);
  const QuinticPolynomial lateral_polynomial(
    projection.d, initial_slope, initial_second_derivative,
    target_d, 0.0, 0.0, horizon);

  for (std::size_t index = 0U; index < sample_count; ++index) {
    const double progress = std::min(
      trajectory_length,
      static_cast<double>(index) * config_.sample_spacing);
    const double polynomial_progress = std::min(progress, horizon);
    const auto reference = reference_line_.sample(projection.s + progress);
    const double lateral = lateral_polynomial.position(polynomial_progress);
    const double lateral_slope =
      lateral_polynomial.slope(polynomial_progress);
    const double lateral_second =
      lateral_polynomial.secondDerivative(polynomial_progress);
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
    point.yaw = normalizeAngle(
      reference.yaw + std::atan2(lateral_slope, jacobian));
    const double curvature_derivative =
      (
      reference_line_.sample(reference.s + derivative_step).curvature -
      reference_line_.sample(reference.s - derivative_step).curvature) /
      (2.0 * derivative_step);
    const double norm_squared =
      jacobian * jacobian + lateral_slope * lateral_slope;
    point.curvature =
      (
      jacobian * jacobian * reference.curvature +
      jacobian * lateral_second +
      curvature_derivative * lateral * lateral_slope +
      2.0 * reference.curvature * lateral_slope * lateral_slope) /
      std::pow(std::max(norm_squared, kEpsilon), 1.5);
    point.speed =
      std::min(reference.speed, config_.max_speed) * speed_scale;
    point.left_width = left_width;
    point.right_width = right_width;
    candidate.points.push_back(point);
  }

  if (!candidate.points.empty()) {
    candidate.points.front().x = ego.x;
    candidate.points.front().y = ego.y;
    candidate.points.front().yaw = ego.yaw;
    candidate.points.front().curvature = desired_initial_curvature;
  }

  std::vector<double> hard_speed_limits;
  hard_speed_limits.reserve(candidate.points.size());
  for (auto & point : candidate.points) {
    const double absolute_curvature = std::abs(point.curvature);
    if (absolute_curvature > config_.max_curvature && candidate.valid) {
      candidate.valid = false;
      candidate.reason = "curvature_limit";
    }
    double hard_speed_limit = config_.max_speed;
    if (absolute_curvature > kEpsilon) {
      hard_speed_limit = std::min(
        hard_speed_limit,
        std::sqrt(config_.max_lateral_acceleration / absolute_curvature));
    }
    hard_speed_limits.push_back(hard_speed_limit);
    point.speed = std::clamp(point.speed, 0.0, hard_speed_limit);
  }

  const double current_speed = std::max(0.0, ego.speed);
  if (!candidate.points.empty()) {
    candidate.points.front().speed = current_speed;
  }
  for (std::size_t index = 1U; index < candidate.points.size(); ++index) {
    const auto & previous = candidate.points[index - 1U];
    auto & point = candidate.points[index];
    const double segment =
      std::hypot(point.x - previous.x, point.y - previous.y);
    const double acceleration_limited_speed = std::sqrt(
      std::max(
        0.0,
        previous.speed * previous.speed +
        2.0 * config_.max_acceleration * segment));
    const double deceleration_limited_speed = std::sqrt(
      std::max(
        0.0,
        previous.speed * previous.speed +
        2.0 * config_.min_acceleration * segment));
    point.speed = std::clamp(
      point.speed, deceleration_limited_speed,
      acceleration_limited_speed);
    if (point.speed > hard_speed_limits[index] + 1.0e-6 &&
      candidate.valid)
    {
      candidate.valid = false;
      candidate.reason = "braking_limit";
    }
    point.speed = std::min(point.speed, hard_speed_limits[index]);
  }

  double elapsed = 0.0;
  double clearance_cost = 0.0;
  double path_distance = 0.0;
  std::optional<double> first_collision_distance;
  std::vector<double> path_distances(candidate.points.size(), 0.0);
  for (std::size_t index = 0U; index < candidate.points.size(); ++index) {
    auto & point = candidate.points[index];
    if (index > 0U) {
      const auto & previous = candidate.points[index - 1U];
      const double segment =
        std::hypot(point.x - previous.x, point.y - previous.y);
      path_distance += segment;
      path_distances[index] = path_distance;
      const double average_speed =
        std::max(0.1, 0.5 * (point.speed + previous.speed));
      elapsed += segment / average_speed;
    }

    const double rear_axle_to_center =
      0.5 * config_.vehicle_length - config_.rear_overhang;
    const Rectangle ego_rectangle{
      point.x + rear_axle_to_center * std::cos(point.yaw),
      point.y + rear_axle_to_center * std::sin(point.yaw), point.yaw,
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
        !first_collision_distance.has_value())
      {
        first_collision_distance = path_distance;
      }
      const double center_distance =
        std::hypot(point.x - predicted_x, point.y - predicted_y);
      clearance_cost += std::exp(-center_distance);
    }
  }

  if (first_collision_distance.has_value()) {
    const double available_stop_distance =
      *first_collision_distance - config_.trailing_stop_margin;
    const double required_stop_distance =
      ego.speed * ego.speed / (-2.0 * config_.min_acceleration);
    const bool nominal_candidate = std::abs(target_d) < kEpsilon;
    const bool can_stop =
      candidate.valid && nominal_candidate &&
      available_stop_distance > kEpsilon &&
      available_stop_distance + kEpsilon >= required_stop_distance;
    if (can_stop) {
      candidate.stopping = true;
      candidate.stop_distance = available_stop_distance;
      candidate.reason = "trailing_stop";
      candidate.name += "_stop";
      const double braking_deceleration = -config_.min_acceleration;
      for (std::size_t index = 0U; index < candidate.points.size(); ++index) {
        const double remaining =
          std::max(0.0, available_stop_distance - path_distances[index]);
        const double braking_speed =
          std::sqrt(2.0 * braking_deceleration * remaining);
        candidate.points[index].speed = std::min(
          {candidate.points[index].speed, braking_speed,
            std::max(0.0, ego.speed)});
      }
      const auto stop_point = std::lower_bound(
        path_distances.begin(), path_distances.end(),
        available_stop_distance);
      const std::size_t stop_index = stop_point == path_distances.end() ?
        candidate.points.size() - 1U :
        static_cast<std::size_t>(
        std::distance(path_distances.begin(), stop_point));
      candidate.points.resize(stop_index + 1U);
      candidate.points.back().speed = 0.0;
    } else if (candidate.valid) {
      candidate.valid = false;
      candidate.reason = "collision";
    }
  }

  for (std::size_t index = 1U; index < candidate.points.size(); ++index) {
    auto & point = candidate.points[index];
    const auto & previous = candidate.points[index - 1U];
    const double segment =
      std::hypot(point.x - previous.x, point.y - previous.y);
    const double acceleration =
      (point.speed * point.speed - previous.speed * previous.speed) /
      std::max(2.0 * segment, kEpsilon);
    point.acceleration = std::clamp(
      acceleration, config_.min_acceleration, config_.max_acceleration);
  }

  const double normalized_clearance =
    clearance_cost / std::max<std::size_t>(candidate.points.size(), 1U);
  candidate.cost =
    config_.weight_lateral_offset * target_d * target_d +
    config_.weight_clearance * normalized_clearance +
    config_.weight_speed * (1.0 - speed_scale) * (1.0 - speed_scale) +
    config_.weight_horizon *
    (trajectory_length - horizon) / trajectory_length;
  if (candidate.stopping) {
    candidate.cost += config_.weight_stop;
  }
  double curvature_cost = 0.0;
  for (const auto & point : candidate.points) {
    curvature_cost +=
      point.curvature * point.curvature * config_.sample_spacing;
  }
  candidate.cost +=
    config_.weight_curvature * curvature_cost /
    std::max(trajectory_length, kEpsilon);
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
