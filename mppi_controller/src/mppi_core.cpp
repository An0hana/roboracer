#include "mppi_controller/mppi_core.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <tuple>
#include <utility>

namespace mppi_controller
{
namespace
{

constexpr double kEpsilon = 1.0e-9;
constexpr double kMaximumRaceLineSegment = 0.25;
constexpr double kRaceLineProgressTolerance = 0.03;

bool finite(double value)
{
  return std::isfinite(value);
}

double square(double value)
{
  return value * value;
}

double stableSoftplus(double value)
{
  if (value > 20.0) {
    return value;
  }
  if (value < -20.0) {
    return std::exp(value);
  }
  return std::log1p(std::exp(value));
}

std::string trim(const std::string & text)
{
  const auto begin = text.find_first_not_of(" \t\r\n");
  if (begin == std::string::npos) {
    return {};
  }
  const auto end = text.find_last_not_of(" \t\r\n");
  return text.substr(begin, end - begin + 1U);
}

std::vector<std::string> splitCsv(const std::string & line)
{
  std::vector<std::string> values;
  std::stringstream stream(line);
  std::string value;
  while (std::getline(stream, value, ',')) {
    values.push_back(trim(value));
  }
  return values;
}

double parseNumber(const std::string & value, std::size_t line_number)
{
  std::size_t parsed = 0U;
  double result = 0.0;
  try {
    result = std::stod(value, &parsed);
  } catch (const std::exception &) {
    throw std::runtime_error("Invalid number on race-line CSV line " +
      std::to_string(line_number));
  }
  if (parsed != value.size() || !finite(result)) {
    throw std::runtime_error("Non-finite or malformed number on race-line CSV line " +
      std::to_string(line_number));
  }
  return result;
}

State addScaled(const State & state, const State & derivative, double scale)
{
  State result;
  result.x = state.x + derivative.x * scale;
  result.y = state.y + derivative.y * scale;
  result.yaw = state.yaw + derivative.yaw * scale;
  result.speed = state.speed + derivative.speed * scale;
  result.steering = state.steering + derivative.steering * scale;
  return result;
}

double signedTrackMargin(
  const State & state, const TrackProjection & projection,
  const Waypoint & waypoint, const VehicleConfig & vehicle)
{
  const double rear = -vehicle.rear_overhang;
  const double front = vehicle.length - vehicle.rear_overhang;
  const double half_width = vehicle.width * 0.5;
  const double sine = std::sin(projection.heading_error);
  const double cosine = std::cos(projection.heading_error);
  double minimum_lateral = std::numeric_limits<double>::infinity();
  double maximum_lateral = -std::numeric_limits<double>::infinity();
  for (double longitudinal : {rear, front}) {
    for (double lateral : {-half_width, half_width}) {
      const double track_lateral = projection.lateral_error +
        longitudinal * sine + lateral * cosine;
      minimum_lateral = std::min(minimum_lateral, track_lateral);
      maximum_lateral = std::max(maximum_lateral, track_lateral);
    }
  }
  (void)state;
  const double left_margin = waypoint.width_left - maximum_lateral - vehicle.safety_margin;
  const double right_margin = waypoint.width_right + minimum_lateral - vehicle.safety_margin;
  return std::min(left_margin, right_margin);
}

double barrierViolation(double current_h, double next_h, double gamma)
{
  if (!finite(current_h) || !finite(next_h)) {
    return 0.0;
  }
  return std::max(0.0, (1.0 - gamma) * current_h - next_h);
}

double signedProgressDelta(double previous, double current, double track_length)
{
  double progress = current - previous;
  if (progress > track_length * 0.5) {
    progress -= track_length;
  } else if (progress < -track_length * 0.5) {
    progress += track_length;
  }
  return progress;
}

double maximumReachableDistance(
  double initial_speed, double maximum_speed, double maximum_acceleration,
  double duration)
{
  const double speed = std::clamp(initial_speed, 0.0, maximum_speed);
  if (duration <= 0.0 || maximum_acceleration <= 0.0 || speed >= maximum_speed) {
    return speed * std::max(0.0, duration);
  }
  const double time_to_limit = (maximum_speed - speed) / maximum_acceleration;
  const double accelerating_time = std::min(duration, time_to_limit);
  const double accelerating_distance =
    speed * accelerating_time +
    0.5 * maximum_acceleration * square(accelerating_time);
  return accelerating_distance +
         maximum_speed * std::max(0.0, duration - accelerating_time);
}

// Felzenszwalb/Huttenlocher one-dimensional squared Euclidean distance transform.
void squaredDistanceTransform1d(
  const std::vector<double> & input, std::vector<double> & output)
{
  const auto count = static_cast<int>(input.size());
  std::vector<int> locations(static_cast<std::size_t>(count));
  std::vector<double> boundaries(static_cast<std::size_t>(count + 1));
  int envelope_index = 0;
  locations[0] = 0;
  boundaries[0] = -std::numeric_limits<double>::infinity();
  boundaries[1] = std::numeric_limits<double>::infinity();
  for (int query = 1; query < count; ++query) {
    double intersection = 0.0;
    do {
      const int location = locations[envelope_index];
      intersection =
        ((input[static_cast<std::size_t>(query)] + static_cast<double>(query * query)) -
        (input[static_cast<std::size_t>(location)] + static_cast<double>(location * location))) /
        static_cast<double>(2 * query - 2 * location);
      if (intersection <= boundaries[envelope_index]) {
        --envelope_index;
      } else {
        break;
      }
    } while (envelope_index >= 0);
    ++envelope_index;
    locations[envelope_index] = query;
    boundaries[envelope_index] = intersection;
    boundaries[envelope_index + 1] = std::numeric_limits<double>::infinity();
  }
  envelope_index = 0;
  for (int query = 0; query < count; ++query) {
    while (boundaries[envelope_index + 1] < static_cast<double>(query)) {
      ++envelope_index;
    }
    const int difference = query - locations[envelope_index];
    output[static_cast<std::size_t>(query)] =
      static_cast<double>(difference * difference) +
      input[static_cast<std::size_t>(locations[envelope_index])];
  }
}

}  // namespace

double normalizeAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

BicycleModel::BicycleModel(VehicleConfig config)
: config_(std::move(config))
{
  if (!finite(config_.wheelbase) || config_.wheelbase <= 0.0) {
    throw std::invalid_argument("wheelbase must be positive");
  }
  if (!finite(config_.length) || config_.length <= 0.0 ||
    !finite(config_.rear_overhang) || config_.rear_overhang < 0.0 ||
    config_.rear_overhang >= config_.length || !finite(config_.width) || config_.width <= 0.0 ||
    !finite(config_.safety_margin) || config_.safety_margin < 0.0)
  {
    throw std::invalid_argument("invalid vehicle footprint geometry");
  }
  const std::array<double, 8> limits{{
    config_.min_steering, config_.max_steering,
    config_.min_steering_rate, config_.max_steering_rate,
    config_.min_acceleration, config_.max_acceleration,
    config_.min_speed, config_.max_speed}};
  if (!std::all_of(limits.begin(), limits.end(), [](double value) {return finite(value);})) {
    throw std::invalid_argument("vehicle limits must all be finite");
  }
  if (config_.min_steering >= config_.max_steering ||
    config_.min_steering_rate >= config_.max_steering_rate ||
    config_.min_acceleration >= config_.max_acceleration ||
    config_.min_speed > config_.max_speed)
  {
    throw std::invalid_argument("invalid vehicle limits");
  }
}

State BicycleModel::derivative(const State & state, const Control & control) const
{
  const Control bounded = clampControl(control);
  const State bounded_state = clampState(state);
  State derivative;
  derivative.x = bounded_state.speed * std::cos(bounded_state.yaw);
  derivative.y = bounded_state.speed * std::sin(bounded_state.yaw);
  derivative.yaw = bounded_state.speed * std::tan(bounded_state.steering) / config_.wheelbase;
  derivative.speed = bounded.acceleration;
  derivative.steering = bounded.steering_rate;
  return derivative;
}

State BicycleModel::step(const State & state, const Control & control, double dt) const
{
  if (!finite(dt) || dt <= 0.0) {
    throw std::invalid_argument("integration dt must be positive");
  }
  const State k1 = derivative(state, control);
  const State k2 = derivative(addScaled(state, k1, 0.5 * dt), control);
  const State k3 = derivative(addScaled(state, k2, 0.5 * dt), control);
  const State k4 = derivative(addScaled(state, k3, dt), control);

  State result;
  result.x = state.x + dt * (k1.x + 2.0 * k2.x + 2.0 * k3.x + k4.x) / 6.0;
  result.y = state.y + dt * (k1.y + 2.0 * k2.y + 2.0 * k3.y + k4.y) / 6.0;
  result.yaw = normalizeAngle(
    state.yaw + dt * (k1.yaw + 2.0 * k2.yaw + 2.0 * k3.yaw + k4.yaw) / 6.0);
  result.speed =
    state.speed + dt * (k1.speed + 2.0 * k2.speed + 2.0 * k3.speed + k4.speed) / 6.0;
  result.steering = state.steering +
    dt * (k1.steering + 2.0 * k2.steering + 2.0 * k3.steering + k4.steering) / 6.0;
  return clampState(result);
}

Control BicycleModel::clampControl(const Control & control) const
{
  return Control{
    std::clamp(control.steering_rate, config_.min_steering_rate, config_.max_steering_rate),
    std::clamp(control.acceleration, config_.min_acceleration, config_.max_acceleration)};
}

State BicycleModel::clampState(const State & state) const
{
  State result = state;
  result.yaw = normalizeAngle(result.yaw);
  result.speed = std::clamp(result.speed, config_.min_speed, config_.max_speed);
  result.steering = std::clamp(
    result.steering, config_.min_steering, config_.max_steering);
  return result;
}

const VehicleConfig & BicycleModel::config() const noexcept
{
  return config_;
}

bool stateWithinLimits(const State & state, const VehicleConfig & vehicle) noexcept
{
  return finite(state.x) && finite(state.y) && finite(state.yaw) && finite(state.speed) &&
         finite(state.steering) && state.speed >= vehicle.min_speed - kEpsilon &&
         state.speed <= vehicle.max_speed + kEpsilon &&
         state.steering >= vehicle.min_steering - kEpsilon &&
         state.steering <= vehicle.max_steering + kEpsilon;
}

State propagateState(
  const BicycleModel & model, const State & state, const Control & applied_control,
  double duration, double maximum_step)
{
  if (!finite(duration) || duration < 0.0 || !finite(maximum_step) || maximum_step <= 0.0) {
    throw std::invalid_argument("state propagation duration and maximum step are invalid");
  }
  if (!stateWithinLimits(state, model.config())) {
    throw std::invalid_argument("cannot propagate an out-of-bounds measured state");
  }
  if (duration == 0.0) {
    return state;
  }
  const std::size_t steps = static_cast<std::size_t>(std::ceil(duration / maximum_step));
  const double dt = duration / static_cast<double>(steps);
  State result = state;
  for (std::size_t step = 0U; step < steps; ++step) {
    result = model.step(result, applied_control, dt);
  }
  return result;
}

bool isLocalizationJump(
  const State & previous, const State & current, double elapsed,
  const VehicleConfig & vehicle, double base_position_threshold,
  double base_yaw_threshold) noexcept
{
  if (!stateWithinLimits(previous, vehicle) || !stateWithinLimits(current, vehicle) ||
    !finite(elapsed) || elapsed <= 0.0 || !finite(base_position_threshold) ||
    base_position_threshold <= 0.0 || !finite(base_yaw_threshold) || base_yaw_threshold <= 0.0)
  {
    return true;
  }
  const double maximum_observed_speed = std::max(
    std::abs(previous.speed), std::abs(current.speed));
  const double allowed_position = base_position_threshold +
    1.5 * maximum_observed_speed * elapsed;
  const double maximum_steering = std::max(
    std::abs(vehicle.min_steering), std::abs(vehicle.max_steering));
  const double maximum_yaw_rate = maximum_observed_speed *
    std::abs(std::tan(maximum_steering)) / vehicle.wheelbase;
  const double allowed_yaw = base_yaw_threshold + 1.5 * maximum_yaw_rate * elapsed;
  return std::hypot(current.x - previous.x, current.y - previous.y) > allowed_position ||
         std::abs(normalizeAngle(current.yaw - previous.yaw)) > allowed_yaw;
}

RaceLine RaceLine::fromCsv(const std::string & path)
{
  std::ifstream input(path);
  if (!input.is_open()) {
    throw std::runtime_error("Unable to open race-line CSV: " + path);
  }

  std::string line;
  std::size_t line_number = 1U;
  if (!std::getline(input, line)) {
    throw std::runtime_error("Race-line CSV is empty: " + path);
  }
  const std::vector<std::string> expected{
    "s", "x", "y", "yaw", "curvature", "v_ref", "width_left", "width_right"};
  if (splitCsv(line) != expected) {
    throw std::runtime_error(
            "Race-line CSV header must be: s,x,y,yaw,curvature,v_ref,width_left,width_right");
  }

  std::vector<Waypoint> waypoints;
  while (std::getline(input, line)) {
    ++line_number;
    if (trim(line).empty()) {
      continue;
    }
    const auto values = splitCsv(line);
    if (values.size() != expected.size()) {
      throw std::runtime_error(
              "Race-line CSV line " + std::to_string(line_number) + " must have 8 fields");
    }
    Waypoint waypoint;
    waypoint.s = parseNumber(values[0], line_number);
    waypoint.x = parseNumber(values[1], line_number);
    waypoint.y = parseNumber(values[2], line_number);
    waypoint.yaw = normalizeAngle(parseNumber(values[3], line_number));
    waypoint.curvature = parseNumber(values[4], line_number);
    waypoint.reference_speed = parseNumber(values[5], line_number);
    waypoint.width_left = parseNumber(values[6], line_number);
    waypoint.width_right = parseNumber(values[7], line_number);
    waypoints.push_back(waypoint);
  }
  return RaceLine(std::move(waypoints));
}

RaceLine RaceLine::fromWaypoints(std::vector<Waypoint> waypoints)
{
  return RaceLine(std::move(waypoints));
}

RaceLine::RaceLine(std::vector<Waypoint> waypoints)
: waypoints_(std::move(waypoints))
{
  if (waypoints_.size() < 3U) {
    throw std::invalid_argument("a closed race line needs at least three waypoints");
  }
  for (std::size_t index = 0U; index < waypoints_.size(); ++index) {
    const auto & point = waypoints_[index];
    if (!finite(point.s) || !finite(point.x) || !finite(point.y) || !finite(point.yaw) ||
      !finite(point.curvature) || !finite(point.reference_speed) ||
      !finite(point.width_left) || !finite(point.width_right) ||
      point.reference_speed < 0.0 || point.width_left <= 0.0 || point.width_right <= 0.0)
    {
      throw std::invalid_argument("race line contains invalid waypoint data");
    }
    if (index > 0U && point.s <= waypoints_[index - 1U].s) {
      throw std::invalid_argument("race-line progress s must be strictly increasing");
    }
    if (index > 0U) {
      const Waypoint & previous = waypoints_[index - 1U];
      const double segment_length = std::hypot(point.x - previous.x, point.y - previous.y);
      const double progress_delta = point.s - previous.s;
      if (segment_length <= kEpsilon || segment_length > kMaximumRaceLineSegment) {
        throw std::invalid_argument(
                "race line contains a zero-length segment or spatial discontinuity (>0.25 m)");
      }
      const double tolerance = std::max(
        kRaceLineProgressTolerance, 0.15 * segment_length);
      if (std::abs(progress_delta - segment_length) > tolerance) {
        throw std::invalid_argument("race-line s is inconsistent with Euclidean segment length");
      }
    }
  }
  const auto & first = waypoints_.front();
  const auto & last = waypoints_.back();
  const double closure = std::hypot(last.x - first.x, last.y - first.y);
  if (closure <= kEpsilon || closure > kMaximumRaceLineSegment) {
    throw std::invalid_argument(
            "race line has a zero-length or discontinuous closing segment (>0.25 m)");
  }
  length_ = last.s - first.s + closure;
  if (length_ <= kEpsilon) {
    throw std::invalid_argument("race-line length must be positive");
  }
}

bool RaceLine::valid() const noexcept
{
  return waypoints_.size() >= 3U && length_ > 0.0;
}

std::size_t RaceLine::size() const noexcept
{
  return waypoints_.size();
}

double RaceLine::length() const noexcept
{
  return length_;
}

const std::vector<Waypoint> & RaceLine::waypoints() const noexcept
{
  return waypoints_;
}

const Waypoint & RaceLine::atWrapped(std::ptrdiff_t index) const
{
  const auto count = static_cast<std::ptrdiff_t>(waypoints_.size());
  index %= count;
  if (index < 0) {
    index += count;
  }
  return waypoints_[static_cast<std::size_t>(index)];
}

std::size_t RaceLine::nearestIndex(
  double x, double y, std::optional<std::size_t> hint, std::size_t search_radius) const
{
  if (!finite(x) || !finite(y) || !valid()) {
    throw std::invalid_argument("nearestIndex requires a valid point and race line");
  }
  std::size_t best_index = 0U;
  double best_distance_squared = std::numeric_limits<double>::infinity();
  const auto consider = [&](std::size_t index) {
      const double dx = x - waypoints_[index].x;
      const double dy = y - waypoints_[index].y;
      const double distance_squared = dx * dx + dy * dy;
      if (distance_squared < best_distance_squared) {
        best_distance_squared = distance_squared;
        best_index = index;
      }
    };

  if (!hint.has_value() || search_radius * 2U + 1U >= waypoints_.size()) {
    for (std::size_t index = 0U; index < waypoints_.size(); ++index) {
      consider(index);
    }
  } else {
    const std::size_t center = *hint % waypoints_.size();
    const auto radius = static_cast<std::ptrdiff_t>(search_radius);
    for (std::ptrdiff_t offset = -radius; offset <= radius; ++offset) {
      const auto wrapped = static_cast<std::size_t>(
        (static_cast<std::ptrdiff_t>(center) + offset +
        static_cast<std::ptrdiff_t>(waypoints_.size())) %
        static_cast<std::ptrdiff_t>(waypoints_.size()));
      consider(wrapped);
    }
  }
  return best_index;
}

TrackProjection RaceLine::project(
  const State & state, std::optional<std::size_t> hint, std::size_t search_radius) const
{
  TrackProjection projection;
  projection.index = nearestIndex(state.x, state.y, hint, search_radius);
  const Waypoint & waypoint = waypoints_[projection.index];
  const double dx = state.x - waypoint.x;
  const double dy = state.y - waypoint.y;
  const double cosine = std::cos(waypoint.yaw);
  const double sine = std::sin(waypoint.yaw);
  projection.longitudinal_error = cosine * dx + sine * dy;
  projection.lateral_error = -sine * dx + cosine * dy;
  projection.heading_error = normalizeAngle(state.yaw - waypoint.yaw);
  projection.distance = std::hypot(dx, dy);
  projection.progress = waypoint.s + projection.longitudinal_error;
  const double start = waypoints_.front().s;
  while (projection.progress < start) {
    projection.progress += length_;
  }
  while (projection.progress >= start + length_) {
    projection.progress -= length_;
  }
  return projection;
}

std::vector<Waypoint> RaceLine::localReference(
  std::size_t start_index, std::size_t count, std::size_t stride) const
{
  if (stride == 0U) {
    throw std::invalid_argument("local-reference stride must be positive");
  }
  std::vector<Waypoint> result;
  result.reserve(count);
  for (std::size_t index = 0U; index < count; ++index) {
    result.push_back(atWrapped(static_cast<std::ptrdiff_t>(
      start_index + index * stride)));
  }
  return result;
}

double RaceLine::forwardProgress(double from_s, double to_s) const
{
  double difference = to_s - from_s;
  while (difference < 0.0) {
    difference += length_;
  }
  while (difference >= length_) {
    difference -= length_;
  }
  return difference;
}

DistanceField::DistanceField(
  std::size_t width, std::size_t height, double resolution,
  double origin_x, double origin_y, std::vector<double> distances,
  double origin_yaw)
: width_(width),
  height_(height),
  resolution_(resolution),
  origin_x_(origin_x),
  origin_y_(origin_y),
  origin_yaw_(origin_yaw),
  distances_(std::move(distances))
{
  if (width_ == 0U || height_ == 0U || !finite(resolution_) || resolution_ <= 0.0 ||
    !finite(origin_yaw_) || distances_.size() != width_ * height_)
  {
    throw std::invalid_argument("invalid distance-field geometry");
  }
}

DistanceField DistanceField::fromOccupancyGrid(
  std::size_t width, std::size_t height, double resolution,
  double origin_x, double origin_y, const std::vector<std::int8_t> & occupancy,
  std::int8_t occupied_threshold, bool unknown_is_occupied, double origin_yaw)
{
  if (width == 0U || height == 0U || occupancy.size() != width * height ||
    !finite(resolution) || resolution <= 0.0)
  {
    throw std::invalid_argument("invalid occupancy-grid geometry");
  }

  const double maximum_squared_cells = square(static_cast<double>(width + height));
  std::vector<double> squared(width * height, maximum_squared_cells);
  for (std::size_t index = 0U; index < occupancy.size(); ++index) {
    if (occupancy[index] >= occupied_threshold || (unknown_is_occupied && occupancy[index] < 0)) {
      squared[index] = 0.0;
    }
  }

  std::vector<double> source(std::max(width, height));
  std::vector<double> transformed(std::max(width, height));
  std::vector<double> intermediate(width * height);
  for (std::size_t x = 0U; x < width; ++x) {
    source.resize(height);
    transformed.resize(height);
    for (std::size_t y = 0U; y < height; ++y) {
      source[y] = squared[y * width + x];
    }
    squaredDistanceTransform1d(source, transformed);
    for (std::size_t y = 0U; y < height; ++y) {
      intermediate[y * width + x] = transformed[y];
    }
  }
  std::vector<double> distances(width * height);
  const double cell_circumradius = resolution * std::sqrt(0.5);
  const double maximum_distance = resolution * std::hypot(
    static_cast<double>(width), static_cast<double>(height));
  for (std::size_t y = 0U; y < height; ++y) {
    source.resize(width);
    transformed.resize(width);
    for (std::size_t x = 0U; x < width; ++x) {
      source[x] = intermediate[y * width + x];
    }
    squaredDistanceTransform1d(source, transformed);
    for (std::size_t x = 0U; x < width; ++x) {
      distances[y * width + x] = std::min(
        maximum_distance,
        std::max(0.0, resolution * std::sqrt(transformed[x]) - cell_circumradius));
    }
  }
  return DistanceField(
    width, height, resolution, origin_x, origin_y, std::move(distances), origin_yaw);
}

bool DistanceField::valid() const noexcept
{
  return width_ > 0U && height_ > 0U && resolution_ > 0.0 &&
         distances_.size() == width_ * height_;
}

double DistanceField::clearance(double world_x, double world_y) const
{
  if (!valid() || !finite(world_x) || !finite(world_y)) {
    return 0.0;
  }
  const double dx = world_x - origin_x_;
  const double dy = world_y - origin_y_;
  const double cosine = std::cos(origin_yaw_);
  const double sine = std::sin(origin_yaw_);
  double grid_x = (cosine * dx + sine * dy) / resolution_ - 0.5;
  double grid_y = (-sine * dx + cosine * dy) / resolution_ - 0.5;
  constexpr double boundary_tolerance = 1.0e-9;
  if (grid_x < -boundary_tolerance || grid_y < -boundary_tolerance ||
    grid_x > static_cast<double>(width_ - 1U) + boundary_tolerance ||
    grid_y > static_cast<double>(height_ - 1U) + boundary_tolerance)
  {
    return 0.0;
  }
  grid_x = std::clamp(grid_x, 0.0, static_cast<double>(width_ - 1U));
  grid_y = std::clamp(grid_y, 0.0, static_cast<double>(height_ - 1U));
  const auto x0 = static_cast<std::size_t>(std::floor(grid_x));
  const auto y0 = static_cast<std::size_t>(std::floor(grid_y));
  const auto x1 = std::min(x0 + 1U, width_ - 1U);
  const auto y1 = std::min(y0 + 1U, height_ - 1U);
  const double tx = grid_x - static_cast<double>(x0);
  const double ty = grid_y - static_cast<double>(y0);
  const auto value = [&](std::size_t x, std::size_t y) {return distances_[y * width_ + x];};
  const double low = value(x0, y0) * (1.0 - tx) + value(x1, y0) * tx;
  const double high = value(x0, y1) * (1.0 - tx) + value(x1, y1) * tx;
  return std::max(0.0, low * (1.0 - ty) + high * ty);
}

std::size_t DistanceField::width() const noexcept
{
  return width_;
}

std::size_t DistanceField::height() const noexcept
{
  return height_;
}

double DistanceField::resolution() const noexcept
{
  return resolution_;
}

double DistanceField::originX() const noexcept
{
  return origin_x_;
}

double DistanceField::originY() const noexcept
{
  return origin_y_;
}

double DistanceField::originYaw() const noexcept
{
  return origin_yaw_;
}

const std::vector<double> & DistanceField::distances() const noexcept
{
  return distances_;
}

double CostBreakdown::total() const noexcept
{
  return lateral + heading + lag + speed + progress + control + control_change +
         lateral_acceleration + boundary + cbf + collision + terminal_lateral +
         terminal_heading + terminal_progress;
}

double obstacleClearance(
  const State & state, const VehicleConfig & vehicle,
  const std::vector<Obstacle> * obstacles, double time)
{
  if (obstacles == nullptr || obstacles->empty()) {
    return std::numeric_limits<double>::infinity();
  }
  const double rear = -vehicle.rear_overhang;
  const double half_width = vehicle.width * 0.5;
  const double vehicle_cosine = std::cos(state.yaw);
  const double vehicle_sine = std::sin(state.yaw);
  const double vehicle_target_segment = std::max(0.02, half_width);
  const std::size_t vehicle_segments = std::max<std::size_t>(
    1U, static_cast<std::size_t>(std::ceil(vehicle.length / vehicle_target_segment)));
  const double vehicle_segment_length =
    vehicle.length / static_cast<double>(vehicle_segments);
  const double vehicle_radius = std::hypot(half_width, vehicle_segment_length * 0.5);

  double minimum = std::numeric_limits<double>::infinity();
  for (const Obstacle & obstacle : *obstacles) {
    if (!finite(obstacle.x) || !finite(obstacle.y) || !finite(obstacle.yaw) ||
      !finite(obstacle.vx) || !finite(obstacle.vy) ||
      !finite(obstacle.half_length) || !finite(obstacle.half_width) ||
      !finite(obstacle.time_offset) || obstacle.half_length < 0.0 ||
      obstacle.half_width < 0.0)
    {
      continue;
    }
    const double horizon_time = time + obstacle.time_offset;
    const double center_x = obstacle.x + obstacle.vx * horizon_time;
    const double center_y = obstacle.y + obstacle.vy * horizon_time;
    const double obstacle_cosine = std::cos(obstacle.yaw);
    const double obstacle_sine = std::sin(obstacle.yaw);
    const double obstacle_target_segment = std::max(0.02, obstacle.half_width);
    const std::size_t obstacle_segments = std::max<std::size_t>(
      1U, static_cast<std::size_t>(
        std::ceil(2.0 * obstacle.half_length / obstacle_target_segment)));
    const double obstacle_segment_length =
      2.0 * obstacle.half_length / static_cast<double>(obstacle_segments);
    const double obstacle_radius =
      std::hypot(obstacle.half_width, obstacle_segment_length * 0.5);
    for (std::size_t vehicle_segment = 0U; vehicle_segment < vehicle_segments;
      ++vehicle_segment)
    {
      const double longitudinal = rear +
        (static_cast<double>(vehicle_segment) + 0.5) * vehicle_segment_length;
      const double vehicle_x = state.x + vehicle_cosine * longitudinal;
      const double vehicle_y = state.y + vehicle_sine * longitudinal;
      for (std::size_t segment = 0U; segment < obstacle_segments; ++segment) {
        const double offset = -obstacle.half_length +
          (static_cast<double>(segment) + 0.5) * obstacle_segment_length;
        const double obstacle_x = center_x + obstacle_cosine * offset;
        const double obstacle_y = center_y + obstacle_sine * offset;
        minimum = std::min(
          minimum,
          std::hypot(vehicle_x - obstacle_x, vehicle_y - obstacle_y) -
          vehicle_radius - obstacle_radius);
      }
    }
  }
  return minimum;
}

CpuMppiBackend::CpuMppiBackend()
{
  configure(MppiConfig{}, VehicleConfig{});
}

CpuMppiBackend::CpuMppiBackend(MppiConfig config, VehicleConfig vehicle)
{
  configure(config, vehicle);
}

void CpuMppiBackend::configure(const MppiConfig & config, const VehicleConfig & vehicle)
{
  const std::array<double, 14> weights{{
    config.weights.lateral, config.weights.heading, config.weights.lag,
    config.weights.speed, config.weights.progress, config.weights.control,
    config.weights.control_change, config.weights.lateral_acceleration,
    config.weights.boundary, config.weights.cbf, config.weights.collision,
    config.weights.terminal_lateral, config.weights.terminal_heading,
    config.weights.terminal_progress}};
  if (config.rollout_count < 2U || config.horizon_steps == 0U ||
    !finite(config.dt) || config.dt <= 0.0 || !finite(config.lambda) || config.lambda <= 0.0 ||
    !finite(config.steering_rate_stddev) || config.steering_rate_stddev < 0.0 ||
    !finite(config.acceleration_stddev) || config.acceleration_stddev < 0.0 ||
    !finite(config.pure_noise_fraction) || config.pure_noise_fraction < 0.0 ||
    config.pure_noise_fraction > 1.0 ||
    config.nearest_search_radius == 0U || config.cuda_max_map_cells == 0U ||
    config.repair_steps > config.horizon_steps || config.repair_iterations == 0U ||
    !finite(config.repair_budget_ms) || config.repair_budget_ms <= 0.0 ||
    !finite(config.cbf_gamma) || config.cbf_gamma < 0.0 || config.cbf_gamma > 1.0 ||
    !finite(config.max_lateral_acceleration) || config.max_lateral_acceleration <= 0.0 ||
    !finite(config.minimum_preview_distance) || config.minimum_preview_distance <= 0.0 ||
    !finite(config.maximum_heading_error) || config.maximum_heading_error <= 0.0 ||
    config.maximum_heading_error >= kPi * 0.5 ||
    !finite(config.reverse_progress_tolerance) || config.reverse_progress_tolerance < 0.0 ||
    !finite(config.repair_clearance) || config.repair_clearance < 0.0 ||
    !std::all_of(
      weights.begin(), weights.end(),
      [](double value) {return finite(value) && value >= 0.0;}))
  {
    throw std::invalid_argument("invalid MPPI configuration");
  }
  if (vehicle.min_acceleration >= 0.0) {
    throw std::invalid_argument("MPPI vehicle configuration requires braking capability");
  }
  config_ = config;
  vehicle_ = vehicle;
  model_ = BicycleModel(vehicle_);
  nominal_controls_.assign(config_.horizon_steps, Control{});
  random_generator_.seed(config_.random_seed);
  configured_ = true;
}

void CpuMppiBackend::reset()
{
  nominal_controls_.assign(config_.horizon_steps, Control{});
  random_generator_.seed(config_.random_seed);
}

bool CpuMppiBackend::warmup(const RaceLine & race_line, const DistanceField * distance_field)
{
  if (!race_line.valid()) {
    return false;
  }
  const Waypoint & point = race_line.waypoints().front();
  MppiRequest request;
  request.initial_state = State{
    point.x, point.y, point.yaw, std::min(0.1, point.reference_speed), 0.0};
  request.race_line = &race_line;
  request.distance_field = distance_field;
  const MppiResult result = compute(request);
  reset();
  return result.valid;
}

CostBreakdown CpuMppiBackend::evaluateTrajectory(
  const State & initial_state, const std::vector<Control> & controls,
  const RaceLine & race_line, const DistanceField * distance_field,
  std::vector<State> * states, const std::vector<Obstacle> * obstacles,
  const MppiBehavior * behavior) const
{
  const MppiBehavior neutral;
  const MppiBehavior & adaptive = behavior == nullptr ? neutral : *behavior;
  CostBreakdown cost;
  State state = model_.clampState(initial_state);
  std::size_t hint = race_line.nearestIndex(state.x, state.y);
  const auto initial_projection = race_line.project(
    state, hint, config_.nearest_search_radius);
  double previous_progress = initial_projection.progress;
  double cumulative_progress = 0.0;
  double previous_map_h = std::min(
    footprintClearance(state, distance_field),
    obstacleClearance(state, vehicle_, obstacles, 0.0)) -
    vehicle_.safety_margin - config_.repair_clearance;
  Control previous_control{};
  if (states != nullptr) {
    states->clear();
    states->reserve(controls.size() + 1U);
    states->push_back(state);
  }

  for (std::size_t step = 0U; step < controls.size(); ++step) {
    const Control control = model_.clampControl(controls[step]);
    state = model_.step(state, control, config_.dt);
    const double time = static_cast<double>(step + 1U) * config_.dt;
    const TrackProjection projection = race_line.project(
      state, hint, config_.nearest_search_radius);
    hint = projection.index;
    const Waypoint & reference = race_line.waypoints()[hint];
    const double track_h = signedTrackMargin(state, projection, reference, vehicle_);
    // Obstacles share the map's collision/CBF pipeline through a combined
    // margin, so both worlds enforce identical safety semantics.
    const double map_margin = std::min(
      footprintClearance(state, distance_field),
      obstacleClearance(state, vehicle_, obstacles, time)) - vehicle_.safety_margin;
    // vehicle_.safety_margin is the hard collision envelope.
    // repair_clearance is an additional preferred buffer for CBF/boundary
    // costs. Keeping it soft allows a physically safe vehicle that entered
    // this buffer to steer back out instead of declaring every control unsafe.
    const double map_h = map_margin - config_.repair_clearance;
    const double progress = signedProgressDelta(
      previous_progress, projection.progress, race_line.length());
    cumulative_progress += progress;
    const double lateral_acceleration =
      state.speed * state.speed * std::tan(state.steering) / vehicle_.wheelbase;

    const double lateral_error =
      projection.lateral_error - adaptive.lateral_reference_offset;
    cost.lateral += config_.weights.lateral * adaptive.raceline_weight_scale *
      square(lateral_error);
    cost.heading += config_.weights.heading * adaptive.raceline_weight_scale *
      square(projection.heading_error);
    cost.lag += config_.weights.lag * adaptive.raceline_weight_scale *
      square(projection.longitudinal_error);
    const double target_speed =
      std::min(reference.reference_speed, vehicle_.max_speed) * adaptive.speed_scale;
    cost.speed += config_.weights.speed * square(state.speed - target_speed);
    cost.progress -= config_.weights.progress * progress;
    cost.control += config_.weights.control *
      (square(control.steering_rate) + square(control.acceleration));
    cost.control_change += config_.weights.control_change *
      (square(control.steering_rate - previous_control.steering_rate) +
      square(control.acceleration - previous_control.acceleration));
    const double excess_lateral_acceleration = std::max(
      0.0, std::abs(lateral_acceleration) - config_.max_lateral_acceleration);
    cost.lateral_acceleration += config_.weights.lateral_acceleration *
      (square(lateral_acceleration) + 20.0 * square(excess_lateral_acceleration));
    // Race-line widths are a soft global preference only. The live scan-derived
    // distance field is the authoritative collision boundary.
    // The CSV widths are prior geometry, so their influence falls together
    // with race-line confidence. Live scan/CBF safety is scaled separately.
    cost.boundary += config_.weights.boundary * adaptive.raceline_weight_scale *
      square(std::max(0.0, -track_h));
    const double map_cbf = barrierViolation(previous_map_h, map_h, config_.cbf_gamma);
    double map_soft_barrier = 0.0;
    if ((distance_field != nullptr && distance_field->valid()) ||
      (obstacles != nullptr && !obstacles->empty()))
    {
      const double barrier_scale = std::max(0.02, (1.0 - config_.cbf_gamma) * 0.10);
      map_soft_barrier = barrier_scale * stableSoftplus(-map_h / barrier_scale);
    }
    cost.cbf += config_.weights.cbf * adaptive.safety_weight_scale *
      (square(map_cbf) + square(map_soft_barrier));
    if (map_margin < 0.0) {
      cost.collision += config_.weights.collision;
    }
    if (std::abs(projection.heading_error) > config_.maximum_heading_error ||
      progress < -config_.reverse_progress_tolerance)
    {
      cost.collision += config_.weights.collision;
    }

    previous_progress = projection.progress;
    previous_map_h = map_h;
    previous_control = control;
    if (states != nullptr) {
      states->push_back(state);
    }
  }
  const TrackProjection terminal_projection = race_line.project(
    state, hint, config_.nearest_search_radius);
  const double horizon_duration =
    static_cast<double>(controls.size()) * config_.dt;
  const double preview_target = std::min(
    config_.minimum_preview_distance,
    maximumReachableDistance(
      initial_state.speed, vehicle_.max_speed, vehicle_.max_acceleration,
      horizon_duration));
  cost.terminal_lateral =
    config_.weights.terminal_lateral * adaptive.raceline_weight_scale *
    square(
    terminal_projection.lateral_error - adaptive.lateral_reference_offset);
  cost.terminal_heading =
    config_.weights.terminal_heading * adaptive.raceline_weight_scale *
    square(terminal_projection.heading_error);
  cost.terminal_progress = config_.weights.terminal_progress *
    square(std::max(0.0, preview_target - cumulative_progress));
  return cost;
}

double CpuMppiBackend::footprintClearance(
  const State & state, const DistanceField * distance_field) const
{
  if (distance_field == nullptr || !distance_field->valid()) {
    return std::numeric_limits<double>::infinity();
  }
  const double rear = -vehicle_.rear_overhang;
  const double half_width = vehicle_.width * 0.5;
  const double cosine = std::cos(state.yaw);
  const double sine = std::sin(state.yaw);
  // Cover the complete rectangle by a chain of circumscribed disks. Every point in
  // each longitudinal rectangle slice lies inside its disk, so a positive result is
  // a conservative clearance certificate rather than sparse point sampling.
  const double target_segment_length = std::max(0.02, half_width);
  const std::size_t segment_count = std::max<std::size_t>(
    1U, static_cast<std::size_t>(std::ceil(vehicle_.length / target_segment_length)));
  const double segment_length = vehicle_.length / static_cast<double>(segment_count);
  const double cover_radius = std::hypot(half_width, segment_length * 0.5);
  double minimum = std::numeric_limits<double>::infinity();
  for (std::size_t segment = 0U; segment < segment_count; ++segment) {
    const double longitudinal = rear +
      (static_cast<double>(segment) + 0.5) * segment_length;
    const double x = state.x + cosine * longitudinal;
    const double y = state.y + sine * longitudinal;
    minimum = std::min(
      minimum, distance_field->clearance(x, y) - cover_radius);
  }
  return minimum;
}

bool CpuMppiBackend::stateSafe(
  const State & state, const RaceLine & race_line,
  const DistanceField * distance_field,
  const std::vector<Obstacle> * obstacles, double time,
  std::size_t * hint) const
{
  const std::optional<std::size_t> optional_hint = hint == nullptr ?
    std::nullopt : std::optional<std::size_t>(*hint);
  const TrackProjection projection = race_line.project(
    state, optional_hint, config_.nearest_search_radius);
  if (hint != nullptr) {
    *hint = projection.index;
  }
  const double map_margin = std::min(
    footprintClearance(state, distance_field),
    obstacleClearance(state, vehicle_, obstacles, time)) - vehicle_.safety_margin;
  return finite(state.x) && finite(state.y) && finite(state.yaw) && finite(state.speed) &&
         finite(state.steering) && map_margin >= 0.0 &&
         std::abs(projection.heading_error) <= config_.maximum_heading_error;
}

TrajectoryMetrics CpuMppiBackend::trajectoryMetrics(
  const State & initial_state, const std::vector<State> & states,
  const RaceLine & race_line, const DistanceField * distance_field,
  const std::vector<Obstacle> * obstacles) const
{
  TrajectoryMetrics metrics;
  State previous_state = initial_state;
  std::size_t hint = race_line.nearestIndex(initial_state.x, initial_state.y);
  TrackProjection previous_projection = race_line.project(
    initial_state, hint, config_.nearest_search_radius);
  hint = previous_projection.index;
  metrics.target_speed = std::min(
    race_line.waypoints()[hint].reference_speed, vehicle_.max_speed);
  const std::size_t predicted_steps = states.empty() ? 0U : states.size() - 1U;
  metrics.preview_target = std::min(
    config_.minimum_preview_distance,
    maximumReachableDistance(
      initial_state.speed, vehicle_.max_speed, vehicle_.max_acceleration,
      static_cast<double>(predicted_steps) * config_.dt));
  metrics.maximum_heading_error = std::abs(previous_projection.heading_error);
  metrics.minimum_obstacle_clearance =
    obstacleClearance(initial_state, vehicle_, obstacles, 0.0) - vehicle_.safety_margin;
  metrics.minimum_clearance = std::min(
    footprintClearance(initial_state, distance_field) - vehicle_.safety_margin,
    metrics.minimum_obstacle_clearance);
  metrics.stopping_distance = square(std::max(0.0, initial_state.speed)) /
    (2.0 * std::abs(vehicle_.min_acceleration));

  const std::size_t first_index =
    !states.empty() &&
    std::hypot(states.front().x - initial_state.x, states.front().y - initial_state.y) < 1.0e-9 ?
    1U : 0U;
  for (std::size_t index = first_index; index < states.size(); ++index) {
    const State & state = states[index];
    // states[first_index] is the pose after the first control step, so its
    // rollout-relative time is one dt regardless of whether the initial pose
    // was included in the vector.
    const double time =
      static_cast<double>(index - first_index + 1U) * config_.dt;
    const TrackProjection projection = race_line.project(
      state, hint, config_.nearest_search_radius);
    hint = projection.index;
    const double progress = signedProgressDelta(
      previous_projection.progress, projection.progress, race_line.length());
    metrics.forward_progress += progress;
    metrics.predicted_distance += std::hypot(
      state.x - previous_state.x, state.y - previous_state.y);
    metrics.maximum_heading_error = std::max(
      metrics.maximum_heading_error, std::abs(projection.heading_error));
    const double obstacle_margin =
      obstacleClearance(state, vehicle_, obstacles, time) - vehicle_.safety_margin;
    metrics.minimum_obstacle_clearance = std::min(
      metrics.minimum_obstacle_clearance, obstacle_margin);
    metrics.minimum_clearance = std::min(
      metrics.minimum_clearance,
      std::min(
        footprintClearance(state, distance_field) - vehicle_.safety_margin,
        obstacle_margin));
    if (progress < -config_.reverse_progress_tolerance) {
      ++metrics.reverse_steps;
    }
    previous_state = state;
    previous_projection = projection;
  }
  return metrics;
}

bool CpuMppiBackend::repairControls(
  const State & initial_state, std::vector<Control> & controls,
  const RaceLine & race_line, const DistanceField * distance_field,
  std::chrono::steady_clock::time_point deadline,
  const std::vector<Obstacle> * obstacles) const
{
  if (controls.size() < config_.repair_steps || !stateSafe(
      initial_state, race_line, distance_field, obstacles, 0.0))
  {
    return false;
  }
  for (std::size_t iteration = 0U; iteration < config_.repair_iterations; ++iteration) {
    if (std::chrono::steady_clock::now() >= deadline) {
      return false;
    }
    bool all_safe = true;
    State state = initial_state;
    std::size_t hint = race_line.nearestIndex(state.x, state.y);
    for (std::size_t step = 0U; step < config_.repair_steps; ++step) {
      controls[step] = model_.clampControl(controls[step]);
      const State candidate = model_.step(state, controls[step], config_.dt);
      const double candidate_time = static_cast<double>(step + 1U) * config_.dt;
      if (!stateSafe(candidate, race_line, distance_field, obstacles, candidate_time, &hint)) {
        all_safe = false;
        const TrackProjection projection = race_line.project(
          state, hint, config_.nearest_search_radius);
        const Waypoint & reference = race_line.waypoints()[projection.index];
        const double desired_heading = normalizeAngle(
          reference.yaw - std::atan(1.5 * projection.lateral_error));
        const double heading_error = normalizeAngle(desired_heading - state.yaw);
        const double desired_curvature = 2.0 * std::sin(heading_error) /
          std::max(0.5, state.speed * 0.5 + 0.5);
        const double desired_steering = std::clamp(
          std::atan(vehicle_.wheelbase * desired_curvature),
          vehicle_.min_steering, vehicle_.max_steering);
        controls[step].steering_rate = std::clamp(
          (desired_steering - state.steering) / config_.dt,
          vehicle_.min_steering_rate, vehicle_.max_steering_rate);
        controls[step].acceleration = std::clamp(
          std::min(controls[step].acceleration, -0.5 * state.speed / config_.dt),
          vehicle_.min_acceleration, vehicle_.max_acceleration);
      }
      state = model_.step(state, controls[step], config_.dt);
    }
    if (all_safe) {
      return true;
    }
  }

  State state = initial_state;
  std::size_t hint = race_line.nearestIndex(state.x, state.y);
  for (std::size_t step = 0U; step < config_.repair_steps; ++step) {
    if (std::chrono::steady_clock::now() >= deadline) {
      return false;
    }
    controls[step] = model_.clampControl(controls[step]);
    state = model_.step(state, controls[step], config_.dt);
    const double time = static_cast<double>(step + 1U) * config_.dt;
    if (!stateSafe(state, race_line, distance_field, obstacles, time, &hint)) {
      return false;
    }
  }
  return true;
}

MppiResult CpuMppiBackend::compute(const MppiRequest & request)
{
  const auto start_time = std::chrono::steady_clock::now();
  MppiResult result;
  result.rollout_count = config_.rollout_count;
  if (!configured_ || request.race_line == nullptr || !request.race_line->valid()) {
    result.reason = "invalid_request";
    return result;
  }
  const std::array<double, 5> state_values{{
    request.initial_state.x, request.initial_state.y, request.initial_state.yaw,
    request.initial_state.speed, request.initial_state.steering}};
  if (!std::all_of(state_values.begin(), state_values.end(), finite)) {
    result.reason = "nonfinite_state";
    return result;
  }
  if (!stateWithinLimits(request.initial_state, vehicle_))
  {
    result.reason = "initial_state_out_of_bounds";
    return result;
  }
  if (!stateSafe(
      request.initial_state, *request.race_line, request.distance_field,
      request.obstacles, 0.0))
  {
    result.reason = "initial_state_unsafe";
    return result;
  }
  if (!finite(request.exploration_scale) || request.exploration_scale < 1.0) {
    result.reason = "invalid_exploration_scale";
    return result;
  }
  const std::array<double, 4> behavior_values{{
    request.behavior.speed_scale,
    request.behavior.raceline_weight_scale,
    request.behavior.safety_weight_scale,
    request.behavior.lateral_reference_offset}};
  if (!std::all_of(behavior_values.begin(), behavior_values.end(), finite) ||
    request.behavior.speed_scale < 0.0 ||
    request.behavior.speed_scale > 1.0 ||
    request.behavior.raceline_weight_scale <= 0.0 ||
    request.behavior.safety_weight_scale < 1.0)
  {
    result.reason = "invalid_behavior";
    return result;
  }

  std::normal_distribution<double> unit_normal(0.0, 1.0);
  const std::size_t sample_count = config_.rollout_count;
  const std::size_t horizon = config_.horizon_steps;
  std::vector<Control> perturbations(sample_count * horizon);
  std::vector<double> costs(sample_count, 0.0);
  std::vector<std::uint8_t> rollout_valid(sample_count, 0U);
  double minimum_cost = std::numeric_limits<double>::infinity();
  std::size_t best_rollout = 0U;
  const std::size_t requested_pure_noise_count = static_cast<std::size_t>(
    std::ceil(config_.pure_noise_fraction * static_cast<double>(sample_count)));
  const std::size_t rounded_pure_noise_count =
    ((requested_pure_noise_count + 3U) / 4U) * 4U;
  const std::size_t pure_noise_count = std::min(
    sample_count, std::max(std::min<std::size_t>(4U, sample_count),
    rounded_pure_noise_count));

  // Four-way antithetic sampling preserves exact left/right symmetry while keeping
  // acceleration exploration symmetric as well.
  for (std::size_t sample = 0U; sample < sample_count; sample += 4U) {
    for (std::size_t step = 0U; step < horizon; ++step) {
      const double steering_noise =
        request.exploration_scale * config_.steering_rate_stddev *
        unit_normal(random_generator_);
      const double acceleration_noise =
        request.exploration_scale * config_.acceleration_stddev *
        unit_normal(random_generator_);
      const std::array<Control, 4> group{{
        {steering_noise, acceleration_noise},
        {-steering_noise, acceleration_noise},
        {steering_noise, -acceleration_noise},
        {-steering_noise, -acceleration_noise}}};
      for (std::size_t member = 0U; member < group.size() && sample + member < sample_count;
        ++member)
      {
        perturbations[(sample + member) * horizon + step] = group[member];
      }
    }
  }

  #ifdef _OPENMP
  #pragma omp parallel for schedule(static)
  #endif
  for (std::ptrdiff_t sample_signed = 0;
    sample_signed < static_cast<std::ptrdiff_t>(sample_count); ++sample_signed)
  {
    const auto sample = static_cast<std::size_t>(sample_signed);
    std::vector<Control> rollout_controls(horizon);
    double importance_cost = 0.0;
    const bool pure_noise = sample < pure_noise_count;
    for (std::size_t step = 0U; step < horizon; ++step) {
      Control & noise = perturbations[sample * horizon + step];
      if (sample < 2U) {
        // Keep a symmetric pair of deterministic emergency-braking rollouts.
        rollout_controls[step] = Control{0.0, vehicle_.min_acceleration};
        noise.steering_rate =
          rollout_controls[step].steering_rate - nominal_controls_[step].steering_rate;
        noise.acceleration =
          rollout_controls[step].acceleration - nominal_controls_[step].acceleration;
      } else if (sample < 4U) {
        // A deterministic launch pair prevents the zero-speed local optimum:
        // independent Gaussian acceleration at every horizon step is very
        // unlikely to discover a smooth sustained launch from rest.
        rollout_controls[step] = Control{0.0, vehicle_.max_acceleration};
        noise.steering_rate = -nominal_controls_[step].steering_rate;
        noise.acceleration =
          rollout_controls[step].acceleration - nominal_controls_[step].acceleration;
      } else {
        const Control center = pure_noise ? Control{} : nominal_controls_[step];
        rollout_controls[step] = model_.clampControl(Control{
          center.steering_rate + noise.steering_rate,
          center.acceleration + noise.acceleration});
        if (pure_noise) {
          // MPPI updates the warm-start sequence, so express pure-noise samples
          // as a delta from that sequence rather than from zero.
          noise.steering_rate =
            rollout_controls[step].steering_rate - nominal_controls_[step].steering_rate;
          noise.acceleration =
            rollout_controls[step].acceleration - nominal_controls_[step].acceleration;
        }
      }
      const double steering_stddev =
        request.exploration_scale * config_.steering_rate_stddev;
      const double acceleration_stddev =
        request.exploration_scale * config_.acceleration_stddev;
      const double normalized_steering_noise = steering_stddev > kEpsilon ?
        noise.steering_rate / square(steering_stddev) : 0.0;
      const double normalized_acceleration_noise = acceleration_stddev > kEpsilon ?
        noise.acceleration / square(acceleration_stddev) : 0.0;
      importance_cost += config_.lambda *
        (nominal_controls_[step].steering_rate * normalized_steering_noise +
        nominal_controls_[step].acceleration * normalized_acceleration_noise);
    }
    const CostBreakdown rollout_cost = evaluateTrajectory(
      request.initial_state, rollout_controls, *request.race_line,
      request.distance_field, nullptr, request.obstacles, &request.behavior);
    if (finite(rollout_cost.total()) && rollout_cost.collision <= 0.0) {
      costs[sample] = rollout_cost.total() + importance_cost;
      rollout_valid[sample] = 1U;
    } else {
      costs[sample] = std::numeric_limits<double>::infinity();
    }
  }
  result.valid_rollouts = static_cast<std::size_t>(std::count(
      rollout_valid.begin(), rollout_valid.end(), static_cast<std::uint8_t>(1U)));
  for (std::size_t sample = 0U; sample < sample_count; ++sample) {
    if (finite(costs[sample]) && costs[sample] < minimum_cost) {
      minimum_cost = costs[sample];
      best_rollout = sample;
    }
  }
  if (!finite(minimum_cost)) {
    result.reason = "all_rollouts_invalid";
    return result;
  }

  std::vector<double> weights(sample_count, 0.0);
  double weight_sum = 0.0;
  for (std::size_t sample = 0U; sample < sample_count; ++sample) {
    const double exponent = std::clamp(
      -(costs[sample] - minimum_cost) / config_.lambda, -700.0, 0.0);
    weights[sample] = finite(costs[sample]) ? std::exp(exponent) : 0.0;
    weight_sum += weights[sample];
  }
  if (!finite(weight_sum) || weight_sum <= kEpsilon) {
    result.reason = "invalid_importance_weights";
    return result;
  }

  for (std::size_t step = 0U; step < horizon; ++step) {
    Control weighted_noise{};
    for (std::size_t sample = 0U; sample < sample_count; ++sample) {
      const double weight = weights[sample] / weight_sum;
      const Control & noise = perturbations[sample * horizon + step];
      weighted_noise.steering_rate += weight * noise.steering_rate;
      weighted_noise.acceleration += weight * noise.acceleration;
    }
    nominal_controls_[step] = model_.clampControl(Control{
      nominal_controls_[step].steering_rate + weighted_noise.steering_rate,
      nominal_controls_[step].acceleration + weighted_noise.acceleration});
  }

  const auto repair_duration = std::chrono::duration<double, std::milli>(
    config_.repair_budget_ms);
  const auto repair_deadline = std::chrono::steady_clock::now() +
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(repair_duration);
  if (!repairControls(
      request.initial_state, nominal_controls_, *request.race_line,
      request.distance_field, repair_deadline, request.obstacles))
  {
    result.reason = "repair_failed";
    result.solve_time_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - start_time).count();
    return result;
  }

  result.control_sequence = nominal_controls_;
  result.control = nominal_controls_.front();
  result.cost = evaluateTrajectory(
    request.initial_state, nominal_controls_, *request.race_line,
    request.distance_field, &result.predicted_states, request.obstacles,
    &request.behavior);
  result.metrics = trajectoryMetrics(
    request.initial_state, result.predicted_states, *request.race_line,
    request.distance_field, request.obstacles);
  const bool finite_cost = finite(result.cost.total());
  const bool complete_trajectory_safe = result.cost.collision <= 0.0;
  result.valid = finite_cost && complete_trajectory_safe;
  result.reason = !finite_cost ? "nonfinite_solution" :
    (complete_trajectory_safe ? "ok" : "final_trajectory_unsafe");
  result.best_rollout = best_rollout;
  result.solve_time_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - start_time).count();

  if (result.valid && nominal_controls_.size() > 1U) {
    std::move(nominal_controls_.begin() + 1, nominal_controls_.end(), nominal_controls_.begin());
    nominal_controls_.back() = nominal_controls_[nominal_controls_.size() - 2U];
  }
  return result;
}

std::string CpuMppiBackend::name() const
{
  return "cpu_reference";
}

std::unique_ptr<MppiBackend> makeCpuBackend(
  const MppiConfig & config, const VehicleConfig & vehicle)
{
  return std::make_unique<CpuMppiBackend>(config, vehicle);
}

}  // namespace mppi_controller
