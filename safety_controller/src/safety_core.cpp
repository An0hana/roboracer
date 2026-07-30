#include "safety_controller/safety_core.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace safety_controller
{
namespace
{

// AckermannDrive fields are float32. A configured double boundary such as 0.20
// round-trips through the ROS message as 0.20000000298, so allow only enough
// tolerance to absorb wire-format rounding and clamp accepted values below.
constexpr double kCommandEnvelopeTolerance = 1.0e-6;

bool finitePositive(double value)
{
  return std::isfinite(value) && value > 0.0;
}

bool finiteNonnegative(double value)
{
  return std::isfinite(value) && value >= 0.0;
}

}  // namespace

const char * toString(ControllerMode mode)
{
  switch (mode) {
    case ControllerMode::kMppi:
      return "mppi";
    case ControllerMode::kFtg:
      return "ftg";
  }
  return "unknown";
}

const char * toString(StopReason reason)
{
  switch (reason) {
    case StopReason::kNone:
      return "none";
    case StopReason::kAeb:
      return "aeb";
    case StopReason::kInvalidScan:
      return "invalid_scan";
    case StopReason::kInvalidState:
      return "invalid_state";
    case StopReason::kStateTimeout:
      return "state_timeout";
    case StopReason::kScanTimeout:
      return "scan_timeout";
    case StopReason::kCommandTimeout:
      return "command_timeout";
    case StopReason::kInvalidCommand:
      return "invalid_command";
  }
  return "unknown";
}

bool DriveCommand::valid() const
{
  return std::isfinite(speed) && std::isfinite(steering_angle);
}

SafetyCore::SafetyCore(const SafetyConfig & config, ControllerMode initial_mode)
: config_(config), selected_mode_(initial_mode)
{
  if (!finitePositive(config_.state_timeout) || !finitePositive(config_.scan_timeout) ||
    !finitePositive(config_.command_timeout) ||
    !finiteNonnegative(config_.switch_speed_threshold) ||
    !finiteNonnegative(config_.stop_steering_center_speed) ||
    !std::isfinite(config_.min_command_speed) ||
    !std::isfinite(config_.max_command_speed) ||
    config_.min_command_speed > config_.max_command_speed ||
    !std::isfinite(config_.min_command_steering) ||
    !std::isfinite(config_.max_command_steering) ||
    config_.min_command_steering >= config_.max_command_steering ||
    !finitePositive(config_.wheelbase) || !finitePositive(config_.vehicle_length) ||
    !finitePositive(config_.vehicle_width) || !finiteNonnegative(config_.rear_overhang) ||
    config_.rear_overhang >= config_.vehicle_length ||
    !finiteNonnegative(config_.footprint_margin) ||
    !std::isfinite(config_.lidar_offset_x) || !std::isfinite(config_.lidar_offset_y) ||
    (config_.self_filter_enabled &&
    (!std::isfinite(config_.self_filter_min_x) ||
    !std::isfinite(config_.self_filter_max_x) ||
    !std::isfinite(config_.self_filter_min_y) ||
    !std::isfinite(config_.self_filter_max_y) ||
    config_.self_filter_min_x >= config_.self_filter_max_x ||
    config_.self_filter_min_y >= config_.self_filter_max_y)) ||
    !finiteNonnegative(config_.aeb_reaction_time) ||
    !finitePositive(config_.aeb_max_deceleration) ||
    !finiteNonnegative(config_.aeb_extra_distance) ||
    !finitePositive(config_.aeb_max_sweep_distance) ||
    !finitePositive(config_.aeb_sweep_step) ||
    !std::isfinite(config_.scan_min_valid_fraction) ||
    config_.scan_min_valid_fraction < 0.0 || config_.scan_min_valid_fraction > 1.0)
  {
    throw std::invalid_argument("invalid SafetyConfig");
  }
}

void SafetyCore::updateState(double speed, double steering_angle, double now_seconds)
{
  state_received_ = true;
  state_stamp_ = now_seconds;
  state_valid_ = std::isfinite(speed) && std::isfinite(steering_angle) &&
    std::isfinite(now_seconds);
  if (state_valid_) {
    current_speed_ = speed;
    held_steering_angle_ = steering_angle;
  }
}

void SafetyCore::updateScan(const ScanData & scan, double now_seconds)
{
  scan_received_ = true;
  scan_stamp_ = now_seconds;
  scan_ = scan;
  scan_valid_ = std::isfinite(now_seconds) &&
    scanStructurallyValid(scan_, &scan_valid_beams_);
}

void SafetyCore::updateCommand(
  ControllerMode source, const DriveCommand & command, double now_seconds)
{
  TimedCommand & target = commandFor(source);
  target.command = command;
  target.stamp = now_seconds;
  target.received = true;
}

bool SafetyCore::requestMode(ControllerMode requested, double now_seconds)
{
  if (requested == selected_mode_) {
    return true;
  }
  if (!stateFresh(now_seconds) ||
    std::abs(current_speed_) >= config_.switch_speed_threshold)
  {
    return false;
  }
  selected_mode_ = requested;
  return true;
}

ArbitrationResult SafetyCore::evaluate(double now_seconds)
{
  ArbitrationResult result;
  result.selected_mode = selected_mode_;
  result.state_age = age(now_seconds, state_stamp_, state_received_);
  result.scan_age = age(now_seconds, scan_stamp_, scan_received_);

  const TimedCommand & selected_command = commandFor(selected_mode_);
  result.command_age = age(now_seconds, selected_command.stamp, selected_command.received);
  result.aeb = assessAeb();

  // Physical and sensing hazards always override controller selection.
  if (scan_received_ && !scan_valid_) {
    result.stop_reason = StopReason::kInvalidScan;
  } else if (result.aeb.emergency) {
    result.stop_reason = StopReason::kAeb;
  } else if (state_received_ && !state_valid_) {
    result.stop_reason = StopReason::kInvalidState;
  } else if (result.state_age > config_.state_timeout) {
    result.stop_reason = StopReason::kStateTimeout;
  } else if (result.scan_age > config_.scan_timeout) {
    result.stop_reason = StopReason::kScanTimeout;
  } else if (!selected_command.received || result.command_age > config_.command_timeout) {
    result.stop_reason = StopReason::kCommandTimeout;
  } else if (!selected_command.command.valid() ||
    selected_command.command.speed <
    config_.min_command_speed - kCommandEnvelopeTolerance ||
    selected_command.command.speed >
    config_.max_command_speed + kCommandEnvelopeTolerance ||
    selected_command.command.steering_angle <
    config_.min_command_steering - kCommandEnvelopeTolerance ||
    selected_command.command.steering_angle >
    config_.max_command_steering + kCommandEnvelopeTolerance)
  {
    result.stop_reason = StopReason::kInvalidCommand;
  }

  if (result.stopped()) {
    result.command = stopCommand();
    // Keep the measured steering while the vehicle is still moving so an
    // emergency stop cannot introduce an abrupt lateral transient. Once
    // stationary, center the wheels and update the latch so a noisy low-speed
    // yaw-rate estimate cannot hold AEB on a stale curved sweep forever.
    held_steering_angle_ = result.command.steering_angle;
  } else {
    result.command = selected_command.command;
    result.command.speed = std::clamp(
      result.command.speed, config_.min_command_speed, config_.max_command_speed);
    result.command.steering_angle = std::clamp(
      result.command.steering_angle,
      config_.min_command_steering, config_.max_command_steering);
    held_steering_angle_ = result.command.steering_angle;
  }
  return result;
}

double SafetyCore::age(double now_seconds, double stamp, bool received)
{
  if (!received || !std::isfinite(now_seconds) || !std::isfinite(stamp) || now_seconds < stamp) {
    return std::numeric_limits<double>::infinity();
  }
  return now_seconds - stamp;
}

bool SafetyCore::stateFresh(double now_seconds) const
{
  return state_valid_ && age(now_seconds, state_stamp_, state_received_) <= config_.state_timeout;
}

bool SafetyCore::scanStructurallyValid(
  const ScanData & scan, std::size_t * valid_beams) const
{
  *valid_beams = 0U;
  if (scan.ranges.empty() || !std::isfinite(scan.angle_min) ||
    !std::isfinite(scan.angle_increment) || scan.angle_increment == 0.0 ||
    !finiteNonnegative(scan.range_min) || !finitePositive(scan.range_max) ||
    scan.range_max <= scan.range_min)
  {
    return false;
  }

  for (const double range : scan.ranges) {
    // LaserScan values below range_min are invalid measurements, not nearby
    // obstacles. Positive infinity is the conventional clear/no-return value.
    if ((std::isfinite(range) && range >= scan.range_min && range <= scan.range_max) ||
      (std::isinf(range) && range > 0.0))
    {
      ++(*valid_beams);
    }
  }

  const double valid_fraction = static_cast<double>(*valid_beams) /
    static_cast<double>(scan.ranges.size());
  return valid_fraction >= config_.scan_min_valid_fraction;
}

AebAssessment SafetyCore::assessAeb() const
{
  AebAssessment assessment;
  assessment.scan_valid = scan_received_ && scan_valid_;
  assessment.valid_beams = scan_valid_beams_;
  if (!assessment.scan_valid) {
    return assessment;
  }

  const double speed = std::abs(current_speed_);
  assessment.sweep_distance = std::min(
    config_.aeb_max_sweep_distance,
    config_.aeb_reaction_time * speed +
    speed * speed / (2.0 * config_.aeb_max_deceleration) +
    config_.aeb_extra_distance);

  const double direction = current_speed_ < 0.0 ? -1.0 : 1.0;
  const double curvature = std::tan(held_steering_angle_) / config_.wheelbase;
  const std::size_t sample_count = std::max<std::size_t>(
    1U, static_cast<std::size_t>(std::ceil(
      assessment.sweep_distance / config_.aeb_sweep_step)));
  const double front_overhang = config_.vehicle_length - config_.rear_overhang;
  const double min_x = -config_.rear_overhang - config_.footprint_margin;
  const double max_x = front_overhang + config_.footprint_margin;
  const double max_abs_y = 0.5 * config_.vehicle_width + config_.footprint_margin;

  for (std::size_t beam = 0U; beam < scan_.ranges.size(); ++beam) {
    const double measured_range = scan_.ranges[beam];
    if (!std::isfinite(measured_range) || measured_range > scan_.range_max ||
      measured_range < scan_.range_min)
    {
      continue;
    }

    const double range = measured_range;
    const double angle = scan_.angle_min +
      static_cast<double>(beam) * scan_.angle_increment;
    const double point_x = config_.lidar_offset_x + range * std::cos(angle);
    const double point_y = config_.lidar_offset_y + range * std::sin(angle);

    // A scanner mounted inside the vehicle can see fixed bodywork or its
    // protective bracket. Such points are physically inside the vehicle and
    // must not be interpreted as an external obstacle at path distance zero.
    if (config_.self_filter_enabled &&
      point_x >= config_.self_filter_min_x &&
      point_x <= config_.self_filter_max_x &&
      point_y >= config_.self_filter_min_y &&
      point_y <= config_.self_filter_max_y)
    {
      ++assessment.self_filtered_beams;
      continue;
    }

    for (std::size_t sample = 0U; sample <= sample_count; ++sample) {
      const double distance = assessment.sweep_distance *
        static_cast<double>(sample) / static_cast<double>(sample_count);
      const double signed_distance = direction * distance;

      double path_x = signed_distance;
      double path_y = 0.0;
      double path_yaw = 0.0;
      if (std::abs(curvature) > 1e-8) {
        path_yaw = curvature * signed_distance;
        path_x = std::sin(path_yaw) / curvature;
        path_y = (1.0 - std::cos(path_yaw)) / curvature;
      }

      const double dx = point_x - path_x;
      const double dy = point_y - path_y;
      const double cosine = std::cos(path_yaw);
      const double sine = std::sin(path_yaw);
      const double local_x = cosine * dx + sine * dy;
      const double local_y = -sine * dx + cosine * dy;
      if (local_x >= min_x && local_x <= max_x && std::abs(local_y) <= max_abs_y) {
        assessment.emergency = true;
        assessment.collision_path_distance = std::min(
          assessment.collision_path_distance, distance);
        break;
      }
    }
  }
  return assessment;
}

DriveCommand SafetyCore::stopCommand() const
{
  const double steering = std::abs(current_speed_) <=
    config_.stop_steering_center_speed ? 0.0 : held_steering_angle_;
  return DriveCommand{
    0.0,
    std::clamp(
      steering, config_.min_command_steering, config_.max_command_steering)};
}

SafetyCore::TimedCommand & SafetyCore::commandFor(ControllerMode mode)
{
  return mode == ControllerMode::kMppi ? mppi_command_ : ftg_command_;
}

const SafetyCore::TimedCommand & SafetyCore::commandFor(ControllerMode mode) const
{
  return mode == ControllerMode::kMppi ? mppi_command_ : ftg_command_;
}

}  // namespace safety_controller
