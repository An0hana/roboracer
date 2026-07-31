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
    !finitePositive(config_.steering_response_time) ||
    !std::isfinite(config_.min_effective_steering_rate) ||
    !std::isfinite(config_.max_effective_steering_rate) ||
    config_.min_effective_steering_rate >= 0.0 ||
    config_.max_effective_steering_rate <= 0.0 ||
    !finiteNonnegative(config_.effective_steering_rate_speed_coefficient) ||
    !finitePositive(config_.steering_effectiveness_at_zero_speed) ||
    config_.steering_effectiveness_at_zero_speed > 1.0 ||
    !finiteNonnegative(config_.steering_effectiveness_speed_squared) ||
    !finitePositive(config_.minimum_steering_effectiveness) ||
    config_.minimum_steering_effectiveness >
    config_.steering_effectiveness_at_zero_speed ||
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
    !finiteNonnegative(config_.aeb_clear_hold_time) ||
    !finiteNonnegative(config_.aeb_release_extra_distance) ||
    !finiteNonnegative(config_.aeb_release_check_speed) ||
    !finitePositive(config_.aeb_resume_acceleration) ||
    !finiteNonnegative(config_.aeb_max_latch_duration) ||
    !finiteNonnegative(config_.aeb_steering_recovery_max_speed) ||
    !finitePositive(config_.aeb_steering_recovery_rate) ||
    !finiteNonnegative(config_.aeb_steering_recovery_tolerance) ||
    !std::isfinite(config_.scan_min_valid_fraction) ||
    config_.scan_min_valid_fraction < 0.0 || config_.scan_min_valid_fraction > 1.0)
  {
    throw std::invalid_argument("invalid SafetyConfig");
  }
}

void SafetyCore::updateState(
  double speed, double steering_angle, double now_seconds,
  bool steering_observed)
{
  state_received_ = true;
  state_stamp_ = now_seconds;
  state_valid_ = std::isfinite(speed) &&
    (!steering_observed || std::isfinite(steering_angle)) &&
    std::isfinite(now_seconds);
  if (state_valid_) {
    current_speed_ = speed;
    if (steering_observed) {
      measured_steering_angle_ = steering_angle;
      estimated_effective_steering_angle_ = std::clamp(
        steering_angle,
        config_.min_command_steering, config_.max_command_steering);
    }
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

  double elapsed = 0.0;
  if (last_evaluation_time_.has_value() &&
    std::isfinite(now_seconds) && now_seconds >= *last_evaluation_time_)
  {
    elapsed = now_seconds - *last_evaluation_time_;
  }
  estimated_effective_steering_angle_ = advanceEffectiveSteering(
    estimated_effective_steering_angle_, output_steering_angle_,
    current_speed_, elapsed);
  const double aeb_steering_target =
    selected_command.received && selected_command.command.valid() ?
    std::clamp(
    selected_command.command.steering_angle,
    config_.min_command_steering, config_.max_command_steering) :
    output_steering_angle_;
  result.aeb = assessAeb(
    current_speed_, aeb_steering_target, estimated_effective_steering_angle_);

  const bool release_inputs_fresh =
    state_valid_ && scan_valid_ &&
    result.state_age <= config_.state_timeout &&
    result.scan_age <= config_.scan_timeout &&
    selected_command.received &&
    result.command_age <= config_.command_timeout &&
    selected_command.command.valid();
  const bool was_latched = aeb_latched_;
  if (result.aeb.emergency) {
    aeb_latched_ = true;
    aeb_resume_active_ = false;
    aeb_clear_since_.reset();
    aeb_resume_speed_limit_ = 0.0;
    if (!was_latched) {
      aeb_recovery_steering_angle_ = estimated_effective_steering_angle_;
      aeb_latch_start_ = now_seconds;
    }
  }

  bool recovery_active = false;
  bool recovery_ready = false;
  double recovery_target = measured_steering_angle_;
  if (aeb_latched_ && release_inputs_fresh) {
    recovery_target = std::clamp(
      selected_command.command.steering_angle,
      config_.min_command_steering, config_.max_command_steering);
    if (!config_.aeb_steering_recovery_enabled) {
      aeb_recovery_steering_angle_ = recovery_target;
    } else if (
      std::abs(current_speed_) <= config_.aeb_steering_recovery_max_speed)
    {
      recovery_active = true;
      const double maximum_change =
        config_.aeb_steering_recovery_rate * std::max(0.0, elapsed);
      aeb_recovery_steering_angle_ += std::clamp(
        recovery_target - aeb_recovery_steering_angle_,
        -maximum_change, maximum_change);
    } else if (std::abs(current_speed_) >
      config_.aeb_steering_recovery_max_speed)
    {
      // During braking retain the measured curvature; do not ask the servo
      // for an abrupt steering transient while lateral force is still high.
      aeb_recovery_steering_angle_ = estimated_effective_steering_angle_;
    }
    aeb_recovery_steering_angle_ = std::clamp(
      aeb_recovery_steering_angle_,
      config_.min_command_steering, config_.max_command_steering);
    recovery_ready = !config_.aeb_steering_recovery_enabled ||
      std::abs(
      estimated_effective_steering_angle_ -
      effectiveSteeringTarget(recovery_target, current_speed_)) <=
      config_.aeb_steering_recovery_tolerance;
  }

  if (!result.aeb.emergency && aeb_latched_) {
    // The release assessment must check whether the forward corridor is
    // clear at a speed the vehicle can actually reach.  Using the full
    // commanded speed (capped at aeb_release_check_speed) when the car is
    // stationary is physically unrealistic: the vehicle would need to
    // accelerate from rest first, and the resume ramp already enforces a
    // gradual speed increase after release.
    //
    // Cap the release speed at what the vehicle could attain within one
    // clear-hold window under the resume acceleration, but never below the
    // measured speed or above the (capped) commanded speed.
    const double resume_cap = std::abs(current_speed_) +
      config_.aeb_resume_acceleration * config_.aeb_clear_hold_time;
    const double release_speed = release_inputs_fresh ?
      std::max(std::abs(current_speed_),
               std::min({resume_cap,
                         std::abs(selected_command.command.speed),
                         config_.aeb_release_check_speed})) :
      std::abs(current_speed_);
    const double release_steering_command = release_inputs_fresh ?
      aeb_recovery_steering_angle_ : measured_steering_angle_;
    const AebAssessment release_assessment = assessAeb(
      release_speed, release_steering_command,
      estimated_effective_steering_angle_, config_.aeb_release_extra_distance);

    // Enforce a hard upper bound on how long AEB may stay latched.  A
    // false-positive trigger in a narrow corridor can otherwise persist
    // indefinitely when the release corridor remains obstructed by the
    // same static geometry that caused the original trigger.
    bool latch_timed_out = false;
    if (config_.aeb_max_latch_duration > 0.0 &&
      aeb_latch_start_.has_value() &&
      std::isfinite(now_seconds))
    {
      const double latch_duration = now_seconds - *aeb_latch_start_;
      if (latch_duration >= config_.aeb_max_latch_duration) {
        latch_timed_out = true;
      }
    }

    const bool mppi_planning_ok = selected_command.received &&
      selected_command.command.valid() &&
      selected_command.command.speed > 0.1;
    if (!release_inputs_fresh || (!latch_timed_out && !recovery_ready) ||
      (!latch_timed_out && !mppi_planning_ok && release_assessment.emergency))
    {
      aeb_clear_since_.reset();
    } else {
      if (!aeb_clear_since_.has_value()) {
        aeb_clear_since_ = now_seconds;
      }
      const double clear_duration = now_seconds - *aeb_clear_since_;
      if (std::isfinite(clear_duration) &&
        clear_duration >= config_.aeb_clear_hold_time)
      {
        aeb_latched_ = false;
        aeb_resume_active_ = true;
        aeb_resume_speed_limit_ = 0.0;
        aeb_clear_since_.reset();
        aeb_latch_start_.reset();
      }
    }
  }
  result.aeb_latched = aeb_latched_;
  result.aeb_resume_active = aeb_resume_active_;
  result.aeb_clear_duration = aeb_clear_since_.has_value() ?
    std::max(0.0, now_seconds - *aeb_clear_since_) : 0.0;
  result.aeb_resume_speed_limit = aeb_resume_speed_limit_;
  result.aeb_steering_recovery_active = recovery_active;
  result.aeb_steering_recovery_ready = recovery_ready;
  result.aeb_steering_recovery_target = recovery_target;
  result.estimated_effective_steering = estimated_effective_steering_angle_;

  // Physical and sensing hazards always override controller selection.
  if (scan_received_ && !scan_valid_) {
    result.stop_reason = StopReason::kInvalidScan;
  } else if (aeb_latched_) {
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
    result.command = stopCommand(result.stop_reason);
    if (result.stop_reason == StopReason::kAeb &&
      selected_command.received && selected_command.command.valid()) {
      result.command.steering_angle = std::clamp(
        selected_command.command.steering_angle,
        config_.min_command_steering, config_.max_command_steering);
    }
    output_steering_angle_ = result.command.steering_angle;
  } else {
    result.command = selected_command.command;
    result.command.speed = std::clamp(
      result.command.speed, config_.min_command_speed, config_.max_command_speed);
    result.command.steering_angle = std::clamp(
      result.command.steering_angle,
      config_.min_command_steering, config_.max_command_steering);
    if (aeb_resume_active_) {
      aeb_resume_speed_limit_ = std::min(
        config_.max_command_speed,
        aeb_resume_speed_limit_ + config_.aeb_resume_acceleration * elapsed);
      result.command.speed = std::min(result.command.speed, aeb_resume_speed_limit_);
      result.aeb_resume_speed_limit = aeb_resume_speed_limit_;
      // Do not end the ramp merely because the upstream controller happens
      // to publish zero during the AEB braking transient. Keep the limiter
      // active until it has traversed the configured command envelope, so a
      // later full-speed command cannot bypass the release ramp.
      if (aeb_resume_speed_limit_ >=
        config_.max_command_speed - kCommandEnvelopeTolerance)
      {
        aeb_resume_active_ = false;
        aeb_resume_speed_limit_ = std::numeric_limits<double>::infinity();
      }
      result.aeb_resume_active = aeb_resume_active_;
    }
    output_steering_angle_ = result.command.steering_angle;
  }
  last_evaluation_time_ = now_seconds;
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

double SafetyCore::steeringEffectiveness(double speed) const
{
  const double squared_speed = std::isfinite(speed) ? speed * speed : 0.0;
  return std::clamp(
    config_.steering_effectiveness_at_zero_speed -
    config_.steering_effectiveness_speed_squared * squared_speed,
    config_.minimum_steering_effectiveness,
    config_.steering_effectiveness_at_zero_speed);
}

double SafetyCore::effectiveSteeringTarget(
  double steering_command, double speed) const
{
  if (!std::isfinite(steering_command)) {
    return 0.0;
  }
  return std::clamp(
    steeringEffectiveness(speed) * steering_command,
    config_.min_command_steering, config_.max_command_steering);
}

double SafetyCore::effectiveSteeringRateLimit(double speed) const
{
  const double squared_speed = std::isfinite(speed) ? speed * speed : 0.0;
  return 1.0 / (
    1.0 + config_.effective_steering_rate_speed_coefficient * squared_speed);
}

double SafetyCore::advanceEffectiveSteering(
  double steering, double steering_command, double speed, double elapsed) const
{
  if (!std::isfinite(steering) || !std::isfinite(elapsed) || elapsed <= 0.0) {
    return std::clamp(
      std::isfinite(steering) ? steering : 0.0,
      config_.min_command_steering, config_.max_command_steering);
  }
  const double error = effectiveSteeringTarget(steering_command, speed) - steering;
  const double scale = effectiveSteeringRateLimit(speed);
  const double rate = std::clamp(
    error / config_.steering_response_time,
    config_.min_effective_steering_rate * scale,
    config_.max_effective_steering_rate * scale);
  const double requested_change = rate * elapsed;
  const double bounded_change = std::clamp(
    requested_change, std::min(0.0, error), std::max(0.0, error));
  return std::clamp(
    steering + bounded_change,
    config_.min_command_steering, config_.max_command_steering);
}

AebAssessment SafetyCore::assessAeb(
  double requested_speed, double steering_command,
  double initial_effective_steering, double extra_distance) const
{
  AebAssessment assessment;
  assessment.scan_valid = scan_received_ && scan_valid_;
  assessment.valid_beams = scan_valid_beams_;
  if (!assessment.scan_valid) {
    return assessment;
  }

  const double speed = std::abs(requested_speed);
  assessment.sweep_distance = std::min(
    config_.aeb_max_sweep_distance,
    config_.aeb_reaction_time * speed +
    speed * speed / (2.0 * config_.aeb_max_deceleration) +
    config_.aeb_extra_distance + extra_distance);

  const double direction = requested_speed < 0.0 ? -1.0 : 1.0;
  const std::size_t sample_count = std::max<std::size_t>(
    1U, static_cast<std::size_t>(std::ceil(
      assessment.sweep_distance / config_.aeb_sweep_step)));
  struct SweepPose
  {
    double x;
    double y;
    double yaw;
    double distance;
  };
  std::vector<SweepPose> sweep;
  sweep.reserve(sample_count + 1U);
  sweep.push_back(SweepPose{0.0, 0.0, 0.0, 0.0});
  const double step_distance = assessment.sweep_distance /
    static_cast<double>(sample_count);
  const double integration_speed = std::max(speed, 0.05);
  double effective_steering = std::clamp(
    initial_effective_steering,
    config_.min_command_steering, config_.max_command_steering);
  for (std::size_t sample = 1U; sample <= sample_count; ++sample) {
    const SweepPose & previous = sweep.back();
    const double elapsed = speed > 1.0e-6 ?
      step_distance / integration_speed : 0.0;
    const double next_effective_steering = advanceEffectiveSteering(
      effective_steering, steering_command, requested_speed, elapsed);
    const double mean_steering =
      0.5 * (effective_steering + next_effective_steering);
    const double curvature = std::tan(mean_steering) / config_.wheelbase;
    const double signed_step = direction * step_distance;
    const double yaw_step = curvature * signed_step;
    const double middle_yaw = previous.yaw + 0.5 * yaw_step;
    sweep.push_back(SweepPose{
      previous.x + signed_step * std::cos(middle_yaw),
      previous.y + signed_step * std::sin(middle_yaw),
      previous.yaw + yaw_step,
      step_distance * static_cast<double>(sample)});
    effective_steering = next_effective_steering;
  }
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

    for (const SweepPose & pose : sweep) {
      const double dx = point_x - pose.x;
      const double dy = point_y - pose.y;
      const double cosine = std::cos(pose.yaw);
      const double sine = std::sin(pose.yaw);
      const double local_x = cosine * dx + sine * dy;
      const double local_y = -sine * dx + cosine * dy;
      if (local_x >= min_x && local_x <= max_x && std::abs(local_y) <= max_abs_y) {
        assessment.emergency = true;
        assessment.collision_path_distance = std::min(
          assessment.collision_path_distance, pose.distance);
        break;
      }
    }
  }
  return assessment;
}

DriveCommand SafetyCore::stopCommand(StopReason reason) const
{
  double steering = measured_steering_angle_;
  if (reason == StopReason::kAeb && config_.aeb_steering_recovery_enabled) {
    steering = aeb_recovery_steering_angle_;
  } else if (std::abs(current_speed_) <= config_.stop_steering_center_speed) {
    steering = 0.0;
  }
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
