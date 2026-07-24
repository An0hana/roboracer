#include "opponent_tracker/multi_object_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <tuple>
#include <utility>
#include <vector>

namespace opponent_tracker
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

bool finiteNonnegative(double value)
{
  return std::isfinite(value) && value >= 0.0;
}

bool finitePositive(double value)
{
  return std::isfinite(value) && value > 0.0;
}

}  // namespace

MultiObjectTracker::MultiObjectTracker(MultiObjectTrackerConfig config)
: config_(std::move(config))
{
  if (!finitePositive(config_.association_distance) ||
    config_.min_confirmed_hits == 0U ||
    !finitePositive(config_.max_missed_time) ||
    !finiteNonnegative(config_.process_noise) ||
    !finitePositive(config_.measurement_noise) ||
    !finitePositive(config_.initial_velocity_stddev) ||
    !std::isfinite(config_.shape_smoothing) ||
    config_.shape_smoothing <= 0.0 || config_.shape_smoothing > 1.0 ||
    !finiteNonnegative(config_.dynamic_speed_threshold) ||
    !finiteNonnegative(config_.heading_exit_speed) ||
    !finitePositive(config_.heading_enter_speed) ||
    config_.heading_enter_speed <= config_.heading_exit_speed ||
    !finitePositive(config_.heading_smoothing) ||
    config_.heading_smoothing > 1.0 ||
    !finitePositive(config_.max_yaw_rate) ||
    !finitePositive(config_.max_prediction_dt))
  {
    throw std::invalid_argument("invalid multi-object tracker configuration");
  }
}

std::vector<TrackEstimate> MultiObjectTracker::update(
  const std::vector<Detection> & detections, double timestamp)
{
  if (!std::isfinite(timestamp)) {
    throw std::invalid_argument("tracker timestamp must be finite");
  }

  if (initialized_ && timestamp < last_timestamp_ - 1.0e-6) {
    reset();
  }

  const double elapsed = initialized_ ?
    std::max(0.0, timestamp - last_timestamp_) : 0.0;
  const double prediction_dt = std::min(elapsed, config_.max_prediction_dt);
  for (auto & track : tracks_) {
    predict(track, prediction_dt);
    track.visible = false;
  }

  std::vector<std::size_t> valid_detection_indices;
  valid_detection_indices.reserve(detections.size());
  for (std::size_t index = 0U; index < detections.size(); ++index) {
    if (validDetection(detections[index])) {
      valid_detection_indices.push_back(index);
    }
  }

  using Candidate = std::tuple<double, std::size_t, std::size_t>;
  std::vector<Candidate> candidates;
  candidates.reserve(tracks_.size() * valid_detection_indices.size());
  for (std::size_t track_index = 0U; track_index < tracks_.size(); ++track_index) {
    const auto & track = tracks_[track_index];
    for (const auto detection_index : valid_detection_indices) {
      const auto & detection = detections[detection_index];
      const double distance = std::hypot(
        detection.x - track.x.position,
        detection.y - track.y.position);
      if (distance <= config_.association_distance) {
        candidates.emplace_back(distance, track_index, detection_index);
      }
    }
  }
  std::sort(candidates.begin(), candidates.end());

  std::vector<bool> track_matched(tracks_.size(), false);
  std::vector<bool> detection_matched(detections.size(), false);
  for (const auto & candidate : candidates) {
    const auto track_index = std::get<1>(candidate);
    const auto detection_index = std::get<2>(candidate);
    if (track_matched[track_index] || detection_matched[detection_index]) {
      continue;
    }
    updateTrack(tracks_[track_index], detections[detection_index], elapsed);
    track_matched[track_index] = true;
    detection_matched[detection_index] = true;
  }

  for (std::size_t index = 0U; index < tracks_.size(); ++index) {
    auto & track = tracks_[index];
    if (track_matched[index]) {
      track.missed_time = 0.0;
      track.visible = true;
      ++track.hit_count;
      if (track.hit_count >= config_.min_confirmed_hits) {
        track.confirmed = true;
      }
    } else {
      track.missed_time += elapsed;
    }
  }

  for (const auto detection_index : valid_detection_indices) {
    if (!detection_matched[detection_index]) {
      tracks_.push_back(createTrack(detections[detection_index]));
    }
  }

  tracks_.erase(
    std::remove_if(
      tracks_.begin(), tracks_.end(),
      [this](const Track & track) {
        return track.missed_time > config_.max_missed_time;
      }),
    tracks_.end());

  initialized_ = true;
  last_timestamp_ = timestamp;
  return estimates();
}

void MultiObjectTracker::reset()
{
  tracks_.clear();
  next_id_ = 1;
  last_timestamp_ = 0.0;
  initialized_ = false;
}

bool MultiObjectTracker::shiftTrackPosition(
  std::int32_t id, double dx, double dy)
{
  if (!std::isfinite(dx) || !std::isfinite(dy)) {
    return false;
  }
  for (auto & track : tracks_) {
    if (track.id == id) {
      track.x.position += dx;
      track.y.position += dy;
      return true;
    }
  }
  return false;
}

void MultiObjectTracker::predict(Track & track, double dt) const
{
  const double process_variance = config_.process_noise * config_.process_noise;
  predictAxis(track.x, dt, process_variance);
  predictAxis(track.y, dt, process_variance);
}

void MultiObjectTracker::updateTrack(
  Track & track, const Detection & detection, double elapsed) const
{
  const double measurement_variance =
    config_.measurement_noise * config_.measurement_noise;
  correctAxis(track.x, detection.x, measurement_variance);
  correctAxis(track.y, detection.y, measurement_variance);

  const double alpha = config_.shape_smoothing;
  track.yaw = normalizeAngle(
    track.yaw + alpha * rectangleAngleDifference(detection.yaw, track.yaw));
  track.length += alpha * (detection.length - track.length);
  track.width += alpha * (detection.width - track.width);
  track.confidence += alpha * (detection.confidence - track.confidence);
  updateMotionYaw(track, elapsed);
}

void MultiObjectTracker::updateMotionYaw(Track & track, double elapsed) const
{
  const double speed = std::hypot(track.x.velocity, track.y.velocity);
  if (track.use_motion_yaw) {
    if (speed < config_.heading_exit_speed) {
      track.use_motion_yaw = false;
      return;
    }
  } else {
    if (speed < config_.heading_enter_speed) {
      return;
    }
    track.use_motion_yaw = true;
  }

  const double measured_yaw =
    std::atan2(track.y.velocity, track.x.velocity);
  if (!track.motion_yaw_initialized) {
    track.motion_yaw = measured_yaw;
    track.motion_yaw_initialized = true;
    return;
  }

  const double filtered_difference =
    config_.heading_smoothing *
    normalizeAngle(measured_yaw - track.motion_yaw);
  const double max_step =
    config_.max_yaw_rate * std::max(elapsed, 1.0e-3);
  track.motion_yaw = normalizeAngle(
    track.motion_yaw +
    std::clamp(filtered_difference, -max_step, max_step));
}

MultiObjectTracker::Track MultiObjectTracker::createTrack(
  const Detection & detection)
{
  const double position_variance =
    config_.measurement_noise * config_.measurement_noise;
  const double velocity_variance =
    config_.initial_velocity_stddev * config_.initial_velocity_stddev;

  Track track;
  track.id = next_id_++;
  track.x.position = detection.x;
  track.x.covariance_pp = position_variance;
  track.x.covariance_vv = velocity_variance;
  track.y.position = detection.y;
  track.y.covariance_pp = position_variance;
  track.y.covariance_vv = velocity_variance;
  track.yaw = normalizeAngle(detection.yaw);
  track.length = detection.length;
  track.width = detection.width;
  track.confidence = detection.confidence;
  track.hit_count = 1U;
  track.confirmed = config_.min_confirmed_hits <= 1U;
  track.visible = true;
  return track;
}

std::vector<TrackEstimate> MultiObjectTracker::estimates() const
{
  std::vector<TrackEstimate> output;
  output.reserve(tracks_.size());
  for (const auto & track : tracks_) {
    if (!track.confirmed) {
      continue;
    }
    TrackEstimate estimate;
    estimate.id = track.id;
    estimate.x = track.x.position;
    estimate.y = track.y.position;
    estimate.yaw =
      track.motion_yaw_initialized ? track.motion_yaw : track.yaw;
    estimate.vx = track.x.velocity;
    estimate.vy = track.y.velocity;
    estimate.length = track.length;
    estimate.width = track.width;
    const double visibility_scale = track.visible ? 1.0 :
      std::max(0.0, 1.0 - track.missed_time / config_.max_missed_time);
    estimate.confidence = std::clamp(
      track.confidence * visibility_scale, 0.0, 1.0);
    estimate.dynamic =
      std::hypot(estimate.vx, estimate.vy) >= config_.dynamic_speed_threshold;
    estimate.visible = track.visible;
    output.push_back(estimate);
  }
  std::sort(
    output.begin(), output.end(),
    [](const TrackEstimate & left, const TrackEstimate & right) {
      return left.id < right.id;
    });
  return output;
}

void MultiObjectTracker::predictAxis(
  AxisFilter & axis, double dt, double process_variance)
{
  axis.position += dt * axis.velocity;

  const double old_pp = axis.covariance_pp;
  const double old_pv = axis.covariance_pv;
  const double old_vp = axis.covariance_vp;
  const double old_vv = axis.covariance_vv;
  const double dt_squared = dt * dt;
  const double dt_cubed = dt_squared * dt;
  const double dt_fourth = dt_squared * dt_squared;

  axis.covariance_pp =
    old_pp + dt * (old_pv + old_vp) + dt_squared * old_vv +
    0.25 * dt_fourth * process_variance;
  axis.covariance_pv =
    old_pv + dt * old_vv + 0.5 * dt_cubed * process_variance;
  axis.covariance_vp =
    old_vp + dt * old_vv + 0.5 * dt_cubed * process_variance;
  axis.covariance_vv =
    old_vv + dt_squared * process_variance;
}

void MultiObjectTracker::correctAxis(
  AxisFilter & axis, double measurement, double measurement_variance)
{
  const double innovation_variance =
    axis.covariance_pp + measurement_variance;
  if (!finitePositive(innovation_variance)) {
    return;
  }

  const double gain_position =
    axis.covariance_pp / innovation_variance;
  const double gain_velocity =
    axis.covariance_vp / innovation_variance;
  const double innovation = measurement - axis.position;

  const double old_pp = axis.covariance_pp;
  const double old_pv = axis.covariance_pv;
  const double old_vp = axis.covariance_vp;
  const double old_vv = axis.covariance_vv;

  axis.position += gain_position * innovation;
  axis.velocity += gain_velocity * innovation;
  axis.covariance_pp = (1.0 - gain_position) * old_pp;
  axis.covariance_pv = (1.0 - gain_position) * old_pv;
  axis.covariance_vp = old_vp - gain_velocity * old_pp;
  axis.covariance_vv = old_vv - gain_velocity * old_pv;

  const double symmetric_cross_covariance =
    0.5 * (axis.covariance_pv + axis.covariance_vp);
  axis.covariance_pv = symmetric_cross_covariance;
  axis.covariance_vp = symmetric_cross_covariance;
}

double MultiObjectTracker::normalizeAngle(double angle)
{
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

double MultiObjectTracker::rectangleAngleDifference(
  double target, double current)
{
  double difference = normalizeAngle(target - current);
  while (difference > 0.5 * kPi) {
    difference -= kPi;
  }
  while (difference < -0.5 * kPi) {
    difference += kPi;
  }
  return difference;
}

bool MultiObjectTracker::validDetection(const Detection & detection)
{
  return std::isfinite(detection.x) &&
         std::isfinite(detection.y) &&
         std::isfinite(detection.yaw) &&
         finitePositive(detection.length) &&
         finitePositive(detection.width) &&
         finiteNonnegative(detection.confidence);
}

}  // namespace opponent_tracker
