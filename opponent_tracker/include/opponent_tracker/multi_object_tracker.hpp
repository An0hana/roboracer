#ifndef OPPONENT_TRACKER__MULTI_OBJECT_TRACKER_HPP_
#define OPPONENT_TRACKER__MULTI_OBJECT_TRACKER_HPP_

#include <cstddef>
#include <cstdint>
#include <vector>

#include "opponent_tracker/scan_clusterer.hpp"

namespace opponent_tracker
{

struct MultiObjectTrackerConfig
{
  double association_distance{0.75};
  std::size_t min_confirmed_hits{3U};
  double max_missed_time{0.30};
  double process_noise{4.0};
  double measurement_noise{0.04};
  double initial_velocity_stddev{2.0};
  double shape_smoothing{0.25};
  double dynamic_speed_threshold{0.15};
  double heading_enter_speed{0.18};
  double heading_exit_speed{0.08};
  double heading_smoothing{0.35};
  double max_yaw_rate{2.5};
  double max_prediction_dt{0.10};
};

struct TrackEstimate
{
  std::int32_t id{-1};
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  double vx{0.0};
  double vy{0.0};
  double length{0.0};
  double width{0.0};
  double confidence{0.0};
  bool dynamic{false};
  bool visible{false};
};

class MultiObjectTracker
{
public:
  explicit MultiObjectTracker(MultiObjectTrackerConfig config);

  std::vector<TrackEstimate> update(
    const std::vector<Detection> & detections, double timestamp);

  std::vector<TrackEstimate> estimates() const;

  bool shiftTrackPosition(std::int32_t id, double dx, double dy);

  void reset();

private:
  struct AxisFilter
  {
    double position{0.0};
    double velocity{0.0};
    double covariance_pp{0.0};
    double covariance_pv{0.0};
    double covariance_vp{0.0};
    double covariance_vv{0.0};
  };

  struct Track
  {
    std::int32_t id{-1};
    AxisFilter x;
    AxisFilter y;
    double yaw{0.0};
    double length{0.0};
    double width{0.0};
    double confidence{0.0};
    double missed_time{0.0};
    double motion_yaw{0.0};
    std::size_t hit_count{0U};
    bool confirmed{false};
    bool visible{false};
    bool motion_yaw_initialized{false};
    bool use_motion_yaw{false};
  };

  void predict(Track & track, double dt) const;
  void updateTrack(
    Track & track, const Detection & detection, double elapsed) const;
  void updateMotionYaw(Track & track, double elapsed) const;
  Track createTrack(const Detection & detection);
  static void predictAxis(AxisFilter & axis, double dt, double process_variance);
  static void correctAxis(
    AxisFilter & axis, double measurement, double measurement_variance);
  static double normalizeAngle(double angle);
  static double rectangleAngleDifference(double target, double current);
  static bool validDetection(const Detection & detection);

  MultiObjectTrackerConfig config_;
  std::vector<Track> tracks_;
  std::int32_t next_id_{1};
  double last_timestamp_{0.0};
  bool initialized_{false};
};

}  // namespace opponent_tracker

#endif  // OPPONENT_TRACKER__MULTI_OBJECT_TRACKER_HPP_
