#ifndef FTG_CONTROLLER__FTG_CORE_HPP_
#define FTG_CONTROLLER__FTG_CORE_HPP_

#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <vector>

namespace ftg_controller
{

enum class StopReason
{
  kNone,
  kInvalidScan,
  kNoGap,
  kPathEmergency,
};

const char * toString(StopReason reason) noexcept;

struct FTGConfig
{
  double field_of_view_deg{180.0};
  double max_lidar_range{10.0};
  double min_lidar_range{0.05};
  int smoothing_window{5};
  double disparity_threshold{0.40};
  double safety_radius{0.35};
  double min_clearance{0.45};
  double min_gap_width{0.65};
  int best_point_window{9};
  int candidate_count{21};

  double depth_weight{0.35};
  double heading_weight{0.25};
  double gap_center_weight{0.65};
  double path_clearance_weight{0.75};
  double path_margin_weight{1.0};
  double continuity_weight{0.5};
  double gap_switch_hysteresis{0.10};

  double lidar_offset{0.250};
  double min_lookahead{0.8};
  double max_lookahead{2.0};
  double wheelbase{0.324};
  double max_steering_angle{0.42};
  double path_sweep_radius{0.35};
  double path_horizon{3.0};
  double emergency_distance{0.45};
};

struct ProcessedScan
{
  std::vector<double> ranges;
  std::vector<double> angles;
  double angle_increment{0.0};

  bool valid() const noexcept;
};

struct Gap
{
  std::size_t begin{0};
  std::size_t end{0};  // One past the final index.
  double angular_width{0.0};
  double physical_width{0.0};
};

struct PathMetrics
{
  double collision_distance{std::numeric_limits<double>::infinity()};
  double lateral_margin{std::numeric_limits<double>::infinity()};
};

struct PlanResult
{
  StopReason stop_reason{StopReason::kInvalidScan};
  std::size_t target_index{0};
  double target_angle{0.0};
  double target_range{0.0};
  double target_bearing{0.0};
  double lookahead{0.0};
  double curvature{0.0};
  double steering_angle{0.0};
  double score{0.0};
  PathMetrics path;
  Gap gap;
  bool retained_previous_target{false};

  bool ok() const noexcept {return stop_reason == StopReason::kNone;}
};

class FollowTheGapPlanner
{
public:
  explicit FollowTheGapPlanner(FTGConfig config = {});

  const FTGConfig & config() const noexcept {return config_;}

  ProcessedScan preprocessScan(
    const std::vector<float> & input_ranges,
    double angle_min,
    double angle_increment,
    double sensor_range_min,
    double sensor_range_max) const;

  std::vector<std::uint8_t> createTraversabilityMask(
    const std::vector<double> & ranges, double angle_increment) const;

  std::vector<Gap> findGaps(
    const std::vector<double> & ranges,
    const std::vector<std::uint8_t> & traversable,
    double angle_increment) const;

  PathMetrics evaluatePath(
    const std::vector<double> & ranges,
    const std::vector<double> & angles,
    double curvature) const;

  PlanResult plan(const ProcessedScan & scan);
  void reset() noexcept;
  std::optional<double> previousTargetAngle() const noexcept {return previous_target_angle_;}

private:
  FTGConfig config_;
  std::optional<double> previous_target_angle_;
};

struct CommandConfig
{
  double wheelbase{0.324};
  double max_steering_angle{0.42};
  double max_steering_rate{2.0};
  double steering_time_constant{0.12};
  double max_speed{4.0};
  double max_lateral_accel{4.0};
  double emergency_distance{0.45};
  double max_accel{2.0};
  double max_decel{3.0};
  double max_jerk{15.0};
  double nominal_scan_period{0.004};
};

struct CommandResult
{
  double steering_angle{0.0};
  double speed{0.0};
  double acceleration{0.0};
  double curvature_speed_limit{0.0};
  double stopping_speed_limit{0.0};
  double requested_speed{0.0};
  bool emergency{false};
};

class CommandController
{
public:
  explicit CommandController(CommandConfig config = {});

  const CommandConfig & config() const noexcept {return config_;}
  CommandResult update(double requested_curvature, double path_collision_distance, double dt);
  CommandResult controlledStop(double dt);
  CommandResult emergencyStop() noexcept;
  void reset(double speed = 0.0, double steering_angle = 0.0) noexcept;

  double currentSpeed() const noexcept {return current_speed_;}
  double currentSteeringAngle() const noexcept {return current_steering_angle_;}
  double currentAcceleration() const noexcept {return current_acceleration_;}

private:
  double validDt(double dt) const noexcept;
  double updateSteering(double requested_curvature, double dt) noexcept;
  double limitSpeed(double requested_speed, double dt) noexcept;
  CommandResult makeResult(
    double curvature_limit, double stopping_limit, double requested_speed,
    bool emergency) const noexcept;

  CommandConfig config_;
  double current_speed_{0.0};
  double current_steering_angle_{0.0};
  double current_acceleration_{0.0};
};

}  // namespace ftg_controller

#endif  // FTG_CONTROLLER__FTG_CORE_HPP_
