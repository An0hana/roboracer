#ifndef SAFETY_CONTROLLER__SAFETY_CORE_HPP_
#define SAFETY_CONTROLLER__SAFETY_CORE_HPP_

#include <cstddef>
#include <limits>
#include <vector>

namespace safety_controller
{

enum class ControllerMode
{
  kMppi,
  kFtg,
};

enum class StopReason
{
  kNone,
  kAeb,
  kInvalidScan,
  kInvalidState,
  kStateTimeout,
  kScanTimeout,
  kCommandTimeout,
  kInvalidCommand,
};

const char * toString(ControllerMode mode);
const char * toString(StopReason reason);

struct DriveCommand
{
  double speed{0.0};
  double steering_angle{0.0};

  bool valid() const;
};

struct ScanData
{
  std::vector<double> ranges;
  double angle_min{0.0};
  double angle_increment{0.0};
  double range_min{0.0};
  double range_max{0.0};
};

struct SafetyConfig
{
  double state_timeout{0.100};
  double scan_timeout{0.150};
  double command_timeout{0.100};
  double switch_speed_threshold{0.200};
  double stop_steering_center_speed{0.050};

  // Independent command envelope. A controller output outside these bounds
  // is a fault, not a value for the hardware driver to clamp silently.
  double min_command_speed{0.0};
  double max_command_speed{2.0};
  double min_command_steering{-0.20};
  double max_command_steering{0.20};

  double wheelbase{0.324};
  double vehicle_length{0.552};
  double vehicle_width{0.320};
  double rear_overhang{0.124};
  double footprint_margin{0.050};
  double lidar_offset_x{0.250};
  double lidar_offset_y{0.0};
  bool self_filter_enabled{false};
  double self_filter_min_x{0.0};
  double self_filter_max_x{0.0};
  double self_filter_min_y{0.0};
  double self_filter_max_y{0.0};

  double aeb_reaction_time{0.100};
  double aeb_max_deceleration{3.0};
  double aeb_extra_distance{0.150};
  double aeb_max_sweep_distance{3.0};
  double aeb_sweep_step{0.050};
  double scan_min_valid_fraction{0.50};
};

struct AebAssessment
{
  bool scan_valid{false};
  bool emergency{false};
  std::size_t valid_beams{0U};
  std::size_t self_filtered_beams{0U};
  double collision_path_distance{std::numeric_limits<double>::infinity()};
  double sweep_distance{0.0};
};

struct ArbitrationResult
{
  DriveCommand command;
  StopReason stop_reason{StopReason::kNone};
  ControllerMode selected_mode{ControllerMode::kMppi};
  AebAssessment aeb;
  double state_age{std::numeric_limits<double>::infinity()};
  double scan_age{std::numeric_limits<double>::infinity()};
  double command_age{std::numeric_limits<double>::infinity()};

  bool stopped() const {return stop_reason != StopReason::kNone;}
};

/// ROS-independent safety arbiter. Times use one clock; missing data and time regressions fail closed.
class SafetyCore
{
public:
  explicit SafetyCore(
    const SafetyConfig & config = SafetyConfig{},
    ControllerMode initial_mode = ControllerMode::kMppi);

  void updateState(double speed, double steering_angle, double now_seconds);
  void updateScan(const ScanData & scan, double now_seconds);
  void updateCommand(
    ControllerMode source, const DriveCommand & command, double now_seconds);

  /// Returns false when a live mode change is unsafe. Selecting the current mode is idempotent.
  bool requestMode(ControllerMode requested, double now_seconds);

  ArbitrationResult evaluate(double now_seconds);

  ControllerMode selectedMode() const {return selected_mode_;}
  double currentSpeed() const {return current_speed_;}
  double currentSteeringAngle() const {return held_steering_angle_;}

private:
  struct TimedCommand
  {
    DriveCommand command;
    double stamp{0.0};
    bool received{false};
  };

  static double age(double now_seconds, double stamp, bool received);
  bool stateFresh(double now_seconds) const;
  bool scanStructurallyValid(const ScanData & scan, std::size_t * valid_beams) const;
  AebAssessment assessAeb() const;
  DriveCommand stopCommand() const;
  TimedCommand & commandFor(ControllerMode mode);
  const TimedCommand & commandFor(ControllerMode mode) const;

  SafetyConfig config_;
  ControllerMode selected_mode_;
  TimedCommand mppi_command_;
  TimedCommand ftg_command_;
  ScanData scan_;
  bool scan_received_{false};
  bool scan_valid_{false};
  std::size_t scan_valid_beams_{0U};
  double scan_stamp_{0.0};
  bool state_received_{false};
  bool state_valid_{false};
  double state_stamp_{0.0};
  double current_speed_{0.0};
  double held_steering_angle_{0.0};
};

}  // namespace safety_controller

#endif  // SAFETY_CONTROLLER__SAFETY_CORE_HPP_
