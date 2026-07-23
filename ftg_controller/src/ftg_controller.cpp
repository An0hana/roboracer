#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <string>

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "ftg_controller/ftg_core.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/header.hpp"

namespace ftg_controller
{

class FTGNode : public rclcpp::Node
{
public:
  FTGNode()
  : Node("ftg_controller")
  {
    const std::string scan_topic = declare_parameter<std::string>("scan_topic", "/scan");
    const std::string drive_topic = declare_parameter<std::string>("drive_topic", "/drive");

    FTGConfig planner_config;
    planner_config.field_of_view_deg = positiveParameter("field_of_view_deg", 180.0);
    planner_config.max_lidar_range = positiveParameter("max_lidar_range", 10.0);
    planner_config.min_lidar_range = positiveParameter("min_lidar_range", 0.05);
    planner_config.smoothing_window = positiveIntegerParameter("smoothing_window", 5);
    planner_config.disparity_threshold = positiveParameter("disparity_threshold", 0.40);
    planner_config.safety_radius = positiveParameter("safety_radius", 0.35);
    planner_config.min_clearance = positiveParameter("min_clearance", 0.45);
    planner_config.min_gap_width = positiveParameter("min_gap_width", 0.65);
    planner_config.best_point_window = positiveIntegerParameter("best_point_window", 9);
    planner_config.candidate_count = positiveIntegerParameter("candidate_count", 21);
    planner_config.depth_weight = nonnegativeParameter("depth_weight", 0.35);
    planner_config.heading_weight = nonnegativeParameter("heading_weight", 0.25);
    planner_config.gap_center_weight = nonnegativeParameter("gap_center_weight", 0.65);
    planner_config.path_clearance_weight = nonnegativeParameter("path_clearance_weight", 0.75);
    planner_config.path_margin_weight = nonnegativeParameter("path_margin_weight", 1.0);
    planner_config.continuity_weight = nonnegativeParameter("continuity_weight", 0.5);
    planner_config.gap_switch_hysteresis = boundedParameter(
      "gap_switch_hysteresis", 0.10, 0.0, 1.0);
    planner_config.lidar_offset = nonnegativeParameter("lidar_offset", 0.275);
    planner_config.min_lookahead = positiveParameter("min_lookahead", 0.8);
    planner_config.max_lookahead = positiveParameter("max_lookahead", 2.0);
    if (planner_config.max_lookahead < planner_config.min_lookahead) {
      RCLCPP_WARN(
        get_logger(), "max_lookahead is smaller than min_lookahead; clamping it");
      planner_config.max_lookahead = planner_config.min_lookahead;
    }
    planner_config.wheelbase = positiveParameter("wheelbase", 0.33);
    planner_config.path_sweep_radius = positiveParameter(
      "path_sweep_radius", planner_config.safety_radius);
    planner_config.path_horizon = positiveParameter("path_horizon", 3.0);
    planner_config.emergency_distance = positiveParameter("emergency_distance", 0.45);
    planner_config.max_steering_angle = positiveParameter("max_steering_angle", 0.42);
    planner_ = std::make_unique<FollowTheGapPlanner>(planner_config);

    CommandConfig command_config;
    command_config.wheelbase = planner_config.wheelbase;
    command_config.max_steering_angle = planner_config.max_steering_angle;
    command_config.max_steering_rate = positiveParameter("max_steering_rate", 2.0);
    command_config.steering_time_constant = positiveParameter(
      "steering_time_constant", 0.12);
    command_config.max_speed = positiveParameter("max_speed", 4.0);
    command_config.max_lateral_accel = positiveParameter("max_lateral_accel", 4.0);
    command_config.emergency_distance = planner_config.emergency_distance;
    command_config.max_accel = positiveParameter("max_accel", 2.0);
    command_config.max_decel = positiveParameter("max_decel", 3.0);
    command_config.max_jerk = positiveParameter("max_jerk", 15.0);
    command_config.nominal_scan_period = positiveParameter("nominal_scan_period", 0.004);
    nominal_scan_period_ = command_config.nominal_scan_period;
    wheelbase_ = command_config.wheelbase;
    emergency_distance_ = command_config.emergency_distance;
    controller_ = std::make_unique<CommandController>(command_config);

    const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    subscriber_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic, qos, std::bind(&FTGNode::scanCallback, this, std::placeholders::_1));
    publisher_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic, 10);

    RCLCPP_INFO(
      get_logger(), "FTG started: %s -> %s, pure LiDAR, speed<=%.1f m/s",
      scan_topic.c_str(), drive_topic.c_str(), command_config.max_speed);
  }

  void publishStop()
  {
    const CommandResult command = controller_->emergencyStop();
    publishCommand(nullptr, command);
  }

private:
  double positiveParameter(const std::string & name, double default_value)
  {
    const double value = declare_parameter<double>(name, default_value);
    if (!std::isfinite(value) || value <= 0.0) {
      RCLCPP_WARN(get_logger(), "Invalid %s=%f; using %f", name.c_str(), value, default_value);
      return default_value;
    }
    return value;
  }

  double nonnegativeParameter(const std::string & name, double default_value)
  {
    const double value = declare_parameter<double>(name, default_value);
    if (!std::isfinite(value) || value < 0.0) {
      RCLCPP_WARN(get_logger(), "Invalid %s=%f; using %f", name.c_str(), value, default_value);
      return default_value;
    }
    return value;
  }

  int positiveIntegerParameter(const std::string & name, int default_value)
  {
    const auto value = declare_parameter<std::int64_t>(name, default_value);
    if (value <= 0 || value > std::numeric_limits<int>::max()) {
      RCLCPP_WARN(
        get_logger(), "Invalid %s=%ld; using %d", name.c_str(),
        static_cast<long>(value), default_value);
      return default_value;
    }
    return static_cast<int>(value);
  }

  double boundedParameter(
    const std::string & name, double default_value, double lower, double upper)
  {
    const double value = declare_parameter<double>(name, default_value);
    if (!std::isfinite(value)) {
      RCLCPP_WARN(get_logger(), "Invalid %s=%f; using %f", name.c_str(), value, default_value);
      return default_value;
    }
    return std::clamp(value, lower, upper);
  }

  double elapsedTime()
  {
    const std::int64_t now_ns = get_clock()->now().nanoseconds();
    double elapsed = nominal_scan_period_;
    if (last_update_ns_.has_value()) {
      elapsed = now_ns > *last_update_ns_ ?
        static_cast<double>(now_ns - *last_update_ns_) * 1e-9 : 0.0;
    }
    last_update_ns_ = now_ns;
    return std::min(elapsed, 0.20);
  }

  void publishCommand(
    const std_msgs::msg::Header * source_header, const CommandResult & command)
  {
    ackermann_msgs::msg::AckermannDriveStamped message;
    message.header.stamp = get_clock()->now();
    if (source_header != nullptr) {
      message.header.frame_id = source_header->frame_id;
    }
    message.drive.steering_angle = static_cast<float>(command.steering_angle);
    message.drive.speed = static_cast<float>(command.speed);
    publisher_->publish(message);
  }

  void logDecision(
    StopReason reason, const PlanResult * plan, const CommandResult & command)
  {
    const double target_degrees = plan != nullptr ? plan->target_angle * 180.0 / pi() : 0.0;
    const double path_clearance = plan != nullptr ? plan->path.collision_distance : 0.0;
    const bool retained = plan != nullptr && plan->retained_previous_target;
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "FTG: mode=%s target=%+.1fdeg steer=%+.3frad speed=%.2f "
      "path=%.2fm curve_limit=%.2f stop_limit=%.2f retained=%s",
      toString(reason), target_degrees, command.steering_angle, command.speed,
      path_clearance, command.curvature_speed_limit, command.stopping_speed_limit,
      retained ? "true" : "false");
  }

  void warnStopReason(StopReason reason)
  {
    if (reason == StopReason::kInvalidScan) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000, "Invalid or empty LaserScan; emergency stop");
    } else if (reason == StopReason::kNoGap) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000, "No traversable gap; controlled braking");
    } else if (reason == StopReason::kPathEmergency) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000, "Obstacle in swept path; emergency stop");
    }
  }

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    const double elapsed = elapsedTime();
    const ProcessedScan scan = planner_->preprocessScan(
      msg->ranges, msg->angle_min, msg->angle_increment, msg->range_min, msg->range_max);

    if (!scan.valid()) {
      planner_->reset();
      const CommandResult command = controller_->emergencyStop();
      publishCommand(&msg->header, command);
      warnStopReason(StopReason::kInvalidScan);
      logDecision(StopReason::kInvalidScan, nullptr, command);
      return;
    }

    const PlanResult plan = planner_->plan(scan);
    StopReason reason = plan.stop_reason;
    CommandResult command;

    if (plan.ok()) {
      command = controller_->update(plan.curvature, plan.path.collision_distance, elapsed);
      if (command.emergency) {
        reason = StopReason::kPathEmergency;
      }
    } else if (plan.stop_reason == StopReason::kNoGap) {
      const double current_curvature = std::tan(controller_->currentSteeringAngle()) / wheelbase_;
      const PathMetrics current_path = planner_->evaluatePath(
        scan.ranges, scan.angles, current_curvature);
      if (current_path.collision_distance <= emergency_distance_) {
        reason = StopReason::kPathEmergency;
        command = controller_->emergencyStop();
      } else {
        command = controller_->controlledStop(elapsed);
      }
    } else {
      command = controller_->emergencyStop();
    }

    publishCommand(&msg->header, command);
    if (reason != StopReason::kNone) {
      warnStopReason(reason);
    }
    logDecision(reason, plan.ok() ? &plan : nullptr, command);
  }

  static double pi()
  {
    static const double value = std::acos(-1.0);
    return value;
  }

  std::unique_ptr<FollowTheGapPlanner> planner_;
  std::unique_ptr<CommandController> controller_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_;
  double nominal_scan_period_{0.004};
  double wheelbase_{0.33};
  double emergency_distance_{0.45};
  std::optional<std::int64_t> last_update_ns_;
};

}  // namespace ftg_controller

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ftg_controller::FTGNode>();
  rclcpp::spin(node);
  if (rclcpp::ok()) {
    node->publishStop();
  }
  rclcpp::shutdown();
  return 0;
}
