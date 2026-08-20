#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "safety_controller/safety_core.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "roboracer_msgs/msg/race_state.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace safety_controller
{
namespace
{

ControllerMode parseMode(const std::string & value)
{
  if (value == "mppi") {
    return ControllerMode::kMppi;
  }
  if (value == "ftg") {
    return ControllerMode::kFtg;
  }
  throw std::invalid_argument("controller_mode must be 'mppi' or 'ftg'");
}

diagnostic_msgs::msg::KeyValue diagnosticValue(
  const std::string & key, const std::string & value)
{
  diagnostic_msgs::msg::KeyValue item;
  item.key = key;
  item.value = value;
  return item;
}

}  // namespace

class SafetyControllerNode : public rclcpp::Node
{
public:
  SafetyControllerNode()
  : Node("safety_controller")
  {
    const std::string scan_topic = declare_parameter<std::string>("scan_topic", "/scan");
    const std::string odom_topic = declare_parameter<std::string>(
      "odom_topic", "/state_estimation/odom");
    const std::string mppi_topic = declare_parameter<std::string>(
      "mppi_cmd_topic", "/control/mppi_cmd");
    const std::string ftg_topic = declare_parameter<std::string>(
      "ftg_cmd_topic", "/control/ftg_cmd");
    const std::string output_topic = declare_parameter<std::string>(
      "output_topic", "/ackermann_cmd");
    const std::string diagnostics_topic = declare_parameter<std::string>(
      "diagnostics_topic", "/diagnostics");
    const std::string race_state_topic = declare_parameter<std::string>(
      "race_state_topic", "/state_machine/state");
    output_frame_ = declare_parameter<std::string>("output_frame", "base_link");

    const std::string initial_mode_text = declare_parameter<std::string>(
      "controller_mode", "mppi");
    const double publish_rate = declare_parameter<double>("publish_rate_hz", 50.0);
    if (!std::isfinite(publish_rate) || publish_rate <= 0.0) {
      throw std::invalid_argument("publish_rate_hz must be positive");
    }

    SafetyConfig config;
    config.state_timeout = declare_parameter<double>("state_timeout", 0.100);
    config.scan_timeout = declare_parameter<double>("scan_timeout", 0.150);
    config.command_timeout = declare_parameter<double>("command_timeout", 0.100);
    config.race_state_timeout =
      declare_parameter<double>("race_state_timeout", 0.150);
    config.switch_speed_threshold = declare_parameter<double>(
      "switch_speed_threshold", 0.200);
    config.stop_steering_center_speed = declare_parameter<double>(
      "stop_steering_center_speed", 0.050);
    config.min_command_speed = declare_parameter<double>("min_command_speed", 0.0);
    config.max_command_speed = declare_parameter<double>("max_command_speed", 2.0);
    config.min_command_steering = declare_parameter<double>(
      "min_command_steering", -0.32);
    config.max_command_steering = declare_parameter<double>(
      "max_command_steering", 0.32);
    config.steering_response_time = declare_parameter<double>(
      "steering_response_time", 0.150);
    config.min_effective_steering_rate = declare_parameter<double>(
      "min_effective_steering_rate", -1.20);
    config.max_effective_steering_rate = declare_parameter<double>(
      "max_effective_steering_rate", 1.20);
    config.effective_steering_rate_speed_coefficient = declare_parameter<double>(
      "effective_steering_rate_speed_coefficient", 0.18);
    config.steering_effectiveness_at_zero_speed = declare_parameter<double>(
      "steering_effectiveness_at_zero_speed", 1.0);
    config.steering_effectiveness_speed_squared = declare_parameter<double>(
      "steering_effectiveness_speed_squared", 0.05);
    config.minimum_steering_effectiveness = declare_parameter<double>(
      "minimum_steering_effectiveness", 0.70);
    config.wheelbase = declare_parameter<double>("wheelbase", 0.324);
    config.vehicle_length = declare_parameter<double>("vehicle_length", 0.552);
    config.vehicle_width = declare_parameter<double>("vehicle_width", 0.320);
    config.rear_overhang = declare_parameter<double>("rear_overhang", 0.124);
    config.footprint_margin = declare_parameter<double>("footprint_margin", 0.050);
    config.lidar_offset_x = declare_parameter<double>("lidar_offset_x", 0.250);
    config.lidar_offset_y = declare_parameter<double>("lidar_offset_y", 0.0);
    config.self_filter_enabled = declare_parameter<bool>("self_filter_enabled", false);
    config.self_filter_min_x = declare_parameter<double>("self_filter_min_x", 0.0);
    config.self_filter_max_x = declare_parameter<double>("self_filter_max_x", 0.0);
    config.self_filter_min_y = declare_parameter<double>("self_filter_min_y", 0.0);
    config.self_filter_max_y = declare_parameter<double>("self_filter_max_y", 0.0);
    config.aeb_reaction_time = declare_parameter<double>("aeb_reaction_time", 0.100);
    config.aeb_max_deceleration = declare_parameter<double>("aeb_max_deceleration", 3.0);
    config.aeb_extra_distance = declare_parameter<double>("aeb_extra_distance", 0.150);
    config.aeb_max_sweep_distance = declare_parameter<double>(
      "aeb_max_sweep_distance", 3.0);
    config.aeb_sweep_step = declare_parameter<double>("aeb_sweep_step", 0.050);
    config.aeb_clear_hold_time = declare_parameter<double>("aeb_clear_hold_time", 0.200);
    config.aeb_release_extra_distance = declare_parameter<double>(
      "aeb_release_extra_distance", 0.100);
    config.aeb_release_check_speed = declare_parameter<double>(
      "aeb_release_check_speed", 1.500);
    config.aeb_resume_acceleration = declare_parameter<double>(
      "aeb_resume_acceleration", 1.000);
    config.aeb_max_latch_duration = declare_parameter<double>(
      "aeb_max_latch_duration", 5.0);
    config.aeb_debounce_duration = declare_parameter<double>(
      "aeb_debounce_duration", 0.50);
    config.aeb_soft_speed_limit = declare_parameter<double>(
      "aeb_soft_speed_limit", 0.80);
    config.aeb_steering_recovery_enabled = declare_parameter<bool>(
      "aeb_steering_recovery_enabled", true);
    config.aeb_steering_recovery_max_speed = declare_parameter<double>(
      "aeb_steering_recovery_max_speed", 0.100);
    config.aeb_steering_recovery_rate = declare_parameter<double>(
      "aeb_steering_recovery_rate", 0.800);
    config.aeb_steering_recovery_tolerance = declare_parameter<double>(
      "aeb_steering_recovery_tolerance", 0.020);
    config.scan_min_valid_fraction = declare_parameter<double>(
      "scan_min_valid_fraction", 0.50);
    config.recovery_rear_box_x_min =
      declare_parameter<double>("recovery_rear_box_x_min", -2.0);
    config.recovery_rear_box_x_max =
      declare_parameter<double>("recovery_rear_box_x_max", 0.10);
    config.recovery_rear_box_y =
      declare_parameter<double>("recovery_rear_box_y", 0.28);
    wheelbase_ = config.wheelbase;
    steering_estimation_min_speed_ = declare_parameter<double>(
      "steering_estimation_min_speed", 0.500);
    max_state_steering_angle_ = declare_parameter<double>(
      "max_state_steering_angle", 0.60);
    if (!std::isfinite(steering_estimation_min_speed_) ||
      steering_estimation_min_speed_ < 0.0 ||
      !std::isfinite(max_state_steering_angle_) || max_state_steering_angle_ <= 0.0)
    {
      throw std::invalid_argument("invalid steering-estimation parameters");
    }

    const ControllerMode initial_mode = parseMode(initial_mode_text);
    core_ = std::make_unique<SafetyCore>(config, initial_mode);

    output_publisher_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
      output_topic, rclcpp::QoS(10));
    diagnostics_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      diagnostics_topic, rclcpp::QoS(10));

    scan_subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic, rclcpp::SensorDataQoS(),
      std::bind(&SafetyControllerNode::scanCallback, this, std::placeholders::_1));
    odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, rclcpp::QoS(10),
      std::bind(&SafetyControllerNode::odomCallback, this, std::placeholders::_1));
    mppi_subscription_ = create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
      mppi_topic, rclcpp::QoS(10),
      [this](ackermann_msgs::msg::AckermannDriveStamped::ConstSharedPtr message) {
        commandCallback(ControllerMode::kMppi, *message);
      });
    ftg_subscription_ = create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
      ftg_topic, rclcpp::QoS(10),
      [this](ackermann_msgs::msg::AckermannDriveStamped::ConstSharedPtr message) {
        commandCallback(ControllerMode::kFtg, *message);
      });
    race_state_subscription_ =
      create_subscription<roboracer_msgs::msg::RaceState>(
      race_state_topic, rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
      std::bind(&SafetyControllerNode::raceStateCallback, this, std::placeholders::_1));

    parameter_callback_handle_ = add_on_set_parameters_callback(
      std::bind(&SafetyControllerNode::parametersCallback, this, std::placeholders::_1));

    const auto period = std::chrono::duration<double>(1.0 / publish_rate);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&SafetyControllerNode::timerCallback, this));

    RCLCPP_INFO(
      get_logger(), "Safety arbiter started: mode=%s output=%s rate=%.1f Hz",
      toString(initial_mode), output_topic.c_str(), publish_rate);
  }

private:
  double nowSeconds()
  {
    return static_cast<double>(get_clock()->now().nanoseconds()) * 1e-9;
  }

  double sourceTimeOrNow(const builtin_interfaces::msg::Time & stamp)
  {
    if (stamp.sec == 0 && stamp.nanosec == 0U) {
      return nowSeconds();
    }
    return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
  }

  void scanCallback(const sensor_msgs::msg::LaserScan::ConstSharedPtr message)
  {
    ScanData scan;
    scan.ranges.assign(message->ranges.begin(), message->ranges.end());
    scan.angle_min = message->angle_min;
    scan.angle_increment = message->angle_increment;
    scan.range_min = message->range_min;
    scan.range_max = message->range_max;
    std::lock_guard<std::mutex> lock(core_mutex_);
    core_->updateScan(scan, sourceTimeOrNow(message->header.stamp));
  }

  void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr message)
  {
    const double speed = message->twist.twist.linear.x;
    const double yaw_rate = message->twist.twist.angular.z;
    std::lock_guard<std::mutex> lock(core_mutex_);
    const bool steering_observed =
      std::isfinite(speed) && std::isfinite(yaw_rate) &&
      std::abs(speed) >= steering_estimation_min_speed_;
    double steering = core_->estimatedEffectiveSteeringAngle();
    if (steering_observed)
    {
      steering = std::clamp(
        std::atan(wheelbase_ * yaw_rate / speed),
        -max_state_steering_angle_, max_state_steering_angle_);
    }
    core_->updateState(
      speed, steering, sourceTimeOrNow(message->header.stamp),
      steering_observed);
  }

  void commandCallback(
    ControllerMode source,
    const ackermann_msgs::msg::AckermannDriveStamped & message)
  {
    const DriveCommand command{
      static_cast<double>(message.drive.speed),
      static_cast<double>(message.drive.steering_angle)};
    std::lock_guard<std::mutex> lock(core_mutex_);
    core_->updateCommand(source, command, sourceTimeOrNow(message.header.stamp));
  }

  void raceStateCallback(const roboracer_msgs::msg::RaceState::ConstSharedPtr message)
  {
    std::lock_guard<std::mutex> lock(core_mutex_);
    core_->updateRaceState(
      message->recovery_active,
      static_cast<double>(message->recovery_speed),
      static_cast<double>(message->recovery_steering),
      sourceTimeOrNow(message->header.stamp));
  }

  rcl_interfaces::msg::SetParametersResult parametersCallback(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    for (const auto & parameter : parameters) {
      if (parameter.get_name() != "controller_mode") {
        result.successful = false;
        result.reason = "safety_controller parameters are immutable after startup";
        return result;
      }
      try {
        const ControllerMode requested = parseMode(parameter.as_string());
        std::lock_guard<std::mutex> lock(core_mutex_);
        if (!core_->requestMode(requested, nowSeconds())) {
          result.successful = false;
          result.reason = "mode change requires fresh state and |speed| < switch_speed_threshold";
          return result;
        }
      } catch (const std::exception & error) {
        result.successful = false;
        result.reason = error.what();
        return result;
      }
    }
    return result;
  }

  void timerCallback()
  {
    ArbitrationResult result;
    {
      std::lock_guard<std::mutex> lock(core_mutex_);
      result = core_->evaluate(nowSeconds());
    }

    ackermann_msgs::msg::AckermannDriveStamped output;
    output.header.stamp = get_clock()->now();
    output.header.frame_id = output_frame_;
    output.drive.speed = static_cast<float>(result.command.speed);
    output.drive.steering_angle = static_cast<float>(result.command.steering_angle);
    output_publisher_->publish(output);
    publishDiagnostics(result, output.header.stamp);

    if (result.stopped()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Safety stop: reason=%s mode=%s state_age=%.3f scan_age=%.3f cmd_age=%.3f",
        toString(result.stop_reason), toString(result.selected_mode),
        result.state_age, result.scan_age, result.command_age);
    }
  }

  void publishDiagnostics(
    const ArbitrationResult & result, const builtin_interfaces::msg::Time & stamp)
  {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = stamp;
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "safety_controller/arbiter";
    status.hardware_id = "vehicle";
    status.level = result.stopped() ?
      diagnostic_msgs::msg::DiagnosticStatus::ERROR :
      diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = result.stopped() ? toString(result.stop_reason) : "active";
    status.values.push_back(diagnosticValue("mode", toString(result.selected_mode)));
    status.values.push_back(diagnosticValue("state_age_s", std::to_string(result.state_age)));
    status.values.push_back(diagnosticValue("scan_age_s", std::to_string(result.scan_age)));
    status.values.push_back(diagnosticValue("command_age_s", std::to_string(result.command_age)));
    status.values.push_back(
      diagnosticValue(
        "aeb_emergency", result.aeb.emergency ? "true" : "false"));
    status.values.push_back(
      diagnosticValue(
        "aeb_latched", result.aeb_latched ? "true" : "false"));
    status.values.push_back(
      diagnosticValue(
        "aeb_resume_active", result.aeb_resume_active ? "true" : "false"));
    status.values.push_back(
      diagnosticValue(
        "aeb_clear_duration_s", std::to_string(result.aeb_clear_duration)));
    status.values.push_back(
      diagnosticValue(
        "aeb_resume_speed_limit_mps", std::to_string(result.aeb_resume_speed_limit)));
    status.values.push_back(
      diagnosticValue(
        "aeb_steering_recovery_active",
        result.aeb_steering_recovery_active ? "true" : "false"));
    status.values.push_back(
      diagnosticValue(
        "aeb_steering_recovery_ready",
        result.aeb_steering_recovery_ready ? "true" : "false"));
    status.values.push_back(
      diagnosticValue(
        "aeb_steering_recovery_target_rad",
        std::to_string(result.aeb_steering_recovery_target)));
    status.values.push_back(
      diagnosticValue(
        "estimated_effective_steering_rad",
        std::to_string(result.estimated_effective_steering)));
    status.values.push_back(
      diagnosticValue(
        "aeb_self_filtered_beams", std::to_string(result.aeb.self_filtered_beams)));
    status.values.push_back(
      diagnosticValue(
        "collision_path_distance_m", std::to_string(result.aeb.collision_path_distance)));
    status.values.push_back(
      diagnosticValue(
        "output_speed_mps", std::to_string(result.command.speed)));
    status.values.push_back(
      diagnosticValue(
        "output_steering_rad", std::to_string(result.command.steering_angle)));
    array.status.push_back(std::move(status));
    diagnostics_publisher_->publish(array);
  }

  std::mutex core_mutex_;
  std::unique_ptr<SafetyCore> core_;
  double wheelbase_{0.324};
  double steering_estimation_min_speed_{0.500};
  double max_state_steering_angle_{0.60};
  std::string output_frame_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr output_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr
    mppi_subscription_;
  rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ftg_subscription_;
  rclcpp::Subscription<roboracer_msgs::msg::RaceState>::SharedPtr race_state_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
};

}  // namespace safety_controller

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<safety_controller::SafetyControllerNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("safety_controller"), "Startup failed: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
