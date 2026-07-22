#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "race_safety/safety_core.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

namespace race_safety
{

class VescCommandAdapterNode : public rclcpp::Node
{
public:
  VescCommandAdapterNode()
  : Node("vesc_command_adapter")
  {
    const bool enabled = declare_parameter<bool>("enabled", false);
    const std::string input_topic = declare_parameter<std::string>(
      "input_topic", "/ackermann_cmd");
    const std::string motor_topic = declare_parameter<std::string>(
      "motor_speed_topic", "/commands/motor/speed");
    const std::string servo_topic = declare_parameter<std::string>(
      "servo_position_topic", "/commands/servo/position");
    const double publish_rate = declare_parameter<double>("publish_rate_hz", 50.0);
    command_timeout_ = declare_parameter<double>("command_timeout", 0.100);

    VescMappingConfig config;
    config.speed_to_erpm_gain = declare_parameter<double>("speed_to_erpm_gain", 4614.0);
    config.speed_to_erpm_offset = declare_parameter<double>("speed_to_erpm_offset", 0.0);
    config.steering_center = declare_parameter<double>("steering_center", 0.50);
    config.steering_left_gain = declare_parameter<double>("steering_left_gain", -1.13);
    config.steering_right_gain = declare_parameter<double>("steering_right_gain", -1.52);
    config.servo_min = declare_parameter<double>("servo_min", 0.15);
    config.servo_max = declare_parameter<double>("servo_max", 0.85);
    config.erpm_min = declare_parameter<double>("erpm_min", -23250.0);
    config.erpm_max = declare_parameter<double>("erpm_max", 23250.0);
    mapper_ = std::make_unique<VescCommandMapper>(config);

    if (!std::isfinite(publish_rate) || publish_rate <= 0.0 ||
      !std::isfinite(command_timeout_) || command_timeout_ <= 0.0)
    {
      throw std::invalid_argument("adapter publish rate and timeout must be positive");
    }

    if (!enabled) {
      RCLCPP_INFO(
        get_logger(),
        "VESC direct adapter is disabled; set vesc_command_adapter.ros__parameters.enabled=true to use it");
      return;
    }

    motor_publisher_ = create_publisher<std_msgs::msg::Float64>(motor_topic, rclcpp::QoS(10));
    servo_publisher_ = create_publisher<std_msgs::msg::Float64>(servo_topic, rclcpp::QoS(10));
    command_subscription_ =
      create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
      input_topic, rclcpp::QoS(10),
      std::bind(&VescCommandAdapterNode::commandCallback, this, std::placeholders::_1));
    const auto period = std::chrono::duration<double>(1.0 / publish_rate);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&VescCommandAdapterNode::timerCallback, this));

    RCLCPP_WARN(
      get_logger(),
      "VESC direct adapter ENABLED: %s -> [%s, %s]. Do not run ackermann_to_vesc concurrently.",
      input_topic.c_str(), motor_topic.c_str(), servo_topic.c_str());
  }

private:
  double nowSeconds()
  {
    return static_cast<double>(get_clock()->now().nanoseconds()) * 1e-9;
  }

  void commandCallback(
    const ackermann_msgs::msg::AckermannDriveStamped::ConstSharedPtr message)
  {
    const DriveCommand command{
      static_cast<double>(message->drive.speed),
      static_cast<double>(message->drive.steering_angle)};
    std::lock_guard<std::mutex> lock(mutex_);
    latest_output_ = mapper_->map(command);
    if (std::isfinite(command.steering_angle)) {
      latest_steering_angle_ = command.steering_angle;
    }
    command_stamp_ = nowSeconds();
    command_received_ = true;
  }

  void timerCallback()
  {
    VescOutput output;
    bool stale = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const double now = nowSeconds();
      stale = !command_received_ || !std::isfinite(command_stamp_) ||
        now < command_stamp_ || now - command_stamp_ > command_timeout_ ||
        !latest_output_.valid;
      output = stale ? mapper_->stop(latest_steering_angle_) : latest_output_;
    }

    std_msgs::msg::Float64 motor;
    motor.data = output.erpm;
    motor_publisher_->publish(motor);
    std_msgs::msg::Float64 servo;
    servo.data = output.servo_position;
    servo_publisher_->publish(servo);

    if (stale) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "VESC adapter command timeout/invalid command: publishing zero ERPM");
    }
  }

  std::mutex mutex_;
  std::unique_ptr<VescCommandMapper> mapper_;
  double command_timeout_{0.100};
  double command_stamp_{0.0};
  double latest_steering_angle_{0.0};
  bool command_received_{false};
  VescOutput latest_output_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr motor_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr servo_publisher_;
  rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr
    command_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace race_safety

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<race_safety::VescCommandAdapterNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(
      rclcpp::get_logger("vesc_command_adapter"), "Startup failed: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
