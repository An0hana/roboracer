#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "roboracer_msgs/msg/race_state.hpp"
#include "roboracer_msgs/msg/tracked_obstacle_array.hpp"

namespace race_manager
{

class RaceManagerNode : public rclcpp::Node
{
public:
  RaceManagerNode()
  : Node("race_manager")
  {
    const auto obstacles_topic = declare_parameter<std::string>(
      "obstacles_topic", "/perception/obstacles");
    const auto odom_topic = declare_parameter<std::string>(
      "odom_topic", "/state_estimation/odom");
    const auto output_topic = declare_parameter<std::string>(
      "output_topic", "/race_manager/state");
    const double publish_rate = declare_parameter<double>("publish_rate_hz", 10.0);
    frame_id_ = declare_parameter<std::string>("frame_id", "map");

    if (!std::isfinite(publish_rate) || publish_rate <= 0.0) {
      throw std::invalid_argument("publish_rate_hz must be positive");
    }

    publisher_ = create_publisher<roboracer_msgs::msg::RaceState>(
      output_topic, rclcpp::QoS(rclcpp::KeepLast(1)).reliable());

    obstacles_subscription_ =
      create_subscription<roboracer_msgs::msg::TrackedObstacleArray>(
      obstacles_topic, rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
      [this](roboracer_msgs::msg::TrackedObstacleArray::ConstSharedPtr) {
        received_obstacles_ = true;
      });

    odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, rclcpp::QoS(rclcpp::KeepLast(5)).reliable(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr) {
        received_odom_ = true;
      });

    const auto period = std::chrono::duration<double>(1.0 / publish_rate);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&RaceManagerNode::publishState, this));

    RCLCPP_INFO(
      get_logger(), "Placeholder started: %s + %s -> %s",
      obstacles_topic.c_str(), odom_topic.c_str(), output_topic.c_str());
  }

private:
  void publishState()
  {
    roboracer_msgs::msg::RaceState output;
    output.header.stamp = get_clock()->now();
    output.header.frame_id = frame_id_;
    output.state = roboracer_msgs::msg::RaceState::INIT;
    output.reason =
      received_obstacles_ && received_odom_ ? "placeholder_ready" : "waiting_for_inputs";
    publisher_->publish(output);
  }

  bool received_obstacles_{false};
  bool received_odom_{false};
  std::string frame_id_;
  rclcpp::Publisher<roboracer_msgs::msg::RaceState>::SharedPtr publisher_;
  rclcpp::Subscription<roboracer_msgs::msg::TrackedObstacleArray>::SharedPtr
    obstacles_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace race_manager

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<race_manager::RaceManagerNode>());
  rclcpp::shutdown();
  return 0;
}
