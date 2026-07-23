#include <functional>
#include <memory>
#include <string>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "roboracer_msgs/msg/tracked_obstacle_array.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace opponent_tracker
{

class OpponentTrackerNode : public rclcpp::Node
{
public:
  OpponentTrackerNode()
  : Node("opponent_tracker")
  {
    const auto scan_topic = declare_parameter<std::string>("scan_topic", "/scan");
    const auto odom_topic = declare_parameter<std::string>(
      "odom_topic", "/state_estimation/odom");
    const auto output_topic = declare_parameter<std::string>(
      "output_topic", "/perception/obstacles");

    publisher_ = create_publisher<roboracer_msgs::msg::TrackedObstacleArray>(
      output_topic, rclcpp::QoS(rclcpp::KeepLast(1)).reliable());

    scan_subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic, rclcpp::SensorDataQoS(),
      std::bind(&OpponentTrackerNode::scanCallback, this, std::placeholders::_1));

    odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, rclcpp::QoS(rclcpp::KeepLast(5)).reliable(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr message) {
        latest_odom_frame_ = message->header.frame_id;
      });

    RCLCPP_INFO(
      get_logger(), "Placeholder started: %s + %s -> %s",
      scan_topic.c_str(), odom_topic.c_str(), output_topic.c_str());
  }

private:
  void scanCallback(const sensor_msgs::msg::LaserScan::ConstSharedPtr message)
  {
    roboracer_msgs::msg::TrackedObstacleArray output;
    output.header = message->header;
    publisher_->publish(output);
  }

  std::string latest_odom_frame_;
  rclcpp::Publisher<roboracer_msgs::msg::TrackedObstacleArray>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
};

}  // namespace opponent_tracker

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<opponent_tracker::OpponentTrackerNode>());
  rclcpp::shutdown();
  return 0;
}
