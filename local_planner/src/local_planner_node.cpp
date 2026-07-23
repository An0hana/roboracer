#include <functional>
#include <memory>
#include <string>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "roboracer_msgs/msg/tracked_obstacle_array.hpp"
#include "roboracer_msgs/msg/trajectory.hpp"

namespace local_planner
{

class LocalPlannerNode : public rclcpp::Node
{
public:
  LocalPlannerNode()
  : Node("local_planner")
  {
    const auto obstacles_topic = declare_parameter<std::string>(
      "obstacles_topic", "/perception/obstacles");
    const auto odom_topic = declare_parameter<std::string>(
      "odom_topic", "/state_estimation/odom");
    const auto output_topic = declare_parameter<std::string>(
      "output_topic", "/planner/local_trajectory");
    trajectory_source_ = declare_parameter<std::string>(
      "trajectory_source", "local_planner_placeholder");

    publisher_ = create_publisher<roboracer_msgs::msg::Trajectory>(
      output_topic, rclcpp::QoS(rclcpp::KeepLast(1)).reliable());

    obstacles_subscription_ =
      create_subscription<roboracer_msgs::msg::TrackedObstacleArray>(
      obstacles_topic, rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
      std::bind(&LocalPlannerNode::obstaclesCallback, this, std::placeholders::_1));

    odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, rclcpp::QoS(rclcpp::KeepLast(5)).reliable(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr message) {
        latest_odom_frame_ = message->header.frame_id;
      });

    RCLCPP_INFO(
      get_logger(), "Placeholder started: %s + %s -> %s",
      obstacles_topic.c_str(), odom_topic.c_str(), output_topic.c_str());
  }

private:
  void obstaclesCallback(
    const roboracer_msgs::msg::TrackedObstacleArray::ConstSharedPtr message)
  {
    roboracer_msgs::msg::Trajectory output;
    output.header = message->header;
    output.source = trajectory_source_;
    output.valid = false;
    publisher_->publish(output);
  }

  std::string trajectory_source_;
  std::string latest_odom_frame_;
  rclcpp::Publisher<roboracer_msgs::msg::Trajectory>::SharedPtr publisher_;
  rclcpp::Subscription<roboracer_msgs::msg::TrackedObstacleArray>::SharedPtr
    obstacles_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
};

}  // namespace local_planner

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<local_planner::LocalPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
