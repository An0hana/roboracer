#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <string>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace race_bringup
{

class StateEstimatorBridge : public rclcpp::Node
{
public:
  StateEstimatorBridge()
  : Node("state_estimator_bridge"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    const auto velocity_topic = declare_parameter<std::string>(
      "velocity_odom_topic", "/odometry/filtered");
    const auto output_topic = declare_parameter<std::string>(
      "output_topic", "/state_estimation/odom");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    stale_timeout_ = declare_parameter<double>("stale_timeout", 0.10);
    const double publish_rate = declare_parameter<double>("publish_rate", 50.0);

    velocity_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      velocity_topic, rclcpp::QoS(20),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr message) {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_velocity_ = std::move(message);
      });
    state_pub_ = create_publisher<nav_msgs::msg::Odometry>(output_topic, rclcpp::QoS(20));

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, publish_rate));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&StateEstimatorBridge::publishState, this));

    RCLCPP_INFO(
      get_logger(), "Publishing map-frame state on %s at %.1f Hz",
      output_topic.c_str(), publish_rate);
  }

private:
  void publishState()
  {
    nav_msgs::msg::Odometry::ConstSharedPtr velocity;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      velocity = latest_velocity_;
    }

    if (!velocity) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Waiting for filtered odometry");
      return;
    }

    const rclcpp::Time now = get_clock()->now();
    const rclcpp::Time velocity_stamp(
      velocity->header.stamp, get_clock()->get_clock_type());
    const double age = (now - velocity_stamp).seconds();
    if (!std::isfinite(age) || age < -0.05 || age > stale_timeout_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Filtered odometry is stale (age %.3f s)", age);
      return;
    }

    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_.lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException & exception) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Waiting for %s -> %s transform: %s",
        map_frame_.c_str(), base_frame_.c_str(), exception.what());
      return;
    }

    const rclcpp::Time transform_stamp(
      transform.header.stamp, get_clock()->get_clock_type());
    const double transform_age = (now - transform_stamp).seconds();
    if (transform_stamp.nanoseconds() == 0 || !std::isfinite(transform_age) ||
      transform_age < -0.05 || transform_age > stale_timeout_)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Localization transform is stale (age %.3f s)", transform_age);
      return;
    }

    nav_msgs::msg::Odometry state;
    // Preserve the age of the oldest source. Re-stamping stale localization
    // with 'now' would defeat the controller and arbiter watchdogs.
    state.header.stamp = velocity_stamp < transform_stamp ?
      velocity_stamp : transform_stamp;
    state.header.frame_id = map_frame_;
    state.child_frame_id = base_frame_;
    state.pose.pose.position.x = transform.transform.translation.x;
    state.pose.pose.position.y = transform.transform.translation.y;
    state.pose.pose.position.z = transform.transform.translation.z;
    state.pose.pose.orientation = transform.transform.rotation;
    state.pose.covariance = velocity->pose.covariance;
    state.twist = velocity->twist;
    state_pub_->publish(state);
  }

  std::string map_frame_;
  std::string base_frame_;
  double stale_timeout_{0.10};
  std::mutex mutex_;
  nav_msgs::msg::Odometry::ConstSharedPtr latest_velocity_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr velocity_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr state_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

}  // namespace race_bringup

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<race_bringup::StateEstimatorBridge>());
  rclcpp::shutdown();
  return 0;
}
