#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "opponent_tracker/scan_clusterer.hpp"
#include "opponent_tracker/static_map_filter.hpp"
#include "rclcpp/rclcpp.hpp"
#include "roboracer_msgs/msg/tracked_obstacle_array.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace opponent_tracker
{

class OpponentTrackerNode : public rclcpp::Node
{
public:
  OpponentTrackerNode()
  : Node("opponent_tracker"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    const auto scan_topic = declare_parameter<std::string>("scan_topic", "/scan");
    const auto odom_topic = declare_parameter<std::string>(
      "odom_topic", "/state_estimation/odom");
    const auto output_topic = declare_parameter<std::string>(
      "output_topic", "/perception/obstacles");
    const auto map_topic = declare_parameter<std::string>("map_topic", "/map");
    target_frame_ = declare_parameter<std::string>("target_frame", "map");

    const double publish_rate = declare_parameter<double>("publish_rate_hz", 50.0);
    transform_timeout_ = declare_parameter<double>("transform_timeout", 0.005);
    if (!std::isfinite(publish_rate) || publish_rate <= 0.0 ||
      !std::isfinite(transform_timeout_) || transform_timeout_ < 0.0)
    {
      throw std::invalid_argument("invalid rate or transform timeout");
    }
    publish_period_ = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / publish_rate));

    const int min_cluster_points = declare_parameter<int>("min_cluster_points", 3);
    const int confidence_full_points =
      declare_parameter<int>("confidence_full_points", 20);
    if (min_cluster_points < 2 || confidence_full_points < min_cluster_points) {
      throw std::invalid_argument("invalid cluster point limits");
    }

    ScanClustererConfig clusterer_config;
    clusterer_config.min_range = declare_parameter<double>("min_range", 0.05);
    clusterer_config.max_range = declare_parameter<double>("max_range", 8.0);
    clusterer_config.breakpoint_base =
      declare_parameter<double>("breakpoint_base", 0.06);
    clusterer_config.breakpoint_scale =
      declare_parameter<double>("breakpoint_scale", 2.0);
    clusterer_config.min_cluster_points =
      static_cast<std::size_t>(min_cluster_points);
    clusterer_config.min_object_length =
      declare_parameter<double>("min_object_length", 0.05);
    clusterer_config.max_object_length =
      declare_parameter<double>("max_object_length", 1.0);
    clusterer_config.max_object_width =
      declare_parameter<double>("max_object_width", 0.8);
    clusterer_config.min_box_dimension =
      declare_parameter<double>("min_box_dimension", 0.04);
    clusterer_config.confidence_full_points =
      static_cast<std::size_t>(confidence_full_points);
    clusterer_ = std::make_unique<ScanClusterer>(clusterer_config);

    StaticMapFilterConfig map_filter_config;
    map_filter_config.enabled =
      declare_parameter<bool>("use_map_filter", true);
    map_filter_config.radius =
      declare_parameter<double>("map_filter_radius", 0.12);
    map_filter_config.occupied_threshold =
      declare_parameter<int>("map_occupied_threshold", 50);
    map_filter_config.unknown_is_occupied =
      declare_parameter<bool>("map_unknown_is_occupied", true);
    use_map_filter_ = map_filter_config.enabled;
    map_filter_ = std::make_unique<StaticMapFilter>(map_filter_config);

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

    map_subscription_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic, rclcpp::QoS(1).transient_local().reliable(),
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr message) {
        map_filter_->update(*message);
      });

    processing_timer_ = create_wall_timer(
      publish_period_,
      std::bind(&OpponentTrackerNode::processLatestScan, this));

    RCLCPP_INFO(
      get_logger(),
      "Single-frame detector started: %s -> %s (%s, %.1f Hz, map filter: %s)",
      scan_topic.c_str(), output_topic.c_str(), target_frame_.c_str(), publish_rate,
      use_map_filter_ ? map_topic.c_str() : "off");
  }

private:
  void scanCallback(const sensor_msgs::msg::LaserScan::ConstSharedPtr message)
  {
    std::lock_guard<std::mutex> lock(scan_mutex_);
    latest_scan_ = message;
    ++latest_scan_generation_;
  }

  void processLatestScan()
  {
    sensor_msgs::msg::LaserScan::ConstSharedPtr message;
    {
      std::lock_guard<std::mutex> lock(scan_mutex_);
      if (!latest_scan_ || processed_scan_generation_ == latest_scan_generation_) {
        return;
      }
      message = latest_scan_;
      processed_scan_generation_ = latest_scan_generation_;
    }

    if (use_map_filter_ && !map_filter_->ready()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Waiting for static map before publishing detections");
      return;
    }

    const std::string source_frame = message->header.frame_id;
    const std::string output_frame =
      target_frame_.empty() ? source_frame : target_frame_;
    if (source_frame.empty() || output_frame.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "LaserScan frame_id is empty");
      return;
    }
    if (use_map_filter_ && !map_filter_->supportsFrame(output_frame)) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Static map frame does not match detection output frame '%s'",
        output_frame.c_str());
      return;
    }

    double transform_x = 0.0;
    double transform_y = 0.0;
    double transform_yaw = 0.0;
    if (output_frame != source_frame) {
      try {
        const auto transform = tf_buffer_.lookupTransform(
          output_frame, source_frame, rclcpp::Time(message->header.stamp),
          rclcpp::Duration::from_seconds(transform_timeout_));
        const auto & translation = transform.transform.translation;
        const auto & rotation = transform.transform.rotation;
        transform_x = translation.x;
        transform_y = translation.y;
        transform_yaw = std::atan2(
          2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
          1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z));
      } catch (const tf2::TransformException & error) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Skipping scan: cannot transform %s to %s: %s",
          source_frame.c_str(), output_frame.c_str(), error.what());
        return;
      }
    }

    const auto detections = clusterer_->detect(
      message->ranges,
      static_cast<double>(message->angle_min),
      static_cast<double>(message->angle_increment),
      static_cast<double>(message->range_min),
      static_cast<double>(message->range_max));

    roboracer_msgs::msg::TrackedObstacleArray output;
    output.header = message->header;
    output.header.frame_id = output_frame;
    output.obstacles.reserve(detections.size());

    const double cos_yaw = std::cos(transform_yaw);
    const double sin_yaw = std::sin(transform_yaw);
    for (const auto & detection : detections) {
      const double detection_x =
        transform_x + cos_yaw * detection.x - sin_yaw * detection.y;
      const double detection_y =
        transform_y + sin_yaw * detection.x + cos_yaw * detection.y;
      if (map_filter_->nearOccupied(detection_x, detection_y, output_frame)) {
        continue;
      }

      roboracer_msgs::msg::TrackedObstacle obstacle;
      obstacle.id = -1;
      obstacle.classification = roboracer_msgs::msg::TrackedObstacle::UNKNOWN;
      obstacle.x = detection_x;
      obstacle.y = detection_y;
      obstacle.yaw = normalizeAngle(detection.yaw + transform_yaw);
      obstacle.vx = 0.0;
      obstacle.vy = 0.0;
      obstacle.s = 0.0;
      obstacle.d = 0.0;
      obstacle.length = detection.length;
      obstacle.width = detection.width;
      obstacle.confidence = detection.confidence;
      obstacle.dynamic = false;
      obstacle.visible = true;
      output.obstacles.push_back(obstacle);
    }

    publisher_->publish(output);
  }

  static double normalizeAngle(double angle)
  {
    constexpr double kPi = 3.14159265358979323846;
    while (angle > kPi) {
      angle -= 2.0 * kPi;
    }
    while (angle < -kPi) {
      angle += 2.0 * kPi;
    }
    return angle;
  }

  std::string latest_odom_frame_;
  std::string target_frame_;
  double transform_timeout_{0.005};
  bool use_map_filter_{true};
  std::chrono::nanoseconds publish_period_{20000000};
  std::mutex scan_mutex_;
  sensor_msgs::msg::LaserScan::ConstSharedPtr latest_scan_;
  std::uint64_t latest_scan_generation_{0U};
  std::uint64_t processed_scan_generation_{0U};
  std::unique_ptr<ScanClusterer> clusterer_;
  std::unique_ptr<StaticMapFilter> map_filter_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<roboracer_msgs::msg::TrackedObstacleArray>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscription_;
  rclcpp::TimerBase::SharedPtr processing_timer_;
};

}  // namespace opponent_tracker

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<opponent_tracker::OpponentTrackerNode>());
  rclcpp::shutdown();
  return 0;
}
