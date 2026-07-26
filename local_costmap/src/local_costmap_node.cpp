#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include "local_costmap/rolling_costmap.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace local_costmap
{

class LocalCostmapNode final : public rclcpp::Node
{
public:
  explicit LocalCostmapNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node("local_costmap", options)
  {
    declare_parameter<std::string>("scan_topic", "/scan");
    declare_parameter<std::string>("odom_topic", "/ego_racecar/odom");
    declare_parameter<std::string>("costmap_topic", "/perception/local_costmap");
    declare_parameter<std::string>("map_frame", "map");
    declare_parameter<std::string>("base_frame", "ego_racecar/base_link");
    declare_parameter<double>("size_x", 22.0);
    declare_parameter<double>("size_y", 22.0);
    declare_parameter<double>("resolution", 0.05);
    declare_parameter<double>("obstacle_persistence", 0.20);
    declare_parameter<double>("forward_offset", 0.0);
    declare_parameter<bool>("align_with_vehicle", false);
    declare_parameter<double>("odom_timeout", 0.10);
    declare_parameter<double>("tf_timeout", 0.02);
    declare_parameter<double>("max_range", 30.0);
    declare_parameter<double>("wall_connection_max_gap", 0.20);
    declare_parameter<double>("wall_connection_spacing", 0.025);

    scan_topic_ = get_parameter("scan_topic").as_string();
    odom_topic_ = get_parameter("odom_topic").as_string();
    costmap_topic_ = get_parameter("costmap_topic").as_string();
    map_frame_ = get_parameter("map_frame").as_string();
    base_frame_ = get_parameter("base_frame").as_string();
    odom_timeout_ = get_parameter("odom_timeout").as_double();
    tf_timeout_ = get_parameter("tf_timeout").as_double();
    max_range_ = get_parameter("max_range").as_double();
    wall_connection_max_gap_ =
      get_parameter("wall_connection_max_gap").as_double();
    wall_connection_spacing_ =
      get_parameter("wall_connection_spacing").as_double();
    if (!positive(odom_timeout_) || !positive(tf_timeout_) || !positive(max_range_) ||
      !positive(wall_connection_max_gap_) || !positive(wall_connection_spacing_) ||
      wall_connection_spacing_ > wall_connection_max_gap_)
    {
      throw std::invalid_argument("local costmap timing and range parameters must be positive");
    }

    RollingCostmapConfig config;
    config.size_x = get_parameter("size_x").as_double();
    config.size_y = get_parameter("size_y").as_double();
    config.resolution = get_parameter("resolution").as_double();
    config.persistence = get_parameter("obstacle_persistence").as_double();
    config.forward_offset = get_parameter("forward_offset").as_double();
    config.align_with_vehicle = get_parameter("align_with_vehicle").as_bool();
    costmap_ = std::make_unique<RollingCostmap>(config);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      costmap_topic_, rclcpp::QoS(1).reliable());
    odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::QoS(10).reliable(),
      std::bind(&LocalCostmapNode::odomCallback, this, std::placeholders::_1));
    scan_subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, rclcpp::SensorDataQoS(),
      std::bind(&LocalCostmapNode::scanCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(), "Local costmap: scan=%s odom=%s output=%s %.1fx%.1fm @ %.3fm",
      scan_topic_.c_str(), odom_topic_.c_str(), costmap_topic_.c_str(),
      config.size_x, config.size_y, config.resolution);
    RCLCPP_INFO(
      get_logger(),
      "%s local costmap covers %.2f m ahead, %.2f m behind, "
      "and %.2f m to each side",
      config.align_with_vehicle ? "Vehicle-aligned" : "Map-aligned",
      0.5 * config.size_x + config.forward_offset,
      0.5 * config.size_x - config.forward_offset,
      0.5 * config.size_y);
  }

private:
  static bool positive(double value)
  {
    return std::isfinite(value) && value > 0.0;
  }

  void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr message)
  {
    const rclcpp::Time received = now();
    rclcpp::Time stamp(message->header.stamp, get_clock()->get_clock_type());
    if (stamp.nanoseconds() == 0) {
      stamp = received;
    }
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_odom_stamp_ = stamp;
    latest_odom_received_ = received;
    have_odom_ = true;
  }

  void scanCallback(const sensor_msgs::msg::LaserScan::ConstSharedPtr message)
  {
    if (message->header.frame_id.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Rejected LaserScan with empty frame_id");
      return;
    }

    const rclcpp::Time received = now();
    rclcpp::Time source_stamp(message->header.stamp, get_clock()->get_clock_type());
    if (source_stamp.nanoseconds() == 0) {
      source_stamp = received;
    }

    rclcpp::Time odom_stamp(0, 0, get_clock()->get_clock_type());
    rclcpp::Time odom_received(0, 0, get_clock()->get_clock_type());
    bool have_odom = false;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      have_odom = have_odom_;
      odom_stamp = latest_odom_stamp_;
      odom_received = latest_odom_received_;
    }
    const double odom_receive_age = (received - odom_received).seconds();
    const double odom_source_age = std::abs((source_stamp - odom_stamp).seconds());
    if (!have_odom || !std::isfinite(odom_receive_age) ||
      odom_receive_age < -0.02 || odom_receive_age > odom_timeout_ ||
      !std::isfinite(odom_source_age) || odom_source_age > odom_timeout_)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Skipping scan: odometry unavailable or stale (receive=%.3fs source=%.3fs)",
        odom_receive_age, odom_source_age);
      return;
    }

    geometry_msgs::msg::TransformStamped map_to_scan;
    geometry_msgs::msg::TransformStamped map_to_base;
    try {
      const auto timeout = tf2::durationFromSec(tf_timeout_);
      map_to_scan = tf_buffer_->lookupTransform(
        map_frame_, message->header.frame_id, source_stamp, timeout);
      map_to_base = tf_buffer_->lookupTransform(
        map_frame_, base_frame_, source_stamp, timeout);
    } catch (const tf2::TransformException & exception) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000, "Skipping scan: transform unavailable: %s",
        exception.what());
      return;
    }

    const double scan_yaw = tf2::getYaw(map_to_scan.transform.rotation);
    const double scan_x = map_to_scan.transform.translation.x;
    const double scan_y = map_to_scan.transform.translation.y;
    const double usable_max_range = std::min<double>(message->range_max, max_range_);
    std::vector<IndexedPoint2d> indexed_hits;
    indexed_hits.reserve(message->ranges.size());
    for (std::size_t index = 0U; index < message->ranges.size(); ++index) {
      const double range = static_cast<double>(message->ranges[index]);
      if (!std::isfinite(range) || range < static_cast<double>(message->range_min) ||
        range >= usable_max_range - 1.0e-4)
      {
        continue;
      }
      const double angle =
        scan_yaw + static_cast<double>(message->angle_min) +
        static_cast<double>(index) * static_cast<double>(message->angle_increment);
      indexed_hits.push_back(IndexedPoint2d{
        index,
        Point2d{
          scan_x + range * std::cos(angle),
          scan_y + range * std::sin(angle)}});
    }
    const std::vector<Point2d> hits = connectAdjacentHits(
      indexed_hits, wall_connection_max_gap_, wall_connection_spacing_);

    CostmapGrid grid;
    try {
      const double base_yaw = tf2::getYaw(map_to_base.transform.rotation);
      grid = costmap_->update(
        map_to_base.transform.translation.x, map_to_base.transform.translation.y,
        base_yaw, source_stamp.seconds(), hits);
    } catch (const std::exception & exception) {
      RCLCPP_ERROR(get_logger(), "Costmap update failed: %s", exception.what());
      costmap_->reset();
      return;
    }

    nav_msgs::msg::OccupancyGrid output;
    output.header = message->header;
    output.header.frame_id = map_frame_;
    output.info.map_load_time = message->header.stamp;
    output.info.resolution = static_cast<float>(grid.resolution);
    output.info.width = static_cast<std::uint32_t>(grid.width);
    output.info.height = static_cast<std::uint32_t>(grid.height);
    output.info.origin.position.x = grid.origin_x;
    output.info.origin.position.y = grid.origin_y;
    output.info.origin.orientation.z = std::sin(0.5 * grid.origin_yaw);
    output.info.origin.orientation.w = std::cos(0.5 * grid.origin_yaw);
    output.data = std::move(grid.data);
    publisher_->publish(output);
  }

  std::string scan_topic_;
  std::string odom_topic_;
  std::string costmap_topic_;
  std::string map_frame_;
  std::string base_frame_;
  double odom_timeout_{0.10};
  double tf_timeout_{0.02};
  double max_range_{30.0};
  double wall_connection_max_gap_{0.20};
  double wall_connection_spacing_{0.025};

  std::unique_ptr<RollingCostmap> costmap_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;

  std::mutex data_mutex_;
  rclcpp::Time latest_odom_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time latest_odom_received_{0, 0, RCL_ROS_TIME};
  bool have_odom_{false};
};

}  // namespace local_costmap

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<local_costmap::LocalCostmapNode>());
  rclcpp::shutdown();
  return 0;
}
