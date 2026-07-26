// Copyright 2026 RoboRacer Team

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <iomanip>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "local_planner/frenet_planner.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "roboracer_msgs/msg/tracked_obstacle_array.hpp"
#include "roboracer_msgs/msg/trajectory.hpp"
#include "std_msgs/msg/header.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace local_planner
{
namespace
{

double yawFromQuaternion(const geometry_msgs::msg::Quaternion & quaternion)
{
  return std::atan2(
    2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
    1.0 - 2.0 * (
      quaternion.y * quaternion.y + quaternion.z * quaternion.z));
}

bool finite(double value)
{
  return std::isfinite(value);
}

}  // namespace

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
    const auto marker_topic = declare_parameter<std::string>(
      "marker_topic", "/planner/candidate_markers");
    const auto race_line_file = declare_parameter<std::string>(
      "race_line_file", "");
    target_frame_ = declare_parameter<std::string>("target_frame", "map");
    trajectory_source_ = declare_parameter<std::string>(
      "trajectory_source", "frenet_quintic");
    state_timeout_ = declare_parameter<double>("state_timeout", 0.10);
    obstacle_timeout_ = declare_parameter<double>("obstacle_timeout", 0.15);
    const double publish_rate =
      declare_parameter<double>("publish_rate_hz", 30.0);
    const int projection_search_radius =
      declare_parameter<int>("projection_search_radius", 60);
    const int lateral_samples_per_side =
      declare_parameter<int>("lateral_samples_per_side", 3);

    FrenetPlannerConfig config;
    config.planning_horizons =
      declare_parameter<std::vector<double>>(
      "planning_horizons", std::vector<double>{3.0, 4.0, 5.0});
    config.speed_scales =
      declare_parameter<std::vector<double>>(
      "speed_scales", std::vector<double>{0.60, 0.80, 1.00});
    config.lateral_samples_per_side =
      lateral_samples_per_side > 0 ?
      static_cast<std::size_t>(lateral_samples_per_side) : 0U;
    config.sample_spacing =
      declare_parameter<double>("sample_spacing", 0.10);
    config.transition_length =
      declare_parameter<double>("transition_length", 2.0);
    config.overtake_offset =
      declare_parameter<double>("overtake_offset", 0.45);
    config.minimum_lateral_offset =
      declare_parameter<double>("minimum_lateral_offset", 0.10);
    config.boundary_sampling_buffer =
      declare_parameter<double>("boundary_sampling_buffer", 0.01);
    config.projection_search_radius =
      projection_search_radius > 0 ?
      static_cast<std::size_t>(projection_search_radius) : 0U;
    config.max_projection_distance =
      declare_parameter<double>("max_projection_distance", 1.0);
    config.minimum_frenet_jacobian =
      declare_parameter<double>("minimum_frenet_jacobian", 0.20);

    config.vehicle_length =
      declare_parameter<double>("vehicle_length", 0.552);
    config.vehicle_width =
      declare_parameter<double>("vehicle_width", 0.320);
    config.safety_margin =
      declare_parameter<double>("safety_margin", 0.05);
    config.collision_margin =
      declare_parameter<double>("collision_margin", 0.08);
    config.default_opponent_length =
      declare_parameter<double>("default_opponent_length", 0.552);
    config.default_opponent_width =
      declare_parameter<double>("default_opponent_width", 0.320);
    config.minimum_obstacle_confidence =
      declare_parameter<double>("minimum_obstacle_confidence", 0.05);

    config.max_speed = declare_parameter<double>("max_speed", 4.0);
    config.max_lateral_acceleration =
      declare_parameter<double>("max_lateral_acceleration", 4.0);
    config.min_acceleration =
      declare_parameter<double>("min_acceleration", -1.5);
    config.max_acceleration =
      declare_parameter<double>("max_acceleration", 1.0);
    config.max_curvature =
      declare_parameter<double>("max_curvature", 1.0);

    config.weight_lateral_offset =
      declare_parameter<double>("weight_lateral_offset", 2.0);
    config.weight_curvature =
      declare_parameter<double>("weight_curvature", 1.0);
    config.weight_clearance =
      declare_parameter<double>("weight_clearance", 0.20);
    config.weight_switch =
      declare_parameter<double>("weight_switch", 3.0);
    config.weight_speed =
      declare_parameter<double>("weight_speed", 1.0);
    config.weight_horizon =
      declare_parameter<double>("weight_horizon", 0.5);

    if (race_line_file.empty()) {
      throw std::invalid_argument("race_line_file is required");
    }
    if (!finite(state_timeout_) || state_timeout_ <= 0.0 ||
      !finite(obstacle_timeout_) || obstacle_timeout_ <= 0.0 ||
      !finite(publish_rate) || publish_rate <= 0.0 ||
      target_frame_.empty())
    {
      throw std::invalid_argument("invalid local planner timing or frame configuration");
    }
    const std::size_t maximum_candidate_count =
      (2U * config.lateral_samples_per_side + 1U) *
      config.planning_horizons.size() * config.speed_scales.size();
    planner_ = std::make_unique<FrenetPlanner>(
      ReferenceLine::fromCsv(race_line_file), config);

    publisher_ = create_publisher<roboracer_msgs::msg::Trajectory>(
      output_topic, rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
    marker_publisher_ =
      create_publisher<visualization_msgs::msg::MarkerArray>(
      marker_topic, rclcpp::QoS(rclcpp::KeepLast(1)).reliable());

    obstacles_subscription_ =
      create_subscription<roboracer_msgs::msg::TrackedObstacleArray>(
      obstacles_topic, rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
      [this](
        roboracer_msgs::msg::TrackedObstacleArray::ConstSharedPtr message)
      {
        latest_obstacles_ = std::move(message);
      });
    odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, rclcpp::QoS(rclcpp::KeepLast(5)).reliable(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr message) {
        latest_odom_ = std::move(message);
      });

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / publish_rate));
    timer_ = create_wall_timer(
      period, std::bind(&LocalPlannerNode::planAndPublish, this));

    RCLCPP_INFO(
      get_logger(),
      "Frenet planner started: %s + %s -> %s "
      "(%.1f Hz, %.2f m line, up to %zu candidates)",
      odom_topic.c_str(), obstacles_topic.c_str(), output_topic.c_str(),
      publish_rate, planner_->referenceLine().length(),
      maximum_candidate_count);
  }

private:
  void planAndPublish()
  {
    if (!latest_odom_) {
      publishInvalid(get_clock()->now(), "missing_odometry");
      return;
    }
    const rclcpp::Time odom_stamp(latest_odom_->header.stamp);
    const double odom_age = (get_clock()->now() - odom_stamp).seconds();
    if (!finite(odom_age) || odom_age < 0.0 || odom_age > state_timeout_) {
      publishInvalid(odom_stamp, "stale_odometry");
      return;
    }
    if (latest_odom_->header.frame_id != target_frame_) {
      publishInvalid(odom_stamp, "odometry_frame_mismatch");
      return;
    }
    if (!latest_obstacles_) {
      publishInvalid(odom_stamp, "missing_obstacles");
      return;
    }
    const rclcpp::Time obstacle_stamp(latest_obstacles_->header.stamp);
    const double obstacle_age =
      (get_clock()->now() - obstacle_stamp).seconds();
    if (!finite(obstacle_age) || obstacle_age < 0.0 ||
      obstacle_age > obstacle_timeout_)
    {
      publishInvalid(odom_stamp, "stale_obstacles");
      return;
    }
    if (latest_obstacles_->header.frame_id != target_frame_) {
      publishInvalid(odom_stamp, "obstacle_frame_mismatch");
      return;
    }

    EgoState ego;
    ego.x = latest_odom_->pose.pose.position.x;
    ego.y = latest_odom_->pose.pose.position.y;
    ego.yaw = yawFromQuaternion(latest_odom_->pose.pose.orientation);
    ego.speed = std::hypot(
      latest_odom_->twist.twist.linear.x,
      latest_odom_->twist.twist.linear.y);

    std::vector<Obstacle> obstacles;
    obstacles.reserve(latest_obstacles_->obstacles.size());
    for (const auto & message : latest_obstacles_->obstacles) {
      Obstacle obstacle;
      obstacle.id = message.id;
      obstacle.x = message.x;
      obstacle.y = message.y;
      obstacle.yaw = message.yaw;
      obstacle.vx = message.vx;
      obstacle.vy = message.vy;
      obstacle.length = message.length;
      obstacle.width = message.width;
      obstacle.confidence = message.confidence;
      obstacle.visible = message.visible;
      obstacles.push_back(obstacle);
    }

    PlanResult result;
    try {
      result = planner_->plan(
        ego, obstacles, projection_hint_, previous_target_d_);
    } catch (const std::exception & exception) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Planner failed: %s", exception.what());
      publishInvalid(odom_stamp, "planner_exception");
      return;
    }
    projection_hint_ = result.projection_index;
    if (result.valid) {
      previous_target_d_ =
        result.candidates[result.selected_index].target_d;
    }

    roboracer_msgs::msg::Trajectory output;
    output.header = latest_odom_->header;
    output.header.frame_id = target_frame_;
    output.source = trajectory_source_;
    output.valid = result.valid;
    if (result.valid) {
      const auto & selected = result.candidates[result.selected_index];
      output.points.reserve(selected.points.size());
      for (const auto & sample : selected.points) {
        roboracer_msgs::msg::TrajectoryPoint point;
        point.x = sample.x;
        point.y = sample.y;
        point.yaw = sample.yaw;
        point.s = sample.s;
        point.d = sample.d;
        point.speed = sample.speed;
        point.acceleration = sample.acceleration;
        point.curvature = sample.curvature;
        point.left_width = sample.left_width;
        point.right_width = sample.right_width;
        output.points.push_back(point);
      }
    }
    publisher_->publish(output);
    publishMarkers(result, output.header);
  }

  void publishInvalid(const rclcpp::Time & stamp, const std::string & reason)
  {
    roboracer_msgs::msg::Trajectory output;
    output.header.stamp = stamp;
    output.header.frame_id = target_frame_;
    output.source = trajectory_source_;
    output.valid = false;
    publisher_->publish(output);

    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker clear;
    clear.header = output.header;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(clear);
    marker_publisher_->publish(markers);
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Local trajectory invalid: %s", reason.c_str());
  }

  void publishMarkers(
    const PlanResult & result, const std_msgs::msg::Header & header)
  {
    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker clear;
    clear.header = header;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(clear);

    std::size_t valid_count = 0U;
    std::size_t collision_count = 0U;
    std::size_t boundary_count = 0U;
    std::size_t curvature_count = 0U;
    std::size_t other_invalid_count = 0U;
    for (std::size_t index = 0U; index < result.candidates.size(); ++index) {
      const auto & candidate = result.candidates[index];
      if (candidate.valid) {
        ++valid_count;
      } else if (candidate.reason == "collision") {
        ++collision_count;
      } else if (candidate.reason == "track_boundary") {
        ++boundary_count;
      } else if (candidate.reason == "curvature_limit") {
        ++curvature_count;
      } else {
        ++other_invalid_count;
      }
      visualization_msgs::msg::Marker line;
      line.header = header;
      line.ns = "frenet_candidates";
      line.id = static_cast<int>(index);
      line.type = visualization_msgs::msg::Marker::LINE_STRIP;
      line.action = visualization_msgs::msg::Marker::ADD;
      line.pose.orientation.w = 1.0;
      line.scale.x = index == result.selected_index && result.valid ? 0.055 : 0.025;
      line.color.a = candidate.valid ? 0.95F : 0.45F;
      if (!candidate.valid) {
        line.color.r = 1.0F;
        line.color.g = 0.10F;
        line.color.b = 0.10F;
      } else if (index == result.selected_index && result.valid) {
        line.color.r = 0.10F;
        line.color.g = 1.0F;
        line.color.b = 0.20F;
      } else {
        line.color.r = 0.10F;
        line.color.g = 0.55F;
        line.color.b = 1.0F;
      }
      for (const auto & sample : candidate.points) {
        geometry_msgs::msg::Point point;
        point.x = sample.x;
        point.y = sample.y;
        point.z = 0.08;
        line.points.push_back(point);
      }
      markers.markers.push_back(line);

      if (!candidate.points.empty() &&
        index == result.selected_index && result.valid)
      {
        visualization_msgs::msg::Marker label;
        label.header = header;
        label.ns = "frenet_selected_label";
        label.id = 0;
        label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        label.action = visualization_msgs::msg::Marker::ADD;
        label.pose.position.x = candidate.points.back().x;
        label.pose.position.y = candidate.points.back().y;
        label.pose.position.z = 0.25;
        label.pose.orientation.w = 1.0;
        label.scale.z = 0.14;
        label.color.r = line.color.r;
        label.color.g = line.color.g;
        label.color.b = line.color.b;
        label.color.a = 1.0F;
        std::ostringstream text;
        text.precision(2);
        text << candidate.name << " cost=" << std::fixed << candidate.cost;
        label.text = text.str();
        markers.markers.push_back(label);
      }
    }

    if (!result.candidates.empty() &&
      !result.candidates.front().points.empty())
    {
      visualization_msgs::msg::Marker summary;
      summary.header = header;
      summary.ns = "frenet_candidate_summary";
      summary.id = 0;
      summary.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      summary.action = visualization_msgs::msg::Marker::ADD;
      summary.pose.position.x = result.candidates.front().points.front().x;
      summary.pose.position.y = result.candidates.front().points.front().y;
      summary.pose.position.z = 0.45;
      summary.pose.orientation.w = 1.0;
      summary.scale.z = 0.16;
      summary.color.r = result.valid ? 0.15F : 1.0F;
      summary.color.g = result.valid ? 1.0F : 0.15F;
      summary.color.b = 0.15F;
      summary.color.a = 1.0F;
      std::ostringstream text;
      text << "total=" << result.candidates.size()
           << " valid=" << valid_count
           << " collision=" << collision_count
           << " boundary=" << boundary_count
           << " curvature=" << curvature_count
           << " other=" << other_invalid_count;
      summary.text = text.str();
      markers.markers.push_back(summary);
    }
    marker_publisher_->publish(markers);
  }

  std::string target_frame_;
  std::string trajectory_source_;
  double state_timeout_{0.10};
  double obstacle_timeout_{0.15};
  std::unique_ptr<FrenetPlanner> planner_;
  std::optional<std::size_t> projection_hint_;
  std::optional<double> previous_target_d_;
  nav_msgs::msg::Odometry::ConstSharedPtr latest_odom_;
  roboracer_msgs::msg::TrackedObstacleArray::ConstSharedPtr latest_obstacles_;
  rclcpp::Publisher<roboracer_msgs::msg::Trajectory>::SharedPtr publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    marker_publisher_;
  rclcpp::Subscription<roboracer_msgs::msg::TrackedObstacleArray>::SharedPtr
    obstacles_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace local_planner

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<local_planner::LocalPlannerNode>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(
      rclcpp::get_logger("local_planner"), "Fatal error: %s", exception.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
