// Copyright 2026 RoboRacer Team

#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

#include "geometry_msgs/msg/quaternion.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "race_manager/race_state_machine.hpp"
#include "rclcpp/rclcpp.hpp"
#include "roboracer_msgs/msg/race_state.hpp"
#include "roboracer_msgs/msg/tracked_obstacle.hpp"
#include "roboracer_msgs/msg/tracked_obstacle_array.hpp"
#include "roboracer_msgs/msg/trajectory.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace race_manager
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

const char * stateName(BehaviorState state)
{
  switch (state) {
    case BehaviorState::INIT:
      return "INIT";
    case BehaviorState::READY:
      return "READY";
    case BehaviorState::GLOBAL_TRACK:
      return "GLOBAL_TRACK";
    case BehaviorState::TRAILING:
      return "TRAILING";
    case BehaviorState::OVERTAKE:
      return "OVERTAKE";
    case BehaviorState::FAULT:
      return "FAULT";
    case BehaviorState::STOP:
      return "STOP";
    case BehaviorState::RETURN:
      return "RETURN";
  }
  return "UNKNOWN";
}

const char * sideName(PreferredSide side)
{
  switch (side) {
    case PreferredSide::LEFT:
      return "LEFT";
    case PreferredSide::RIGHT:
      return "RIGHT";
    case PreferredSide::NONE:
      return "NONE";
  }
  return "NONE";
}

}  // namespace

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
    const auto trajectory_topic = declare_parameter<std::string>(
      "trajectory_topic", "/planner/local_trajectory");
    const auto output_topic = declare_parameter<std::string>(
      "output_topic", "/race_manager/state");
    const auto marker_topic = declare_parameter<std::string>(
      "marker_topic", "/race_manager/state_marker");
    target_frame_ = declare_parameter<std::string>("target_frame", "map");
    state_timeout_ = declare_parameter<double>("state_timeout", 0.10);
    obstacle_timeout_ = declare_parameter<double>("obstacle_timeout", 0.15);
    trajectory_timeout_ =
      declare_parameter<double>("trajectory_timeout", 0.15);
    minimum_obstacle_confidence_ =
      declare_parameter<double>("minimum_obstacle_confidence", 0.05);
    const double publish_rate =
      declare_parameter<double>("publish_rate_hz", 20.0);

    StateMachineConfig config;
    config.follow_distance =
      declare_parameter<double>("follow_distance", 4.0);
    config.opponent_corridor_half_width =
      declare_parameter<double>("opponent_corridor_half_width", 0.80);
    config.pass_margin = declare_parameter<double>("pass_margin", 0.30);
    config.overtake_lateral_threshold =
      declare_parameter<double>("overtake_lateral_threshold", 0.08);
    config.return_lateral_threshold =
      declare_parameter<double>("return_lateral_threshold", 0.05);
    config.transition_confirmation =
      declare_parameter<double>("transition_confirmation", 0.20);
    config.minimum_state_duration =
      declare_parameter<double>("minimum_state_duration", 0.40);
    config.recovery_confirmation =
      declare_parameter<double>("recovery_confirmation", 0.50);
    config.opponent_lost_timeout =
      declare_parameter<double>("opponent_lost_timeout", 0.50);
    config.cruise_speed_scale =
      declare_parameter<double>("cruise_speed_scale", 1.0);
    config.trailing_speed_scale =
      declare_parameter<double>("trailing_speed_scale", 0.60);
    config.overtake_speed_scale =
      declare_parameter<double>("overtake_speed_scale", 1.0);
    config.return_speed_scale =
      declare_parameter<double>("return_speed_scale", 0.80);

    if (!finite(publish_rate) || publish_rate <= 0.0 ||
      !finite(state_timeout_) || state_timeout_ <= 0.0 ||
      !finite(obstacle_timeout_) || obstacle_timeout_ <= 0.0 ||
      !finite(trajectory_timeout_) || trajectory_timeout_ <= 0.0 ||
      !finite(minimum_obstacle_confidence_) ||
      minimum_obstacle_confidence_ < 0.0 || target_frame_.empty())
    {
      throw std::invalid_argument("invalid race manager node configuration");
    }
    state_machine_ = std::make_unique<RaceStateMachine>(config);

    state_publisher_ = create_publisher<roboracer_msgs::msg::RaceState>(
      output_topic, rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
    marker_publisher_ = create_publisher<visualization_msgs::msg::Marker>(
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
    trajectory_subscription_ =
      create_subscription<roboracer_msgs::msg::Trajectory>(
      trajectory_topic, rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
      [this](roboracer_msgs::msg::Trajectory::ConstSharedPtr message) {
        latest_trajectory_ = std::move(message);
      });

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / publish_rate));
    timer_ = create_wall_timer(
      period, std::bind(&RaceManagerNode::updateAndPublish, this));

    RCLCPP_INFO(
      get_logger(), "Race manager started: %s + %s + %s -> %s (%.1f Hz)",
      odom_topic.c_str(), obstacles_topic.c_str(), trajectory_topic.c_str(),
      output_topic.c_str(), publish_rate);
  }

private:
  bool fresh(
    const builtin_interfaces::msg::Time & stamp, double timeout,
    const rclcpp::Time & now) const
  {
    const double age = (now - rclcpp::Time(stamp)).seconds();
    return finite(age) && age >= 0.0 && age <= timeout;
  }

  void updateAndPublish()
  {
    const rclcpp::Time now = get_clock()->now();
    StateObservation observation;
    observation.time = now.seconds();

    const bool odom_valid =
      latest_odom_ &&
      latest_odom_->header.frame_id == target_frame_ &&
      fresh(latest_odom_->header.stamp, state_timeout_, now);
    const bool obstacles_valid =
      latest_obstacles_ &&
      latest_obstacles_->header.frame_id == target_frame_ &&
      fresh(latest_obstacles_->header.stamp, obstacle_timeout_, now);
    const bool trajectory_fresh =
      latest_trajectory_ &&
      latest_trajectory_->header.frame_id == target_frame_ &&
      fresh(latest_trajectory_->header.stamp, trajectory_timeout_, now);
    observation.inputs_ready =
      odom_valid && obstacles_valid && trajectory_fresh;
    observation.trajectory_valid =
      trajectory_fresh && latest_trajectory_->valid &&
      !latest_trajectory_->points.empty();

    double marker_x = 0.0;
    double marker_y = 0.0;
    if (odom_valid) {
      marker_x = latest_odom_->pose.pose.position.x;
      marker_y = latest_odom_->pose.pose.position.y;
      const double ego_yaw =
        yawFromQuaternion(latest_odom_->pose.pose.orientation);
      const double cosine = std::cos(ego_yaw);
      const double sine = std::sin(ego_yaw);
      double nearest_distance = std::numeric_limits<double>::infinity();
      if (obstacles_valid) {
        for (const auto & obstacle : latest_obstacles_->obstacles) {
          if (obstacle.classification !=
            roboracer_msgs::msg::TrackedObstacle::OPPONENT ||
            !finite(obstacle.x) || !finite(obstacle.y) ||
            !finite(obstacle.confidence) ||
            obstacle.confidence < minimum_obstacle_confidence_)
          {
            continue;
          }
          const double delta_x =
            obstacle.x - latest_odom_->pose.pose.position.x;
          const double delta_y =
            obstacle.y - latest_odom_->pose.pose.position.y;
          const double longitudinal = cosine * delta_x + sine * delta_y;
          const double lateral = -sine * delta_x + cosine * delta_y;
          const double distance = std::hypot(longitudinal, lateral);
          if (distance < nearest_distance) {
            nearest_distance = distance;
            observation.opponent_detected = true;
            observation.opponent_longitudinal = longitudinal;
            observation.opponent_lateral = lateral;
          }
        }
      }
    }

    if (observation.trajectory_valid) {
      const double initial_lateral = latest_trajectory_->points.front().d;
      double maximum_absolute_lateral_change = 0.0;
      for (const auto & point : latest_trajectory_->points) {
        const double lateral_change = point.d - initial_lateral;
        if (finite(point.d) && finite(lateral_change) &&
          std::abs(lateral_change) >
          maximum_absolute_lateral_change)
        {
          maximum_absolute_lateral_change =
            std::abs(lateral_change);
          observation.selected_lateral = lateral_change;
        }
      }
    }

    StateCommand command;
    try {
      command = state_machine_->update(observation);
    } catch (const std::exception & exception) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "State machine update failed: %s", exception.what());
      return;
    }

    roboracer_msgs::msg::RaceState output;
    output.header.stamp = now;
    output.header.frame_id = target_frame_;
    output.state = static_cast<std::uint8_t>(command.state);
    output.preferred_side =
      static_cast<std::int8_t>(command.preferred_side);
    output.speed_scale = command.speed_scale;
    output.use_local_trajectory = command.use_local_trajectory;
    output.fallback_ftg = command.fallback_ftg;
    output.reason = command.reason;
    state_publisher_->publish(output);
    publishMarker(output, marker_x, marker_y);

    if (!last_reported_state_.has_value() ||
      *last_reported_state_ != command.state)
    {
      RCLCPP_INFO(
        get_logger(), "State -> %s, side=%s, speed=%.2f, reason=%s",
        stateName(command.state), sideName(command.preferred_side),
        command.speed_scale, command.reason.c_str());
      last_reported_state_ = command.state;
    }
  }

  void publishMarker(
    const roboracer_msgs::msg::RaceState & state, double x, double y)
  {
    visualization_msgs::msg::Marker marker;
    marker.header = state.header;
    marker.ns = "race_manager_state";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = 0.65;
    marker.pose.orientation.w = 1.0;
    marker.scale.z = 0.22;
    marker.color.a = 1.0F;
    marker.color.r = state.fallback_ftg ? 1.0F : 0.10F;
    marker.color.g = state.fallback_ftg ? 0.10F : 1.0F;
    marker.color.b = 0.20F;
    marker.text =
      std::string(stateName(static_cast<BehaviorState>(state.state))) +
      " | " + sideName(
      static_cast<PreferredSide>(state.preferred_side)) +
      " | v=" + std::to_string(state.speed_scale) +
      " | " + state.reason;
    marker_publisher_->publish(marker);
  }

  std::string target_frame_;
  double state_timeout_{0.10};
  double obstacle_timeout_{0.15};
  double trajectory_timeout_{0.15};
  double minimum_obstacle_confidence_{0.05};
  std::unique_ptr<RaceStateMachine> state_machine_;
  std::optional<BehaviorState> last_reported_state_;
  nav_msgs::msg::Odometry::ConstSharedPtr latest_odom_;
  roboracer_msgs::msg::TrackedObstacleArray::ConstSharedPtr latest_obstacles_;
  roboracer_msgs::msg::Trajectory::ConstSharedPtr latest_trajectory_;
  rclcpp::Publisher<roboracer_msgs::msg::RaceState>::SharedPtr
    state_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
    marker_publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Subscription<roboracer_msgs::msg::TrackedObstacleArray>::SharedPtr
    obstacles_subscription_;
  rclcpp::Subscription<roboracer_msgs::msg::Trajectory>::SharedPtr
    trajectory_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace race_manager

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<race_manager::RaceManagerNode>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(
      rclcpp::get_logger("race_manager"), "Fatal error: %s", exception.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
