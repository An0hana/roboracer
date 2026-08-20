// Copyright 2026 RoboRacer Team

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "state_machine/race_state_machine.hpp"
#include "rclcpp/rclcpp.hpp"
#include "roboracer_msgs/msg/race_state.hpp"
#include "roboracer_msgs/msg/tracked_obstacle.hpp"
#include "roboracer_msgs/msg/tracked_obstacle_array.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace state_machine
{
namespace
{

struct RaceLinePoint
{
  double s{0.0};
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  double curvature{0.0};
  double width_left{0.0};
  double width_right{0.0};
};

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

std::vector<RaceLinePoint> loadRaceLine(const std::string & path)
{
  std::ifstream stream(path);
  if (!stream) {
    throw std::runtime_error("cannot open race line: " + path);
  }
  std::string line;
  std::getline(stream, line);
  std::vector<RaceLinePoint> points;
  while (std::getline(stream, line)) {
    if (line.empty()) {
      continue;
    }
    std::stringstream row(line);
    std::vector<double> values;
    std::string field;
    while (std::getline(row, field, ',')) {
      values.push_back(std::stod(field));
    }
    if (values.size() < 8U) {
      throw std::runtime_error("race line row has fewer than eight columns");
    }
    RaceLinePoint point{
      values[0], values[1], values[2], values[3], values[4], values[6], values[7]};
    if (!finite(point.s) || !finite(point.x) || !finite(point.y) ||
      !finite(point.yaw) || !finite(point.curvature) ||
      !finite(point.width_left) || !finite(point.width_right) ||
      point.width_left <= 0.0 || point.width_right <= 0.0)
    {
      throw std::runtime_error("race line contains invalid values");
    }
    points.push_back(point);
  }
  if (points.size() < 3U) {
    throw std::runtime_error("race line needs at least three points");
  }
  return points;
}

const char * safetyName(SafetyState state)
{
  switch (state) {
    case SafetyState::INIT: return "INIT";
    case SafetyState::READY: return "READY";
    case SafetyState::FAULT: return "FAULT";
    case SafetyState::STOP: return "STOP";
    case SafetyState::RECOVERY: return "RECOVERY";
  }
  return "UNKNOWN";
}

bool isMppiFailure(const std::string & message)
{
  // Treat both legacy failure messages and active solver-failure creep as
  // planner-side evidence. The state machine separately requires low measured
  // progress, so a moving creep command cannot trigger reverse recovery.
  if (message.rfind("cuda_", 0U) == 0U) {
    return true;
  }
  return message == "solver_exception" || message == "solve_timeout" ||
         message == "stuck_warm_start_recovery" ||
         message == "curvature_infeasible" ||
         message == "solver_failure_recovery";
}

const char * behaviorName(BehaviorState state)
{
  switch (state) {
    case BehaviorState::RACING: return "RACING";
    case BehaviorState::TRAILING: return "TRAILING";
    case BehaviorState::OVERTAKE: return "OVERTAKE";
  }
  return "UNKNOWN";
}

const char * sideName(PreferredSide side)
{
  switch (side) {
    case PreferredSide::LEFT: return "LEFT";
    case PreferredSide::RIGHT: return "RIGHT";
    case PreferredSide::NONE: return "NONE";
  }
  return "NONE";
}

}  // namespace

class StateMachineNode : public rclcpp::Node
{
public:
  StateMachineNode()
  : Node("state_machine")
  {
    const auto obstacles_topic = declare_parameter<std::string>(
      "obstacles_topic", "/perception/obstacles");
    const auto odom_topic = declare_parameter<std::string>(
      "odom_topic", "/ego_racecar/odom");
    const auto costmap_topic = declare_parameter<std::string>(
      "costmap_topic", "/perception/local_costmap");
    const auto output_topic = declare_parameter<std::string>(
      "output_topic", "/state_machine/state");
    const auto marker_topic = declare_parameter<std::string>(
      "marker_topic", "/state_machine/state_marker");
    const auto race_line_file = declare_parameter<std::string>("race_line_file", "");
    const auto diagnostics_topic = declare_parameter<std::string>(
      "diagnostics_topic", "/diagnostics");
    diagnostics_timeout_ = declare_parameter<double>("diagnostics_timeout", 0.5);
    target_frame_ = declare_parameter<std::string>("target_frame", "map");
    state_timeout_ = declare_parameter<double>("state_timeout", 0.10);
    obstacle_timeout_ = declare_parameter<double>("obstacle_timeout", 0.20);
    costmap_timeout_ = declare_parameter<double>("costmap_timeout", 0.12);
    minimum_obstacle_confidence_ =
      declare_parameter<double>("minimum_obstacle_confidence", 0.25);
    occupied_threshold_ = declare_parameter<int>("occupied_threshold", 50);
    boundary_match_tolerance_ =
      declare_parameter<double>("boundary_match_tolerance", 0.30);
    confidence_preview_distance_ =
      declare_parameter<double>("confidence_preview_distance", 6.0);
    minimum_confidence_samples_ =
      declare_parameter<int>("minimum_confidence_samples", 8);
    corridor_clearance_radius_ =
      declare_parameter<double>("corridor_clearance_radius", 0.24);
    emergency_stop_distance_ =
      declare_parameter<double>("emergency_stop_distance", 0.55);
    confidence_filter_ = std::make_unique<TrackConfidenceFilter>(
      declare_parameter<double>("confidence_rise_time", 1.5),
      declare_parameter<double>("confidence_fall_time", 0.35), 1.0);
    const double publish_rate = declare_parameter<double>("publish_rate_hz", 20.0);

    StateMachineConfig config;
    config.follow_distance = declare_parameter<double>("follow_distance", 4.0);
    config.follow_time_headway =
      declare_parameter<double>("follow_time_headway", 1.50);
    config.maximum_follow_distance =
      declare_parameter<double>("maximum_follow_distance", 12.0);
    config.opponent_corridor_half_width =
      declare_parameter<double>("opponent_corridor_half_width", 0.80);
    config.pass_margin = declare_parameter<double>("pass_margin", 0.30);
    config.transition_confirmation =
      declare_parameter<double>("transition_confirmation", 0.20);
    config.minimum_state_duration =
      declare_parameter<double>("minimum_state_duration", 0.40);
    config.recovery_confirmation =
      declare_parameter<double>("recovery_confirmation", 0.50);
    config.opponent_lost_timeout =
      declare_parameter<double>("opponent_lost_timeout", 0.50);
    config.return_blend_duration =
      declare_parameter<double>("return_blend_duration", 1.50);
    config.overtake_lateral_offset =
      declare_parameter<double>("overtake_lateral_offset", 0.45);
    config.allow_direct_overtake =
      declare_parameter<bool>("allow_direct_overtake", true);
    overtake_lateral_offset_ = config.overtake_lateral_offset;
    config.cruise_speed_scale =
      declare_parameter<double>("cruise_speed_scale", 1.0);
    config.trailing_speed_scale =
      declare_parameter<double>("trailing_speed_scale", 0.60);
    config.overtake_speed_scale =
      declare_parameter<double>("overtake_speed_scale", 1.0);
    config.degraded_speed_scale =
      declare_parameter<double>("degraded_speed_scale", 0.55);
    config.minimum_raceline_weight_scale =
      declare_parameter<double>("minimum_raceline_weight_scale", 0.25);
    config.maximum_safety_weight_scale =
      declare_parameter<double>("maximum_safety_weight_scale", 2.0);
    config.stuck_speed_threshold =
      declare_parameter<double>("stuck_speed_threshold", 0.08);
    config.recovery_entry_time =
      declare_parameter<double>("recovery_entry_time", 0.75);
    config.failure_evidence_hold_time =
      declare_parameter<double>("failure_evidence_hold_time", 1.0);
    config.tight_curve_steering_threshold =
      declare_parameter<double>("tight_curve_steering_threshold", 0.18);
    config.tight_curve_exit_threshold =
      declare_parameter<double>("tight_curve_exit_threshold", 0.14);
    config.steering_saturation_fraction =
      declare_parameter<double>("steering_saturation_fraction", 0.85);
    config.min_steering = declare_parameter<double>("min_steering", -0.404);
    config.max_steering = declare_parameter<double>("max_steering", 0.381);
    config.wheelbase = declare_parameter<double>("wheelbase", 0.324);
    wheelbase_ = config.wheelbase;
    config.recovery_settle_time =
      declare_parameter<double>("recovery_settle_time", 0.30);
    config.reverse_speed = declare_parameter<double>("reverse_speed", 0.30);
    config.reverse_steer_sign =
      declare_parameter<double>("reverse_steer_sign", -1.0);
    config.reverse_steer_fraction =
      declare_parameter<double>("reverse_steer_fraction", 0.90);
    config.recovery_min_reverse_distance =
      declare_parameter<double>("recovery_min_reverse_distance", 0.3);
    config.recovery_target_reverse_distance =
      declare_parameter<double>("recovery_target_reverse_distance", 0.6);
    config.reverse_max_duration =
      declare_parameter<double>("reverse_max_duration", 6.0);
    config.reverse_max_distance =
      declare_parameter<double>("reverse_max_distance", 1.8);
    config.recovery_stop_speed_threshold =
      declare_parameter<double>("recovery_stop_speed_threshold", 0.05);
    config.recovery_stop_hold_time =
      declare_parameter<double>("recovery_stop_hold_time", 0.30);
    config.reentry_debounce = declare_parameter<double>("reentry_debounce", 4.0);
    // Window (in raceline points) for averaging curvature when judging whether
    // the car is stuck at a turn too sharp for the mechanical steering limit.
    // Keep in sync with MPPI's creep.curvature_window_points.
    curvature_window_points_ = declare_parameter<int>("curvature_window_points", 5);

    if (race_line_file.empty() || !finite(publish_rate) || publish_rate <= 0.0 ||
      !finite(state_timeout_) || state_timeout_ <= 0.0 ||
      !finite(obstacle_timeout_) || obstacle_timeout_ <= 0.0 ||
      !finite(costmap_timeout_) || costmap_timeout_ <= 0.0 ||
      occupied_threshold_ < 0 || occupied_threshold_ > 100 ||
      minimum_confidence_samples_ < 1 || target_frame_.empty() ||
      curvature_window_points_ < 0)
    {
      throw std::invalid_argument("invalid state machine node configuration");
    }
    race_line_ = loadRaceLine(race_line_file);
    state_machine_ = std::make_unique<RaceStateMachine>(config);

    state_publisher_ = create_publisher<roboracer_msgs::msg::RaceState>(
      output_topic, rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
    marker_publisher_ = create_publisher<visualization_msgs::msg::Marker>(
      marker_topic, rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
    obstacles_subscription_ =
      create_subscription<roboracer_msgs::msg::TrackedObstacleArray>(
      obstacles_topic, rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
      [this](roboracer_msgs::msg::TrackedObstacleArray::ConstSharedPtr message) {
        latest_obstacles_ = std::move(message);
      });
    odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, rclcpp::QoS(rclcpp::KeepLast(5)).reliable(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr message) {
        latest_odom_ = std::move(message);
      });
    costmap_subscription_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      costmap_topic, rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr message) {
        latest_costmap_ = std::move(message);
      });
    diagnostics_subscription_ =
      create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      diagnostics_topic, rclcpp::QoS(rclcpp::KeepLast(5)).reliable(),
      std::bind(&StateMachineNode::diagnosticsCallback, this, std::placeholders::_1));

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / publish_rate));
    timer_ = create_wall_timer(period, std::bind(&StateMachineNode::updateAndPublish, this));
    RCLCPP_INFO(
      get_logger(), "Hierarchical state machine: %s + %s + %s -> %s",
      odom_topic.c_str(), obstacles_topic.c_str(), costmap_topic.c_str(),
      output_topic.c_str());
  }

private:
  bool fresh(
    const builtin_interfaces::msg::Time & stamp, double timeout,
    const rclcpp::Time & now) const
  {
    const double age = (now - rclcpp::Time(stamp)).seconds();
    return finite(age) && age >= 0.0 && age <= timeout;
  }

  void diagnosticsCallback(
    const diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr message)
  {
    const double now = get_clock()->now().seconds();
    bool saw_mppi = false;
    bool saw_safety = false;
    bool mppi_fail = false;
    bool saw_mppi_steering = false;
    double mppi_steering_command = mppi_steering_command_;
    bool aeb_emergency = false;
    bool aeb_latched = false;
    for (const auto & status : message->status) {
      const std::string & name = status.name;
      if (name.find("mppi_controller") != std::string::npos &&
        name.find(": controller") != std::string::npos)
      {
        saw_mppi = true;
        mppi_fail = isMppiFailure(status.message);
        for (const auto & item : status.values) {
          if (item.key == "steering_command_target_rad") {
            try {
              const double parsed = std::stod(item.value);
              if (finite(parsed)) {
                mppi_steering_command = parsed;
                saw_mppi_steering = true;
              }
            } catch (const std::exception &) {
              // Ignore malformed third-party diagnostic values. Freshness of
              // the last valid steering value is tied to the MPPI status.
            }
          }
        }
      } else if (name == "safety_controller/arbiter") {
        saw_safety = true;
        for (const auto & item : status.values) {
          if (item.key == "aeb_emergency") {
            aeb_emergency = item.value == "true";
          } else if (item.key == "aeb_latched") {
            aeb_latched = item.value == "true";
          }
        }
      }
    }
    if (saw_mppi) {
      mppi_diag_time_ = now;
      mppi_solver_failed_ = mppi_fail;
      if (saw_mppi_steering) {
        mppi_steering_command_ = mppi_steering_command;
      }
    }
    if (saw_safety) {
      safety_diag_time_ = now;
      aeb_emergency_ = aeb_emergency;
      aeb_latched_ = aeb_latched;
    }
  }

  std::size_t nearestRaceLine(double x, double y) const
  {
    std::size_t best = 0U;
    double best_distance = std::numeric_limits<double>::infinity();
    for (std::size_t index = 0U; index < race_line_.size(); ++index) {
      const double dx = race_line_[index].x - x;
      const double dy = race_line_[index].y - y;
      const double distance = dx * dx + dy * dy;
      if (distance < best_distance) {
        best_distance = distance;
        best = index;
      }
    }
    return best;
  }

  // Signed mean curvature over the closed-loop window. Opposite bends cancel
  // instead of being misclassified as one impossible turn at an S transition.
  // Half-width is capped at the track midpoint so a large window never
  // over-samples a short line.  Mirrors MPPI's creep gate (same default window)
  // so the stuck gate and the creep gate judge the same turn segment.
  double meanCurvature(std::size_t center, std::size_t half_window) const
  {
    const std::size_t size = race_line_.size();
    if (size == 0U) {
      return 0.0;
    }
    const std::size_t half = std::min(half_window, size / 2U);
    double sum = 0.0;
    const std::size_t samples = 2U * half + 1U;
    for (std::size_t offset = size - half; offset < size; ++offset) {
      sum += race_line_[(center + offset) % size].curvature;
    }
    for (std::size_t offset = 0U; offset <= half; ++offset) {
      sum += race_line_[(center + offset) % size].curvature;
    }
    return sum / static_cast<double>(samples);
  }

  bool occupiedNear(double world_x, double world_y, double radius) const
  {
    if (!latest_costmap_ || latest_costmap_->info.resolution <= 0.0 ||
      latest_costmap_->data.empty())
    {
      return false;
    }
    const auto & map = *latest_costmap_;
    const double yaw = yawFromQuaternion(map.info.origin.orientation);
    const double dx = world_x - map.info.origin.position.x;
    const double dy = world_y - map.info.origin.position.y;
    const double local_x = std::cos(yaw) * dx + std::sin(yaw) * dy;
    const double local_y = -std::sin(yaw) * dx + std::cos(yaw) * dy;
    const int center_x = static_cast<int>(std::floor(local_x / map.info.resolution));
    const int center_y = static_cast<int>(std::floor(local_y / map.info.resolution));
    const int cells = static_cast<int>(std::ceil(radius / map.info.resolution));
    for (int y = center_y - cells; y <= center_y + cells; ++y) {
      for (int x = center_x - cells; x <= center_x + cells; ++x) {
        if (x < 0 || y < 0 || x >= static_cast<int>(map.info.width) ||
          y >= static_cast<int>(map.info.height))
        {
          continue;
        }
        const double cell_dx =
          (static_cast<double>(x) + 0.5) * map.info.resolution - local_x;
        const double cell_dy =
          (static_cast<double>(y) + 0.5) * map.info.resolution - local_y;
        if (cell_dx * cell_dx + cell_dy * cell_dy > radius * radius) {
          continue;
        }
        const auto value = map.data[
          static_cast<std::size_t>(y) * map.info.width +
          static_cast<std::size_t>(x)];
        if (value >= occupied_threshold_) {
          return true;
        }
      }
    }
    return false;
  }

  double rawTrackConfidence(std::size_t nearest) const
  {
    int considered = 0;
    int matched = 0;
    double travelled = 0.0;
    for (std::size_t offset = 0U;
      offset < race_line_.size() && travelled <= confidence_preview_distance_;
      offset += 2U)
    {
      const auto & point = race_line_[(nearest + offset) % race_line_.size()];
      if (offset > 0U) {
        const auto & previous =
          race_line_[(nearest + offset - 2U) % race_line_.size()];
        travelled += std::hypot(point.x - previous.x, point.y - previous.y);
      }
      const double nx = -std::sin(point.yaw);
      const double ny = std::cos(point.yaw);
      const double left_x = point.x + nx * point.width_left;
      const double left_y = point.y + ny * point.width_left;
      const double right_x = point.x - nx * point.width_right;
      const double right_y = point.y - ny * point.width_right;
      ++considered;
      matched += occupiedNear(left_x, left_y, boundary_match_tolerance_) ? 1 : 0;
      ++considered;
      matched += occupiedNear(right_x, right_y, boundary_match_tolerance_) ? 1 : 0;
    }
    if (considered < minimum_confidence_samples_) {
      return confidence_filter_->value();
    }
    return static_cast<double>(matched) / static_cast<double>(considered);
  }

  double corridorScore(std::size_t nearest, double signed_offset) const
  {
    int samples = 0;
    int clear = 0;
    double travelled = 0.0;
    for (std::size_t offset = 1U;
      offset < race_line_.size() && travelled <= confidence_preview_distance_;
      ++offset)
    {
      const auto & previous =
        race_line_[(nearest + offset - 1U) % race_line_.size()];
      const auto & point = race_line_[(nearest + offset) % race_line_.size()];
      travelled += std::hypot(point.x - previous.x, point.y - previous.y);
      if (travelled < 0.4) {
        continue;
      }
      const double x = point.x - std::sin(point.yaw) * signed_offset;
      const double y = point.y + std::cos(point.yaw) * signed_offset;
      ++samples;
      clear += occupiedNear(x, y, corridor_clearance_radius_) ? 0 : 1;
    }
    return samples > 0 ? static_cast<double>(clear) / samples : 0.0;
  }

  void updateAndPublish()
  {
    const rclcpp::Time now = get_clock()->now();
    StateObservation observation;
    observation.time = now.seconds();
    const bool odom_valid =
      latest_odom_ && latest_odom_->header.frame_id == target_frame_ &&
      fresh(latest_odom_->header.stamp, state_timeout_, now);
    const bool obstacles_valid =
      latest_obstacles_ && latest_obstacles_->header.frame_id == target_frame_ &&
      fresh(latest_obstacles_->header.stamp, obstacle_timeout_, now);
    const bool costmap_valid =
      latest_costmap_ && latest_costmap_->header.frame_id == target_frame_ &&
      fresh(latest_costmap_->header.stamp, costmap_timeout_, now);
    observation.inputs_ready = odom_valid && obstacles_valid && costmap_valid;

    // Planner / safety failure evidence.  The flags are latched booleans, so
    // they expire when their publisher stops reporting (freshness guard).
    const double now_seconds = now.seconds();
    observation.mppi_solver_failed =
      mppi_diag_time_ >= 0.0 &&
      now_seconds - mppi_diag_time_ <= diagnostics_timeout_ && mppi_solver_failed_;
    observation.aeb_latched =
      safety_diag_time_ >= 0.0 &&
      now_seconds - safety_diag_time_ <= diagnostics_timeout_ && aeb_latched_;
    observation.aeb_emergency =
      safety_diag_time_ >= 0.0 &&
      now_seconds - safety_diag_time_ <= diagnostics_timeout_ && aeb_emergency_;
    observation.mppi_steering_command =
      mppi_diag_time_ >= 0.0 &&
      now_seconds - mppi_diag_time_ <= diagnostics_timeout_ ?
      mppi_steering_command_ : 0.0;

    double marker_x = 0.0;
    double marker_y = 0.0;
    if (odom_valid) {
      marker_x = latest_odom_->pose.pose.position.x;
      marker_y = latest_odom_->pose.pose.position.y;
      const double ego_yaw = yawFromQuaternion(latest_odom_->pose.pose.orientation);
      // Signed speed: negative while reversing. The recovery gate uses |v|.
      observation.ego_speed = latest_odom_->twist.twist.linear.x;
      const std::size_t nearest = nearestRaceLine(marker_x, marker_y);
      // Signed window curvature rejects alternating S-bends and directly
      // provides the turn direction for reverse steering.
      const double mean_curvature = meanCurvature(
        nearest, curvature_window_points_);
      observation.required_steering = std::atan(mean_curvature * wheelbase_);
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
          const double dx = obstacle.x - marker_x;
          const double dy = obstacle.y - marker_y;
          const double longitudinal = cosine * dx + sine * dy;
          const double lateral = -sine * dx + cosine * dy;
          const double distance = std::hypot(longitudinal, lateral);
          if (distance < nearest_distance) {
            nearest_distance = distance;
            observation.opponent_detected = true;
            observation.opponent_longitudinal = longitudinal;
            observation.opponent_lateral = lateral;
            observation.opponent_longitudinal_speed =
              cosine * obstacle.vx + sine * obstacle.vy;
          }
        }
      }
      observation.emergency_stop =
        observation.opponent_detected &&
        observation.opponent_longitudinal > 0.0 &&
        observation.opponent_longitudinal < emergency_stop_distance_ &&
        std::abs(observation.opponent_lateral) < 0.5;

      if (costmap_valid) {
        const double dt = last_update_time_.has_value() ?
          std::max(0.0, (now - *last_update_time_).seconds()) : 0.0;
        observation.track_confidence =
          confidence_filter_->update(rawTrackConfidence(nearest), dt);
        observation.left_clearance_score =
          corridorScore(nearest, overtake_lateral_offset_);
        observation.right_clearance_score =
          corridorScore(nearest, -overtake_lateral_offset_);
        observation.left_available = observation.left_clearance_score >= 0.95;
        observation.right_available = observation.right_clearance_score >= 0.95;
      }
    }
    last_update_time_ = now;

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
    output.safety_state = static_cast<std::uint8_t>(command.safety_state);
    output.behavior_state = static_cast<std::uint8_t>(command.behavior_state);
    output.track_confidence = command.track_confidence;
    output.raceline_weight_scale = command.raceline_weight_scale;
    output.safety_weight_scale = command.safety_weight_scale;
    output.lateral_reference_offset = command.lateral_reference_offset;
    output.stop_requested = command.stop_requested;
    output.preferred_side = static_cast<std::int8_t>(command.preferred_side);
    output.speed_scale = command.speed_scale;
    output.use_local_trajectory = false;
    output.fallback_ftg = false;
    output.recovery_active = command.recovery_active;
    output.recovery_speed = command.recovery_speed;
    output.recovery_steering = command.recovery_steering;
    if (command.safety_state == SafetyState::INIT) {
      output.state = roboracer_msgs::msg::RaceState::INIT;
    } else if (command.safety_state == SafetyState::FAULT) {
      output.state = roboracer_msgs::msg::RaceState::FAULT;
    } else if (command.safety_state == SafetyState::STOP) {
      output.state = roboracer_msgs::msg::RaceState::STOP;
    } else if (command.safety_state == SafetyState::RECOVERY) {
      output.state = roboracer_msgs::msg::RaceState::RECOVERING;
    } else if (command.behavior_state == BehaviorState::TRAILING) {
      output.state = roboracer_msgs::msg::RaceState::TRAILING;
    } else if (command.behavior_state == BehaviorState::OVERTAKE) {
      output.state = roboracer_msgs::msg::RaceState::OVERTAKE;
    } else {
      output.state = roboracer_msgs::msg::RaceState::GLOBAL_TRACK;
    }
    output.reason = command.reason;
    state_publisher_->publish(output);
    publishMarker(output, marker_x, marker_y);

    if (!last_reported_safety_.has_value() ||
      *last_reported_safety_ != command.safety_state ||
      !last_reported_behavior_.has_value() ||
      *last_reported_behavior_ != command.behavior_state)
    {
      RCLCPP_INFO(
        get_logger(), "State -> %s/%s, side=%s, confidence=%.2f, reason=%s",
        safetyName(command.safety_state), behaviorName(command.behavior_state),
        sideName(command.preferred_side), command.track_confidence,
        command.reason.c_str());
      last_reported_safety_ = command.safety_state;
      last_reported_behavior_ = command.behavior_state;
    }
  }

  void publishMarker(
    const roboracer_msgs::msg::RaceState & state, double x, double y)
  {
    visualization_msgs::msg::Marker marker;
    marker.header = state.header;
    marker.ns = "state_machine_state";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = 0.65;
    marker.pose.orientation.w = 1.0;
    marker.scale.z = 0.20;
    marker.color.a = 1.0F;
    marker.color.r = state.stop_requested ? 1.0F : 0.10F;
    marker.color.g = state.stop_requested ? 0.10F : 1.0F;
    marker.color.b = 0.20F;
    marker.text =
      std::string(safetyName(static_cast<SafetyState>(state.safety_state))) +
      "/" + behaviorName(static_cast<BehaviorState>(state.behavior_state)) +
      " | " + sideName(static_cast<PreferredSide>(state.preferred_side)) +
      " | conf=" + std::to_string(state.track_confidence) +
      " | v=" + std::to_string(state.speed_scale) +
      " | " + state.reason;
    marker_publisher_->publish(marker);
  }

  std::string target_frame_;
  double wheelbase_{0.324};
  int curvature_window_points_{5};
  double diagnostics_timeout_{0.5};
  bool mppi_solver_failed_{false};
  bool aeb_emergency_{false};
  bool aeb_latched_{false};
  double mppi_steering_command_{0.0};
  double mppi_diag_time_{-1.0e9};
  double safety_diag_time_{-1.0e9};
  double state_timeout_{0.10};
  double obstacle_timeout_{0.20};
  double costmap_timeout_{0.12};
  double minimum_obstacle_confidence_{0.25};
  int occupied_threshold_{50};
  double boundary_match_tolerance_{0.30};
  double confidence_preview_distance_{6.0};
  int minimum_confidence_samples_{8};
  double corridor_clearance_radius_{0.24};
  double emergency_stop_distance_{0.55};
  double overtake_lateral_offset_{0.45};
  std::vector<RaceLinePoint> race_line_;
  std::unique_ptr<TrackConfidenceFilter> confidence_filter_;
  std::unique_ptr<RaceStateMachine> state_machine_;
  std::optional<SafetyState> last_reported_safety_;
  std::optional<BehaviorState> last_reported_behavior_;
  std::optional<rclcpp::Time> last_update_time_;
  nav_msgs::msg::Odometry::ConstSharedPtr latest_odom_;
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr latest_costmap_;
  roboracer_msgs::msg::TrackedObstacleArray::ConstSharedPtr latest_obstacles_;
  rclcpp::Publisher<roboracer_msgs::msg::RaceState>::SharedPtr state_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_subscription_;
  rclcpp::Subscription<roboracer_msgs::msg::TrackedObstacleArray>::SharedPtr
    obstacles_subscription_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
    diagnostics_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace state_machine

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<state_machine::StateMachineNode>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(
      rclcpp::get_logger("state_machine"), "Fatal error: %s", exception.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
