#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "mppi_controller/mppi_core.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "roboracer_msgs/msg/race_state.hpp"
#include "roboracer_msgs/msg/tracked_obstacle.hpp"
#include "roboracer_msgs/msg/tracked_obstacle_array.hpp"
#include "rclcpp/create_timer.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace mppi_controller
{

class MppiControllerNode final : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit MppiControllerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp_lifecycle::LifecycleNode("mppi_controller", options)
  {
    control_callback_group_ = create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
    sensor_callback_group_ = create_callback_group(
      rclcpp::CallbackGroupType::Reentrant);
    costmap_callback_group_ = create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
    declareParameters();
    parameter_callback_handle_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> &) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful =
          get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
        if (!result.successful) {
          result.reason = "MPPI parameters cannot change while the lifecycle node is active";
        }
        return result;
      });
  }

private:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  struct TimedState
  {
    State state{};
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    rclcpp::Time received{0, 0, RCL_ROS_TIME};
  };

  struct TimedDistanceField
  {
    std::shared_ptr<const DistanceField> field;
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    rclcpp::Time received{0, 0, RCL_ROS_TIME};
  };

  struct TimedObstacles
  {
    // time_offset stays zero in storage; the control tick stamps the actual
    // measurement age into the request copy.
    std::vector<Obstacle> obstacles;
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    rclcpp::Time received{0, 0, RCL_ROS_TIME};
  };

  struct TimedRaceState
  {
    roboracer_msgs::msg::RaceState state;
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    rclcpp::Time received{0, 0, RCL_ROS_TIME};
  };

  void declareParameters()
  {
    declare_parameter<std::string>("odom_topic", "/ego_racecar/odom");
    declare_parameter<std::string>("costmap_topic", "/perception/local_costmap");
    declare_parameter<std::string>("command_topic", "/control/mppi_cmd");
    declare_parameter<std::string>("path_topic", "/control/mppi_path");
    declare_parameter<std::string>("raceline_path_topic", "/debug/raceline");
    declare_parameter<std::string>("diagnostics_topic", "/diagnostics");
    declare_parameter<std::string>("map_frame", "map");
    declare_parameter<std::string>("base_frame", "base_link");
    declare_parameter<std::string>("race_line_file", "");
    declare_parameter<std::string>("backend", "cuda");
    declare_parameter<double>("control_frequency", 30.0);
    declare_parameter<double>("state_timeout", 0.10);
    declare_parameter<double>("costmap_timeout", 0.10);
    declare_parameter<double>("costmap_processing_frequency", 30.0);
    declare_parameter<double>("output_timeout", 0.10);
    declare_parameter<double>("tf_timeout", 0.02);
    declare_parameter<double>("max_solve_time_ms", 25.0);
    declare_parameter<double>("localization_position_jump_threshold", 0.50);
    declare_parameter<double>("localization_yaw_jump_threshold", 0.70);
    declare_parameter<double>("localization_jump_hold_time", 0.50);
    declare_parameter<double>("measured_speed_tolerance", 0.15);
    declare_parameter<double>("recovery.overspeed_entry_margin", 0.10);
    declare_parameter<double>("recovery.overspeed_exit_margin", 0.03);
    declare_parameter<double>("recovery.overspeed_exit_hold_time", 0.20);
    declare_parameter<double>("recovery.overspeed_command_reduction", 0.10);
    declare_parameter<double>("recovery.overspeed_proportional_gain", 0.50);
    declare_parameter<double>("recovery.overspeed_command_deceleration", 1.50);
    declare_parameter<double>("recovery.solver_failure_deceleration", 1.50);
    declare_parameter<double>("recovery.solver_failure_min_speed", 1.50);
    declare_parameter<double>("recovery.cached_control_max_age", 0.10);
    declare_parameter<bool>("require_costmap", true);
    declare_parameter<int>("occupied_threshold", 50);
    declare_parameter<bool>("unknown_is_occupied", false);
    declare_parameter<std::string>("obstacles_topic", "/perception/obstacles");
    // Stale opponent data degrades to costmap-only avoidance instead of
    // stopping: an empty track legitimately publishes no obstacles.
    declare_parameter<double>("obstacles_timeout", 0.20);
    declare_parameter<int>("max_obstacle_count", 8);
    declare_parameter<double>("min_obstacle_confidence", 0.25);
    declare_parameter<std::string>("race_state_topic", "/state_machine/state");
    declare_parameter<double>("race_state_timeout", 0.15);
    declare_parameter<bool>("require_race_state", false);

    declare_parameter<int>("mppi.rollout_count", 2048);
    declare_parameter<int>("mppi.horizon_steps", 72);
    declare_parameter<double>("mppi.dt", 1.0 / 30.0);
    declare_parameter<double>("mppi.lambda", 1.0);
    declare_parameter<double>("mppi.steering_rate_stddev", 0.8);
    declare_parameter<double>("mppi.acceleration_stddev", 0.8);
    declare_parameter<double>("mppi.pure_noise_fraction", 0.05);
    declare_parameter<double>("mppi.near_obstacle_distance", 0.50);
    declare_parameter<double>("mppi.near_obstacle_exploration_scale", 1.50);
    declare_parameter<double>("recovery.stuck_speed_threshold", 0.05);
    declare_parameter<double>("recovery.stuck_speed_scale_threshold", 0.25);
    declare_parameter<double>("recovery.stuck_minimum_clearance", 0.05);
    declare_parameter<double>("recovery.stuck_timeout", 1.50);
    declare_parameter<double>("recovery.stuck_cooldown", 2.00);
    declare_parameter<double>("recovery.reverse_speed", 1.00);
    declare_parameter<double>("recovery.kick_speed", 1.50);
    declare_parameter<double>("recovery.kick_duration", 0.90);
    declare_parameter<double>("recovery.kick_release_speed", 0.30);
    declare_parameter<double>("recovery.reverse_distance", 0.50);
    declare_parameter<double>("recovery.reverse_sample_step", 0.025);
    declare_parameter<double>("recovery.entry_stop_time", 0.30);
    declare_parameter<double>("recovery.initial_clearance_tolerance", 0.25);
    declare_parameter<double>("recovery.clearance_regression_tolerance", 0.025);
    declare_parameter<int>("mppi.random_seed", 7);
    declare_parameter<int>("mppi.nearest_search_radius", 80);
    declare_parameter<double>("mppi.cbf_gamma", 0.35);
    declare_parameter<double>("mppi.max_lateral_acceleration", 6.0);
    declare_parameter<double>("mppi.minimum_preview_distance", 4.0);
    declare_parameter<double>("mppi.maximum_heading_error", 1.20);
    declare_parameter<double>("mppi.reverse_progress_tolerance", 0.05);
    declare_parameter<int>("mppi.repair_steps", 4);
    declare_parameter<int>("mppi.repair_iterations", 2);
    declare_parameter<double>("mppi.repair_budget_ms", 3.0);
    declare_parameter<double>("mppi.repair_clearance", 0.05);
    declare_parameter<double>("mppi.initial_clearance_tolerance", 0.0);
    declare_parameter<int>("mppi.clearance_recovery_steps", 0);
    declare_parameter<double>("mppi.clearance_recovery_speed_threshold", 0.10);
    declare_parameter<double>(
      "mppi.clearance_recovery_acceleration_speed_threshold", 0.10);
    declare_parameter<int>("cuda.max_map_cells", 4 * 1024 * 1024);

    declare_parameter<double>("vehicle.wheelbase", 0.324);
    declare_parameter<double>("vehicle.length", 0.552);
    declare_parameter<double>("vehicle.rear_overhang", 0.124);
    declare_parameter<double>("vehicle.width", 0.320);
    declare_parameter<double>("vehicle.safety_margin", 0.05);
    declare_parameter<double>("vehicle.min_steering", -0.40);
    declare_parameter<double>("vehicle.max_steering", 0.38);
    declare_parameter<double>("vehicle.min_steering_rate", -1.5);
    declare_parameter<double>("vehicle.max_steering_rate", 1.5);
    declare_parameter<double>("vehicle.steering_response_time", 0.15);
    declare_parameter<double>("vehicle.min_effective_steering_rate", -1.20);
    declare_parameter<double>("vehicle.max_effective_steering_rate", 1.20);
    declare_parameter<double>(
      "vehicle.effective_steering_rate_speed_coefficient", 0.18);
    declare_parameter<double>("vehicle.steering_effectiveness_at_zero_speed", 1.0);
    declare_parameter<double>("vehicle.steering_effectiveness_speed_squared", 0.05);
    declare_parameter<double>("vehicle.minimum_steering_effectiveness", 0.70);
    declare_parameter<double>("steering_estimator.minimum_observation_speed", 0.50);
    declare_parameter<double>("steering_estimator.observation_gain", 0.60);
    declare_parameter<double>("vehicle.min_acceleration", -1.5);
    declare_parameter<double>("vehicle.max_acceleration", 1.0);
    declare_parameter<double>("vehicle.min_speed", 0.0);
    declare_parameter<double>("vehicle.max_speed", 2.0);

    declare_parameter<double>("weights.lateral", 12.0);
    declare_parameter<double>("weights.heading", 8.0);
    declare_parameter<double>("weights.lag", 1.0);
    declare_parameter<double>("weights.speed", 50.0);
    declare_parameter<double>("weights.progress", 8.0);
    declare_parameter<double>("weights.control", 0.15);
    declare_parameter<double>("weights.control_change", 0.4);
    declare_parameter<double>("weights.lateral_acceleration", 0.25);
    declare_parameter<double>("weights.boundary", 250.0);
    declare_parameter<double>("weights.cbf", 400.0);
    declare_parameter<double>("weights.collision", 1.0e6);
    declare_parameter<double>("weights.terminal_lateral", 80.0);
    declare_parameter<double>("weights.terminal_heading", 60.0);
    declare_parameter<double>("weights.terminal_progress", 120.0);
  }

  template<typename T>
  T parameter(const std::string & name) const
  {
    return get_parameter(name).get_value<T>();
  }

  static bool positive(double value)
  {
    return std::isfinite(value) && value > 0.0;
  }

  bool loadConfiguration(std::string & error)
  {
    try {
      race_line_file_ = parameter<std::string>("race_line_file");
      if (race_line_file_.empty()) {
        error = "race_line_file is required";
        return false;
      }
      race_line_.emplace(RaceLine::fromCsv(race_line_file_));
      requested_backend_ = parameter<std::string>("backend");

      vehicle_.wheelbase = parameter<double>("vehicle.wheelbase");
      vehicle_.length = parameter<double>("vehicle.length");
      vehicle_.rear_overhang = parameter<double>("vehicle.rear_overhang");
      vehicle_.width = parameter<double>("vehicle.width");
      vehicle_.safety_margin = parameter<double>("vehicle.safety_margin");
      vehicle_.min_steering = parameter<double>("vehicle.min_steering");
      vehicle_.max_steering = parameter<double>("vehicle.max_steering");
      vehicle_.min_steering_rate = parameter<double>("vehicle.min_steering_rate");
      vehicle_.max_steering_rate = parameter<double>("vehicle.max_steering_rate");
      vehicle_.steering_response_time =
        parameter<double>("vehicle.steering_response_time");
      vehicle_.min_effective_steering_rate =
        parameter<double>("vehicle.min_effective_steering_rate");
      vehicle_.max_effective_steering_rate =
        parameter<double>("vehicle.max_effective_steering_rate");
      vehicle_.effective_steering_rate_speed_coefficient =
        parameter<double>("vehicle.effective_steering_rate_speed_coefficient");
      vehicle_.steering_effectiveness_at_zero_speed =
        parameter<double>("vehicle.steering_effectiveness_at_zero_speed");
      vehicle_.steering_effectiveness_speed_squared =
        parameter<double>("vehicle.steering_effectiveness_speed_squared");
      vehicle_.minimum_steering_effectiveness =
        parameter<double>("vehicle.minimum_steering_effectiveness");
      steering_observation_min_speed_ =
        parameter<double>("steering_estimator.minimum_observation_speed");
      steering_observation_gain_ =
        parameter<double>("steering_estimator.observation_gain");
      if (!std::isfinite(steering_observation_min_speed_) ||
        steering_observation_min_speed_ < 0.0 ||
        !std::isfinite(steering_observation_gain_) ||
        steering_observation_gain_ < 0.0 || steering_observation_gain_ > 1.0)
      {
        error = "invalid steering estimator configuration";
        return false;
      }
      vehicle_.min_acceleration = parameter<double>("vehicle.min_acceleration");
      vehicle_.max_acceleration = parameter<double>("vehicle.max_acceleration");
      vehicle_.min_speed = parameter<double>("vehicle.min_speed");
      vehicle_.max_speed = parameter<double>("vehicle.max_speed");
      vehicle_.max_lateral_acceleration =
        parameter<double>("mppi.max_lateral_acceleration");
      model_ = std::make_unique<BicycleModel>(vehicle_);

      const int rollouts = parameter<int>("mppi.rollout_count");
      const int horizon = parameter<int>("mppi.horizon_steps");
      const int seed = parameter<int>("mppi.random_seed");
      const int search_radius = parameter<int>("mppi.nearest_search_radius");
      const int repair_steps = parameter<int>("mppi.repair_steps");
      const int repair_iterations = parameter<int>("mppi.repair_iterations");
      const int clearance_recovery_steps =
        parameter<int>("mppi.clearance_recovery_steps");
      const int cuda_max_map_cells = parameter<int>("cuda.max_map_cells");
      if (rollouts < 2 || horizon < 1 || seed < 0 || search_radius < 1 ||
        repair_steps < 0 || repair_iterations < 1 || clearance_recovery_steps < 0 ||
        clearance_recovery_steps > horizon || cuda_max_map_cells < 1)
      {
        error = "MPPI integer parameters are out of range";
        return false;
      }
      mppi_.rollout_count = static_cast<std::size_t>(rollouts);
      mppi_.horizon_steps = static_cast<std::size_t>(horizon);
      mppi_.random_seed = static_cast<std::uint32_t>(seed);
      mppi_.nearest_search_radius = static_cast<std::size_t>(search_radius);
      mppi_.repair_steps = static_cast<std::size_t>(repair_steps);
      mppi_.repair_iterations = static_cast<std::size_t>(repair_iterations);
      mppi_.clearance_recovery_steps =
        static_cast<std::size_t>(clearance_recovery_steps);
      mppi_.cuda_max_map_cells = static_cast<std::size_t>(cuda_max_map_cells);
      mppi_.dt = parameter<double>("mppi.dt");
      mppi_.lambda = parameter<double>("mppi.lambda");
      mppi_.steering_rate_stddev = parameter<double>("mppi.steering_rate_stddev");
      mppi_.acceleration_stddev = parameter<double>("mppi.acceleration_stddev");
      mppi_.pure_noise_fraction = parameter<double>("mppi.pure_noise_fraction");
      mppi_.cbf_gamma = parameter<double>("mppi.cbf_gamma");
      mppi_.max_lateral_acceleration = parameter<double>("mppi.max_lateral_acceleration");
      mppi_.minimum_preview_distance =
        parameter<double>("mppi.minimum_preview_distance");
      mppi_.maximum_heading_error =
        parameter<double>("mppi.maximum_heading_error");
      mppi_.reverse_progress_tolerance =
        parameter<double>("mppi.reverse_progress_tolerance");
      mppi_.repair_budget_ms = parameter<double>("mppi.repair_budget_ms");
      mppi_.repair_clearance = parameter<double>("mppi.repair_clearance");
      mppi_.initial_clearance_tolerance =
        parameter<double>("mppi.initial_clearance_tolerance");
      mppi_.clearance_recovery_speed_threshold =
        parameter<double>("mppi.clearance_recovery_speed_threshold");
      mppi_.clearance_recovery_acceleration_speed_threshold =
        parameter<double>("mppi.clearance_recovery_acceleration_speed_threshold");
      mppi_.weights.lateral = parameter<double>("weights.lateral");
      mppi_.weights.heading = parameter<double>("weights.heading");
      mppi_.weights.lag = parameter<double>("weights.lag");
      mppi_.weights.speed = parameter<double>("weights.speed");
      mppi_.weights.progress = parameter<double>("weights.progress");
      mppi_.weights.control = parameter<double>("weights.control");
      mppi_.weights.control_change = parameter<double>("weights.control_change");
      mppi_.weights.lateral_acceleration = parameter<double>("weights.lateral_acceleration");
      mppi_.weights.boundary = parameter<double>("weights.boundary");
      mppi_.weights.cbf = parameter<double>("weights.cbf");
      mppi_.weights.collision = parameter<double>("weights.collision");
      mppi_.weights.terminal_lateral =
        parameter<double>("weights.terminal_lateral");
      mppi_.weights.terminal_heading =
        parameter<double>("weights.terminal_heading");
      mppi_.weights.terminal_progress =
        parameter<double>("weights.terminal_progress");
      if (vehicle_.min_acceleration >= 0.0) {
        error = "vehicle.min_acceleration must provide braking";
        return false;
      }
      const double horizon_distance =
        vehicle_.max_speed * mppi_.dt * static_cast<double>(mppi_.horizon_steps);
      const double stopping_distance =
        vehicle_.max_speed * vehicle_.max_speed /
        (2.0 * std::abs(vehicle_.min_acceleration));
      if (horizon_distance <
        stopping_distance + vehicle_.length + vehicle_.safety_margin)
      {
        error = "MPPI horizon is shorter than the maximum-speed stopping requirement";
        return false;
      }

      odom_topic_ = parameter<std::string>("odom_topic");
      costmap_topic_ = parameter<std::string>("costmap_topic");
      command_topic_ = parameter<std::string>("command_topic");
      path_topic_ = parameter<std::string>("path_topic");
      raceline_path_topic_ = parameter<std::string>("raceline_path_topic");
      diagnostics_topic_ = parameter<std::string>("diagnostics_topic");
      map_frame_ = parameter<std::string>("map_frame");
      base_frame_ = parameter<std::string>("base_frame");
      control_frequency_ = parameter<double>("control_frequency");
      state_timeout_ = parameter<double>("state_timeout");
      costmap_timeout_ = parameter<double>("costmap_timeout");
      costmap_processing_frequency_ =
        parameter<double>("costmap_processing_frequency");
      output_timeout_ = parameter<double>("output_timeout");
      tf_timeout_ = parameter<double>("tf_timeout");
      max_solve_time_ms_ = parameter<double>("max_solve_time_ms");
      localization_position_jump_threshold_ =
        parameter<double>("localization_position_jump_threshold");
      localization_yaw_jump_threshold_ = parameter<double>("localization_yaw_jump_threshold");
      localization_jump_hold_time_ = parameter<double>("localization_jump_hold_time");
      measured_speed_tolerance_ = parameter<double>("measured_speed_tolerance");
      OverspeedRecoveryConfig overspeed_config;
      overspeed_config.min_speed = vehicle_.min_speed;
      overspeed_config.max_command_speed = vehicle_.max_speed;
      overspeed_config.entry_margin =
        parameter<double>("recovery.overspeed_entry_margin");
      overspeed_config.exit_margin =
        parameter<double>("recovery.overspeed_exit_margin");
      overspeed_config.exit_hold_time =
        parameter<double>("recovery.overspeed_exit_hold_time");
      overspeed_config.command_reduction =
        parameter<double>("recovery.overspeed_command_reduction");
      overspeed_config.proportional_gain =
        parameter<double>("recovery.overspeed_proportional_gain");
      overspeed_config.command_deceleration =
        parameter<double>("recovery.overspeed_command_deceleration");
      overspeed_recovery_ = OverspeedRecovery(overspeed_config);
      solver_failure_deceleration_ =
        parameter<double>("recovery.solver_failure_deceleration");
      solver_failure_min_speed_ =
        parameter<double>("recovery.solver_failure_min_speed");
      cached_control_max_age_ =
        parameter<double>("recovery.cached_control_max_age");
      require_costmap_ = parameter<bool>("require_costmap");
      occupied_threshold_ = parameter<int>("occupied_threshold");
      unknown_is_occupied_ = parameter<bool>("unknown_is_occupied");
      obstacles_topic_ = parameter<std::string>("obstacles_topic");
      obstacles_timeout_ = parameter<double>("obstacles_timeout");
      max_obstacle_count_ = parameter<int>("max_obstacle_count");
      min_obstacle_confidence_ = parameter<double>("min_obstacle_confidence");
      race_state_topic_ = parameter<std::string>("race_state_topic");
      race_state_timeout_ = parameter<double>("race_state_timeout");
      require_race_state_ = parameter<bool>("require_race_state");
      if (obstacles_topic_.empty() || !positive(obstacles_timeout_) ||
        max_obstacle_count_ < 1 || max_obstacle_count_ > 64 ||
        !std::isfinite(min_obstacle_confidence_) || min_obstacle_confidence_ < 0.0 ||
        min_obstacle_confidence_ > 1.0 || race_state_topic_.empty() ||
        !positive(race_state_timeout_))
      {
        error = "obstacle parameters are out of range";
        return false;
      }
      near_obstacle_distance_ = parameter<double>("mppi.near_obstacle_distance");
      near_obstacle_exploration_scale_ =
        parameter<double>("mppi.near_obstacle_exploration_scale");
      stuck_speed_threshold_ =
        parameter<double>("recovery.stuck_speed_threshold");
      stuck_speed_scale_threshold_ =
        parameter<double>("recovery.stuck_speed_scale_threshold");
      stuck_minimum_clearance_ =
        parameter<double>("recovery.stuck_minimum_clearance");
      stuck_timeout_ = parameter<double>("recovery.stuck_timeout");
      stuck_cooldown_ = parameter<double>("recovery.stuck_cooldown");
      recovery_reverse_speed_ = parameter<double>("recovery.reverse_speed");
      recovery_kick_speed_ = parameter<double>("recovery.kick_speed");
      recovery_kick_duration_ = parameter<double>("recovery.kick_duration");
      recovery_kick_release_speed_ =
        parameter<double>("recovery.kick_release_speed");
      recovery_reverse_distance_ = parameter<double>("recovery.reverse_distance");
      recovery_reverse_sample_step_ =
        parameter<double>("recovery.reverse_sample_step");
      recovery_entry_stop_time_ = parameter<double>("recovery.entry_stop_time");
      recovery_initial_clearance_tolerance_ =
        parameter<double>("recovery.initial_clearance_tolerance");
      recovery_clearance_regression_tolerance_ =
        parameter<double>("recovery.clearance_regression_tolerance");
      if (!positive(control_frequency_) || !positive(state_timeout_) ||
        !positive(costmap_timeout_) || !positive(costmap_processing_frequency_) ||
        !positive(output_timeout_) || !positive(tf_timeout_) || !positive(max_solve_time_ms_) ||
        !positive(localization_position_jump_threshold_) ||
        !positive(localization_yaw_jump_threshold_) || !positive(localization_jump_hold_time_) ||
        !std::isfinite(measured_speed_tolerance_) || measured_speed_tolerance_ < 0.0 ||
        !positive(solver_failure_deceleration_) ||
        !std::isfinite(solver_failure_min_speed_) || solver_failure_min_speed_ < 0.0 ||
        solver_failure_min_speed_ > vehicle_.max_speed ||
        !positive(cached_control_max_age_) ||
        !positive(near_obstacle_distance_) ||
        !std::isfinite(near_obstacle_exploration_scale_) ||
        near_obstacle_exploration_scale_ < 1.0 ||
        !positive(stuck_speed_threshold_) ||
        !std::isfinite(stuck_speed_scale_threshold_) ||
        stuck_speed_scale_threshold_ < 0.0 || stuck_speed_scale_threshold_ > 1.0 ||
        !std::isfinite(stuck_minimum_clearance_) || stuck_minimum_clearance_ < 0.0 ||
        !positive(stuck_timeout_) || !positive(stuck_cooldown_) ||
        !positive(recovery_reverse_speed_) || recovery_reverse_speed_ > 2.0 ||
        !positive(recovery_kick_speed_) || recovery_kick_speed_ > 2.0 ||
        recovery_kick_speed_ < recovery_reverse_speed_ ||
        !std::isfinite(recovery_kick_duration_) || recovery_kick_duration_ < 0.0 ||
        !std::isfinite(recovery_kick_release_speed_) ||
        recovery_kick_release_speed_ < 0.0 ||
        !positive(recovery_reverse_distance_) ||
        !positive(recovery_reverse_sample_step_) ||
        recovery_reverse_sample_step_ > recovery_reverse_distance_ ||
        !std::isfinite(recovery_entry_stop_time_) || recovery_entry_stop_time_ < 0.0 ||
        !std::isfinite(recovery_initial_clearance_tolerance_) ||
        recovery_initial_clearance_tolerance_ < 0.0 ||
        !std::isfinite(recovery_clearance_regression_tolerance_) ||
        recovery_clearance_regression_tolerance_ < 0.0 ||
        occupied_threshold_ < 0 || occupied_threshold_ > 100)
      {
        error = "node timing or occupancy parameters are out of range";
        return false;
      }
      if (raceline_path_topic_.empty()) {
        error = "raceline_path_topic must not be empty";
        return false;
      }
      backend_ = makeBackend(requested_backend_, mppi_, vehicle_);
      recovery_validator_.configure(mppi_, vehicle_);
    } catch (const std::exception & exception) {
      error = exception.what();
      return false;
    }
    return true;
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    std::string error;
    if (!loadConfiguration(error)) {
      RCLCPP_ERROR(get_logger(), "Configuration failed: %s", error.c_str());
      return CallbackReturn::FAILURE;
    }

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    command_publisher_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
      command_topic_, rclcpp::QoS(1).reliable());
    path_publisher_ = create_publisher<nav_msgs::msg::Path>(path_topic_, rclcpp::QoS(1));
    raceline_path_publisher_ = create_publisher<nav_msgs::msg::Path>(
      raceline_path_topic_, rclcpp::QoS(1).reliable().transient_local());
    diagnostic_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      diagnostics_topic_, rclcpp::QoS(10));
    if (!backend_->warmup(*race_line_, nullptr)) {
      RCLCPP_ERROR(
        get_logger(), "%s MPPI warmup failed; check CUDA runtime, race line, and vehicle bounds",
        backend_->name().c_str());
      return CallbackReturn::FAILURE;
    }
    backend_->reset();
    clearCachedTrajectory();
    last_commanded_speed_ = 0.0;
    last_commanded_steering_ = 0.0;
    last_commanded_steering_atomic_.store(0.0);
    {
      std::lock_guard<std::mutex> lock(steering_estimator_mutex_);
      estimated_effective_steering_ = 0.0;
      estimated_effective_steering_atomic_.store(0.0);
      last_steering_estimator_stamp_.reset();
      active_steering_measurement_ =
        std::numeric_limits<double>::quiet_NaN();
    }
    last_applied_control_ = Control{};
    last_command_time_.reset();
    overspeed_recovery_.reset();
    rclcpp::SubscriptionOptions sensor_options;
    sensor_options.callback_group = sensor_callback_group_;
    rclcpp::SubscriptionOptions costmap_options;
    costmap_options.callback_group = costmap_callback_group_;
    odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::QoS(10),
      std::bind(&MppiControllerNode::odomCallback, this, std::placeholders::_1),
      sensor_options);
    costmap_subscription_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      costmap_topic_, rclcpp::QoS(1).reliable(),
      std::bind(&MppiControllerNode::costmapCallback, this, std::placeholders::_1),
      costmap_options);
    obstacles_subscription_ = create_subscription<roboracer_msgs::msg::TrackedObstacleArray>(
      obstacles_topic_, rclcpp::QoS(1).reliable(),
      std::bind(&MppiControllerNode::obstaclesCallback, this, std::placeholders::_1),
      sensor_options);
    race_state_subscription_ = create_subscription<roboracer_msgs::msg::RaceState>(
      race_state_topic_, rclcpp::QoS(1).reliable(),
      std::bind(&MppiControllerNode::raceStateCallback, this, std::placeholders::_1),
      sensor_options);
    stuck_since_.reset();
    last_stuck_recovery_time_.reset();
    last_solver_failure_time_.reset();
    clearCachedTrajectory();
    recovery_behavior_active_ = false;
    recovery_behavior_enter_time_.reset();
    RCLCPP_INFO(
      get_logger(),
      "Configured %s backend: %zu rollouts x %zu steps, %.1f Hz; "
      "speed=[%.2f, %.2f] m/s steering=[%.2f, %.2f] rad",
      backend_->name().c_str(), mppi_.rollout_count, mppi_.horizon_steps, control_frequency_,
      vehicle_.min_speed, vehicle_.max_speed,
      vehicle_.min_steering, vehicle_.max_steering);
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    command_publisher_->on_activate();
    path_publisher_->on_activate();
    raceline_path_publisher_->on_activate();
    diagnostic_publisher_->on_activate();
    publishRaceLine(now());
    const auto period = rclcpp::Duration::from_seconds(1.0 / control_frequency_);
    control_timer_ = rclcpp::create_timer(
      this, get_clock(), period, std::bind(&MppiControllerNode::controlTick, this),
      control_callback_group_);
    stuck_since_.reset();
    last_stuck_recovery_time_.reset();
    last_solver_failure_time_.reset();
    clearCachedTrajectory();
    recovery_behavior_active_ = false;
    recovery_behavior_enter_time_.reset();
    last_commanded_speed_ = 0.0;
    overspeed_recovery_.reset();
    last_output_time_ = now();
    RCLCPP_INFO(get_logger(), "MPPI controller activated; output=%s", command_topic_.c_str());
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    control_timer_.reset();
    publishStop("deactivated", diagnostic_msgs::msg::DiagnosticStatus::WARN);
    command_publisher_->on_deactivate();
    path_publisher_->on_deactivate();
    raceline_path_publisher_->on_deactivate();
    diagnostic_publisher_->on_deactivate();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override
  {
    control_timer_.reset();
    odom_subscription_.reset();
    costmap_subscription_.reset();
    obstacles_subscription_.reset();
    race_state_subscription_.reset();
    command_publisher_.reset();
    path_publisher_.reset();
    raceline_path_publisher_.reset();
    diagnostic_publisher_.reset();
    tf_listener_.reset();
    tf_buffer_.reset();
    backend_.reset();
    model_.reset();
    race_line_.reset();
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      latest_state_.reset();
      latest_distance_field_.reset();
      latest_obstacles_.reset();
      latest_race_state_.reset();
      last_localization_jump_time_.reset();
    }
    stuck_since_.reset();
    last_stuck_recovery_time_.reset();
    last_solver_failure_time_.reset();
    clearCachedTrajectory();
    recovery_behavior_active_ = false;
    recovery_behavior_enter_time_.reset();
    {
      std::lock_guard<std::mutex> lock(costmap_callback_mutex_);
      last_costmap_processing_time_.reset();
    }
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override
  {
    control_timer_.reset();
    return CallbackReturn::SUCCESS;
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    geometry_msgs::msg::PoseStamped input;
    input.header = message->header;
    input.pose = message->pose.pose;
    geometry_msgs::msg::PoseStamped transformed;
    try {
      if (input.header.frame_id.empty() || input.header.frame_id == map_frame_) {
        transformed = input;
        transformed.header.frame_id = map_frame_;
      } else {
        transformed = tf_buffer_->transform(
          input, map_frame_, tf2::durationFromSec(tf_timeout_));
      }
    } catch (const tf2::TransformException & exception) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000, "Cannot transform odometry into %s: %s",
        map_frame_.c_str(), exception.what());
      return;
    }

    const rclcpp::Time received = now();
    rclcpp::Time source_stamp(message->header.stamp, get_clock()->get_clock_type());
    if (source_stamp.nanoseconds() == 0) {
      source_stamp = received;
    }
    TimedState timed;
    timed.state.x = transformed.pose.position.x;
    timed.state.y = transformed.pose.position.y;
    timed.state.yaw = tf2::getYaw(transformed.pose.orientation);
    // nav_msgs/Odometry twist is expressed in child_frame_id; preserve the signed
    // longitudinal velocity instead of turning reverse motion into positive speed.
    timed.state.speed = message->twist.twist.linear.x;
    const double steering_command = std::clamp(
      last_commanded_steering_atomic_.load(),
      vehicle_.min_steering, vehicle_.max_steering);
    const double yaw_rate = message->twist.twist.angular.z;
    {
      std::lock_guard<std::mutex> estimator_lock(steering_estimator_mutex_);
      if (!last_steering_estimator_stamp_.has_value()) {
        estimated_effective_steering_ =
          effectiveSteeringTarget(vehicle_, steering_command, timed.state.speed);
      } else {
        const double elapsed =
          (source_stamp - *last_steering_estimator_stamp_).seconds();
        if (std::isfinite(elapsed) && elapsed > 0.0 && elapsed <= 0.25) {
          State estimator_state{
            0.0, 0.0, 0.0, timed.state.speed,
            estimated_effective_steering_, steering_command};
          // The measured speed is deliberately kept unmodified in timed.state
          // so the controller's overspeed recovery can observe the real value.
          // The steering actuator model, however, requires a state inside the
          // configured vehicle envelope.  A small physical overshoot above
          // max_speed (full_lap_stop_go_04 reached 1.537 m/s for a 1.50 m/s
          // limit) must therefore be tolerance-clamped before propagation.
          // Previously this uncaught precondition violation terminated the
          // entire lifecycle node from the odometry callback.
          if (clampMeasuredSpeedWithinTolerance(
              estimator_state, vehicle_, measured_speed_tolerance_))
          {
            try {
              estimated_effective_steering_ = propagateState(
                *model_, estimator_state, Control{}, elapsed, 0.01).steering;
            } catch (const std::exception & exception) {
              RCLCPP_ERROR_THROTTLE(
                get_logger(), *get_clock(), 1000,
                "Steering estimator propagation failed; resetting estimate: %s",
                exception.what());
              estimated_effective_steering_ = effectiveSteeringTarget(
                vehicle_, steering_command, estimator_state.speed);
            }
          } else {
            const double fallback_speed = std::isfinite(timed.state.speed) ?
              std::clamp(
              timed.state.speed, vehicle_.min_speed, vehicle_.max_speed) :
              vehicle_.min_speed;
            estimated_effective_steering_ = effectiveSteeringTarget(
              vehicle_, steering_command, fallback_speed);
            RCLCPP_WARN_THROTTLE(
              get_logger(), *get_clock(), 1000,
              "Skipping steering estimator propagation: measured speed %.3f m/s "
              "is outside tolerated range [%.3f, %.3f] m/s",
              timed.state.speed,
              vehicle_.min_speed - measured_speed_tolerance_,
              vehicle_.max_speed + measured_speed_tolerance_);
          }
        }
      }
      active_steering_measurement_ =
        std::numeric_limits<double>::quiet_NaN();
      if (std::isfinite(timed.state.speed) && std::isfinite(yaw_rate) &&
        std::abs(timed.state.speed) >= steering_observation_min_speed_)
      {
        active_steering_measurement_ = std::clamp(
          std::atan(vehicle_.wheelbase * yaw_rate / timed.state.speed),
          vehicle_.min_steering, vehicle_.max_steering);
        estimated_effective_steering_ += steering_observation_gain_ *
          (active_steering_measurement_ - estimated_effective_steering_);
      }
      estimated_effective_steering_ = std::clamp(
        estimated_effective_steering_,
        vehicle_.min_steering, vehicle_.max_steering);
      estimated_effective_steering_atomic_.store(estimated_effective_steering_);
      last_steering_estimator_stamp_ = source_stamp;
      timed.state.steering = estimated_effective_steering_;
    }
    timed.state.steering_command = steering_command;
    timed.stamp = source_stamp;
    timed.received = received;
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (latest_state_.has_value() && stateWithinLimits(latest_state_->state, vehicle_) &&
      stateWithinLimits(timed.state, vehicle_))
    {
      const double elapsed = (timed.stamp - latest_state_->stamp).seconds();
      if (isLocalizationJump(
          latest_state_->state, timed.state, elapsed, vehicle_,
          localization_position_jump_threshold_, localization_yaw_jump_threshold_))
      {
        last_localization_jump_time_ = received;
      }
    }
    latest_state_ = timed;
  }

  void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr message)
  {
    std::lock_guard<std::mutex> callback_lock(costmap_callback_mutex_);
    if (!message->header.frame_id.empty() && message->header.frame_id != map_frame_) {
      RCLCPP_ERROR(
        get_logger(), "Rejected occupancy grid in frame '%s'; expected '%s'",
        message->header.frame_id.c_str(), map_frame_.c_str());
      return;
    }
    const rclcpp::Time received = now();
    if (last_costmap_processing_time_.has_value()) {
      const double elapsed = (received - *last_costmap_processing_time_).seconds();
      if (std::isfinite(elapsed) && elapsed >= 0.0 &&
        elapsed < 0.75 / costmap_processing_frequency_)
      {
        return;
      }
    }
    last_costmap_processing_time_ = received;
    try {
      auto field = std::make_shared<DistanceField>(DistanceField::fromOccupancyGrid(
          message->info.width, message->info.height, message->info.resolution,
          message->info.origin.position.x, message->info.origin.position.y,
          message->data, static_cast<std::int8_t>(occupied_threshold_),
          unknown_is_occupied_, tf2::getYaw(message->info.origin.orientation)));
      if (backend_ == nullptr || !backend_->updateDistanceField(*field)) {
        throw std::runtime_error(
                "selected MPPI backend rejected the distance field or its dimensions");
      }
      rclcpp::Time source_stamp(message->header.stamp, get_clock()->get_clock_type());
      if (source_stamp.nanoseconds() == 0) {
        source_stamp = received;
      }
      {
        std::lock_guard<std::mutex> lock(data_mutex_);
        latest_distance_field_ = TimedDistanceField{
          std::move(field), source_stamp, received};
      }
      RCLCPP_INFO_ONCE(
        get_logger(), "Local distance field ready: %ux%u at %.3f m/cell",
        message->info.width, message->info.height, message->info.resolution);
    } catch (const std::exception & exception) {
      RCLCPP_ERROR(get_logger(), "Rejected local costmap: %s", exception.what());
    }
  }

  void obstaclesCallback(const roboracer_msgs::msg::TrackedObstacleArray::SharedPtr message)
  {
    if (!message->header.frame_id.empty() && message->header.frame_id != map_frame_) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Rejected obstacle array in frame '%s'; expected '%s'",
        message->header.frame_id.c_str(), map_frame_.c_str());
      return;
    }
    const rclcpp::Time received = now();
    rclcpp::Time source_stamp(message->header.stamp, get_clock()->get_clock_type());
    if (source_stamp.nanoseconds() == 0) {
      source_stamp = received;
    }
    TimedObstacles timed;
    timed.stamp = source_stamp;
    timed.received = received;
    timed.obstacles.reserve(message->obstacles.size());
    for (const auto & tracked : message->obstacles) {
      // Track boundaries are already the costmap's job; forwarding them here
      // would double-count walls and crowd out the limited obstacle slots.
      if (tracked.classification ==
        roboracer_msgs::msg::TrackedObstacle::TRACK_BOUNDARY)
      {
        continue;
      }
      const std::array<double, 7> values{{
        tracked.x, tracked.y, tracked.yaw, tracked.vx, tracked.vy,
        tracked.length, tracked.width}};
      if (!std::all_of(
          values.begin(), values.end(),
          [](double value) {return std::isfinite(value);}) ||
        !std::isfinite(tracked.confidence) ||
        tracked.confidence < min_obstacle_confidence_)
      {
        continue;
      }
      Obstacle obstacle;
      obstacle.x = tracked.x;
      obstacle.y = tracked.y;
      obstacle.yaw = tracked.yaw;
      obstacle.vx = tracked.dynamic ? tracked.vx : 0.0;
      obstacle.vy = tracked.dynamic ? tracked.vy : 0.0;
      // Size floors keep barely-clustered detections from shrinking to a
      // point that slips between footprint cover disks.
      obstacle.half_length = std::max(tracked.length * 0.5, 0.10);
      obstacle.half_width = std::max(tracked.width * 0.5, 0.10);
      timed.obstacles.push_back(obstacle);
      if (timed.obstacles.size() >= 64U) {
        break;
      }
    }
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_obstacles_ = std::move(timed);
  }

  void raceStateCallback(const roboracer_msgs::msg::RaceState::SharedPtr message)
  {
    if (!message->header.frame_id.empty() && message->header.frame_id != map_frame_) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Rejected race state in frame '%s'; expected '%s'",
        message->header.frame_id.c_str(), map_frame_.c_str());
      return;
    }
    const rclcpp::Time received = now();
    rclcpp::Time source_stamp(message->header.stamp, get_clock()->get_clock_type());
    if (source_stamp.nanoseconds() == 0) {
      source_stamp = received;
    }
    TimedRaceState timed;
    timed.state = *message;
    timed.stamp = source_stamp;
    timed.received = received;
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_race_state_ = std::move(timed);
  }

  std::vector<State> reverseRecoveryPath(
    const State & initial_state, double reverse_speed,
    double reverse_distance) const
  {
    VehicleConfig reverse_vehicle = vehicle_;
    reverse_vehicle.min_speed = -reverse_speed;
    BicycleModel reverse_model(reverse_vehicle);
    State state = initial_state;
    state.speed = -reverse_speed;
    // Recovery first centres the front wheels while stationary.  Model the
    // subsequent escape as a straight reverse along the vehicle's current
    // longitudinal tangent instead of extending the steering angle that put
    // the vehicle against the wall.
    state.steering_command = 0.0;
    state.steering = 0.0;
    const double dt = recovery_reverse_sample_step_ / reverse_speed;
    const std::size_t sample_count = static_cast<std::size_t>(
      std::ceil(reverse_distance / recovery_reverse_sample_step_));
    std::vector<State> path;
    path.reserve(sample_count + 1U);
    path.push_back(state);
    for (std::size_t index = 0U; index < sample_count; ++index) {
      state = reverse_model.step(state, Control{}, dt);
      path.push_back(state);
    }
    return path;
  }

  bool reverseRecoveryPathSafe(
    const std::vector<State> & path, const DistanceField * distance_field,
    const std::vector<Obstacle> * obstacles, double reverse_speed) const
  {
    if (path.empty()) {
      return false;
    }
    const auto margin = [this, distance_field, obstacles](
        const State & state, double time) {
        return std::min(
          vehicleFootprintClearance(state, vehicle_, distance_field),
          obstacleClearance(state, vehicle_, obstacles, time)) - vehicle_.safety_margin;
      };
    std::vector<double> margins;
    margins.reserve(path.size());
    const double dt = recovery_reverse_sample_step_ / reverse_speed;
    for (std::size_t index = 0U; index < path.size(); ++index) {
      margins.push_back(
        margin(path[index], dt * static_cast<double>(index)));
    }
    return reverseRecoveryClearanceSafe(
      margins, recovery_initial_clearance_tolerance_,
      recovery_clearance_regression_tolerance_);
  }

  void publishRecoveryCommand(
    const MppiRequest & request, const std::vector<Obstacle> & obstacles,
    const rclcpp::Time & tick_time, double state_age, double costmap_age)
  {
    if (!recovery_behavior_active_) {
      recovery_behavior_active_ = true;
      recovery_behavior_enter_time_ = tick_time;
      recovery_reverse_progress_ = 0.0;
      recovery_progress_update_time_ = tick_time;
      safeResetBackend();
      last_commanded_speed_ = 0.0;
      last_applied_control_ = Control{};
    }

    const double elapsed = recovery_behavior_enter_time_.has_value() ?
      std::max(0.0, (tick_time - *recovery_behavior_enter_time_).seconds()) : 0.0;
    if (recovery_progress_update_time_.has_value()) {
      const double progress_dt = std::clamp(
        (tick_time - *recovery_progress_update_time_).seconds(), 0.0,
        2.0 / control_frequency_);
      recovery_reverse_progress_ = std::min(
        recovery_reverse_distance_, recovery_reverse_progress_ +
        std::max(0.0, -request.initial_state.speed) * progress_dt);
    }
    recovery_progress_update_time_ = tick_time;
    double target_speed = 0.0;
    std::string reason = "recovery_settle";
    std::vector<State> path;
    if (active_recovery_phase_ ==
      roboracer_msgs::msg::RaceState::RECOVERY_PHASE_REVERSE)
    {
      target_speed = reverseRecoveryTargetSpeed(
        elapsed, request.initial_state.speed, recovery_entry_stop_time_,
        recovery_reverse_speed_, recovery_kick_speed_, recovery_kick_duration_,
        recovery_kick_release_speed_);
      if (target_speed >= 0.0) {
        target_speed = 0.0;
        reason = "recovery_center_steering";
      } else {
        const double reverse_speed = std::abs(target_speed);
        // Validate only the distance still required in this recovery. The old
        // code planned the full distance again on every control tick, so after
        // moving 0.2 m it demanded another 0.4 m of rear clearance and could
        // chatter between safe and unsafe on adjacent costmap frames.
        const double remaining_distance = std::max(
          recovery_reverse_sample_step_,
          recovery_reverse_distance_ - recovery_reverse_progress_);
        path = reverseRecoveryPath(
          request.initial_state, reverse_speed, remaining_distance);
        if (reverseRecoveryPathSafe(
            path, request.distance_field, &obstacles, reverse_speed))
        {
          reason = reverse_speed > recovery_reverse_speed_ + 1.0e-6 ?
            "recovery_reverse_kick" : "recovery_reverse";
        } else {
          target_speed = 0.0;
          reason = "recovery_reverse_unsafe";
        }
      }
    } else if (active_recovery_phase_ !=
      roboracer_msgs::msg::RaceState::RECOVERY_PHASE_SETTLE)
    {
      reason = "recovery_phase_invalid";
    }

    State commanded_state = request.initial_state;
    commanded_state.speed = target_speed;
    commanded_state.steering_command = 0.0;
    const double command_dt = last_command_time_.has_value() ?
      std::clamp((tick_time - *last_command_time_).seconds(), 1.0e-3, 2.0 * mppi_.dt) :
      mppi_.dt;
    Control control;
    control.steering_rate = std::clamp(
      (commanded_state.steering_command - last_commanded_steering_) / command_dt,
      vehicle_.min_steering_rate, vehicle_.max_steering_rate);
    control.acceleration = std::clamp(
      (target_speed - last_commanded_speed_) / command_dt,
      vehicle_.min_acceleration, vehicle_.max_acceleration);
    publishCommand(commanded_state, control, tick_time);
    if (!path.empty()) {
      publishPath(path, tick_time);
    }
    publishDiagnostics(
      reason, diagnostic_msgs::msg::DiagnosticStatus::WARN,
      state_age, nullptr, costmap_age);
    last_commanded_speed_ = target_speed;
    last_commanded_steering_ = commanded_state.steering_command;
    last_commanded_steering_atomic_.store(last_commanded_steering_);
    last_applied_control_ = control;
    last_command_time_ = tick_time;
    last_output_time_ = tick_time;
  }

  void clearCachedTrajectory() noexcept
  {
    last_valid_control_sequence_.clear();
    last_valid_trajectory_time_.reset();
  }

  bool cachedControlForSingleFailure(
    const MppiRequest & request, const rclcpp::Time & tick_time,
    double command_dt, Control & control, State & commanded_state,
    std::vector<State> & near_states)
  {
    if (!last_valid_trajectory_time_.has_value() ||
      last_valid_control_sequence_.size() < 2U ||
      request.race_line == nullptr || !request.race_line->valid() ||
      !stateWithinLimits(request.initial_state, vehicle_))
    {
      return false;
    }
    const double cache_age = (tick_time - *last_valid_trajectory_time_).seconds();
    if (!std::isfinite(cache_age) || cache_age < 0.0 ||
      cache_age > cached_control_max_age_)
    {
      return false;
    }

    const std::vector<Control> shifted(
      last_valid_control_sequence_.begin() + 1,
      last_valid_control_sequence_.end());
    const std::size_t near_count = std::min(
      shifted.size(), std::max<std::size_t>(1U, mppi_.repair_steps));
    const std::vector<Control> near_controls(
      shifted.begin(), shifted.begin() + static_cast<std::ptrdiff_t>(near_count));
    const CostBreakdown near_cost = recovery_validator_.evaluateTrajectory(
      request.initial_state, near_controls, *request.race_line,
      request.distance_field, &near_states, request.obstacles, &request.behavior);
    if (!std::isfinite(near_cost.total()) || near_cost.collision > 0.0) {
      near_states.clear();
      return false;
    }

    control = shifted.front();
    commanded_state = model_->step(request.initial_state, control, command_dt);
    commanded_state.speed = std::clamp(
      last_commanded_speed_ + control.acceleration * command_dt,
      vehicle_.min_speed, vehicle_.max_speed);
    return true;
  }

  void controlTick()
  {
    const rclcpp::Time tick_time = now();
    active_measured_speed_ = std::numeric_limits<double>::quiet_NaN();
    active_recovery_speed_threshold_ = std::numeric_limits<double>::quiet_NaN();
    active_stuck_duration_ = 0.0;
    active_safety_state_ = roboracer_msgs::msg::RaceState::SAFETY_READY;
    active_behavior_state_ = roboracer_msgs::msg::RaceState::BEHAVIOR_RACING;
    active_recovery_phase_ = roboracer_msgs::msg::RaceState::RECOVERY_PHASE_NONE;
    active_track_confidence_ = 1.0;
    const double output_age = (tick_time - last_output_time_).seconds();
    if (!std::isfinite(output_age) || output_age < -0.02 || output_age > output_timeout_) {
      safeResetBackend();
      publishStop("output_watchdog", diagnostic_msgs::msg::DiagnosticStatus::ERROR);
      return;
    }
    std::optional<TimedState> timed_state;
    std::optional<rclcpp::Time> localization_jump_time;
    std::optional<TimedDistanceField> timed_distance_field;
    std::optional<TimedObstacles> timed_obstacles;
    std::optional<TimedRaceState> timed_race_state;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      timed_state = latest_state_;
      timed_distance_field = latest_distance_field_;
      timed_obstacles = latest_obstacles_;
      timed_race_state = latest_race_state_;
      localization_jump_time = last_localization_jump_time_;
    }
    if (!timed_state.has_value()) {
      safeResetBackend();
      publishStop("state_unavailable", diagnostic_msgs::msg::DiagnosticStatus::ERROR);
      return;
    }
    const double state_age = (tick_time - timed_state->stamp).seconds();
    if (!std::isfinite(state_age) || state_age < -0.02 || state_age > state_timeout_) {
      safeResetBackend();
      publishStop("state_stale", diagnostic_msgs::msg::DiagnosticStatus::ERROR, state_age);
      return;
    }
    if (localization_jump_time.has_value()) {
      const double jump_age = (tick_time - *localization_jump_time).seconds();
      if (!std::isfinite(jump_age) || jump_age < 0.0 ||
        jump_age < localization_jump_hold_time_)
      {
        safeResetBackend();
        publishStop("localization_jump", diagnostic_msgs::msg::DiagnosticStatus::ERROR, state_age);
        return;
      }
    }
    if (require_costmap_ && !timed_distance_field.has_value()) {
      safeResetBackend();
      publishStop("costmap_unavailable", diagnostic_msgs::msg::DiagnosticStatus::ERROR, state_age);
      return;
    }
    double costmap_age = std::numeric_limits<double>::quiet_NaN();
    std::shared_ptr<const DistanceField> distance_field;
    if (timed_distance_field.has_value()) {
      costmap_age = (tick_time - timed_distance_field->stamp).seconds();
      const double receive_age = (tick_time - timed_distance_field->received).seconds();
      if (!std::isfinite(costmap_age) || costmap_age < -0.02 ||
        costmap_age > costmap_timeout_ || !std::isfinite(receive_age) ||
        receive_age < -0.02 || receive_age > costmap_timeout_)
      {
        safeResetBackend();
        publishStop(
          "costmap_stale", diagnostic_msgs::msg::DiagnosticStatus::ERROR,
          state_age, nullptr, costmap_age);
        return;
      }
      distance_field = timed_distance_field->field;
    }

    MppiRequest request;
    request.initial_state = timed_state->state;
    active_measured_speed_ = request.initial_state.speed;
    double command_dt = mppi_.dt;
    if (last_command_time_.has_value()) {
      const double elapsed = (tick_time - *last_command_time_).seconds();
      if (std::isfinite(elapsed) && elapsed > 0.0) {
        command_dt = std::min(elapsed, 2.0 * mppi_.dt);
      }
    }
    const OverspeedRecoveryResult overspeed_result = overspeed_recovery_.update(
      active_measured_speed_, last_commanded_speed_, command_dt);
    active_overspeed_recovery_ = overspeed_result.active;
    active_overspeed_command_limit_ = overspeed_result.command_limit;
    active_overspeed_clear_duration_ = overspeed_result.clear_duration;
    active_measured_overspeed_ = overspeed_result.measured_excess;
    if (clampMeasuredSpeedWithinTolerance(
        request.initial_state, vehicle_, measured_speed_tolerance_))
    {
      try {
        request.initial_state = propagateState(
          *model_, request.initial_state, last_applied_control_,
          std::max(0.0, state_age), std::min(0.01, mppi_.dt));
      } catch (const std::exception & exception) {
        RCLCPP_ERROR(get_logger(), "State extrapolation failed: %s", exception.what());
        safeResetBackend();
        publishStop(
          "state_extrapolation_error", diagnostic_msgs::msg::DiagnosticStatus::ERROR, state_age);
        return;
      }
    }
    request.race_line = &(*race_line_);
    request.distance_field = distance_field.get();

    double race_state_age = std::numeric_limits<double>::quiet_NaN();
    if (timed_race_state.has_value()) {
      race_state_age = (tick_time - timed_race_state->stamp).seconds();
      const double receive_age = (tick_time - timed_race_state->received).seconds();
      if (!std::isfinite(race_state_age) || race_state_age < -0.02 ||
        race_state_age > race_state_timeout_ || !std::isfinite(receive_age) ||
        receive_age < -0.02 || receive_age > race_state_timeout_)
      {
        if (require_race_state_) {
          safeResetBackend();
          publishStop(
            "race_state_stale", diagnostic_msgs::msg::DiagnosticStatus::ERROR,
            state_age, nullptr, costmap_age);
          return;
        }
      } else {
        const auto & behavior = timed_race_state->state;
        if (require_race_state_ &&
          (behavior.stop_requested ||
          behavior.safety_state != roboracer_msgs::msg::RaceState::SAFETY_READY))
        {
          safeResetBackend();
          publishStop(
            "state_machine_stop:" + behavior.reason,
            diagnostic_msgs::msg::DiagnosticStatus::WARN,
            state_age, nullptr, costmap_age);
          return;
        }
        request.behavior.speed_scale =
          std::clamp(behavior.speed_scale, 0.0, 1.0);
        request.behavior.raceline_weight_scale =
          std::clamp(behavior.raceline_weight_scale, 0.05, 2.0);
        request.behavior.safety_weight_scale =
          std::clamp(behavior.safety_weight_scale, 1.0, 4.0);
        request.behavior.lateral_reference_offset =
          std::clamp(behavior.lateral_reference_offset, -1.0, 1.0);
        active_track_confidence_ =
          std::clamp(behavior.track_confidence, 0.0, 1.0);
        active_safety_state_ = behavior.safety_state;
        active_behavior_state_ = behavior.behavior_state;
        active_recovery_phase_ = behavior.recovery_phase;
      }
    } else if (require_race_state_) {
      safeResetBackend();
      publishStop(
        "race_state_unavailable", diagnostic_msgs::msg::DiagnosticStatus::ERROR,
        state_age, nullptr, costmap_age);
      return;
    }
    active_behavior_ = request.behavior;
    active_race_state_age_ = race_state_age;

    std::vector<Obstacle> request_obstacles;
    double obstacle_age = std::numeric_limits<double>::quiet_NaN();
    bool obstacles_stale = false;
    if (timed_obstacles.has_value()) {
      obstacle_age = (tick_time - timed_obstacles->stamp).seconds();
      const double receive_age = (tick_time - timed_obstacles->received).seconds();
      if (!std::isfinite(obstacle_age) || obstacle_age < -0.02 ||
        obstacle_age > obstacles_timeout_ || !std::isfinite(receive_age) ||
        receive_age < -0.02 || receive_age > obstacles_timeout_)
      {
        // Degrade to costmap-only avoidance rather than stopping: opponents
        // remain visible to the costmap as untracked occupancy.
        obstacles_stale = true;
      } else if (!timed_obstacles->obstacles.empty()) {
        request_obstacles = timed_obstacles->obstacles;
        for (Obstacle & obstacle : request_obstacles) {
          obstacle.time_offset = std::max(0.0, obstacle_age);
        }
        const std::size_t keep = static_cast<std::size_t>(max_obstacle_count_);
        if (request_obstacles.size() > keep) {
          const double ego_x = request.initial_state.x;
          const double ego_y = request.initial_state.y;
          std::partial_sort(
            request_obstacles.begin(), request_obstacles.begin() + keep,
            request_obstacles.end(),
            [ego_x, ego_y](const Obstacle & left, const Obstacle & right) {
              return std::hypot(left.x - ego_x, left.y - ego_y) <
                     std::hypot(right.x - ego_x, right.y - ego_y);
            });
          request_obstacles.resize(keep);
        }
        request.obstacles = &request_obstacles;
      }
    }

    double proximity_clearance = std::numeric_limits<double>::infinity();
    if (distance_field != nullptr) {
      proximity_clearance = distance_field->clearance(
        request.initial_state.x, request.initial_state.y);
    }
    proximity_clearance = std::min(
      proximity_clearance,
      obstacleClearance(request.initial_state, vehicle_, request.obstacles, 0.0));
    if (std::isfinite(proximity_clearance) &&
      proximity_clearance < near_obstacle_distance_)
    {
      const double proximity = std::clamp(
        (near_obstacle_distance_ - proximity_clearance) / near_obstacle_distance_,
        0.0, 1.0);
      request.exploration_scale = 1.0 +
        proximity * (near_obstacle_exploration_scale_ - 1.0);
    }

    if (active_behavior_state_ ==
      roboracer_msgs::msg::RaceState::BEHAVIOR_RECOVERY)
    {
      publishRecoveryCommand(
        request, request_obstacles, tick_time, state_age, costmap_age);
      return;
    }
    if (recovery_behavior_active_) {
      // Recovery exits only after odometry confirms the car has settled. A
      // fresh warm start avoids reusing the pre-recovery braking solution.
      safeResetBackend();
      recovery_behavior_active_ = false;
      recovery_behavior_enter_time_.reset();
      recovery_progress_update_time_.reset();
      recovery_reverse_progress_ = 0.0;
      last_commanded_speed_ = 0.0;
      last_applied_control_ = Control{};
      last_command_time_.reset();
      overspeed_recovery_.reset();
    }
    MppiResult result;
    try {
      result = backend_->compute(request);
    } catch (const std::exception & exception) {
      RCLCPP_ERROR(get_logger(), "MPPI backend exception: %s", exception.what());
      safeResetBackend();
      publishStop("solver_exception", diagnostic_msgs::msg::DiagnosticStatus::ERROR, state_age);
      return;
    } catch (...) {
      RCLCPP_ERROR(get_logger(), "MPPI backend threw a non-standard exception");
      safeResetBackend();
      publishStop("solver_exception", diagnostic_msgs::msg::DiagnosticStatus::ERROR, state_age);
      return;
    }
    if (!result.valid || result.solve_time_ms > max_solve_time_ms_) {
      const std::string reason = result.valid ? "solve_timeout" : result.reason;
      const bool single_failure = !last_solver_failure_time_.has_value();
      if (single_failure) {
        last_solver_failure_time_ = tick_time;
      }
      const double failure_duration =
        (tick_time - *last_solver_failure_time_).seconds();

      constexpr double kAebResetDuration = 0.5;
      if (failure_duration > kAebResetDuration &&
          request.initial_state.speed < 0.10) {
        safeResetBackend();
      }

      {
        State recovery_state = request.initial_state;
        Control recovery_control;
        std::vector<State> recovery_path;
        const bool used_cached_control = single_failure && cachedControlForSingleFailure(
          request, tick_time, command_dt, recovery_control, recovery_state, recovery_path);
        // A cached sequence may bridge only one failed cycle. A later success
        // installs a fresh sequence; sustained failures always decelerate.
        clearCachedTrajectory();

        if (!used_cached_control) {
          double recovery_steering_rate = 0.0;
          if (race_line_.has_value() && race_line_->valid()) {
            const TrackProjection proj = race_line_->project(
              recovery_state, std::nullopt, mppi_.nearest_search_radius);
            const Waypoint & ref = race_line_->atWrapped(
              static_cast<std::ptrdiff_t>(proj.index));
            const double lat_err = proj.lateral_error;
            const double desired_heading = normalizeAngle(
              ref.yaw - std::atan(1.5 * lat_err));
            const double heading_correction =
              normalizeAngle(desired_heading - recovery_state.yaw);
            const double lookahead =
              std::max(0.45, 0.45 + 0.25 * recovery_state.speed);
            const double desired_curvature =
              ref.curvature + 2.0 * std::sin(heading_correction) / lookahead;
            const double desired_steering = std::atan(
              vehicle_.wheelbase * desired_curvature);
            const double eff = steeringEffectiveness(
              vehicle_, recovery_state.speed);
            const double desired_command = std::clamp(
              desired_steering / std::max(eff, 0.01),
              vehicle_.min_steering, vehicle_.max_steering);
            recovery_steering_rate =
              (desired_command - recovery_state.steering_command) /
              std::max(0.15, command_dt);
            recovery_state.steering_command = std::clamp(
              recovery_state.steering_command +
              recovery_steering_rate * command_dt,
              vehicle_.min_steering, vehicle_.max_steering);
          } else {
            recovery_state.steering_command = last_commanded_steering_;
          }
          recovery_state.speed = solverFailureDeceleratedSpeed(
            last_commanded_speed_, request.initial_state.speed,
            solver_failure_deceleration_, command_dt,
            solver_failure_min_speed_, vehicle_.max_speed);
          recovery_control.steering_rate = recovery_steering_rate;
          // The velocity setpoint is anchored to the lower of command and
          // measurement, so its jump relative to the old setpoint is not a
          // physical acceleration. Report the intended physical ramp instead.
          recovery_control.acceleration =
            request.initial_state.speed > recovery_state.speed + 1.0e-6 ?
            std::clamp(
              -solver_failure_deceleration_, vehicle_.min_acceleration,
              vehicle_.max_acceleration) : 0.0;

          recovery_path.reserve(mppi_.horizon_steps + 1U);
          State predicted = recovery_state;
          recovery_path.push_back(predicted);
          for (std::size_t i = 0U; i < mppi_.horizon_steps; ++i) {
            predicted = model_->step(predicted, recovery_control, mppi_.dt);
            recovery_path.push_back(predicted);
          }
        }

        publishCommand(recovery_state, recovery_control, tick_time);
        if (!recovery_path.empty()) {
          publishPath(recovery_path, tick_time);
        }

        publishDiagnostics(
          "solver_failure_recovery",
          diagnostic_msgs::msg::DiagnosticStatus::WARN,
          state_age, &result, costmap_age);
        last_commanded_steering_ = recovery_state.steering_command;
        last_commanded_steering_atomic_.store(last_commanded_steering_);
        last_commanded_speed_ = recovery_state.speed;
        last_applied_control_ = recovery_control;
        last_command_time_ = tick_time;
        last_output_time_ = tick_time;
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "MPPI solver recovery: reason=%s mode=%s speed=%.2f m/s dur=%.1f s",
          reason.c_str(), used_cached_control ? "cached_control" : "smooth_deceleration",
          recovery_state.speed, failure_duration);
        return;
      }
    }
    // Solver succeeded — clear failure tracking.
    last_solver_failure_time_.reset();

    const double requested_target_speed =
      result.metrics.target_speed * request.behavior.speed_scale;
    active_recovery_speed_threshold_ = stuck_speed_threshold_;
    const bool stopped_despite_forward_request =
      result.reason == "ok_braking_fallback" &&
      request.initial_state.speed <= active_recovery_speed_threshold_ &&
      request.behavior.speed_scale >= stuck_speed_scale_threshold_ &&
      requested_target_speed > stuck_speed_threshold_ &&
      result.metrics.minimum_clearance >= stuck_minimum_clearance_ &&
      request_obstacles.empty();
    if (stopped_despite_forward_request) {
      if (!stuck_since_.has_value()) {
        stuck_since_ = tick_time;
      }
      const double stuck_duration = (tick_time - *stuck_since_).seconds();
      active_stuck_duration_ = std::max(0.0, stuck_duration);
      const bool cooldown_elapsed =
        !last_stuck_recovery_time_.has_value() ||
        (tick_time - *last_stuck_recovery_time_).seconds() >= stuck_cooldown_;
      if (std::isfinite(stuck_duration) && stuck_duration >= stuck_timeout_ &&
        cooldown_elapsed)
      {
        // The CUDA importance sampler can converge to a self-reinforcing
        // braking warm start. Resetting is safe: the backend seeds a
        // curvature-aware race-line tracking sequence, then still repairs and
        // validates it against the same footprint and collision constraints.
        safeResetBackend();
        last_stuck_recovery_time_ = tick_time;
        stuck_since_.reset();
        publishStop(
          "stuck_warm_start_recovery",
          diagnostic_msgs::msg::DiagnosticStatus::WARN,
          state_age, &result, costmap_age);
        // Recovery is a sampler restart, not a real braking interval. Do not
        // extrapolate the next measured state with publishStop()'s fail-safe
        // deceleration.
        last_applied_control_ = Control{};
        last_command_time_.reset();
        RCLCPP_WARN(
          get_logger(),
          "Reset CUDA warm start after %.2f s stationary under a forward request",
          stuck_duration);
        return;
      }
    } else {
      stuck_since_.reset();
      active_stuck_duration_ = 0.0;
    }

    State commanded_state = model_->step(request.initial_state, result.control, command_dt);
    // The actuator consumes a velocity setpoint while MPPI optimizes
    // acceleration. Integrate the acceleration into a persistent setpoint;
    // rebuilding it from the measured speed every cycle traps the command
    // below the motor's static-friction/minimum-ERPM threshold.
    commanded_state.speed = std::clamp(
      last_commanded_speed_ + result.control.acceleration * command_dt,
      vehicle_.min_speed, vehicle_.max_speed);
    Control applied_control = result.control;
    std::string output_reason = result.reason;
    if (overspeed_result.active) {
      commanded_state.speed = std::min(
        commanded_state.speed, overspeed_result.command_limit);
      applied_control.acceleration = std::min(
        applied_control.acceleration,
        (commanded_state.speed - last_commanded_speed_) /
        std::max(command_dt, 1.0e-6));
      output_reason = "ok_overspeed_recovery";
    }
    publishCommand(commanded_state, applied_control, tick_time);
    publishPath(result.predicted_states, tick_time);
    publishDiagnostics(
      output_reason,
      obstacles_stale ? diagnostic_msgs::msg::DiagnosticStatus::WARN :
      diagnostic_msgs::msg::DiagnosticStatus::OK,
      state_age, &result, costmap_age, request.exploration_scale,
      obstacle_age, request_obstacles.size(), obstacles_stale);
    last_commanded_steering_ = commanded_state.steering_command;
    last_commanded_steering_atomic_.store(last_commanded_steering_);
    last_commanded_speed_ = commanded_state.speed;
    last_applied_control_ = applied_control;
    last_command_time_ = tick_time;
    last_output_time_ = tick_time;
    if (result.control_sequence.size() >= 2U) {
      last_valid_control_sequence_ = result.control_sequence;
      last_valid_trajectory_time_ = tick_time;
    } else {
      clearCachedTrajectory();
    }
  }

  void safeResetBackend() noexcept
  {
    clearCachedTrajectory();
    try {
      if (backend_ != nullptr) {
        backend_->reset();
      }
    } catch (...) {
      // A reset failure must not suppress the zero-speed command.
    }
  }

  void publishCommand(const State & state, const Control & control, const rclcpp::Time & stamp)
  {
    if (command_publisher_ == nullptr || !command_publisher_->is_activated()) {
      return;
    }
    ackermann_msgs::msg::AckermannDriveStamped command;
    command.header.stamp = stamp;
    command.header.frame_id = base_frame_;
    command.drive.steering_angle = static_cast<float>(state.steering_command);
    command.drive.steering_angle_velocity = static_cast<float>(control.steering_rate);
    command.drive.speed = static_cast<float>(state.speed);
    command.drive.acceleration = static_cast<float>(control.acceleration);
    command_publisher_->publish(command);
  }

  void publishStop(
    const std::string & reason, std::uint8_t level,
    double state_age = std::numeric_limits<double>::quiet_NaN(),
    const MppiResult * result = nullptr,
    double costmap_age = std::numeric_limits<double>::quiet_NaN())
  {
    const rclcpp::Time stamp = now();
    State stopped;
    stopped.steering = estimated_effective_steering_atomic_.load();
    stopped.steering_command = last_commanded_steering_;
    publishCommand(stopped, Control{0.0, vehicle_.min_acceleration}, stamp);
    clearCachedTrajectory();
    last_commanded_speed_ = 0.0;
    last_applied_control_ = Control{0.0, vehicle_.min_acceleration};
    last_output_time_ = stamp;
    publishDiagnostics(reason, level, state_age, result, costmap_age);
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000, "MPPI stop: %s", reason.c_str());
  }

  void publishPath(const std::vector<State> & states, const rclcpp::Time & stamp)
  {
    if (path_publisher_ == nullptr || !path_publisher_->is_activated()) {
      return;
    }
    nav_msgs::msg::Path path;
    path.header.stamp = stamp;
    path.header.frame_id = map_frame_;
    path.poses.reserve(states.size());
    for (const State & state : states) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = state.x;
      pose.pose.position.y = state.y;
      pose.pose.orientation.z = std::sin(state.yaw * 0.5);
      pose.pose.orientation.w = std::cos(state.yaw * 0.5);
      path.poses.push_back(pose);
    }
    path_publisher_->publish(path);
  }

  void publishRaceLine(const rclcpp::Time & stamp)
  {
    if (raceline_path_publisher_ == nullptr ||
      !raceline_path_publisher_->is_activated() ||
      !race_line_.has_value() || !race_line_->valid())
    {
      return;
    }

    nav_msgs::msg::Path path;
    path.header.stamp = stamp;
    path.header.frame_id = map_frame_;
    const auto & waypoints = race_line_->waypoints();
    path.poses.reserve(waypoints.size() + 1U);
    for (const Waypoint & waypoint : waypoints) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = waypoint.x;
      pose.pose.position.y = waypoint.y;
      pose.pose.orientation.z = std::sin(waypoint.yaw * 0.5);
      pose.pose.orientation.w = std::cos(waypoint.yaw * 0.5);
      path.poses.push_back(pose);
    }
    // RaceLine is cyclic; close the loop explicitly for RViz Path rendering.
    if (!path.poses.empty()) {
      path.poses.push_back(path.poses.front());
    }
    raceline_path_publisher_->publish(path);
  }

  static diagnostic_msgs::msg::KeyValue diagnosticValue(
    const std::string & key, double value)
  {
    diagnostic_msgs::msg::KeyValue output;
    output.key = key;
    output.value = std::to_string(value);
    return output;
  }

  void publishDiagnostics(
    const std::string & reason, std::uint8_t level, double state_age,
    const MppiResult * result,
    double costmap_age = std::numeric_limits<double>::quiet_NaN(),
    double exploration_scale = 1.0,
    double obstacle_age = std::numeric_limits<double>::quiet_NaN(),
    std::size_t obstacle_count = 0U, bool obstacles_stale = false)
  {
    if (diagnostic_publisher_ == nullptr || !diagnostic_publisher_->is_activated()) {
      return;
    }
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = std::string(get_namespace()) + "/" + get_name() + ": controller";
    status.hardware_id = backend_ == nullptr ? "none" : backend_->name();
    status.level = level;
    status.message = reason;
    diagnostic_msgs::msg::KeyValue requested_backend;
    requested_backend.key = "backend_requested";
    requested_backend.value = requested_backend_;
    status.values.push_back(std::move(requested_backend));
    diagnostic_msgs::msg::KeyValue selected_backend;
    selected_backend.key = "backend_selected";
    selected_backend.value = backend_ == nullptr ? "none" : backend_->name();
    status.values.push_back(std::move(selected_backend));
    diagnostic_msgs::msg::KeyValue cuda_compiled;
    cuda_compiled.key = "cuda_backend_compiled";
    cuda_compiled.value = cudaBackendCompiled() ? "true" : "false";
    status.values.push_back(std::move(cuda_compiled));
    status.values.push_back(diagnosticValue("state_age_s", state_age));
    status.values.push_back(diagnosticValue("costmap_age_s", costmap_age));
    status.values.push_back(diagnosticValue("obstacle_age_s", obstacle_age));
    status.values.push_back(diagnosticValue(
      "obstacle_count", static_cast<double>(obstacle_count)));
    diagnostic_msgs::msg::KeyValue stale_value;
    stale_value.key = "obstacles_stale";
    stale_value.value = obstacles_stale ? "true" : "false";
    status.values.push_back(std::move(stale_value));
    status.values.push_back(diagnosticValue("exploration_scale", exploration_scale));
    status.values.push_back(diagnosticValue(
      "measured_speed", active_measured_speed_));
    status.values.push_back(diagnosticValue(
      "estimated_effective_steering_rad",
      estimated_effective_steering_atomic_.load()));
    status.values.push_back(diagnosticValue(
      "steering_command_target_rad",
      last_commanded_steering_atomic_.load()));
    diagnostic_msgs::msg::KeyValue overspeed_active;
    overspeed_active.key = "overspeed_recovery_active";
    overspeed_active.value = active_overspeed_recovery_ ? "true" : "false";
    status.values.push_back(std::move(overspeed_active));
    status.values.push_back(diagnosticValue(
      "measured_overspeed_mps", active_measured_overspeed_));
    status.values.push_back(diagnosticValue(
      "overspeed_command_limit_mps", active_overspeed_command_limit_));
    status.values.push_back(diagnosticValue(
      "overspeed_clear_duration_s", active_overspeed_clear_duration_));
    status.values.push_back(diagnosticValue(
      "recovery_speed_threshold", active_recovery_speed_threshold_));
    status.values.push_back(diagnosticValue(
      "recovery_stagnation_age_s", active_stuck_duration_));
    status.values.push_back(diagnosticValue(
      "recovery_reverse_progress_m", recovery_reverse_progress_));
    status.values.push_back(diagnosticValue(
      "race_state_age_s", active_race_state_age_));
    status.values.push_back(diagnosticValue(
      "track_confidence", active_track_confidence_));
    status.values.push_back(diagnosticValue(
      "safety_state", static_cast<double>(active_safety_state_)));
    status.values.push_back(diagnosticValue(
      "behavior_state", static_cast<double>(active_behavior_state_)));
    status.values.push_back(diagnosticValue(
      "recovery_phase", static_cast<double>(active_recovery_phase_)));
    status.values.push_back(diagnosticValue(
      "behavior_speed_scale", active_behavior_.speed_scale));
    status.values.push_back(diagnosticValue(
      "raceline_weight_scale", active_behavior_.raceline_weight_scale));
    status.values.push_back(diagnosticValue(
      "safety_weight_scale", active_behavior_.safety_weight_scale));
    status.values.push_back(diagnosticValue(
      "lateral_reference_offset", active_behavior_.lateral_reference_offset));
    status.values.push_back(diagnosticValue(
      "output_age_s", (now() - last_output_time_).seconds()));
    if (result != nullptr) {
      status.values.push_back(diagnosticValue("solve_time_ms", result->solve_time_ms));
      status.values.push_back(diagnosticValue("cost_total", result->cost.total()));
      status.values.push_back(diagnosticValue("cost_lateral", result->cost.lateral));
      status.values.push_back(diagnosticValue("cost_heading", result->cost.heading));
      status.values.push_back(diagnosticValue("cost_lag", result->cost.lag));
      status.values.push_back(diagnosticValue("cost_speed", result->cost.speed));
      status.values.push_back(diagnosticValue("cost_progress", result->cost.progress));
      status.values.push_back(diagnosticValue("cost_control", result->cost.control));
      status.values.push_back(diagnosticValue(
        "cost_control_change", result->cost.control_change));
      status.values.push_back(diagnosticValue(
        "cost_lateral_acceleration", result->cost.lateral_acceleration));
      status.values.push_back(diagnosticValue("cost_boundary", result->cost.boundary));
      status.values.push_back(diagnosticValue("cost_cbf", result->cost.cbf));
      status.values.push_back(diagnosticValue("cost_collision", result->cost.collision));
      status.values.push_back(diagnosticValue(
        "cost_terminal_lateral", result->cost.terminal_lateral));
      status.values.push_back(diagnosticValue(
        "cost_terminal_heading", result->cost.terminal_heading));
      status.values.push_back(diagnosticValue(
        "cost_terminal_progress", result->cost.terminal_progress));
      status.values.push_back(diagnosticValue(
        "rollout_count", static_cast<double>(result->rollout_count)));
      diagnostic_msgs::msg::KeyValue valid_rollouts;
      valid_rollouts.key = "valid_rollouts";
      valid_rollouts.value = result->valid_rollouts.has_value() ?
        std::to_string(*result->valid_rollouts) : "not_exposed_by_cuda_backend";
      status.values.push_back(std::move(valid_rollouts));
      diagnostic_msgs::msg::KeyValue valid_rollout_ratio;
      valid_rollout_ratio.key = "valid_rollout_ratio";
      if (result->valid_rollouts.has_value() && result->rollout_count > 0U) {
        valid_rollout_ratio.value = std::to_string(
          static_cast<double>(*result->valid_rollouts) /
          static_cast<double>(result->rollout_count));
      } else {
        valid_rollout_ratio.value = "not_exposed_by_cuda_backend";
      }
      status.values.push_back(std::move(valid_rollout_ratio));
      status.values.push_back(diagnosticValue(
        "horizon_time_s",
        mppi_.dt * static_cast<double>(mppi_.horizon_steps)));
      status.values.push_back(diagnosticValue(
        "target_speed", result->metrics.target_speed));
      status.values.push_back(diagnosticValue(
        "preview_target_m", result->metrics.preview_target));
      status.values.push_back(diagnosticValue(
        "predicted_distance_m", result->metrics.predicted_distance));
      status.values.push_back(diagnosticValue(
        "forward_progress_m", result->metrics.forward_progress));
      status.values.push_back(diagnosticValue(
        "maximum_heading_error_rad", result->metrics.maximum_heading_error));
      status.values.push_back(diagnosticValue(
        "minimum_obstacle_clearance_m",
        result->metrics.minimum_obstacle_clearance));
      status.values.push_back(diagnosticValue(
        "minimum_clearance_m", result->metrics.minimum_clearance));
      status.values.push_back(diagnosticValue(
        "stopping_distance_m", result->metrics.stopping_distance));
      status.values.push_back(diagnosticValue(
        "reverse_steps", static_cast<double>(result->metrics.reverse_steps)));
    }
    array.status.push_back(std::move(status));
    diagnostic_publisher_->publish(array);
  }

  MppiConfig mppi_{};
  VehicleConfig vehicle_{};
  std::optional<RaceLine> race_line_;
  std::unique_ptr<MppiBackend> backend_;
  std::unique_ptr<BicycleModel> model_;
  std::string race_line_file_;
  std::string requested_backend_{"cuda"};
  std::string odom_topic_;
  std::string costmap_topic_;
  std::string command_topic_;
  std::string path_topic_;
  std::string raceline_path_topic_;
  std::string diagnostics_topic_;
  std::string map_frame_;
  std::string base_frame_;
  double control_frequency_{30.0};
  double state_timeout_{0.10};
  double costmap_timeout_{0.10};
  double costmap_processing_frequency_{30.0};
  double output_timeout_{0.10};
  double tf_timeout_{0.02};
  double max_solve_time_ms_{25.0};
  double localization_position_jump_threshold_{0.50};
  double localization_yaw_jump_threshold_{0.70};
  double localization_jump_hold_time_{0.50};
  double measured_speed_tolerance_{0.15};
  double solver_failure_deceleration_{1.50};
  double solver_failure_min_speed_{1.50};
  double cached_control_max_age_{0.10};
  double steering_observation_min_speed_{0.50};
  double steering_observation_gain_{0.60};
  OverspeedRecovery overspeed_recovery_{};
  CpuMppiBackend recovery_validator_{};
  bool require_costmap_{true};
  int occupied_threshold_{50};
  bool unknown_is_occupied_{true};
  std::string obstacles_topic_;
  double obstacles_timeout_{0.20};
  int max_obstacle_count_{8};
  double min_obstacle_confidence_{0.25};
  std::string race_state_topic_;
  double race_state_timeout_{0.15};
  bool require_race_state_{false};
  double near_obstacle_distance_{0.50};
  double near_obstacle_exploration_scale_{1.50};
  double stuck_speed_threshold_{0.05};
  double stuck_speed_scale_threshold_{0.25};
  double stuck_minimum_clearance_{0.05};
  double stuck_timeout_{1.50};
  double stuck_cooldown_{2.00};
  double recovery_reverse_speed_{1.00};
  double recovery_kick_speed_{1.50};
  double recovery_kick_duration_{0.90};
  double recovery_kick_release_speed_{0.30};
  double recovery_reverse_distance_{0.50};
  double recovery_reverse_sample_step_{0.025};
  double recovery_entry_stop_time_{0.30};
  double recovery_initial_clearance_tolerance_{0.25};
  double recovery_clearance_regression_tolerance_{0.025};
  double active_recovery_speed_threshold_{
    std::numeric_limits<double>::quiet_NaN()};
  double active_measured_speed_{
    std::numeric_limits<double>::quiet_NaN()};
  bool active_overspeed_recovery_{false};
  double active_measured_overspeed_{0.0};
  double active_overspeed_command_limit_{
    std::numeric_limits<double>::infinity()};
  double active_overspeed_clear_duration_{0.0};
  double active_stuck_duration_{0.0};
  MppiBehavior active_behavior_{};
  double active_race_state_age_{std::numeric_limits<double>::quiet_NaN()};
  double active_track_confidence_{1.0};
  std::uint8_t active_safety_state_{
    roboracer_msgs::msg::RaceState::SAFETY_READY};
  std::uint8_t active_behavior_state_{
    roboracer_msgs::msg::RaceState::BEHAVIOR_RACING};
  std::uint8_t active_recovery_phase_{
    roboracer_msgs::msg::RaceState::RECOVERY_PHASE_NONE};
  bool recovery_behavior_active_{false};
  std::optional<rclcpp::Time> recovery_behavior_enter_time_;
  std::optional<rclcpp::Time> recovery_progress_update_time_;
  double recovery_reverse_progress_{0.0};

  std::mutex data_mutex_;
  std::mutex costmap_callback_mutex_;
  std::mutex steering_estimator_mutex_;
  std::optional<TimedState> latest_state_;
  std::optional<TimedDistanceField> latest_distance_field_;
  std::optional<TimedObstacles> latest_obstacles_;
  std::optional<TimedRaceState> latest_race_state_;
  std::optional<rclcpp::Time> last_localization_jump_time_;
  std::optional<rclcpp::Time> last_costmap_processing_time_;
  std::optional<rclcpp::Time> stuck_since_;
  std::optional<rclcpp::Time> last_stuck_recovery_time_;
  std::optional<rclcpp::Time> last_solver_failure_time_;
  std::vector<Control> last_valid_control_sequence_;
  std::optional<rclcpp::Time> last_valid_trajectory_time_;
  double last_commanded_speed_{0.0};
  double last_commanded_steering_{0.0};
  std::atomic<double> last_commanded_steering_atomic_{0.0};
  double estimated_effective_steering_{0.0};
  std::atomic<double> estimated_effective_steering_atomic_{0.0};
  double active_steering_measurement_{
    std::numeric_limits<double>::quiet_NaN()};
  std::optional<rclcpp::Time> last_steering_estimator_stamp_;
  Control last_applied_control_{};
  std::optional<rclcpp::Time> last_command_time_;
  rclcpp::Time last_output_time_{0, 0, RCL_ROS_TIME};

  rclcpp::CallbackGroup::SharedPtr control_callback_group_;
  rclcpp::CallbackGroup::SharedPtr sensor_callback_group_;
  rclcpp::CallbackGroup::SharedPtr costmap_callback_group_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_subscription_;
  rclcpp::Subscription<roboracer_msgs::msg::TrackedObstacleArray>::SharedPtr
    obstacles_subscription_;
  rclcpp::Subscription<roboracer_msgs::msg::RaceState>::SharedPtr
    race_state_subscription_;
  rclcpp_lifecycle::LifecyclePublisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr
    command_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr
    raceline_path_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
    diagnostic_publisher_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
};

}  // namespace mppi_controller

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<mppi_controller::MppiControllerNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
