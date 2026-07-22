#include <algorithm>
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
#include "race_mppi/mppi_core.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/create_timer.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace race_mppi
{

class MppiControllerNode final : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit MppiControllerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp_lifecycle::LifecycleNode("mppi_controller", options)
  {
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

  void declareParameters()
  {
    declare_parameter<std::string>("odom_topic", "/state_estimation/odom");
    declare_parameter<std::string>("map_topic", "/map");
    declare_parameter<std::string>("command_topic", "/control/mppi_cmd");
    declare_parameter<std::string>("path_topic", "/control/mppi_path");
    declare_parameter<std::string>("diagnostics_topic", "/diagnostics");
    declare_parameter<std::string>("map_frame", "map");
    declare_parameter<std::string>("base_frame", "base_link");
    declare_parameter<std::string>("race_line_file", "");
    declare_parameter<std::string>("backend", "auto");
    declare_parameter<double>("control_frequency", 20.0);
    declare_parameter<double>("state_timeout", 0.10);
    declare_parameter<double>("output_timeout", 0.10);
    declare_parameter<double>("tf_timeout", 0.02);
    declare_parameter<double>("max_solve_time_ms", 40.0);
    declare_parameter<double>("localization_position_jump_threshold", 0.50);
    declare_parameter<double>("localization_yaw_jump_threshold", 0.70);
    declare_parameter<double>("localization_jump_hold_time", 0.50);
    declare_parameter<bool>("require_map", true);
    declare_parameter<int>("occupied_threshold", 50);
    declare_parameter<bool>("unknown_is_occupied", true);

    declare_parameter<int>("mppi.rollout_count", 2048);
    declare_parameter<int>("mppi.horizon_steps", 32);
    declare_parameter<double>("mppi.dt", 0.05);
    declare_parameter<double>("mppi.lambda", 1.0);
    declare_parameter<double>("mppi.steering_rate_stddev", 0.8);
    declare_parameter<double>("mppi.acceleration_stddev", 0.8);
    declare_parameter<int>("mppi.random_seed", 7);
    declare_parameter<int>("mppi.nearest_search_radius", 80);
    declare_parameter<double>("mppi.cbf_gamma", 0.35);
    declare_parameter<double>("mppi.max_lateral_acceleration", 4.0);
    declare_parameter<int>("mppi.repair_steps", 4);
    declare_parameter<int>("mppi.repair_iterations", 2);
    declare_parameter<double>("mppi.repair_budget_ms", 3.0);
    declare_parameter<double>("mppi.repair_clearance", 0.05);
    declare_parameter<int>("cuda.max_map_cells", 4 * 1024 * 1024);

    declare_parameter<double>("vehicle.wheelbase", 0.324);
    declare_parameter<double>("vehicle.length", 0.568);
    declare_parameter<double>("vehicle.rear_overhang", 0.100);
    declare_parameter<double>("vehicle.width", 0.296);
    declare_parameter<double>("vehicle.safety_margin", 0.05);
    declare_parameter<double>("vehicle.min_steering", -0.20);
    declare_parameter<double>("vehicle.max_steering", 0.20);
    declare_parameter<double>("vehicle.min_steering_rate", -1.5);
    declare_parameter<double>("vehicle.max_steering_rate", 1.5);
    declare_parameter<double>("vehicle.min_acceleration", -1.5);
    declare_parameter<double>("vehicle.max_acceleration", 1.0);
    declare_parameter<double>("vehicle.min_speed", 0.0);
    declare_parameter<double>("vehicle.max_speed", 2.0);

    declare_parameter<double>("weights.lateral", 12.0);
    declare_parameter<double>("weights.heading", 3.0);
    declare_parameter<double>("weights.lag", 1.0);
    declare_parameter<double>("weights.speed", 2.0);
    declare_parameter<double>("weights.progress", 4.0);
    declare_parameter<double>("weights.control", 0.15);
    declare_parameter<double>("weights.control_change", 0.4);
    declare_parameter<double>("weights.lateral_acceleration", 0.25);
    declare_parameter<double>("weights.boundary", 250.0);
    declare_parameter<double>("weights.cbf", 400.0);
    declare_parameter<double>("weights.collision", 1.0e6);
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
      vehicle_.min_acceleration = parameter<double>("vehicle.min_acceleration");
      vehicle_.max_acceleration = parameter<double>("vehicle.max_acceleration");
      vehicle_.min_speed = parameter<double>("vehicle.min_speed");
      vehicle_.max_speed = parameter<double>("vehicle.max_speed");
      model_ = std::make_unique<BicycleModel>(vehicle_);

      const int rollouts = parameter<int>("mppi.rollout_count");
      const int horizon = parameter<int>("mppi.horizon_steps");
      const int seed = parameter<int>("mppi.random_seed");
      const int search_radius = parameter<int>("mppi.nearest_search_radius");
      const int repair_steps = parameter<int>("mppi.repair_steps");
      const int repair_iterations = parameter<int>("mppi.repair_iterations");
      const int cuda_max_map_cells = parameter<int>("cuda.max_map_cells");
      if (rollouts < 2 || horizon < 1 || seed < 0 || search_radius < 1 ||
        repair_steps < 0 || repair_iterations < 1 || cuda_max_map_cells < 1)
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
      mppi_.cuda_max_map_cells = static_cast<std::size_t>(cuda_max_map_cells);
      mppi_.dt = parameter<double>("mppi.dt");
      mppi_.lambda = parameter<double>("mppi.lambda");
      mppi_.steering_rate_stddev = parameter<double>("mppi.steering_rate_stddev");
      mppi_.acceleration_stddev = parameter<double>("mppi.acceleration_stddev");
      mppi_.cbf_gamma = parameter<double>("mppi.cbf_gamma");
      mppi_.max_lateral_acceleration = parameter<double>("mppi.max_lateral_acceleration");
      mppi_.repair_budget_ms = parameter<double>("mppi.repair_budget_ms");
      mppi_.repair_clearance = parameter<double>("mppi.repair_clearance");
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

      odom_topic_ = parameter<std::string>("odom_topic");
      map_topic_ = parameter<std::string>("map_topic");
      command_topic_ = parameter<std::string>("command_topic");
      path_topic_ = parameter<std::string>("path_topic");
      diagnostics_topic_ = parameter<std::string>("diagnostics_topic");
      map_frame_ = parameter<std::string>("map_frame");
      base_frame_ = parameter<std::string>("base_frame");
      control_frequency_ = parameter<double>("control_frequency");
      state_timeout_ = parameter<double>("state_timeout");
      output_timeout_ = parameter<double>("output_timeout");
      tf_timeout_ = parameter<double>("tf_timeout");
      max_solve_time_ms_ = parameter<double>("max_solve_time_ms");
      localization_position_jump_threshold_ =
        parameter<double>("localization_position_jump_threshold");
      localization_yaw_jump_threshold_ = parameter<double>("localization_yaw_jump_threshold");
      localization_jump_hold_time_ = parameter<double>("localization_jump_hold_time");
      require_map_ = parameter<bool>("require_map");
      occupied_threshold_ = parameter<int>("occupied_threshold");
      unknown_is_occupied_ = parameter<bool>("unknown_is_occupied");
      if (!positive(control_frequency_) || !positive(state_timeout_) ||
        !positive(output_timeout_) || !positive(tf_timeout_) || !positive(max_solve_time_ms_) ||
        !positive(localization_position_jump_threshold_) ||
        !positive(localization_yaw_jump_threshold_) || !positive(localization_jump_hold_time_) ||
        occupied_threshold_ < 0 || occupied_threshold_ > 100)
      {
        error = "node timing or occupancy parameters are out of range";
        return false;
      }
      backend_ = makeBackend(requested_backend_, mppi_, vehicle_);
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
    diagnostic_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      diagnostics_topic_, rclcpp::QoS(10));
    odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::QoS(10),
      std::bind(&MppiControllerNode::odomCallback, this, std::placeholders::_1));
    map_subscription_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic_, rclcpp::QoS(1).transient_local().reliable(),
      std::bind(&MppiControllerNode::mapCallback, this, std::placeholders::_1));

    if (!backend_->warmup(*race_line_, nullptr)) {
      RCLCPP_ERROR(
        get_logger(), "%s MPPI warmup failed; check CUDA runtime, race line, and vehicle bounds",
        backend_->name().c_str());
      return CallbackReturn::FAILURE;
    }
    backend_->reset();
    RCLCPP_INFO(
      get_logger(), "Configured %s backend: %zu rollouts x %zu steps, %.1f Hz",
      backend_->name().c_str(), mppi_.rollout_count, mppi_.horizon_steps, control_frequency_);
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    command_publisher_->on_activate();
    path_publisher_->on_activate();
    diagnostic_publisher_->on_activate();
    const auto period = rclcpp::Duration::from_seconds(1.0 / control_frequency_);
    control_timer_ = rclcpp::create_timer(
      this, get_clock(), period, std::bind(&MppiControllerNode::controlTick, this));
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
    diagnostic_publisher_->on_deactivate();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override
  {
    control_timer_.reset();
    odom_subscription_.reset();
    map_subscription_.reset();
    command_publisher_.reset();
    path_publisher_.reset();
    diagnostic_publisher_.reset();
    tf_listener_.reset();
    tf_buffer_.reset();
    backend_.reset();
    model_.reset();
    race_line_.reset();
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      latest_state_.reset();
      distance_field_.reset();
      last_localization_jump_time_.reset();
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
    timed.state.steering = last_commanded_steering_;
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

  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr message)
  {
    if (!message->header.frame_id.empty() && message->header.frame_id != map_frame_) {
      RCLCPP_ERROR(
        get_logger(), "Rejected occupancy grid in frame '%s'; expected '%s'",
        message->header.frame_id.c_str(), map_frame_.c_str());
      return;
    }
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
      {
        std::lock_guard<std::mutex> lock(data_mutex_);
        distance_field_ = std::move(field);
      }
      RCLCPP_INFO(
        get_logger(), "Distance field ready: %ux%u at %.3f m/cell",
        message->info.width, message->info.height, message->info.resolution);
    } catch (const std::exception & exception) {
      RCLCPP_ERROR(get_logger(), "Rejected occupancy grid: %s", exception.what());
    }
  }

  void controlTick()
  {
    const rclcpp::Time tick_time = now();
    const double output_age = (tick_time - last_output_time_).seconds();
    if (!std::isfinite(output_age) || output_age < -0.02 || output_age > output_timeout_) {
      safeResetBackend();
      publishStop("output_watchdog", diagnostic_msgs::msg::DiagnosticStatus::ERROR);
      return;
    }
    std::optional<TimedState> timed_state;
    std::optional<rclcpp::Time> localization_jump_time;
    std::shared_ptr<const DistanceField> distance_field;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      timed_state = latest_state_;
      distance_field = distance_field_;
      localization_jump_time = last_localization_jump_time_;
    }
    if (!timed_state.has_value()) {
      publishStop("state_unavailable", diagnostic_msgs::msg::DiagnosticStatus::ERROR);
      return;
    }
    const double state_age = (tick_time - timed_state->stamp).seconds();
    if (!std::isfinite(state_age) || state_age < -0.02 || state_age > state_timeout_) {
      publishStop("state_stale", diagnostic_msgs::msg::DiagnosticStatus::ERROR, state_age);
      return;
    }
    if (localization_jump_time.has_value()) {
      const double jump_age = (tick_time - *localization_jump_time).seconds();
      if (!std::isfinite(jump_age) || jump_age < 0.0 ||
        jump_age < localization_jump_hold_time_)
      {
        publishStop("localization_jump", diagnostic_msgs::msg::DiagnosticStatus::ERROR, state_age);
        return;
      }
    }
    if (require_map_ && distance_field == nullptr) {
      publishStop("map_unavailable", diagnostic_msgs::msg::DiagnosticStatus::ERROR, state_age);
      return;
    }

    MppiRequest request;
    request.initial_state = timed_state->state;
    if (stateWithinLimits(request.initial_state, vehicle_)) {
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
      safeResetBackend();
      publishStop(reason, diagnostic_msgs::msg::DiagnosticStatus::ERROR, state_age, &result);
      return;
    }

    double command_dt = mppi_.dt;
    if (last_command_time_.has_value()) {
      const double elapsed = (tick_time - *last_command_time_).seconds();
      if (std::isfinite(elapsed) && elapsed > 0.0) {
        command_dt = std::min(elapsed, 2.0 * mppi_.dt);
      }
    }
    const State commanded_state = model_->step(request.initial_state, result.control, command_dt);
    publishCommand(commanded_state, result.control, tick_time);
    publishPath(result.predicted_states, tick_time);
    publishDiagnostics("ok", diagnostic_msgs::msg::DiagnosticStatus::OK, state_age, &result);
    last_commanded_steering_ = commanded_state.steering;
    last_applied_control_ = result.control;
    last_command_time_ = tick_time;
    last_output_time_ = tick_time;
  }

  void safeResetBackend() noexcept
  {
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
    command.drive.steering_angle = static_cast<float>(state.steering);
    command.drive.steering_angle_velocity = static_cast<float>(control.steering_rate);
    command.drive.speed = static_cast<float>(state.speed);
    command.drive.acceleration = static_cast<float>(control.acceleration);
    command_publisher_->publish(command);
  }

  void publishStop(
    const std::string & reason, std::uint8_t level,
    double state_age = std::numeric_limits<double>::quiet_NaN(),
    const MppiResult * result = nullptr)
  {
    const rclcpp::Time stamp = now();
    State stopped;
    stopped.steering = last_commanded_steering_;
    publishCommand(stopped, Control{0.0, vehicle_.min_acceleration}, stamp);
    last_applied_control_ = Control{0.0, vehicle_.min_acceleration};
    last_output_time_ = stamp;
    publishDiagnostics(reason, level, state_age, result);
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
    const MppiResult * result)
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
  std::string requested_backend_{"auto"};
  std::string odom_topic_;
  std::string map_topic_;
  std::string command_topic_;
  std::string path_topic_;
  std::string diagnostics_topic_;
  std::string map_frame_;
  std::string base_frame_;
  double control_frequency_{20.0};
  double state_timeout_{0.10};
  double output_timeout_{0.10};
  double tf_timeout_{0.02};
  double max_solve_time_ms_{40.0};
  double localization_position_jump_threshold_{0.50};
  double localization_yaw_jump_threshold_{0.70};
  double localization_jump_hold_time_{0.50};
  bool require_map_{true};
  int occupied_threshold_{50};
  bool unknown_is_occupied_{true};

  std::mutex data_mutex_;
  std::optional<TimedState> latest_state_;
  std::shared_ptr<const DistanceField> distance_field_;
  std::optional<rclcpp::Time> last_localization_jump_time_;
  double last_commanded_steering_{0.0};
  Control last_applied_control_{};
  std::optional<rclcpp::Time> last_command_time_;
  rclcpp::Time last_output_time_{0, 0, RCL_ROS_TIME};

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscription_;
  rclcpp_lifecycle::LifecyclePublisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr
    command_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
    diagnostic_publisher_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
};

}  // namespace race_mppi

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<race_mppi::MppiControllerNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
