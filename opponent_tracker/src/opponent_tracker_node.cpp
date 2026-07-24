#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <functional>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "opponent_tracker/multi_object_tracker.hpp"
#include "opponent_tracker/scan_clusterer.hpp"
#include "opponent_tracker/static_map_filter.hpp"
#include "rclcpp/rclcpp.hpp"
#include "roboracer_msgs/msg/tracked_obstacle_array.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

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
    const auto measurement_topic = declare_parameter<std::string>(
      "measurement_topic", "/perception/obstacles_measurement");
    const auto output_topic = declare_parameter<std::string>(
      "output_topic", "/perception/obstacles");
    const auto marker_topic = declare_parameter<std::string>(
      "marker_topic", "/perception/obstacle_markers");
    const auto map_topic = declare_parameter<std::string>("map_topic", "/map");
    target_frame_ = declare_parameter<std::string>("target_frame", "map");

    const double publish_rate = declare_parameter<double>("publish_rate_hz", 50.0);
    const double prediction_publish_rate =
      declare_parameter<double>("prediction_publish_rate_hz", 100.0);
    max_prediction_age_ =
      declare_parameter<double>("max_prediction_age", 0.15);
    transform_timeout_ = declare_parameter<double>("transform_timeout", 0.005);
    if (!std::isfinite(publish_rate) || publish_rate <= 0.0 ||
      !std::isfinite(prediction_publish_rate) || prediction_publish_rate <= 0.0 ||
      !std::isfinite(max_prediction_age_) || max_prediction_age_ <= 0.0 ||
      !std::isfinite(transform_timeout_) || transform_timeout_ < 0.0)
    {
      throw std::invalid_argument("invalid rate or transform timeout");
    }
    publish_period_ = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / publish_rate));
    prediction_publish_period_ =
      std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / prediction_publish_rate));

    const int min_cluster_points = declare_parameter<int>("min_cluster_points", 3);
    const int confidence_full_points =
      declare_parameter<int>("confidence_full_points", 20);
    if (min_cluster_points < 2 || confidence_full_points < min_cluster_points) {
      throw std::invalid_argument("invalid cluster point limits");
    }

    ScanClustererConfig clusterer_config;
    clusterer_config.min_range = declare_parameter<double>("min_range", 0.05);
    clusterer_config.max_range = declare_parameter<double>("max_range", 15.0);
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

    const int min_confirmed_hits =
      declare_parameter<int>("min_confirmed_hits", 3);
    if (min_confirmed_hits <= 0) {
      throw std::invalid_argument("min_confirmed_hits must be positive");
    }
    MultiObjectTrackerConfig tracker_config;
    tracker_config.association_distance =
      declare_parameter<double>("association_distance", 0.75);
    tracker_config.min_confirmed_hits =
      static_cast<std::size_t>(min_confirmed_hits);
    tracker_config.max_missed_time =
      declare_parameter<double>("max_missed_time", 0.30);
    tracker_config.process_noise =
      declare_parameter<double>("tracking_process_noise", 4.0);
    tracker_config.measurement_noise =
      declare_parameter<double>("tracking_measurement_noise", 0.04);
    tracker_config.initial_velocity_stddev =
      declare_parameter<double>("initial_velocity_stddev", 2.0);
    tracker_config.shape_smoothing =
      declare_parameter<double>("shape_smoothing", 0.25);
    tracker_config.dynamic_speed_threshold =
      declare_parameter<double>("dynamic_speed_threshold", 0.15);
    tracker_config.heading_enter_speed =
      declare_parameter<double>("heading_enter_speed", 0.18);
    tracker_config.heading_exit_speed =
      declare_parameter<double>("heading_exit_speed", 0.08);
    tracker_config.heading_smoothing =
      declare_parameter<double>("heading_smoothing", 0.35);
    tracker_config.max_yaw_rate =
      declare_parameter<double>("max_yaw_rate", 2.5);
    tracker_config.max_prediction_dt =
      declare_parameter<double>("max_prediction_dt", 0.10);
    tracker_ = std::make_unique<MultiObjectTracker>(tracker_config);

    marker_height_ = declare_parameter<double>("marker_height", 0.20);
    marker_velocity_scale_ =
      declare_parameter<double>("marker_velocity_scale", 0.75);
    use_known_opponent_size_ =
      declare_parameter<bool>("use_known_opponent_size", true);
    opponent_length_ =
      declare_parameter<double>("opponent_length", 0.552);
    opponent_width_ =
      declare_parameter<double>("opponent_width", 0.320);
    center_fit_support_quantile_ =
      declare_parameter<double>("center_fit_support_quantile", 0.10);
    center_fit_lateral_ratio_threshold_ =
      declare_parameter<double>("center_fit_lateral_ratio_threshold", 0.15);
    center_fit_association_distance_ = tracker_config.association_distance;
    const int marker_history_length =
      declare_parameter<int>("marker_history_length", 50);
    if (!std::isfinite(marker_height_) || marker_height_ <= 0.0 ||
      !std::isfinite(marker_velocity_scale_) || marker_velocity_scale_ <= 0.0 ||
      !std::isfinite(opponent_length_) || opponent_length_ <= 0.0 ||
      !std::isfinite(opponent_width_) || opponent_width_ <= 0.0 ||
      !std::isfinite(center_fit_support_quantile_) ||
      center_fit_support_quantile_ < 0.0 ||
      center_fit_support_quantile_ >= 0.5 ||
      !std::isfinite(center_fit_lateral_ratio_threshold_) ||
      center_fit_lateral_ratio_threshold_ < 0.0 ||
      marker_history_length < 2)
    {
      throw std::invalid_argument("invalid marker configuration");
    }
    marker_history_length_ = static_cast<std::size_t>(marker_history_length);

    measurement_publisher_ =
      create_publisher<roboracer_msgs::msg::TrackedObstacleArray>(
      measurement_topic, rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
    publisher_ = create_publisher<roboracer_msgs::msg::TrackedObstacleArray>(
      output_topic, rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
    marker_publisher_ =
      create_publisher<visualization_msgs::msg::MarkerArray>(
      marker_topic, rclcpp::QoS(rclcpp::KeepLast(1)).reliable());

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
    prediction_timer_ = create_wall_timer(
      prediction_publish_period_,
      std::bind(&OpponentTrackerNode::publishPredictedObstacles, this));

    RCLCPP_INFO(
      get_logger(),
      "Opponent tracker started: %s -> %s -> %s (%s, %.1f/%.1f Hz, map filter: %s)",
      scan_topic.c_str(), measurement_topic.c_str(), output_topic.c_str(),
      target_frame_.c_str(), publish_rate, prediction_publish_rate,
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
    std::uint64_t scan_generation = 0U;
    {
      std::lock_guard<std::mutex> lock(scan_mutex_);
      if (!latest_scan_ || processed_scan_generation_ == latest_scan_generation_) {
        return;
      }
      message = latest_scan_;
      scan_generation = latest_scan_generation_;
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

    std::vector<Detection> transformed_detections;
    transformed_detections.reserve(detections.size());
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

      Detection transformed_detection = detection;
      transformed_detection.x = detection_x;
      transformed_detection.y = detection_y;
      transformed_detection.yaw =
        normalizeAngle(detection.yaw + transform_yaw);
      for (auto & point : transformed_detection.points) {
        const double point_x =
          transform_x + cos_yaw * point.x - sin_yaw * point.y;
        const double point_y =
          transform_y + sin_yaw * point.x + cos_yaw * point.y;
        point.x = point_x;
        point.y = point_y;
      }
      transformed_detections.push_back(transformed_detection);
    }

    // With a known opponent footprint, convert each visible-surface measurement
    // into a body-center measurement before the Kalman update. Real opponents
    // with unknown dimensions can disable this correction and retain the
    // measured, smoothed footprint.
    const auto prior_tracks = tracker_->estimates();
    if (use_known_opponent_size_) {
      for (auto & detection : transformed_detections) {
        const TrackEstimate * matched_track = nullptr;
        double matched_distance = center_fit_association_distance_;
        for (const auto & track : prior_tracks) {
          if (!track.visible || !track.dynamic) {
            continue;
          }
          const double distance =
            std::hypot(detection.x - track.x, detection.y - track.y);
          if (distance <= matched_distance) {
            matched_distance = distance;
            matched_track = &track;
          }
        }
        if (matched_track == nullptr) {
          continue;
        }

        double fitted_x = detection.x;
        double fitted_y = detection.y;
        if (fitKnownRectangleCenter(
            detection, matched_track->yaw, transform_x, transform_y,
            fitted_x, fitted_y))
        {
          detection.x = fitted_x;
          detection.y = fitted_y;
          const bool first_center_measurement =
            center_tracking_ids_.insert(matched_track->id).second;
          if (first_center_measurement) {
            tracker_->shiftTrackPosition(
              matched_track->id,
              fitted_x - matched_track->x,
              fitted_y - matched_track->y);
          }
        }
      }
    }

    const double timestamp =
      static_cast<double>(message->header.stamp.sec) +
      1.0e-9 * static_cast<double>(message->header.stamp.nanosec);
    const auto tracks = tracker_->update(transformed_detections, timestamp);

    roboracer_msgs::msg::TrackedObstacleArray output;
    output.header = message->header;
    output.header.frame_id = output_frame;
    output.obstacles.reserve(tracks.size());
    for (const auto & track : tracks) {
      roboracer_msgs::msg::TrackedObstacle obstacle;
      obstacle.id = track.id;
      obstacle.classification = roboracer_msgs::msg::TrackedObstacle::OPPONENT;
      obstacle.vx = track.vx;
      obstacle.vy = track.vy;
      obstacle.yaw = track.yaw;

      obstacle.x = track.x;
      obstacle.y = track.y;
      if (use_known_opponent_size_) {
        const Detection * matched_detection = nullptr;
        double matched_distance = center_fit_association_distance_;
        if (track.visible && track.dynamic) {
          for (const auto & detection : transformed_detections) {
            const double distance =
              std::hypot(detection.x - track.x, detection.y - track.y);
            if (distance <= matched_distance) {
              matched_distance = distance;
              matched_detection = &detection;
            }
          }
        }

        if ((matched_detection == nullptr ||
          !fitKnownRectangleCenter(
            *matched_detection, obstacle.yaw, transform_x, transform_y,
            obstacle.x, obstacle.y)) &&
          center_tracking_ids_.count(track.id) == 0U)
        {
          const double ray_x = track.x - transform_x;
          const double ray_y = track.y - transform_y;
          const double ray_length = std::hypot(ray_x, ray_y);
          if (ray_length > 1.0e-6) {
            const double ray_yaw = std::atan2(ray_y, ray_x);
            const double physical_half_extent = 0.5 * (
              std::abs(std::cos(obstacle.yaw - ray_yaw)) * opponent_length_ +
              std::abs(std::sin(obstacle.yaw - ray_yaw)) * opponent_width_);
            const double measured_half_extent = 0.5 * track.width;
            const double center_correction =
              std::max(0.0, physical_half_extent - measured_half_extent);
            obstacle.x += center_correction * ray_x / ray_length;
            obstacle.y += center_correction * ray_y / ray_length;
          }
        }
      }
      obstacle.s = 0.0;
      obstacle.d = 0.0;
      obstacle.length =
        use_known_opponent_size_ ? opponent_length_ : track.length;
      obstacle.width =
        use_known_opponent_size_ ? opponent_width_ : track.width;
      obstacle.confidence = track.confidence;
      obstacle.dynamic = track.dynamic;
      obstacle.visible = track.visible;
      output.obstacles.push_back(obstacle);
    }

    std::unordered_set<std::int32_t> active_track_ids;
    for (const auto & track : tracks) {
      active_track_ids.insert(track.id);
    }
    for (auto iterator = center_tracking_ids_.begin();
      iterator != center_tracking_ids_.end(); )
    {
      if (active_track_ids.count(*iterator) == 0U) {
        iterator = center_tracking_ids_.erase(iterator);
      } else {
        ++iterator;
      }
    }

    measurement_publisher_->publish(output);
    {
      std::lock_guard<std::mutex> lock(prediction_mutex_);
      latest_measurement_ = output;
      has_measurement_ = true;
    }

    // Mark a scan as processed only after its timestamped TF was available.
    // At 50 Hz odometry and 40 Hz LiDAR, some scans naturally arrive before
    // the following TF sample and must be retried instead of discarded.
    {
      std::lock_guard<std::mutex> lock(scan_mutex_);
      processed_scan_generation_ =
        std::max(processed_scan_generation_, scan_generation);
    }
  }

  void publishPredictedObstacles()
  {
    roboracer_msgs::msg::TrackedObstacleArray prediction;
    {
      std::lock_guard<std::mutex> lock(prediction_mutex_);
      if (!has_measurement_) {
        return;
      }
      prediction = latest_measurement_;
    }

    const auto current_time = get_clock()->now();
    const auto measurement_time = rclcpp::Time(prediction.header.stamp);
    const double age = (current_time - measurement_time).seconds();
    prediction.header.stamp = current_time;
    if (!std::isfinite(age) || age < 0.0 || age > max_prediction_age_) {
      prediction.obstacles.clear();
    } else {
      for (auto & obstacle : prediction.obstacles) {
        obstacle.x += age * obstacle.vx;
        obstacle.y += age * obstacle.vy;
      }
    }

    publisher_->publish(prediction);
    publishMarkers(prediction);
  }

  void publishMarkers(
    const roboracer_msgs::msg::TrackedObstacleArray & obstacles)
  {
    visualization_msgs::msg::MarkerArray output;
    const auto marker_header = obstacles.header;

    visualization_msgs::msg::Marker clear;
    clear.header = marker_header;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    output.markers.push_back(clear);

    std::unordered_set<std::int32_t> active_ids;
    for (const auto & obstacle : obstacles.obstacles) {
      const double display_x = obstacle.x;
      const double display_y = obstacle.y;
      active_ids.insert(obstacle.id);
      auto & history = marker_history_[obstacle.id];
      if (obstacle.visible) {
        geometry_msgs::msg::Point point;
        point.x = display_x;
        point.y = display_y;
        point.z = 0.03;
        if (history.empty() ||
          std::hypot(point.x - history.back().x, point.y - history.back().y) > 0.01)
        {
          history.push_back(point);
        }
        while (history.size() > marker_history_length_) {
          history.pop_front();
        }
      }

      visualization_msgs::msg::Marker box;
      box.header = marker_header;
      box.ns = "opponent_boxes";
      box.id = obstacle.id;
      box.type = visualization_msgs::msg::Marker::LINE_STRIP;
      box.action = visualization_msgs::msg::Marker::ADD;
      box.pose.orientation.w = 1.0;
      const double speed = std::hypot(obstacle.vx, obstacle.vy);
      const double box_yaw = obstacle.yaw;
      const double cos_box_yaw = std::cos(box_yaw);
      const double sin_box_yaw = std::sin(box_yaw);
      const double half_length =
        0.5 * obstacle.length;
      const double half_width =
        0.5 * obstacle.width;
      const double corner_x[] = {
        half_length, half_length, -half_length, -half_length, half_length};
      const double corner_y[] = {
        half_width, -half_width, -half_width, half_width, half_width};
      for (std::size_t corner = 0U; corner < 5U; ++corner) {
        geometry_msgs::msg::Point point;
        point.x =
          display_x + cos_box_yaw * corner_x[corner] -
          sin_box_yaw * corner_y[corner];
        point.y =
          display_y + sin_box_yaw * corner_x[corner] +
          cos_box_yaw * corner_y[corner];
        point.z = marker_height_ + 0.03;
        box.points.push_back(point);
      }
      box.scale.x = 0.035;
      box.color.r = 1.0F;
      box.color.g = obstacle.visible ? 0.90F : 0.65F;
      box.color.b = obstacle.visible ? 0.05F : 0.65F;
      box.color.a = obstacle.visible ? 1.0F : 0.45F;
      output.markers.push_back(box);

      if (speed > 0.02) {
        visualization_msgs::msg::Marker velocity;
        velocity.header = marker_header;
        velocity.ns = "opponent_velocity";
        velocity.id = obstacle.id;
        velocity.type = visualization_msgs::msg::Marker::ARROW;
        velocity.action = visualization_msgs::msg::Marker::ADD;
        geometry_msgs::msg::Point start;
        start.x = display_x;
        start.y = display_y;
        start.z = marker_height_ + 0.05;
        geometry_msgs::msg::Point end = start;
        end.x += marker_velocity_scale_ * obstacle.vx;
        end.y += marker_velocity_scale_ * obstacle.vy;
        velocity.points = {start, end};
        velocity.scale.x = 0.035;
        velocity.scale.y = 0.070;
        velocity.scale.z = 0.090;
        velocity.color.r = 0.05F;
        velocity.color.g = 0.85F;
        velocity.color.b = 1.0F;
        velocity.color.a = obstacle.visible ? 0.95F : 0.35F;
        output.markers.push_back(velocity);
      }

      visualization_msgs::msg::Marker label;
      label.header = marker_header;
      label.ns = "opponent_labels";
      label.id = obstacle.id;
      label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      label.action = visualization_msgs::msg::Marker::ADD;
      label.pose.position.x = display_x;
      label.pose.position.y = display_y;
      label.pose.position.z = marker_height_ + 0.25;
      label.pose.orientation.w = 1.0;
      label.scale.z = 0.16;
      label.color.r = 1.0F;
      label.color.g = 1.0F;
      label.color.b = 1.0F;
      label.color.a = obstacle.visible ? 1.0F : 0.45F;
      std::ostringstream text;
      text << "ID " << obstacle.id << "  " << std::fixed << std::setprecision(2)
           << speed << " m/s";
      label.text = text.str();
      output.markers.push_back(label);

      if (history.size() >= 2U) {
        visualization_msgs::msg::Marker trail;
        trail.header = marker_header;
        trail.ns = "opponent_history";
        trail.id = obstacle.id;
        trail.type = visualization_msgs::msg::Marker::LINE_STRIP;
        trail.action = visualization_msgs::msg::Marker::ADD;
        trail.pose.orientation.w = 1.0;
        trail.scale.x = 0.035;
        trail.color.r = 0.15F;
        trail.color.g = 1.0F;
        trail.color.b = 0.25F;
        trail.color.a = 0.80F;
        trail.points.assign(history.begin(), history.end());
        output.markers.push_back(trail);
      }
    }

    for (auto iterator = marker_history_.begin(); iterator != marker_history_.end(); ) {
      if (active_ids.count(iterator->first) == 0U) {
        iterator = marker_history_.erase(iterator);
      } else {
        ++iterator;
      }
    }
    marker_publisher_->publish(output);
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

  static double quantile(std::vector<double> values, double fraction)
  {
    std::sort(values.begin(), values.end());
    const double index =
      fraction * static_cast<double>(values.size() - 1U);
    const auto lower = static_cast<std::size_t>(std::floor(index));
    const auto upper = static_cast<std::size_t>(std::ceil(index));
    const double weight = index - static_cast<double>(lower);
    return values[lower] + weight * (values[upper] - values[lower]);
  }

  bool fitKnownRectangleCenter(
    const Detection & detection, double yaw,
    double sensor_x, double sensor_y,
    double & center_x, double & center_y) const
  {
    if (detection.points.size() < 3U || !std::isfinite(yaw)) {
      return false;
    }

    const double forward_x = std::cos(yaw);
    const double forward_y = std::sin(yaw);
    const double lateral_x = -forward_y;
    const double lateral_y = forward_x;
    std::vector<double> longitudinal;
    std::vector<double> lateral;
    longitudinal.reserve(detection.points.size());
    lateral.reserve(detection.points.size());
    for (const auto & point : detection.points) {
      longitudinal.push_back(point.x * forward_x + point.y * forward_y);
      lateral.push_back(point.x * lateral_x + point.y * lateral_y);
    }

    const double low_longitudinal =
      quantile(longitudinal, center_fit_support_quantile_);
    const double high_longitudinal =
      quantile(longitudinal, 1.0 - center_fit_support_quantile_);
    const double low_lateral =
      quantile(lateral, center_fit_support_quantile_);
    const double high_lateral =
      quantile(lateral, 1.0 - center_fit_support_quantile_);
    const double approximate_longitudinal =
      0.5 * (low_longitudinal + high_longitudinal);
    const double approximate_lateral =
      0.5 * (low_lateral + high_lateral);
    const double sensor_longitudinal =
      sensor_x * forward_x + sensor_y * forward_y;
    const double sensor_lateral =
      sensor_x * lateral_x + sensor_y * lateral_y;

    const double fitted_longitudinal =
      sensor_longitudinal < approximate_longitudinal ?
      low_longitudinal + 0.5 * opponent_length_ :
      high_longitudinal - 0.5 * opponent_length_;

    const double normalized_longitudinal_offset =
      std::abs(sensor_longitudinal - approximate_longitudinal) /
      opponent_length_;
    const double normalized_lateral_offset =
      std::abs(sensor_lateral - approximate_lateral) /
      opponent_width_;
    double fitted_lateral = 0.5 * (low_lateral + high_lateral);
    if (normalized_lateral_offset >=
      center_fit_lateral_ratio_threshold_ * normalized_longitudinal_offset)
    {
      fitted_lateral =
        sensor_lateral < approximate_lateral ?
        low_lateral + 0.5 * opponent_width_ :
        high_lateral - 0.5 * opponent_width_;
    }

    center_x =
      fitted_longitudinal * forward_x + fitted_lateral * lateral_x;
    center_y =
      fitted_longitudinal * forward_y + fitted_lateral * lateral_y;
    return std::isfinite(center_x) && std::isfinite(center_y);
  }

  std::string latest_odom_frame_;
  std::string target_frame_;
  double transform_timeout_{0.005};
  double max_prediction_age_{0.15};
  bool use_map_filter_{true};
  std::chrono::nanoseconds publish_period_{20000000};
  std::chrono::nanoseconds prediction_publish_period_{10000000};
  std::mutex scan_mutex_;
  sensor_msgs::msg::LaserScan::ConstSharedPtr latest_scan_;
  std::uint64_t latest_scan_generation_{0U};
  std::uint64_t processed_scan_generation_{0U};
  std::unique_ptr<ScanClusterer> clusterer_;
  std::unique_ptr<StaticMapFilter> map_filter_;
  std::unique_ptr<MultiObjectTracker> tracker_;
  bool use_known_opponent_size_{true};
  double opponent_length_{0.552};
  double opponent_width_{0.320};
  double center_fit_support_quantile_{0.10};
  double center_fit_lateral_ratio_threshold_{0.15};
  double center_fit_association_distance_{0.75};
  std::unordered_set<std::int32_t> center_tracking_ids_;
  double marker_height_{0.20};
  double marker_velocity_scale_{0.75};
  std::size_t marker_history_length_{50U};
  std::unordered_map<std::int32_t, std::deque<geometry_msgs::msg::Point>>
  marker_history_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::mutex prediction_mutex_;
  roboracer_msgs::msg::TrackedObstacleArray latest_measurement_;
  bool has_measurement_{false};
  rclcpp::Publisher<roboracer_msgs::msg::TrackedObstacleArray>::SharedPtr
    measurement_publisher_;
  rclcpp::Publisher<roboracer_msgs::msg::TrackedObstacleArray>::SharedPtr publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscription_;
  rclcpp::TimerBase::SharedPtr processing_timer_;
  rclcpp::TimerBase::SharedPtr prediction_timer_;
};

}  // namespace opponent_tracker

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<opponent_tracker::OpponentTrackerNode>());
  rclcpp::shutdown();
  return 0;
}
