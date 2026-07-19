#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <numeric>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/header.hpp"

namespace ftg_controller
{

struct FTGConfig
{
  double field_of_view_deg{180.0};
  double max_lidar_range{10.0};
  double min_lidar_range{0.05};
  int smoothing_window{5};
  double disparity_threshold{0.40};
  double safety_radius{0.35};
  double min_clearance{0.45};
  double min_gap_width{0.65};
  int best_point_window{9};
  double distance_weight{1.0};
  double heading_weight{0.35};
  double gap_center_weight{0.25};
};

class FollowTheGapPlanner
{
public:
  explicit FollowTheGapPlanner(FTGConfig config)
  : config_(std::move(config)) {}

  std::pair<std::vector<double>, std::vector<double>> preprocessScan(
    const std::vector<float> & input_ranges,
    double angle_min,
    double angle_increment,
    double sensor_range_min,
    double sensor_range_max) const
  {
    if (input_ranges.empty() || !std::isfinite(angle_increment) || angle_increment <= 0.0) {
      return {};
    }

    const double finite_sensor_max =
      std::isfinite(sensor_range_max) && sensor_range_max > 0.0 ?
      sensor_range_max : config_.max_lidar_range;
    const double range_cap = std::min(config_.max_lidar_range, finite_sensor_max);
    const double finite_sensor_min =
      std::isfinite(sensor_range_min) && sensor_range_min > 0.0 ?
      sensor_range_min : config_.min_lidar_range;
    const double range_floor = std::max(config_.min_lidar_range, finite_sensor_min);
    if (range_floor >= range_cap) {
      return {};
    }

    std::vector<double> clean;
    std::vector<double> angles;
    clean.reserve(input_ranges.size());
    angles.reserve(input_ranges.size());

    const double half_fov = degreesToRadians(config_.field_of_view_deg) * 0.5;
    for (std::size_t index = 0; index < input_ranges.size(); ++index) {
      const double angle = angle_min + static_cast<double>(index) * angle_increment;
      if (!std::isfinite(angle) || std::abs(angle) > half_fov) {
        continue;
      }

      double range = static_cast<double>(input_ranges[index]);
      if (std::isnan(range) || range == -std::numeric_limits<double>::infinity()) {
        range = 0.0;
      } else if (range == std::numeric_limits<double>::infinity()) {
        range = range_cap;
      }
      range = range >= range_floor ? std::min(range, range_cap) : 0.0;
      clean.push_back(range);
      angles.push_back(angle);
    }

    clean = movingAverage(clean, config_.smoothing_window);
    return {std::move(clean), std::move(angles)};
  }

  std::vector<double> createSafeZone(
    const std::vector<double> & ranges, double angle_increment) const
  {
    std::vector<double> safe = ranges;
    if (safe.empty()) {
      return safe;
    }

    const std::vector<double> source = ranges;
    for (std::size_t left_index = 0; left_index + 1 < source.size(); ++left_index) {
      const std::size_t right_index = left_index + 1;
      if (std::abs(source[right_index] - source[left_index]) < config_.disparity_threshold) {
        continue;
      }
      const double left_distance = source[left_index];
      const double right_distance = source[right_index];
      if (left_distance <= 0.0 || right_distance <= 0.0) {
        continue;
      }
      if (left_distance < right_distance) {
        extendObstacle(safe, static_cast<int>(left_index), left_distance, 1, angle_increment);
      } else if (right_distance < left_distance) {
        extendObstacle(safe, static_cast<int>(right_index), right_distance, -1, angle_increment);
      }
    }

    std::optional<std::size_t> closest_index;
    double closest_distance = std::numeric_limits<double>::infinity();
    for (std::size_t index = 0; index < safe.size(); ++index) {
      if (safe[index] > 0.0 && safe[index] < closest_distance) {
        closest_index = index;
        closest_distance = safe[index];
      }
    }
    if (!closest_index.has_value()) {
      return safe;
    }

    const double bubble_angle = std::atan2(config_.safety_radius, closest_distance);
    const int bubble_points = std::max(
      1, static_cast<int>(std::ceil(bubble_angle / angle_increment)));
    const int start = std::max(0, static_cast<int>(*closest_index) - bubble_points);
    const int end = std::min(
      static_cast<int>(safe.size()), static_cast<int>(*closest_index) + bubble_points + 1);
    std::fill(safe.begin() + start, safe.begin() + end, 0.0);
    return safe;
  }

  std::optional<std::pair<double, double>> findTarget(
    const std::vector<double> & ranges,
    const std::vector<double> & angles,
    double angle_increment) const
  {
    if (ranges.empty() || ranges.size() != angles.size()) {
      return std::nullopt;
    }

    struct GapCandidate
    {
      double score;
      std::size_t start;
      std::size_t end;
    };
    std::vector<GapCandidate> gap_candidates;

    std::size_t index = 0;
    while (index < ranges.size()) {
      while (index < ranges.size() && ranges[index] < config_.min_clearance) {
        ++index;
      }
      const std::size_t start = index;
      while (index < ranges.size() && ranges[index] >= config_.min_clearance) {
        ++index;
      }
      const std::size_t end = index;
      if (start == end) {
        continue;
      }

      const std::size_t width_points = end - start;
      const double width_angle = static_cast<double>(width_points) * angle_increment;
      const std::vector<double> gap_ranges(ranges.begin() + start, ranges.begin() + end);
      const double representative_distance = percentile(gap_ranges, 50.0);
      const double physical_width = 2.0 * representative_distance *
        std::sin(std::min(width_angle, pi()) * 0.5);
      const double mean_range = std::accumulate(gap_ranges.begin(), gap_ranges.end(), 0.0) /
        static_cast<double>(gap_ranges.size());
      const double center_angle = (angles[start] + angles[end - 1]) * 0.5;
      const double max_range = *std::max_element(gap_ranges.begin(), gap_ranges.end());
      const double score =
        width_angle * (0.5 + mean_range / config_.max_lidar_range) +
        0.35 * std::cos(center_angle) +
        0.20 * max_range / config_.max_lidar_range;

      if (physical_width >= config_.min_gap_width) {
        gap_candidates.push_back({score, start, end});
      }
    }
    if (gap_candidates.empty()) {
      return std::nullopt;
    }

    const auto best_gap = std::max_element(
      gap_candidates.begin(), gap_candidates.end(),
      [](const GapCandidate & left, const GapCandidate & right) {
        return left.score < right.score;
      });
    const std::size_t start = best_gap->start;
    const std::size_t end = best_gap->end;
    const std::size_t count = end - start;
    const std::vector<double> gap_ranges(ranges.begin() + start, ranges.begin() + end);

    int window = std::min(config_.best_point_window, static_cast<int>(count));
    if (window % 2 == 0) {
      --window;
    }
    const std::vector<double> local_depth =
      window > 1 ? movingMean(gap_ranges, window) : gap_ranges;

    std::size_t target_offset = 0;
    double best_score = -std::numeric_limits<double>::infinity();
    const double max_edge_distance = static_cast<double>((count + 1) / 2);
    for (std::size_t offset = 0; offset < count; ++offset) {
      const double edge_distance = static_cast<double>(std::min(offset + 1, count - offset));
      const double edge_score = edge_distance / std::max(max_edge_distance, 1.0);
      const double normalized_depth = std::clamp(
        local_depth[offset] / config_.max_lidar_range, 0.0, 1.0);
      const double score =
        config_.distance_weight * normalized_depth +
        config_.heading_weight * std::cos(angles[start + offset]) +
        config_.gap_center_weight * edge_score;
      if (score > best_score) {
        best_score = score;
        target_offset = offset;
      }
    }
    return std::make_pair(angles[start + target_offset], ranges[start + target_offset]);
  }

private:
  static double pi()
  {
    static const double value = std::acos(-1.0);
    return value;
  }

  static double degreesToRadians(double degrees)
  {
    return degrees * pi() / 180.0;
  }

  static std::vector<double> movingMean(const std::vector<double> & ranges, int window)
  {
    if (window <= 1 || ranges.size() < 2) {
      return ranges;
    }
    const int half = window / 2;
    std::vector<double> averaged(ranges.size(), 0.0);
    for (std::size_t index = 0; index < ranges.size(); ++index) {
      double sum = 0.0;
      for (int offset = -half; offset <= half; ++offset) {
        const int padded_index = std::clamp(
          static_cast<int>(index) + offset, 0, static_cast<int>(ranges.size()) - 1);
        sum += ranges[static_cast<std::size_t>(padded_index)];
      }
      averaged[index] = sum / static_cast<double>(window);
    }
    return averaged;
  }

  static std::vector<double> movingAverage(const std::vector<double> & ranges, int window)
  {
    if (window <= 1 || ranges.size() < 2) {
      return ranges;
    }
    window = std::min(window, static_cast<int>(ranges.size()));
    if (window % 2 == 0) {
      --window;
    }
    if (window <= 1) {
      return ranges;
    }
    const std::vector<double> averaged = movingMean(ranges, window);
    std::vector<double> result(ranges.size(), 0.0);
    for (std::size_t index = 0; index < ranges.size(); ++index) {
      result[index] = ranges[index] > 0.0 ?
        std::min(ranges[index], averaged[index]) : 0.0;
    }
    return result;
  }

  static double percentile(std::vector<double> values, double percent)
  {
    if (values.empty()) {
      return 0.0;
    }
    std::sort(values.begin(), values.end());
    const double position = (static_cast<double>(values.size()) - 1.0) * percent / 100.0;
    const std::size_t lower = static_cast<std::size_t>(std::floor(position));
    const std::size_t upper = static_cast<std::size_t>(std::ceil(position));
    const double fraction = position - static_cast<double>(lower);
    return values[lower] + (values[upper] - values[lower]) * fraction;
  }

  void extendObstacle(
    std::vector<double> & ranges,
    int close_index,
    double close_distance,
    int direction,
    double angle_increment) const
  {
    const double half_angle = std::atan2(config_.safety_radius, close_distance);
    const int point_count = std::max(
      1, static_cast<int>(std::ceil(half_angle / angle_increment)));
    for (int offset = 1; offset <= point_count; ++offset) {
      const int index = close_index + direction * offset;
      if (index < 0 || index >= static_cast<int>(ranges.size()) ||
        ranges[static_cast<std::size_t>(index)] <= 0.0)
      {
        break;
      }
      ranges[static_cast<std::size_t>(index)] = std::min(
        ranges[static_cast<std::size_t>(index)], close_distance);
    }
  }

  FTGConfig config_;
};

class FTGNode : public rclcpp::Node
{
public:
  FTGNode()
  : Node("ftg_node")
  {
    const std::string scan_topic = declare_parameter<std::string>("scan_topic", "/scan");
    const std::string drive_topic = declare_parameter<std::string>("drive_topic", "/drive");

    FTGConfig planner_config;
    planner_config.field_of_view_deg = positiveParameter("field_of_view_deg", 180.0);
    planner_config.max_lidar_range = positiveParameter("max_lidar_range", 10.0);
    planner_config.min_lidar_range = positiveParameter("min_lidar_range", 0.05);
    planner_config.smoothing_window = positiveIntegerParameter("smoothing_window", 5);
    planner_config.disparity_threshold = positiveParameter("disparity_threshold", 0.40);
    planner_config.safety_radius = positiveParameter("safety_radius", 0.35);
    planner_config.min_clearance = positiveParameter("min_clearance", 0.45);
    planner_config.min_gap_width = positiveParameter("min_gap_width", 0.65);
    planner_config.best_point_window = positiveIntegerParameter("best_point_window", 9);
    planner_config.distance_weight = positiveParameter("distance_weight", 1.0);
    planner_config.heading_weight = positiveParameter("heading_weight", 0.35);
    planner_config.gap_center_weight = positiveParameter("gap_center_weight", 0.25);
    planner_ = std::make_unique<FollowTheGapPlanner>(planner_config);

    max_steering_angle_ = positiveParameter("max_steering_angle", 0.42);
    steering_gain_ = positiveParameter("steering_gain", 1.0);
    steering_smoothing_ = boundedParameter("steering_smoothing", 0.35, 0.0, 1.0);
    max_steering_rate_ = positiveParameter("max_steering_rate", 1.5);
    min_speed_ = positiveParameter("min_speed", 1.0);
    max_speed_ = positiveParameter("max_speed", 4.0);
    if (min_speed_ > max_speed_) {
      RCLCPP_WARN(get_logger(), "min_speed exceeds max_speed; clamping it");
      min_speed_ = max_speed_;
    }
    emergency_distance_ = positiveParameter("emergency_distance", 0.45);
    slow_distance_ = positiveParameter("slow_distance", 2.5);
    if (slow_distance_ <= emergency_distance_) {
      RCLCPP_WARN(get_logger(), "slow_distance must exceed emergency_distance; using safe default");
      slow_distance_ = std::max(2.5, emergency_distance_ + 0.5);
    }
    front_sector_deg_ = positiveParameter("front_sector_deg", 20.0);
    wheelbase_ = positiveParameter("wheelbase", 0.33);
    max_lateral_accel_ = positiveParameter("max_lateral_accel", 4.0);
    max_accel_ = positiveParameter("max_accel", 2.0);
    max_decel_ = positiveParameter("max_decel", 5.0);

    const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    subscriber_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic, qos, std::bind(&FTGNode::scanCallback, this, std::placeholders::_1));
    publisher_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic, 10);

    RCLCPP_INFO(
      get_logger(), "FTG started: %s -> %s, speed=%.1f..%.1f m/s",
      scan_topic.c_str(), drive_topic.c_str(), min_speed_, max_speed_);
  }

  void publishStop()
  {
    publishStop(nullptr);
  }

private:
  double positiveParameter(const std::string & name, double default_value)
  {
    const double value = declare_parameter<double>(name, default_value);
    if (!std::isfinite(value) || value <= 0.0) {
      RCLCPP_WARN(get_logger(), "Invalid %s=%f; using %f", name.c_str(), value, default_value);
      return default_value;
    }
    return value;
  }

  int positiveIntegerParameter(const std::string & name, int default_value)
  {
    const auto value = declare_parameter<std::int64_t>(name, default_value);
    if (value <= 0 || value > std::numeric_limits<int>::max()) {
      RCLCPP_WARN(
        get_logger(), "Invalid %s=%ld; using %d",
        name.c_str(), static_cast<long>(value), default_value);
      return default_value;
    }
    return static_cast<int>(value);
  }

  double boundedParameter(
    const std::string & name, double default_value, double lower, double upper)
  {
    const double value = declare_parameter<double>(name, default_value);
    if (!std::isfinite(value)) {
      RCLCPP_WARN(get_logger(), "Invalid %s=%f; using %f", name.c_str(), value, default_value);
      return default_value;
    }
    return std::clamp(value, lower, upper);
  }

  double elapsedTime()
  {
    const std::int64_t now_ns = get_clock()->now().nanoseconds();
    double elapsed = 0.05;
    if (last_update_ns_.has_value() && now_ns > *last_update_ns_) {
      elapsed = static_cast<double>(now_ns - *last_update_ns_) * 1e-9;
    }
    last_update_ns_ = now_ns;
    return std::clamp(elapsed, 0.01, 0.20);
  }

  double frontClearance(
    const std::vector<double> & ranges, const std::vector<double> & angles) const
  {
    const double half_sector = front_sector_deg_ * std::acos(-1.0) / 180.0 * 0.5;
    std::vector<double> valid_front;
    for (std::size_t index = 0; index < ranges.size(); ++index) {
      if (std::abs(angles[index]) <= half_sector && ranges[index] > 0.0) {
        valid_front.push_back(ranges[index]);
      }
    }
    if (valid_front.empty()) {
      return 0.0;
    }
    std::sort(valid_front.begin(), valid_front.end());
    const double position = (static_cast<double>(valid_front.size()) - 1.0) * 0.10;
    const std::size_t lower = static_cast<std::size_t>(std::floor(position));
    const std::size_t upper = static_cast<std::size_t>(std::ceil(position));
    const double fraction = position - static_cast<double>(lower);
    return valid_front[lower] + (valid_front[upper] - valid_front[lower]) * fraction;
  }

  double steeringCommand(double target_angle, double elapsed)
  {
    const double requested = std::clamp(
      steering_gain_ * target_angle, -max_steering_angle_, max_steering_angle_);
    const double filtered = steering_smoothing_ * requested +
      (1.0 - steering_smoothing_) * current_steering_angle_;
    const double maximum_change = max_steering_rate_ * elapsed;
    const double command = std::clamp(
      filtered,
      current_steering_angle_ - maximum_change,
      current_steering_angle_ + maximum_change);
    current_steering_angle_ = command;
    return command;
  }

  double speedCommand(double steering, double clearance, double elapsed)
  {
    if (clearance <= emergency_distance_) {
      current_speed_ = 0.0;
      return 0.0;
    }
    const double steering_ratio = std::min(std::abs(steering) / max_steering_angle_, 1.0);
    double corner_speed = max_speed_ -
      (max_speed_ - min_speed_) * std::pow(steering_ratio, 1.5);
    const double tangent = std::abs(std::tan(steering));
    if (tangent > 1e-4) {
      corner_speed = std::min(
        corner_speed, std::sqrt(max_lateral_accel_ * wheelbase_ / tangent));
    }
    const double clearance_span = std::max(slow_distance_ - emergency_distance_, 1e-3);
    const double clearance_ratio = std::clamp(
      (clearance - emergency_distance_) / clearance_span, 0.0, 1.0);
    const double clearance_speed = min_speed_ + clearance_ratio * (max_speed_ - min_speed_);
    const double stopping_speed = std::sqrt(
      2.0 * max_decel_ * std::max(clearance - emergency_distance_, 0.0));
    const double requested = std::min(
      {corner_speed, clearance_speed, stopping_speed, max_speed_});
    const double lower = std::max(0.0, current_speed_ - max_decel_ * elapsed);
    const double upper = std::min(max_speed_, current_speed_ + max_accel_ * elapsed);
    current_speed_ = std::clamp(requested, lower, upper);
    return current_speed_;
  }

  void publishStop(const std_msgs::msg::Header * source_header)
  {
    current_speed_ = 0.0;
    ackermann_msgs::msg::AckermannDriveStamped command;
    command.header.stamp = get_clock()->now();
    if (source_header != nullptr) {
      command.header.frame_id = source_header->frame_id;
    }
    command.drive.steering_angle = static_cast<float>(current_steering_angle_);
    command.drive.speed = 0.0F;
    publisher_->publish(command);
  }

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    ++scan_count_;
    const double elapsed = elapsedTime();
    auto [clean_ranges, angles] = planner_->preprocessScan(
      msg->ranges, msg->angle_min, msg->angle_increment, msg->range_min, msg->range_max);
    if (clean_ranges.empty()) {
      publishStop(&msg->header);
      if (scan_count_ % 20 == 1) {
        RCLCPP_WARN(get_logger(), "Invalid or empty LaserScan; stopping");
      }
      return;
    }

    const std::vector<double> safe_ranges = planner_->createSafeZone(
      clean_ranges, msg->angle_increment);
    const auto target = planner_->findTarget(safe_ranges, angles, msg->angle_increment);
    if (!target.has_value()) {
      publishStop(&msg->header);
      if (scan_count_ % 20 == 1) {
        RCLCPP_WARN(get_logger(), "No traversable gap; stopping");
      }
      return;
    }

    const double target_angle = target->first;
    const double target_clearance = target->second;
    const double front_clearance = frontClearance(clean_ranges, angles);
    const double path_clearance = std::min(front_clearance, target_clearance);
    const double steering = steeringCommand(target_angle, elapsed);
    const double speed = speedCommand(steering, path_clearance, elapsed);

    ackermann_msgs::msg::AckermannDriveStamped command;
    command.header.stamp = get_clock()->now();
    command.header.frame_id = msg->header.frame_id;
    command.drive.steering_angle = static_cast<float>(steering);
    command.drive.speed = static_cast<float>(speed);
    publisher_->publish(command);

    if (scan_count_ % 20 == 0) {
      RCLCPP_INFO(
        get_logger(),
        "FTG: target=%+.1f deg, steer=%+.3f rad, speed=%.2f m/s, front=%.2f m",
        target_angle * 180.0 / std::acos(-1.0), steering, speed, front_clearance);
    }
  }

  std::unique_ptr<FollowTheGapPlanner> planner_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_;
  double max_steering_angle_{0.42};
  double steering_gain_{1.0};
  double steering_smoothing_{0.35};
  double max_steering_rate_{1.5};
  double min_speed_{1.0};
  double max_speed_{4.0};
  double emergency_distance_{0.45};
  double slow_distance_{2.5};
  double front_sector_deg_{20.0};
  double wheelbase_{0.33};
  double max_lateral_accel_{4.0};
  double max_accel_{2.0};
  double max_decel_{5.0};
  double current_steering_angle_{0.0};
  double current_speed_{0.0};
  std::optional<std::int64_t> last_update_ns_;
  std::uint64_t scan_count_{0};
};

}  // namespace ftg_controller

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ftg_controller::FTGNode>();
  rclcpp::spin(node);
  if (rclcpp::ok()) {
    node->publishStop();
  }
  rclcpp::shutdown();
  return 0;
}
