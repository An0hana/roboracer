// Copyright 2026 RoboRacer Team

#ifndef LOCAL_PLANNER__FRENET_PLANNER_HPP_
#define LOCAL_PLANNER__FRENET_PLANNER_HPP_

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace local_planner
{

struct ReferencePoint
{
  double s{0.0};
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  double curvature{0.0};
  double speed{0.0};
  double width_left{0.0};
  double width_right{0.0};
};

struct FrenetProjection
{
  std::size_t index{0U};
  double s{0.0};
  double d{0.0};
  double distance{0.0};
  double reference_yaw{0.0};
  double heading_error{0.0};
};

class ReferenceLine
{
public:
  static ReferenceLine fromCsv(const std::string & path);
  static ReferenceLine fromPoints(std::vector<ReferencePoint> points);

  [[nodiscard]] bool valid() const noexcept;
  [[nodiscard]] std::size_t size() const noexcept;
  [[nodiscard]] double length() const noexcept;
  [[nodiscard]] const std::vector<ReferencePoint> & points() const noexcept;
  [[nodiscard]] ReferencePoint sample(double s) const;
  [[nodiscard]] FrenetProjection project(
    double x, double y, std::optional<std::size_t> hint = std::nullopt,
    std::size_t search_radius = 60U,
    std::optional<double> yaw = std::nullopt,
    double maximum_heading_error = 3.14159265358979323846) const;

private:
  explicit ReferenceLine(std::vector<ReferencePoint> points);
  [[nodiscard]] double wrapS(double s) const;

  std::vector<ReferencePoint> points_;
  double start_s_{0.0};
  double length_{0.0};
  double closure_length_{0.0};
};

struct EgoState
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  double speed{0.0};
  std::optional<double> curvature;
};

struct Obstacle
{
  std::int32_t id{-1};
  std::uint8_t classification{0U};
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  double vx{0.0};
  double vy{0.0};
  double length{0.0};
  double width{0.0};
  double confidence{0.0};
  bool visible{false};
};

struct TrajectorySample
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  double s{0.0};
  double d{0.0};
  double speed{0.0};
  double acceleration{0.0};
  double curvature{0.0};
  double left_width{0.0};
  double right_width{0.0};
};

struct CandidateTrajectory
{
  std::string name;
  double target_d{0.0};
  double horizon{0.0};
  double speed_scale{1.0};
  double stop_distance{0.0};
  double cost{0.0};
  bool valid{false};
  bool stopping{false};
  std::string reason;
  std::vector<TrajectorySample> points;
};

struct PlanResult
{
  bool valid{false};
  std::string reason{"no_valid_candidate"};
  std::size_t selected_index{0U};
  std::size_t projection_index{0U};
  double ego_s{0.0};
  double ego_d{0.0};
  std::vector<CandidateTrajectory> candidates;
};

struct FrenetPlannerConfig
{
  std::vector<double> planning_horizons{3.0, 4.0, 5.0};
  std::vector<double> speed_scales{0.60, 0.80, 1.00};
  std::size_t lateral_samples_per_side{3U};
  double sample_spacing{0.10};
  double overtake_offset{0.45};
  double minimum_lateral_offset{0.10};
  double boundary_sampling_buffer{0.01};
  std::size_t projection_search_radius{60U};
  double max_projection_distance{1.0};
  double max_projection_heading_error{1.0471975511965976};
  double minimum_frenet_jacobian{0.20};

  double vehicle_length{0.552};
  double vehicle_width{0.320};
  double rear_overhang{0.124};
  double safety_margin{0.05};
  double collision_margin{0.08};
  double default_opponent_length{0.552};
  double default_opponent_width{0.320};
  double minimum_obstacle_confidence{0.05};

  double max_speed{4.0};
  double max_lateral_acceleration{4.0};
  double min_acceleration{-1.5};
  double max_acceleration{1.0};
  double max_curvature{1.0};
  double trailing_stop_margin{0.40};

  double weight_lateral_offset{2.0};
  double weight_curvature{1.0};
  double weight_clearance{0.20};
  double weight_switch{3.0};
  double weight_speed{1.0};
  double weight_horizon{0.5};
  double weight_stop{2.0};
};

class FrenetPlanner
{
public:
  FrenetPlanner(ReferenceLine reference_line, FrenetPlannerConfig config);

  [[nodiscard]] PlanResult plan(
    const EgoState & ego, const std::vector<Obstacle> & obstacles,
    std::optional<std::size_t> projection_hint = std::nullopt,
    std::optional<double> previous_target_d = std::nullopt,
    std::optional<int> preferred_side = std::nullopt,
    bool return_to_raceline = false,
    double behavior_speed_scale = 1.0) const;

  [[nodiscard]] const ReferenceLine & referenceLine() const noexcept;

private:
  [[nodiscard]] std::vector<double> lateralOffsets(
    const FrenetProjection & projection,
    std::optional<int> preferred_side,
    bool return_to_raceline) const;

  [[nodiscard]] CandidateTrajectory generateCandidate(
    const EgoState & ego, const FrenetProjection & projection,
    double target_d, double horizon,
    double speed_scale, const std::string & name,
    const std::vector<Obstacle> & obstacles,
    std::optional<double> previous_target_d) const;

  [[nodiscard]] bool hasRequiredPassingClearance(
    const FrenetProjection & projection,
    const CandidateTrajectory & candidate,
    const std::vector<Obstacle> & obstacles) const;

  ReferenceLine reference_line_;
  FrenetPlannerConfig config_;
};

}  // namespace local_planner

#endif  // LOCAL_PLANNER__FRENET_PLANNER_HPP_
