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
    std::size_t search_radius = 60U) const;

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
};

struct Obstacle
{
  std::int32_t id{-1};
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
  double cost{0.0};
  bool valid{false};
  std::string reason;
  std::vector<TrajectorySample> points;
};

struct PlanResult
{
  bool valid{false};
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
  double transition_length{2.0};
  double overtake_offset{0.45};
  double minimum_lateral_offset{0.10};
  double boundary_sampling_buffer{0.01};
  std::size_t projection_search_radius{60U};
  double max_projection_distance{1.0};
  double minimum_frenet_jacobian{0.20};

  double vehicle_length{0.552};
  double vehicle_width{0.320};
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

  double weight_lateral_offset{2.0};
  double weight_curvature{1.0};
  double weight_clearance{0.20};
  double weight_switch{3.0};
  double weight_speed{1.0};
  double weight_horizon{0.5};
};

class FrenetPlanner
{
public:
  FrenetPlanner(ReferenceLine reference_line, FrenetPlannerConfig config);

  [[nodiscard]] PlanResult plan(
    const EgoState & ego, const std::vector<Obstacle> & obstacles,
    std::optional<std::size_t> projection_hint = std::nullopt,
    std::optional<double> previous_target_d = std::nullopt) const;

  [[nodiscard]] const ReferenceLine & referenceLine() const noexcept;

private:
  [[nodiscard]] std::vector<double> lateralOffsets(
    const FrenetProjection & projection) const;

  [[nodiscard]] CandidateTrajectory generateCandidate(
    const FrenetProjection & projection, double target_d, double horizon,
    double speed_scale, const std::string & name,
    const std::vector<Obstacle> & obstacles,
    std::optional<double> previous_target_d) const;

  ReferenceLine reference_line_;
  FrenetPlannerConfig config_;
};

}  // namespace local_planner

#endif  // LOCAL_PLANNER__FRENET_PLANNER_HPP_
