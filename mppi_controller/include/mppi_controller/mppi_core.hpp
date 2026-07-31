#ifndef MPPI_CONTROLLER__MPPI_CORE_HPP_
#define MPPI_CONTROLLER__MPPI_CORE_HPP_

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <random>
#include <string>
#include <vector>

namespace mppi_controller
{

constexpr double kPi = 3.14159265358979323846;

double normalizeAngle(double angle);

struct State
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  double speed{0.0};
  // Effective bicycle steering inferred from the vehicle's realised
  // curvature. This is the state used by yaw dynamics and safety validation.
  double steering{0.0};
  // Actuator target sent to the steering servo. It is deliberately separate
  // from effective steering because the real mechanism and tyre response lag.
  double steering_command{0.0};

  State() = default;
  State(double x_in, double y_in, double yaw_in, double speed_in, double steering_in)
  : x(x_in), y(y_in), yaw(yaw_in), speed(speed_in),
    steering(steering_in), steering_command(steering_in)
  {
  }
  State(
    double x_in, double y_in, double yaw_in, double speed_in,
    double steering_in, double steering_command_in)
  : x(x_in), y(y_in), yaw(yaw_in), speed(speed_in),
    steering(steering_in), steering_command(steering_command_in)
  {
  }
};

struct Control
{
  double steering_rate{0.0};
  double acceleration{0.0};
};

struct VehicleConfig
{
  double wheelbase{0.324};
  double length{0.552};
  // base_link is the rear-axle center; body longitudinal extent is
  // [-rear_overhang, length - rear_overhang].
  double rear_overhang{0.124};
  double width{0.320};
  double safety_margin{0.05};
  double min_steering{-0.32};
  double max_steering{0.32};
  double min_steering_rate{-1.5};
  double max_steering_rate{1.5};
  // Identified steering actuator/tyre model. The command can move at the
  // steering-rate bounds above, while effective steering follows with a
  // slower first-order, speed-dependent response.
  double steering_response_time{0.15};
  double min_effective_steering_rate{-1.20};
  double max_effective_steering_rate{1.20};
  double effective_steering_rate_speed_coefficient{0.18};
  double steering_effectiveness_at_zero_speed{1.0};
  double steering_effectiveness_speed_squared{0.05};
  double minimum_steering_effectiveness{0.70};
  double min_acceleration{-1.5};
  double max_acceleration{1.0};
  double min_speed{0.0};
  double max_speed{2.0};
};

[[nodiscard]] double steeringEffectiveness(
  const VehicleConfig & vehicle, double speed) noexcept;
[[nodiscard]] double effectiveSteeringTarget(
  const VehicleConfig & vehicle, double steering_command, double speed) noexcept;
[[nodiscard]] double effectiveSteeringRateLimit(
  const VehicleConfig & vehicle, double speed, bool positive) noexcept;

class BicycleModel
{
public:
  explicit BicycleModel(VehicleConfig config = {});

  [[nodiscard]] State derivative(const State & state, const Control & control) const;
  [[nodiscard]] State step(const State & state, const Control & control, double dt) const;
  [[nodiscard]] Control clampControl(const Control & control) const;
  [[nodiscard]] State clampState(const State & state) const;
  [[nodiscard]] const VehicleConfig & config() const noexcept;

private:
  VehicleConfig config_;
};

[[nodiscard]] bool stateWithinLimits(
  const State & state, const VehicleConfig & vehicle) noexcept;
[[nodiscard]] bool clampMeasuredSpeedWithinTolerance(
  State & state, const VehicleConfig & vehicle, double speed_tolerance) noexcept;

struct OverspeedRecoveryConfig
{
  double min_speed{0.0};
  double max_command_speed{2.0};
  double entry_margin{0.10};
  double exit_margin{0.03};
  double exit_hold_time{0.20};
  double command_reduction{0.10};
  double proportional_gain{0.50};
  double command_deceleration{1.50};
};

struct OverspeedRecoveryResult
{
  bool active{false};
  double command_limit{std::numeric_limits<double>::infinity()};
  double clear_duration{0.0};
  double measured_excess{0.0};
};

/// Hysteretic limiter for a measured speed above the desired command cap.
/// It preserves steering/path planning and lowers only the velocity setpoint;
/// gross measurement errors remain governed by the independent hard tolerance.
class OverspeedRecovery
{
public:
  explicit OverspeedRecovery(OverspeedRecoveryConfig config = {});

  void reset() noexcept;
  [[nodiscard]] OverspeedRecoveryResult update(
    double measured_speed, double previous_command_speed, double dt) noexcept;

private:
  OverspeedRecoveryConfig config_;
  bool active_{false};
  double clear_duration_{0.0};
};

[[nodiscard]] State propagateState(
  const BicycleModel & model, const State & state, const Control & applied_control,
  double duration, double maximum_step = 0.01);
[[nodiscard]] bool isLocalizationJump(
  const State & previous, const State & current, double elapsed,
  const VehicleConfig & vehicle, double base_position_threshold,
  double base_yaw_threshold) noexcept;

struct Waypoint
{
  double s{0.0};
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  double curvature{0.0};
  double reference_speed{0.0};
  double width_left{0.0};
  double width_right{0.0};
};

struct TrackProjection
{
  std::size_t index{0U};
  double lateral_error{0.0};
  double longitudinal_error{0.0};
  double heading_error{0.0};
  double progress{0.0};
  double distance{std::numeric_limits<double>::infinity()};
};

class RaceLine
{
public:
  static RaceLine fromCsv(const std::string & path);
  static RaceLine fromWaypoints(std::vector<Waypoint> waypoints);

  [[nodiscard]] bool valid() const noexcept;
  [[nodiscard]] std::size_t size() const noexcept;
  [[nodiscard]] double length() const noexcept;
  [[nodiscard]] const std::vector<Waypoint> & waypoints() const noexcept;
  [[nodiscard]] const Waypoint & atWrapped(std::ptrdiff_t index) const;
  [[nodiscard]] std::size_t nearestIndex(
    double x, double y, std::optional<std::size_t> hint = std::nullopt,
    std::size_t search_radius = 80U) const;
  [[nodiscard]] TrackProjection project(
    const State & state, std::optional<std::size_t> hint = std::nullopt,
    std::size_t search_radius = 80U) const;
  [[nodiscard]] std::vector<Waypoint> localReference(
    std::size_t start_index, std::size_t count, std::size_t stride = 1U) const;
  [[nodiscard]] double forwardProgress(double from_s, double to_s) const;

private:
  explicit RaceLine(std::vector<Waypoint> waypoints);
  std::vector<Waypoint> waypoints_;
  double length_{0.0};
};

class DistanceField
{
public:
  DistanceField() = default;
  DistanceField(
    std::size_t width, std::size_t height, double resolution,
    double origin_x, double origin_y, std::vector<double> distances,
    double origin_yaw = 0.0);

  static DistanceField fromOccupancyGrid(
    std::size_t width, std::size_t height, double resolution,
    double origin_x, double origin_y, const std::vector<std::int8_t> & occupancy,
    std::int8_t occupied_threshold = 50, bool unknown_is_occupied = true,
    double origin_yaw = 0.0);

  [[nodiscard]] bool valid() const noexcept;
  [[nodiscard]] double clearance(double world_x, double world_y) const;
  [[nodiscard]] std::size_t width() const noexcept;
  [[nodiscard]] std::size_t height() const noexcept;
  [[nodiscard]] double resolution() const noexcept;
  [[nodiscard]] double originX() const noexcept;
  [[nodiscard]] double originY() const noexcept;
  [[nodiscard]] double originYaw() const noexcept;
  [[nodiscard]] const std::vector<double> & distances() const noexcept;

private:
  std::size_t width_{0U};
  std::size_t height_{0U};
  double resolution_{0.0};
  double origin_x_{0.0};
  double origin_y_{0.0};
  double origin_yaw_{0.0};
  std::vector<double> distances_;
};

// A tracked dynamic (or static) obstacle in the map frame, moved by a
// constant-velocity model during rollouts. The position at rollout time t is
// (x + vx * (t + time_offset), y + vy * (t + time_offset)), where time_offset
// is the age of the measurement relative to the rollout start so that stale
// detections are extrapolated forward before the horizon even begins.
struct Obstacle
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  double vx{0.0};
  double vy{0.0};
  double half_length{0.0};
  double half_width{0.0};
  double time_offset{0.0};
};

// Smallest distance between the vehicle body (disk-chain cover, same
// construction as the distance-field footprint check) and any obstacle
// rectangle extrapolated to rollout time `time`. Infinity when there are no
// obstacles; a conservative lower bound otherwise.
[[nodiscard]] double obstacleClearance(
  const State & state, const VehicleConfig & vehicle,
  const std::vector<Obstacle> * obstacles, double time);

struct CostWeights
{
  double lateral{12.0};
  double heading{8.0};
  double lag{1.0};
  double speed{50.0};
  double progress{8.0};
  double control{0.15};
  double control_change{0.4};
  double lateral_acceleration{0.25};
  double boundary{250.0};
  double cbf{400.0};
  double collision{1.0e6};
  double terminal_lateral{80.0};
  double terminal_heading{60.0};
  double terminal_progress{120.0};
};

struct CostBreakdown
{
  double lateral{0.0};
  double heading{0.0};
  double lag{0.0};
  double speed{0.0};
  double progress{0.0};
  double control{0.0};
  double control_change{0.0};
  double lateral_acceleration{0.0};
  double boundary{0.0};
  double cbf{0.0};
  double collision{0.0};
  double terminal_lateral{0.0};
  double terminal_heading{0.0};
  double terminal_progress{0.0};

  [[nodiscard]] double total() const noexcept;
};

struct MppiConfig
{
  std::size_t rollout_count{2048U};
  std::size_t horizon_steps{72U};
  double dt{1.0 / 30.0};
  double lambda{1.0};
  double steering_rate_stddev{0.8};
  double acceleration_stddev{0.8};
  double pure_noise_fraction{0.05};
  std::uint32_t random_seed{7U};
  std::size_t nearest_search_radius{80U};
  double cbf_gamma{0.35};
  double max_lateral_acceleration{6.0};
  double minimum_preview_distance{4.0};
  double maximum_heading_error{1.20};
  double reverse_progress_tolerance{0.05};
  std::size_t repair_steps{4U};
  std::size_t repair_iterations{2U};
  double repair_budget_ms{3.0};
  double repair_clearance{0.05};
  // A measured vehicle can be a few millimetres inside the configured safety
  // envelope because the local grid is discrete. Permit only a short,
  // non-worsening trajectory that restores the full margin. Once moving above
  // the acceleration threshold, recovery must not command positive acceleration.
  double initial_clearance_tolerance{0.0};
  std::size_t clearance_recovery_steps{0U};
  double clearance_recovery_speed_threshold{0.10};
  double clearance_recovery_acceleration_speed_threshold{0.10};
  // CUDA backend allocates this capacity once during warmup. It never grows
  // the device buffer from the control or map callback paths.
  std::size_t cuda_max_map_cells{4U * 1024U * 1024U};
  CostWeights weights{};
};

struct TrajectoryMetrics
{
  double target_speed{0.0};
  double preview_target{0.0};
  double predicted_distance{0.0};
  double forward_progress{0.0};
  double maximum_heading_error{0.0};
  double minimum_clearance{std::numeric_limits<double>::infinity()};
  double minimum_obstacle_clearance{std::numeric_limits<double>::infinity()};
  double stopping_distance{0.0};
  std::size_t reverse_steps{0U};
};

struct MppiBehavior
{
  double speed_scale{1.0};
  double raceline_weight_scale{1.0};
  double safety_weight_scale{1.0};
  // Signed in the race-line normal direction: positive is left.
  double lateral_reference_offset{0.0};
};

struct MppiRequest
{
  State initial_state{};
  const RaceLine * race_line{nullptr};
  const DistanceField * distance_field{nullptr};
  // Obstacles already expressed in the map frame; each carries its own
  // measurement age via Obstacle::time_offset. Null or empty means none.
  const std::vector<Obstacle> * obstacles{nullptr};
  double exploration_scale{1.0};
  MppiBehavior behavior{};
};

struct MppiResult
{
  bool valid{false};
  std::string reason{"not_computed"};
  Control control{};
  std::vector<Control> control_sequence;
  std::vector<State> predicted_states;
  CostBreakdown cost{};
  double solve_time_ms{0.0};
  std::size_t best_rollout{0U};
  std::size_t rollout_count{0U};
  std::optional<std::size_t> valid_rollouts;
  TrajectoryMetrics metrics{};
};

class MppiBackend
{
public:
  virtual ~MppiBackend() = default;
  virtual void configure(const MppiConfig & config, const VehicleConfig & vehicle) = 0;
  virtual void reset() = 0;
  virtual bool warmup(const RaceLine & race_line, const DistanceField * distance_field) = 0;
  // Static maps are uploaded from the map callback, never from the real-time
  // compute path. CPU implementations require no upload.
  virtual bool updateDistanceField(const DistanceField & distance_field)
  {
    return distance_field.valid();
  }
  [[nodiscard]] virtual MppiResult compute(const MppiRequest & request) = 0;
  [[nodiscard]] virtual std::string name() const = 0;
};

// Stable controller-facing name. CUDA implementations should implement this same interface.
using ControllerBackend = MppiBackend;

class CpuMppiBackend final : public MppiBackend
{
public:
  CpuMppiBackend();
  CpuMppiBackend(MppiConfig config, VehicleConfig vehicle);

  void configure(const MppiConfig & config, const VehicleConfig & vehicle) override;
  void reset() override;
  bool warmup(const RaceLine & race_line, const DistanceField * distance_field) override;
  [[nodiscard]] MppiResult compute(const MppiRequest & request) override;
  [[nodiscard]] std::string name() const override;

  [[nodiscard]] CostBreakdown evaluateTrajectory(
    const State & initial_state, const std::vector<Control> & controls,
    const RaceLine & race_line, const DistanceField * distance_field,
    std::vector<State> * states = nullptr,
    const std::vector<Obstacle> * obstacles = nullptr,
    const MppiBehavior * behavior = nullptr) const;

  [[nodiscard]] bool repairControls(
    const State & initial_state, std::vector<Control> & controls,
    const RaceLine & race_line, const DistanceField * distance_field,
    std::chrono::steady_clock::time_point deadline,
    const std::vector<Obstacle> * obstacles = nullptr) const;

  [[nodiscard]] TrajectoryMetrics trajectoryMetrics(
    const State & initial_state, const std::vector<State> & states,
    const RaceLine & race_line, const DistanceField * distance_field,
    const std::vector<Obstacle> * obstacles = nullptr) const;

private:
  // `time` is the rollout-relative time of `state`; obstacles are evaluated at
  // their constant-velocity extrapolation for that instant.
  [[nodiscard]] bool stateSafe(
    const State & state, const RaceLine & race_line,
    const DistanceField * distance_field,
    const std::vector<Obstacle> * obstacles, double time,
    std::size_t * hint = nullptr) const;
  [[nodiscard]] bool initialStateRecoverable(
    const State & state, const RaceLine & race_line,
    const DistanceField * distance_field,
    const std::vector<Obstacle> * obstacles, double * initial_margin,
    std::size_t * hint = nullptr) const;
  [[nodiscard]] bool stateSafeOrRecovering(
    const State & state, const RaceLine & race_line,
    const DistanceField * distance_field,
    const std::vector<Obstacle> * obstacles, double time,
    double initial_margin, std::size_t recovery_step,
    std::size_t * hint = nullptr) const;
  [[nodiscard]] double mapMargin(
    const State & state, const DistanceField * distance_field,
    const std::vector<Obstacle> * obstacles, double time) const;
  [[nodiscard]] double footprintClearance(
    const State & state, const DistanceField * distance_field) const;

  MppiConfig config_{};
  VehicleConfig vehicle_{};
  BicycleModel model_{};
  std::vector<Control> nominal_controls_;
  std::mt19937 random_generator_;
  bool configured_{false};
};

std::unique_ptr<MppiBackend> makeCpuBackend(
  const MppiConfig & config = {}, const VehicleConfig & vehicle = {});

// True only when the MPPI-Generic CUDA translation unit was compiled and
// linked into mppi_controller.  Finding CUDA headers or the vendor package alone is
// deliberately not treated as availability.
[[nodiscard]] bool cudaBackendCompiled() noexcept;

// True when a CUDA device is present whose compute capability can execute the
// architecture this library was compiled for.  Guards hardware-gated tests:
// the vendor kernels abort the process instead of failing recoverably when
// launched on an incompatible device.
[[nodiscard]] bool cudaBackendDeviceCompatible() noexcept;

// backend may be "auto", "cuda", "cpu", or the legacy "cpu_reference".
// "auto" prefers the compiled CUDA backend and otherwise returns the CPU
// reference implementation.  Explicit "cuda" throws when CUDA was not built.
std::unique_ptr<MppiBackend> makeBackend(
  const std::string & backend, const MppiConfig & config = {},
  const VehicleConfig & vehicle = {});

}  // namespace mppi_controller

#endif  // MPPI_CONTROLLER__MPPI_CORE_HPP_
