// Copyright 2026 RoboRacer Team
//
// MPPI-Generic v0.9.0 adapter.  This file is compiled only when the pinned
// vendor package has actually built and exported MPPI::MPPI.

#include "mppi_controller/mppi_core.hpp"

#include <cuda_runtime.h>

#include <mppi/controllers/MPPI/mppi_controller.cuh>
#include <mppi/cost_functions/cost.cuh>
#include <mppi/dynamics/dynamics.cuh>
#include <mppi/feedback_controllers/feedback.cuh>
#include <mppi/sampling_distributions/gaussian/gaussian.cuh>

#include <Eigen/Core>

#include <algorithm>
#include <array>
#include <cfloat>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace mppi_controller
{
namespace
{

constexpr int kCudaRollouts = 2048;
constexpr int kCudaTimesteps = 48;
constexpr int kLocalWaypointCapacity = 256;
constexpr int kReferencePointsBehind = 24;
constexpr float kMinimumBarrierScale = 0.02F;

__host__ __device__ inline float clampFloat(float value, float lower, float upper)
{
  return fminf(fmaxf(value, lower), upper);
}

__host__ __device__ inline float squareFloat(float value)
{
  return value * value;
}

__host__ __device__ inline float normalizeAngleFloat(float angle)
{
  return atan2f(sinf(angle), cosf(angle));
}

double maximumReachableDistance(
  double initial_speed, const VehicleConfig & vehicle, double duration)
{
  const double speed = std::clamp(initial_speed, 0.0, vehicle.max_speed);
  if (duration <= 0.0 || vehicle.max_acceleration <= 0.0 ||
    speed >= vehicle.max_speed)
  {
    return speed * std::max(0.0, duration);
  }
  const double time_to_limit =
    (vehicle.max_speed - speed) / vehicle.max_acceleration;
  const double accelerating_time = std::min(duration, time_to_limit);
  return speed * accelerating_time +
         0.5 * vehicle.max_acceleration * accelerating_time * accelerating_time +
         vehicle.max_speed * std::max(0.0, duration - accelerating_time);
}

struct F1TenthDynamicsParams : public DynamicsParams
{
  enum class StateIndex : int
  {
    POS_X = 0,
    POS_Y,
    YAW,
    SPEED,
    STEERING,
    NUM_STATES
  };

  enum class ControlIndex : int
  {
    STEERING_RATE = 0,
    ACCELERATION,
    NUM_CONTROLS
  };

  enum class OutputIndex : int
  {
    POS_X = 0,
    POS_Y,
    YAW,
    SPEED,
    STEERING,
    NUM_OUTPUTS
  };

  float wheelbase{0.324F};
  float min_steering{-0.20F};
  float max_steering{0.20F};
  float min_speed{0.0F};
  float max_speed{2.0F};
};

constexpr int kSteeringRateIndex =
  static_cast<int>(F1TenthDynamicsParams::ControlIndex::STEERING_RATE);
constexpr int kAccelerationIndex =
  static_cast<int>(F1TenthDynamicsParams::ControlIndex::ACCELERATION);

class F1TenthDynamics final
  : public MPPI_internal::Dynamics<F1TenthDynamics, F1TenthDynamicsParams>
{
public:
  using PARENT_CLASS = MPPI_internal::Dynamics<F1TenthDynamics, F1TenthDynamicsParams>;
  using PARENT_CLASS::updateState;

  explicit F1TenthDynamics(
    const F1TenthDynamicsParams & params,
    const VehicleConfig & vehicle, cudaStream_t stream = nullptr)
  : PARENT_CLASS(stream)
  {
    setParams(params);
    std::array<float2, CONTROL_DIM> ranges{};
    ranges[kSteeringRateIndex] = make_float2(
      static_cast<float>(vehicle.min_steering_rate),
      static_cast<float>(vehicle.max_steering_rate));
    ranges[kAccelerationIndex] = make_float2(
      static_cast<float>(vehicle.min_acceleration),
      static_cast<float>(vehicle.max_acceleration));
    setControlRanges(ranges);
  }

  std::string getDynamicsModelName() const override
  {
    return "F1TENTH five-state RK4 bicycle";
  }

  void computeKinematics(
    const Eigen::Ref<const state_array> &, Eigen::Ref<state_array>)
  {
  }

  void computeDynamics(
    const Eigen::Ref<const state_array> & state,
    const Eigen::Ref<const control_array> & control,
    Eigen::Ref<state_array> derivative)
  {
    derivative(S_INDEX(POS_X)) = state(S_INDEX(SPEED)) * cosf(state(S_INDEX(YAW)));
    derivative(S_INDEX(POS_Y)) = state(S_INDEX(SPEED)) * sinf(state(S_INDEX(YAW)));
    derivative(S_INDEX(YAW)) = state(S_INDEX(SPEED)) * tanf(state(S_INDEX(STEERING))) /
      this->params_.wheelbase;
    derivative(S_INDEX(SPEED)) = control(kAccelerationIndex);
    derivative(S_INDEX(STEERING)) = control(kSteeringRateIndex);
  }

  __device__ void computeKinematics(float *, float *)
  {
  }

  __device__ void computeDynamics(
    float * state, float * control, float * derivative, float * = nullptr)
  {
    if (threadIdx.y != 0) {
      return;
    }
    derivative[S_INDEX(POS_X)] = state[S_INDEX(SPEED)] * cosf(state[S_INDEX(YAW)]);
    derivative[S_INDEX(POS_Y)] = state[S_INDEX(SPEED)] * sinf(state[S_INDEX(YAW)]);
    derivative[S_INDEX(YAW)] = state[S_INDEX(SPEED)] * tanf(state[S_INDEX(STEERING)]) /
      this->params_.wheelbase;
    derivative[S_INDEX(SPEED)] = control[kAccelerationIndex];
    derivative[S_INDEX(STEERING)] = control[kSteeringRateIndex];
  }

  void step(
    Eigen::Ref<state_array> state, Eigen::Ref<state_array> next_state,
    Eigen::Ref<state_array> state_derivative,
    const Eigen::Ref<const control_array> & control,
    Eigen::Ref<output_array> output, const float, const float dt)
  {
    const state_array k1 = derivativeHost(state, control);
    state_array intermediate = state + 0.5F * dt * k1;
    clampStateHost(intermediate);
    const state_array k2 = derivativeHost(intermediate, control);
    intermediate = state + 0.5F * dt * k2;
    clampStateHost(intermediate);
    const state_array k3 = derivativeHost(intermediate, control);
    intermediate = state + dt * k3;
    clampStateHost(intermediate);
    const state_array k4 = derivativeHost(intermediate, control);
    state_derivative = (k1 + 2.0F * k2 + 2.0F * k3 + k4) / 6.0F;
    next_state = state + dt * state_derivative;
    clampStateHost(next_state);
    output = next_state;
  }

  __device__ void step(
    float * state, float * next_state, float * state_derivative,
    float * control, float * output, float *, const float, const float dt)
  {
    if (threadIdx.y != 0) {
      return;
    }
    float k1[STATE_DIM];
    float k2[STATE_DIM];
    float k3[STATE_DIM];
    float k4[STATE_DIM];
    float intermediate[STATE_DIM];

    derivativeDevice(state, control, k1);
    for (int i = 0; i < STATE_DIM; ++i) {
      intermediate[i] = state[i] + 0.5F * dt * k1[i];
    }
    clampStateDevice(intermediate);
    derivativeDevice(intermediate, control, k2);
    for (int i = 0; i < STATE_DIM; ++i) {
      intermediate[i] = state[i] + 0.5F * dt * k2[i];
    }
    clampStateDevice(intermediate);
    derivativeDevice(intermediate, control, k3);
    for (int i = 0; i < STATE_DIM; ++i) {
      intermediate[i] = state[i] + dt * k3[i];
    }
    clampStateDevice(intermediate);
    derivativeDevice(intermediate, control, k4);

    for (int i = 0; i < STATE_DIM; ++i) {
      state_derivative[i] = (k1[i] + 2.0F * k2[i] + 2.0F * k3[i] + k4[i]) / 6.0F;
      next_state[i] = state[i] + dt * state_derivative[i];
    }
    clampStateDevice(next_state);
    for (int i = 0; i < OUTPUT_DIM; ++i) {
      output[i] = next_state[i];
    }
  }

  state_array stateFromMap(const std::map<std::string, float> & values) override
  {
    state_array state;
    state << values.at("POS_X"), values.at("POS_Y"), values.at("YAW"),
      values.at("SPEED"), values.at("STEERING");
    return state;
  }

  void getStoppingControl(
    const Eigen::Ref<const state_array> &, Eigen::Ref<control_array> control) override
  {
    control.setZero();
    control(kAccelerationIndex) = this->control_rngs_[kAccelerationIndex].x;
  }

private:
  state_array derivativeHost(
    const Eigen::Ref<const state_array> & state,
    const Eigen::Ref<const control_array> & control) const
  {
    state_array derivative;
    derivative(S_INDEX(POS_X)) = state(S_INDEX(SPEED)) * cosf(state(S_INDEX(YAW)));
    derivative(S_INDEX(POS_Y)) = state(S_INDEX(SPEED)) * sinf(state(S_INDEX(YAW)));
    derivative(S_INDEX(YAW)) = state(S_INDEX(SPEED)) * tanf(state(S_INDEX(STEERING))) /
      this->params_.wheelbase;
    derivative(S_INDEX(SPEED)) = control(kAccelerationIndex);
    derivative(S_INDEX(STEERING)) = control(kSteeringRateIndex);
    return derivative;
  }

  void clampStateHost(Eigen::Ref<state_array> state) const
  {
    state(S_INDEX(YAW)) = normalizeAngleFloat(state(S_INDEX(YAW)));
    state(S_INDEX(SPEED)) = clampFloat(
      state(S_INDEX(SPEED)), this->params_.min_speed, this->params_.max_speed);
    state(S_INDEX(STEERING)) = clampFloat(
      state(S_INDEX(STEERING)), this->params_.min_steering,
      this->params_.max_steering);
  }

  __device__ void derivativeDevice(
    const float * state, const float * control, float * derivative) const
  {
    derivative[S_INDEX(POS_X)] = state[S_INDEX(SPEED)] * cosf(state[S_INDEX(YAW)]);
    derivative[S_INDEX(POS_Y)] = state[S_INDEX(SPEED)] * sinf(state[S_INDEX(YAW)]);
    derivative[S_INDEX(YAW)] = state[S_INDEX(SPEED)] * tanf(state[S_INDEX(STEERING)]) /
      this->params_.wheelbase;
    derivative[S_INDEX(SPEED)] = control[kAccelerationIndex];
    derivative[S_INDEX(STEERING)] = control[kSteeringRateIndex];
  }

  __device__ void clampStateDevice(float * state) const
  {
    state[S_INDEX(YAW)] = normalizeAngleFloat(state[S_INDEX(YAW)]);
    state[S_INDEX(SPEED)] = clampFloat(
      state[S_INDEX(SPEED)], this->params_.min_speed, this->params_.max_speed);
    state[S_INDEX(STEERING)] = clampFloat(
      state[S_INDEX(STEERING)], this->params_.min_steering,
      this->params_.max_steering);
  }
};

constexpr int kMaxCostObstacles = 8;

struct F1TenthCostParams : public CostParams<2>
{
  int waypoint_count{0};
  float waypoint_x[kLocalWaypointCapacity]{};
  float waypoint_y[kLocalWaypointCapacity]{};
  float waypoint_yaw[kLocalWaypointCapacity]{};
  float waypoint_speed[kLocalWaypointCapacity]{};
  float waypoint_progress[kLocalWaypointCapacity]{};
  float waypoint_width_left[kLocalWaypointCapacity]{};
  float waypoint_width_right[kLocalWaypointCapacity]{};
  float previous_controls[2 * kCudaTimesteps]{};
  float last_control[2]{};

  const float * distance_field{nullptr};
  int map_width{0};
  int map_height{0};
  int map_valid{0};
  int footprint_segment_count{1};
  float map_resolution{0.0F};
  float map_origin_x{0.0F};
  float map_origin_y{0.0F};
  float map_origin_cosine{1.0F};
  float map_origin_sine{0.0F};
  float footprint_segment_length{0.552F};
  float footprint_cover_radius{0.319F};

  // Constant-velocity obstacle set, extrapolated inside the kernel by the
  // rollout timestep so every sample sees the opponent where it will be, not
  // where it was measured. Disk-chain covers are precomputed on the host with
  // the same formula the CPU reference uses.
  int obstacle_count{0};
  float obstacle_time_offset{0.0F};
  float obstacle_x[kMaxCostObstacles]{};
  float obstacle_y[kMaxCostObstacles]{};
  float obstacle_vx[kMaxCostObstacles]{};
  float obstacle_vy[kMaxCostObstacles]{};
  float obstacle_cosine[kMaxCostObstacles]{};
  float obstacle_sine[kMaxCostObstacles]{};
  float obstacle_half_length[kMaxCostObstacles]{};
  float obstacle_segment_length[kMaxCostObstacles]{};
  float obstacle_cover_radius[kMaxCostObstacles]{};
  int obstacle_segments[kMaxCostObstacles]{};

  float dt{0.05F};
  float wheelbase{0.324F};
  float rear_extent{-0.124F};
  float front_extent{0.428F};
  float half_width{0.160F};
  float safety_margin{0.05F};
  float max_lateral_acceleration{6.0F};
  float maximum_heading_error{1.20F};
  float minimum_preview_distance{4.0F};
  float barrier_distance{0.05F};
  float barrier_scale{0.05F};
  float speed_scale{1.0F};
  float raceline_weight_scale{1.0F};
  float safety_weight_scale{1.0F};
  float lateral_reference_offset{0.0F};

  float weight_lateral{12.0F};
  float weight_heading{8.0F};
  float weight_lag{1.0F};
  float weight_speed{50.0F};
  float weight_progress{8.0F};
  float weight_control{0.15F};
  float weight_control_change{0.4F};
  float weight_lateral_acceleration{0.25F};
  float weight_boundary{250.0F};
  float weight_barrier{400.0F};
  float weight_collision{1.0e6F};
  float weight_terminal_lateral{80.0F};
  float weight_terminal_heading{60.0F};
  float weight_terminal_progress{120.0F};
};

class F1TenthRaceCost final
  : public Cost<F1TenthRaceCost, F1TenthCostParams, F1TenthDynamicsParams>
{
public:
  explicit F1TenthRaceCost(cudaStream_t stream = nullptr)
  {
    bindToStream(stream);
  }

  std::string getCostFunctionName() const override
  {
    return "F1TENTH race-line soft-barrier cost";
  }

  float computeStateCost(
    const Eigen::Ref<const output_array> state, int timestep = 0,
    int * crash_status = nullptr)
  {
    return stateCost(state.data(), timestep, crash_status);
  }

  __device__ float computeStateCost(
    float * state, int timestep = 0, float * = nullptr, int * crash_status = nullptr)
  {
    return stateCost(state, timestep, crash_status);
  }

  float computeControlCost(
    const Eigen::Ref<const control_array> control, int timestep,
    int * = nullptr)
  {
    return controlCost(control.data(), timestep);
  }

  __device__ float computeControlCost(
    float * control, int timestep, float *, int *)
  {
    return controlCost(control, timestep);
  }

  float terminalCost(const Eigen::Ref<const output_array> state)
  {
    return terminalStateCost(state.data());
  }

  __device__ float terminalCost(float * state, float *)
  {
    return terminalStateCost(state);
  }

private:
  __host__ __device__ float pointMapClearance(float world_x, float world_y) const
  {
#if defined(__CUDA_ARCH__)
    if (this->params_.map_valid == 0 || this->params_.distance_field == nullptr ||
      this->params_.map_width < 1 || this->params_.map_height < 1 ||
      this->params_.map_resolution <= 0.0F)
    {
      return 1.0e6F;
    }
    const float dx = world_x - this->params_.map_origin_x;
    const float dy = world_y - this->params_.map_origin_y;
    float grid_x = (this->params_.map_origin_cosine * dx +
      this->params_.map_origin_sine * dy) / this->params_.map_resolution - 0.5F;
    float grid_y = (-this->params_.map_origin_sine * dx +
      this->params_.map_origin_cosine * dy) / this->params_.map_resolution - 0.5F;
    if (grid_x < 0.0F || grid_y < 0.0F ||
      grid_x > static_cast<float>(this->params_.map_width - 1) ||
      grid_y > static_cast<float>(this->params_.map_height - 1))
    {
      return 0.0F;
    }
    grid_x = clampFloat(grid_x, 0.0F, static_cast<float>(this->params_.map_width - 1));
    grid_y = clampFloat(grid_y, 0.0F, static_cast<float>(this->params_.map_height - 1));
    const int x0 = static_cast<int>(floorf(grid_x));
    const int y0 = static_cast<int>(floorf(grid_y));
    const int x1 = min(x0 + 1, this->params_.map_width - 1);
    const int y1 = min(y0 + 1, this->params_.map_height - 1);
    const float tx = grid_x - static_cast<float>(x0);
    const float ty = grid_y - static_cast<float>(y0);
    const float low = this->params_.distance_field[y0 * this->params_.map_width + x0] *
      (1.0F - tx) +
      this->params_.distance_field[y0 * this->params_.map_width + x1] * tx;
    const float high = this->params_.distance_field[y1 * this->params_.map_width + x0] *
      (1.0F - tx) +
      this->params_.distance_field[y1 * this->params_.map_width + x1] * tx;
    return fmaxf(0.0F, low * (1.0F - ty) + high * ty);
#else
    // The host cost is diagnostic-only; the authoritative host-side map cost
    // is evaluated by CpuMppiBackend after repair.
    (void)world_x;
    (void)world_y;
    return 1.0e6F;
#endif
  }

  // Mirrors CpuMppiBackend/obstacleClearance: minimum disk-chain distance to
  // every obstacle extrapolated to rollout time `time`, minus safety margin.
  __host__ __device__ float footprintObstacleMargin(
    const float * state, float time) const
  {
    if (this->params_.obstacle_count <= 0) {
      return 1.0e6F;
    }
    const float yaw = state[S_IND_CLASS(F1TenthDynamicsParams, YAW)];
    const float cosine = cosf(yaw);
    const float sine = sinf(yaw);
    const float horizon_time = time + this->params_.obstacle_time_offset;
    float minimum = 1.0e6F;
    const int segment_count = this->params_.footprint_segment_count > 0 ?
      this->params_.footprint_segment_count : 1;
    for (int index = 0; index < this->params_.obstacle_count &&
      index < kMaxCostObstacles; ++index)
    {
      const float center_x = this->params_.obstacle_x[index] +
        this->params_.obstacle_vx[index] * horizon_time;
      const float center_y = this->params_.obstacle_y[index] +
        this->params_.obstacle_vy[index] * horizon_time;
      for (int segment = 0; segment < segment_count; ++segment) {
        const float longitudinal = this->params_.rear_extent +
          (static_cast<float>(segment) + 0.5F) *
          this->params_.footprint_segment_length;
        const float vehicle_x = state[S_IND_CLASS(F1TenthDynamicsParams, POS_X)] +
          cosine * longitudinal;
        const float vehicle_y = state[S_IND_CLASS(F1TenthDynamicsParams, POS_Y)] +
          sine * longitudinal;
        for (int piece = 0; piece < this->params_.obstacle_segments[index]; ++piece) {
          const float offset = -this->params_.obstacle_half_length[index] +
            (static_cast<float>(piece) + 0.5F) *
            this->params_.obstacle_segment_length[index];
          const float obstacle_x = center_x +
            this->params_.obstacle_cosine[index] * offset;
          const float obstacle_y = center_y +
            this->params_.obstacle_sine[index] * offset;
          const float dx = vehicle_x - obstacle_x;
          const float dy = vehicle_y - obstacle_y;
          minimum = fminf(
            minimum, sqrtf(dx * dx + dy * dy) -
            this->params_.footprint_cover_radius -
            this->params_.obstacle_cover_radius[index]);
        }
      }
    }
    return minimum - this->params_.safety_margin;
  }

  __host__ __device__ float footprintMapMargin(const float * state) const
  {
    if (this->params_.map_valid == 0) {
      return 1.0e6F;
    }
    const float cosine = cosf(state[S_IND_CLASS(F1TenthDynamicsParams, YAW)]);
    const float sine = sinf(state[S_IND_CLASS(F1TenthDynamicsParams, YAW)]);
    float minimum = 1.0e6F;
    const int segment_count = this->params_.footprint_segment_count > 0 ?
      this->params_.footprint_segment_count : 1;
    for (int segment = 0; segment < segment_count; ++segment) {
      const float longitudinal = this->params_.rear_extent +
        (static_cast<float>(segment) + 0.5F) * this->params_.footprint_segment_length;
      const float x = state[S_IND_CLASS(F1TenthDynamicsParams, POS_X)] +
        cosine * longitudinal;
      const float y = state[S_IND_CLASS(F1TenthDynamicsParams, POS_Y)] +
        sine * longitudinal;
      minimum = fminf(
        minimum, pointMapClearance(x, y) - this->params_.footprint_cover_radius);
    }
    return minimum - this->params_.safety_margin;
  }

  __host__ __device__ int nearestWaypoint(const float * state) const
  {
    int best = 0;
    float best_distance = FLT_MAX;
    for (int index = 0; index < this->params_.waypoint_count; ++index) {
      const float dx = state[S_IND_CLASS(F1TenthDynamicsParams, POS_X)] -
        this->params_.waypoint_x[index];
      const float dy = state[S_IND_CLASS(F1TenthDynamicsParams, POS_Y)] -
        this->params_.waypoint_y[index];
      const float distance = dx * dx + dy * dy;
      if (distance < best_distance) {
        best_distance = distance;
        best = index;
      }
    }
    return best;
  }

  __host__ __device__ float stateCost(
    const float * state, int timestep, int * crash_status) const
  {
    if (this->params_.waypoint_count <= 0) {
      if (crash_status != nullptr) {
        crash_status[0] = 1;
      }
      return this->params_.weight_collision;
    }
    // The rollout kernel evaluates cost on the post-step state with the
    // pre-step index, so the physical rollout time is (timestep + 1) * dt —
    // identical to the CPU reference's step convention.
    const float rollout_time =
      static_cast<float>(timestep + 1) * this->params_.dt;

    const int index = nearestWaypoint(state);
    const float reference_yaw = this->params_.waypoint_yaw[index];
    const float dx = state[S_IND_CLASS(F1TenthDynamicsParams, POS_X)] -
      this->params_.waypoint_x[index];
    const float dy = state[S_IND_CLASS(F1TenthDynamicsParams, POS_Y)] -
      this->params_.waypoint_y[index];
    const float cosine = cosf(reference_yaw);
    const float sine = sinf(reference_yaw);
    const float lag_error = cosine * dx + sine * dy;
    const float physical_lateral_error = -sine * dx + cosine * dy;
    const float lateral_error =
      physical_lateral_error - this->params_.lateral_reference_offset;
    const float heading_error = normalizeAngleFloat(
      state[S_IND_CLASS(F1TenthDynamicsParams, YAW)] - reference_yaw);
    const float speed = state[S_IND_CLASS(F1TenthDynamicsParams, SPEED)];
    const float steering = state[S_IND_CLASS(F1TenthDynamicsParams, STEERING)];

    const float heading_sine = sinf(heading_error);
    const float heading_cosine = cosf(heading_error);
    float minimum_lateral = FLT_MAX;
    float maximum_lateral = -FLT_MAX;
    const float longitudinal_extents[2]{
      this->params_.rear_extent, this->params_.front_extent};
    const float lateral_extents[2]{
      -this->params_.half_width, this->params_.half_width};
    for (int longitudinal_index = 0; longitudinal_index < 2; ++longitudinal_index) {
      for (int lateral_index = 0; lateral_index < 2; ++lateral_index) {
        const float corner_lateral = physical_lateral_error +
          longitudinal_extents[longitudinal_index] * heading_sine +
          lateral_extents[lateral_index] * heading_cosine;
        minimum_lateral = fminf(minimum_lateral, corner_lateral);
        maximum_lateral = fmaxf(maximum_lateral, corner_lateral);
      }
    }
    const float left_margin = this->params_.waypoint_width_left[index] -
      maximum_lateral - this->params_.safety_margin;
    const float right_margin = this->params_.waypoint_width_right[index] +
      minimum_lateral - this->params_.safety_margin;
    const float track_margin = fminf(left_margin, right_margin);
    const float boundary_violation = fmaxf(0.0F, -track_margin);

    const float map_margin = fminf(
      footprintMapMargin(state),
      footprintObstacleMargin(state, rollout_time));
    const float safe_map_margin = map_margin - this->params_.barrier_distance;
    const float map_violation = fmaxf(0.0F, -safe_map_margin);

    float map_soft_barrier = 0.0F;
    if (this->params_.map_valid != 0 || this->params_.obstacle_count > 0) {
      const float map_barrier_argument = clampFloat(
        (this->params_.barrier_distance - map_margin) /
        fmaxf(this->params_.barrier_scale, kMinimumBarrierScale), -20.0F, 20.0F);
      map_soft_barrier = this->params_.barrier_scale *
        log1pf(expf(map_barrier_argument));
    }

    const float lateral_acceleration =
      speed * speed * tanf(steering) / this->params_.wheelbase;
    const float lateral_acceleration_excess = fmaxf(
      0.0F, fabsf(lateral_acceleration) - this->params_.max_lateral_acceleration);
    const float forward_velocity = speed * cosf(heading_error);
    const bool forward_constraint_violated =
      fabsf(heading_error) > this->params_.maximum_heading_error ||
      forward_velocity < -1.0e-3F;

    float cost = 0.0F;
    cost += this->params_.weight_lateral *
      this->params_.raceline_weight_scale * squareFloat(lateral_error);
    cost += this->params_.weight_heading *
      this->params_.raceline_weight_scale * squareFloat(heading_error);
    cost += this->params_.weight_lag *
      this->params_.raceline_weight_scale * squareFloat(lag_error);
    cost += this->params_.weight_speed * squareFloat(
      speed - this->params_.waypoint_speed[index] * this->params_.speed_scale);
    cost -= this->params_.weight_progress * forward_velocity * this->params_.dt;
    cost += this->params_.weight_lateral_acceleration *
      (squareFloat(lateral_acceleration) +
      20.0F * squareFloat(lateral_acceleration_excess));
    // Static race-line widths lose authority with the rest of the prior
    // geometry; only the live scan barrier receives safety_weight_scale.
    cost += this->params_.weight_boundary * this->params_.raceline_weight_scale *
      (squareFloat(boundary_violation) + squareFloat(map_violation));
    // Race-line widths remain a soft global preference. Only the live
    // scan-derived local distance field can declare a collision.
    cost += this->params_.weight_barrier *
      this->params_.safety_weight_scale * squareFloat(map_soft_barrier);
    if (safe_map_margin < 0.0F) {
      cost += this->params_.weight_collision;
      if (crash_status != nullptr) {
        crash_status[0] = 1;
      }
    }
    if (forward_constraint_violated) {
      cost += this->params_.weight_collision;
      if (crash_status != nullptr) {
        crash_status[0] = 1;
      }
    }
    return cost;
  }

  __host__ __device__ float terminalStateCost(const float * state) const
  {
    const int index = nearestWaypoint(state);
    const float reference_yaw = this->params_.waypoint_yaw[index];
    const float dx = state[S_IND_CLASS(F1TenthDynamicsParams, POS_X)] -
      this->params_.waypoint_x[index];
    const float dy = state[S_IND_CLASS(F1TenthDynamicsParams, POS_Y)] -
      this->params_.waypoint_y[index];
    const float cosine = cosf(reference_yaw);
    const float sine = sinf(reference_yaw);
    const float longitudinal_error = cosine * dx + sine * dy;
    const float lateral_error = -sine * dx + cosine * dy -
      this->params_.lateral_reference_offset;
    const float heading_error = normalizeAngleFloat(
      state[S_IND_CLASS(F1TenthDynamicsParams, YAW)] - reference_yaw);
    const float forward_progress =
      this->params_.waypoint_progress[index] + longitudinal_error;
    const float preview_shortfall = fmaxf(
      0.0F, this->params_.minimum_preview_distance - forward_progress);
    return 0.5F * stateCost(state, kCudaTimesteps - 1, nullptr) +
           this->params_.weight_terminal_lateral *
           this->params_.raceline_weight_scale * squareFloat(lateral_error) +
           this->params_.weight_terminal_heading *
           this->params_.raceline_weight_scale * squareFloat(heading_error) +
           this->params_.weight_terminal_progress * squareFloat(preview_shortfall);
  }

  __host__ __device__ float controlCost(const float * control, int timestep) const
  {
    const int bounded_timestep = timestep < 0 ? 0 :
      (timestep >= kCudaTimesteps ? kCudaTimesteps - 1 : timestep);
    const float previous_steering_rate = bounded_timestep == 0 ?
      this->params_.last_control[0] :
      this->params_.previous_controls[2 * (bounded_timestep - 1)];
    const float previous_acceleration = bounded_timestep == 0 ?
      this->params_.last_control[1] :
      this->params_.previous_controls[2 * (bounded_timestep - 1) + 1];
    return this->params_.weight_control *
           (squareFloat(control[0]) + squareFloat(control[1])) +
           this->params_.weight_control_change *
           (squareFloat(control[0] - previous_steering_rate) +
           squareFloat(control[1] - previous_acceleration));
  }
};

struct NullFeedbackState : GPUState
{
};

struct NullFeedbackParams
{
};

template<class DynamicsT>
class DeviceNullFeedback final
  : public GPUFeedbackController<DeviceNullFeedback<DynamicsT>, DynamicsT, NullFeedbackState>
{
public:
  using PARENT_CLASS = GPUFeedbackController<
    DeviceNullFeedback<DynamicsT>, DynamicsT, NullFeedbackState>;

  explicit DeviceNullFeedback(cudaStream_t stream = nullptr)
  : PARENT_CLASS(stream)
  {
  }

  __device__ void k(
    const float *, const float *, const int, float *, float * control_output)
  {
    for (int index = threadIdx.y; index < DynamicsT::CONTROL_DIM; index += blockDim.y) {
      control_output[index] = 0.0F;
    }
  }
};

template<class DynamicsT, int Timesteps>
class NullFeedback final
  : public FeedbackController<
    DeviceNullFeedback<DynamicsT>, NullFeedbackParams, Timesteps>
{
public:
  using PARENT_CLASS = FeedbackController<
    DeviceNullFeedback<DynamicsT>, NullFeedbackParams, Timesteps>;
  using typename PARENT_CLASS::control_array;
  using typename PARENT_CLASS::control_trajectory;
  using typename PARENT_CLASS::state_array;
  using typename PARENT_CLASS::state_trajectory;
  using INTERNAL_STATE = typename PARENT_CLASS::TEMPLATED_FEEDBACK_STATE;

  explicit NullFeedback(float dt, cudaStream_t stream = nullptr)
  : PARENT_CLASS(dt, Timesteps, stream)
  {
  }

  void initTrackingController() override
  {
  }

  control_array k_(
    const Eigen::Ref<const state_array> &, const Eigen::Ref<const state_array> &,
    int, INTERNAL_STATE &) override
  {
    return control_array::Zero();
  }

  void computeFeedback(
    const Eigen::Ref<const state_array> &,
    const Eigen::Ref<const state_trajectory> &,
    const Eigen::Ref<const control_trajectory> &) override
  {
  }
};

using SamplingDistribution =
  mppi::sampling_distributions::GaussianDistribution<F1TenthDynamicsParams>;
using FeedbackControllerT = NullFeedback<F1TenthDynamics, kCudaTimesteps>;
using ControllerT = VanillaMPPIController<
  F1TenthDynamics, F1TenthRaceCost, FeedbackControllerT,
  kCudaTimesteps, kCudaRollouts, SamplingDistribution>;

class MppiGenericCudaBackend final : public MppiBackend
{
public:
  MppiGenericCudaBackend(const MppiConfig & config, const VehicleConfig & vehicle)
  {
    configure(config, vehicle);
  }

  ~MppiGenericCudaBackend() override
  {
    controller_.reset();
    feedback_.reset();
    sampler_.reset();
    cost_.reset();
    dynamics_.reset();
    releaseDistanceFieldBuffer();
  }

  void configure(const MppiConfig & config, const VehicleConfig & vehicle) override
  {
    if (config.rollout_count != static_cast<std::size_t>(kCudaRollouts) ||
      config.horizon_steps != static_cast<std::size_t>(kCudaTimesteps))
    {
      throw std::invalid_argument(
              "CUDA backend is compiled for exactly 2048 rollouts and 48 horizon steps");
    }
    // Reuse the CPU implementation's validation and exact final safety model.
    validator_.configure(config, vehicle);
    config_ = config;
    vehicle_ = vehicle;
    last_control_ = Control{};
    launch_seed_pending_ = true;
    initialized_ = false;
    controller_.reset();
    feedback_.reset();
    sampler_.reset();
    cost_.reset();
    dynamics_.reset();
    releaseDistanceFieldBuffer();
    cost_params_ = F1TenthCostParams{};
  }

  void reset() override
  {
    std::lock_guard<std::mutex> lock(backend_mutex_);
    last_control_ = Control{};
    launch_seed_pending_ = true;
    validator_.reset();
    if (controller_ != nullptr) {
      ControllerT::control_trajectory zero = ControllerT::control_trajectory::Zero();
      controller_->updateImportanceSampler(zero);
    }
  }

  bool warmup(const RaceLine & race_line, const DistanceField * distance_field) override
  {
    if (!race_line.valid()) {
      return false;
    }
    try {
      const Waypoint & waypoint = race_line.waypoints().front();
      const State warm_state{
        waypoint.x, waypoint.y, waypoint.yaw,
        std::min(0.1, waypoint.reference_speed), 0.0};
      prepareCost(warm_state, race_line);
      if (!initializeController()) {
        return false;
      }
      MppiRequest request;
      request.initial_state = warm_state;
      request.race_line = &race_line;
      request.distance_field = distance_field;
      const MppiResult result = compute(request);
      reset();
      return result.valid;
    } catch (const std::exception &) {
      initialized_ = false;
      return false;
    }
  }

  bool updateDistanceField(const DistanceField & distance_field) override
  {
    std::lock_guard<std::mutex> lock(backend_mutex_);
    if (!initialized_ || distance_field_device_ == nullptr || !distance_field.valid()) {
      return false;
    }
    const std::size_t cell_count = distance_field.width() * distance_field.height();
    if (distance_field.width() > static_cast<std::size_t>(std::numeric_limits<int>::max()) ||
      distance_field.height() > static_cast<std::size_t>(std::numeric_limits<int>::max()) ||
      cell_count == 0U || cell_count > config_.cuda_max_map_cells)
    {
      return false;
    }

    // This allocation is intentionally in the low-frequency map callback,
    // never in compute(). The device capacity was allocated during warmup.
    std::vector<float> upload(cell_count);
    const auto & source = distance_field.distances();
    std::transform(
      source.begin(), source.end(), upload.begin(),
      [](double value) {return static_cast<float>(value);});
    cudaError_t status = cudaMemcpy(
      distance_field_device_, upload.data(), cell_count * sizeof(float),
      cudaMemcpyHostToDevice);
    if (status != cudaSuccess) {
      return false;
    }

    cost_params_.distance_field = distance_field_device_;
    cost_params_.map_width = static_cast<int>(distance_field.width());
    cost_params_.map_height = static_cast<int>(distance_field.height());
    cost_params_.map_resolution = static_cast<float>(distance_field.resolution());
    cost_params_.map_origin_x = static_cast<float>(distance_field.originX());
    cost_params_.map_origin_y = static_cast<float>(distance_field.originY());
    cost_params_.map_origin_cosine = static_cast<float>(std::cos(distance_field.originYaw()));
    cost_params_.map_origin_sine = static_cast<float>(std::sin(distance_field.originYaw()));
    cost_params_.map_valid = 1;
    cost_->setParams(cost_params_);
    return cudaDeviceSynchronize() == cudaSuccess;
  }

  MppiResult compute(const MppiRequest & request) override
  {
    std::lock_guard<std::mutex> lock(backend_mutex_);
    const auto start = std::chrono::steady_clock::now();
    MppiResult result;
    result.rollout_count = config_.rollout_count;
    const auto finish = [&](const std::string & reason) {
        result.reason = reason;
        result.solve_time_ms = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - start).count();
        return result;
      };

    if (!initialized_ || controller_ == nullptr || request.race_line == nullptr ||
      !request.race_line->valid())
    {
      return finish("cuda_not_initialized");
    }
    if (request.distance_field != nullptr && request.distance_field->valid() &&
      cost_params_.map_valid == 0)
    {
      return finish("cuda_map_not_uploaded");
    }
    const std::array<double, 5> state_values{{
      request.initial_state.x, request.initial_state.y, request.initial_state.yaw,
      request.initial_state.speed, request.initial_state.steering}};
    if (!std::all_of(
        state_values.begin(), state_values.end(),
        [](double value) {return std::isfinite(value);}))
    {
      return finish("nonfinite_state");
    }
    if (!std::isfinite(request.exploration_scale) || request.exploration_scale < 1.0) {
      return finish("invalid_exploration_scale");
    }
    const std::array<double, 4> behavior_values{{
      request.behavior.speed_scale,
      request.behavior.raceline_weight_scale,
      request.behavior.safety_weight_scale,
      request.behavior.lateral_reference_offset}};
    if (!std::all_of(
        behavior_values.begin(), behavior_values.end(),
        [](double value) {return std::isfinite(value);}) ||
      request.behavior.speed_scale < 0.0 ||
      request.behavior.speed_scale > 1.0 ||
      request.behavior.raceline_weight_scale <= 0.0 ||
      request.behavior.safety_weight_scale < 1.0)
    {
      return finish("invalid_behavior");
    }
    constexpr double bounds_epsilon = 1.0e-9;
    if (request.initial_state.speed < vehicle_.min_speed - bounds_epsilon ||
      request.initial_state.speed > vehicle_.max_speed + bounds_epsilon ||
      request.initial_state.steering < vehicle_.min_steering - bounds_epsilon ||
      request.initial_state.steering > vehicle_.max_steering + bounds_epsilon)
    {
      // Measured state is authoritative.  Clamping it would optimize from a
      // fictitious pose/curvature and can make the safety certificate unsound.
      return finish("initial_state_out_of_bounds");
    }

    try {
      prepareCost(
        request.initial_state, *request.race_line, request.obstacles,
        request.behavior);
      if (launch_seed_pending_ && request.initial_state.speed <= 0.05) {
        ControllerT::control_trajectory launch_controls =
          ControllerT::control_trajectory::Zero();
        launch_controls.row(kAccelerationIndex).setConstant(
          static_cast<float>(vehicle_.max_acceleration));
        controller_->updateImportanceSampler(launch_controls);
        launch_seed_pending_ = false;
      }
      auto sampling_params = sampler_->getParams();
      sampling_params.std_dev[kSteeringRateIndex] = static_cast<float>(
        config_.steering_rate_stddev * request.exploration_scale);
      sampling_params.std_dev[kAccelerationIndex] = static_cast<float>(
        config_.acceleration_stddev * request.exploration_scale);
      sampling_params.pure_noise_trajectories_percentage =
        static_cast<float>(config_.pure_noise_fraction);
      sampler_->setParams(sampling_params);
      ControllerT::state_array initial_state;
      initial_state <<
        static_cast<float>(request.initial_state.x),
        static_cast<float>(request.initial_state.y),
        static_cast<float>(normalizeAngle(request.initial_state.yaw)),
        static_cast<float>(request.initial_state.speed),
        static_cast<float>(request.initial_state.steering);

      controller_->computeControl(initial_state, 1);
      const cudaError_t cuda_status = cudaDeviceSynchronize();
      if (cuda_status != cudaSuccess) {
        return finish(std::string("cuda_runtime_error:") + cudaGetErrorString(cuda_status));
      }

      const ControllerT::control_trajectory cuda_controls = controller_->getControlSeq();
      std::vector<Control> controls;
      controls.reserve(kCudaTimesteps);
      for (int step = 0; step < kCudaTimesteps; ++step) {
        const Control control{
          static_cast<double>(cuda_controls(kSteeringRateIndex, step)),
          static_cast<double>(cuda_controls(kAccelerationIndex, step))};
        if (!std::isfinite(control.steering_rate) || !std::isfinite(control.acceleration)) {
          return finish("cuda_nonfinite_control");
        }
        controls.push_back(control);
      }

      const auto repair_duration = std::chrono::duration<double, std::milli>(
        config_.repair_budget_ms);
      const auto repair_deadline = std::chrono::steady_clock::now() +
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(repair_duration);
      bool used_braking_fallback = false;
      if (!validator_.repairControls(
          request.initial_state, controls, *request.race_line,
          request.distance_field, repair_deadline, request.obstacles))
      {
        if (!selectBrakingFallback(request, controls, controls)) {
          return finish("cuda_solution_repair_failed");
        }
        used_braking_fallback = true;
      }

      result.control_sequence = controls;
      result.control = controls.front();
      result.cost = validator_.evaluateTrajectory(
        request.initial_state, controls, *request.race_line,
        request.distance_field, &result.predicted_states, request.obstacles,
        &request.behavior);
      bool finite_cost = std::isfinite(result.cost.total());
      bool complete_trajectory_safe = result.cost.collision <= 0.0;
      if ((!finite_cost || !complete_trajectory_safe) &&
        selectBrakingFallback(request, controls, controls))
      {
        used_braking_fallback = true;
        result.control_sequence = controls;
        result.control = controls.front();
        result.predicted_states.clear();
        result.cost = validator_.evaluateTrajectory(
          request.initial_state, controls, *request.race_line,
          request.distance_field, &result.predicted_states, request.obstacles,
          &request.behavior);
        finite_cost = std::isfinite(result.cost.total());
        complete_trajectory_safe = result.cost.collision <= 0.0;
      }
      result.metrics = validator_.trajectoryMetrics(
        request.initial_state, result.predicted_states, *request.race_line,
        request.distance_field, request.obstacles);
      result.valid = finite_cost && complete_trajectory_safe;
      result.reason = !finite_cost ? "cuda_nonfinite_final_cost" :
        (complete_trajectory_safe ?
        (used_braking_fallback ? "ok_braking_fallback" : "ok") :
        "cuda_final_trajectory_unsafe");
      result.best_rollout = 0U;  // MPPI-Generic v0.9.0 does not expose this index.
      result.solve_time_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - start).count();

      if (result.valid) {
        ControllerT::control_trajectory safe_controls;
        for (int step = 0; step < kCudaTimesteps; ++step) {
          safe_controls(kSteeringRateIndex, step) =
            static_cast<float>(controls[static_cast<std::size_t>(step)].steering_rate);
          safe_controls(kAccelerationIndex, step) =
            static_cast<float>(controls[static_cast<std::size_t>(step)].acceleration);
        }
        controller_->updateImportanceSampler(safe_controls);
        last_control_ = result.control;
        controller_->slideControlSequence(1);
      }
      return result;
    } catch (const std::exception & exception) {
      return finish(std::string("cuda_backend_exception:") + exception.what());
    }
  }

  std::string name() const override
  {
    return "cuda_mppi_generic_v0.9.0";
  }

private:
  bool selectBrakingFallback(
    const MppiRequest & request, const std::vector<Control> & steering_template,
    std::vector<Control> & selected_controls)
  {
    if (request.race_line == nullptr ||
      steering_template.size() != static_cast<std::size_t>(kCudaTimesteps))
    {
      return false;
    }

    std::array<std::vector<Control>, 2> candidates;
    candidates[0] = steering_template;
    for (Control & control : candidates[0]) {
      control.acceleration = vehicle_.min_acceleration;
    }

    candidates[1].resize(kCudaTimesteps);
    double steering = request.initial_state.steering;
    for (Control & control : candidates[1]) {
      control.steering_rate = std::clamp(
        -steering / config_.dt,
        vehicle_.min_steering_rate, vehicle_.max_steering_rate);
      control.acceleration = vehicle_.min_acceleration;
      steering = std::clamp(
        steering + control.steering_rate * config_.dt,
        vehicle_.min_steering, vehicle_.max_steering);
    }

    bool found = false;
    double best_cost = std::numeric_limits<double>::infinity();
    for (std::vector<Control> & candidate : candidates) {
      const auto repair_duration = std::chrono::duration<double, std::milli>(
        config_.repair_budget_ms);
      const auto repair_deadline = std::chrono::steady_clock::now() +
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(repair_duration);
      if (!validator_.repairControls(
          request.initial_state, candidate, *request.race_line,
          request.distance_field, repair_deadline, request.obstacles))
      {
        continue;
      }
      const CostBreakdown cost = validator_.evaluateTrajectory(
        request.initial_state, candidate, *request.race_line,
        request.distance_field, nullptr, request.obstacles, &request.behavior);
      if (cost.collision <= 0.0 && std::isfinite(cost.total()) &&
        cost.total() < best_cost)
      {
        selected_controls = candidate;
        best_cost = cost.total();
        found = true;
      }
    }
    return found;
  }

  bool initializeController()
  {
    int device_count = 0;
    const cudaError_t device_status = cudaGetDeviceCount(&device_count);
    if (device_status != cudaSuccess || device_count < 1) {
      return false;
    }
    if (config_.cuda_max_map_cells == 0U ||
      config_.cuda_max_map_cells >
      std::numeric_limits<std::size_t>::max() / sizeof(float))
    {
      return false;
    }
    releaseDistanceFieldBuffer();
    cudaError_t allocation_status = cudaMalloc(
      reinterpret_cast<void **>(&distance_field_device_),
      config_.cuda_max_map_cells * sizeof(float));
    if (allocation_status != cudaSuccess) {
      distance_field_device_ = nullptr;
      return false;
    }

    F1TenthDynamicsParams dynamics_params;
    dynamics_params.wheelbase = static_cast<float>(vehicle_.wheelbase);
    dynamics_params.min_steering = static_cast<float>(vehicle_.min_steering);
    dynamics_params.max_steering = static_cast<float>(vehicle_.max_steering);
    dynamics_params.min_speed = static_cast<float>(vehicle_.min_speed);
    dynamics_params.max_speed = static_cast<float>(vehicle_.max_speed);
    dynamics_ = std::make_unique<F1TenthDynamics>(dynamics_params, vehicle_);

    cost_ = std::make_unique<F1TenthRaceCost>();
    cost_->setParams(cost_params_);

    SamplingDistribution::SAMPLING_PARAMS_T sampling_params;
    sampling_params.std_dev[kSteeringRateIndex] =
      static_cast<float>(config_.steering_rate_stddev);
    sampling_params.std_dev[kAccelerationIndex] =
      static_cast<float>(config_.acceleration_stddev);
    sampling_params.control_cost_coeff[kSteeringRateIndex] = 1.0F;
    sampling_params.control_cost_coeff[kAccelerationIndex] = 1.0F;
    sampling_params.pure_noise_trajectories_percentage =
      static_cast<float>(config_.pure_noise_fraction);
    sampling_params.rewrite_controls_block_dim = dim3(32, 8, 1);
    sampling_params.sum_strides = 32;
    sampler_ = std::make_unique<SamplingDistribution>(sampling_params);
    feedback_ = std::make_unique<FeedbackControllerT>(static_cast<float>(config_.dt));

    ControllerT::control_trajectory initial_controls =
      ControllerT::control_trajectory::Zero();
    controller_ = std::make_unique<ControllerT>(
      dynamics_.get(), cost_.get(), feedback_.get(), sampler_.get(),
      static_cast<float>(config_.dt), 1, static_cast<float>(config_.lambda),
      0.0F, kCudaTimesteps, initial_controls);
    auto controller_params = controller_->getParams();
    controller_params.seed_ = config_.random_seed;
    // MPPI-Generic requires cost block x <= num_timesteps. Keep 32-thread
    // rollout blocks while the controller uses a fixed 48-step horizon.
    controller_params.dynamics_rollout_dim_ = dim3(32, 1, 1);
    controller_params.cost_rollout_dim_ = dim3(32, 1, 1);
    controller_params.visualize_dim_ = dim3(32, 1, 1);
    controller_params.norm_exp_kernel_parallelization_ = 128;
    controller_params.slide_control_scale_.setOnes();
    controller_->setParams(controller_params);
    controller_->disableFeedbackController();
    controller_->chooseAppropriateKernel();
    initialized_ = true;
    return true;
  }

  void releaseDistanceFieldBuffer() noexcept
  {
    if (distance_field_device_ != nullptr) {
      (void)cudaFree(distance_field_device_);
      distance_field_device_ = nullptr;
    }
  }

  void prepareCost(
    const State & state, const RaceLine & race_line,
    const std::vector<Obstacle> * obstacles = nullptr,
    const MppiBehavior & behavior = MppiBehavior{})
  {
    cost_params_.obstacle_count = 0;
    cost_params_.obstacle_time_offset = 0.0F;
    if (obstacles != nullptr) {
      for (const Obstacle & obstacle : *obstacles) {
        if (cost_params_.obstacle_count >= kMaxCostObstacles) {
          break;
        }
        const int index = cost_params_.obstacle_count;
        cost_params_.obstacle_x[index] = static_cast<float>(obstacle.x);
        cost_params_.obstacle_y[index] = static_cast<float>(obstacle.y);
        cost_params_.obstacle_vx[index] = static_cast<float>(obstacle.vx);
        cost_params_.obstacle_vy[index] = static_cast<float>(obstacle.vy);
        cost_params_.obstacle_cosine[index] =
          static_cast<float>(std::cos(obstacle.yaw));
        cost_params_.obstacle_sine[index] =
          static_cast<float>(std::sin(obstacle.yaw));
        cost_params_.obstacle_half_length[index] =
          static_cast<float>(obstacle.half_length);
        // Same disk-chain construction as obstacleClearance so both backends
        // agree on what counts as contact.
        const double obstacle_target_segment = std::max(0.02, obstacle.half_width);
        const std::size_t obstacle_segments = std::max<std::size_t>(
          1U, static_cast<std::size_t>(
            std::ceil(2.0 * obstacle.half_length / obstacle_target_segment)));
        const double obstacle_segment_length =
          2.0 * obstacle.half_length / static_cast<double>(obstacle_segments);
        cost_params_.obstacle_segments[index] = static_cast<int>(obstacle_segments);
        cost_params_.obstacle_segment_length[index] =
          static_cast<float>(obstacle_segment_length);
        cost_params_.obstacle_cover_radius[index] = static_cast<float>(
          std::hypot(obstacle.half_width, obstacle_segment_length * 0.5));
        // Per-message age is identical across the array; keep the largest so
        // a mixed set stays conservative.
        cost_params_.obstacle_time_offset = std::max(
          cost_params_.obstacle_time_offset,
          static_cast<float>(obstacle.time_offset));
        ++cost_params_.obstacle_count;
      }
    }
    const std::size_t nearest = race_line.nearestIndex(state.x, state.y);
    const Waypoint & initial_waypoint = race_line.waypoints()[nearest];
    const int count = static_cast<int>(std::min<std::size_t>(
      race_line.size(), static_cast<std::size_t>(kLocalWaypointCapacity)));
    const auto start = static_cast<std::ptrdiff_t>(nearest) - kReferencePointsBehind;
    cost_params_.waypoint_count = count;
    for (int index = 0; index < count; ++index) {
      const Waypoint & waypoint = race_line.atWrapped(start + index);
      cost_params_.waypoint_x[index] = static_cast<float>(waypoint.x);
      cost_params_.waypoint_y[index] = static_cast<float>(waypoint.y);
      cost_params_.waypoint_yaw[index] = static_cast<float>(waypoint.yaw);
      cost_params_.waypoint_speed[index] = static_cast<float>(
        std::min(waypoint.reference_speed, vehicle_.max_speed));
      double signed_progress = waypoint.s - initial_waypoint.s;
      if (signed_progress > race_line.length() * 0.5) {
        signed_progress -= race_line.length();
      } else if (signed_progress < -race_line.length() * 0.5) {
        signed_progress += race_line.length();
      }
      cost_params_.waypoint_progress[index] = static_cast<float>(signed_progress);
      cost_params_.waypoint_width_left[index] = static_cast<float>(waypoint.width_left);
      cost_params_.waypoint_width_right[index] = static_cast<float>(waypoint.width_right);
    }

    cost_params_.dt = static_cast<float>(config_.dt);
    cost_params_.wheelbase = static_cast<float>(vehicle_.wheelbase);
    cost_params_.rear_extent = static_cast<float>(-vehicle_.rear_overhang);
    cost_params_.front_extent = static_cast<float>(
      vehicle_.length - vehicle_.rear_overhang);
    cost_params_.half_width = static_cast<float>(vehicle_.width * 0.5);
    cost_params_.safety_margin = static_cast<float>(vehicle_.safety_margin);
    const double target_segment_length = std::max(0.02, vehicle_.width * 0.5);
    const std::size_t segment_count = std::max<std::size_t>(
      1U, static_cast<std::size_t>(std::ceil(vehicle_.length / target_segment_length)));
    const double segment_length = vehicle_.length / static_cast<double>(segment_count);
    cost_params_.footprint_segment_count = static_cast<int>(segment_count);
    cost_params_.footprint_segment_length = static_cast<float>(segment_length);
    cost_params_.footprint_cover_radius = static_cast<float>(
      std::hypot(vehicle_.width * 0.5, segment_length * 0.5));
    cost_params_.max_lateral_acceleration =
      static_cast<float>(config_.max_lateral_acceleration);
    cost_params_.maximum_heading_error =
      static_cast<float>(config_.maximum_heading_error);
    cost_params_.minimum_preview_distance = static_cast<float>(std::min(
        config_.minimum_preview_distance,
        maximumReachableDistance(
          state.speed, vehicle_, config_.dt * static_cast<double>(kCudaTimesteps))));
    cost_params_.barrier_distance = static_cast<float>(config_.repair_clearance);
    cost_params_.barrier_scale = static_cast<float>(std::max(
      0.02, (1.0 - config_.cbf_gamma) * 0.10));
    cost_params_.speed_scale = static_cast<float>(behavior.speed_scale);
    cost_params_.raceline_weight_scale =
      static_cast<float>(behavior.raceline_weight_scale);
    cost_params_.safety_weight_scale =
      static_cast<float>(behavior.safety_weight_scale);
    cost_params_.lateral_reference_offset =
      static_cast<float>(behavior.lateral_reference_offset);
    cost_params_.weight_lateral = static_cast<float>(config_.weights.lateral);
    cost_params_.weight_heading = static_cast<float>(config_.weights.heading);
    cost_params_.weight_lag = static_cast<float>(config_.weights.lag);
    cost_params_.weight_speed = static_cast<float>(config_.weights.speed);
    cost_params_.weight_progress = static_cast<float>(config_.weights.progress);
    cost_params_.weight_control = static_cast<float>(config_.weights.control);
    cost_params_.weight_control_change = static_cast<float>(config_.weights.control_change);
    cost_params_.weight_lateral_acceleration =
      static_cast<float>(config_.weights.lateral_acceleration);
    cost_params_.weight_boundary = static_cast<float>(config_.weights.boundary);
    cost_params_.weight_barrier = static_cast<float>(config_.weights.cbf);
    cost_params_.weight_collision = static_cast<float>(config_.weights.collision);
    cost_params_.weight_terminal_lateral =
      static_cast<float>(config_.weights.terminal_lateral);
    cost_params_.weight_terminal_heading =
      static_cast<float>(config_.weights.terminal_heading);
    cost_params_.weight_terminal_progress =
      static_cast<float>(config_.weights.terminal_progress);
    cost_params_.last_control[0] = static_cast<float>(last_control_.steering_rate);
    cost_params_.last_control[1] = static_cast<float>(last_control_.acceleration);

    if (controller_ != nullptr) {
      const ControllerT::control_trajectory previous = controller_->getControlSeq();
      for (int step = 0; step < kCudaTimesteps; ++step) {
        cost_params_.previous_controls[2 * step] =
          previous(kSteeringRateIndex, step);
        cost_params_.previous_controls[2 * step + 1] =
          previous(kAccelerationIndex, step);
      }
    }
    if (cost_ != nullptr) {
      cost_->setParams(cost_params_);
    }
  }

  MppiConfig config_{};
  VehicleConfig vehicle_{};
  CpuMppiBackend validator_{};
  Control last_control_{};
  F1TenthCostParams cost_params_{};
  bool initialized_{false};
  bool launch_seed_pending_{true};
  float * distance_field_device_{nullptr};
  std::mutex backend_mutex_;

  // Keep the controller last so it is destroyed before the objects it points to.
  std::unique_ptr<F1TenthDynamics> dynamics_;
  std::unique_ptr<F1TenthRaceCost> cost_;
  std::unique_ptr<SamplingDistribution> sampler_;
  std::unique_ptr<FeedbackControllerT> feedback_;
  std::unique_ptr<ControllerT> controller_;
};

}  // namespace

std::unique_ptr<MppiBackend> makeMppiGenericCudaBackend(
  const MppiConfig & config, const VehicleConfig & vehicle)
{
  return std::make_unique<MppiGenericCudaBackend>(config, vehicle);
}

#ifndef MPPI_CONTROLLER_CUDA_ARCH
#define MPPI_CONTROLLER_CUDA_ARCH 87
#endif

bool cudaDeviceMatchesCompiledArchitecture() noexcept
{
  int device_count = 0;
  if (cudaGetDeviceCount(&device_count) != cudaSuccess || device_count < 1) {
    return false;
  }
  cudaDeviceProp properties{};
  if (cudaGetDeviceProperties(&properties, 0) != cudaSuccess) {
    return false;
  }
  // The fatbin carries SASS for exactly one architecture and the vendor
  // kernels abort the process on a mismatched launch, so require equality.
  return properties.major * 10 + properties.minor == MPPI_CONTROLLER_CUDA_ARCH;
}

}  // namespace mppi_controller
