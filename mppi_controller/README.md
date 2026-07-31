# mppi_controller

ROS 2 Humble lifecycle controller with two implementations of the same `MppiBackend`
interface:

- `cpu_reference`: deterministic ROS-independent correctness and safety reference.
- `cuda_mppi_generic_v0.9.0`: required runtime, fixed-shape 2048-rollout/72-step C++/CUDA backend
  built against the pinned `mppi_generic_vendor` package.

Simulation and vehicle launches default to `backend=cuda` and fail configuration when CUDA is
unavailable; they never silently fall back. `cpu_reference` remains compiled only for unit
tests, CI and numerical comparison.

## Dynamic obstacles

The controller subscribes to `/perception/obstacles` (`roboracer_msgs/TrackedObstacleArray`,
map frame). Every tracked obstacle except `TRACK_BOUNDARY` enters both backends as a
constant-velocity extrapolation: rollout step `k` evaluates the opponent at
`t = k * dt + measurement_age`, so stale detections are pushed forward before the horizon
even begins. Obstacle clearance shares the map's collision/CBF pipeline through a combined
margin (the vehicle disk chain against a per-obstacle disk chain), which means the repair
stage, the braking fallback and the final CPU safety validation all reject
opponent-intersecting trajectories with exactly the same semantics as wall collisions.
`obstacles_timeout` (default 0.20 s) degrades stale opponent data to costmap-only avoidance
with a WARN diagnostic — never a stop, because an empty track legitimately publishes nothing.
Diagnostics report `obstacle_count`, `obstacle_age_s`, `obstacles_stale` and
`minimum_obstacle_clearance_m`. Up to `max_obstacle_count` (default 8, the compiled CUDA
capacity) nearest obstacles are used per cycle.

Local development note: the CUDA backend compiles for `sm_87` (Jetson Orin) by default.
To run the hardware-gated CUDA tests on an Ada development GPU, rebuild with
`--cmake-args -DMPPI_CONTROLLER_CUDA_ARCHITECTURE=89` and restore `87` before deploying.

## Race line

The controller accepts exactly this CSV header:

```text
s,x,y,yaw,curvature,v_ref,width_left,width_right
```

Generate it from a closed centerline CSV containing at least `x,y`:

```bash
ros2 run mppi_controller generate_raceline.py centerline.csv raceline.csv \
  --spacing 0.05 --width-left 0.75 --width-right 0.75 --max-speed 2.0
```

The optional input columns `width_left`, `width_right`, and `v_ref` override the defaults and
are interpolated. The tool validates all values, resamples the closed loop, computes yaw and
curvature, and applies a lateral-acceleration speed limit.

For maps whose track surface is a closed free-space corridor between inner and outer walls,
generate the complete race line directly from the ROS map:

```bash
ros2 run mppi_controller extract_raceline_from_map.py \
  /absolute/path/Spielberg_map.yaml /absolute/path/Spielberg_raceline.csv \
  --seed-x 0.0 --seed-y 0.0 --spacing 0.05 --max-speed 2.0
```

The seed is any known point on the drivable track; it prevents selecting an infield or the
map exterior. The tool matches both track boundaries, smooths their midpoint, derives local
width from map clearance, and rejects a line that leaves free space or lacks the requested
minimum clearance. Use `--reverse` if the extracted direction opposes the desired lap direction.

## Run

Build the CPU reference path in Release mode:

```bash
colcon build --packages-select mppi_controller --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 launch mppi_controller mppi_controller.launch.py \
  race_line_file:=/absolute/path/raceline.csv \
  odom_topic:=/state_estimation/odom \
  costmap_topic:=/perception/local_costmap \
  command_topic:=/control/mppi_cmd
```

For the current single-car Gym integration, launch the scan-derived rolling costmap and
controller together:

```bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py \
  params_file:=/workspace/src/f1tenth_gym_ros/config/sim_racetrack_1_5x_single.yaml

# In a second terminal:
ros2 launch mppi_controller scan_mppi_sim.launch.py
```

This launch uses `/scan`, `/ego_racecar/odom`, the `racetrack_1_5x` race line and publishes
directly to `/drive`. The Gym simulator must use the matching `racetrack_1_5x` geometry.

On Jetson Orin, build the pinned vendor first and then this package:

```bash
colcon build --packages-select mppi_generic_vendor \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
               -DMPPI_GENERIC_VENDOR_ENABLE_CUDA=ON \
               -DMPPI_GENERIC_CUDA_ARCH=87
source install/setup.bash
colcon build --packages-select mppi_controller \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
               -DMPPI_CONTROLLER_ENABLE_CUDA=ON \
               -DMPPI_CONTROLLER_CUDA_ARCHITECTURE=87
```

The CUDA backend allocates `cuda.max_map_cells` float cells once during lifecycle warmup
(the default 4,194,304 cells use 16 MiB). The latest rolling local costmap is converted to a
distance field and uploaded from its callback. Every GPU rollout evaluates the
rear-axle-referenced rectangular
body through a conservative disk chain against this field, in addition to race-line,
boundary, speed, control, control-change, lateral-acceleration and smooth barrier costs.
The selected nominal trajectory then passes the CPU bounded first-four-step repair and a
full-horizon CPU cost and collision diagnostic pass before publication. If that trajectory
is unsafe, two deterministic maximum-braking candidates (MPPI steering and steering-neutral)
are checked independently before the controller requests a stop. The repair rejects an unsafe
or over-budget first control command; it is a bounded projected repair, not a general-purpose
external QP/SQP solver.

For online Gym simulation, pass `command_topic:=/drive use_sim_time:=false` because the current
bridge does not publish `/clock`. Use simulated time only for clocked simulation or rosbag
playback. On hardware, keep `/control/mppi_cmd` and let the safety arbiter own `/ackermann_cmd`.

The default is 2048 rollouts, 72 steps, 1/30 s model time (2.4 s horizon), and 30 Hz. The
controller rejects reverse/U-turn rollouts, penalizes insufficient terminal progress, and
requires the configured horizon to cover the maximum-speed braking distance. Those rollout
and horizon sizes are compile-time constants for CUDA; requesting different values with
`backend=cuda` is rejected. The CPU backend uses OpenMP when available, but is a correctness
reference rather than the final Orin performance path. `DiagnosticArray` reports requested,
selected and compiled backend state, solve time, stop reason, cost terms, preview/progress,
minimum clearance and braking distance. MPPI-Generic does not expose CUDA rollout validity
counts, so that diagnostic is explicitly reported as unavailable rather than estimated.

## CUDA integration notes

- The adapter targets the exact MPPI-Generic v0.9.0 API: `VanillaMPPIController`,
  `GaussianDistribution`, CRTP `Dynamics`/`Cost`, and a disabled zero-feedback controller.
- MPPI-Generic v0.9.0 does not expose the minimum-cost rollout index; `best_rollout` remains
  zero for this backend.
- Upstream v0.9.0 error macros terminate the process for some internal CUDA failures. Run the
  controller behind the independent `safety_controller` watchdog and validate process-restart
  behavior before driving hardware.
