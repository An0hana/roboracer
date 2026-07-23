# mppi_controller

ROS 2 Humble lifecycle controller with two implementations of the same `MppiBackend`
interface:

- `cpu_reference`: deterministic ROS-independent correctness and safety reference.
- `cuda_mppi_generic_v0.9.0`: optional, fixed-shape 2048-rollout/32-step C++/CUDA backend
  built against the pinned `mppi_generic_vendor` package.

`backend=auto` selects CUDA only when the CUDA translation unit was actually compiled and
otherwise reports and uses `cpu_reference`. `backend=cuda` fails configuration on CPU-only
builds; it never silently falls back.

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
  odom_topic:=/state_estimation/odom map_topic:=/map \
  command_topic:=/control/mppi_cmd
```

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
(the default 4,194,304 cells use 16 MiB). A static distance field is converted and uploaded
only in the map callback. Every GPU rollout evaluates the rear-axle-referenced rectangular
body through a conservative disk chain against this field, in addition to race-line,
boundary, speed, control, control-change, lateral-acceleration and smooth barrier costs.
The selected nominal trajectory then passes the CPU bounded first-four-step repair and a
full-horizon CPU cost and collision diagnostic pass before publication. The repair rejects
an unsafe or over-budget first control command; it is a bounded projected repair, not a
general-purpose external QP/SQP solver.

For simulation, pass `command_topic:=/drive use_sim_time:=true` only when no separate command
arbiter owns `/drive`. On hardware, keep `/control/mppi_cmd` and let the safety arbiter own
`/ackermann_cmd`.

The default is 2048 rollouts, 32 steps, 0.05 s model time, and 20 Hz. Those rollout and horizon
sizes are compile-time constants for CUDA; requesting different values with `backend=cuda`
is rejected. The CPU backend uses OpenMP when available, but is a correctness reference
rather than the final Orin performance path. `DiagnosticArray` reports requested, selected,
and compiled backend state, solve time, stop reason, and cost terms.

## CUDA integration notes

- The adapter targets the exact MPPI-Generic v0.9.0 API: `VanillaMPPIController`,
  `GaussianDistribution`, CRTP `Dynamics`/`Cost`, and a disabled zero-feedback controller.
- MPPI-Generic v0.9.0 does not expose the minimum-cost rollout index; `best_rollout` remains
  zero for this backend.
- Upstream v0.9.0 error macros terminate the process for some internal CUDA failures. Run the
  controller behind the independent `safety_controller` watchdog and validate process-restart
  behavior before driving hardware.
