# Orin Nano / TRA6804R deployment checklist

This stack fails closed, but software stopping is not a substitute for the radio kill
switch. Keep the driven wheels off the ground until every item in sections 1--3 passes.

## 1. Build and platform

- Install ROS 2 Humble packages for `nav2_map_server`, `nav2_amcl`,
  `nav2_lifecycle_manager`, and `robot_localization`.
- Select an Orin Nano power profile that is explicitly rated for sustained 15 W on the
  installed JetPack release. Verify it with `nvpmodel -q`; record `tegrastats` during tests.
- Build the pinned CUDA dependency and controller in Release mode:

  ```bash
  colcon build --packages-select mppi_generic_vendor \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
                 -DMPPI_GENERIC_VENDOR_ENABLE_CUDA=ON \
                 -DMPPI_GENERIC_CUDA_ARCH=87
  colcon build --packages-up-to race_bringup \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
                 -DRACE_MPPI_ENABLE_CUDA=ON
  ```

- Run all tests after sourcing the resulting workspace. Do not use a build produced for
  another JetPack/CUDA version on the competition computer.

## 2. Geometry, steering, and TF

- Define `base_link` at the rear-axle centre. Measure wheelbase, front/rear overhang,
  body width and the full six-degree-of-freedom LiDAR transform.
- Current provisional values are wheelbase `0.324 m`, body `0.568 x 0.296 m`, rear
  overhang `0.100 m`, and LiDAR `x=0.275 m`. Replace every provisional value before
  increasing speed.
- Current measured steering endpoints imply these initial piecewise mappings:

  ```text
  centre: servo 0.50
  left:   servo = 0.50 - 1.13 * steering_rad
  right:  servo = 0.50 - 1.52 * steering_rad
  ```

  Positive steering means left. Confirm the sign with wheels raised. Keep the controller
  at the conservative symmetric limit `[-0.20, +0.20] rad` initially; only enable the
  measured asymmetric limits after a repeatable wheel-angle calibration.

## 3. VESC and stopping

- Keep command-side and odometry-side `speed_to_erpm_gain` identical. The current
  `4614 erpm/(m/s)` is only a starting value: calibrate it over a measured distance in
  both directions and record the offset and latency.
- Verify closed-loop speed at `0.3`, `0.5`, then `1.0 m/s`; compare requested speed,
  VESC ERPM-derived speed and an external distance/time measurement.
- Separately test zero-speed holding, coast behaviour, regenerative/current braking and
  `/commands/motor/brake`. The optional direct adapter sends zero ERPM on timeout; that
  alone is **not evidence of a reliable brake**.
- Measure stopping distance at each allowed speed on the real surface and use the result
  to set the AEB deceleration/reaction margins.
- Never run the stock `ackermann_to_vesc_node` and
  `vesc_command_adapter_node` simultaneously.

## 4. Localization and track data

- Verify a single TF tree: `map -> odom -> base_link -> laser`; there must be no second
  publisher for any transform in that chain.
- Confirm the EKF uses measured longitudinal wheel speed and IMU yaw rate. Do not treat
  steering-command-derived yaw rate as an independent IMU measurement.
- Create a closed race line with the exact header
  `s,x,y,yaw,curvature,v_ref,width_left,width_right`, sampled every `0.05 m`.
- Visually overlay the race line and rectangular footprint on the occupancy map before
  enabling commands. Unknown and out-of-map cells are occupied by default.

## 5. Release gates

- Simulation: three collision-free laps at `2.0 m/s`; no NaN or unexpected stop;
  lateral error p95 below `0.10 m` and maximum below `0.18 m`.
- Orin soak: at least ten minutes, solver p99 below `40 ms`, no two consecutive missed
  50 ms periods, no thermal throttling and no control-loop CUDA allocations.
- Fault injection: stop on stale state, stale scan, stale command, missing map, invalid
  race line, localization jump, NaN, solver timeout, CUDA failure and controller exit.
- Real car: pass wheels-up tests, straight speed calibration, left/right constant-radius
  tests and braking tests before the first closed loop. Unlock `0.3 -> 0.5 -> 1.0 m/s`
  only after each previous level passes.

