# Race controller bringup

## Interfaces

```text
/scan -> AMCL -> map->odom
VESC odom + IMU -> EKF -> odom->base_link
                          -> state_estimator_bridge -> /state_estimation/odom
/state_estimation/odom + map + raceline -> race_mppi -> /control/mppi_cmd
/scan -> ftg_controller -> /control/ftg_cmd
both commands + scan + state -> race_safety -> /ackermann_cmd
```

`base_link` is the rear-axle centre. The hardware launch publishes the current
provisional `base_link -> laser` translation of `0.275 m`; measure all six TF
components before increasing speed.

## Simulation

The simulator supplies global odometry, so localization is initially bypassed. The static
map is still required for the MPPI footprint/clearance cost:

The normal Spielberg setup is stored in `config/simulation.yaml`. After generating
`/workspace/Spielberg_raceline.csv`, start the complete controller without launch arguments:

```bash
ros2 launch race_bringup race_sim.launch.py
```

Edit `simulation.yaml` to change topics, race-line path, steering limits or speed. The initial
speed limit is deliberately `0.5 m/s`. The generic launch below remains available for ad-hoc
overrides and other maps:

```bash
ros2 launch race_bringup race_stack.launch.py \
  mode:=sim \
  map:=/absolute/path/map.yaml \
  raceline:=/absolute/path/track.csv \
  scan_topic:=/scan \
  state_topic:=/ego_racecar/odom \
  final_drive_topic:=/drive \
  max_speed:=2.0
```

To test AMCL in simulation, add `use_localization:=true` and
`velocity_odom_topic:=/ego_racecar/odom`. The launch then publishes the AMCL pose plus
simulator-measured twist on `/state_estimation/odom`. `require_map:=false` exists only for
isolated controller debugging; do not use it for lap or hardware acceptance.

Keep one MPPI parameter file. Override environment-specific topics, `use_sim_time`, map,
race line and speed at launch time. The CPU backend is a deterministic correctness/reference
path; a build containing the pinned MPPI-Generic CUDA backend is selected by `backend:=auto`.

## Hardware

Install `robot_localization`, verify the independent radio kill switch, and
start the VESC/IMU/LiDAR drivers before this launch:

```bash
ros2 launch race_bringup race_stack.launch.py \
  mode:=hardware \
  use_ekf:=true \
  use_localization:=true \
  map:=/absolute/path/map.yaml \
  raceline:=/absolute/path/track.csv \
  final_drive_topic:=/ackermann_cmd \
  max_speed:=0.3
```

If you intentionally bypass the stock `ackermann_to_vesc` process, add
`use_direct_vesc_adapter:=true`. Never run both converters at the same time.

The values in `config/vesc.yaml` are initial calibration values, not proof of
a safe calibration. `speed_to_erpm_gain` must be identical in the command and
odometry directions. The stock single steering gain cannot represent the
measured left/right asymmetry; use the piecewise adapter or extend the
source-built `vesc_ackermann` node before physical-angle control.

Run with the wheels raised first. Confirm steering direction, centre and hard
stops, then closed-loop zero-speed braking. Unlock real-car speed in the order
`0.3 -> 0.5 -> 1.0 m/s`; at every level verify localization, command/measured
speed error, stopping distance, solver deadline, VESC temperature and Jetson
thermal throttling. Never rely on the software stop instead of the independent
hardware kill switch.

The selected controller can be changed through `race_safety.controller_mode`, but the
request is rejected unless state is fresh and measured speed is below `0.2 m/s`. A failed
MPPI command causes a stop; it never causes an automatic high-speed switch to FTG.

See `DEPLOYMENT_CHECKLIST.md` for build, calibration, fault-injection and release gates.
