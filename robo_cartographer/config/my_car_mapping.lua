-- Cartographer configuration: MAPPING mode.
--
-- Produces the .pbstream that my_car_localization.lua later consumes.
-- Sensor settings here must stay in sync with the localization config;
-- if max_range differs between the two, the live scans will not match the
-- submaps that were built with it.

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,

  map_frame = "map",
  tracking_frame = "gyro_link",

  -- No trusted wheel odometry yet, so Cartographer owns the whole chain:
  -- it publishes map -> odom -> base_link by itself.
  --
  -- Once ackermann_wheel_odom is verified, switch to:
  --   published_frame    = "odom"
  --   provide_odom_frame = false
  --   use_odometry       = true
  -- and let the odom node publish odom -> base_link.
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = true,
  use_odometry = false,

  publish_frame_projected_to_2d = true,
  use_nav_sat = false,
  use_landmarks = false,

  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,

  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,

  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.
}

MAP_BUILDER.use_trajectory_builder_2d = true

TRAJECTORY_BUILDER_2D.use_imu_data = true
TRAJECTORY_BUILDER_2D.min_range = 0.1

-- Set max_range just under the lidar's real range. Check yours with:
--   ros2 topic echo /scan --field range_max --once
TRAJECTORY_BUILDER_2D.max_range = 10.0

-- Must stay BELOW max_range. When it exceeds max_range, every non-return
-- ray carves free space past the distance the hits are trusted to, which
-- erases wall evidence and bloats the map.
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 3.0

TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1

POSE_GRAPH.optimize_every_n_nodes = 50

return options
