-- Cartographer configuration: PURE LOCALIZATION mode.
--
-- Runs against a .pbstream produced by my_car_mapping.lua. Cartographer
-- localizes within the frozen map instead of extending it.
--
-- Enabled by the combination of:
--   1. TRAJECTORY_BUILDER.pure_localization_trimmer below, AND
--   2. the -load_state_filename argument on cartographer_node.
-- Both are required. With only the trimmer, there is no map to localize
-- against; with only the argument, Cartographer keeps building the map.

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,

  map_frame = "map",
  tracking_frame = "gyro_link",

  -- Same reasoning as the mapping config: no trusted wheel odometry yet,
  -- so Cartographer publishes map -> odom -> base_link itself.
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = true,
  use_odometry = false,
  publish_tracked_pose = true,
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

-- These MUST match my_car_mapping.lua. Live scans are matched against
-- submaps that were built with these settings.
TRAJECTORY_BUILDER_2D.use_imu_data = true
TRAJECTORY_BUILDER_2D.min_range = 0.1
TRAJECTORY_BUILDER_2D.max_range = 10.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 3.0
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1

-- Keep only a rolling window of new submaps. This is what makes the mode
-- "pure": the loaded map stays frozen and memory stays bounded.
TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 3,
}

-- Optimize more often than while mapping. The graph is small here, and
-- faster optimization means the pose converges sooner after startup.
POSE_GRAPH.optimize_every_n_nodes = 20

return options
