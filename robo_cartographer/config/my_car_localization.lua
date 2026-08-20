-- Cartographer configuration: PURE LOCALIZATION mode.
--
-- Measured behaviour this config responds to (bag: full_vehicle_after_msg_fix_02,
-- 169 s, 2 laps, 178 m, speeds to 2.8 m/s):
--
--   * map->odom x sat at -0.574 m for 14 s, snapped to -1.157 m (+0.585 m),
--     held there for a full 52 s lap, then snapped back to -0.540 m. A
--     bistable ~0.6 m along-x flip, not accumulated drift.
--   * The weak axis is x. The worst-conditioned 5% of scans cluster on the
--     20 m bottom straight (x in [-10,-2], y ~ 0) with weak direction 171 deg
--     in the map frame.
--   * Shifting a scan 0.6 m longitudinally on that straight costs only ~22% of
--     match score, versus ~50% in the twisty section. No single min_score is
--     safe for both, which is why threshold tuning alone kept failing:
--     0.55 -> 0.65 moved the jumps from 1.37 m to 0.59 m, not to zero.
--
-- Things checked and ruled out, so nobody re-tries them:
--   * Scan quality: 1021/1021 valid returns per scan, zero inf/nan.
--   * max_range: only ~5 rays/scan land between 10 and 15 m on that straight,
--     everything past 30 m is a no-return. Raising max_range to 25 changes
--     the conditioning from 2.74 to 2.71. Not worth it.
--   * Map consistency: lap-A vs lap-B scan overlays are crisp, no doubling.

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,

  map_frame = "map",
  tracking_frame = "gyro_link",

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

  -- 40 Hz scans, 17.7 ms sweep, yaw rates to 108 deg/s. That is up to 14 cm of
  -- un-modelled skew at 4 m range across one sweep. Subdividing lets the pose
  -- extrapolator de-skew properly.
  num_subdivisions_per_laser_scan = 10,
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

-- These MUST match my_car_mapping.lua. Live scans are matched against submaps
-- built with these settings, so changing one without remapping breaks both.
TRAJECTORY_BUILDER_2D.use_imu_data = true
TRAJECTORY_BUILDER_2D.min_range = 0.1
TRAJECTORY_BUILDER_2D.max_range = 10.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 3.0

-- Must equal num_subdivisions_per_laser_scan so a full sweep is accumulated
-- before matching.
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 10

TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(15.)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 10.
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1.

TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 20.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 40.

-- Smaller active graph means a re-solve has less freedom to move the whole
-- trajectory at once.
TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 2,
}

-- NOTE: initial_trajectory_pose does NOT belong here.
--
-- Cartographer reference-counts every Lua key and aborts if one is never read:
--   Check failed: 1 == reference_counts_.count(key) (1 vs 0)
--   Key 'initial_trajectory_pose' was used the wrong number of times.
--
-- That key is only consumed on the /start_trajectory SERVICE path, not by the
-- default trajectory the node starts at launch. Because global relocalization
-- is disabled below, the start pose has to come from that service instead --
-- localization.launch.py issues the call automatically a few seconds after the
-- node comes up, so it cannot be forgotten. Edit the pose there, not here.

-- ---------------------------------------------------------------------------
-- Constraint policy.
--
-- Global relocalization stays OFF. A full-map branch-and-bound search on a
-- repetitive track will eventually score a wrong match highly, and on the
-- bottom straight a 0.6 m error barely dents the score.
POSE_GRAPH.global_sampling_ratio = 0.0
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.85

-- CORRECTION to earlier advice: sampling_ratio was briefly raised to 0.3.
-- That is 6x more constraint attempts, which on this track means more chances
-- to land a wrong one. Back down. 0.1 still gives the graph plenty to work
-- with while keeping the local matcher, not the graph, in charge.
POSE_GRAPH.constraint_builder.sampling_ratio = 0.1
POSE_GRAPH.constraint_builder.max_constraint_distance = 6.0
POSE_GRAPH.constraint_builder.min_score = 0.75

-- Bounds how far any single local constraint can move the solution. A 0.6 m
-- candidate cannot be proposed at all with a 0.3 m window. Note this does NOT
-- bound MatchFullSubmap, which is why global_sampling_ratio must be 0.
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 0.3
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(10.)
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.branch_and_bound_depth = 6

-- Huber down-weights any outlier constraint that still gets through, so one
-- bad match cannot dominate the solve.
POSE_GRAPH.optimization_problem.huber_scale = 1e1

-- Optimize more often so corrections arrive as many small nudges rather than
-- one large step. Cheap: the graph is tiny in pure localization.
POSE_GRAPH.optimize_every_n_nodes = 20

POSE_GRAPH.optimization_problem.ceres_solver_options.num_threads = 6
POSE_GRAPH.constraint_builder.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 10

return options
