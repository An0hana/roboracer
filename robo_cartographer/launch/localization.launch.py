"""Cartographer in PURE LOCALIZATION mode.

  ros2 launch robo_cartographer localization.launch.py \\
      pbstream:=/home/seeed/f1tenth_ws/map/racetrack.pbstream

The map must have been saved with /write_state during the mapping run.
A .pgm/.yaml pair is NOT sufficient -- Cartographer needs the .pbstream.

HOW THE START POSE WORKS
------------------------
my_car_localization.lua sets POSE_GRAPH.global_sampling_ratio = 0, so
Cartographer will not search the map to find itself. It has to be told where it
starts, and there is exactly one supported way to do that:

  * NOT via TRAJECTORY_BUILDER.initial_trajectory_pose in the .lua. That key is
    only read on the /start_trajectory service path. Left in the .lua it is
    never consumed and Cartographer aborts with
    "Key 'initial_trajectory_pose' was used the wrong number of times."

  * Via the /start_trajectory service. Which means the default trajectory must
    be suppressed (-start_trajectory_with_default_topics=false), and something
    must actually make the call -- if nothing does, no trajectory ever starts,
    the map frame never appears, and every downstream node reports
    state_age=inf with "map passed to lookupTransform does not exist".

This launch file makes that call itself, on a timer after the node is up, so it
cannot be forgotten. Set the grid position via start_x/start_y/start_yaw.

POSE_ODOM OWNERSHIP
-------------------
launch_pose_odom defaults to TRUE. vehicle_bringup includes this file at its
Stage 2 and expects it to own the single pose_odom instance, so this file must
start it. Set false only if some other launch file in your tree starts
pose_odom -- two instances publish to /state_estimation/odom independently,
each with its own gate state, and the controller sees them interleaved.

ARGUMENTS FROM vehicle_bringup
------------------------------
vehicle_bringup passes map, params_file and pose_odom_params because the same
include also drives amcl_localization.launch.py. Only pose_odom_params is used
here; map and params_file are declared so they are accepted and ignored rather
than silently dropped.

It does NOT pass pbstream, so `pbstream:=...` on the vehicle_bringup command
line goes nowhere and the default below is what gets loaded. To make that
argument work, add to vehicle_bringup:
    DeclareLaunchArgument("pbstream", default_value="<path>.pbstream")
and forward it in the localization include:
    {"pbstream": LaunchConfiguration("pbstream"), ...}
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, RegisterEventHandler,
                            TimerAction)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_path = get_package_share_directory("robo_cartographer")
    config_dir = os.path.join(pkg_path, "config")

    declarations = [
        DeclareLaunchArgument(
            "pbstream",
            default_value="/home/seeed/f1tenth_ws/map/racetrack.pbstream",
            description="Frozen map state produced by /write_state.",
        ),
        # Bag full_vehicle_after_msg_fix_02 started here. Change to wherever
        # the car actually grids up -- with global relocalization off, a wrong
        # value here is not recoverable.
        DeclareLaunchArgument("start_x", default_value="-3.80"),
        DeclareLaunchArgument("start_y", default_value="-0.16"),
        DeclareLaunchArgument("start_yaw", default_value="-0.133"),
        DeclareLaunchArgument(
            "start_delay",
            default_value="2.0",
            description="Seconds to wait after the node starts before calling "
                        "/start_trajectory. The client also waits for the "
                        "service itself, so this only needs to be small.",
        ),
        # TRUE by default: vehicle_bringup includes this file and expects it
        # to own the single pose_odom instance ("amcl_localization.launch.py
        # owns the only pose_odom instance so map->odom and odom->base_link
        # each have exactly one publisher"). Defaulting this to false meant no
        # pose_odom started at all, /state_estimation/odom was never published,
        # and every downstream node reported state_age=inf.
        # Set false ONLY if something else in your tree starts pose_odom.
        DeclareLaunchArgument(
            "launch_pose_odom",
            default_value="true",
            description="Start pose_odom here. Set false only if another "
                        "launch file already starts it -- two instances fight "
                        "over /state_estimation/odom.",
        ),
        # Config path for pose_odom. vehicle_bringup passes this through.
        DeclareLaunchArgument(
            "pose_odom_params",
            default_value=PathJoinSubstitution(
                [FindPackageShare("pose_odom"), "config", "pose_odom.yaml"]),
            description="pose_odom parameter file.",
        ),
        # Accepted and ignored. vehicle_bringup passes these because it also
        # drives amcl_localization.launch.py; declaring them here keeps the
        # same include working for both without editing the bringup.
        DeclareLaunchArgument("map", default_value=""),
        DeclareLaunchArgument("params_file", default_value=""),
    ]

    cartographer_node = Node(
        package="cartographer_ros",
        executable="cartographer_node",
        name="cartographer_node",
        output="screen",
        parameters=[{"use_sim_time": False}],
        arguments=[
            "-configuration_directory", config_dir,
            "-configuration_basename", "my_car_localization.lua",
            "-load_state_filename", LaunchConfiguration("pbstream"),
            # Suppressed so the trajectory can be started with an explicit
            # pose below. Without the service call that follows, nothing ever
            # starts and no map frame is published.
            "-start_trajectory_with_default_topics=false",
        ],
        remappings=[
            ("scan", "/scan"),
            ("imu", "/imu/data_raw"),
            ("odom", "/wheel/odometry"),
        ],
    )

    # A real service client, not ExecuteProcess(["ros2", "service", "call"]).
    # Shelling out failed silently: `ros2` is not guaranteed to be on the
    # launch subprocess PATH, and a non-zero exit is not surfaced anywhere, so
    # the trajectory never started and every node reported state_age=inf.
    # This node waits for the service, retries, refuses to create a second
    # active trajectory, and logs the actual status code on failure.
    start_trajectory = RegisterEventHandler(
        OnProcessStart(
            target_action=cartographer_node,
            on_start=[
                TimerAction(
                    period=LaunchConfiguration("start_delay"),
                    actions=[Node(
                        package="robo_cartographer",
                        executable="start_trajectory.py",
                        name="start_trajectory_client",
                        output="screen",
                        parameters=[{
                            "configuration_directory": config_dir,
                            "configuration_basename":
                                "my_car_localization.lua",
                            "start_x": ParameterValue(
                                LaunchConfiguration("start_x"), value_type=float),
                            "start_y": ParameterValue(
                                LaunchConfiguration("start_y"), value_type=float),
                            "start_yaw": ParameterValue(
                                LaunchConfiguration("start_yaw"), value_type=float),
                            "relative_to_trajectory_id": 0,
                        }],
                    )],
                )
            ],
        )
    )

    occupancy_grid_node = Node(
        package="cartographer_ros",
        executable="cartographer_occupancy_grid_node",
        name="cartographer_occupancy_grid_node",
        output="screen",
        arguments=["-resolution", "0.05"],
    )

    # Gates Cartographer's map pose against dead reckoning before it reaches
    # the controller. publish_odom_tf stays false: Cartographer owns
    # odom->base_link in this stack (provide_odom_frame = true).
    pose_odom = Node(
        package="pose_odom",
        executable="pose_odom_node",
        name="pose_odom",
        output="screen",
        condition=IfCondition(LaunchConfiguration("launch_pose_odom")),
        parameters=[
            LaunchConfiguration("pose_odom_params"),
            {
                # Cartographer owns odom->base_link here
                # (provide_odom_frame = true), so pose_odom must not also
                # publish that edge.
                "publish_odom_tf": False,
                "map_pose_source": "tracked_pose",
            },
        ],
    )

    # 2D projection: both sensors are 0.25 m ahead of the rear-axle base_link
    # origin. Height is intentionally ignored.
    laser_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="laser_tf",
        arguments=["0.25", "0", "0", "0", "0", "0", "base_link", "laser"],
    )

    imu_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="imu_tf",
        arguments=["0.25", "0", "0", "0", "0", "0", "base_link", "gyro_link"],
    )

    return LaunchDescription(
        declarations + [
            cartographer_node,
            start_trajectory,
            occupancy_grid_node,
            pose_odom,
            laser_tf,
            imu_tf,
        ]
    )
