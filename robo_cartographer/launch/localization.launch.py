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

DUPLICATE NODES
---------------
launch_pose_odom defaults to false because the main bringup already starts
pose_odom. Two instances publish to /state_estimation/odom independently, each
with its own gate state, and the controller sees them interleaved. Set it true
only when running this launch file standalone.
"""

import math
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            LogInfo, OpaqueFunction,
                            RegisterEventHandler, TimerAction)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _start_trajectory(context, *args, **kwargs):
    """Build and issue the /start_trajectory request from launch arguments."""
    del args, kwargs

    config_dir = os.path.join(
        get_package_share_directory("robo_cartographer"), "config")

    x = float(LaunchConfiguration("start_x").perform(context))
    y = float(LaunchConfiguration("start_y").perform(context))
    yaw = float(LaunchConfiguration("start_yaw").perform(context))

    # Planar, so only the z/w quaternion components are non-trivial.
    qz = math.sin(0.5 * yaw)
    qw = math.cos(0.5 * yaw)

    request = (
        "{{configuration_directory: '{cfg}', "
        "configuration_basename: 'my_car_localization.lua', "
        "use_initial_pose: true, "
        "initial_pose: {{position: {{x: {x}, y: {y}, z: 0.0}}, "
        "orientation: {{x: 0.0, y: 0.0, z: {qz}, w: {qw}}}}}, "
        "relative_to_trajectory_id: 0}}"
    ).format(cfg=config_dir, x=x, y=y, qz=qz, qw=qw)

    return [
        LogInfo(msg="[localization] starting trajectory at "
                    "x={:.3f} y={:.3f} yaw={:.3f} rad".format(x, y, yaw)),
        ExecuteProcess(
            cmd=["ros2", "service", "call", "/start_trajectory",
                 "cartographer_ros_msgs/srv/StartTrajectory", request],
            output="screen",
        ),
    ]


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
            default_value="4.0",
            description="Seconds to wait after the node starts before calling "
                        "/start_trajectory. Loading a large .pbstream takes a "
                        "while; the call fails if it arrives too early.",
        ),
        DeclareLaunchArgument(
            "launch_pose_odom",
            default_value="false",
            description="Start pose_odom here. Leave false if the main bringup "
                        "already starts it -- two instances fight over "
                        "/state_estimation/odom.",
        ),
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

    start_trajectory = RegisterEventHandler(
        OnProcessStart(
            target_action=cartographer_node,
            on_start=[
                TimerAction(
                    period=LaunchConfiguration("start_delay"),
                    actions=[OpaqueFunction(function=_start_trajectory)],
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
            os.path.join(
                get_package_share_directory("pose_odom"),
                "config", "pose_odom.yaml"),
            {
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
