"""Cartographer in PURE LOCALIZATION mode.

Loads a frozen .pbstream and localizes within it. Usage:

  ros2 launch robo_cartographer localization.launch.py \\
      pbstream:=/home/seeed/maps/racetrack.pbstream

The map must have been saved with /write_state during the mapping run.
A .pgm/.yaml pair is NOT sufficient -- Cartographer needs the .pbstream.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_path = get_package_share_directory("robo_cartographer")
    config_dir = os.path.join(pkg_path, "config")

    pbstream_arg = DeclareLaunchArgument(
        "pbstream",
        default_value="/home/seeed/maps/racetrack.pbstream",
        description="Frozen map state produced by /write_state.",
    )

    cartographer_node = Node(
        package="cartographer_ros",
        executable="cartographer_node",
        name="cartographer_node",
        output="screen",
        parameters=[{"use_sim_time": False}],
        arguments=[
            "-configuration_directory", config_dir,
            "-configuration_basename", "my_car_localization.lua",
	    "-start_trajectory_with_default_topics=false",
            # This argument is half of what enables pure localization; the
            # other half is pure_localization_trimmer in the .lua.
            "-load_state_filename", LaunchConfiguration("pbstream"),
        ],
        remappings=[
            ("scan", "/scan"),
            ("imu", "/imu/data_raw"),
            ("odom", "/wheel/odometry"),
        ],
    )

    occupancy_grid_node = Node(
        package="cartographer_ros",
        executable="cartographer_occupancy_grid_node",
        name="cartographer_occupancy_grid_node",
        output="screen",
        arguments=["-resolution", "0.05"],
    )

    laser_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="laser_tf",
        # 2D projection: both sensors are 0.25 m ahead of the rear-axle
        # base_link origin. Height is intentionally ignored.
        arguments=["0.25", "0", "0", "0", "0", "0", "base_link", "laser"],
    )

    imu_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="imu_tf",
        arguments=["0.25", "0", "0", "0", "0", "0", "base_link", "gyro_link"],
    )

    return LaunchDescription([
        pbstream_arg,
        cartographer_node,
        occupancy_grid_node,
        laser_tf,
        imu_tf,
    ])
