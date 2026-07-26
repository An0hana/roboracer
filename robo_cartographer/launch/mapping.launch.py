"""Cartographer in MAPPING mode.

Builds a new map. When finished, save the .pbstream BEFORE stopping this
launch, or the map is lost:

  ros2 service call /finish_trajectory \\
    cartographer_ros_msgs/srv/FinishTrajectory "{trajectory_id: 0}"

  ros2 service call /write_state \\
    cartographer_ros_msgs/srv/WriteState \\
    "{filename: '/home/seeed/maps/racetrack.pbstream',
      include_unfinished_submaps: true}"

Optionally also save a .pgm/.yaml pair for nav2 and other consumers:

  ros2 run nav2_map_server map_saver_cli -f /home/seeed/maps/racetrack
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_path = get_package_share_directory("robo_cartographer")
    config_dir = os.path.join(pkg_path, "config")

    cartographer_node = Node(
        package="cartographer_ros",
        executable="cartographer_node",
        name="cartographer_node",
        output="screen",
        parameters=[{"use_sim_time": False}],
        arguments=[
            "-configuration_directory", config_dir,
            "-configuration_basename", "my_car_mapping.lua",
        ],
        remappings=[
            ("scan", "/scan"),
            ("imu", "/imu/data_raw"),
            # Only consumed when use_odometry is true in the .lua.
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

    # Positional argument order: x y z yaw pitch roll parent child
    # These are placeholders. Measure the real offsets from the rear axle
    # centre; identity values cost accuracy during rotation.
    laser_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="laser_tf",
        arguments=["0", "0", "0", "0", "0", "0", "base_link", "laser"],
    )

    imu_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="imu_tf",
        arguments=["0", "0", "0", "0", "0", "0", "base_link", "gyro_link"],
    )

    return LaunchDescription([
        cartographer_node,
        occupancy_grid_node,
        laser_tf,
        imu_tf,
    ])
