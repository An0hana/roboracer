from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg_path = get_package_share_directory("robo_cartographer")

    config_dir = os.path.join(pkg_path, "config")

    return LaunchDescription([

        Node(
            package="cartographer_ros",
            executable="cartographer_node",
            output="screen",

            parameters=[
                {"use_sim_time": False}
            ],

            arguments=[
                "-configuration_directory", config_dir,
                "-configuration_basename", "my_car.lua"
            ],

            remappings=[
                ("scan", "/scan"),
                ("imu", "/imu/data_raw"),
                ("odom", "/wheel/odometry")
            ]
        ),

        Node(
            package="cartographer_ros",
            executable="cartographer_occupancy_grid_node",
            output="screen",

            arguments=[
                "-resolution", "0.05"
            ]
        ),

        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="laser_tf",
            arguments=["0.25", "0", "0", "0", "0", "0", "base_link", "laser",
            ],
        ),

        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="imu_tf",
            arguments=["0.25", "0", "0", "0", "0", "0", "base_link", "gyro_link",
            ],
        )
    ])
