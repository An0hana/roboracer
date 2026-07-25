"""Launch the adaptive Follow-the-Gap controller."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory("ftg_controller"), "config", "params.yaml"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=params_file,
                description="Full path to the FTG parameter file",
            ),
            DeclareLaunchArgument(
                "node_name",
                default_value="ftg_controller",
                description="Unique ROS node name",
            ),
            DeclareLaunchArgument(
                "scan_topic",
                default_value="/scan",
                description="LaserScan input topic",
            ),
            DeclareLaunchArgument(
                "drive_topic",
                default_value="/ackermann_cmd",
                description="Ackermann command output topic",
            ),
            Node(
                package="ftg_controller",
                executable="ftg_controller_node",
                name=LaunchConfiguration("node_name"),
                output="screen",
                parameters=[
                    LaunchConfiguration("params_file"),
                    {
                        "scan_topic": LaunchConfiguration("scan_topic"),
                        "drive_topic": LaunchConfiguration("drive_topic"),
                    },
                ],
            ),
        ]
    )
