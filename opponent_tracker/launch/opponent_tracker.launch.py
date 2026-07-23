from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory("opponent_tracker"), "config", "params.yaml"
    )
    params_file = LaunchConfiguration("params_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="Path to opponent tracker parameters",
            ),
            Node(
                package="opponent_tracker",
                executable="opponent_tracker_node",
                name="opponent_tracker",
                output="screen",
                parameters=[params_file],
            ),
        ]
    )
