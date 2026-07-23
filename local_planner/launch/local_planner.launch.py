from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory("local_planner"), "config", "params.yaml"
    )
    params_file = LaunchConfiguration("params_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="Path to local planner parameters",
            ),
            Node(
                package="local_planner",
                executable="local_planner_node",
                name="local_planner",
                output="screen",
                parameters=[params_file],
            ),
        ]
    )
