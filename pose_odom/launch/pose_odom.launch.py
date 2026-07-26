import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('pose_odom'),
        'config',
        'pose_odom.yaml',
    )

    config_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='pose_odom parameter file.',
    )

    node = Node(
        package='pose_odom',
        executable='pose_odom_node',
        name='pose_odom',
        output='screen',
        parameters=[LaunchConfiguration('config_file')],
    )

    return LaunchDescription([config_arg, node])
