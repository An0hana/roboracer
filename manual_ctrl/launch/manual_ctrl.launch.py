"""Launch the command mux and the manual gamepad node together.

IMPORTANT -- the exclusive takeover only works if the mux is the ONLY node
publishing to the VESC command topics. Your autonomous stack must be remapped
to publish to the mux's autonomous inputs instead:

    /commands/motor/speed     -> /autonomous/motor
    /commands/servo/position  -> /autonomous/servo

For ackermann_to_vesc, remap its outputs on its own launch, e.g.:
    remappings=[
        ('commands/motor/speed', '/autonomous/motor'),
        ('commands/servo/position', '/autonomous/servo'),
    ]
If anything still publishes directly to /commands/motor/speed, it bypasses
the mux and the takeover will not be exclusive.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('manual_ctrl'),
        'config', 'manual_ctrl.yaml')

    config_arg = DeclareLaunchArgument(
        'config_file', default_value=default_config,
        description='Parameter file for both nodes.')

    config = LaunchConfiguration('config_file')

    mux = Node(
        package='manual_ctrl',
        executable='manual_mux_node',
        name='manual_mux',
        output='screen',
        parameters=[config],
    )

    manual = Node(
        package='manual_ctrl',
        executable='manual_control_node',
        name='manual_ctrl',
        output='screen',
        parameters=[config],
    )

    return LaunchDescription([config_arg, mux, manual])
