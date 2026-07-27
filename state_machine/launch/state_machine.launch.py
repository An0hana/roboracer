from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory("state_machine"), "config", "params.yaml"
    )
    params_file = LaunchConfiguration("params_file")
    odom_topic = LaunchConfiguration("odom_topic")
    race_line_file = LaunchConfiguration("race_line_file")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "odom_topic",
                default_value="/ego_racecar/odom",
                description="Vehicle localization topic",
            ),
            DeclareLaunchArgument(
                "race_line_file",
                description="Absolute path to the strict raceline CSV",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use ROS simulation clock",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="Path to state machine parameters",
            ),
            Node(
                package="state_machine",
                executable="state_machine_node",
                name="state_machine",
                output="screen",
                parameters=[
                    params_file,
                    {
                        "odom_topic": odom_topic,
                        "race_line_file": race_line_file,
                        "use_sim_time": use_sim_time,
                    },
                ],
            ),
        ]
    )
