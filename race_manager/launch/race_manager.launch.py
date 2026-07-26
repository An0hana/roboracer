from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory("race_manager"), "config", "params.yaml"
    )
    params_file = LaunchConfiguration("params_file")
    odom_topic = LaunchConfiguration("odom_topic")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "odom_topic",
                default_value="/state_estimation/odom",
                description="Vehicle localization topic",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use ROS simulation clock",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="Path to race manager parameters",
            ),
            Node(
                package="race_manager",
                executable="race_manager_node",
                name="race_manager",
                output="screen",
                parameters=[
                    params_file,
                    {
                        "odom_topic": odom_topic,
                        "use_sim_time": use_sim_time,
                    },
                ],
            ),
        ]
    )
