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
    odom_topic = LaunchConfiguration("odom_topic")
    use_known_opponent_size = LaunchConfiguration("use_known_opponent_size")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="Path to opponent tracker parameters",
            ),
            DeclareLaunchArgument(
                "odom_topic",
                default_value="/state_estimation/odom",
                description="Vehicle odometry topic",
            ),
            DeclareLaunchArgument(
                "use_known_opponent_size",
                default_value="true",
                description="Use configured opponent dimensions for center fitting",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use /clock time during rosbag playback",
            ),
            Node(
                package="opponent_tracker",
                executable="opponent_tracker_node",
                name="opponent_tracker",
                output="screen",
                parameters=[
                    params_file,
                    {
                        "odom_topic": odom_topic,
                        "use_known_opponent_size": use_known_opponent_size,
                        "use_sim_time": use_sim_time,
                    },
                ],
            ),
        ]
    )
