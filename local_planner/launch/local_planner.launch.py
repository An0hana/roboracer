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
    default_raceline = os.path.join(
        get_package_share_directory("roboracer_maps"),
        "maps",
        "racetrack",
        "raceline.csv",
    )
    params_file = LaunchConfiguration("params_file")
    race_line_file = LaunchConfiguration("race_line_file")
    odom_topic = LaunchConfiguration("odom_topic")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "race_line_file",
                default_value=default_raceline,
                description="Standard RoboRacer race-line CSV",
            ),
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
                description="Path to local planner parameters",
            ),
            Node(
                package="local_planner",
                executable="local_planner_node",
                name="local_planner",
                output="screen",
                parameters=[
                    params_file,
                    {
                        "race_line_file": race_line_file,
                        "odom_topic": odom_topic,
                        "use_sim_time": use_sim_time,
                    },
                ],
            ),
        ]
    )
