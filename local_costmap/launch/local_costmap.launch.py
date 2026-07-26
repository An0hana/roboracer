from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    scan_topic = LaunchConfiguration("scan_topic")
    odom_topic = LaunchConfiguration("odom_topic")
    costmap_topic = LaunchConfiguration("costmap_topic")
    base_frame = LaunchConfiguration("base_frame")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("local_costmap"), "config", "params.yaml"]
            ),
        ),
        DeclareLaunchArgument("scan_topic", default_value="/scan"),
        DeclareLaunchArgument("odom_topic", default_value="/ego_racecar/odom"),
        DeclareLaunchArgument(
            "costmap_topic", default_value="/perception/local_costmap"
        ),
        DeclareLaunchArgument(
            "base_frame", default_value="ego_racecar/base_link"
        ),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        Node(
            package="local_costmap",
            executable="local_costmap_node",
            name="local_costmap",
            output="screen",
            parameters=[
                params_file,
                {
                    "scan_topic": scan_topic,
                    "odom_topic": odom_topic,
                    "costmap_topic": costmap_topic,
                    "base_frame": base_frame,
                    "use_sim_time": use_sim_time,
                },
            ],
        ),
    ])
