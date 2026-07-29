"""Low-speed hardware controller stack.

This launch intentionally starts only local_costmap, MPPI, and the independent
safety arbiter. Sensor drivers, localization, VESC, and manual takeover must be
started and verified before this file is launched.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def include(package, launch_file, arguments):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package), "launch", launch_file])
        ),
        launch_arguments=arguments.items(),
    )


def generate_launch_description():
    race_line_file = LaunchConfiguration("race_line_file")
    scan_topic = LaunchConfiguration("scan_topic")
    odom_topic = LaunchConfiguration("odom_topic")
    base_frame = LaunchConfiguration("base_frame")
    max_speed = LaunchConfiguration("max_speed")
    min_steering = LaunchConfiguration("min_steering")
    max_steering = LaunchConfiguration("max_steering")

    costmap = include(
        "local_costmap",
        "local_costmap.launch.py",
        {
            "params_file": PathJoinSubstitution(
                [FindPackageShare("local_costmap"), "config", "params.yaml"]
            ),
            "scan_topic": scan_topic,
            "odom_topic": odom_topic,
            "costmap_topic": "/perception/local_costmap",
            "base_frame": base_frame,
            "use_sim_time": "false",
        },
    )
    controller = include(
        "mppi_controller",
        "mppi_controller.launch.py",
        {
            "params_file": PathJoinSubstitution(
                [FindPackageShare("mppi_controller"), "config", "params.yaml"]
            ),
            "race_line_file": race_line_file,
            "backend": "cuda",
            "odom_topic": odom_topic,
            "costmap_topic": "/perception/local_costmap",
            "command_topic": "/control/mppi_cmd",
            "max_speed": max_speed,
            "min_steering": min_steering,
            "max_steering": max_steering,
            "use_sim_time": "false",
            "autostart": "true",
            "require_race_state": "false",
        },
    )
    safety = include(
        "safety_controller",
        "safety_controller.launch.py",
        {
            "params_file": PathJoinSubstitution(
                [FindPackageShare("safety_controller"), "config", "params.yaml"]
            ),
            "scan_topic": scan_topic,
            "odom_topic": odom_topic,
            "mppi_cmd_topic": "/control/mppi_cmd",
            "output_topic": "/ackermann_cmd",
            "max_speed": max_speed,
            "min_steering": min_steering,
            "max_steering": max_steering,
            "use_sim_time": "false",
        },
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "race_line_file",
                description="Absolute path to the map-aligned race-line CSV",
            ),
            DeclareLaunchArgument("scan_topic", default_value="/scan"),
            DeclareLaunchArgument(
                "odom_topic", default_value="/state_estimation/odom"
            ),
            DeclareLaunchArgument("base_frame", default_value="base_link"),
            DeclareLaunchArgument(
                "max_speed",
                default_value="0.30",
                description="First-run hard speed limit in m/s",
            ),
            DeclareLaunchArgument("min_steering", default_value="-0.32"),
            DeclareLaunchArgument("max_steering", default_value="0.32"),
            costmap,
            controller,
            safety,
        ]
    )
