from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    odom_topic = LaunchConfiguration("odom_topic")
    scan_topic = LaunchConfiguration("scan_topic")
    base_frame = LaunchConfiguration("base_frame")
    command_topic = LaunchConfiguration("command_topic")
    max_speed = LaunchConfiguration("max_speed")
    speed_weight = LaunchConfiguration("speed_weight")
    use_sim_time = LaunchConfiguration("use_sim_time")
    race_line_file = LaunchConfiguration("race_line_file")
    backend = LaunchConfiguration("backend")
    params_file = LaunchConfiguration("params_file")

    local_costmap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("local_costmap"),
                "launch",
                "local_costmap.launch.py",
            ])
        ),
        launch_arguments={
            "scan_topic": scan_topic,
            "odom_topic": odom_topic,
            "costmap_topic": "/perception/local_costmap",
            "base_frame": base_frame,
            "use_sim_time": use_sim_time,
        }.items(),
    )

    mppi = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("mppi_controller"),
                "launch",
                "mppi_controller.launch.py",
            ])
        ),
        launch_arguments={
            "params_file": params_file,
            "race_line_file": race_line_file,
            "backend": backend,
            "odom_topic": odom_topic,
            "costmap_topic": "/perception/local_costmap",
            "command_topic": command_topic,
            "max_speed": max_speed,
            "speed_weight": speed_weight,
            "use_sim_time": use_sim_time,
            "autostart": "true",
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument("scan_topic", default_value="/scan"),
        DeclareLaunchArgument("backend", default_value="cuda"),
        DeclareLaunchArgument("odom_topic", default_value="/ego_racecar/odom"),
        DeclareLaunchArgument(
            "base_frame", default_value="ego_racecar/base_link"
        ),
        DeclareLaunchArgument("command_topic", default_value="/drive"),
        DeclareLaunchArgument("max_speed", default_value="0.5"),
        DeclareLaunchArgument(
            "params_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("mppi_controller"),
                "config",
                "params.yaml",
            ]),
            description="MPPI parameter YAML (tuning experiments may override)",
        ),
        DeclareLaunchArgument("speed_weight", default_value="50.0"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument(
            "race_line_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("roboracer_maps"),
                "maps",
                "racetrack_1_5x",
                "raceline.csv",
            ]),
        ),
        local_costmap,
        mppi,
    ])
