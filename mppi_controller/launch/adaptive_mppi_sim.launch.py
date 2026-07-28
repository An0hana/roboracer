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
    odom_topic = LaunchConfiguration("odom_topic")
    scan_topic = LaunchConfiguration("scan_topic")
    base_frame = LaunchConfiguration("base_frame")
    command_topic = LaunchConfiguration("command_topic")
    max_speed = LaunchConfiguration("max_speed")
    use_sim_time = LaunchConfiguration("use_sim_time")
    race_line_file = LaunchConfiguration("race_line_file")
    costmap_params_file = LaunchConfiguration("costmap_params_file")
    tracker_params_file = LaunchConfiguration("tracker_params_file")
    state_machine_params_file = LaunchConfiguration("state_machine_params_file")
    mppi_params_file = LaunchConfiguration("mppi_params_file")

    costmap = include(
        "local_costmap",
        "local_costmap.launch.py",
        {
            "params_file": costmap_params_file,
            "scan_topic": scan_topic,
            "odom_topic": odom_topic,
            "costmap_topic": "/perception/local_costmap",
            "base_frame": base_frame,
            "use_sim_time": use_sim_time,
        },
    )
    tracker = include(
        "opponent_tracker",
        "opponent_tracker.launch.py",
        {
            "params_file": tracker_params_file,
            "odom_topic": odom_topic,
            "use_sim_time": use_sim_time,
            "use_known_opponent_size": "true",
        },
    )
    state_machine = include(
        "state_machine",
        "state_machine.launch.py",
        {
            "params_file": state_machine_params_file,
            "odom_topic": odom_topic,
            "race_line_file": race_line_file,
            "use_sim_time": use_sim_time,
        },
    )
    controller = include(
        "mppi_controller",
        "mppi_controller.launch.py",
        {
            "params_file": mppi_params_file,
            "race_line_file": race_line_file,
            "backend": "cuda",
            "odom_topic": odom_topic,
            "costmap_topic": "/perception/local_costmap",
            "command_topic": command_topic,
            "max_speed": max_speed,
            "use_sim_time": use_sim_time,
            "autostart": "true",
            "require_race_state": "true",
        },
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("scan_topic", default_value="/scan"),
            DeclareLaunchArgument("odom_topic", default_value="/ego_racecar/odom"),
            DeclareLaunchArgument(
                "base_frame", default_value="ego_racecar/base_link"
            ),
            DeclareLaunchArgument("command_topic", default_value="/drive"),
            DeclareLaunchArgument("max_speed", default_value="2.0"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "costmap_params_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("local_costmap"), "config", "params.yaml"]
                ),
            ),
            DeclareLaunchArgument(
                "tracker_params_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("opponent_tracker"), "config", "params.yaml"]
                ),
            ),
            DeclareLaunchArgument(
                "state_machine_params_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("state_machine"), "config", "params.yaml"]
                ),
            ),
            DeclareLaunchArgument(
                "mppi_params_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("mppi_controller"), "config", "params.yaml"]
                ),
            ),
            DeclareLaunchArgument(
                "race_line_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("roboracer_maps"),
                        "maps",
                        "racetrack_1_5x",
                        "raceline.csv",
                    ]
                ),
            ),
            costmap,
            tracker,
            state_machine,
            controller,
        ]
    )
