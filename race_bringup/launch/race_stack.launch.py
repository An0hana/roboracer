"""Bring up the single-car Shield-MPPI stack in simulation or on hardware."""

import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node


def _bool(context, name):
    return LaunchConfiguration(name).perform(context).strip().lower() in {
        "1",
        "true",
        "yes",
        "on",
    }


def _optional_bool(context, name, default):
    value = LaunchConfiguration(name).perform(context).strip()
    if not value:
        return default
    return value.lower() in {"1", "true", "yes", "on"}


def _setup(context):
    mode = LaunchConfiguration("mode").perform(context).strip().lower()
    if mode not in {"sim", "hardware"}:
        raise RuntimeError("mode must be 'sim' or 'hardware'")

    use_sim_time = _optional_bool(context, "use_sim_time", mode == "sim")
    use_localization = _bool(context, "use_localization")
    state_topic = LaunchConfiguration("state_topic").perform(context).strip()
    final_drive_topic = LaunchConfiguration("final_drive_topic").perform(context).strip()
    if not state_topic:
        state_topic = (
            "/state_estimation/odom"
            if use_localization or mode == "hardware"
            else "/ego_racecar/odom"
        )
    if not final_drive_topic:
        final_drive_topic = "/drive" if mode == "sim" else "/ackermann_cmd"

    scan_topic = LaunchConfiguration("scan_topic").perform(context)
    controller_mode = LaunchConfiguration("controller_mode").perform(context)
    raceline = LaunchConfiguration("raceline").perform(context)
    map_yaml = LaunchConfiguration("map").perform(context)
    map_topic = LaunchConfiguration("map_topic").perform(context)
    require_map = _bool(context, "require_map")
    use_ekf = _bool(context, "use_ekf")
    use_direct_vesc_adapter = _bool(context, "use_direct_vesc_adapter")
    max_speed_text = LaunchConfiguration("max_speed").perform(context).strip()
    max_speed = float(max_speed_text) if max_speed_text else (2.0 if mode == "sim" else 0.3)
    if not math.isfinite(max_speed) or max_speed <= 0.0:
        raise RuntimeError("max_speed must be positive")
    min_steering = float(LaunchConfiguration("min_steering").perform(context))
    max_steering = float(LaunchConfiguration("max_steering").perform(context))
    if (not math.isfinite(min_steering) or not math.isfinite(max_steering) or
            min_steering >= 0.0 or max_steering <= 0.0):
        raise RuntimeError("min_steering must be negative and max_steering must be positive")
    ftg_steering_limit = min(abs(min_steering), max_steering)

    mppi_share = Path(get_package_share_directory("race_mppi"))
    safety_share = Path(get_package_share_directory("race_safety"))
    ftg_share = Path(get_package_share_directory("ftg_controller"))
    bringup_share = Path(get_package_share_directory("race_bringup"))

    nodes = [
        LifecycleNode(
            package="race_mppi",
            executable="race_mppi_node",
            name="mppi_controller",
            namespace="",
            output="screen",
            parameters=[
                str(mppi_share / "config" / "params.yaml"),
                {
                    "use_sim_time": use_sim_time,
                    "odom_topic": state_topic,
                    "map_topic": map_topic,
                    "command_topic": "/control/mppi_cmd",
                    "race_line_file": raceline,
                    "require_map": require_map,
                    "vehicle.max_speed": max_speed,
                    "vehicle.min_steering": min_steering,
                    "vehicle.max_steering": max_steering,
                },
            ],
        ),
        Node(
            package="ftg_controller",
            executable="ftg_controller_node",
            name="ftg_node",
            output="screen",
            parameters=[
                str(ftg_share / "config" / "params.yaml"),
                {
                    "use_sim_time": use_sim_time,
                    "scan_topic": scan_topic,
                    "drive_topic": "/control/ftg_cmd",
                    "max_speed": max_speed,
                    "max_steering_angle": ftg_steering_limit,
                },
            ],
        ),
        Node(
            package="race_safety",
            executable="race_safety_node",
            name="race_safety",
            output="screen",
            parameters=[
                str(safety_share / "config" / "params.yaml"),
                {
                    "use_sim_time": use_sim_time,
                    "scan_topic": scan_topic,
                    "odom_topic": state_topic,
                    "mppi_cmd_topic": "/control/mppi_cmd",
                    "ftg_cmd_topic": "/control/ftg_cmd",
                    "output_topic": final_drive_topic,
                    "controller_mode": controller_mode,
                    "max_command_speed": max_speed,
                    "min_command_steering": min_steering,
                    "max_command_steering": max_steering,
                },
            ],
        ),
    ]

    # Lifecycle controllers are configured and activated before commands are
    # accepted by the safety node. Harmless for a regular Node implementation.
    nodes.append(
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="control_lifecycle_manager",
            output="screen",
            parameters=[
                {
                    "use_sim_time": use_sim_time,
                    "autostart": True,
                    "node_names": ["mppi_controller"],
                    "bond_timeout": 0.0,
                }
            ],
        )
    )

    if mode == "hardware":
        nodes.append(
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_to_laser_tf",
                arguments=[
                    "--x", "0.275", "--y", "0", "--z", "0",
                    "--roll", "0", "--pitch", "0", "--yaw", "0",
                    "--frame-id", "base_link", "--child-frame-id", "laser",
                ],
            )
        )

        if use_direct_vesc_adapter:
            nodes.append(
                Node(
                    package="race_safety",
                    executable="vesc_command_adapter_node",
                    name="vesc_command_adapter",
                    output="screen",
                    parameters=[
                        str(safety_share / "config" / "params.yaml"),
                        {"enabled": True, "input_topic": final_drive_topic},
                    ],
                )
            )

    if use_ekf:
        nodes.append(
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node",
                output="screen",
                parameters=[str(bringup_share / "config" / "localization.yaml")],
            )
        )

    if use_localization and not map_yaml:
        raise RuntimeError("map:=/absolute/path/map.yaml is required with localization")

    if map_yaml:
        managed_localization_nodes = ["map_server"]
        nodes.append(
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                output="screen",
                parameters=[
                    str(bringup_share / "config" / "localization.yaml"),
                    {"yaml_filename": map_yaml, "use_sim_time": use_sim_time},
                ],
                remappings=[("map", map_topic)],
            )
        )

        if use_localization:
            managed_localization_nodes.append("amcl")
            nodes.append(
                Node(
                    package="nav2_amcl",
                    executable="amcl",
                    name="amcl",
                    output="screen",
                    parameters=[
                        str(bringup_share / "config" / "localization.yaml"),
                        {"scan_topic": scan_topic, "use_sim_time": use_sim_time},
                    ],
                    remappings=[("map", map_topic)],
                )
            )

        nodes.append(
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="localization_lifecycle_manager",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "autostart": True,
                        "node_names": managed_localization_nodes,
                        "bond_timeout": 0.0,
                    }
                ],
            )
        )

    if use_localization:
        velocity_topic = LaunchConfiguration("velocity_odom_topic").perform(context).strip()
        if not velocity_topic:
            velocity_topic = "/ego_racecar/odom" if mode == "sim" else "/odometry/filtered"
        nodes.append(
            Node(
                package="race_bringup",
                executable="state_estimator_bridge",
                name="state_estimator_bridge",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "velocity_odom_topic": velocity_topic,
                        "output_topic": state_topic,
                        "map_frame": "map",
                        "base_frame": "base_link",
                        "publish_rate": 50.0,
                        "stale_timeout": 0.10,
                    }
                ],
            )
        )

    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("mode", default_value="sim"),
            DeclareLaunchArgument("controller_mode", default_value="mppi"),
            DeclareLaunchArgument("scan_topic", default_value="/scan"),
            DeclareLaunchArgument("state_topic", default_value=""),
            DeclareLaunchArgument("final_drive_topic", default_value=""),
            DeclareLaunchArgument("raceline", default_value=""),
            DeclareLaunchArgument("map", default_value=""),
            DeclareLaunchArgument("map_topic", default_value="/map"),
            DeclareLaunchArgument("require_map", default_value="true"),
            DeclareLaunchArgument("max_speed", default_value=""),
            DeclareLaunchArgument("min_steering", default_value="-0.20"),
            DeclareLaunchArgument("max_steering", default_value="0.20"),
            DeclareLaunchArgument("use_sim_time", default_value=""),
            DeclareLaunchArgument("use_localization", default_value="false"),
            DeclareLaunchArgument("use_ekf", default_value="false"),
            DeclareLaunchArgument("use_direct_vesc_adapter", default_value="false"),
            DeclareLaunchArgument("velocity_odom_topic", default_value=""),
            OpaqueFunction(function=_setup),
        ]
    )
