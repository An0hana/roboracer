"""One-command bringup for the complete RoboRacer hardware stack.

Startup is staged so that control is the last part to become active:

  hardware -> wheel odometry -> localization -> fused state
           -> perception/behavior -> controller/safety

The VESC command path remains exclusive:

  MPPI -> safety -> ackermann_to_vesc -> /autonomous/*
       -> manual_mux -> /commands/* -> VESC
"""

import os

import yaml

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
    Shutdown,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def include_python(package, launch_file, arguments=None, condition=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package), "launch", launch_file])
        ),
        launch_arguments=(arguments or {}).items(),
        condition=condition,
    )


def shutdown_if_process_exits(node, enabled, label):
    return RegisterEventHandler(
        OnProcessExit(
            target_action=node,
            on_exit=[Shutdown(reason=f"critical process exited: {label}")],
        ),
        condition=IfCondition(enabled),
    )


def _is_true(context, value):
    return value.perform(context).strip().lower() in ("1", "true", "yes", "on")


def _yaml_parameter(filename, node_name, parameter_name):
    with open(filename, "r", encoding="utf-8") as stream:
        document = yaml.safe_load(stream) or {}
    return document.get(node_name, {}).get("ros__parameters", {}).get(parameter_name)


def preflight(context, *args, **kwargs):
    del args, kwargs
    required_files = {
        "race line": LaunchConfiguration("race_line_file").perform(context),
    }
    if _is_true(context, LaunchConfiguration("launch_lidar")):
        required_files["LiDAR config"] = LaunchConfiguration("lidar_config").perform(
            context
        )
    if _is_true(context, LaunchConfiguration("launch_localization")):
        required_files.update(
            {
                "AMCL map": LaunchConfiguration("map").perform(context),
                "AMCL config": LaunchConfiguration("amcl_config").perform(context),
                "pose odometry config": LaunchConfiguration(
                    "pose_odom_config"
                ).perform(context),
            }
        )
    if _is_true(context, LaunchConfiguration("launch_vesc")):
        required_files.update(
            {
                "VESC config": LaunchConfiguration("vesc_config").perform(context),
                "wheel odometry config": LaunchConfiguration(
                    "wheel_odom_config"
                ).perform(context),
            }
        )
    if _is_true(context, LaunchConfiguration("launch_imu")):
        required_files["IMU config"] = LaunchConfiguration("imu_config").perform(
            context
        )
    if _is_true(context, LaunchConfiguration("launch_manual")):
        required_files["manual config"] = LaunchConfiguration("manual_config").perform(
            context
        )

    errors = [
        f"{label} does not exist: {path}"
        for label, path in required_files.items()
        if not os.path.isfile(path)
    ]
    if not errors and _is_true(context, LaunchConfiguration("launch_vesc")):
        config = LaunchConfiguration("vesc_config").perform(context)
        port = _yaml_parameter(config, "vesc_driver_node", "port")
        if not port or not os.path.exists(port):
            errors.append(f"VESC device does not exist: {port}")
    if not errors and _is_true(context, LaunchConfiguration("launch_imu")):
        config = LaunchConfiguration("imu_config").perform(context)
        port = _yaml_parameter(config, "yesense_pub", "serial_port")
        if not port or not os.path.exists(port):
            errors.append(f"IMU device does not exist: {port}")
    if not errors and _is_true(context, LaunchConfiguration("launch_manual")):
        config = LaunchConfiguration("manual_config").perform(context)
        device = _yaml_parameter(config, "manual_ctrl", "gamepad_device")
        if not device or not os.path.exists(device):
            errors.append(f"gamepad device does not exist: {device}")

    if errors:
        raise RuntimeError("vehicle bringup preflight failed:\n  - " + "\n  - ".join(errors))
    return [LogInfo(msg="[vehicle_bringup] preflight passed")]


def generate_launch_description():
    race_line_file = LaunchConfiguration("race_line_file")
    map_yaml = LaunchConfiguration("map")
    amcl_config = LaunchConfiguration("amcl_config")
    vesc_config = LaunchConfiguration("vesc_config")
    lidar_config = LaunchConfiguration("lidar_config")
    imu_config = LaunchConfiguration("imu_config")
    wheel_odom_config = LaunchConfiguration("wheel_odom_config")
    pose_odom_config = LaunchConfiguration("pose_odom_config")
    manual_config = LaunchConfiguration("manual_config")

    scan_topic = LaunchConfiguration("scan_topic")
    odom_topic = LaunchConfiguration("odom_topic")
    base_frame = LaunchConfiguration("base_frame")
    max_speed = LaunchConfiguration("max_speed")
    min_steering = LaunchConfiguration("min_steering")
    max_steering = LaunchConfiguration("max_steering")

    launch_lidar = LaunchConfiguration("launch_lidar")
    launch_imu = LaunchConfiguration("launch_imu")
    launch_vesc = LaunchConfiguration("launch_vesc")
    launch_manual = LaunchConfiguration("launch_manual")
    launch_localization = LaunchConfiguration("launch_localization")
    launch_control = LaunchConfiguration("launch_control")
    launch_foxglove = LaunchConfiguration("launch_foxglove")
    foxglove_address = LaunchConfiguration("foxglove_address")
    foxglove_port = LaunchConfiguration("foxglove_port")
    enable_behavior_stack = LaunchConfiguration("enable_behavior_stack")

    lidar = Node(
        package="urg_node",
        executable="urg_node_driver",
        name="urg_node",
        output="screen",
        parameters=[lidar_config],
        remappings=[("scan", scan_topic)],
        condition=IfCondition(launch_lidar),
    )
    # Launch directly because the vendor launch file cannot accept a config
    # override and contains a tuple-valued parameters bug.
    imu = Node(
        package="yesense_std_ros2",
        executable="yesense_node_publisher",
        name="yesense_pub",
        output="screen",
        parameters=[imu_config],
        condition=IfCondition(launch_imu),
    )
    vesc = Node(
        package="vesc_driver",
        executable="vesc_driver_node",
        name="vesc_driver_node",
        output="screen",
        parameters=[vesc_config],
        condition=IfCondition(launch_vesc),
    )
    command_converter = Node(
        package="vesc_ackermann",
        executable="ackermann_to_vesc_node",
        name="ackermann_to_vesc_node",
        output="screen",
        parameters=[vesc_config],
        remappings=[
            ("commands/motor/speed", "/autonomous/motor"),
            ("commands/servo/position", "/autonomous/servo"),
        ],
        condition=IfCondition(launch_vesc),
    )
    manual_mux = Node(
        package="manual_ctrl",
        executable="manual_mux_node",
        name="manual_mux",
        output="screen",
        parameters=[manual_config],
        condition=IfCondition(launch_manual),
    )
    manual = Node(
        package="manual_ctrl",
        executable="manual_control_node",
        name="manual_ctrl",
        output="screen",
        parameters=[manual_config],
        condition=IfCondition(launch_manual),
    )
    foxglove = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge",
        output="screen",
        parameters=[
            {
                "address": foxglove_address,
                "port": ParameterValue(foxglove_port, value_type=int),
                # Vehicle telemetry is deliberately read-only. Foxglove may
                # inspect ROS data but cannot publish commands or call services.
                "capabilities": ["connectionGraph", "assets"],
                "client_topic_whitelist": ["^$"],
                "service_whitelist": ["^$"],
                "param_whitelist": ["^$"],
                "topic_whitelist": [".*"],
                "include_hidden": False,
                "send_buffer_limit": 10000000,
                "num_threads": 2,
                "use_sim_time": False,
            }
        ],
        condition=IfCondition(launch_foxglove),
    )

    wheel_odom = include_python(
        "ackermann_wheel_odom",
        "odom.launch.py",
        {"config_file": wheel_odom_config},
        IfCondition(launch_vesc),
    )
    localization = include_python(
        "robo_cartographer",
        "localization.launch.py",
        {
            "map": map_yaml,
            "params_file": amcl_config,
            "pose_odom_params": pose_odom_config,
        },
        IfCondition(launch_localization),
    )

    costmap = include_python(
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
    tracker = include_python(
        "opponent_tracker",
        "opponent_tracker.launch.py",
        {
            "params_file": PathJoinSubstitution(
                [FindPackageShare("opponent_tracker"), "config", "params.yaml"]
            ),
            "odom_topic": odom_topic,
            "use_known_opponent_size": "false",
            "use_sim_time": "false",
        },
        IfCondition(enable_behavior_stack),
    )
    state_machine = include_python(
        "state_machine",
        "state_machine.launch.py",
        {
            "params_file": PathJoinSubstitution(
                [FindPackageShare("state_machine"), "config", "params.yaml"]
            ),
            "odom_topic": odom_topic,
            "race_line_file": race_line_file,
            "use_sim_time": "false",
        },
        IfCondition(enable_behavior_stack),
    )
    mppi = include_python(
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
            "require_race_state": enable_behavior_stack,
        },
    )
    safety = include_python(
        "safety_controller",
        "safety_controller.launch.py",
        {
            "params_file": PathJoinSubstitution(
                [FindPackageShare("safety_controller"), "config", "params.yaml"]
            ),
            "scan_topic": scan_topic,
            "odom_topic": odom_topic,
            "mppi_cmd_topic": "/control/mppi_cmd",
            "race_state_topic": "/state_machine/state",
            "output_topic": "/ackermann_cmd",
            "max_speed": max_speed,
            "min_steering": min_steering,
            "max_steering": max_steering,
            "use_sim_time": "false",
        },
    )

    declarations = [
        DeclareLaunchArgument(
            "race_line_file",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("roboracer_maps"),
                    "maps",
                    "racetrack_v1",
                    "raceline.csv",
                ]
            ),
            description="Map-aligned race-line CSV",
        ),
        DeclareLaunchArgument(
            "map",
            default_value="/home/seeed/f1tenth_ws/map/racetrack.yaml",
            description="AMCL occupancy-grid map YAML",
        ),
        DeclareLaunchArgument(
            "amcl_config",
            default_value=PathJoinSubstitution(
                [FindPackageShare("robo_cartographer"), "config", "amcl.yaml"]
            ),
            description="AMCL and map_server parameter file",
        ),
        DeclareLaunchArgument(
            "vesc_config",
            default_value="/home/seeed/f1tenth_ws/configs/vesc.yaml",
        ),
        DeclareLaunchArgument(
            "lidar_config",
            default_value="/home/seeed/f1tenth_ws/configs/urg.yaml",
        ),
        DeclareLaunchArgument(
            "imu_config",
            default_value=PathJoinSubstitution(
                [FindPackageShare("yesense_std_ros2"), "config", "yesense_config.yaml"]
            ),
        ),
        DeclareLaunchArgument(
            "wheel_odom_config",
            default_value=PathJoinSubstitution(
                [FindPackageShare("ackermann_wheel_odom"), "config", "odom.yaml"]
            ),
        ),
        DeclareLaunchArgument(
            "pose_odom_config",
            default_value=PathJoinSubstitution(
                [FindPackageShare("pose_odom"), "config", "pose_odom.yaml"]
            ),
        ),
        DeclareLaunchArgument(
            "manual_config",
            default_value=PathJoinSubstitution(
                [FindPackageShare("manual_ctrl"), "config", "manual_ctrl.yaml"]
            ),
        ),
        DeclareLaunchArgument("scan_topic", default_value="/scan"),
        DeclareLaunchArgument("odom_topic", default_value="/state_estimation/odom"),
        DeclareLaunchArgument("base_frame", default_value="base_link"),
        DeclareLaunchArgument(
            "max_speed",
            default_value="1.5",
            description="Hard speed limit shared by MPPI and safety controller [m/s]",
        ),
        DeclareLaunchArgument("min_steering", default_value="-0.40"),
        DeclareLaunchArgument("max_steering", default_value="0.38"),
        DeclareLaunchArgument("launch_lidar", default_value="true"),
        DeclareLaunchArgument("launch_imu", default_value="true"),
        DeclareLaunchArgument("launch_vesc", default_value="true"),
        DeclareLaunchArgument("launch_manual", default_value="true"),
        DeclareLaunchArgument("launch_localization", default_value="true"),
        DeclareLaunchArgument(
            "launch_foxglove",
            default_value="true",
            description="Start the read-only Foxglove WebSocket bridge",
        ),
        DeclareLaunchArgument(
            "foxglove_address",
            default_value="0.0.0.0",
            description="Foxglove bridge listen address",
        ),
        DeclareLaunchArgument(
            "foxglove_port",
            default_value="8765",
            description="Foxglove bridge WebSocket port",
        ),
        DeclareLaunchArgument(
            "enable_behavior_stack",
            default_value="true",
            description="Start opponent tracking/state machine and require RaceState",
        ),
        DeclareLaunchArgument(
            "launch_control",
            default_value="true",
            description="Start MPPI and the safety arbiter",
        ),
    ]

    return LaunchDescription(
        declarations
        + [
            SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1"),
            LogInfo(
                msg=[
                    "[vehicle_bringup] complete hardware stack; max_speed=",
                    max_speed,
                    " m/s, behavior_stack=",
                    enable_behavior_stack,
                ]
            ),
            OpaqueFunction(function=preflight),
            # Register critical-process interlocks before starting hardware.
            shutdown_if_process_exits(lidar, launch_lidar, "LiDAR"),
            shutdown_if_process_exits(imu, launch_imu, "IMU"),
            shutdown_if_process_exits(vesc, launch_vesc, "VESC"),
            shutdown_if_process_exits(
                command_converter, launch_vesc, "Ackermann command converter"
            ),
            shutdown_if_process_exits(manual_mux, launch_manual, "manual mux"),
            shutdown_if_process_exits(manual, launch_manual, "manual controller"),
            # Stage 0: hardware and the exclusive final command mux.
            lidar,
            imu,
            vesc,
            command_converter,
            manual_mux,
            manual,
            foxglove,
            # Stage 1: raw wheel odometry diagnostics.
            TimerAction(period=1.0, actions=[wheel_odom]),
            # Stage 2: AMCL, map server, controller-facing pose odometry and
            # required sensor transforms. amcl_localization.launch.py owns the
            # only pose_odom instance so map->odom and odom->base_link each
            # have exactly one publisher.
            TimerAction(period=2.0, actions=[localization]),
            # Stage 3: obstacle costmap, opponent tracking and behavior.
            TimerAction(
                period=6.0,
                actions=[costmap, tracker, state_machine],
            ),
            # Stage 4: control is deliberately last.
            TimerAction(
                period=9.0,
                actions=[mppi, safety],
                condition=IfCondition(launch_control),
            ),
        ]
    )
