"""One-command bringup for Cartographer MAPPING with manual gamepad control.

Start everything needed to drive the vehicle by hand while building a new
map.  The VESC command path is:

  gamepad -> manual_control_node -> /manual/motor, /manual/servo
                                 -> manual_mux -> /commands/* -> VESC

A read-only Foxglove WebSocket bridge is started by default so you can
watch the map being built live at ws://<ip>:8765.

No autonomous or safety nodes are launched.

When you are finished driving, save the map BEFORE stopping this launch:

  ros2 service call /finish_trajectory \\
    cartographer_ros_msgs/srv/FinishTrajectory "{trajectory_id: 0}"

  ros2 service call /write_state \\
    cartographer_ros_msgs/srv/WriteState \\
    "{filename: '/home/seeed/f1tenth_ws/map/racetrack.pbstream',
      include_unfinished_submaps: true}"

Optionally export a .pgm/.yaml pair for other consumers:

  ros2 run nav2_map_server map_saver_cli -f /home/seeed/f1tenth_ws/map/racetrack
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _preflight(context, *args, **kwargs):
    """Verify that the gamepad device and VESC serial port exist."""
    del args, kwargs
    errors = []

    manual_config = LaunchConfiguration("manual_config").perform(context)
    gamepad_device = LaunchConfiguration("gamepad_device").perform(context)
    if not os.path.exists(gamepad_device):
        errors.append(f"gamepad device does not exist: {gamepad_device}")

    vesc_config = LaunchConfiguration("vesc_config").perform(context)
    import yaml
    with open(vesc_config, "r") as f:
        doc = yaml.safe_load(f) or {}
    port = doc.get("vesc_driver_node", {}).get("ros__parameters", {}).get("port")
    if not port or not os.path.exists(port):
        errors.append(f"VESC device does not exist: {port}")

    lidar_config = LaunchConfiguration("lidar_config").perform(context)
    if not os.path.isfile(lidar_config):
        errors.append(f"LiDAR config does not exist: {lidar_config}")

    if errors:
        raise RuntimeError(
            "mapping preflight failed:\n  - " + "\n  - ".join(errors)
        )
    return [LogInfo(msg="[mapping_manual] preflight passed")]


def generate_launch_description():
    pkg_path = get_package_share_directory("robo_cartographer")
    config_dir = os.path.join(pkg_path, "config")

    # ---- launch arguments ---------------------------------------------------
    lidar_config = LaunchConfiguration("lidar_config")
    vesc_config = LaunchConfiguration("vesc_config")
    imu_config = LaunchConfiguration("imu_config")
    manual_config = LaunchConfiguration("manual_config")
    gamepad_device = LaunchConfiguration("gamepad_device")
    launch_foxglove = LaunchConfiguration("launch_foxglove")
    foxglove_address = LaunchConfiguration("foxglove_address")
    foxglove_port = LaunchConfiguration("foxglove_port")

    declarations = [
        DeclareLaunchArgument(
            "lidar_config",
            default_value="/home/seeed/f1tenth_ws/configs/urg.yaml",
            description="URG LiDAR parameter file",
        ),
        DeclareLaunchArgument(
            "vesc_config",
            default_value="/home/seeed/f1tenth_ws/configs/vesc.yaml",
            description="VESC driver and Ackermann converter parameter file",
        ),
        DeclareLaunchArgument(
            "imu_config",
            default_value=PathJoinSubstitution(
                [FindPackageShare("yesense_std_ros2"), "config", "yesense_config.yaml"]
            ),
            description="Yesense IMU parameter file",
        ),
        DeclareLaunchArgument(
            "manual_config",
            default_value=PathJoinSubstitution(
                [FindPackageShare("manual_ctrl"), "config", "manual_ctrl.yaml"]
            ),
            description="Manual control parameter file",
        ),
        DeclareLaunchArgument(
            "gamepad_device",
            default_value="/dev/input/event1",
            description="evdev gamepad device node",
        ),
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
    ]

    # ---- hardware drivers ---------------------------------------------------
    lidar = Node(
        package="urg_node",
        executable="urg_node_driver",
        name="urg_node",
        output="screen",
        parameters=[lidar_config],
        remappings=[("scan", "/scan")],
    )

    # Launched directly because the vendor launch file cannot accept a config
    # override and contains a tuple-valued parameters bug.
    imu = Node(
        package="yesense_std_ros2",
        executable="yesense_node_publisher",
        name="yesense_pub",
        output="screen",
        parameters=[imu_config],
    )

    vesc = Node(
        package="vesc_driver",
        executable="vesc_driver_node",
        name="vesc_driver_node",
        output="screen",
        parameters=[vesc_config],
    )

    # The Ackermann-to-VESC converter is started so the topic topology matches
    # the full bringup, but it is idle during manual mapping because nothing
    # publishes to /ackermann_cmd.
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
    )

    # ---- manual control -----------------------------------------------------
    manual_mux = Node(
        package="manual_ctrl",
        executable="manual_mux_node",
        name="manual_mux",
        output="screen",
        parameters=[manual_config],
    )

    manual = Node(
        package="manual_ctrl",
        executable="manual_control_node",
        name="manual_ctrl",
        output="screen",
        parameters=[manual_config, {"gamepad_device": gamepad_device}],
    )

    # ---- Foxglove bridge (read-only) ----------------------------------------
    foxglove = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge",
        output="screen",
        parameters=[
            {
                "address": foxglove_address,
                "port": ParameterValue(foxglove_port, value_type=int),
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

    # ---- Cartographer mapping -----------------------------------------------
    cartographer_node = Node(
        package="cartographer_ros",
        executable="cartographer_node",
        name="cartographer_node",
        output="screen",
        parameters=[{"use_sim_time": False}],
        arguments=[
            "-configuration_directory", config_dir,
            "-configuration_basename", "my_car_mapping.lua",
        ],
        remappings=[
            ("scan", "/scan"),
            ("imu", "/imu/data_raw"),
            ("odom", "/wheel/odometry"),
        ],
    )

    occupancy_grid_node = Node(
        package="cartographer_ros",
        executable="cartographer_occupancy_grid_node",
        name="cartographer_occupancy_grid_node",
        output="screen",
        arguments=["-resolution", "0.05"],
    )

    # ---- static transforms --------------------------------------------------
    laser_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="laser_tf",
        arguments=["0.25", "0", "0", "0", "0", "0", "base_link", "laser"],
    )

    imu_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="imu_tf",
        arguments=["0.25", "0", "0", "0", "0", "0", "base_link", "gyro_link"],
    )

    return LaunchDescription(
        declarations
        + [
            OpaqueFunction(function=_preflight),
            LogInfo(msg=["[mapping_manual] hardware + manual control + mapping"]),
            lidar,
            imu,
            vesc,
            command_converter,
            manual_mux,
            manual,
            foxglove,
            laser_tf,
            imu_tf,
            cartographer_node,
            occupancy_grid_node,
        ]
    )
