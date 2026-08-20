"""Localization via AMCL instead of Cartographer.

  ros2 launch robo_cartographer amcl_localization.launch.py \\
      map:=/home/seeed/f1tenth_ws/map/racetrack.yaml

WHY THIS EXISTS
---------------
Cartographer's pose graph can re-optimize at any time, which is what produced
the ~0.6 m bistable flip. AMCL cannot do that: it has no graph to re-solve.
It is not jump-free either -- it also corrects via map->odom -- but the odom
frame underneath it is physically grounded dead reckoning rather than a
scan-match byproduct, so a bad map correction degrades to honest coasting.

FRAME OWNERSHIP -- the part that breaks if you get it wrong
-----------------------------------------------------------
  map  -> odom       AMCL
  odom -> base_link  pose_odom (dead reckoning, publish_odom_tf:=true)
  base_link -> laser static
  base_link -> gyro_link static

Cartographer MUST NOT be running. It publishes both map->odom and
odom->base_link (provide_odom_frame = true), and two publishers on one TF edge
produce erratic lookups with no error message.

pose_odom runs with map_pose_source:="tf" here, because AMCL has no pose topic
at controller rate. It composes map->odom with its own odom->base_link to
produce /state_estimation/odom for MPPI, with the same jump gate applied.

PREREQUISITE
------------
A .pgm/.yaml map pair, not a .pbstream:
  ros2 run nav2_map_server map_saver_cli -f /home/seeed/f1tenth_ws/map/racetrack
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_path = get_package_share_directory("robo_cartographer")
    default_params = os.path.join(pkg_path, "config", "amcl.yaml")

    declarations = [
        DeclareLaunchArgument(
            "map",
            default_value="/home/seeed/f1tenth_ws/map/racetrack.yaml",
            description="Occupancy grid .yaml produced by map_saver_cli.",
        ),
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params,
            description="AMCL and map_server parameter file.",
        ),
        DeclareLaunchArgument(
            "pose_odom_params",
            default_value=os.path.join(
                get_package_share_directory("pose_odom"),
                "config", "pose_odom.yaml"),
            description="pose_odom parameter file.",
        ),
    ]

    params_file = LaunchConfiguration("params_file")

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[params_file,
                    {"yaml_filename": LaunchConfiguration("map")}],
    )

    amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        parameters=[params_file],
        remappings=[("scan", "/scan")],
    )

    # nav2 nodes are lifecycle nodes and stay UNCONFIGURED until something
    # transitions them. Without this they launch, log nothing useful, and
    # never publish a map or a transform.
    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[{
            "use_sim_time": False,
            "autostart": True,
            "node_names": ["map_server", "amcl"],
        }],
    )

    # Owns odom -> base_link here, and applies the same jump gate to AMCL's
    # map correction that it applies to Cartographer's.
    pose_odom = Node(
        package="pose_odom",
        executable="pose_odom_node",
        name="pose_odom",
        output="screen",
        parameters=[
            LaunchConfiguration("pose_odom_params"),
            {
                "publish_odom_tf": True,
                "map_pose_source": "tf",
            },
        ],
    )

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
        declarations + [
            map_server,
            amcl,
            lifecycle_manager,
            pose_odom,
            laser_tf,
            imu_tf,
        ]
    )
