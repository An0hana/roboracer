from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.substitutions import FindPackageShare
from lifecycle_msgs.msg import Transition
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    package_config = LaunchConfiguration("params_file")
    race_line = LaunchConfiguration("race_line_file")
    odom_topic = LaunchConfiguration("odom_topic")
    map_topic = LaunchConfiguration("map_topic")
    command_topic = LaunchConfiguration("command_topic")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")

    controller = LifecycleNode(
        package="mppi_controller",
        executable="mppi_controller_node",
        name="mppi_controller",
        namespace="",
        output="screen",
        parameters=[
            package_config,
            {
                "race_line_file": race_line,
                "odom_topic": odom_topic,
                "map_topic": map_topic,
                "command_topic": command_topic,
                "use_sim_time": use_sim_time,
            },
        ],
    )

    activate_when_inactive = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=controller,
            goal_state="inactive",
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(controller),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                )
            ],
        ),
        condition=IfCondition(autostart),
    )
    configure = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(controller),
            transition_id=Transition.TRANSITION_CONFIGURE,
        ),
        condition=IfCondition(autostart),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("mppi_controller"), "config", "params.yaml"]
            ),
            description="MPPI parameter YAML",
        ),
        DeclareLaunchArgument(
            "race_line_file",
            description="Absolute path to strict race-line CSV",
        ),
        DeclareLaunchArgument("odom_topic", default_value="/state_estimation/odom"),
        DeclareLaunchArgument("map_topic", default_value="/map"),
        DeclareLaunchArgument("command_topic", default_value="/control/mppi_cmd"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("autostart", default_value="true"),
        controller,
        activate_when_inactive,
        configure,
    ])
