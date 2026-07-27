from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from lifecycle_msgs.msg import Transition
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    package_config = LaunchConfiguration("params_file")
    race_line = LaunchConfiguration("race_line_file")
    backend = LaunchConfiguration("backend")
    odom_topic = LaunchConfiguration("odom_topic")
    costmap_topic = LaunchConfiguration("costmap_topic")
    command_topic = LaunchConfiguration("command_topic")
    max_speed = LaunchConfiguration("max_speed")
    speed_weight = LaunchConfiguration("speed_weight")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    require_race_state = LaunchConfiguration("require_race_state")

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
                "backend": backend,
                "odom_topic": odom_topic,
                "costmap_topic": costmap_topic,
                "command_topic": command_topic,
                "vehicle.max_speed": max_speed,
                "weights.speed": ParameterValue(speed_weight, value_type=float),
                "use_sim_time": use_sim_time,
                "require_race_state": require_race_state,
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
    # Emitting CONFIGURE at description time races the node's lifecycle
    # services and silently drops the transition on a slow start. Wait for the
    # process, then give its services a moment to come up.
    configure = RegisterEventHandler(
        OnProcessStart(
            target_action=controller,
            on_start=[
                TimerAction(
                    period=2.0,
                    actions=[
                        EmitEvent(
                            event=ChangeState(
                                lifecycle_node_matcher=matches_action(controller),
                                transition_id=Transition.TRANSITION_CONFIGURE,
                            )
                        )
                    ],
                )
            ],
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
        DeclareLaunchArgument("backend", default_value="cuda"),
        DeclareLaunchArgument("odom_topic", default_value="/ego_racecar/odom"),
        DeclareLaunchArgument(
            "costmap_topic", default_value="/perception/local_costmap"
        ),
        DeclareLaunchArgument("command_topic", default_value="/control/mppi_cmd"),
        DeclareLaunchArgument("max_speed", default_value="2.0"),
        DeclareLaunchArgument("speed_weight", default_value="50.0"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("autostart", default_value="true"),
        DeclareLaunchArgument("require_race_state", default_value="false"),
        controller,
        activate_when_inactive,
        configure,
    ])
