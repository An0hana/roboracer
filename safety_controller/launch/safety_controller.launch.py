from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    scan_topic = LaunchConfiguration("scan_topic")
    odom_topic = LaunchConfiguration("odom_topic")
    mppi_cmd_topic = LaunchConfiguration("mppi_cmd_topic")
    output_topic = LaunchConfiguration("output_topic")
    max_speed = LaunchConfiguration("max_speed")
    min_steering = LaunchConfiguration("min_steering")
    max_steering = LaunchConfiguration("max_steering")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("safety_controller"), "config", "params.yaml"]
                ),
            ),
            DeclareLaunchArgument("scan_topic", default_value="/scan"),
            DeclareLaunchArgument(
                "odom_topic", default_value="/state_estimation/odom"
            ),
            DeclareLaunchArgument(
                "mppi_cmd_topic", default_value="/control/mppi_cmd"
            ),
            DeclareLaunchArgument("output_topic", default_value="/ackermann_cmd"),
            DeclareLaunchArgument("max_speed", default_value="2.0"),
            DeclareLaunchArgument("min_steering", default_value="-0.32"),
            DeclareLaunchArgument("max_steering", default_value="0.32"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            Node(
                package="safety_controller",
                executable="safety_controller_node",
                name="safety_controller",
                output="screen",
                parameters=[
                    params_file,
                    {
                        "scan_topic": scan_topic,
                        "odom_topic": odom_topic,
                        "mppi_cmd_topic": mppi_cmd_topic,
                        "output_topic": output_topic,
                        "max_command_speed": ParameterValue(
                            max_speed, value_type=float
                        ),
                        "min_command_steering": ParameterValue(
                            min_steering, value_type=float
                        ),
                        "max_command_steering": ParameterValue(
                            max_steering, value_type=float
                        ),
                        "use_sim_time": ParameterValue(
                            use_sim_time, value_type=bool
                        ),
                    },
                ],
            ),
        ]
    )
