"""Start the complete simulation control stack from config/simulation.yaml."""

from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def _launch_text(value):
    if isinstance(value, bool):
        return "true" if value else "false"
    return str(value)


def generate_launch_description():
    share = Path(get_package_share_directory("race_bringup"))
    config_path = share / "config" / "simulation.yaml"
    with config_path.open(encoding="utf-8") as source:
        document = yaml.safe_load(source)
    if not isinstance(document, dict) or not isinstance(document.get("race_bringup"), dict):
        raise RuntimeError(f"invalid simulation configuration: {config_path}")

    arguments = {
        name: _launch_text(value)
        for name, value in document["race_bringup"].items()
    }
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / "launch" / "race_stack.launch.py")),
            launch_arguments=arguments.items(),
        )
    ])
