"""A3 4x6 descend/close/lift/open client; launch the MoveIt demo separately."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = Path(get_package_share_directory("ur5e_a3_grid"))
    defaults = {
        "config_file": str(share / "config/a3_grid.yaml"),
        "moveit_config_package": "ur5e_ag95_moveit_config",
        "robot_name": "ur5e_on_box",
        "use_sim_time": "false",
        "execute": "false",
        "operation": "pick_cycle",
        "cell_id": "1",
        "keep_alive": "true",
        "coordinates_file": "",
    }
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()],
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / "launch/a3_grid.launch.py")),
            launch_arguments={name: LaunchConfiguration(name) for name in defaults}.items(),
        ),
    ])
