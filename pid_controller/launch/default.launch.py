from ament_index_python.packages import get_package_share_path
from launch_ros.actions import Node, PushRosNamespace

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
)
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    launch_description = LaunchDescription()

    package_path = get_package_share_path('pid_controller')
    param_file_path = str(package_path / 'config/controller_params.yaml')
    # expose the parameter to the launch command line
    param_file_arg = DeclareLaunchArgument('controller_config_file',
                                           default_value=param_file_path)
    launch_description.add_action(param_file_arg)

    node = Node(executable='main.py',
                package='pid_controller',
                parameters=[
                    LaunchConfiguration('controller_config_file'),
                ])
    group = GroupAction([
        node,
    ])
    launch_description.add_action(group)

    return launch_description
