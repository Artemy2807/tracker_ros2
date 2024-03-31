from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
)
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_path, get_package_share_directory

def generate_launch_description():
    package_path = get_package_share_path('target_processor')
    param_file_path = str(package_path / 'config/processor_params.yaml')
    param_file_arg = DeclareLaunchArgument('processor_config_file',
                                           default_value=param_file_path)

    return LaunchDescription([
        param_file_arg,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                get_package_share_directory('pid_controller') + '/launch/default.launch.py'
            )
        ),
        GroupAction([
            Node(
                package='target_processor',
                executable='main.py',
                parameters=[
                    LaunchConfiguration('processor_config_file'),
                ]
            )
        ])
    ])