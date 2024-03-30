from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                get_package_share_directory('pid_controller') + '/launch/default.launch.py'
            )
        ),
        Node(
            package='target_processor',
            executable='main.py'
        )
    ])