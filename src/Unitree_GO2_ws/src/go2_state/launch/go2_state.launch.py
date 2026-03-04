from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    go2_drive_pkg = get_package_share_directory('go2_drive_py')
    go2_drive_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(go2_drive_pkg, 'launch', 'drive.launch.py')
        )
    )


    return LaunchDescription([
        go2_drive_launch,
        Node(
            package='go2_state',
            executable='go2_state',
            name='go2_state',
            output='screen',
            parameters=[os.path.join(get_package_share_directory('go2_state'), 'params', 'go2_state.yaml')]
        ),
    ])

