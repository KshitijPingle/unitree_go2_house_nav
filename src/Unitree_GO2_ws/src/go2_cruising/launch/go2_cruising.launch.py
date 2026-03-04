
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
import os
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    go2_drive_pkg = get_package_share_directory('go2_drive_py')

    go2_drive_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(go2_drive_pkg, 'launch', 'drive.launch.py')
        )
    )

    cru_service_node = Node(
        package='go2_cruising',
        executable='go2_cruising_service',
        name='cru_service_node',
        output='screen',
        parameters=[os.path.join(get_package_share_directory('go2_cruising'), 'params', 'go2_cruising_service.yaml')]
    )

    return LaunchDescription([
        go2_drive_launch,
        cru_service_node,
    ])