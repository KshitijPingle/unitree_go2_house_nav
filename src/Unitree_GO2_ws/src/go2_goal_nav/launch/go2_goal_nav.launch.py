
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

    nav_service_node = Node(
        package='go2_goal_nav',
        executable='goal_subscriber',
        name='goal_subscriber_node',
        output='screen',
        parameters=[os.path.join(get_package_share_directory('go2_goal_nav'), 'params', 'nav_params.yaml')]
    )

    return LaunchDescription([
        go2_drive_launch,
        nav_service_node,
    ])