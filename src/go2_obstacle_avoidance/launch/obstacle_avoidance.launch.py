from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('go2_obstacle_avoidance'),
        'config',
        'obstacle_avoidance.yaml'
    )

    return LaunchDescription([
        Node(
            package='go2_obstacle_avoidance',
            executable='obstacle_avoidance_node',
            name='go2_obstacle_avoidance',
            output='screen',
            parameters=[config]
        )
    ])
