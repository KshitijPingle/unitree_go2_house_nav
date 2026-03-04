from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory 
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
import os

def generate_launch_description():
    go2_desc_pak = get_package_share_directory('go2_description')
    go2_drive_pkg = get_package_share_directory('go2_drive_py')

    use_rviz = DeclareLaunchArgument(
        name="use_rviz",
        default_value="true",
        description="Whether to start RViz2")



    return LaunchDescription([
        use_rviz,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(go2_desc_pak, 'launch', 'display.launch.py')
            ),
            launch_arguments=[('use_joint_state_publisher', 'false')],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(go2_drive_pkg, 'rviz', 'display.rviz')],
            condition=IfCondition(LaunchConfiguration('use_rviz')),
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            output='screen',
            arguments=['--frame-id','radar','--child-frame-id','utlidar_lidar'],
        ),
        Node(
            package='go2_twist_bridge',
            executable='twist_bridge',
            name='twist_bridge',
            output='screen',
        ),
        Node(
            package='go2_drive_py',
            executable='driver',
            parameters=[os.path.join(go2_drive_pkg, 'params', 'drive.yaml')],
            ),
    ])