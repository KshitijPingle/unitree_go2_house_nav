from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command
from launch.actions import DeclareLaunchArgument
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition
import os

def generate_launch_description():
    # package_file path
    go2_description_path = get_package_share_directory('go2_description')
    use_joint_state_publisher = DeclareLaunchArgument(
        name="use_joint_state_publisher",
        default_value="true",
        description="Whether to start the joint state publisher")
    # Load robot description from xacro file
    model = DeclareLaunchArgument(
        name='urdf_path',
        default_value=os.path.join(go2_description_path, 'urdf', 'go2_description.urdf'),
        description='Absolute path to robot urdf file')
    robot_desc = ParameterValue(Command([
        'xacro ',
        # go2_description_path, 'urdf', 'go2_description.urdf',
        LaunchConfiguration('urdf_path')
        ]))
    # urdf file
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc
        }]
    )
    # joint_state_publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_joint_state_publisher'))
    )

    return LaunchDescription([
        model,
        use_joint_state_publisher,
        robot_state_publisher,
        joint_state_publisher,
    ])
