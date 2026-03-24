from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition
import os

def generate_launch_description():

    #获取各功能包
    go2_driver_pkg = get_package_share_directory("go2_driver")
    go2_core_pkg = get_package_share_directory("go2_core")
    go2_slam_pkg = get_package_share_directory("go2_slam")
    go2_perception_pkg = get_package_share_directory("go2_perception")
    go2_goal_pkg = get_package_share_directory("go2_goal_nav")			# Added by Mar J
    go2_description_pkg = get_package_share_directory("go2_description")	# Added by Kshitij
    
    # Path to your URDF file
    urdf_file = os.path.join(go2_description_pkg, 'urdf', 'go2_description.urdf')
    
    # Read the URDF file content
    with open(urdf_file, 'r') as infp:
        robot_description_config = infp.read()
    
    # Create the Robot State Publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_config,
            'use_sim_time': False
        }]
    )
    
    # 添加启动开关
    use_slamtoolbox = DeclareLaunchArgument(
        name="use_slamtoolbox",
        default_value="true"
    )

    # 里程计融合imu
    go2_robot_localization = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(go2_core_pkg, "launch", "go2_robot_localization.launch.py")
            )
        )

    # 启动驱动包
    go2_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(go2_driver_pkg, "launch", "driver.launch.py")
        )   
    )

    # 点云处理
    go2_pointcloud_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(go2_perception_pkg, "launch", "go2_pointcloud.launch.py")
            )
        )

    # slam-toolbox 配置
    go2_slamtoolbox_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(go2_slam_pkg, "launch", "go2_slamtoolbox.launch.py")
            ),
            condition=IfCondition(LaunchConfiguration('use_slamtoolbox'))
        )

    # 包含rviz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(go2_core_pkg, "rviz2", "display.rviz")],
        output='screen'
    )
    
    # Added by Mar J
    nav_service_node = Node(
        package='go2_goal_nav',
        executable='goal_subscriber',
        name='goal_subscriber_node',
        output='screen',
        parameters=[os.path.join(go2_goal_pkg, 'params', 'nav_params.yaml')]
    )

    return LaunchDescription([
        go2_driver_launch,
        robot_state_publisher_node,
        use_slamtoolbox,
        go2_robot_localization,
        go2_pointcloud_launch,
        go2_slamtoolbox_launch,
        rviz_node,
        nav_service_node
    ])
