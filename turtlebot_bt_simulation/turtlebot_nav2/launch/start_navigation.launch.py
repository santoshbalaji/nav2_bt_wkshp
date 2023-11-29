import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    # sim time parameter from launch configuration
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    # argument to get map name to be used
    map_dir = LaunchConfiguration(
        'map',
        default=os.path.join(
            get_package_share_directory('turtlebot_nav2'),
            'map',
            'map_small_room.yaml'))

    # argument to get configuration file for nav components
    param_file_name = 'navigation.yaml'

    # nav param config path
    param_dir = LaunchConfiguration(
        'params_file',
        default=os.path.join(
            get_package_share_directory('turtlebot_nav2'),
            'config',
            param_file_name))

    # nav2 launch file path
    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')

    # rviz config path
    rviz_config_dir = os.path.join(
        get_package_share_directory('turtlebot_nav2'),
        'rviz',
        'view_robot.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=map_dir,
            description='Full path to map file to load'),

        DeclareLaunchArgument(
            'params_file',
            default_value=param_dir,
            description='Full path to param file to load'),

        # including all launch files needed for nav and passing the configuration file for all the nav components
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/bringup_launch.py']),
            launch_arguments={
                'map': map_dir,
                'use_sim_time': use_sim_time,
                'params_file': param_dir}.items(),
        ),

        # node of spawining rviz to visualize the robot
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),
        
        # node for converting laser scan to point cloud messages
        Node(
            package='pointcloud_to_laserscan',
            executable='laserscan_to_pointcloud_node',
            name='laserscan_to_pointcloud',
            remappings=[('scan_in', '/scan'),
                        ('cloud', '/cloud')],
            parameters=[{'target_frame': 'base_scan', 'transform_tolerance': 0.01}]
        )
    ])
