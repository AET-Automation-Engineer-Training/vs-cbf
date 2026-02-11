import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

import xacro


def generate_launch_description():

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    frame_prefix = LaunchConfiguration('frame_prefix')

    # Process xacro
    pkg_path = get_package_share_directory('rur_detailed_description')
    xacro_file = os.path.join(pkg_path, 'urdf', 'rur_detailed.xacro')
    robot_description_config = xacro.process_file(xacro_file, mappings={'frame_prefix': frame_prefix})
    robot_desc = robot_description_config.toxml()

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock (Gazebo, etc)'
        ),
        DeclareLaunchArgument(
            'frame_prefix',
            default_value='',
            description='Prefix for TF frames'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot_desc
            }],
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_to_base_footprint_broadcaster',
            arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link']
        ),
    ])
