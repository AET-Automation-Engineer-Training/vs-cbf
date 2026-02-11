from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    robot_description_content = Command([
        PathJoinSubstitution([
            FindExecutable(name='xacro')
        ]),
        ' ',
        PathJoinSubstitution([
            FindPackageShare('rur_detailed_description'),
            'urdf',
            'rur_detailed.xacro'
        ])
    ])

    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str)
    }

    use_sim_time = {'use_sim_time': True}

    return LaunchDescription([
        # Joint state publisher
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[use_sim_time]
        ),

        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[robot_description, use_sim_time]
        ),

        # Spawn model in Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_urdf',
            arguments=['-topic', 'robot_description', '-entity', 'rur_detailed'],
            output='screen'
        ),

        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='odom_to_base_footprint_broadcaster',
        #     arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link']
        # ),

        # Include Gazebo empty world
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('gazebo_ros'),
                    'launch',
                    'gazebo.launch.py'
                ])
            ]),
            launch_arguments={
                'paused': 'false',
                'use_sim_time': 'true',
                'gui': 'true',
                'headless': 'false',
                'debug': 'false'
            }.items()
        ),
    ])
