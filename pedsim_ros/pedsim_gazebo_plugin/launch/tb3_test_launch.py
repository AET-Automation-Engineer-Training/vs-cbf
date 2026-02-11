import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Paths to required packages
    tb3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    gazebo_ros_dir = get_package_share_directory('gazebo_ros')
    pedsim_gazebo_dir = get_package_share_directory('pedsim_gazebo_plugin')
    pedsim_simulator_dir = get_package_share_directory('pedsim_simulator')

    # World and scene
    scene = 'office_env_large' # Change this to your desired scene
    world_file = os.path.join(pedsim_gazebo_dir, 'worlds', f'{scene}.world')
    pedsim_scene_file = os.path.join(pedsim_simulator_dir, 'scenarios', f'{scene}.xml')
    pedsim_config_file = os.path.join(pedsim_simulator_dir, 'config', 'params.yaml')

    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='-7')
    y_pose = LaunchConfiguration('y_pose', default='-5')

    # Gazebo Server and Client
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_file}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, 'launch', 'gzclient.launch.py')
        )
    )

    # TurtleBot3 launch files
    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_gazebo_dir, 'launch', 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_gazebo_dir, 'launch', 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose
        }.items()
    )

    # Pedsim agent spawner (spawn_pedsim_agents node)
    agent_spawner_cmd = Node(
        package='pedsim_gazebo_plugin',
        executable='spawn_pedsim_agents',
        name='spawn_pedsim_agents',
        output='screen'
    )

    # Pedsim simulator (delayed start to wait for Gazebo to be ready)
    pedsim_launch_cmd = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pedsim_simulator_dir, 'launch', 'simulator_launch.py')
                ),
                launch_arguments={
                    'scene_file': LaunchConfiguration('pedsim_scene_file'),
                    'config_file': LaunchConfiguration('pedsim_config_file'),
                    'namespace': '',
                    'use_rviz': 'True'
                }.items()
            )
        ]
    )

    # Declare extra launch arguments
    declare_scene_file_cmd = DeclareLaunchArgument(
        'pedsim_scene_file',
        default_value=pedsim_scene_file,
        description='Pedsim scene XML file'
    )

    declare_config_file_cmd = DeclareLaunchArgument(
        'pedsim_config_file',
        default_value=pedsim_config_file,
        description='Pedsim config YAML file'
    )

    # Launch description
    ld = LaunchDescription()

    # Add declared args
    ld.add_action(declare_scene_file_cmd)
    ld.add_action(declare_config_file_cmd)

    # Add actions
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_turtlebot_cmd)
    ld.add_action(agent_spawner_cmd)
    ld.add_action(pedsim_launch_cmd)

    return ld
