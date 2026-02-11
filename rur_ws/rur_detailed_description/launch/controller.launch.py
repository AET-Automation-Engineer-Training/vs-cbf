from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    controller_config = os.path.join(
        get_package_share_directory('rur_detailed_description'),
        'launch',
        'controller.yaml'
    )

    controller_names = [
        'joint_state_controller',
        'motor_right_position_controller',
        'motor_left_position_controller',
        'slider_position_controller',
        'camera_joint_position_controller'
    ]

    controller_spawners = [
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=[name],
            parameters=[controller_config],
            namespace='rur_detailed',
            output='screen'
        ) for name in controller_names
    ]

    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        remappings=[('/joint_states', '/rur_detailed/joint_states')],
    )

    return LaunchDescription(controller_spawners + [robot_state_pub])
