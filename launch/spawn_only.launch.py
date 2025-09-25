import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_name = 'gocargo'
    xacro_file_name = 'AllPart_ROS_1.xacro'

    pkg_share = get_package_share_directory(pkg_name)
    xacro_path = os.path.join(pkg_share, 'urdf', xacro_file_name)

    robot_description_config = xacro.process_file(xacro_path).toxml()
    robot_description = {
        'robot_description': robot_description_config,
        'use_sim_time': True
    }

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
    )

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'security_robot'],
        output='screen'
    )
    
    teleop_keyboard = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        prefix='xterm -e',
        arguments=[],
        )

    return LaunchDescription([
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
            teleop_keyboard,
    ])
