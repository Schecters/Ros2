import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, IncludeLaunchDescription
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # --- Path ของ Package และไฟล์ ---
    pkg_name = 'gocargo'
    xacro_file_name = 'AllPart_ROS_1.xacro'
    #world_file_name = 'roomgocargo.world.xml'

    pkg_share = get_package_share_directory(pkg_name)
    xacro_path = os.path.join(pkg_share, 'urdf', xacro_file_name)
    controllers_yaml_path = os.path.join(pkg_share, 'config', 'controllers.yaml')
    #world_path = os.path.join(pkg_share, 'worlds', world_file_name)

    # --- สร้าง robot_description จาก xacro ---
    robot_description_config = xacro.process_file(xacro_path).toxml()
    robot_description = {
        'robot_description': robot_description_config,
        'use_sim_time': True
    }

    # --- Gazebo ---
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        #launch_arguments={'world': world_path}.items()
    )

    # --- Robot State Publisher ---
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # --- Spawn Entity ---
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'security_robot'],
        output='screen'
    )

    # --- Launch Description ---
    return LaunchDescription([
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
        #laser_filter_node,
    ])
