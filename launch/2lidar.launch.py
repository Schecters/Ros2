import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import LifecycleNode, Node
import xacro
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # --- Paths ---
    pkg_name = 'gocargo'
    pkg_share = get_package_share_directory(pkg_name)
    xacro_file = os.path.join(pkg_share, 'urdf', 'AllPart_ROS_1.xacro')
    controllers_yaml = os.path.join(pkg_share, 'config', 'controllers.yaml')
    world_file = '/home/nine/ballzaros/worlds/roomgocargo.world.xml'
    map_file = '/home/nine/ballzaros/map/gocargo.yaml'

    # --- Robot description from xacro ---
    robot_description_config = xacro.process_file(xacro_file).toxml()
    robot_description = {'robot_description': robot_description_config, 'use_sim_time': True}

    # --- Gazebo ---
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_file}.items()
    )

    # --- Map Server ---
    map_server_node = LifecycleNode(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        namespace='',
        output='screen',
        parameters=[{'yaml_filename': map_file}]
    )

    activate_map_server = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_map',
                namespace='',
                output='screen',
                parameters=[{'use_sim_time': True, 'autostart': True,
                             'node_names': ['map_server']}]
            )
        ]
    )

    # --- RViz ---
    rviz_node = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', '/opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz']
            )
        ]
    )

    # --- Robot State Publisher ---
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # --- Spawn Robot Entity in Gazebo ---
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'gocargo_robot'],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        map_server_node,
        activate_map_server,
        rviz_node,
        node_robot_state_publisher,
        spawn_entity
    ])
