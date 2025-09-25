from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import LifecycleNode, Node

def generate_launch_description():
    world_file = '/home/nine/ballzaros/worlds/roomgocargo.world.xml'
    map_file = '/home/nine/ballzaros/map/gocargo.yaml'

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource('/opt/ros/humble/share/gazebo_ros/launch/gazebo.launch.py'),
        launch_arguments={'world': world_file}.items()
    )

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
                output='screen',
                parameters=[{'use_sim_time': True, 'autostart': True,
                             'node_names': ['map_server']}]
            )
        ]
    )

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

    return LaunchDescription([
        gazebo,
        map_server_node,
        activate_map_server,
        rviz_node
    ])

