from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    world_file = '/home/nine/ballzaros/worlds/roomgocargo.world.xml'
    map_file = '/home/nine/ballzaros/map/gocargo.yaml'
    xacro_file = '/home/nine/ballzaros/urdf/AllPart_ROS_1.xacro'

    # Launch Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource('/opt/ros/humble/share/gazebo_ros/launch/gazebo.launch.py'),
        launch_arguments={'world': world_file}.items()
    )

    # Load robot_description from XACRO
    robot_description_content = Command(['xacro ', xacro_file])
    robot_description_param = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description_param]
    )

    # Spawn robot after Gazebo loads (~2s delay)
    spawn_robot = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                name='spawn_gocargo',
                arguments=['-topic', 'robot_description', '-entity', 'gocargo_robot'],
                output='screen'
            )
        ]
    )

    # Launch map_server after robot spawns (~4s delay)
    map_server_node = TimerAction(
        period=4.0,
        actions=[
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                parameters=[{'yaml_filename': map_file}]
            )
        ]
    )

    # Launch RViz after map_server (~6s delay)
    rviz_node = TimerAction(
        period=6.0,
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

    # Teleop keyboard node after RViz (~8s delay)
    teleop_node = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='teleop_twist_keyboard',
                executable='teleop_twist_keyboard',
                name='teleop_twist_keyboard',
                output='screen',
                prefix='xterm -e'  # เปิด terminal สำหรับรับ keyboard input
            )
        ]
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher_node,
        spawn_robot,
        map_server_node,
        rviz_node,
        teleop_node
    ])
