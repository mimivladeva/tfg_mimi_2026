import os

import launch.actions
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    nav2_yaml = os.path.join(get_package_share_directory('my_nav2_system'), 'param', 'burger.yaml')
    map_file = os.path.join(get_package_share_directory('my_nav2_system'), 'map', 'aidguide_04_map.yaml')
    rviz_config_dir = os.path.join(get_package_share_directory('my_nav2_system'), 'rviz', 'tb3_navigation2.rviz')

    # 🔴 AÑADE ESTO
    urdf_file = os.path.join(
        get_package_share_directory('turtlebot3_description'),
        'urdf',
        'turtlebot3_burger.urdf'
    )

    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    return LaunchDescription([

        # ✔ ROBOT STATE PUBLISHER
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {'use_sim_time': False},
                {'robot_description': robot_desc}
            ]
        ),

        # ✔ MAP SERVER
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[
                nav2_yaml,
                {'yaml_filename': map_file}
            ]
        ),

        # ✔ AMCL
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[nav2_yaml]
        ),

        # ✔ LIFECYCLE
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[
                {'use_sim_time': False},
                {'autostart': True},
                {'node_names': ['map_server', 'amcl']}
            ]
        ),
        Node(
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    output='screen',
    arguments=['-d', rviz_config_dir]
),
        
    ])