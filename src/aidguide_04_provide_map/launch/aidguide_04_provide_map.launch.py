import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_share = get_package_share_directory('aidguide_04_provide_map')

    map_yaml = os.path.join(pkg_share, 'map', 'aidguide_04_map.yaml')
    params_yaml = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    urdf_file = os.path.join(pkg_share, 'urdf', 'turtlebot3_burger.urdf')

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([

        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {'use_sim_time': True},
                {'robot_description': robot_desc}
            ]
        ),

        # Map server
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[
                {'use_sim_time': True},
                {'yaml_filename': map_yaml}
            ],
        ),

        # AMCL
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[params_yaml, {'use_sim_time': True}],
        ),

        # Lifecycle manager for localization
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[
                {'use_sim_time': True},
                {'autostart': True},
                {'node_names': ['map_server', 'amcl']}
            ],
        ),
    ])