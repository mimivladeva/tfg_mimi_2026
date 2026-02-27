import os

from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue



def generate_launch_description():

    pkg_share = get_package_share_directory('aidguide_04_provide_map')

    map_file = os.path.join(pkg_share, 'map', 'aidguide_04_map.yaml')
    rviz_config = os.path.join(pkg_share, 'rviz', 'aidguide_config2.rviz')
    urdf_file = os.path.join(pkg_share, 'urdf', 'turtlebot3_burger.urdf')

    # Leer URDF correctamente
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()
   # rviz_config = os.path.join(pkg_share, 'rviz', 'map_fixed.rviz')

    return LaunchDescription([

        # 🔥 Forzar renderizado por software (VM fix)
        SetEnvironmentVariable(
            name='LIBGL_ALWAYS_SOFTWARE',
            value='1'
        ),
            # Robot State Publisher (AQUÍ VA ANTES DE RVIZ)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[
                {'use_sim_time': True},
                {'robot_description': robot_desc}
            ],
            output='screen'
        ),

        # Map Server
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[
                {'use_sim_time': True},
                {'yaml_filename': map_file}
            ]
        ),

        # Static TF map -> odom

        # Lifecycle Manager
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[
                {'use_sim_time': True},
                {'autostart': True},
                {'node_names': ['map_server']}
            ]
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': True}]
        ),


    ])
