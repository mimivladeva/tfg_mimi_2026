import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_share = get_package_share_directory('aidguide_04_provide_map')

    params_yaml = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    rviz_config = os.path.join(pkg_share, 'rviz', 'aidguide_config2.rviz')

    lifecycle_nodes = [
        'planner_server',
        'controller_server',
        'bt_navigator',
        'waypoint_follower'
    ]

    return LaunchDescription([

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[params_yaml, {'use_sim_time': True}],
        ),

        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[params_yaml, {'use_sim_time': True}],
        ),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[params_yaml, {'use_sim_time': True}],
        ),

        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[params_yaml, {'use_sim_time': True}],
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[
                {'use_sim_time': True},
                {'autostart': True},
                {'node_names': lifecycle_nodes}
            ],
        ),

      Node(
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    output='screen',
    arguments=['-d', rviz_config],
    parameters=[{'use_sim_time': True}],
    additional_env={'LIBGL_ALWAYS_SOFTWARE': '1'},
),
    ])