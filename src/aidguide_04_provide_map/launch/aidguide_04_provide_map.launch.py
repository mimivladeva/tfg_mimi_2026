import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg = get_package_share_directory('aidguide_04_provide_map')

    params = os.path.join(pkg, 'config', 'nav2_param_robot_real.yaml')
    map_file = os.path.join(pkg, 'map', 'aidguide_04_map.yaml')
    urdf_file = os.path.join(pkg, 'urdf', 'turtlebot3_burger.urdf')
    rviz_config = os.path.join(pkg, 'rviz', 'tb3_navigation2.rviz')

    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    return LaunchDescription([

        # 🔵 ROBOT
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_desc}],
            output='screen'
        ),

        # 🔵 MAP + LOCALIZATION
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            parameters=[params, {'yaml_filename': map_file}],
            output='screen'
        ),

        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            parameters=[params],
            output='screen'
        ),

        # 🔵 NAVIGATION STACK
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            parameters=[params],
            output='screen'
        ),

        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            parameters=[params],
            output='screen'
        ),

        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            parameters=[params],
            output='screen'
        ),
        # Node(
        #     package='nav2_smoother',
        #     executable='smoother_server',
        #     name='smoother_server',
        #     parameters=[params],
        #     output='screen'
        # ),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            parameters=[
                params,
                {
                    'default_nav_to_pose_bt_xml':
                    '/opt/ros/jazzy/share/nav2_bt_navigator/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml'
                }
            ],
            output='screen'
        ),

        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            parameters=[params],
            output='screen'
        ),

        # 🔴 LIFECYCLE LOCALIZATION
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            parameters=[
                {'autostart': True},
                {'node_names': ['map_server', 'amcl']}
            ],
            output='screen'
        ),

        # 🔴 LIFECYCLE NAVIGATION
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            parameters=[
                {'autostart': True},
                {'node_names': [
                    'planner_server',
                    'controller_server',
                    #'smoother_server',
                    'behavior_server',
                    'bt_navigator',
                    'waypoint_follower'
                ]}
            ],
            output='screen'
        ),

        # 🔵 RVIZ (opcional)
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        ),
    ])

