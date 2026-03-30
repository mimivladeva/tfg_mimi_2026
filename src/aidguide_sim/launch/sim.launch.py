from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_path = os.path.dirname(os.path.dirname(__file__))
    world = os.path.join(pkg_path, 'worlds', 'tfg_world.sdf')
    models = os.path.join(pkg_path, 'models')

    robot_name = 'burger'  # Debe coincidir con el <name> del modelo en el SDF
    world_name = 'MI_MUNDO_TFG_MIMI'

    return LaunchDescription([

        # Permitir modelos custom
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=models + ':' + os.environ.get('GZ_SIM_RESOURCE_PATH', '')
        ),

        # Lanzar Gazebo Harmonic
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', world],
            output='screen'
        ),

        # ===============================
        # BRIDGE GAZEBO <-> ROS2
        # ===============================
      Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    name='gz_bridge',
    output='screen',
    arguments=[
        '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',

        # cmd_vel (ROS -> GZ)
        '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',

        # ODOM (GZ -> ROS)
        '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',

        # joint states
        '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
        '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',

        '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
    ],
    parameters=[{'use_sim_time': True}],
)


        
     



    ])
