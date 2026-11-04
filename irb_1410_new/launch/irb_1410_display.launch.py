import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_path = get_package_share_directory('irb_1410_new')
    urdf_path = os.path.join(pkg_path, 'urdf', 'newirb1410.urdf.xacro')
    rviz_config_path = os.path.join(pkg_path, 'rviz', 'irb1410_rviz_config_model_t1.rviz')

    # Process xacro and get robot description as a plain string
    robot_description = xacro.process_file(urdf_path).toxml()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}]
        ),

        # REMOVE this block (joint_state_publisher_gui):
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
        ),

        # ADD this: our socket player node
        
        Node(
            package='irb1410_bridge',
            executable='irb1410_socket_player',
            output='screen'
        ),
        
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path]
        )
    ])
