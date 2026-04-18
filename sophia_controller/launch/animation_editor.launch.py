from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_desc = get_package_share_directory('sophia_description')
    xacro_file = os.path.join(pkg_desc, 'urdf', 'mech', 'sophia_hexapod.urdf.xacro')
    rviz_config = os.path.join(pkg_desc, 'rviz', 'sophia_spider.rviz')

    robot_description = ParameterValue(
        Command(['xacro', ' ', xacro_file, ' ', 'use_gazebo:=false']),
        value_type=str
    )

    ld = LaunchDescription()

    # Robot State Publisher (URDF → TF for RViz)
    ld.add_action(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    ))

    # Animation Editor Node (Spider kinematics, HTTP API, and GUI Server)
    ld.add_action(Node(
        package='sophia_controller',
        executable='animation_editor.py',
        name='animation_editor',
        output='screen',
    ))

    # RViz
    ld.add_action(Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
    ))

    return ld
