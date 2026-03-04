import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_name = 'sophia_description'
    urdf_file = 'sophia_hexapod.urdf.xacro'
    xacro_file = os.path.join(get_package_share_directory(pkg_name), 'urdf', 'mech', urdf_file)

    rviz_file = 'sophia_spider.rviz'
    rviz_config_path = os.path.join(get_package_share_directory(pkg_name), 'rviz', rviz_file)
    
    # Procesar Xacro
    robot_description_config = xacro.process_file(xacro_file)
    params = {'robot_description': robot_description_config.toxml()}

    # Nodo: Robot State Publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # Nodo: Joint State Publisher GUI (Para mover las patas con barritas)
    node_joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    # Nodo: RViz
    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path]
    )

    return LaunchDescription([
        node_robot_state_publisher,
        node_joint_state_publisher_gui,
        node_rviz
    ])