from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    pkg_name = 'sophia_description'
    urdf_file = 'sophia_hexapod.urdf.xacro'
    xacro_file = os.path.join(get_package_share_directory(pkg_name), 'urdf', 'mech', urdf_file)

    rviz_file = 'sophia_spider.rviz'
    rviz_config_path = os.path.join(get_package_share_directory(pkg_name), 'rviz', rviz_file)
    
    # Launch configuration variables
    use_rviz = LaunchConfiguration('use_rviz')
    use_rviz_gui = LaunchConfiguration('use_rviz_gui')
    use_gazebo = LaunchConfiguration('use_gazebo')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        name='use_rviz',
        default_value='false',
        description='Whether to start RVIZ')

    declare_use_rviz_gui_cmd = DeclareLaunchArgument(
        name='use_rviz_gui',
        default_value='false',
        description='Whether to start Joint State Publisher GUI')

    declare_use_gazebo_cmd = DeclareLaunchArgument(
        name='use_gazebo',
        default_value='true',
        description='Whether to start Gazebo')

    robot_description_content = ParameterValue(Command([
        'xacro', ' ', xacro_file, ' ',
        'use_gazebo:=', use_gazebo, ' ',
    ]), value_type=str)
    params = {'robot_description': robot_description_content}

    # Nodo: Robot State Publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # Nodo: Joint State Publisher (sin Gazebo, publica estados de joints por defecto)
    # node_joint_state_publisher = Node(
    #     condition=UnlessCondition(use_gazebo),
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher',
    #     name='joint_state_publisher'
    # )

    node_static_tf_gazebo = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_base_footprint_gazebo',
        arguments=['0', '0', '-0.15', '0', '0', '0', 'odom', 'base_footprint']
    )

    # Nodo: Joint State Publisher GUI (Para mover las patas con barritas)
    node_joint_state_publisher_gui = Node(
        condition=IfCondition(use_rviz_gui),
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    # Nodo: RViz
    node_rviz = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path]
    )

    ld = LaunchDescription()

    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_rviz_gui_cmd)
    ld.add_action(declare_use_gazebo_cmd)

    # ld.add_action(node_joint_state_publisher)
    ld.add_action(node_joint_state_publisher_gui)
    ld.add_action(node_robot_state_publisher)
    ld.add_action(node_static_tf_gazebo)
    ld.add_action(node_rviz)

    return ld