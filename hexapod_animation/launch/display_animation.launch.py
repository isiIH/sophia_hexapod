import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_animation = 'hexapod_animation'
    
    urdf_file = 'sophia.urdf'
    urdf_path = os.path.join(get_package_share_directory(pkg_animation), 'urdf', urdf_file)
    
    rviz_file = 'sophia_spider.rviz'
    rviz_config_path = os.path.join(get_package_share_directory(pkg_animation), 'rviz', rviz_file)
    
    default_animation = os.path.join(get_package_share_directory(pkg_animation), 'animations', 'attack.json')
    
    animation_file_arg = DeclareLaunchArgument(
        'animation_file',
        default_value=default_animation,
        description='Path to the animation JSON file'
    )
    
    with open(urdf_path, 'r') as f:
        robot_description_content = f.read()
    
    params = {'robot_description': robot_description_content}
    
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )
    
    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path]
    )
    
    node_trajectory_to_state = Node(
        package='hexapod_animation',
        executable='trajectory_to_state',
        name='trajectory_to_state',
        output='screen'
    )
    
    node_play_animation = Node(
        package='hexapod_animation',
        executable='play_animation',
        name='play_animation',
        output='screen',
        parameters=[{'animation_file': LaunchConfiguration('animation_file')}]
    )

    return LaunchDescription([
        animation_file_arg,
        node_robot_state_publisher,
        node_rviz,
        node_trajectory_to_state,
        node_play_animation
    ])
