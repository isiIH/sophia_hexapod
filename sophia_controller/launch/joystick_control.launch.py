from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        # Init walk node to control the walk movement and IK
        Node(
            package='sophia_controller',
            executable='walk.py',
            name='walk'
        ),

        # Init joy_node to read joystick input
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{
                'deadzone': 0.05,
                'autorepeat_rate': 20.0,
            }]
        ),

        # Init Joystick input transformer to twist
        Node(
            package='sophia_controller',
            executable='state_controller.py',
            name='state_controller',
        ),

    ])