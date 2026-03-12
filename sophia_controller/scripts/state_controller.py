#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

GAITS = ['tripod', 'ripple', 'wave', 'bi', 'tetrapod']

class StateController(Node):
    def __init__(self):
        super().__init__('state_controller')
        self.state_subscriber = self.create_subscription(
            Joy,
            'joy',
            self.state_callback,
            10
        )

        self.movement_publisher = self.create_publisher(
            Twist,
            '/joy1/cmd_vel',
            10
        )

        self.gait_publisher = self.create_publisher(
            String,
            'gait',
            10
        )

        # Movement params
        self.linear_limit = 0.3
        self.angular_limit = 1.0

        # Gait params
        self.is_pressed = False
        self.gait_idx = 0

        self.get_logger().info('State Controller has been started...')

    def state_callback(self, joy_msg : Joy):
        # self.get_logger().info(f'Recibiendo: ' + str(joy_msg))

        self.send_movement(joy_msg.axes[:3])

        self.send_gait_command(joy_msg.buttons[5])
        

    def send_movement(self, axes):
        twist = Twist()

        # Left Stick (Movement)
        twist.linear.x = axes[1] * self.linear_limit 
        twist.linear.y = axes[0] * self.linear_limit 

        # Right Stick (Rotation)
        twist.angular.z = axes[2] * self.angular_limit

        self.movement_publisher.publish(twist)

    def send_gait_command(self, str_button):
        r1 = bool(str_button)

        if(r1 != self.is_pressed):
            self.is_pressed = not self.is_pressed

            if(self.is_pressed):
                self.gait_idx = (self.gait_idx + 1) % 5

                msg = String()

                msg.data = GAITS[self.gait_idx]

                self.gait_publisher.publish(msg)

                self.get_logger().info("Sending gait change to " + GAITS[self.gait_idx])

def main(args=None):
    rclpy.init(args=args)

    state = StateController()

    rclpy.spin(state)

    rclpy.shutdown()

if __name__ == '__main__':
    main()