#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Int32, Empty

import numpy as np

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

        self.height_publisher = self.create_publisher(
            Int32,
            'up_down',
            10
        )

        self.animation_publisher = self.create_publisher(
            Empty,
            'animation',
            10
        )

        # Movement params
        self.linear_limit = 0.1
        self.angular_limit = np.pi / 36 # 5 degrees
        self.previous_btn = 0

        # Gait params
        self.is_r1_pressed = False
        self.gait_idx = 0

        # Animation params
        self.is_x_pressed = False

        self.get_logger().info('State Controller has been started...')

    def state_callback(self, joy_msg : Joy):
        # self.get_logger().info(f'Recibiendo: ' + str(joy_msg))

        self.send_movement(joy_msg.axes[:4])

        self.send_gait_command(joy_msg.buttons[5])

        self.send_height_command(joy_msg.axes[-1])

        self.send_animation_command(joy_msg.buttons[0])
        

    def send_movement(self, axes):
        twist = Twist()

        # Left Stick (Movement)
        x_axis = axes[1]
        y_axis = axes[0]
        mag = np.sqrt(x_axis**2 + y_axis**2)
        
        # Deadzone & Minimum Step Size:
        # Si el joystick se mueve un poco (mag > 0.1), asegurar un paso de al menos el 60% (0.6).
        # Esto evita los pasos microscópicos y cambia la dinámica a "pasos grandes, pero lentos".
        if mag > 0.1:
            scaled_mag = 0.6 + 0.4 * min((mag - 0.1) / 0.9, 1.0)
            x_axis = (x_axis / mag) * scaled_mag
            y_axis = (y_axis / mag) * scaled_mag
        else:
            x_axis = 0.0
            y_axis = 0.0

        twist.linear.x = x_axis * self.linear_limit 
        twist.linear.y = y_axis * self.linear_limit 

        # Right Stick (Rotation)
        rot_val = axes[3]
        if abs(rot_val) > 0.1:
            scaled_rot = 0.6 + 0.4 * min((abs(rot_val) - 0.1) / 0.9, 1.0)
            twist.angular.z = np.sign(rot_val) * scaled_rot * self.angular_limit
        else:
            twist.angular.z = 0.0

        self.movement_publisher.publish(twist)

    def send_gait_command(self, str_button):
        r1 = bool(str_button)

        if(r1 != self.is_r1_pressed):
            self.is_r1_pressed = not self.is_r1_pressed

            if(self.is_r1_pressed):
                self.gait_idx = (self.gait_idx + 1) % 5

                msg = String()

                msg.data = GAITS[self.gait_idx]

                self.gait_publisher.publish(msg)

                self.get_logger().info("Sending gait change to " + GAITS[self.gait_idx])

    def send_height_command(self, str_up_down):
        up_down = int(str_up_down)

        if up_down != 0 and self.previous_btn != up_down:
            msg = Int32()
            msg.data = int(up_down)
            self.height_publisher.publish(msg)

        # Always update previous_btn so we know when it is released (returns to 0)
        self.previous_btn = up_down

    def send_animation_command(self, str_x_button):
        x = bool(str_x_button)
        
        if x != self.is_x_pressed:
            self.is_x_pressed = not self.is_x_pressed

            if self.is_x_pressed:
                self.get_logger().info("X button pressed")
                
                msg = Empty()
                self.animation_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    state = StateController()

    rclpy.spin(state)

    rclpy.shutdown()

if __name__ == '__main__':
    main()