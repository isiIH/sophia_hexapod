#!/usr/bin/env python3
import sys
from ament_index_python.packages import get_package_share_directory

package_share_directory = get_package_share_directory('sophia_controller')

sys.path.insert(0, package_share_directory)

import rclpy
from rclpy.node import Node

import numpy as np
from utils.spider import Spider

from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String, Int32
from geometry_msgs.msg import Twist

class WalkNode(Node):
    def __init__(self):
        super().__init__('walk')

        self.joint_pub = self.create_publisher(
            Float64MultiArray,
            '/leg_controller/commands',
            10
        )

        self.target_pub = self.create_publisher(
            Float64MultiArray,
            '/leg_target_positions',
            10
        )

        self.gait_sub = self.create_subscription(
            String,
            "gait",
            self.set_gait,
            10
        )

        self.up_down_sub = self.create_subscription(
            Int32,
            "up_down",
            self.set_height,
            10
        )

        self.cmd_sub = self.create_subscription(
            Twist,
            "joy1/cmd_vel",
            self.set_cmd_vel,
            10
        )

        self.spider = Spider()
        self.time_step = 0.02

        self.walk_controller()

    def walk_controller(self):

        self.get_logger().info("Send Standing Pose...")

        # Stand-up
        joint_angles = self.spider.get_joint_angles()

        self.send_angles(joint_angles)

        self.get_logger().info("Init Walk Cycle...")

        # Walk loop
        self.timer = self.create_timer(self.time_step, self.walk_loop)
    
    def walk_loop(self):
        self.leg_positions = self.spider.gait.get_next_step()

        # Publish the raw leg target points
        msg = Float64MultiArray()
        msg.data = self.leg_positions.flatten().tolist()
        self.target_pub.publish(msg)

        joint_angles = self.spider.move_legs(self.leg_positions)

        self.send_angles(joint_angles)

    def send_angles(self, angles):
        msg = Float64MultiArray()
        msg.data = angles
        self.joint_pub.publish(msg)

    def set_height(self, msg : Int32):
        self.spider.set_height(msg.data)

    def set_gait(self, msg : String):
        self.get_logger().info("Setting gait to " + msg.data)
        self.spider.gait.set_gait_type(msg.data)

    def set_cmd_vel(self, msg : Twist):
        linear_speed = np.array([msg.linear.x, msg.linear.y, 0.0])
        angular_speed = msg.angular.z

        self.spider.gait.set_linear_speed(linear_speed)
        self.spider.gait.set_angular_speed(angular_speed)


def main(args=None):
    rclpy.init(args=args)

    node = WalkNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Stopping Walk Cycle...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
