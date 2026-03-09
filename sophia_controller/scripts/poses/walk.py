#!/usr/bin/env python3
import sys
from ament_index_python.packages import get_package_share_directory

package_share_directory = get_package_share_directory('sophia_controller')

sys.path.insert(0, package_share_directory)

import rclpy
from rclpy.node import Node

import numpy as np
from utils.spider import Spider
from utils.gait_generator import GaitGenerator

from std_msgs.msg import Float64MultiArray

import time

class WalkNode(Node):
    def __init__(self):
        super().__init__('walk')

        self.joint_pub = self.create_publisher(
            Float64MultiArray,
            '/leg_controller/commands',
            10
        )

        self.spider = Spider()
        self.time_step = 0.01
        self.first_step = True
        self.steps = 0
        self.t = 0.0

        # Walk Parameters
        self.step_vector = np.array([0.0, 0.15, 0.0])  # 5 cm step forward per cycle
        self.height = 0.05

        self.walk_controller()

    def walk_controller(self):

        self.get_logger().info("Send Standing Pose...")

        # Stand-up
        # target_pos = [0.213, 0, -0.165]
        # joint_angles = self.spider.move_legs([target_pos] * 6, local=True)

        h = -0.165 # Altura Z
        xf = 0.12  # X de patas Frontales/Traseras
        xm = 0.24  # X de patas Medias (¡Exactamente el doble de xf!)
        yf = 0.22  # Y de patas Frontales/Traseras

        standing_pose = [
            [ xf,  yf, h], # RF (Derecha Frontal)
            [ xm, 0.0, h], # RM (Derecha Media)
            [ xf, -yf, h], # RB (Derecha Trasera)
            [-xf,  yf, h], # LF (Izquierda Frontal)
            [-xm, 0.0, h], # LM (Izquierda Media)
            [-xf, -yf, h]  # LB (Izquierda Trasera)
        ]

        joint_angles = self.spider.move_legs(standing_pose, local=False)

        self.send_msg(joint_angles)

        self.gait = GaitGenerator(self.spider, "tripod")

        self.get_logger().info("Init Walk Cycle...")

        # Walk loop
        self.timer = self.create_timer(self.time_step, self.walk_loop)
    
    def walk_loop(self):
        if self.t == 0:
            self.gait.set_trajectory(self.step_vector, self.height, self.first_step)

        leg_positions = self.gait.get_next_step(self.t)

        joint_angles = self.gait.spider.move_legs(leg_positions)

        self.get_logger().info("t: " + str(self.t))

        self.get_logger().info("leg_pos: " + str(leg_positions))

        if self.t >= 1.0:
            self.steps += 1
            self.get_logger().info("Step " + str(self.steps))
            self.t = 0.0
            if self.first_step:
                self.first_step = False
        else:
            self.t = round(self.t + self.time_step, 3)

        self.send_msg(joint_angles)
        

    def send_msg(self, angles):
        msg = Float64MultiArray()
        msg.data = angles
        self.joint_pub.publish(msg)


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
