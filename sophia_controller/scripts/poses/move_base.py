#!/usr/bin/env python3
import sys
import math
import time
import numpy as np

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration

package_share_directory = get_package_share_directory('sophia_controller')
sys.path.insert(0, package_share_directory)

from utils.spider import Spider

class MoveBaseController(Node):
    def __init__(self):
        super().__init__('move_base')

        self.spider_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/leg_controller/follow_joint_trajectory'
        )

        # List of joint names for the robot
        self.joint_names = [
            'rf_coxa_joint',
            'rf_coxa_link_to_rf_femur_link',
            'rf_femur_link_to_rf_tibia_link',
            'rm_coxa_joint',
            'rm_coxa_link_to_rm_femur_link',
            'rm_femur_link_to_rm_tibia_link',
            'rb_coxa_joint',
            'rb_coxa_link_to_rb_femur_link',
            'rb_femur_link_to_rb_tibia_link',
            'lf_coxa_joint',
            'lf_coxa_link_to_lf_femur_link',
            'lf_femur_link_to_lf_tibia_link',
            'lm_coxa_joint',
            'lm_coxa_link_to_lm_femur_link',
            'lm_femur_link_to_lm_tibia_link',
            'lb_coxa_joint',
            'lb_coxa_link_to_lb_femur_link',
            'lb_femur_link_to_lb_tibia_link',
        ]

        self.spider = Spider()

    def move(self, positions, time_sec=1.0):
        point = JointTrajectoryPoint()
        point.positions = positions
        
        sec = int(time_sec)
        nanosec = int((time_sec - sec) * 1e9)
        point.time_from_start = Duration(sec=sec, nanosec=nanosec)

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names
        goal_msg.trajectory.points = [point]

        if not self.spider_client.server_is_ready():
            self.get_logger().info('Waiting for action servers...')
            self.spider_client.wait_for_server()
            self.get_logger().info('Action servers connected!')

        return self.spider_client.send_goal_async(goal_msg)


def main(args=None):
    rclpy.init(args=args)
    action_client = MoveBaseController()

    target_pos = [0.213, 0, -0.165]
    angles = action_client.spider.move_legs([target_pos] * 6)
    
    action_client.get_logger().info('Standing up...')
    future = action_client.move(angles, time_sec=1.5)
    rclpy.spin_until_future_complete(action_client, future)
    time.sleep(2)

    
    action_client.get_logger().info('Lowering base...')

    angles = action_client.spider.update_body_pos(0, 0, -0.05, 0, 0, 0)
    future = action_client.move(angles, time_sec=1.0)
    rclpy.spin_until_future_complete(action_client, future)
    time.sleep(2.0)

    # Pitch - Roll
    action_client.get_logger().info('Starting circular motion...')
    
    for ang in range(0, 361, 10): 
        rad = ang * math.pi / 180.0
        
        angles = action_client.spider.update_body_pos(
            0, 0, -0.05, 
            0.2 * np.cos(rad), 
            0.2 * np.sin(rad), 
            0
        )

        future = action_client.move(angles, time_sec=0.05)
        rclpy.spin_until_future_complete(action_client, future)
        
        time.sleep(0.1)

    action_client.get_logger().info('Sequence complete!')
    
    angles = action_client.spider.update_body_pos(0, 0, 0, 0, 0, 0)
    future = action_client.move(angles, time_sec=1.0)
    rclpy.spin_until_future_complete(action_client, future)

if __name__ == '__main__':
    main()