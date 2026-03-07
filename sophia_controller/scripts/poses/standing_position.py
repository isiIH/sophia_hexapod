#!/usr/bin/env python3
import sys
from ament_index_python.packages import get_package_share_directory

package_share_directory = get_package_share_directory('sophia_controller')

sys.path.insert(0, package_share_directory)

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration

from utils.spider import Spider

class StandingPositionNode(Node):
    def __init__(self):
        super().__init__('standing_position')

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
        target_pos = [0.213, 0, -0.165]
        self.angles = self.spider.move_legs([target_pos] * 6)
        self.get_logger().info(str(self.angles))

    def stand_up_controller(self):
        # Create a trajectory point with the target positions
        point = JointTrajectoryPoint()
        point.positions = self.angles
        # faster movement: allow only 0.2 seconds to reach the target
        point.time_from_start = Duration(sec=0, nanosec=200000000)

        # Create and send the goal message
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names
        goal_msg.trajectory.points = [point]

        # Wait for both action servers to be available
        self.get_logger().info('Waiting for action servers...')
        self.spider_client.wait_for_server()
        self.get_logger().info('Action servers connected!')

        return self.spider_client.send_goal_async(goal_msg)


def main(args=None):
    rclpy.init(args=args)

    action_client = StandingPositionNode()

    future = action_client.stand_up_controller()

    rclpy.spin_until_future_complete(action_client, future)

if __name__ == '__main__':
    main()
