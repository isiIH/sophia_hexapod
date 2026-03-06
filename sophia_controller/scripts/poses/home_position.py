#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class HomePositionNode(Node):
    def __init__(self):
        super().__init__('home_position')

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

        # Define target position
        self.target_pos = [
            0, 0, 0, # Leg 1
            0, 0, 0, # Leg 2
            0, 0, 0, # Leg 3
            0, 0, 0, # Leg 4
            0, 0, 0, # Leg 5
            0, 0, 0, # Leg 6
        ]

    def get_home_controller(self):
        # Create a trajectory point with the target positions
        point = JointTrajectoryPoint()
        point.positions = self.target_pos
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

    action_client = HomePositionNode()

    future = action_client.get_home_controller()

    rclpy.spin_until_future_complete(action_client, future)

if __name__ == '__main__':
    main()
