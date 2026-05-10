#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState

class TrajectoryToStateNode(Node):
    def __init__(self):
        super().__init__('trajectory_to_state')
        
        self.subscription = self.create_subscription(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            self.trajectory_callback,
            10
        )
        
        self.publisher = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )
        
        self.get_logger().info('Trajectory to State Bridge Node started.')

    def trajectory_callback(self, msg: JointTrajectory):
        if not msg.points:
            return
            
        state_msg = JointState()
        state_msg.header.stamp = self.get_clock().now().to_msg()
        state_msg.name = msg.joint_names
        
        # We take the first point (which is usually the only point in this 50Hz stream)
        state_msg.position = list(msg.points[0].positions)
        
        self.publisher.publish(state_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryToStateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except rclpy.executors.ExternalShutdownException:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
