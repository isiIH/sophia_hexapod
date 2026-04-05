#!/usr/bin/env python3
"""
RViz trajectory visualizer for hexapod leg tips.

Uses TF2 to track EXACTLY where RViz placed the foot meshes
so the trailing paths perfectly align with the actual visual model.
"""

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration
from collections import deque
import tf2_ros


LEG_NAMES = ['RF', 'RM', 'RB', 'LF', 'LM', 'LB']
LEG_PREFIXES = ['rf_', 'rm_', 'rb_', 'lf_', 'lm_', 'lb_']

# Reduce max length so the trail vanishes cleanly as requested
TRAIL_LENGTH = 20  


def qv_mult(q1, v1):
    # Applies quaternion q1 (x, y, z, w) to vector v1 (x, y, z)
    q2 = (v1[0], v1[1], v1[2], 0.0)
    
    def q_mult(qa, qb):
        x = qa[3]*qb[0] + qa[0]*qb[3] + qa[1]*qb[2] - qa[2]*qb[1]
        y = qa[3]*qb[1] - qa[0]*qb[2] + qa[1]*qb[3] + qa[2]*qb[0]
        z = qa[3]*qb[2] + qa[0]*qb[1] - qa[1]*qb[0] + qa[2]*qb[3]
        w = qa[3]*qb[3] - qa[0]*qb[0] - qa[1]*qb[1] - qa[2]*qb[2]
        return x, y, z, w

    def q_conjugate(q):
        return -q[0], -q[1], -q[2], q[3]

    q_tmp = q_mult(q1, q2)
    res = q_mult(q_tmp, q_conjugate(q1))
    return res[0], res[1], res[2]


class LegTrajectoryVisualizer(Node):
    def __init__(self):
        super().__init__('leg_trajectory_visualizer')

        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/leg_trajectories',
            10
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Rolling buffer of positions for each leg
        self.trails = [deque(maxlen=TRAIL_LENGTH) for _ in range(6)]
        
        # 50 Hz UI timer to actively poll TF
        self.timer = self.create_timer(0.02, self.timer_callback)

    def timer_callback(self):
        now = self.get_clock().now()
        
        marker_array = MarkerArray()
        
        # In URDF, the foot tip is a sphere located at 
        # x = tibia_link_width - tibia_sphere_radius 
        # x = 0.09 - 0.0075 = 0.0825 in the tibia_link frame.
        FOOT_OFFSET = (0.0825, 0.0, 0.0)
        
        for i, prefix in enumerate(LEG_PREFIXES):
            try:
                t = self.tf_buffer.lookup_transform(
                    'base_link',
                    f'{prefix}tibia_link',
                    rclpy.time.Time()
                )
                
                # Transform origin to foot tip
                q = (
                    t.transform.rotation.x,
                    t.transform.rotation.y,
                    t.transform.rotation.z,
                    t.transform.rotation.w
                )
                
                rx, ry, rz = qv_mult(q, FOOT_OFFSET)
                
                tip_x = t.transform.translation.x + rx
                tip_y = t.transform.translation.y + ry
                tip_z = t.transform.translation.z + rz
                
                # Update trail
                self.trails[i].append((tip_x, tip_y, tip_z))
                
                if len(self.trails[i]) < 2:
                    continue
                    
                marker = Marker()
                marker.header.frame_id = 'base_link'
                marker.header.stamp = now.to_msg()
                marker.ns = f'leg_trail_{LEG_NAMES[i]}'
                marker.id = i
                marker.type = Marker.LINE_STRIP
                marker.action = Marker.ADD

                marker.scale.x = 0.003
                marker.lifetime = Duration(sec=0, nanosec=100_000_000)
                marker.pose.orientation.w = 1.0

                # Build points with fading alpha
                trail_len = len(self.trails[i])
                for j, pt in enumerate(self.trails[i]):
                    from geometry_msgs.msg import Point
                    from std_msgs.msg import ColorRGBA

                    p = Point(x=float(pt[0]), y=float(pt[1]), z=float(pt[2]))
                    marker.points.append(p)

                    alpha = (j + 1) / trail_len
                    c = ColorRGBA(r=1.0, g=0.0, b=0.0, a=float(alpha))
                    marker.colors.append(c)

                marker_array.markers.append(marker)

            except tf2_ros.TransformException:
                pass  # TF not ready yet
                
        if marker_array.markers:
            self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = LegTrajectoryVisualizer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
