#!/usr/bin/env python3
import sys
import json
import math
import xml.etree.ElementTree as ET
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from ament_index_python.packages import get_package_share_directory
import os

def euler_to_quaternion(roll, pitch, yaw):
    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    return [qx, qy, qz, qw]

def mat_T(x=0, y=0, z=0, roll=0, pitch=0, yaw=0):
    """ZYX Euler homogeneous transform"""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [
        [cp*cy,            -cp*sy,            sp,    x],
        [sr*sp*cy + cr*sy, -sr*sp*sy + cr*cy, -sr*cp, y],
        [-cr*sp*cy + sr*sy, cr*sp*sy + sr*cy, cr*cp,  z],
        [0,                 0,                 0,      1],
    ]

def mat_mul(A, B):
    C = [[0]*4 for _ in range(4)]
    for i in range(4):
        for j in range(4):
            for k in range(4):
                C[i][j] += A[i][k] * B[k][j]
    return C

def mat_inv(M):
    """Inverse of a homogeneous 4x4 matrix (R^T, -R^T * t)."""
    R = [[M[0][0], M[1][0], M[2][0]],
         [M[0][1], M[1][1], M[2][1]],
         [M[0][2], M[1][2], M[2][2]]]
    t = [M[0][3], M[1][3], M[2][3]]
    nt = [
        -(R[0][0]*t[0] + R[0][1]*t[1] + R[0][2]*t[2]),
        -(R[1][0]*t[0] + R[1][1]*t[1] + R[1][2]*t[2]),
        -(R[2][0]*t[0] + R[2][1]*t[1] + R[2][2]*t[2]),
    ]
    return [
        [R[0][0], R[0][1], R[0][2], nt[0]],
        [R[1][0], R[1][1], R[1][2], nt[1]],
        [R[2][0], R[2][1], R[2][2], nt[2]],
        [0, 0, 0, 1],
    ]

# ==================== URDF PARSER ====================
def parse_urdf_leg_params(urdf_path):
    """Parse URDF to extract leg geometry: base origin and link lengths."""
    tree = ET.parse(urdf_path)
    root = tree.getroot()
    prefixes = ['rf', 'rm', 'rb', 'lf', 'lm', 'lb']
    legs = []

    for prefix in prefixes:
        base_joint = root.find(f".//joint[@name='{prefix}_fixed_base_joint']")
        origin = base_joint.find('origin')
        xyz = [float(v) for v in origin.get('xyz').split()]
        rpy = [float(v) for v in origin.get('rpy').split()]

        femur_joint = root.find(f".//joint[@name='{prefix}_coxa_link_to_{prefix}_femur_link']")
        coxa_len = float(femur_joint.find('origin').get('xyz').split()[0])

        tibia_joint = root.find(f".//joint[@name='{prefix}_femur_link_to_{prefix}_tibia_link']")
        femur_len = float(tibia_joint.find('origin').get('xyz').split()[0])

        tibia_link = root.find(f".//link[@name='{prefix}_tibia_link']")
        tibia_len = 0.0
        for visual in tibia_link.findall('visual'):
            geom = visual.find('geometry')
            if geom is not None and geom.find('sphere') is not None:
                tibia_len = float(visual.find('origin').get('xyz').split()[0])
                break

        # Build the coxa transform
        T_coxa = mat_T(xyz[0], xyz[1], xyz[2], rpy[0], rpy[1], rpy[2])

        legs.append({
            'T_coxa': T_coxa,
            'coxa_len': coxa_len, 'femur_len': femur_len, 'tibia_len': tibia_len,
        })

    return legs

# ==================== FK ====================
def leg_fk(coxa, femur, tibia, leg_param):
    """Forward kinematics."""
    Lc = leg_param['coxa_len']
    Lf = leg_param['femur_len']
    Lt = leg_param['tibia_len']

    T0 = mat_T(yaw=coxa)
    T1 = mat_mul(mat_mul(T0, mat_T(x=Lc)), mat_T(pitch=femur))
    T2 = mat_mul(mat_mul(T1, mat_T(x=Lf)), mat_T(pitch=tibia))
    Tf = mat_mul(T2, mat_T(x=Lt))
    return [Tf[0][3], Tf[1][3], Tf[2][3]]

def recover_legs_value(coxa, femur, tibia, body, leg_param):
    """Recover the legs[i] value"""
    pos_cf = leg_fk(coxa, femur, tibia, leg_param)

    T_pos = [[1,0,0,pos_cf[0]], [0,1,0,pos_cf[1]], [0,0,1,pos_cf[2]], [0,0,0,1]]
    T_sb = mat_T(body.get('x',0), body.get('y',0), body.get('z',0),
                 body.get('roll',0), body.get('pitch',0), body.get('yaw',0))
    T_coxa = leg_param['T_coxa']

    Ts_foot = mat_mul(mat_mul(T_sb, T_coxa), T_pos)
    Tc_f = mat_mul(mat_inv(T_coxa), Ts_foot)

    return [Tc_f[0][3], Tc_f[1][3], Tc_f[2][3]]

# ==================== EASING FUNCTIONS ====================
def ease_linear(t):
    return t

def ease_in(t):
    return t ** 3

def ease_out(t):
    return 1.0 - (1.0 - t) ** 3

def ease_in_out(t):
    if t < 0.5:
        return 4.0 * t**3
    else:
        return 1.0 - (-2.0 * t + 2.0)**3 / 2.0

EASING = {
    'linear': ease_linear,
    'ease-in': ease_in,
    'ease-out': ease_out,
    'ease-in-out': ease_in_out,
}

class AnimationPlayer:
    def __init__(self, keyframes, leg_params=None):
        self.keyframes = keyframes
        self.leg_params = leg_params
        self.total_duration = sum(kf.get('duration', 1.0) for kf in self.keyframes[1:]) if len(self.keyframes) > 1 else 0
        self.elapsed = 0.0
        self.playing = False

    def play(self):
        self.elapsed = 0.0
        self.playing = True

    def is_playing(self):
        return self.playing

    def get_frame(self, dt):
        if not self.playing:
            return None

        self.elapsed += dt

        if self.elapsed >= self.total_duration:
            if self.total_duration > 0:
                self.elapsed = self.elapsed % self.total_duration
            else:
                self.elapsed = 0.0

        if len(self.keyframes) == 0:
            return None
        if len(self.keyframes) == 1:
            body = self.keyframes[0].get('body', {'x':0, 'y':0, 'z':0, 'roll':0, 'pitch':0, 'yaw':0})
            return body, self.keyframes[0]['angles']

        accumulated = 0.0
        for seg_idx in range(1, len(self.keyframes)):
            kf_a = self.keyframes[seg_idx - 1]
            kf_b = self.keyframes[seg_idx]
            duration = kf_b.get('duration', 1.0)

            if accumulated + duration >= self.elapsed or seg_idx == len(self.keyframes) - 1:
                local_elapsed = self.elapsed - accumulated
                t = min(local_elapsed / duration, 1.0) if duration > 0 else 1.0
                return self._interpolate(kf_a, kf_b, t, kf_b.get('easing', 'ease-in-out'))

            accumulated += duration

        return None

    def _interpolate(self, kf_a, kf_b, t, easing_name):
        ease_fn = EASING.get(easing_name, ease_linear)
        et = ease_fn(t)

        arc_height = kf_b.get('arc_height', 0.0)
        traj = kf_b.get('trajectory', 'linear')
        effective_arc = arc_height
        if traj == 'bezier' and effective_arc < 0.001:
            effective_arc = 0.04

        has_legs = 'legs' in kf_a and 'legs' in kf_b
        can_fk = (not has_legs) and self.leg_params is not None
        body_a = kf_a.get('body', {'x':0, 'y':0, 'z':0, 'roll':0, 'pitch':0, 'yaw':0})
        body_b = kf_b.get('body', {'x':0, 'y':0, 'z':0, 'roll':0, 'pitch':0, 'yaw':0})

        angles = []
        for i in range(6):
            ci, fi, ti = i*3, i*3+1, i*3+2

            a0_c, a1_c = kf_a['angles'][ci], kf_b['angles'][ci]
            a0_f, a1_f = kf_a['angles'][fi], kf_b['angles'][fi]
            a0_t, a1_t = kf_a['angles'][ti], kf_b['angles'][ti]

            c_val = a0_c * (1.0 - et) + a1_c * et
            f_val = a0_f * (1.0 - et) + a1_f * et
            t_val = a0_t * (1.0 - et) + a1_t * et

            if effective_arc > 0.001:
                dist = 0.0
                if has_legs:
                    p0x, p0y = kf_a['legs'][i][0], kf_a['legs'][i][1]
                    p2x, p2y = kf_b['legs'][i][0], kf_b['legs'][i][1]
                    dist = math.hypot(p2x - p0x, p2y - p0y)
                elif can_fk:
                    # Recover the legs[i] values by undoing the body transform
                    # (mirrors Spider.calcLegPosFromAngles)
                    leg0 = recover_legs_value(a0_c, a0_f, a0_t, body_a, self.leg_params[i])
                    leg1 = recover_legs_value(a1_c, a1_f, a1_t, body_b, self.leg_params[i])
                    dist = math.hypot(leg1[0] - leg0[0], leg1[1] - leg0[1])

                if dist > 0.005:
                    f_val -= 4.0 * effective_arc * et * (1.0 - et) * 15.0

            angles.extend([c_val, f_val, t_val])

        body = {}
        for k in ['x', 'y', 'z', 'roll', 'pitch', 'yaw']:
            body[k] = body_a.get(k, 0) * (1.0 - et) + body_b.get(k, 0) * et

        return body, angles

class AnimationNode(Node):
    def __init__(self):
        super().__init__('animation_node')

        self.publisher = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.declare_parameter('animation_file', '')
        self.declare_parameter('urdf_file', '')

        json_file = self.get_parameter('animation_file').get_parameter_value().string_value
        urdf_file = self.get_parameter('urdf_file').get_parameter_value().string_value

        if not json_file and len(sys.argv) > 1:
            for arg in sys.argv[1:]:
                if arg.endswith('.json'):
                    json_file = arg
                    break

        if not json_file:
            self.get_logger().error("No animation JSON file provided!")
            sys.exit(1)

        if not urdf_file:
            urdf_file = os.path.join(
                get_package_share_directory('hexapod_animation'), 'urdf', 'sophia.urdf')

        try:
            with open(json_file, 'r') as f:
                data = json.load(f)
            keyframes = data.get('keyframes', data)
        except Exception as e:
            self.get_logger().error(f"Failed to load JSON file {json_file}: {e}")
            sys.exit(1)

        leg_params = None
        try:
            leg_params = parse_urdf_leg_params(urdf_file)
            self.get_logger().info(f"Parsed leg geometry from {urdf_file}")
        except Exception as e:
            self.get_logger().warn(f"Could not parse URDF: {e}. arc_height disabled for JSONs without 'legs'.")

        self.player = AnimationPlayer(keyframes, leg_params)
        self.dt = 0.02

        self.get_logger().info(f"Ready. Playing {json_file} in 1 second...")
        self.startup_timer = self.create_timer(1.0, self.start_playback)

    def start_playback(self):
        self.startup_timer.cancel()
        self.player.play()
        self.get_logger().info("Playing animation...")
        self.timer = self.create_timer(self.dt, self.timer_callback)

    def timer_callback(self):
        if not self.player.is_playing():
            self.get_logger().info("Animation finished. Exiting...")
            self.timer.cancel()
            rclpy.shutdown()
            return

        frame_data = self.player.get_frame(self.dt)
        if frame_data is None:
            return

        body, angles = frame_data

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = float(body.get('x', 0))
        t.transform.translation.y = float(body.get('y', 0))
        t.transform.translation.z = float(body.get('z', 0))
        q = euler_to_quaternion(body.get('roll', 0), body.get('pitch', 0), body.get('yaw', 0))
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)

        msg = JointTrajectory()
        prefixes = ['rf', 'rm', 'rb', 'lf', 'lm', 'lb']
        for prefix in prefixes:
            msg.joint_names.extend([
                f'{prefix}_coxa_joint',
                f'{prefix}_coxa_link_to_{prefix}_femur_link',
                f'{prefix}_femur_link_to_{prefix}_tibia_link'
            ])
        point = JointTrajectoryPoint()
        point.positions = [float(a) for a in angles]
        point.time_from_start = Duration(sec=0, nanosec=int(self.dt * 1e9))
        msg.points.append(point)
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = AnimationNode()
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
