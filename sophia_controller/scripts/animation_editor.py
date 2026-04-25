#!/usr/bin/env python3
"""
Hexapod Animation Program - ROS2 Backend Node

Full-featured animation tool for the hexapod robot:
  - Timeline with keyframes (body 6DOF + 6 leg positions each)
  - Cubic Bezier interpolation between keyframes
  - Per-segment duration and easing curves
  - Playback engine with loop/ping-pong modes
  - Onion skinning (ghost markers in RViz)
  - Python code export

GUI served on http://localhost:8080.
Communication via built-in HTTP API (no rosbridge needed):
  GET  /api/state  → JSON state
  POST /api/cmd    → JSON command {action, payload}
"""

import sys
import json
import math
import os
import time
import threading
from http.server import HTTPServer, SimpleHTTPRequestHandler
from pathlib import Path
from urllib.parse import urlparse

from ament_index_python.packages import get_package_share_directory

package_share_directory = get_package_share_directory('sophia_controller')
sys.path.insert(0, package_share_directory)

import rclpy
from rclpy.node import Node
import numpy as np

from utils.spider import Spider

from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, TransformStamped
from std_msgs.msg import ColorRGBA
import tf2_ros

# Joint name pattern (must match URDF)
PREFIXES = ['rf', 'rm', 'rb', 'lf', 'lm', 'lb']
LEG_NAMES = ['RF', 'RM', 'RB', 'LF', 'LM', 'LB']

# ==================== Easing Functions ====================

def ease_linear(t):
    return t

def ease_in(t):
    return t * t * t

def ease_out(t):
    return 1.0 - (1.0 - t) ** 3

def ease_in_out(t):
    if t < 0.5:
        return 4.0 * t * t * t
    else:
        return 1.0 - (-2.0 * t + 2.0) ** 3 / 2.0

def ease_bezier(t, p1x, p1y, p2x, p2y):
    """Custom cubic bezier easing (like CSS cubic-bezier)."""
    u = t
    for _ in range(8):
        bx = 3.0 * (1 - u)**2 * u * p1x + 3.0 * (1 - u) * u**2 * p2x + u**3
        dbx = 3.0 * (1 - u)**2 * p1x + 6.0 * (1 - u) * u * (p2x - p1x) + 3.0 * u**2 * (1.0 - p2x)
        if abs(dbx) < 1e-8:
            break
        u -= (bx - t) / dbx
        u = max(0.0, min(1.0, u))
    return 3.0 * (1 - u)**2 * u * p1y + 3.0 * (1 - u) * u**2 * p2y + u**3

EASING_MAP = {
    'linear': ease_linear,
    'ease-in': ease_in,
    'ease-out': ease_out,
    'ease-in-out': ease_in_out,
}


# ==================== HTTP Server with API ====================

class AnimationHTTPHandler(SimpleHTTPRequestHandler):
    """Serves static GUI files + JSON API endpoints."""

    # Reference to the node (set by the factory)
    node = None

    def __init__(self, *args, directory=None, **kwargs):
        self._gui_dir = directory
        super().__init__(*args, directory=directory, **kwargs)

    def log_message(self, fmt, *args):
        pass

    def do_GET(self):
        parsed = urlparse(self.path)
        if parsed.path == '/api/state':
            state_json = self.node.get_state_json()
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(state_json.encode())
        else:
            super().do_GET()

    def do_POST(self):
        parsed = urlparse(self.path)
        if parsed.path == '/api/cmd':
            content_len = int(self.headers.get('Content-Length', 0))
            body = self.rfile.read(content_len).decode()
            try:
                cmd = json.loads(body)
                self.node.handle_command(cmd)
                self.send_response(200)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(b'{"ok":true}')
            except Exception as e:
                self.send_response(400)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(json.dumps({'error': str(e)}).encode())
        else:
            self.send_response(404)
            self.end_headers()

    def do_OPTIONS(self):
        """Handle CORS preflight."""
        self.send_response(200)
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')
        self.end_headers()


# ==================== Main Node ====================

class AnimationEditorNode(Node):
    def __init__(self):
        super().__init__('animation_editor')

        # --- Spider ---
        self.spider = Spider(0.02)
        self.current_angles = self.spider.get_joint_angles()

        # --- Current editing state ---
        self.body = {'x': 0.0, 'y': 0.0, 'z': 0.0,
                     'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}
        self.home_legs = self.spider.home_positions.tolist()
        self.legs = [list(pos) for pos in self.home_legs]

        # --- Timeline ---
        self.keyframes = []
        self.selected_kf = -1

        # --- Playback ---
        self.playing = False
        self.play_time = 0.0
        self.play_mode = 'once'
        self.play_direction = 1
        self.play_speed = 1.0
        self.last_tick = None

        # --- Thread lock for state access ---
        self.lock = threading.Lock()

        # --- Angle overrides: legs explicitly set via angle sliders ---
        # Persists across _apply_pose() calls so changing one leg doesn't corrupt another.
        # Key = leg index (0-5), Value = list of 3 angles in radians.
        self._angle_overrides = {}

        # --- Publishers & Broadcasters ---
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.ghost_pub = self.create_publisher(MarkerArray, '/anim/ghosts', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # --- Timers ---
        self.create_timer(0.02, self.tick)          # 50Hz: joint publish + playback
        self.create_timer(0.5, self.publish_ghosts) # 2Hz: onion skinning

        # --- HTTP server ---
        gui_dir_share = os.path.join(package_share_directory, 'animation_gui')
        gui_dir_local = str(Path(__file__).parent / 'animation_gui')
        gui_dir = gui_dir_share if os.path.isdir(gui_dir_share) else gui_dir_local

        # Wire up the handler class to this node
        AnimationHTTPHandler.node = self
        handler_factory = lambda *a, **kw: AnimationHTTPHandler(
            *a, directory=gui_dir, **kw)
        self.http = HTTPServer(('0.0.0.0', 8080), handler_factory)
        threading.Thread(target=self.http.serve_forever, daemon=True).start()

        self.get_logger().info('Animation Editor ready — GUI at http://localhost:8080')

    # ==================== HTTP API ====================

    def get_state_json(self):
        """Generate state JSON for the GUI (called from HTTP thread)."""
        with self.lock:
            angles_deg = []
            for i in range(6):
                angles_deg.append([
                    round(math.degrees(self.current_angles[i * 3]), 2),
                    round(math.degrees(self.current_angles[i * 3 + 1]), 2),
                    round(math.degrees(self.current_angles[i * 3 + 2]), 2),
                ])

            state = {
                'body': {k: round(v, 5) for k, v in self.body.items()},
                'legs': [[round(v, 5) for v in l] for l in self.legs],
                'home_legs': self.home_legs,
                'joint_angles_deg': angles_deg,
                'leg_names': LEG_NAMES,
                'keyframes': self.keyframes,
                'selected_kf': self.selected_kf,
                'playing': self.playing,
                'play_time': round(self.play_time, 3),
                'total_duration': round(self._total_duration(), 3),
                'play_mode': self.play_mode,
                'play_speed': self.play_speed,
            }
        return json.dumps(state)

    def handle_command(self, cmd):
        """Process a command from the GUI (called from HTTP thread)."""
        action = cmd.get('action', '')
        payload = cmd.get('payload', {})

        with self.lock:
            self._dispatch(action, payload)

    # ==================== Command Dispatch ====================

    def _dispatch(self, action, payload):
        match action:
            # --- Pose editing ---
            case 'set_body':
                self.body.update(payload)
                self._apply_pose()
            case 'set_leg':
                idx = payload.get('index', 0)
                pos = payload.get('pos', self.home_legs[idx])
                self.legs[idx] = list(pos)
                # Moving position slider cancels any angle override for this leg
                self._angle_overrides.pop(idx, None)
                self._apply_pose()
            case 'set_leg_angles':
                idx = payload.get('index', 0)
                # Note: angles come in degrees from GUI, convert to rads
                angles_deg = payload.get('angles', [0,0,0])
                angles_rad = [math.radians(a) for a in angles_deg]
                
                # FK returns foot position relative to the INSTANTANEOUS coxa frame 
                # (which is currently translated/rotated by T_sb body offsets)
                pos_cf = self.spider.legs[idx].leg_fk(angles_rad)
                T_pos = np.eye(4)
                T_pos[:3, 3] = pos_cf
                
                # We must map this physical position back to the logical "anchor" coordinate space 
                # (which expects footprint relative to the UN-TRANSFORMED coxa)
                T_sb = self.spider.T_sb
                T_coxa = self.spider.legs[idx].T_coxa
                Ts_foot = T_sb @ T_coxa @ T_pos
                Tc_f_stored = np.linalg.inv(T_coxa) @ Ts_foot
                
                self.legs[idx] = Tc_f_stored[:3, 3].tolist()
                # Save override BEFORE _apply_pose so it is preserved immediately
                self._angle_overrides[idx] = list(angles_rad)
                self.spider.legs[idx].ths[:] = angles_rad
                # Run full pose (IK for all legs, then overrides are restored inside)
                self._apply_pose()
            case 'home':
                self.body = {'x': 0.0, 'y': 0.0, 'z': 0.0,
                             'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}
                self.legs = [list(h) for h in self.home_legs]
                self._angle_overrides.clear()  # home resets all explicit angle overrides
                self._apply_pose()

            # --- Keyframe management ---
            case 'kf_add':
                kf = {
                    'body': dict(self.body),
                    'legs': [list(l) for l in self.legs],
                    'duration': payload.get('duration', 1.0),
                    'easing': payload.get('easing', 'ease-in-out'),
                    'bezier_p1': payload.get('bezier_p1', [0.42, 0.0]),
                    'bezier_p2': payload.get('bezier_p2', [0.58, 1.0]),
                }
                insert_at = payload.get('index', len(self.keyframes))
                self.keyframes.insert(insert_at, kf)
            case 'kf_update':
                idx = payload.get('index', -1)
                if 0 <= idx < len(self.keyframes):
                    self.keyframes[idx]['body'] = dict(self.body)
                    self.keyframes[idx]['legs'] = [list(l) for l in self.legs]
                    if 'duration' in payload:
                        self.keyframes[idx]['duration'] = payload['duration']
                    if 'easing' in payload:
                        self.keyframes[idx]['easing'] = payload['easing']
                    if 'bezier_p1' in payload:
                        self.keyframes[idx]['bezier_p1'] = payload['bezier_p1']
                    if 'bezier_p2' in payload:
                        self.keyframes[idx]['bezier_p2'] = payload['bezier_p2']
            case 'kf_delete':
                idx = payload.get('index', -1)
                if 0 <= idx < len(self.keyframes):
                    self.keyframes.pop(idx)
            case 'kf_clear_all':
                self.keyframes.clear()
            case 'kf_reorder':
                old_i = payload.get('from', 0)
                new_i = payload.get('to', 0)
                if 0 <= old_i < len(self.keyframes) and 0 <= new_i < len(self.keyframes):
                    kf = self.keyframes.pop(old_i)
                    self.keyframes.insert(new_i, kf)
            case 'kf_select':
                idx = payload.get('index', -1)
                if 0 <= idx < len(self.keyframes):
                    self.selected_kf = idx
                    kf = self.keyframes[idx]
                    self.body = dict(kf['body'])
                    self.legs = [list(l) for l in kf['legs']]
                    self._angle_overrides.clear()  # keyframe restore uses IK, not angle overrides
                    self._apply_pose()
                    
                    # Update play_time so the timeline playhead visually moves to the keyframe
                    accumulated = 0.0
                    for i in range(1, idx + 1):
                        accumulated += self.keyframes[i]['duration']
                    self.play_time = accumulated
            case 'kf_set_duration':
                idx = payload.get('index', -1)
                dur = payload.get('duration', 1.0)
                if 0 <= idx < len(self.keyframes):
                    self.keyframes[idx]['duration'] = max(0.05, dur)
            case 'kf_set_easing':
                idx = payload.get('index', -1)
                easing = payload.get('easing', 'ease-in-out')
                if 0 <= idx < len(self.keyframes):
                    self.keyframes[idx]['easing'] = easing
                    if 'bezier_p1' in payload:
                        self.keyframes[idx]['bezier_p1'] = payload['bezier_p1']
                    if 'bezier_p2' in payload:
                        self.keyframes[idx]['bezier_p2'] = payload['bezier_p2']
            case 'kf_set_trajectory':
                idx = payload.get('index', -1)
                if 0 <= idx < len(self.keyframes):
                    self.keyframes[idx]['trajectory'] = payload.get('trajectory', 'linear')
            case 'kf_set_arc_height':
                idx = payload.get('index', -1)
                if 0 <= idx < len(self.keyframes):
                    arc_h = float(payload.get('arc_height', 0.0))
                    self.keyframes[idx]['arc_height'] = arc_h
                    # Use bezier trajectory automatically when arc > 0
                    self.keyframes[idx]['trajectory'] = 'bezier' if arc_h > 0.001 else 'linear'

            # --- Playback ---
            case 'play':
                if len(self.keyframes) >= 2:
                    resume = payload.get('resume', False)
                    if not resume:
                        self.play_time = 0.0
                    self.playing = True
                    self.play_direction = 1
                    self.last_tick = time.monotonic()
                    self.play_mode = payload.get('mode', 'once')
                    self.play_speed = payload.get('speed', 1.0)
            case 'pause':
                self.playing = False
            case 'stop':
                self.playing = False
                self.play_time = 0.0
                if self.keyframes:
                    kf = self.keyframes[0]
                    self.body = dict(kf['body'])
                    self.legs = [list(l) for l in kf['legs']]
                    self._apply_pose()
            case 'seek':
                t = payload.get('time', 0.0)
                self.play_time = max(0.0, t)
                self._apply_playback_at(self.play_time)

            # --- Import ---
            case 'import':
                data = payload.get('keyframes', [])
                self.keyframes = data
                if self.keyframes:
                    kf = self.keyframes[0]
                    self.body = dict(kf['body'])
                    self.legs = [list(l) for l in kf['legs']]
                    self._angle_overrides.clear()
                    self._apply_pose()

    # ==================== Pose Application ====================

    def _apply_pose(self):
        """Apply current body + legs to spider and get joint angles."""
        angles = self.spider.apply_pose(self.body, self.legs)
        self.current_angles = np.array(angles).flatten().tolist()

        # 3. Restore any explicit angle overrides so that per-leg angle slider
        #    values are never corrupted by IK recalculation of other legs.
        for leg_idx, override_angles in self._angle_overrides.items():
            for j, a in enumerate(override_angles):
                self.current_angles[leg_idx * 3 + j] = a
            self.spider.legs[leg_idx].ths[:] = override_angles

    # ==================== Playback Engine ====================

    def _total_duration(self):
        if len(self.keyframes) < 2:
            return 0.0
        return sum(kf['duration'] for kf in self.keyframes[1:])

    def _apply_playback_at(self, t):
        if len(self.keyframes) < 2:
            return
        accumulated = 0.0
        for i in range(1, len(self.keyframes)):
            seg_dur = self.keyframes[i]['duration']
            if accumulated + seg_dur >= t or i == len(self.keyframes) - 1:
                local_t = (t - accumulated) / seg_dur if seg_dur > 0 else 1.0
                local_t = max(0.0, min(1.0, local_t))
                self._interpolate_keyframes(i - 1, i, local_t)
                return
            accumulated += seg_dur

    def _interpolate_keyframes(self, idx_a, idx_b, t):
        kf_a = self.keyframes[idx_a]
        kf_b = self.keyframes[idx_b]

        easing = kf_b.get('easing', 'ease-in-out')
        if easing == 'custom':
            p1 = kf_b.get('bezier_p1', [0.42, 0.0])
            p2 = kf_b.get('bezier_p2', [0.58, 1.0])
            et = ease_bezier(t, p1[0], p1[1], p2[0], p2[1])
        elif easing in EASING_MAP:
            et = EASING_MAP[easing](t)
        else:
            et = t

        for key in ['x', 'y', 'z', 'roll', 'pitch', 'yaw']:
            self.body[key] = kf_a['body'][key] * (1.0 - et) + kf_b['body'][key] * et

        arc_height = kf_b.get('arc_height', 0.0)

        for i in range(6):
            p0x, p0y, p0z = kf_a['legs'][i]
            p2x, p2y, p2z = kf_b['legs'][i]

            dist = math.hypot(p2x - p0x, p2y - p0y)

            self.legs[i][0] = p0x * (1.0 - et) + p2x * et
            self.legs[i][1] = p0y * (1.0 - et) + p2y * et

            # Backwards compatibility for trajectory: bezier
            traj = kf_b.get('trajectory', 'linear')
            effective_arc = arc_height
            if traj == 'bezier' and effective_arc < 0.001:
                effective_arc = 0.04

            if effective_arc > 0.001 and dist > 0.005:
                # Parabolic arc: z(t) = lerp + 4*H*et*(1-et)
                self.legs[i][2] = p0z * (1.0 - et) + p2z * et + 4.0 * effective_arc * et * (1.0 - et)
            else:
                self.legs[i][2] = p0z * (1.0 - et) + p2z * et

        self._apply_pose()

    def tick(self):
        """50Hz tick: publish joints + advance playback."""
        with self.lock:
            if self.playing:
                now = time.monotonic()
                if self.last_tick is not None:
                    dt = (now - self.last_tick) * self.play_speed * self.play_direction
                    self.play_time += dt
                self.last_tick = now

                total = self._total_duration()
                if total <= 0:
                    self.playing = False
                else:
                    if self.play_mode == 'once':
                        if self.play_time >= total:
                            self.play_time = total
                            self.playing = False
                    elif self.play_mode == 'loop':
                        if self.play_time >= total:
                            self.play_time = self.play_time % total
                    elif self.play_mode == 'pingpong':
                        if self.play_direction == 1 and self.play_time >= total:
                            self.play_time = total
                            self.play_direction = -1
                        elif self.play_direction == -1 and self.play_time <= 0:
                            self.play_time = 0.0
                            self.play_direction = 1

                    self._apply_playback_at(max(0.0, min(self.play_time, total)))

            self._publish_joints()
            self._publish_tf()

    def _publish_joints(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        for i, prefix in enumerate(PREFIXES):
            msg.name.extend([
                f'{prefix}_coxa_joint',
                f'{prefix}_coxa_link_to_{prefix}_femur_link',
                f'{prefix}_femur_link_to_{prefix}_tibia_link'
            ])
            msg.position.extend([
                self.current_angles[i * 3],
                self.current_angles[i * 3 + 1],
                self.current_angles[i * 3 + 2]
            ])
        self.joint_state_pub.publish(msg)

    def _publish_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'

        # Directly convert the IK Matrix to quaternion so we NEVER diverge on Euler conventions
        M = self.spider.T_sb

        # Compute translation for base_footprint.
        # IK (T_sb) rotates around base_link, which is +0.15z from base_footprint.
        # We must shift the translation -0.15m in the local Z axis to find base_footprint's origin.
        t.transform.translation.x = M[0,3] + M[0,2] * (-0.15)
        t.transform.translation.y = M[1,3] + M[1,2] * (-0.15)
        t.transform.translation.z = M[2,3] + M[2,2] * (-0.15)

        tr = M[0,0] + M[1,1] + M[2,2]
        if tr > 0:
            S = math.sqrt(tr + 1.0) * 2
            qw = 0.25 * S
            qx = (M[2,1] - M[1,2]) / S
            qy = (M[0,2] - M[2,0]) / S
            qz = (M[1,0] - M[0,1]) / S
        elif (M[0,0] > M[1,1]) and (M[0,0] > M[2,2]):
            S = math.sqrt(1.0 + M[0,0] - M[1,1] - M[2,2]) * 2
            qw = (M[2,1] - M[1,2]) / S
            qx = 0.25 * S
            qy = (M[0,1] + M[1,0]) / S
            qz = (M[0,2] + M[2,0]) / S
        elif M[1,1] > M[2,2]:
            S = math.sqrt(1.0 + M[1,1] - M[0,0] - M[2,2]) * 2
            qw = (M[0,2] - M[2,0]) / S
            qx = (M[0,1] + M[1,0]) / S
            qy = 0.25 * S
            qz = (M[1,2] + M[2,1]) / S
        else:
            S = math.sqrt(1.0 + M[2,2] - M[0,0] - M[1,1]) * 2
            qw = (M[1,0] - M[0,1]) / S
            qx = (M[0,2] + M[2,0]) / S
            qy = (M[1,2] + M[2,1]) / S
            qz = 0.25 * S

        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(t)

    # ==================== Onion Skinning ====================

    def publish_ghosts(self):
        ma = MarkerArray()

        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        delete_marker.header.frame_id = 'base_link'
        delete_marker.header.stamp = self.get_clock().now().to_msg()
        ma.markers.append(delete_marker)

        with self.lock:
            if len(self.keyframes) < 2:
                self.ghost_pub.publish(ma)
                return
            kf_snapshot = [dict(kf) for kf in self.keyframes]

        ghost_spider = Spider(0.02)
        for ki, kf in enumerate(kf_snapshot):
            ghost_spider.home()
            b = kf['body']
            ghost_spider.update_body_pos(
                x=b['x'], y=b['y'], z=b['z'],
                roll=b['roll'], pitch=b['pitch'], yaw=b['yaw']
            )
            positions = np.array(kf['legs'])
            ghost_spider.move_legs(positions, local=True)

            for li in range(6):
                foot = ghost_spider.legs[li].Ts_foot[:3, 3]
                m = Marker()
                m.header.frame_id = 'base_link'
                m.header.stamp = self.get_clock().now().to_msg()
                m.ns = f'ghost_kf{ki}'
                m.id = li
                m.type = Marker.SPHERE
                m.action = Marker.ADD
                m.pose.position = Point(
                    x=float(foot[0]), y=float(foot[1]), z=float(foot[2]))
                m.pose.orientation.w = 1.0
                m.scale.x = 0.008
                m.scale.y = 0.008
                m.scale.z = 0.008

                hue = (ki / max(len(kf_snapshot), 1)) * 0.8
                r, g, b_c = _hsv_to_rgb(hue, 0.8, 1.0)
                m.color = ColorRGBA(r=r, g=g, b=b_c, a=0.5)
                m.lifetime.sec = 1
                ma.markers.append(m)

        self.ghost_pub.publish(ma)

    def destroy_node(self):
        self.http.shutdown()
        super().destroy_node()


def _hsv_to_rgb(h, s, v):
    import colorsys
    r, g, b = colorsys.hsv_to_rgb(h, s, v)
    return float(r), float(g), float(b)


def main(args=None):
    rclpy.init(args=args)
    node = AnimationEditorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Animation Editor...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
