from utils.tf_matrix import T
import numpy as np
from utils.leg import Leg

import xacro
import os
from urdf_parser_py.urdf import URDF
from ament_index_python.packages import get_package_share_directory

from utils.gait_generator import GaitGenerator
from utils.animation_player import AnimationLoader

class Spider:
    def __init__(self, time_step):
        self.legs = []
        self.T_sb = T()
        self.read_config_robot()

        self.home_positions = np.array([[0.085, 0, -0.0627]] * 6)
        self.home()
        home_s_foot = self.get_leg_positions()

        self.time_step = time_step

        self.gait = GaitGenerator(home_s_foot, time_step=time_step)

        self.height = 0.0

        # Animation
        self.animation_player = None
        self.animation_loader = AnimationLoader("animations")

    def read_config_robot(self):
        pkg_path = get_package_share_directory('sophia_description')
        xacro_file = os.path.join(pkg_path, 'urdf', 'mech', 'sophia_hexapod.urdf.xacro')

        doc = xacro.process_file(xacro_file)
        robot_urdf = URDF.from_xml_string(doc.toxml())

        # Get the length of the links
        hip_length = float(robot_urdf.link_map['rf_coxa_link'].visual.geometry.size[0])
        femur_length = float(robot_urdf.link_map['rf_femur_link'].visual.geometry.size[0])
        tibia_length = float(robot_urdf.link_map['rf_tibia_link'].visual.geometry.size[0])

        dims = [hip_length, femur_length, tibia_length]

        # Get the origin of each leg
        prefixes = ['rf_', 'rm_', 'rb_', 'lf_', 'lm_', 'lb_']
        for p in prefixes:
            joint_name = f"{p}fixed_base_joint"
            xyz = robot_urdf.joint_map[joint_name].origin.xyz
            rpy = robot_urdf.joint_map[joint_name].origin.rpy

            self.legs.append(Leg(dims, xyz_rpy= xyz + rpy))

    def update_body_pos(self, x=0.0, y=0.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0):
        self.T_sb = T(x, y, z, roll, pitch, yaw)

        angles = []
        for leg in self.legs:
            pos_cf = leg.get_local_pos(self.T_sb)
            angles.append(leg.leg_ik(pos_cf))
        
        return angles

    def move_legs(self, positions, targets=[1]*6, local=False):
        angles = []
        for i, leg_pos in enumerate(positions):
            if targets[i] == 0: continue

            if local:
                Tc_f = np.eye(4)
                Tc_f[:3, 3] = leg_pos
                self.legs[i].Ts_foot = self.T_sb @ self.legs[i].T_coxa @ Tc_f
                pos_cf = leg_pos
            else:
                self.legs[i].Ts_foot[:3, 3] = leg_pos
                pos_cf = self.legs[i].get_local_pos(self.T_sb)
      
            angles.append(self.legs[i].leg_ik(pos_cf))

        return np.array(angles).flatten().tolist()
    
    def apply_pose(self, body, legs):
        """
        Anchor feet to the given leg targets (relative to coxa),
        then update the body pose to calculate full IK.
        """
        for i in range(6):
            Tc_f = np.eye(4)
            Tc_f[:3, 3] = legs[i]
            self.legs[i].Ts_foot = self.legs[i].T_coxa @ Tc_f

        angles = self.update_body_pos(**body)
        return np.array(angles).flatten().tolist()

    def get_next_frame(self):
        if self.is_animation_playing():
            self.gait.stop()

            frame = self.animation_player.get_frame(self.time_step)
            if frame:
                body, legs = frame
                joint_angles = self.apply_pose(body, legs)
                leg_positions = self.get_leg_positions()
        else:
            leg_positions = self.gait.get_next_step()
            joint_angles = self.move_legs(leg_positions)

        return joint_angles, leg_positions.flatten().tolist()
    
    def get_leg_positions(self):
        return np.array([leg.Ts_foot[:3, 3].copy() for leg in self.legs])

    def get_joint_angles(self):
        return np.array([leg.ths.copy() for leg in self.legs]).flatten().tolist()

    def set_height(self, mul):
        self.height += mul * 0.005
        self.height = max(-0.05, min(self.height, 0.05))

        self.update_body_pos(z = self.height)

    def play_animation(self, animation_name):
        if self.is_animation_playing():
            return

        self.animation_player = self.animation_loader.get_animation(animation_name)
        if self.animation_player:
            self.animation_player.play()

    def is_animation_playing(self):
        return self.animation_player.is_playing() if self.animation_player else False
    
    def home(self):
        self.move_legs(self.home_positions, local=True)
