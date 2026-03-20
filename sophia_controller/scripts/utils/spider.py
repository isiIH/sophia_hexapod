from utils.tf_matrix import T
import numpy as np
from utils.leg import Leg

import xacro
import os
from urdf_parser_py.urdf import URDF
from ament_index_python.packages import get_package_share_directory

class Spider:
    def __init__(self):
        self.legs = []
        self.T_sb = T()
        self.read_config_robot()

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

    def update_body_pos(self, x, y, z, roll, pitch, yaw):
        self.T_sb = T(x, y, z, roll, pitch, yaw)

        angles = []
        for leg in self.legs:
            pos_cf = self.get_local_pos(leg, leg.s_foot)

            angles.append(leg.leg_ik(pos_cf))
        
        return angles

    def move_legs(self, positions, targets=[1]*6, local=False):
        angles = []
        for i, leg_pos in enumerate(positions):
            if targets[i] == 0: continue

            if local:
                self.legs[i].s_foot = self.T_sb @ self.legs[i].T_coxa @ np.array([leg_pos[0], leg_pos[1], leg_pos[2], 1])
                pos_cf = leg_pos
            else:
                pos_s = np.array([leg_pos[0], leg_pos[1], leg_pos[2], 1])
                self.legs[i].s_foot = np.array(pos_s)
                pos_cf = self.get_local_pos(self.legs[i], pos_s)

            
            angles.append(self.legs[i].leg_ik(pos_cf))

        return np.array(angles).flatten().tolist()
    
    def get_local_pos(self, leg, position):
        T_sc_i = self.T_sb @ leg.T_coxa

        pos_cf = np.linalg.inv(T_sc_i) @ position

        return pos_cf[:3]
    
    def get_leg_positions(self):
        return np.array([leg.s_foot[:3].copy() for leg in self.legs])
    
    def home(self):
        return self.move_legs([self.legs[0].p_foot_default] * 6, [1] * 6)