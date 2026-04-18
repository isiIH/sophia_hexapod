from utils.tf_matrix import T
import numpy as np
from utils.leg import Leg

import xacro
import os
from urdf_parser_py.urdf import URDF
from ament_index_python.packages import get_package_share_directory

from utils.gait_generator import GaitGenerator
from utils.animation_player import AnimationPlayer

KEYFRAMES = [
    {  # Keyframe 0
        'body': {'x': 0, 'y': 0, 'z': 0, 'roll': 0, 'pitch': 0, 'yaw': 0},
        'legs': [
            [0.085, 0, -0.0627],  # RF
            [0.085, 0, -0.0627],  # RM
            [0.085, 0, -0.0627],  # RB
            [0.085, 0, -0.0627],  # LF
            [0.085, 0, -0.0627],  # LM
            [0.085, 0, -0.0627],  # LB
        ],
        'duration': 0.5,
        'easing': 'ease-in-out',
    },
    {  # Keyframe 1
        'body': {'x': 0, 'y': 0, 'z': 0, 'roll': 0, 'pitch': 0, 'yaw': 0},
        'legs': [
            [0.10561287858115681, 0, 0.10462138496267549],  # RF
            [0.085, 0, -0.0627],  # RM
            [0.085, 0, -0.0627],  # RB
            [0.085, 0, -0.0627],  # LF
            [0.085, 0, -0.0627],  # LM
            [0.085, 0, -0.0627],  # LB
        ],
        'duration': 1,
        'easing': 'ease-in-out',
    },
    {  # Keyframe 2
        'body': {'x': 0, 'y': 0, 'z': 0, 'roll': 0, 'pitch': 0, 'yaw': 0},
        'legs': [
            [0.12609066257374163, 0, 0.003969860990764818],  # RF
            [0.085, 0, -0.0627],  # RM
            [0.085, 0, -0.0627],  # RB
            [0.085, 0, -0.0627],  # LF
            [0.085, 0, -0.0627],  # LM
            [0.085, 0, -0.0627],  # LB
        ],
        'duration': 0.5,
        'easing': 'ease-in-out',
    },
    {  # Keyframe 3
        'body': {'x': 0, 'y': 0, 'z': 0, 'roll': 0, 'pitch': 0, 'yaw': 0},
        'legs': [
            [0.10561916662559263, 0, 0.10459047823685506],  # RF
            [0.085, 0, -0.0627],  # RM
            [0.085, 0, -0.0627],  # RB
            [0.085, 0, -0.0627],  # LF
            [0.085, 0, -0.0627],  # LM
            [0.085, 0, -0.0627],  # LB
        ],
        'duration': 0.5,
        'easing': 'ease-in-out',
    },
    {  # Keyframe 4
        'body': {'x': 0, 'y': 0, 'z': 0, 'roll': 0, 'pitch': 0, 'yaw': 0},
        'legs': [
            [0.085, 0, -0.0627],  # RF
            [0.085, 0, -0.0627],  # RM
            [0.085, 0, -0.0627],  # RB
            [0.085, 0, -0.0627],  # LF
            [0.085, 0, -0.0627],  # LM
            [0.085, 0, -0.0627],  # LB
        ],
        'duration': 1,
        'easing': 'ease-in-out',
    },
]

class Spider:
    def __init__(self):
        self.legs = []
        self.T_sb = T()
        self.read_config_robot()

        self.home_positions = np.array([[0.085, 0, -0.0627]] * 6)
        self.home()
        home_s_foot = self.get_leg_positions()

        self.gait = GaitGenerator(home_s_foot)

        self.height = 0.0

        # Animation
        self.animation_player = AnimationPlayer(KEYFRAMES)

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
    
    def get_leg_positions(self):
        return np.array([leg.Ts_foot[:3, 3].copy() for leg in self.legs])

    def get_joint_angles(self):
        return np.array([leg.ths.copy() for leg in self.legs]).flatten().tolist()

    def set_height(self, mul):
        self.height += mul * 0.005
        self.height = max(-0.05, min(self.height, 0.05))

        self.update_body_pos(z = self.height)

    def play_animation(self):
        self.animation_player.play()

    def is_animation_playing(self):
        return self.animation_player.is_playing()
    
    def home(self):
        self.move_legs(self.home_positions, local=True)