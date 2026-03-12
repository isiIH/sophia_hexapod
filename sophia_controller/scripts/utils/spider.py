from utils.tf_matrix import T
import numpy as np
from utils.leg import Leg

class Spider:
    def __init__(self):
        self.T_sb = T(0, 0, 0, 0, 0, 0)
        self.legs : np.ndarray[Leg] = np.array([
            Leg(0.06, 0.08, 0, 0, 0, np.pi/4), #rf
            Leg(0.06, 0, 0, 0, 0, 0), #rm
            Leg(0.06, -0.08, 0, 0, 0, -np.pi/4), #rb
            Leg(-0.06, 0.08, 0, 0, 0, 3*np.pi/4), #lf
            Leg(-0.06, 0, 0, 0, 0, np.pi), #rm
            Leg(-0.06, -0.08, 0, 0, 0, 5*np.pi/4) #rb
        ])

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


    