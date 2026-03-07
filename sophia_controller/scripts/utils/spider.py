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

        angles = np.array([])
        for leg in self.legs:
            T_sc_i = self.T_sb @ leg.T_coxa

            pos_cf = np.linalg.inv(T_sc_i) @ leg.s_foot

            angles = np.append(angles, leg.leg_ik(pos_cf))
        
        return angles

    def move_legs(self, positions):
        angles = np.array([])
        for i, leg_pos in enumerate(positions):
            self.legs[i].s_foot = self.T_sb @ self.legs[i].T_coxa @ np.array([leg_pos[0], leg_pos[1], leg_pos[2], 1])
            angles = np.append(angles, self.legs[i].leg_ik(leg_pos))

        return angles
    
    def home(self):
        return self.move_legs([self.legs[0].p_foot_default] * 6)


    