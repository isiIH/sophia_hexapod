from utils.tf_matrix import T
import utils.config as cf
import math
import numpy as np

class Leg:
    def __init__(self, x, y, z, roll, pitch, yaw):
        self.total_length = cf.HIP_LENGTH + cf.FEMUR_LENGTH + cf.TIBIA_LENGTH

        self.T_coxa = T(x, y, z, roll, pitch, yaw)

        self.p_foot_default = np.array([self.total_length, 0, 0, 1])

        self.s_foot = self.T_coxa @ self.p_foot_default

        self.ths = np.zeros(cf.N_JOINTS)

    def leg_ik(self, pos):
        x = pos[0]
        y = pos[1]
        z = pos[2]

        l = (x**2 + y**2) ** (1/2)

        l_left = l - cf.HIP_LENGTH

        hf = (l_left**2 + z**2) ** (1/2)

        a1 = math.atan2(l_left, abs(z))
        arg_a2 = (cf.FEMUR_LENGTH**2 + hf**2 - cf.TIBIA_LENGTH**2) / (2 * cf.FEMUR_LENGTH * hf)
        a2 = math.acos(np.clip(arg_a2, -1.0, 1.0))

        arg_b1 = (cf.FEMUR_LENGTH**2 + cf.TIBIA_LENGTH**2 - hf**2) / (2 * cf.FEMUR_LENGTH * cf.TIBIA_LENGTH)
        b1 = math.acos(np.clip(arg_b1, -1.0, 1.0))

        self.ths[0] = math.atan2(y, x)
        self.ths[1] = np.pi/2 - (a1 + a2)
        self.ths[2] = np.pi - b1

        return self.ths

