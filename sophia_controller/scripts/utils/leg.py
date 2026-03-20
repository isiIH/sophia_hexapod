from utils.tf_matrix import T
import math
import numpy as np

class Leg:
    def __init__(self, dims, xyz_rpy):
        x, y, z, r, p, y = xyz_rpy
        self.T_coxa = T(x, y,z, r, p, y)

        self.hip_length = dims[0]
        self.femur_length = dims[1]
        self.tibia_length = dims[2]

        self.total_length = self.hip_length + self.femur_length + self.tibia_length

        self.p_foot_default = np.array([self.total_length, 0, 0, 1])

        self.s_foot = self.T_coxa @ self.p_foot_default

        self.ths = np.zeros(len(dims))

    def leg_ik(self, pos):
        x = pos[0]
        y = pos[1]
        z = pos[2]

        l = (x**2 + y**2) ** (1/2)

        l_left = l - self.hip_length

        hf = (l_left**2 + z**2) ** (1/2)

        a1 = math.atan2(l_left, abs(z))
        arg_a2 = (self.femur_length**2 + hf**2 - self.tibia_length**2) / (2 * self.femur_length * hf)
        a2 = math.acos(np.clip(arg_a2, -1.0, 1.0))

        arg_b1 = (self.femur_length**2 + self.tibia_length**2 - hf**2) / (2 * self.femur_length * self.femur_length)
        b1 = math.acos(np.clip(arg_b1, -1.0, 1.0))

        self.ths[0] = math.atan2(y, x)
        self.ths[1] = np.pi/2 - (a1 + a2)
        self.ths[2] = np.pi - b1

        return self.ths
