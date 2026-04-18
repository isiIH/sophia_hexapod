from utils.tf_matrix import T
import math
import numpy as np

class Leg:
    def __init__(self, dims, xyz_rpy):
        x, y, z, r, p, yaw = xyz_rpy
        self.T_coxa = T(x, y, z, r, p, yaw)

        self.hip_length = dims[0]
        self.femur_length = dims[1]
        self.tibia_length = dims[2]

        self.total_length = self.hip_length + self.femur_length + self.tibia_length

        self.Ts_foot = self.T_coxa @ T(x=self.total_length)

        self.ths = np.zeros(len(dims))

    def get_local_pos(self, T_sb):
        Ts_c = T_sb @ self.T_coxa
        Tc_f = np.linalg.inv(Ts_c) @ self.Ts_foot
        return Tc_f[:3, 3]

    def leg_ik(self, pos):
        x = pos[0]
        y = pos[1]
        z = pos[2]

        l_sign = 1.0 if x >= 0 else -1.0
        l_mag = (x**2 + y**2) ** (1/2)
        l = l_sign * l_mag

        l_left = l - self.hip_length

        hf = (l_left**2 + z**2) ** (1/2)

        a1 = math.atan2(l_left, -z)
        arg_a2 = (self.femur_length**2 + hf**2 - self.tibia_length**2) / (2 * self.femur_length * hf)
        a2 = math.acos(np.clip(arg_a2, -1.0, 1.0))

        arg_b1 = (self.femur_length**2 + self.tibia_length**2 - hf**2) / (2 * self.femur_length * self.tibia_length)
        b1 = math.acos(np.clip(arg_b1, -1.0, 1.0))

        # Determine coxa angle (flip quadrant if leg reaches backwards to prevent 180° violent snaps)
        if x >= 0:
            self.ths[0] = math.atan2(y, x)
        else:
            self.ths[0] = math.atan2(-y, -x)

        # Dual Solution IK (knee inwards vs outwards)
        angle_down = np.pi - b1
        angle_up = b1 - np.pi

        # Pick the geometric solution closest to the leg's current physical joint state
        if abs(self.ths[2] - angle_down) <= abs(self.ths[2] - angle_up):
            self.ths[1] = np.pi/2 - (a1 + a2)
            self.ths[2] = angle_down
        else:
            self.ths[1] = np.pi/2 - (a1 - a2)
            self.ths[2] = angle_up

        return self.ths

    def leg_fk(self, ths):
        """Forward kinematics returning local [x, y, z] from [th0, th1, th2]."""
        self.ths = np.array(ths)
        T0 = T(yaw=ths[0])
        T1 = T0 @ T(x=self.hip_length) @ T(pitch=ths[1])
        T2 = T1 @ T(x=self.femur_length) @ T(pitch=ths[2])
        T_foot = T2 @ T(x=self.tibia_length)
        return T_foot[:3, 3]
