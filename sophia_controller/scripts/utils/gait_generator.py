import numpy as np
from utils.bezier_curve import BezierCurve
from utils.tf_matrix import T

class GaitGenerator:

    def __init__(self, home_pos, type="tripod", time_step=0.02):
        self.set_gait_type(type)

        # Start point phase
        self.was_swing = [False] * 6
        self.home_positions = np.copy(home_pos)
        self.current_leg_positions = np.copy(home_pos)
        self.phase_start_pos = np.copy(home_pos)

        # Cycle parameters
        self.t = 0.0
        self.time_step = time_step
        self.cycle_duration = 1.0
        self.t_inc = self.time_step / self.cycle_duration
        self.step_factor = 0.05
        self.step_vector = np.zeros(3, dtype=np.float32)
        self.is_moving = False
        self.linear_speed = np.zeros(3, dtype=np.float32)
        self.angular_speed = 0.0

        self.cmd_linear_speed = np.zeros(3, dtype=np.float32)
        self.cmd_angular_speed = 0.0

        # Bezier params
        self.height = 0.1
        self.swings = [BezierCurve(np.copy(pos), np.copy(pos), self.height) for pos in home_pos] 
        self.stances = [None] * 6 
              
    def set_gait_type(self, type):
        match type:
            case 'tripod': 
                self.offsets = np.array([0, 0.5, 0, 0.5, 0, 0.5])
                self.swing_time = 0.5
            case 'tetrapod':
                self.offsets = np.array([0.66, 0.33, 0, 0.33, 0, 0.66])
                self.swing_time = 0.33
            case 'ripple':
                self.offsets = np.array([0.16, 0.83, 0.5, 0.66, 0.33, 0])
                self.swing_time = 0.33
            case 'wave':
                self.offsets = np.array([0, 0.16, 0.33, 0.5, 0.66, 0.83])
                self.swing_time = 0.16
            case 'bi':
                self.offsets = np.array([0.66, 0.33, 0, 0, 0.33, 0.66])
                self.swing_time = 0.66
            case _: # Tripod gait
                self.offsets = np.array([0, 0.5, 0, 0.5, 0, 0.5])
                self.swing_time = 0.5
        
        self.stance_time = 1 - self.swing_time

    def set_linear_speed(self, linear_speed):
        self.cmd_linear_speed = linear_speed

    def set_angular_speed(self, angular_speed):
        self.cmd_angular_speed = angular_speed

    def set_trajectory(self):
        T_forward = T(
            x = self.step_vector[0] / 2.0, 
            y = self.step_vector[1] / 2.0, 
            yaw = self.angular_speed / 2.0
        )

        T_backward = T(
            x = -self.step_vector[0] / 2.0, 
            y = -self.step_vector[1] / 2.0, 
            yaw = -self.angular_speed / 2.0
        )

        for i, home_leg in enumerate(self.home_positions):
            t_local = (self.t - self.offsets[i]) % 1.0
            is_swing = t_local < self.swing_time

            # Target endpoints based on step_vector and rotation angle
            # Stance: body moves forward -> leg moves from +step/2 to -step/2 relative to home
            # Swing: leg reaches forward -> move to +step/2 relative to home
            home_leg_homo = np.array([home_leg[0], home_leg[1], home_leg[2], 1.0])
            start_pos_rel = (T_forward @ home_leg_homo)[:3]
            end_pos_rel = (T_backward @ home_leg_homo)[:3]

            # Update phase start position only on transition
            if is_swing != self.was_swing[i]:
                if not is_swing:
                    # Leg finishes swing -> snap perfectly to the exact ground target
                    # This prevents the leg from 'floating' into stance due to discrete time steps
                    self.phase_start_pos[i] = np.copy(start_pos_rel)
                else:
                    # Leg lifts into swing
                    self.phase_start_pos[i] = np.copy(end_pos_rel)
                self.was_swing[i] = is_swing

            if is_swing:
                # Swing targets the "reach" point (+step/2)
                self.swings[i].update(self.phase_start_pos[i], start_pos_rel)
            else:
                # Stance targets the "push" point (-step/2)
                # s: current phase start, e: final push point
                s = np.copy(self.phase_start_pos[i])
                e = np.copy(end_pos_rel)
                self.stances[i] = lambda t, s=s, e=e: s + (e - s) * t

    def get_next_step(self):
        self.adjust_cycle()

        if not self.is_moving and self.t == 0.0:
            return self.current_leg_positions
        
        self.set_trajectory()

        for i, offset in enumerate(self.offsets):
            t_local = (self.t - offset) % 1.0

            if t_local < self.swing_time:
                t_swing = t_local / self.swing_time
                next_pos = self.swings[i].get_point(t_swing)
            else:
                t_stance = (t_local - self.swing_time) / (self.stance_time)
                next_pos = self.stances[i](t_stance)
            
            self.current_leg_positions[i] = next_pos

        if self.t == 1.0:
            self.t = 0.0
        else:
            self.t = min(1.0, round(self.t + self.t_inc, 4))

        return self.current_leg_positions
            
    def adjust_cycle(self):
        # Apply low-pass filter for smooth transitions (alpha = 0.1 at 50Hz = ~0.2s time constant)
        alpha = 0.1
        self.linear_speed = self.linear_speed * (1.0 - alpha) + self.cmd_linear_speed * alpha
        self.angular_speed = self.angular_speed * (1.0 - alpha) + self.cmd_angular_speed * alpha

        mag = np.linalg.norm(self.linear_speed)
        mag_angular = abs(self.angular_speed)

        if mag < 1e-3 and mag_angular < 1e-3:
            self.linear_speed = np.zeros(3, dtype=np.float32)
            self.angular_speed = 0.0
            self.step_vector = np.zeros(3, dtype=np.float32)
            self.t_inc = self.time_step

            if self.t == 0.0:
                self.is_moving = False
                self.t_inc = 0.0
        else:
            self.is_moving = True

            if mag >= 1e-3:
                dir_speed = self.linear_speed / mag
                self.step_vector =  dir_speed * self.step_factor
            else:
                self.step_vector = np.zeros(3, dtype=np.float32)

            self.t_inc = self.time_step / self.cycle_duration