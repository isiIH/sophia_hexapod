import numpy as np
from utils.bezier_curve import BezierCurve

class GaitGenerator:

    def __init__(self, leg_pos, type="tripod", time_step=0.02):
        self.set_gait_type(type)

        self.stand_positions = np.copy(leg_pos)
        self.current_leg_positions = np.copy(leg_pos)

        # Start point phase
        self.was_swing = [False] * 6
        self.phase_start_pos = np.copy(leg_pos)

        # Cycle parameters
        self.t = 0.0
        self.time_step = time_step
        self.cycle_duration = 0.7
        self.t_inc = self.time_step / self.cycle_duration
        self.step_factor = 0.05
        self.step_vector = np.zeros(3, dtype=np.float32)
        self.is_moving = False
        self.linear_speed = np.zeros(3, dtype=np.float32)
        # self.step_vector = np.array([0.0, 0.15, 0.0])
        # self.is_moving = True
        # self.linear_speed = np.array([0.0, 0.3, 0.0])

        # Bezier params
        self.height = 0.1
        self.swings = [BezierCurve(np.zeros(3), np.zeros(3), self.height) for _ in range(6)] # Bezier curves for each leg
        self.stances = [None] * 6 # Linear interpolation (ground trajectory)
              
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
        self.linear_speed = linear_speed

    def set_trajectory(self):
        for i, home_leg in enumerate(self.stand_positions):
            is_swing = (self.t - self.offsets[i]) % 1.0 < self.swing_time

            if is_swing and not self.was_swing[i]:
                self.phase_start_pos[i] = self.current_leg_positions[i].copy()

            elif not is_swing and self.was_swing[i]:
                self.phase_start_pos[i] = self.current_leg_positions[i].copy()

            self.was_swing[i] = is_swing

            start_home = home_leg - (self.step_vector / 2.0)
            end_home = home_leg + (self.step_vector / 2.0)

            if is_swing:
                self.swings[i].update(self.phase_start_pos[i], end_home)
            else:
                self.stances[i] = lambda t, s=self.phase_start_pos[i], e=start_home: s + (e - s) * t

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
        mag = np.linalg.norm(self.linear_speed)

        if(mag < 1e-3):
            self.step_vector = np.zeros(3, dtype=np.float32)
            self.t_inc = self.time_step

            if self.t == 0.0:
                self.is_moving = False
                self.t_inc = 0.0
        else:
            self.is_moving = True

            dir_speed = self.linear_speed / mag
            self.step_vector =  dir_speed * self.step_factor

            self.cycle_duration = max(self.step_factor / mag, 1.0)
            self.t_inc = self.time_step / self.cycle_duration

    