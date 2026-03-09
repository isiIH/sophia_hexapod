import numpy as np
from utils.bezier_curve import BezierCurve
from utils.spider import Spider

TRIPOD_GAIT = np.array([1, 0, 1, 0, 1, 0], dtype=bool)

class GaitGenerator:

    def __init__(self, spider, type="tripod"):
        self.spider : Spider = spider
        self.targets : bool = self.get_targets(type)
        self.step_phase = True # switch between groups

        self.stand_positions = [leg.s_foot[:3].copy() for leg in self.spider.legs]

        self.swings = [None] * 6 # Bezier curves for each leg
        self.stances = [None] * 6 # Linear interpolation (ground trajectory)
              
    def get_targets(self, type):
       match type:
          case 'tripod': return TRIPOD_GAIT
          case _: return TRIPOD_GAIT

    def set_trajectory(self, step_vector, height, first_step=False):
        for i, foot_point in enumerate(self.stand_positions):
            is_swing = (self.targets[i] == self.step_phase)

            if first_step:
                if is_swing:
                    start_point = foot_point.copy()
                    end_point = foot_point + (step_vector / 2.0)
                else:
                    start_point = foot_point.copy()
                    end_point = foot_point - (step_vector / 2.0)
            else:
                if is_swing:
                    start_point = foot_point - (step_vector / 2.0)
                    end_point = foot_point + (step_vector / 2.0)
                else:
                    start_point = foot_point + (step_vector / 2.0)
                    end_point = foot_point - (step_vector / 2.0)

            self.swings[i] = BezierCurve(start_point, end_point, height)
            self.stances[i] = lambda t, s=start_point, e=end_point: s + t * (e - s)

    def get_next_step(self, t):
        positions = []

        for i,target in enumerate(self.targets):
           next_pos = self.swings[i].get_point(t) if target == self.step_phase else self.stances[i](t)
           positions.append(next_pos)

        if t >= 1.0:
            self.step_phase = not self.step_phase

        return positions

    