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
        self.phase_start_stance_pos = np.copy(home_pos)

        # Cycle parameters
        self.t = 0.0
        self.time_step = time_step
        self.cycle_duration = 1.0
        self.t_inc = self.time_step
        self.is_moving = False
        self.linear_speed = np.zeros(3, dtype=np.float32)
        self.angular_speed = 0.0

        self.cmd_linear_speed = np.zeros(3, dtype=np.float32)
        self.cmd_angular_speed = 0.0

        # Bezier swing height.
        # CRITICAL: must be ≤ 0.03m.
        # At 0.05m the bezier midpoint falls inside the inner workspace dead-zone
        # (hf < |femur - tibia| = 0.0375m) for legs whose swing target is near x_local≈COXA_L.
        # This causes leg_ik() to return tibia=π (fully-extended singularity) → joint
        # hammers its stop → asymmetric reaction force → yaw kick every step cycle.
        # At 0.03m all midpoints stay outside the dead-zone: hf_min ≈ 0.047m > 0.0375m.
        self.height = 0.03
        self.swings = [BezierCurve(np.copy(pos), np.copy(pos), self.height) for pos in home_pos] 
              
    def _get_gait_params(self, type):
        match type:
            case 'tripod': 
                return np.array([0, 0.5, 0, 0.5, 0, 0.5]), 0.5
            case 'tetrapod':
                return np.array([0.66, 0.33, 0, 0.33, 0, 0.66]), 0.33
            case 'ripple':
                return np.array([0.16, 0.83, 0.5, 0.66, 0.33, 0]), 0.33
            case 'wave':
                return np.array([0, 0.16, 0.33, 0.5, 0.66, 0.83]), 0.16
            case 'bi':
                return np.array([0.66, 0.33, 0, 0, 0.33, 0.66]), 0.66
            case _: # Tripod gait
                return np.array([0, 0.5, 0, 0.5, 0, 0.5]), 0.5

    def set_gait_type(self, type):
        offsets, swing_time = self._get_gait_params(type)
        if not hasattr(self, 'offsets'):
            self.offsets = np.copy(offsets)
            self.swing_time = swing_time
        
        # We smoothly transition towards the targets to prevent foot jumping
        self.target_offsets = np.copy(offsets)
        self.target_swing_time = swing_time
        self.stance_time = 1 - self.swing_time

    def set_linear_speed(self, linear_speed):
        self.cmd_linear_speed = linear_speed

    def set_angular_speed(self, angular_speed):
        self.cmd_angular_speed = angular_speed

    def set_trajectory(self):
        yaw_half = self.angular_speed / 2.0
        R_forward = T(yaw=yaw_half)[:3, :3]
        R_backward = T(yaw=-yaw_half)[:3, :3]
        
        trans_vec = np.array([
            self.linear_speed[0] / 2.0, 
            self.linear_speed[1] / 2.0, 
            0.0
        ])

        self.current_start_poses = []
        self.current_end_poses = []

        for i, home_leg in enumerate(self.home_positions):
            t_local = (self.t - self.offsets[i]) % 1.0
            is_swing = t_local < self.swing_time

            start_pos_rel = R_forward @ home_leg + trans_vec
            end_pos_rel = R_backward @ home_leg - trans_vec

            self.current_start_poses.append(start_pos_rel)
            self.current_end_poses.append(end_pos_rel)

            if is_swing != self.was_swing[i]:
                if not is_swing:
                    # Leg finishes swing -> just switch state and lock landing position
                    self.phase_start_stance_pos[i] = np.copy(self.current_leg_positions[i])
                else:
                    # Leg lifts into swing: LOCK the start point exactly where it left the ground
                    self.phase_start_pos[i] = np.copy(self.current_leg_positions[i])
                self.was_swing[i] = is_swing

            if is_swing:
                # Swing targets the "reach" point (+step/2), dynamically updating target
                self.swings[i].update(self.phase_start_pos[i], start_pos_rel)

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
                # Stance uses exact interpolation to the target endpoint.
                # Because current targets are low-pass filtered, no teleportation occurs 
                # but this completely prevents the legs from drifting out of bounds during a turn!
                t_stance = (t_local - self.swing_time) / self.stance_time
                next_pos = (1.0 - t_stance) * self.phase_start_stance_pos[i] + t_stance * self.current_end_poses[i]
            
            self.current_leg_positions[i] = np.copy(next_pos)

        if self.t == 1.0:
            self.t = 0.0
        else:
            self.t = min(1.0, round(self.t + self.t_inc, 4))

        return self.current_leg_positions
            
    def adjust_cycle(self):
        # Smoothly interpolate into target gait phase offsets
        diff = self.target_offsets - self.offsets
        diff = (diff + 0.5) % 1.0 - 0.5 # keep shortest path wrapping
        max_shift = self.t_inc * 0.5 # bound rate of phase shift
        shift = np.clip(diff * 0.1, -max_shift, max_shift)
        self.offsets = (self.offsets + shift) % 1.0
        
        self.swing_time += (self.target_swing_time - self.swing_time) * 0.05
        self.stance_time = 1 - self.swing_time

        # self.linear_speed = self.cmd_linear_speed
        # self.angular_speed = self.cmd_angular_speed

        # Extract strictly analog target velocities bounded by physical step max limits
        target_linear = self.cmd_linear_speed
        
        # Boost angular speed when doing pure rotation (no/low linear speed).
        # At full linear speed: boost = 1.0 (no change)
        # At zero linear speed: boost = 3.0 (3× wider turn steps)
        linear_mag_cmd = np.linalg.norm(self.cmd_linear_speed)
        linear_ratio = np.clip(linear_mag_cmd / 0.1, 0.0, 1.0)  # 0.1 = max linear speed
        angular_boost = 3.0 - 2.0 * linear_ratio  # 3.0 → 1.0
        target_angular = self.cmd_angular_speed * angular_boost

        # Apply low-pass filter for smooth transitions (alpha = 0.1 at 50Hz = ~0.2s time constant)
        alpha = 0.1
        self.linear_speed = self.linear_speed * (1.0 - alpha) + target_linear * alpha
        self.angular_speed = self.angular_speed * (1.0 - alpha) + target_angular * alpha

        mag = np.linalg.norm(self.linear_speed)
        mag_angular = abs(self.angular_speed)

        # Dynamically scale the cycle duration based on movement velocity
        # Higher velocity commands = shorter cycle time (faster stepping rate)
        # We account for the 3.0x angular boost here to map cleanly to 0.0-1.0
        speed_factor = max(mag / 0.1, mag_angular / (np.pi/36 * 3.0))
        speed_factor = np.clip(speed_factor, 0.0, 1.0)
        
        # Cycle duration range: 1.2s (slow) → 0.8s (fast).
        # Do NOT go below 0.8s: the Bullet contact solver needs enough steps per
        # gait cycle to converge symmetrically, or yaw drift reappears.
        self.cycle_duration = 1.2 - (0.4 * speed_factor)

        # Handle stop conditions / Return to stance exactly cleanly
        if mag < 1e-4 and mag_angular < 1e-4:
            self.t_inc = self.time_step

            # Only stop fully when all legs drop cleanly and smoothly to home frame 0.0!
            if self.t == 0.0:
                self.is_moving = False
                self.t_inc = 0.0
                
                # Snapping accurately to true home preventing 1e-5 floating point drift over hours of idling
                self.current_leg_positions = np.copy(self.home_positions)
        else:
            self.is_moving = True
            # Standard ticking speed
            self.t_inc = self.time_step / self.cycle_duration

    def stop(self):
        if self.is_moving:
            self.is_moving = False
            self.t = 0.0
            self.t_inc = 0.0
            self.current_leg_positions = np.copy(self.home_positions)
            self.offsets = np.copy(self.target_offsets)
            self.swing_time = self.target_swing_time
            self.stance_time = 1 - self.swing_time
            self.linear_speed = np.zeros(3)
            self.angular_speed = 0.0