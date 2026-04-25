import math
from pathlib import Path
import json
from ament_index_python.packages import get_package_share_directory
import os

# ==================== EASING FUNCTIONS ====================
def ease_linear(t):
    return t

def ease_in(t):
    return t ** 3

def ease_out(t):
    return 1.0 - (1.0 - t) ** 3

def ease_in_out(t):
    if t < 0.5:
        return 4.0 * t**3
    else:
        return 1.0 - (-2.0 * t + 2.0)**3 / 2.0

EASING = {
    'linear': ease_linear,
    'ease-in': ease_in,
    'ease-out': ease_out,
    'ease-in-out': ease_in_out,
}

class AnimationLoader:
    def __init__(self, path):
        self.directory = Path(os.path.join(get_package_share_directory('sophia_controller'), path))
        self.animations = {}
        self._load()

    def _load(self):
        if not self.directory.exists():
            print(f"Error: The directory '{self.directory}' does not exist.")
            return

        for file in self.directory.glob("*.json"):
            try:
                with open(file, 'r') as f:
                    data = json.load(f)
                    kfs = data.get('keyframes', data) if isinstance(data, dict) else data
                    self.animations[file.stem] = AnimationPlayer(kfs)
            except json.JSONDecodeError:
                print(f"Syntax error in {file.name}")
            except Exception as e:
                print(f"Error: Could not load animation from {file.name}: {e}")
            
    def get_animation(self, name):
        return self.animations.get(name, None)

class AnimationPlayer:
    def __init__(self, keyframes):
        self.keyframes = keyframes
        self.total_duration = sum(kf['duration'] for kf in self.keyframes[1:]) if len(self.keyframes) > 1 else 0
        self.elapsed = 0.0
        self.playing = False

    def play(self):
        """Start or restart the animation."""
        self.elapsed = 0.0
        self.playing = True

    def is_playing(self):
        return self.playing

    def get_frame(self, dt):
        """Advance time by dt and return the current (body, legs) state."""
        if not self.playing:
            return None

        self.elapsed += dt

        if self.elapsed >= self.total_duration:
            self.playing = False
            self.elapsed = self.total_duration

        # Find the current segment
        accumulated = 0.0
        for seg_idx in range(1, len(self.keyframes)):
            kf_a = self.keyframes[seg_idx - 1]
            kf_b = self.keyframes[seg_idx]
            duration = kf_b['duration']

            if accumulated + duration >= self.elapsed or seg_idx == len(self.keyframes) - 1:
                # Interpolate locally inside this segment
                local_elapsed = self.elapsed - accumulated
                t = min(local_elapsed / duration, 1.0) if duration > 0 else 1.0
                
                return self._interpolate(kf_a, kf_b, t, kf_b.get('easing', 'ease-in-out'))
            
            accumulated += duration
        
        return None

    def _interpolate(self, kf_a, kf_b, t, easing_name):
        ease_fn = EASING.get(easing_name, ease_linear)
        et = ease_fn(t)

        body = {}
        for key in ['x', 'y', 'z', 'roll', 'pitch', 'yaw']:
            body[key] = kf_a['body'][key] * (1.0 - et) + kf_b['body'][key] * et

        arc_height = kf_b.get('arc_height', 0.0)

        legs = []
        for i in range(6):
            leg = [0.0, 0.0, 0.0]
            p0x, p0y, p0z = kf_a['legs'][i]
            p2x, p2y, p2z = kf_b['legs'][i]

            dist = math.hypot(p2x - p0x, p2y - p0y)

            leg[0] = p0x * (1.0 - et) + p2x * et
            leg[1] = p0y * (1.0 - et) + p2y * et

            traj = kf_b.get('trajectory', 'linear')
            effective_arc = arc_height
            if traj == 'bezier' and effective_arc < 0.001:
                effective_arc = 0.04

            if effective_arc > 0.001 and dist > 0.005:
                leg[2] = p0z * (1.0 - et) + p2z * et + 4.0 * effective_arc * et * (1.0 - et)
            else:
                leg[2] = p0z * (1.0 - et) + p2z * et

            legs.append(leg)

        return body, legs
