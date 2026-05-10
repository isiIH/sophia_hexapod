# Hexapod Animation

A standalone ROS 2 package to play and visualize `.json` animations for the Sophia Hexapod. 

This package is self-contained, it includes its own URDF and RViz configurations, so you can share this folder directly with anyone—they don't need the rest of the robot's codebase to run it.

## 🛠️ Build

```bash
cd ~/ros2_ws
colcon build --packages-select hexapod_animation
source install/setup.bash
```

## 🚀 Usage

### Play the default animation (attack.json)

```bash
ros2 launch hexapod_animation display_animation.launch.py
```

### Play your own animation

If you generated a `.json` animation using [HexAnimator](https://isiih.github.io/HexAnimator/), you can play it by passing its absolute path:

```bash
ros2 launch hexapod_animation display_animation.launch.py animation_file:=/absolute/path/to/your_animation.json
```

**Note**: The animation will play in an infinite loop. Press `Ctrl + C` in the terminal to stop it.
