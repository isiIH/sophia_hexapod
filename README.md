# Sophia Hexapod Robot #

## Installation & Build ##

Clone this repository into your ROS 2 workspace (e.g., `~/ros2_ws/src`) and run the following:

```bash
# Navigate to workspace
cd ~/ros2_ws/

# Build the packages
colcon build --symlink-install

# Source the overlay
source install/setup.bash
```

> **[IMPORTANT]** If you use **Conda**, deactivate your environment before building to avoid library conflicts: `conda deactivate`.


## Usage ##

1. Launch the simulation in Gazebo:
```bash
ros2 launch sophia_gazebo sophia_gazebo.launch.py
```

2. Run the following tests in a new terminal:

```bash
ros2 run sophia_controller standing_position.py

ros2 run sophia_controller home_position.py
```