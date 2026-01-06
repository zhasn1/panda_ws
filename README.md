# Panda Robot Pick-and-Place Simulation
A ROS 2 workspace for simulating a Franka Panda robotic arm with vision-guided pick-and-place capabilities using MoveIt 2 and Gazebo.

## Dependencies
- ROS 2 Jazzy
- MoveIt 2
- ros_gz_sim
- cv_bridge
- opencv-python

Make sure you have all dependencies installed and that the ROS 2 environment is sourced before continuing.

## Installation
1. **Clone this repository in your ROS 2 workspace:**
```bash
git clone https://github.com/zhasn1/panda_ws
```
2. **Build the package:**
```bash
cd ~/rs_ws
colcon build
source install/setup.bash
```

## Running the simulation
Launch all components using the launch file:
```bash
ros2 launch panda_bringup panda.launch.py
```
