# ROS 2 Workspace

This directory contains the ROS 2 packages for robot description, simulation, MoveIt 2, vision integration, inverse kinematics, joint trajectories, and servo control.

Build from this directory:

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths . --ignore-src -r -y
colcon build
source install/setup.bash
```

Important packages include `arm_vision_tracking`, `arm_control_py`, `arduinobot_description`, `arduinobot_moveit`, and `arduinobot_bringup`.
