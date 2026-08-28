# 3-DOF Robotic Arm for Retail Item Retrieval

A ROS 2 based robotic manipulation project developed for **retail shelf item retrieval** using a compact **3-DOF robotic arm**, **camera based position refinement**, **inverse kinematics**, **MoveIt 2**, **Gazebo** and **RViz**.

The system is designed as part of an autonomous shopping assistant concept. Instead of relying only on fixed coordinates or only on vision, it uses a **hybrid localisation strategy**: predefined product coordinates are used for coarse positioning, while camera feedback is used for local correction and final alignment.

---

## Project Overview

Retail environments are more constrained and less structured than traditional warehouses. Products may be placed with small positional variations, shelves are narrow and the robot must operate with limited computational and mechanical complexity.

This project addresses that problem using a simplified 3-DOF manipulator that can:

- move toward a predefined shelf position
- calculate joint angles using inverse kinematics
- use ROS 2 and MoveIt 2 for motion planning and control
- receive camera based object localisation data
- refine the arm position using visual feedback
- approach the detected target through gradual intermediate movements
- execute the control pipeline in simulation and on physical hardware.

The manipulator was designed for a constrained retail shelf environment with a maximum reach of approximately **0.226 m**.

---

## System Architecture

```text
User / predefined product target
            |
            v
Target coordinates (x, y, z)
            |
            v
ROS 2 control node
            |
            v
Inverse Kinematics / MoveIt 2
            |
            v
Joint trajectory generation
            |
            v
Move arm to stand-off position
            |
            v
Camera-based object detection / localisation
            |
            v
Position refinement
            |
            v
Generate gradual approach points
            |
            v
Move end effector toward object
            |
            v
Final target position
```

The software architecture is modular and separates perception, control and actuation into ROS 2 components.

---

## Hardware

The physical system uses:

- **Raspberry Pi 4 Model B**
  - Quad-core CPU
  - 4 GB RAM
  - Ubuntu 22.04 server OS
  - Executes ROS 2 nodes, image processing, and control algorithms

- **Adeept Robot HAT V3.3**
  - 16-channel servo motor driver
  - I2C / GPIO based integration

- **3-DOF Robotic Arm**
  - 3 rotational joints
  - Base rotation, shoulder and elbow
  - Aluminium structure
  - Approximate link dimensions:
    - Link 1: 0.085 m
    - Link 2: 0.106 m
    - Link 3: 0.120 m
  - Maximum payload: approximately 0.1 kg

- **MG996R Servo Motors**
  - Quantity: 3
  - Used for the base, shoulder and elbow joints

- **1080p RGB Camera**
  - Used for object centre detection and local position refinement

---

## Software and Technologies

- ROS 2
- MoveIt 2
- Gazebo
- RViz
- Python
- OpenCV
- Inverse Kinematics
- Forward Kinematics
- Denavit Hartenberg modelling
- Camera based localisation
- Visual servoing / image centre alignment
- Joint trajectory control
- Raspberry Pi
- Servo actuation

---

## Kinematic Model

The robotic manipulator consists of three revolute joints:

```text
q1 -> Base rotation
q2 -> Shoulder joint
q3 -> Elbow joint
```

### Denavit-Hartenberg Parameters

| Link | a | α | d | θ |
|---|---:|---:|---:|---:|
| 1 | 0 | 90° | L1 | q1 |
| 2 | L2 | 0 | 0 | q2 |
| 3 | L3 | 0 | 0 | q3 |

### Forward Kinematics

The horizontal reach is:

```text
r = L2 cos(q2) + L3 cos(q2 + q3)
```

The end-effector position is calculated as:

```text
x = r cos(q1)

y = r sin(q1)

z = L1 + L2 cos(q2) + L3 cos(q2 + q3)
```

### Inverse Kinematics

For a desired end effector position `(x, y, z)`:

```text
q1 = atan2(y, x)
```

Then:

```text
r = sqrt(x² + y²)
s = z - L1
```

The elbow angle is obtained using the cosine rule and the shoulder angle is calculated from the geometric relationship between the two planar arm links.

The analytical IK solution allows the arm to calculate joint angles efficiently for real time target positioning.

---

## Hybrid Localisation Strategy

A key feature of the project is the use of **hybrid localisation**.

### 1. Coarse Positioning

The robot begins with predefined product coordinates and moves to a stand off position near the expected product location.

### 2. Camera-Based Refinement

The camera detects the object centre in image coordinates:

```text
(u, v)
```

The image error is calculated relative to the camera centre:

```text
e = [u - cx,
     v - cy]
```

The goal is to reduce this error so the product appears near the image centre.

### 3. Final Approach

After the object position is refined, the robot generates intermediate points between the stand off position and the detected target.

This gradual approach reduces abrupt changes in joint position and improves motion stability.

---

## Camera to Robot Coordinate Transformation

Detected image coordinates can be projected into the camera frame using the camera intrinsic parameters:

```text
xc = ((u - cx) / fx) * zc
yc = ((v - cy) / fy) * zc
```

The coordinates are then transformed into the robot/world frame using the forward kinematic transformation and the fixed camera to end effector relationship.

This allows the detected target location to be expressed in coordinates that can be used by the robotic arm controller.

---

## ROS 2 Control Workflow

The main control sequence is:

```text
1. Initialise ROS 2 node
2. Read target coordinates
3. Request IK solution
4. Generate joint trajectory
5. Move to stand off position
6. Wait for object detection
7. Receive refined target coordinates
8. Calculate new IK solution
9. Generate gradual approach points
10. Move through intermediate positions
11. Reach final target
12. Return to home position
```

This makes the motion structured while still allowing the arm to react to small positional changes detected by the camera.

---

## Results

The system was tested through multiple ROS 2 based experimental runs.

The results showed that:

- the arm consistently reached predefined stand-off positions;
- inverse kinematics produced valid joint configurations without instability;
- camera-based localisation produced target coordinates close to the expected physical object location;
- minor localisation tolerance was observed, which is expected when using a 2D camera;
- intermediate approach points created smooth, step-wise trajectories;
- the arm successfully followed the generated points in sequence;
- the final end-effector position matched the detected target coordinates during the recorded tests;
- the complete motion cycle included movement from home to target and return to home.

The report also shows that the coordinate profiles changed gradually without sudden discontinuities, indicating stable motion execution.

---

## Example Localisation Result

For one test, the physical target was approximately:

```text
x = 1.500
y = 0.045
z = 0.850
```

Two example camera detections for the same target were approximately:

```text
Detection 1:
x = 1.541
y = 0.048
z = 0.850

Detection 2:
x = 1.498
y = 0.038
z = 0.850
```

This demonstrates the small positional variation produced by the vision subsystem and supports the use of camera-based refinement rather than relying only on fixed coordinates.

---

## Project Strengths

- Simplified 3-DOF mechanical design
- Low computational complexity
- Analytical inverse kinematics
- ROS 2 modular architecture
- Simulation and real-hardware validation
- Camera based correction
- Gradual approach trajectory
- Suitable for constrained shelf environments
- Integration of perception, control, and actuation

---

## Limitations

The project also identified several limitations:

- limited end effector orientation control due to only 3 DOF;
- dependence on predefined target coordinates;
- perception performance can be affected by lighting and visibility;
- no active gripper force feedback;
- reduced flexibility in unstructured environments.

---

## Future Improvements

Possible future developments include:

- increasing the number of degrees of freedom;
- adding active gripper control with force feedback;
- improving object detection and pose estimation;
- reducing dependence on predefined coordinates;
- improving camera calibration;
- improving timing and synchronization;
- adding more robust perception for changing retail environments.

---

## Suggested GitHub Topics

`ros2` `robotics` `robot-arm` `moveit2` `gazebo` `rviz` `inverse-kinematics` `computer-vision` `opencv` `visual-servoing` `retail-automation` `raspberry-pi`

---


