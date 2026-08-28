# Project Summary

## Project

**Design and Development of a 3-DOF Robotic Arm for Retail Item Retrieval**

## Purpose

The project develops a compact robotic manipulation subsystem for structured retail shelf item retrieval.

The design combines:

- predefined product coordinates for coarse positioning
- analytical inverse kinematics
- ROS 2 based modular control
- MoveIt 2 trajectory planning
- Gazebo and RViz simulation
- camera-based position refinement
- gradual multi-step approach motion
- Raspberry Pi based real-hardware deployment.

## Main Hardware

- Raspberry Pi 4 Model B
- Adeept Robot HAT V3.3
- 3-DOF Adeept robotic arm
- 3 × MG996R servo motors
- 1080p RGB camera

## Main Arm Parameters

- Link 1: 0.085 m
- Link 2: 0.106 m
- Link 3: 0.120 m
- Maximum payload: approximately 0.1 kg
- Maximum reach: approximately 0.226 m

## Core Control Sequence

```text
Predefined target
      ↓
Inverse kinematics
      ↓
Move to stand-off position
      ↓
Camera-based target refinement
      ↓
Generate intermediate approach points
      ↓
Move to refined target
      ↓
Return to home/final pose
```

## Key Engineering Idea

The system uses **hybrid localisation** rather than relying entirely on either fixed coordinates or computer vision. Fixed coordinates provide efficient coarse positioning, while visual feedback provides local correction close to the target.
