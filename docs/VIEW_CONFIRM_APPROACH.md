# View, Confirm, Approach and Return Controller

`view_confirm_approach.py` implements the high level object confirmation sequence.

## Sequence

```text
Move to requested XYZ pose
        |
        v
Wait for confident object detection
        |
        +-- Wrong class --> Return to final pose
        |
        v
Confirm requested class
        |
        v
Centre object horizontally (joint_1)
        |
        v
Centre object vertically (joint_3)
        |
        v
Approach in small joint_2 steps
        |
        v
Wait
        |
        v
Move to configured final/return pose
```

Default return pose:

```text
joint_1 = 0.0
joint_2 = 0.0
joint_3 = -1.5708
```

The default confidence threshold is `0.482`. Horizontal and vertical pixel deadbands are configurable through ROS 2 parameters.
