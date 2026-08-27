# Computer Vision

This directory stores the trained Ultralytics model weights used and evaluated during the project.

The ROS detector loads a model through the `MODEL_PATH` environment variable. The Docker configuration defaults to `V3weights.pt`.

Classes used by the ROS detector:

- Apple
- Banana
- Lemon
- Orange

Before publishing model weights publicly, confirm that the training data and generated weights may be redistributed.
