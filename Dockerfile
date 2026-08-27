FROM ros:jazzy-ros-base

# System dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    ros-jazzy-v4l2-camera \
    ros-jazzy-cv-bridge \
    ros-jazzy-tf2-ros \
    ros-jazzy-tf2-geometry-msgs \
    ros-jazzy-foxglove-bridge \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    libserial-dev \
    && rm -rf /var/lib/apt/lists/*

# CPU-only torch first, then ultralytics — avoids it pulling the full CUDA build
RUN pip3 install torch torchvision --index-url https://download.pytorch.org/whl/cpu \
    --no-cache-dir --break-system-packages \
    && pip3 install \
    ultralytics \
    opencv-python-headless \
    "numpy<2" \
    --break-system-packages \
    --ignore-installed \
    --no-cache-dir \
    && pip3 cache purge

WORKDIR /ros_ws

# Model weights copied early since they rarely change (better layer caching)
COPY computer_vision/models/V3weights.pt /ros_ws/models/V3weights.pt

# Build workspace and clean up artifacts in one layer
COPY ros_ws/ /ros_ws/
RUN rm -rf /ros_ws/build /ros_ws/install /ros_ws/log \
    && /bin/bash -c "source /opt/ros/jazzy/setup.bash && colcon build" \
    && rm -rf /ros_ws/build /ros_ws/log

RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc \
    && echo "source /ros_ws/install/setup.bash" >> ~/.bashrc

EXPOSE 8765
ENV MODEL_PATH=/ros_ws/models/V3weights.pt
ENV VIDEO_DEVICE=/dev/video0
CMD ["/bin/bash", "-c", "source /opt/ros/jazzy/setup.bash && source /ros_ws/install/setup.bash && ros2 launch arm_vision_tracking vision.launch.py"]
