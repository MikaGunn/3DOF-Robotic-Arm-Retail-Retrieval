import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0.3', '0', '0.5', '0', '0', '0', 'base_link', 'camera_frame']
        ),

        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            remappings=[('image_raw', '/camera/image_raw')],
            parameters=[{'video_device': os.environ.get('VIDEO_DEVICE', '/dev/video0')}]
        ),

        Node(
            package='arm_vision_tracking',
            executable='yolo_detector_node',
            output='screen'
        ),

        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            parameters=[{'port': 8765}]
        ),

    ])
