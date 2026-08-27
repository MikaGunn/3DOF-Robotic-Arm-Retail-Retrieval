from launch.actions import TimerAction
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('arduinobot_moveit'),
                'launch',
                'moveit.launch.py'
            )
        )
    )

    trajectory_executor = TimerAction(
	period=10.0,
	actions=[Node(
	        package='arm_control_py',
	        executable='trajectory_executor',
	        output='screen'
	)]
    )

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.3', '0', '0.5', '0', '0', '0', 'base_link', 'camera_frame']
    )

    camera = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        remappings=[('image_raw', '/camera/image_raw')],
        parameters=[{'video_device': os.environ.get('VIDEO_DEVICE', '/dev/video0')}]
    )

    yolo_detector = Node(
        package='arm_vision_tracking',
        executable='yolo_detector_node',
        output='screen'
    )

    foxglove = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        parameters=[{'port': 8765}]
    )

    move_to_xyz = Node(
        package='arm_control_py',
        executable='move_to_xyz',
        output='screen'
    )

    return LaunchDescription([
        moveit_launch,
        trajectory_executor,
        static_tf,
        camera,
        yolo_detector,
        foxglove,
        move_to_xyz,
    ])
