import os
from os import pathsep
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    arduinobot_description = get_package_share_directory('arduinobot_description')
    arduinobot_description_prefix = get_package_prefix('arduinobot_description')

    model_arg = DeclareLaunchArgument(
        name='model',
        default_value=os.path.join(arduinobot_description, 'urdf', 'arduinobot.urdf.xacro'),
        description='Absolute path to robot urdf file'
    )

    model_path = os.path.join(arduinobot_description, 'models')
    model_path += pathsep + os.path.join(arduinobot_description_prefix, 'share')

    env_var = SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', model_path)

    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('model')]),
        value_type=str
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items()
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'arduinobot',
            '-topic', 'robot_description',
        ],
        output='screen'
    )

    return LaunchDescription([
        env_var,
        model_arg,
        gz_sim,
        robot_state_publisher_node,
        spawn_robot,
    ])
