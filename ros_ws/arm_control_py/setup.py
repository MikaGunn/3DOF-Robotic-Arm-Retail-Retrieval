from setuptools import find_packages, setup

package_name = 'arm_control_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rashmika',
    maintainer_email='',
    description='ROS 2 trajectory, IK, visual-servo and target-confirmation control for the robotic arm',
    license='Not specified',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
    'send_joint_trajectory = arm_control_py.send_joint_trajectory:main',
    'move_to_xyz = arm_control_py.move_to_xyz:main','trajectory_executor = arm_control_py.trajectory_executor:main', 'visual_servo_node = arm_control_py.visual_servo_node:main',
    'view_confirm_approach = arm_control_py.view_confirm_approach:main',
        ],
    },
)
