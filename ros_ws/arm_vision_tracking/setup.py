from setuptools import find_packages, setup

package_name = 'arm_vision_tracking'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/vision.launch.py',
            'launch/full_system.launch.py',
            'launch/test_system.launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rashmika',
    maintainer_email='',
    description='ROS 2 computer-vision package for YOLO grocery detection and object localization',
    license='Not specified',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'object_plane_localizer = arm_vision_tracking.object_plane_localizer:main',
            'yolo_detector_node = arm_vision_tracking.yolo_detector_node:main',
        ],
    },
)
