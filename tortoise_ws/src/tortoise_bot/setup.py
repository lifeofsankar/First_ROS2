from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'tortoise_bot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), [
            'launch/teleop_launch.py',
            'launch/test_gazebo.launch.py'
        ]),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),  # Add SDF support
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='admin',
    maintainer_email='jaisankar@asimovtrainees.com',
    description='Tortoise Bot package for ROS2 simulation and control',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'simple_joint_publisher = tortoise_bot.simple_joint_publisher:main',
            'tortoise_controller = tortoise_bot.tortoise_controller:main',
            'fake_joint_publisher = tortoise_bot.fake_joint_publisher:main',
            'turtle_bot_teleop = tortoise_bot.turtle_bot_teleop:main',
            'velocity_controller = tortoise_bot.velocity_controller:main',
            'lidar_processor = tortoise_bot.lidar_processor:main'  # Add LIDAR processor
        ],
    },
)