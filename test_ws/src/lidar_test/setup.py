from setuptools import setup
import os
from glob import glob

package_name = 'lidar_test'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # ✅ install launch folder
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jai',
    maintainer_email='jai@example.com',
    description='Python-only LiDAR test using ROS2 Jazzy',
    license='MIT',
    entry_points={
        'console_scripts': [
            'lidar_listener = lidar_test.lidar_listener:main',
        ],
    },
)
