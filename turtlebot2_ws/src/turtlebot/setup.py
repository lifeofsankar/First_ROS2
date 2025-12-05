import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'turtlebot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='admin',
    maintainer_email='jaisankarjaikishan369@gmail.com',
    description='TurtleBot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_joint_publisher = turtlebot.simple_joint_publisher:main',
            'lidar_processor = turtlebot.lidar_processor:main',
            'ball_follower = turtlebot.ball_follower:main',
        ],
    },
)
