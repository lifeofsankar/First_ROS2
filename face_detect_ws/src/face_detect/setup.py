from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'face_detect'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'models'), glob('models/*')),
    ],
    install_requires=[  'setuptools',
                        'opencv-python>=4.5.0',
                        'numpy>=1.20.0'
                      ],
    zip_safe=True,
    maintainer='admin',
    maintainer_email='jaisankar@asimovtrainees.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_publisher = face_detect.camera_publisher:main',
            'hc_face_detect_node = face_detect.hc_face_detect_node:main',
            'face_detect_node = face_detect.face_detect_node:main',
        ],
    },
)