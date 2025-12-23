from setuptools import find_packages, setup

package_name = 'arm_python_control'

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
    maintainer='admin',
    maintainer_email='jaisankarjaikishan369@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'move_arm = arm_python_control.move_arm:main',
            'move_arm_cartesian = arm_python_control.move_arm_cartesian:main',
            'pick_and_place = arm_python_control.pick_and_place:main',
            'add_box_moveit = arm_python_control.add_box_moveit:main',
        ],
    },
)
