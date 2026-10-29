import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'teleop_command_mapper'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Tran Huynh',
    maintainer_email='tranhuynh7777777@gmail.com',
    description='Keyboard teleop: drive mode (/cmd_vel) + arm Cartesian jog mode (joint targets via KDL).',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_command_mapper = teleop_command_mapper.teleop_command_mapper_node:main',
            'rover_odom_sim = teleop_command_mapper.rover_odom_sim:main',
            'joy_command_mapper = teleop_command_mapper.joy_command_mapper_node:main',
            'joy_sim_gui = teleop_command_mapper.joy_sim_gui:main',
            'arm_command_bridge = teleop_command_mapper.arm_command_bridge:main',
        ],
    },
)
