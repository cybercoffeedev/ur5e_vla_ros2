import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'ur5e_vla_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cybercoffeedev',
    maintainer_email='cybercoffeedev@proton.me',
    description=(
        'ros2_control configuration of the UR5e cell: one controller manager '
        'for the arm and the Robotiq 2F-85 gripper.'
    ),
    tests_require=['pytest'],
)
