import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'ur5e_vla_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cybercoffeedev',
    maintainer_email='cybercoffeedev@proton.me',
    description='One launch for the whole UR5e cell with gripper and camera.',
    tests_require=['pytest'],
)
