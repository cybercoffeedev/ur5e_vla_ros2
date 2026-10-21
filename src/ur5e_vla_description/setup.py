import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'ur5e_vla_description'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/urdf', glob('urdf/*.xacro')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cybercoffeedev',
    maintainer_email='cybercoffeedev@proton.me',
    description=(
        'Robot description of the UR5e cell with a Robotiq 2F-85 gripper '
        'mounted on the tool flange.'
    ),
    tests_require=['pytest'],
)
