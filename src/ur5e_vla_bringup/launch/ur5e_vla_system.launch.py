#!/usr/bin/env python3
"""
The whole cell in one command: UR5e + Robotiq 2F-85 under a single
controller_manager, plus the wrist camera.

Without hardware:
    ros2 launch ur5e_vla_bringup ur5e_vla_system.launch.py \
        use_mock_hardware:=true use_mock_gripper:=true

With the real robot (pendant in Remote Control, External Control running):
    ros2 launch ur5e_vla_bringup ur5e_vla_system.launch.py \
        robot_ip:=<ROBOT_IP> headless_mode:=true
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    control_share = FindPackageShare('ur5e_vla_control')

    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip', default_value='192.168.56.101',
        description='IP address of the UR5e controller',
    )
    use_mock_hardware_arg = DeclareLaunchArgument(
        'use_mock_hardware', default_value='false',
        description='Mock hardware for the arm',
    )
    use_mock_gripper_arg = DeclareLaunchArgument(
        'use_mock_gripper', default_value='false',
        description='Mock hardware for the gripper',
    )
    headless_mode_arg = DeclareLaunchArgument(
        'headless_mode', default_value='false',
        description='The driver sends its own URScript (no External Control program)',
    )
    com_port_arg = DeclareLaunchArgument(
        'com_port', default_value='/dev/ttyACM0',
        description='Serial port of the gripper',
    )
    slave_address_arg = DeclareLaunchArgument(
        'slave_address', default_value='0x2',
        description='Modbus slave address of the gripper',
    )
    video_device_arg = DeclareLaunchArgument(
        'video_device', default_value='/dev/video0',
        description='Video device of the wrist camera',
    )
    launch_camera_arg = DeclareLaunchArgument(
        'launch_camera', default_value='true',
        description='Start the wrist camera',
    )

    control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                control_share, 'launch', 'ur5e_control.launch.py',
            ]),
        ),
        launch_arguments={
            'robot_ip': LaunchConfiguration('robot_ip'),
            'use_mock_hardware': LaunchConfiguration('use_mock_hardware'),
            'use_mock_gripper': LaunchConfiguration('use_mock_gripper'),
            'headless_mode': LaunchConfiguration('headless_mode'),
            'com_port': LaunchConfiguration('com_port'),
            'slave_address': LaunchConfiguration('slave_address'),
        }.items(),
    )

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                control_share, 'launch', 'wrist_camera.launch.py',
            ]),
        ),
        launch_arguments={
            'video_device': LaunchConfiguration('video_device'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('launch_camera')),
    )

    return LaunchDescription([
        robot_ip_arg,
        use_mock_hardware_arg,
        use_mock_gripper_arg,
        headless_mode_arg,
        com_port_arg,
        slave_address_arg,
        video_device_arg,
        launch_camera_arg,
        control,
        camera,
    ])
