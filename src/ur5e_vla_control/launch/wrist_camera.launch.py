#!/usr/bin/env python3
"""
Wrist camera (Robotiq) through the upstream USB camera driver.

    ros2 launch ur5e_vla_control wrist_camera.launch.py
    ros2 launch ur5e_vla_control wrist_camera.launch.py video_device:=/dev/video1
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    video_device_arg = DeclareLaunchArgument(
        'video_device', default_value='/dev/video0',
        description='Video device of the wrist camera',
    )
    framerate_arg = DeclareLaunchArgument(
        'framerate', default_value='30.0', description='Frames per second',
    )

    camera = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='wrist_camera',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('ur5e_vla_control'), 'config', 'usb_cam.yaml',
            ]),
            {
                'video_device': LaunchConfiguration('video_device'),
                'framerate': LaunchConfiguration('framerate'),
                'publish_rate': LaunchConfiguration('framerate'),
            },
        ],
    )

    return LaunchDescription([video_device_arg, framerate_arg, camera])
