#!/usr/bin/env python3
"""
The cell: UR5e + Robotiq 2F-85 under ONE controller_manager.

Starts
  * robot_state_publisher   - publishes /robot_description and the TF tree
                              (the controller_manager reads the description
                              from that topic, not from a parameter),
  * ros2_control_node       - one controller manager for the arm and the gripper,
  * the controllers:
      joint_state_broadcaster           - one /joint_states for the whole robot,
      scaled_joint_trajectory_controller - arm, honours the pendant speed slider,
      robotiq_gripper_controller        - gripper, ParallelGripperCommand action,
      robotiq_activation_controller     - gripper (re)activation.

Real robot:
    ros2 launch ur5e_vla_control ur5e_control.launch.py \
        robot_ip:=<ROBOT_IP> headless_mode:=true

Without hardware (no robot, no network, no serial port):
    ros2 launch ur5e_vla_control ur5e_control.launch.py use_mock_hardware:=true
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _default_kinematics():
    """Nominal kinematics; replace with the file from ur_calibration."""
    return os.path.join(
        get_package_share_directory('ur5e_vla_description'),
        'config',
        'ur5e_calibration.example.yaml',
    )


def generate_launch_description():
    control_share = FindPackageShare('ur5e_vla_control')
    description_share = FindPackageShare('ur5e_vla_description')

    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip', default_value='192.168.56.101',
        description='IP address of the UR5e controller',
    )
    ur_type_arg = DeclareLaunchArgument(
        'ur_type', default_value='ur5e', description='UR robot type',
    )
    use_mock_hardware_arg = DeclareLaunchArgument(
        'use_mock_hardware', default_value='false',
        description='Mock hardware for the ARM - no robot, no network needed',
    )
    use_mock_gripper_arg = DeclareLaunchArgument(
        'use_mock_gripper', default_value='false',
        description='Mock hardware for the GRIPPER - no serial port needed',
    )
    headless_mode_arg = DeclareLaunchArgument(
        'headless_mode', default_value='false',
        description=(
            'Headless mode: the driver sends its own URScript. Required when '
            'the pendant has remote control enabled but no External Control '
            'program is running.'
        ),
    )
    kinematics_params_arg = DeclareLaunchArgument(
        'kinematics_params', default_value=_default_kinematics(),
        description='Kinematics/calibration file (see ur_calibration)',
    )
    com_port_arg = DeclareLaunchArgument(
        'com_port', default_value='/dev/ttyACM0',
        description='Serial port of the gripper (RS-485 over CDC-ACM)',
    )
    slave_address_arg = DeclareLaunchArgument(
        'slave_address', default_value='0x2',
        description='Modbus slave address of the gripper (hex, e.g. 0x2)',
    )
    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz', default_value='false', description='Start RViz2',
    )

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([
            description_share, 'urdf', 'ur5e_with_gripper.urdf.xacro',
        ]),
        ' ',
        'use_mock_hardware:=', LaunchConfiguration('use_mock_hardware'),
        ' ',
        'use_mock_gripper:=', LaunchConfiguration('use_mock_gripper'),
        ' ',
        'robot_ip:=', LaunchConfiguration('robot_ip'),
        ' ',
        'ur_type:=', LaunchConfiguration('ur_type'),
        ' ',
        'headless_mode:=', LaunchConfiguration('headless_mode'),
        ' ',
        'kinematics_params:=', LaunchConfiguration('kinematics_params'),
        ' ',
        'com_port:=', LaunchConfiguration('com_port'),
        ' ',
        'slave_address:=', LaunchConfiguration('slave_address'),
    ])

    robot_description_param = {
        'robot_description': ParameterValue(
            robot_description_content, value_type=str,
        ),
    }

    controllers_file = PythonExpression([
        "'", control_share, "/config/ur5e_controllers",
        "' + ('_mock' if '", LaunchConfiguration('use_mock_gripper'),
        "' == 'true' else '') + '.yaml'",
    ])

    state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description_param],
    )

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[robot_description_param, controllers_file],
    )

    def spawner(controller):
        return Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                controller,
                '--controller-manager', '/controller_manager',
                # Loading the hardware (and the robot description from the
                # topic) can take longer than the default 10 s; without this
                # the spawner retries and reports "can not be configured from
                # 'active' state" although the controller is up and running.
                '--service-call-timeout', '120',
            ],
            output='screen',
        )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', PathJoinSubstitution([
            description_share, 'rviz', 'ur5e.rviz',
        ])],
        condition=IfCondition(LaunchConfiguration('launch_rviz')),
    )

    return LaunchDescription([
        robot_ip_arg,
        ur_type_arg,
        use_mock_hardware_arg,
        use_mock_gripper_arg,
        headless_mode_arg,
        kinematics_params_arg,
        com_port_arg,
        slave_address_arg,
        launch_rviz_arg,
        state_publisher,
        control_node,
        spawner('joint_state_broadcaster'),
        spawner('scaled_joint_trajectory_controller'),
        spawner('robotiq_gripper_controller'),
        spawner('robotiq_activation_controller'),
        rviz,
    ])
