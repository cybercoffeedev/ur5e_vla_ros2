#!/usr/bin/env python3
"""L1 tests for the robot description.

These are pure software tests: the xacro is rendered with mock hardware, so no
robot, no network and no serial port is involved.

They guard the mistakes that are invisible until the robot moves:
  * the gripper attached to the wrong link (or to a second `world`),
  * a wrong hardware plugin,
  * a stale slave address,
  * a joint name that does not match the controller configuration.
"""
import os
import xml.etree.ElementTree as ET

import pytest
import xacro
from ament_index_python.packages import get_package_share_directory

SHARE = get_package_share_directory('ur5e_vla_description')
XACRO_FILE = os.path.join(SHARE, 'urdf', 'ur5e_with_gripper.urdf.xacro')

ARM_JOINTS = [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint',
]
GRIPPER_JOINT = 'robotiq_85_left_knuckle_joint'


def _render(mappings):
    """Render the description and return it as an ElementTree element."""
    doc = xacro.process_file(XACRO_FILE, mappings=mappings)
    # xacro returns a minidom document; work on an ElementTree instead.
    return ET.fromstring(doc.toxml())


@pytest.fixture(scope='module')
def robot():
    """The rendered robot description (mock hardware, no robot needed)."""
    return _render({'use_mock_hardware': 'true'})


def _ros2_control_blocks(robot):
    return robot.findall('ros2_control')


def _block_named(robot, name):
    for block in _ros2_control_blocks(robot):
        if block.get('name') == name:
            return block
    raise AssertionError(f'no <ros2_control> block named {name!r}')


def _plugin_of(block):
    plugin = block.find('./hardware/plugin')
    return plugin.text if plugin is not None else None


def _params_of(block):
    return {p.get('name'): (p.text or '').strip() for p in block.findall('./hardware/param')}


def test_robot_has_exactly_two_hardware_components(robot):
    """One <ros2_control> per hardware component: the arm and the gripper."""
    blocks = _ros2_control_blocks(robot)
    assert len(blocks) == 2, (
        f'expected exactly 2 <ros2_control> blocks (arm + gripper), got {len(blocks)}'
    )
    names = sorted(b.get('name') for b in blocks)
    assert names == ['RobotiqGripperHardwareInterface', 'ur5e']


def test_hardware_plugins_are_the_upstream_ones():
    """With real hardware the two upstream plugins are used."""
    real = _render({'use_mock_hardware': 'false'})
    plugins = {b.get('name'): _plugin_of(b) for b in _ros2_control_blocks(real)}
    assert plugins['ur5e'] == 'ur_robot_driver/URPositionHardwareInterface'
    assert plugins['RobotiqGripperHardwareInterface'] == (
        'robotiq_driver/RobotiqGripperHardwareInterface'
    )


def test_mock_rendering_uses_mock_hardware():
    """In mock mode neither component may touch the hardware."""
    mock = _render({'use_mock_hardware': 'true', 'use_mock_gripper': 'true'})
    for block in _ros2_control_blocks(mock):
        assert _plugin_of(block) == 'mock_components/GenericSystem'


def test_arm_and_gripper_mocks_are_independent():
    """Mocking the arm must not mock the gripper (and the other way round)."""
    arm_mock = _render({'use_mock_hardware': 'true', 'use_mock_gripper': 'false'})
    plugins = {b.get('name'): _plugin_of(b) for b in _ros2_control_blocks(arm_mock)}
    assert plugins['ur5e'] == 'mock_components/GenericSystem'
    assert plugins['RobotiqGripperHardwareInterface'] == (
        'robotiq_driver/RobotiqGripperHardwareInterface'
    )

    gripper_mock = _render({'use_mock_hardware': 'false', 'use_mock_gripper': 'true'})
    plugins = {b.get('name'): _plugin_of(b) for b in _ros2_control_blocks(gripper_mock)}
    assert plugins['ur5e'] == 'ur_robot_driver/URPositionHardwareInterface'
    assert plugins['RobotiqGripperHardwareInterface'] == 'mock_components/GenericSystem'


def test_gripper_is_mounted_on_the_flange(robot):
    """The gripper must hang on the tool flange, not on a second world link."""
    base_joint = None
    for joint in robot.findall('joint'):
        if joint.get('name') == 'robotiq_85_base_joint':
            base_joint = joint
            break
    assert base_joint is not None, 'gripper base joint is missing'

    parent = base_joint.find('parent').get('link')
    assert parent == 'flange', (
        f'gripper is attached to {parent!r}, expected the tool flange'
    )

    links = [link.get('name') for link in robot.findall('link')]
    assert links.count('world') == 1, 'the robot must have exactly one world link'


def test_gripper_hardware_parameters(robot):
    """The gripper block carries the parameters robotiq_driver expects."""
    gripper = _block_named(robot, 'RobotiqGripperHardwareInterface')
    params = _params_of(gripper)

    if _plugin_of(gripper) == 'mock_components/GenericSystem':
        # Mock rendering: the real parameters are not emitted.
        return

    assert params['COM_port'] == '/dev/ttyACM0'
    assert params['baudrate'] == '115200'
    # This unit answers on slave 2 - the upstream default of 9 does not reply.
    assert params['slave_address'] == '0x2'
    assert float(params['gripper_closed_position']) == pytest.approx(0.7929)


def test_gripper_joint_interfaces(robot):
    """One position command interface, position + velocity state interfaces."""
    gripper = _block_named(robot, 'RobotiqGripperHardwareInterface')
    joints = [j for j in gripper.findall('joint') if j.get('name') == GRIPPER_JOINT]
    assert len(joints) == 1, f'{GRIPPER_JOINT} is not declared exactly once'

    joint = joints[0]
    commands = [c.get('name') for c in joint.findall('command_interface')]
    states = [s.get('name') for s in joint.findall('state_interface')]
    assert commands == ['position']
    assert sorted(states) == ['position', 'velocity']


def test_arm_joints_are_present(robot):
    names = [joint.get('name') for joint in robot.findall('joint')]
    for arm_joint in ARM_JOINTS:
        assert arm_joint in names, f'arm joint {arm_joint} is missing'


def test_no_duplicate_link_or_joint_names(robot):
    links = [link.get('name') for link in robot.findall('link')]
    joints = [joint.get('name') for joint in robot.findall('joint')]
    assert len(links) == len(set(links)), 'duplicate link names'
    assert len(joints) == len(set(joints)), 'duplicate joint names'


def test_gripper_joint_name_matches_controller_configuration(robot):
    """The controller config and the URDF have to agree on the joint name."""
    import yaml

    control_share = get_package_share_directory('ur5e_vla_control')
    with open(os.path.join(control_share, 'config', 'ur5e_controllers.yaml')) as handle:
        config = yaml.safe_load(handle)

    controller_joint = config['robotiq_gripper_controller']['ros__parameters']['joint']
    names = [joint.get('name') for joint in robot.findall('joint')]
    assert controller_joint in names, (
        f'controller commands {controller_joint!r}, which is not a joint of the robot'
    )


def test_real_hardware_rendering_uses_requested_slave_address():
    """The slave address is a parameter, not a constant."""
    doc = _render(
        {
            'use_mock_hardware': 'false',
            'slave_address': '0x2',
            'com_port': '/dev/ttyACM9',
        },
    )
    gripper = next(
        b for b in _ros2_control_blocks(doc)
        if _plugin_of(b) == 'robotiq_driver/RobotiqGripperHardwareInterface'
    )
    params = _params_of(gripper)
    assert params['slave_address'] == '0x2'
    assert params['COM_port'] == '/dev/ttyACM9'
