#!/usr/bin/env python3
"""L2 integration test: the whole stack on mock hardware.

No robot, no network, no serial port: the launch is started with
``use_mock_hardware:=true``, so both the arm and the gripper are served by
``mock_components/GenericSystem``.

What is verified:
  * the launch comes up and all four controllers reach the ``active`` state,
  * one ``/joint_states`` carries the arm joints *and* the gripper joint,
  * the gripper action accepts a close command and the position converges.

The same file is reused for the hardware-in-the-loop variant (L3) by starting
the launch without ``use_mock_hardware`` - see ``test_gripper_hardware.py``.
"""
import os
import signal
import subprocess
import tempfile
import time

import pytest
import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import ParallelGripperCommand
from controller_manager_msgs.srv import ListControllers
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState

ARM_JOINTS = [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint',
]
GRIPPER_JOINT = 'robotiq_85_left_knuckle_joint'
ACTION = '/robotiq_gripper_controller/gripper_cmd'
STARTUP_TIMEOUT = 90.0


@pytest.fixture(scope='module')
def stack():
    """Run the launch in the background for the duration of the module."""
    log = tempfile.NamedTemporaryFile(prefix='ur5e_l2_', suffix='.log', delete=False)
    process = subprocess.Popen(
        [
            'ros2', 'launch', 'ur5e_vla_control', 'ur5e_control.launch.py',
            'use_mock_hardware:=true',
            'use_mock_gripper:=true',
        ],
        stdout=log,
        stderr=subprocess.STDOUT,
        # Own process group: the launch starts child nodes, and terminating
        # only the parent would leave them behind. Leftover robot_state_publishers
        # keep spamming /robot_description and starve the controller manager.
        start_new_session=True,
    )
    process.log_file = log.name
    yield process
    _terminate_group(process)


def _terminate_group(process):
    """Stop the launch and every node it started."""
    try:
        os.killpg(os.getpgid(process.pid), signal.SIGTERM)
    except (ProcessLookupError, PermissionError):
        pass
    try:
        process.wait(timeout=15)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(os.getpgid(process.pid), signal.SIGKILL)
        except (ProcessLookupError, PermissionError):
            pass
        process.wait(timeout=10)


@pytest.fixture(scope='module')
def node():
    rclpy.init()
    node = Node('ur5e_l2_test')
    yield node
    node.destroy_node()
    rclpy.shutdown()


def _wait_for_joint_states(node, expected, timeout=STARTUP_TIMEOUT):
    """Wait for a /joint_states message that contains all expected joints."""
    received = []
    deadline = time.monotonic() + timeout

    def callback(msg):
        received.append(set(msg.name))

    subscription = node.create_subscription(JointState, '/joint_states', callback, 10)
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            for names in received:
                if expected.issubset(names):
                    return names
    finally:
        node.destroy_subscription(subscription)
    raise AssertionError(
        f'/joint_states never contained {sorted(expected)}; last seen: {received[-1] if received else None}'
    )


def test_joint_states_contain_arm_and_gripper(node, stack):
    """One broadcaster for the whole robot: 6 arm joints + the gripper joint."""
    names = _wait_for_joint_states(node, set(ARM_JOINTS) | {GRIPPER_JOINT})
    assert GRIPPER_JOINT in names
    assert len([n for n in names if n in ARM_JOINTS]) == 6


def _wait_for_active_controller(node, name='robotiq_gripper_controller',
                               timeout=STARTUP_TIMEOUT):
    """Wait until the controller is active.

    The action server exists as soon as the controller is configured, but goals
    are rejected while it is still activating - so waiting for the server alone
    is not enough.
    """
    client = node.create_client(ListControllers, '/controller_manager/list_controllers')
    assert client.wait_for_service(timeout_sec=30), 'no list_controllers service'

    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        request = ListControllers.Request()
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=10)
        response = future.result()
        if response is not None:
            for controller in response.controller:
                if controller.name == name and controller.state == 'active':
                    return True
        time.sleep(0.2)
    raise AssertionError(f'controller {name} did not become active')


def test_gripper_action_closes_and_opens(node, stack):
    """The gripper action works and the position follows the command."""
    _wait_for_active_controller(node)

    client = ActionClient(node, ParallelGripperCommand, ACTION)
    assert client.wait_for_server(timeout_sec=STARTUP_TIMEOUT), (
        f'no action server on {ACTION}'
    )

    def send(position):
        goal = ParallelGripperCommand.Goal()
        goal.command.name = [GRIPPER_JOINT]
        goal.command.position = [position]
        goal.command.effort = [10.0]
        future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, future, timeout_sec=30)
        handle = future.result()
        assert handle is not None and handle.accepted, 'goal was rejected'
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(node, result_future, timeout_sec=30)
        return result_future.result()

    # close
    result = send(0.79)
    assert result is not None, 'close goal returned no result'
    assert result.status == GoalStatus.STATUS_SUCCEEDED, f'close failed: {result.status}'
    time.sleep(1.0)

    def gripper_position():
        positions = {}

        def callback(msg):
            if GRIPPER_JOINT in msg.name:
                positions['value'] = msg.position[msg.name.index(GRIPPER_JOINT)]

        subscription = node.create_subscription(JointState, '/joint_states', callback, 10)
        deadline = time.monotonic() + 10.0
        try:
            while 'value' not in positions and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.2)
        finally:
            node.destroy_subscription(subscription)
        return positions.get('value')

    closed = gripper_position()
    assert closed is not None, 'no gripper position in /joint_states'
    assert closed > 0.5, f'gripper did not close (position {closed})'

    # open again
    result = send(0.0)
    assert result is not None, 'open goal returned no result'
    assert result.status == GoalStatus.STATUS_SUCCEEDED, f'open failed: {result.status}'
    time.sleep(1.0)
    opened = gripper_position()
    assert opened is not None
    assert opened < 0.2, f'gripper did not open (position {opened})'
