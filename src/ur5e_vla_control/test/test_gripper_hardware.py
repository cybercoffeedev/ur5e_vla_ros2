#!/usr/bin/env python3
"""L3: hardware-in-the-loop test of the real gripper.

Opt-in, because it moves physical hardware:

    UR5E_VLA_HITL=1 pytest src/ur5e_vla_control/test/test_gripper_hardware.py

The arm stays on mock hardware - only the gripper has to be connected
(USB CDC-ACM, slave address 2). The test closes the jaws, checks that the
position rises, and opens them again.

Keep the gripper empty and the e-stop within reach.
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
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState

GRIPPER_JOINT = 'robotiq_85_left_knuckle_joint'
ACTION = '/robotiq_gripper_controller/gripper_cmd'
CLOSED = 0.79
OPEN = 0.0
STARTUP_TIMEOUT = 90.0

pytestmark = pytest.mark.skipif(
    os.environ.get('UR5E_VLA_HITL') != '1',
    reason='set UR5E_VLA_HITL=1 to run the hardware-in-the-loop test',
)


@pytest.fixture(scope='module')
def stack():
    log = tempfile.NamedTemporaryFile(prefix='ur5e_l3_', suffix='.log', delete=False)
    process = subprocess.Popen(
        [
            'ros2', 'launch', 'ur5e_vla_control', 'ur5e_control.launch.py',
            'use_mock_hardware:=true', 'use_mock_gripper:=false',
        ],
        stdout=log,
        stderr=subprocess.STDOUT,
        # Own process group, so that the teardown can stop the child nodes too
        # (a leftover robot_state_publisher keeps spamming /robot_description).
        start_new_session=True,
    )
    process.log_file = log.name
    yield process
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
    node = Node('ur5e_l3_test')
    yield node
    node.destroy_node()
    rclpy.shutdown()


def _gripper_position(node, timeout=15.0):
    positions = {}

    def callback(msg):
        if GRIPPER_JOINT in msg.name:
            positions['value'] = msg.position[msg.name.index(GRIPPER_JOINT)]

    subscription = node.create_subscription(JointState, '/joint_states', callback, 10)
    deadline = time.monotonic() + timeout
    try:
        while 'value' not in positions and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        node.destroy_subscription(subscription)
    return positions.get('value')


def _send(node, client, position):
    goal = ParallelGripperCommand.Goal()
    goal.command.name = [GRIPPER_JOINT]
    goal.command.position = [position]
    goal.command.effort = [20.0]
    future = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, future, timeout_sec=30)
    handle = future.result()
    assert handle is not None and handle.accepted, f'goal {position} was rejected'
    result_future = handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future, timeout_sec=30)
    result = result_future.result()
    assert result.status == GoalStatus.STATUS_SUCCEEDED, f'goal {position} failed'


def test_gripper_closes_and_opens(node, stack):
    """The real gripper follows the action: closed, then open again."""
    client = ActionClient(node, ParallelGripperCommand, ACTION)
    assert client.wait_for_server(timeout_sec=STARTUP_TIMEOUT), (
        f'no action server on {ACTION} - is the gripper connected?'
    )

    before = _gripper_position(node)
    assert before is not None, 'no gripper position in /joint_states'

    _send(node, client, CLOSED)
    time.sleep(1.5)
    closed = _gripper_position(node)
    assert closed is not None
    assert closed > 0.5, f'gripper did not close (position {closed}, was {before})'

    _send(node, client, OPEN)
    time.sleep(1.5)
    opened = _gripper_position(node)
    assert opened is not None
    assert opened < 0.2, f'gripper did not open (position {opened})'
