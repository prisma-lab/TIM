"""Exercise the public topic/action protocol against a controllable ROS server."""

import json
from pathlib import Path
import signal
import subprocess
import tempfile
import threading
import time
from uuid import uuid4

from control_msgs.action import FollowJointTrajectory
from ament_index_python.packages import get_package_prefix
import pytest
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import yaml


def wait_for(predicate, timeout=4.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.01)
    assert predicate(), 'Timed out waiting for the expected ROS event'


class Controller(Node):
    def __init__(self, prefix, with_server=True):
        super().__init__('test_controller')
        self.mode = 'success'
        self.position = 0.75
        self.publish_feedback = True
        self.requests = []
        self.cancelled = 0
        self.accept_delay = 0.0
        self.cancel_complete = threading.Event()
        self.cancel_complete.set()
        self.release = threading.Event()
        self.feedback = self.create_publisher(JointState, prefix + '/joints', 10)
        self.timer = self.create_timer(0.02, self.publish_joint)
        self.server = None
        if with_server:
            self.server = ActionServer(
                self, FollowJointTrajectory, prefix + '/trajectory',
                execute_callback=self.execute,
                goal_callback=self.accept,
                cancel_callback=lambda _: CancelResponse.ACCEPT,
                callback_group=ReentrantCallbackGroup())

    def publish_joint(self):
        if self.publish_feedback:
            self.feedback.publish(JointState(
                name=['robotiq_85_left_knuckle_joint'], position=[self.position]))

    def accept(self, request):
        self.requests.append(request)
        time.sleep(self.accept_delay)
        return GoalResponse.REJECT if self.mode == 'reject' else GoalResponse.ACCEPT

    def execute(self, handle):
        deadline = time.monotonic() + 3.0
        while self.mode in ('hold', 'hang') and not self.release.is_set():
            if handle.is_cancel_requested:
                self.cancelled += 1
                self.cancel_complete.wait(timeout=4.0)
                handle.canceled()
                return FollowJointTrajectory.Result()
            if time.monotonic() > deadline:
                handle.abort()
                return FollowJointTrajectory.Result(error_code=-4)
            time.sleep(0.01)
        if self.mode == 'abort':
            handle.abort()
            return FollowJointTrajectory.Result(error_code=-5, error_string='test tolerance failure')
        if self.mode != 'wrong_position':
            self.position = handle.request.trajectory.points[-1].positions[0]
            self.publish_joint()
        handle.succeed()
        return FollowJointTrajectory.Result()


class Rig:
    def __init__(self, with_server=True, overrides=None):
        prefix = '/gripper_test_' + uuid4().hex
        self.controller = Controller(prefix, with_server)
        settings = {
            'command_topic': prefix + '/command',
            'status_topic': prefix + '/status',
            'seed_state_topic': prefix + '/state',
            'failure_fact': 'gripper.failed',
            'gripper.joint_states_topic': prefix + '/joints',
            'gripper.action_name': prefix + '/trajectory',
            'gripper.server_timeout': 0.6,
            'gripper.execution_timeout': 1.5,
            'gripper.feedback_timeout': 0.4,
            'gripper.motion_duration': 0.1,
            # No /clock is published: watchdogs must use wall time regardless.
            'use_sim_time': True,
            'primitives': ['open_gripper', 'close_gripper', 'half_gripper'],
        }
        for name, position, fact in (
                ('open_gripper', 0.0, 'gripper.open'),
                ('close_gripper', 0.75, 'gripper.closed'),
                ('half_gripper', 0.3, 'gripper.half')):
            settings[name + '.plugin'] = 'ur10_primitives/GripperPrimitive'
            settings[name + '.target_position'] = position
            settings[name + '.state_fact'] = fact
        settings.update(overrides or {})
        self.temp = tempfile.TemporaryDirectory(prefix='gripper_plugins_')
        params = Path(self.temp.name) / 'params.yaml'
        params.write_text(yaml.safe_dump({'/**': {'ros__parameters': settings}}))
        self.log = tempfile.TemporaryFile(mode='w+')
        executable = Path(get_package_prefix('primitive_manager')) / 'lib/primitive_manager/primitive_manager'
        self.process = subprocess.Popen(
            [str(executable), '--ros-args', '--params-file', str(params)],
            stdout=self.log, stderr=subprocess.STDOUT)
        self.observer = Node('test_observer')
        self.statuses = []
        self.states = {}
        self.pub = self.observer.create_publisher(String, prefix + '/command', 10)
        self.status_sub = self.observer.create_subscription(
            String, prefix + '/status', lambda msg: self.statuses.append(json.loads(msg.data)), 10)
        self.state_sub = self.observer.create_subscription(
            String, prefix + '/state', self.record_state, 100)
        self.executor = MultiThreadedExecutor(num_threads=4)
        for node in (self.controller, self.observer):
            self.executor.add_node(node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()
        try:
            wait_for(lambda: self.pub.get_subscription_count() == 1, timeout=8)
            wait_for(lambda: self.states.get('gripper.closed') is True)
        except AssertionError:
            self.log.seek(0)
            output = self.log.read()
            self.close()
            pytest.fail('Manager failed to start: ' + output)

    def record_state(self, msg):
        self.states[msg.data.lstrip('-')] = not msg.data.startswith('-')

    def send(self, command):
        self.pub.publish(String(data=command))

    def saw(self, command, status):
        return any(item['command'] == command and item['status'] == status
                   for item in self.statuses)

    def close(self):
        self.controller.cancel_complete.set()
        self.controller.release.set()
        self.process.send_signal(signal.SIGINT)
        try:
            self.process.wait(timeout=4)
        except subprocess.TimeoutExpired:
            self.process.kill()
            self.process.wait(timeout=4)
        self.executor.shutdown(timeout_sec=3)
        self.thread.join(timeout=3)
        if self.controller.server is not None:
            self.controller.server.destroy()
        for node in (self.controller, self.observer):
            node.destroy_node()
        self.log.close()
        self.temp.cleanup()


@pytest.fixture
def rig(request):
    rclpy.init()
    options = getattr(request, 'param', True)
    fixture = None
    try:
        fixture = Rig(**options) if isinstance(options, dict) else Rig(with_server=options)
        yield fixture
    finally:
        if fixture is not None:
            fixture.close()
        rclpy.try_shutdown()


def test_success_feedback_and_duplicate_commands(rig):
    rig.controller.mode = 'hold'
    rig.send('open_gripper')
    wait_for(lambda: rig.saw('open_gripper', 'running'))
    wait_for(lambda: len(rig.controller.requests) == 1)
    for _ in range(3):
        rig.send('open_gripper')
    rig.send('close_gripper')
    wait_for(lambda: rig.saw('close_gripper', 'rejected'))
    assert len(rig.controller.requests) == 1
    assert not rig.saw('open_gripper', 'succeeded')
    assert rig.states.get('gripper.open') is False
    assert rig.controller.requests[0].trajectory.joint_names == ['robotiq_85_left_knuckle_joint']
    assert list(rig.controller.requests[0].trajectory.points[0].positions) == [0.0]
    rig.controller.release.set()
    wait_for(lambda: rig.saw('open_gripper', 'succeeded'))
    wait_for(lambda: rig.states.get('gripper.open') is True)
    count = len(rig.statuses)
    rig.send('open_gripper')
    wait_for(lambda: len(rig.statuses) > count)
    assert len(rig.controller.requests) == 1


def test_rejection_is_latched_until_reset(rig):
    rig.controller.mode = 'reject'
    rig.send('open_gripper')
    wait_for(lambda: rig.saw('open_gripper', 'failed'))
    wait_for(lambda: rig.states.get('gripper.failed') is True)
    rig.send('open_gripper')
    wait_for(lambda: rig.saw('open_gripper', 'rejected'))
    assert len(rig.controller.requests) == 1
    rig.controller.mode = 'success'
    rig.send('reset')
    wait_for(lambda: rig.saw('reset', 'succeeded'))
    rig.send('open_gripper')
    wait_for(lambda: rig.saw('open_gripper', 'succeeded'))
    assert len(rig.controller.requests) == 2


def test_controller_abort_does_not_satisfy_seed_goal(rig):
    rig.controller.mode = 'abort'
    rig.send('open_gripper')
    wait_for(lambda: rig.saw('open_gripper', 'failed'))
    wait_for(lambda: rig.states.get('gripper.failed') is True)
    assert not rig.saw('open_gripper', 'succeeded')
    assert rig.states.get('gripper.open') is False


def test_action_success_alone_is_not_enough(rig):
    rig.controller.mode = 'wrong_position'
    rig.send('open_gripper')
    wait_for(lambda: rig.saw('open_gripper', 'failed'))
    assert not rig.saw('open_gripper', 'succeeded')
    assert rig.states.get('gripper.open') is False


def test_timeout_cancels_with_frozen_simulation_clock(rig):
    rig.controller.mode = 'hang'
    rig.send('open_gripper')
    wait_for(lambda: rig.saw('open_gripper', 'failed'))
    wait_for(lambda: rig.controller.cancelled == 1)
    assert not rig.saw('open_gripper', 'succeeded')
    assert len(rig.controller.requests) == 1


@pytest.mark.parametrize('rig', [False], indirect=True)
def test_missing_action_server(rig):
    rig.send('open_gripper')
    wait_for(lambda: rig.saw('open_gripper', 'failed'))
    assert not rig.saw('open_gripper', 'succeeded')


def test_stale_feedback_clears_seed_state(rig):
    rig.controller.publish_feedback = False
    wait_for(lambda: rig.states.get('gripper.closed') is False)


def test_unknown_command_does_not_move_or_latch_failure(rig):
    rig.send('not_a_primitive')
    wait_for(lambda: rig.saw('not_a_primitive', 'rejected'))
    assert rig.controller.requests == []
    assert rig.states.get('gripper.failed') is False


def test_cancel_waits_for_controller_before_reset_or_new_motion(rig):
    rig.controller.mode = 'hold'
    rig.controller.cancel_complete.clear()
    rig.send('open_gripper')
    wait_for(lambda: len(rig.controller.requests) == 1)
    rig.send('cancel')
    wait_for(lambda: rig.controller.cancelled == 1)
    wait_for(lambda: rig.states.get('cancelling(open_gripper)') is True)
    rig.send('reset')
    rig.send('half_gripper')
    wait_for(lambda: rig.saw('reset', 'rejected') and rig.saw('half_gripper', 'rejected'))
    assert len(rig.controller.requests) == 1
    rig.controller.cancel_complete.set()
    wait_for(lambda: rig.saw('open_gripper', 'cancelled'))
    wait_for(lambda: rig.states.get('cancelled(open_gripper)') is True)
    assert rig.states['cancelling(open_gripper)'] is False
    rig.controller.mode = 'success'
    rig.send('half_gripper')
    wait_for(lambda: rig.saw('half_gripper', 'succeeded'))
    assert len(rig.controller.requests) == 2


@pytest.mark.parametrize('rig', [{'overrides': {'gripper.execution_timeout': 0.2}}], indirect=True)
def test_late_goal_acceptance_after_timeout_is_cancelled(rig):
    rig.controller.mode = 'hold'
    rig.controller.accept_delay = 0.6
    rig.controller.cancel_complete.clear()
    rig.send('open_gripper')
    wait_for(lambda: rig.saw('open_gripper', 'failed'))
    rig.send('reset')
    wait_for(lambda: rig.saw('reset', 'rejected'))
    wait_for(lambda: rig.controller.cancelled == 1)
    assert len(rig.controller.requests) == 1
    rig.controller.cancel_complete.set()
    # Keep trying until the controller's terminal result releases the busy slot.
    deadline = time.monotonic() + 3
    while not rig.saw('reset', 'succeeded') and time.monotonic() < deadline:
        rig.send('reset')
        time.sleep(0.05)
    assert rig.saw('reset', 'succeeded')


def test_completed_command_needs_explicit_reset_to_repeat(rig):
    rig.send('open_gripper')
    wait_for(lambda: rig.saw('open_gripper', 'succeeded'))
    wait_for(lambda: rig.states.get('succeeded(open_gripper)') is True)
    rig.controller.position = 0.75
    wait_for(lambda: rig.states.get('gripper.closed') is True)
    before = len(rig.statuses)
    rig.send('open_gripper')
    wait_for(lambda: len(rig.statuses) > before)
    assert len(rig.controller.requests) == 1
    # Execution history and observed position remain distinct.
    assert rig.states['succeeded(open_gripper)'] is True
    assert rig.states['gripper.open'] is False
    rig.send('reset')
    wait_for(lambda: rig.saw('reset', 'succeeded'))
    wait_for(lambda: rig.states.get('succeeded(open_gripper)') is False)
    rig.send('open_gripper')
    wait_for(lambda: len(rig.controller.requests) == 2)
    wait_for(lambda: rig.states.get('gripper.open') is True)


def test_invalid_plugin_arguments_report_failure_and_recover(rig):
    rig.send('open_gripper(unexpected)')
    wait_for(lambda: rig.saw('open_gripper(unexpected)', 'failed'))
    assert rig.process.poll() is None
    assert len(rig.controller.requests) == 0
    rig.send('reset')
    wait_for(lambda: rig.saw('reset', 'succeeded'))
    rig.send('open_gripper')
    wait_for(lambda: rig.saw('open_gripper', 'succeeded'))


@pytest.mark.parametrize('command', ['open_gripper(', 'open_gripper([a,b))', 'open_gripper(a,)'])
def test_malformed_command_is_rejected_without_motion(rig, command):
    rig.send(command)
    wait_for(lambda: rig.saw(command, 'rejected'))
    assert rig.controller.requests == []
    assert rig.states['gripper.failed'] is False


def test_new_plugin_instance_is_selected_from_configuration(rig):
    rig.send('half_gripper')
    wait_for(lambda: rig.saw('half_gripper', 'succeeded'))
    wait_for(lambda: rig.states.get('gripper.half') is True)
    assert list(rig.controller.requests[0].trajectory.points[0].positions) == [0.3]
    assert rig.states['gripper.open'] is False
    assert rig.states['gripper.closed'] is False
