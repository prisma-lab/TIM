"""Actual C++ plugins with mock ROS services; no robot drivers or motion actions."""
import copy
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import tempfile
import threading
import time

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from inverse_msgs.srv import EnqueueTrigger, ReachPosition
import pytest
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Int64, String
import yaml


def wait_for(predicate, timeout=6):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(.01)
    assert predicate(), 'Timed out waiting for ROS result'


class Provider(Node):
    def __init__(self, event_type=String):
        super().__init__('mock_motion_service_provider')
        self.requests = []
        self.event_type = event_type
        self.next_id = 9223372036854775808 if event_type is String else 9223372036854775700
        self.response_ids = None
        self.accept = True
        self.automatic = False
        self.hold_response = False
        self.release = threading.Event()
        self.start_pub = self.create_publisher(event_type, '/service_test/start', 100)
        self.end_pub = self.create_publisher(event_type, '/service_test/end', 100)
        callbacks = ReentrantCallbackGroup()
        self.service_servers = [self.create_service(
            ReachPosition, '/service_test/move',
            lambda request, response: self.call('move', request, response),
            callback_group=callbacks)]
        for name in ('pick', 'place'):
            self.service_servers.append(self.create_service(
                EnqueueTrigger, '/service_test/' + name,
                lambda request, response, name=name: self.call(name, request, response),
                callback_group=callbacks))

    def call(self, operation, request, response):
        ids = self.response_ids
        if ids is None:
            ids = [self.next_id + i for i in range(3)]
            self.next_id += 3
        self.requests.append((operation, copy.deepcopy(request), list(ids)))
        if self.automatic and self.accept and ids:
            self.start(ids[0])
            self.end(ids[-1])  # Deliberately before the service response.
        while self.hold_response and not self.release.wait(.01):
            pass
        response.success = self.accept
        response.motion_ids = ids
        return response

    def start(self, motion_id):
        data = str(motion_id) if self.event_type is String else motion_id
        self.start_pub.publish(self.event_type(data=data))

    def end(self, motion_id):
        data = str(motion_id) if self.event_type is String else motion_id
        self.end_pub.publish(self.event_type(data=data))


class Rig:
    def __init__(self, full=False, response_timeout=2., execution_timeout=4.,
                 event_message_type='std_msgs/msg/String'):
        self.provider = Provider(Int64 if event_message_type == 'std_msgs/msg/Int64' else String)
        self.observer = Node('service_adapter_observer')
        self.statuses = []
        self.facts = {}
        self.command = self.observer.create_publisher(String, '/service_test/command', 10)
        self.status_sub = self.observer.create_subscription(
            String, '/service_test/status', lambda message: self.statuses.append(json.loads(message.data)), 100)
        self.state_sub = self.observer.create_subscription(
            String, '/service_test/state',
            lambda message: self.facts.update({message.data.lstrip('-'): not message.data.startswith('-')}), 100)
        package = Path(__file__).resolve().parents[1]
        config = yaml.safe_load((package / 'config/services.yaml').read_text())['ur10_service_manager']['ros__parameters']
        config.update(command_topic='/service_test/command', status_topic='/service_test/status',
                      seed_state_topic='/service_test/state', primitives=['move', 'pick', 'place'] if full else ['move'])
        config['services'].update(start_topic='/service_test/start',
                                  end_topic='/service_test/end', response_timeout=response_timeout,
                                  execution_timeout=execution_timeout,
                                  event_message_type=event_message_type)
        config['move']['service'] = '/service_test/move'
        config['pick']['service'] = '/service_test/pick'
        config['place']['service'] = '/service_test/place'
        self.temp = tempfile.TemporaryDirectory()
        parameters = Path(self.temp.name) / 'params.yaml'
        parameters.write_text(yaml.safe_dump({'/**': {'ros__parameters': config}}))
        self.log = tempfile.TemporaryFile(mode='w+')
        executable = Path(get_package_prefix('primitive_manager')) / 'lib/primitive_manager/primitive_manager'
        self.process = subprocess.Popen([str(executable), '--ros-args', '--params-file', str(parameters)],
                                        stdout=self.log, stderr=subprocess.STDOUT)
        self.executor = MultiThreadedExecutor(num_threads=4)
        self.executor.add_node(self.provider)
        self.executor.add_node(self.observer)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()
        try:
            wait_for(lambda: self.command.get_subscription_count() == 1)
            wait_for(lambda: self.provider.start_pub.get_subscription_count() == (3 if full else 1))
            wait_for(lambda: self.provider.end_pub.get_subscription_count() == (3 if full else 1))
        except AssertionError:
            self.log.seek(0)
            output = self.log.read()
            self.close()
            pytest.fail(output)

    def send(self, command):
        self.command.publish(String(data=command))

    def saw(self, command, status, since=0):
        return any(item['command'] == command and item['status'] == status for item in self.statuses[since:])

    def run(self, command):
        since = len(self.statuses)
        self.send(command)
        wait_for(lambda: self.saw(command, 'succeeded', since) or self.saw(command, 'failed', since))
        assert self.saw(command, 'succeeded', since), self.statuses[since:]
        wait_for(lambda: self.facts.get('succeeded(' + command + ')') is True)

    def close(self):
        self.provider.release.set()
        self.process.send_signal(signal.SIGINT)
        try:
            self.process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            self.process.kill()
            self.process.wait(timeout=3)
        self.executor.shutdown(timeout_sec=3)
        self.thread.join(timeout=3)
        self.provider.destroy_node()
        self.observer.destroy_node()
        self.log.close()
        self.temp.cleanup()


@pytest.fixture
def rig(request):
    options = getattr(request, 'param', {})
    rclpy.init()
    test = None
    try:
        test = Rig(**options)
        yield test
    finally:
        if test is not None:
            test.close()
        rclpy.try_shutdown()


@pytest.mark.parametrize('frame', ['pick', 'via(bus_bar)', 'obs(bus_bar)'])
def test_move_requests_frame_origin_without_target_publisher(rig, frame):
    command = f'move({frame})'
    fact = f'arm.at({frame})'
    rig.send(command)
    wait_for(lambda: len(rig.provider.requests) == 1)
    _, request, ids = rig.provider.requests[0]
    assert request.desired_pos.header.frame_id == frame
    position = request.desired_pos.pose.position
    assert [position.x, position.y, position.z] == [0., 0., 0.]
    orientation = request.desired_pos.pose.orientation
    assert [orientation.x, orientation.y, orientation.z, orientation.w] == [0., 0., 0., 1.]
    assert request.desired_pos.header.stamp.sec == 0
    assert request.desired_pos.header.stamp.nanosec == 0
    assert request.max_vel == .05
    assert request.immediate_execution is False
    rig.provider.start(ids[0])
    wait_for(lambda: any('Started at /motion_start ID' in item['detail'] for item in rig.statuses))
    rig.provider.end(ids[1])
    time.sleep(.1)
    assert not rig.saw(command, 'succeeded')
    assert rig.facts[fact] is False
    rig.provider.end(ids[-1])
    wait_for(lambda: rig.facts.get(fact) is True)
    assert rig.observer.count_subscribers('/target_poses') == 0


@pytest.mark.parametrize('ids', [None, [7]])
def test_early_end_before_response_and_before_start_is_not_lost(rig, ids):
    rig.provider.response_ids = ids
    rig.provider.hold_response = True
    rig.send('move(pick)')
    wait_for(lambda: rig.provider.requests)
    ids = rig.provider.requests[0][2]
    rig.provider.end(ids[-1])
    time.sleep(.1)
    assert not rig.saw('move(pick)', 'succeeded')
    rig.provider.release.set()
    wait_for(lambda: rig.saw('move(pick)', 'succeeded'))


def test_duplicate_commands_and_unrelated_events_do_not_complete_or_resubmit(rig):
    rig.send('move(pick)')
    wait_for(lambda: rig.provider.requests)
    for _ in range(3):
        rig.send('move(pick)')
    for text in ('-1', 'unrelated', '18446744073709551616', ''):
        rig.provider.end_pub.publish(String(data=text))
    rig.send('move(place)')
    wait_for(lambda: rig.saw('move(place)', 'rejected'))
    assert len(rig.provider.requests) == 1
    assert not rig.saw('move(pick)', 'succeeded')
    rig.provider.end(rig.provider.requests[0][2][-1])
    wait_for(lambda: rig.saw('move(pick)', 'succeeded'))
    rig.send('move(pick)')
    time.sleep(.1)
    assert len(rig.provider.requests) == 1


def test_old_completion_cannot_finish_next_request(rig):
    rig.provider.automatic = True
    rig.run('move(pick)')
    previous = rig.provider.requests[0][2][-1]
    rig.provider.automatic = False
    rig.send('move(place)')
    wait_for(lambda: len(rig.provider.requests) == 2)
    rig.provider.end(previous)
    time.sleep(.1)
    assert not rig.saw('move(place)', 'succeeded')
    rig.provider.end(rig.provider.requests[-1][2][-1])
    wait_for(lambda: rig.saw('move(place)', 'succeeded'))


@pytest.mark.parametrize('ids', [[], [5, 5]])
def test_invalid_accepted_ids_fail_and_keep_execution_slot(rig, ids):
    rig.provider.response_ids = ids
    rig.send('move(pick)')
    wait_for(lambda: rig.saw('move(pick)', 'failed'))
    rig.send('reset')
    wait_for(lambda: rig.saw('reset', 'rejected'))
    assert rig.facts['arm.at(pick)'] is False


def test_service_rejection_schedules_nothing_and_allows_reset(rig):
    rig.provider.accept = False
    rig.send('move(pick)')
    wait_for(lambda: rig.saw('move(pick)', 'failed'))
    rig.send('reset')
    wait_for(lambda: rig.saw('reset', 'succeeded'))
    rig.provider.accept = rig.provider.automatic = True
    rig.run('move(pick)')


def test_missing_event_publisher_prevents_dispatch(rig):
    rig.provider.destroy_publisher(rig.provider.end_pub)
    time.sleep(.3)
    rig.send('move(pick)')
    wait_for(lambda: rig.saw('move(pick)', 'failed'))
    assert not rig.provider.requests


@pytest.mark.parametrize('rig', [{'response_timeout': .2, 'execution_timeout': .5}], indirect=True)
def test_response_timeout_keeps_slot_until_late_response_and_end(rig):
    rig.provider.hold_response = True
    rig.send('move(pick)')
    wait_for(lambda: rig.saw('move(pick)', 'failed'))
    rig.send('reset')
    wait_for(lambda: rig.saw('reset', 'rejected'))
    rig.provider.end(rig.provider.requests[0][2][-1])
    rig.provider.release.set()
    time.sleep(.2)
    rig.send('reset')
    wait_for(lambda: rig.saw('reset', 'succeeded'))
    assert not rig.saw('move(pick)', 'succeeded')


@pytest.mark.parametrize('rig', [{'execution_timeout': .25}], indirect=True)
def test_missing_end_times_out_without_claiming_completion(rig):
    rig.send('move(pick)')
    wait_for(lambda: rig.saw('move(pick)', 'failed'))
    rig.send('reset')
    wait_for(lambda: rig.saw('reset', 'rejected'))
    assert not rig.saw('move(pick)', 'succeeded')


def test_cancel_reports_unsupported_stop_and_waits_for_remote_end(rig):
    rig.send('move(pick)')
    wait_for(lambda: rig.provider.requests)
    rig.send('cancel')
    wait_for(lambda: rig.saw('move(pick)', 'failed'))
    assert any('robot not stopped' in item['detail'] for item in rig.statuses)
    rig.send('reset')
    wait_for(lambda: rig.saw('reset', 'rejected'))
    rig.provider.end(rig.provider.requests[0][2][-1])
    time.sleep(.15)
    rig.send('reset')
    wait_for(lambda: rig.saw('reset', 'succeeded'))
    assert not rig.saw('move(pick)', 'cancelled')


@pytest.mark.parametrize('command', ['move', 'move(pick,place)'])
def test_invalid_arguments_are_rejected_before_service_call(rig, command):
    rig.send(command)
    wait_for(lambda: rig.saw(command, 'failed'))
    assert not rig.provider.requests


SEQUENCE = ['move(pick_approach)', 'move(pick)', 'pick',
            'move(pick_approach)', 'move(place_approach)', 'move(place)',
            'place', 'move(place_approach)']


@pytest.mark.parametrize('rig', [{'full': True, 'event_message_type': 'std_msgs/msg/Int64'}], indirect=True)
def test_hardware_int64_events_complete_move_pick_and_place(rig):
    rig.send('move(pick_approach)')
    wait_for(lambda: len(rig.provider.requests) == 1)
    ids = rig.provider.requests[0][2]
    rig.provider.end(-1)
    rig.provider.start(ids[0])
    rig.provider.end(ids[1])
    time.sleep(.1)
    assert not rig.saw('move(pick_approach)', 'succeeded')
    rig.provider.end(ids[-1])
    wait_for(lambda: rig.facts.get('arm.at(pick_approach)') is True)
    rig.provider.automatic = True
    for command in SEQUENCE[1:]:
        rig.run(command)
    assert len(rig.provider.requests) == 8
    wait_for(lambda: rig.facts.get('gripper.open') is True)


@pytest.mark.parametrize('rig', [{'full': True}], indirect=True)
def test_full_sequence_uses_empty_gripper_requests_and_shared_task_effects(rig):
    rig.provider.automatic = True
    for command in SEQUENCE:
        rig.run(command)
    assert [operation for operation, _, _ in rig.provider.requests] == [
        'move', 'move', 'pick', 'move', 'move', 'move', 'place', 'move']
    for operation, request, _ in rig.provider.requests:
        if operation in ('pick', 'place'):
            assert request.get_fields_and_field_types() == {}
    wait_for(lambda: rig.facts.get('gripper.open') is True)
    assert rig.facts['gripper.closed'] is False
    assert rig.facts['arm.at(place_approach)'] is True


@pytest.mark.parametrize('rig', [{'full': True}], indirect=True)
def test_gripper_commands_need_no_object_or_arm_bookkeeping(rig):
    rig.provider.automatic = True
    for command in ('pick', 'place', 'pick'):
        rig.run(command)
    assert [operation for operation, _, _ in rig.provider.requests] == ['pick', 'place', 'pick']
    wait_for(lambda: rig.facts.get('gripper.closed') is True)
    assert rig.facts['gripper.open'] is False


@pytest.mark.parametrize('rig', [{'full': True}], indirect=True)
@pytest.mark.parametrize('command', ['pick(workpiece,pick)', 'place(workpiece,place)'])
def test_gripper_rejects_unexpected_arguments(rig, command):
    rig.send(command)
    wait_for(lambda: rig.saw(command, 'failed'))
    assert not rig.provider.requests


def test_nested_frame_moves_update_exact_seed_facts(rig):
    rig.provider.automatic = True
    rig.run('move(via(bus_bar))')
    wait_for(lambda: rig.facts.get('arm.at(via(bus_bar))') is True)
    rig.provider.automatic = False
    rig.send('move(obs(bus_bar))')
    wait_for(lambda: len(rig.provider.requests) == 2)
    wait_for(lambda: rig.facts.get('arm.at(via(bus_bar))') is False)
    wait_for(lambda: rig.facts.get('arm.at(obs(bus_bar))') is False)
    _, request, ids = rig.provider.requests[-1]
    assert request.desired_pos.header.frame_id == 'obs(bus_bar)'
    rig.provider.end(ids[-1])
    wait_for(lambda: rig.facts.get('arm.at(obs(bus_bar))') is True)
    assert rig.facts['arm.at(via(bus_bar))'] is False


@pytest.mark.parametrize('rig', [{'full': True}], indirect=True)
@pytest.mark.parametrize('source', ['named_task', 'pddl', 'pddl_pick_only'])
def test_seed_executes_named_task_or_pddl_plan_against_mock_services(rig, source):
    if not shutil.which('swipl'):
        pytest.skip('SEED runtime unavailable')
    rig.provider.automatic = True
    # Isolate SEED learning/log files from the user's source checkout.
    with tempfile.TemporaryDirectory(prefix='tim-service-seed-') as temporary:
        root = Path(temporary)
        prefix = root / 'install/seed'
        marker = prefix / 'share/ament_index/resource_index/packages'
        marker.mkdir(parents=True)
        (marker / 'seed').touch()
        (prefix / 'share/seed').mkdir()
        seed_source = (Path(get_package_share_directory('seed')) / '../../../../src/seed').resolve()
        for folder in ('LTM', 'learning'):
            shutil.copytree(seed_source / folder, root / 'src/seed' / folder)
        (root / 'src/seed/log').mkdir()
        # The profile is unchanged; only the test manager's topics are remapped.
        ltm = root / 'src/seed/LTM/seed_ur10_services_LTM.prolog'
        ltm.write_text(ltm.read_text().replace('ur10/hardware/primitives/command', 'service_test/command'))
        env = dict(os.environ, AMENT_PREFIX_PATH=str(prefix) + ':' + os.environ['AMENT_PREFIX_PATH'])
        executable = Path(get_package_prefix('seed')) / 'lib/seed/seed'
        with (root / 'output.log').open('w+') as output:
            seed = subprocess.Popen([str(executable), 'ur10_services', '--ros-args',
                                     '-r', '/seed_ur10_services/state:=/service_test/state'],
                                    stdin=subprocess.PIPE, stdout=output, stderr=subprocess.STDOUT,
                                    env=env, text=True)
            try:
                wait_for(lambda: rig.observer.count_subscribers('/service_test/state') >= 2)
                if source == 'named_task':
                    seed.stdin.write('hardware_pick_place_demo\n')
                    seed.stdin.flush()
                else:
                    pddl = Path(get_package_share_directory('task_planner')) / 'pddl/hardware'
                    problem = (pddl / 'problem.pddl').read_text()
                    if source == 'pddl_pick_only':
                        problem = problem.replace(
                            '(and (placed) (hand-empty) (arm-at place_approach))', '(holding)')
                    problem_file = root / 'problem.pddl'
                    problem_file.write_text(problem)
                    planner = Path(get_package_prefix('task_planner')) / 'lib/task_planner/pddl_to_seed_hardware'
                    result = subprocess.run([
                        str(planner), '--domain', str(pddl / 'domain.pddl'),
                        '--problem', str(problem_file), '--execute'],
                        capture_output=True, text=True, timeout=20)
                    assert result.returncode == 0, result.stdout + result.stderr
                expected = ['move', 'move', 'pick', 'move', 'move', 'move', 'place', 'move']
                if source == 'pddl_pick_only':
                    expected = expected[:3]
                wait_for(lambda: len(rig.provider.requests) == len(expected), timeout=15)
                assert [operation for operation, _, _ in rig.provider.requests] == expected
                goal = 'gripper.closed' if source == 'pddl_pick_only' else 'gripper.open'
                wait_for(lambda: rig.facts.get(goal) is True)
                def completed():
                    output.seek(0)
                    return 'sequence accomplished!' in output.read()
                wait_for(completed)
            finally:
                seed.send_signal(signal.SIGINT)
                try:
                    seed.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    seed.kill()
                    seed.wait()
