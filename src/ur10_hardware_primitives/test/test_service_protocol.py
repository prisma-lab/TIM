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
from geometry_msgs.msg import PoseStamped, TransformStamped
from inverse_msgs.srv import PointToPointMotion
import pytest
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster
import yaml

try:
    from inverse_msgs.srv import Pick, Place
    HAS_GRIPPER = True
except ImportError:
    HAS_GRIPPER = False

if os.environ.get('TIM_REQUIRE_GRIPPER_SERVICES') == '1' and not HAS_GRIPPER:
    raise RuntimeError('Source the test overlay containing Pick/Place before running the full check')


def wait_for(predicate, timeout=6):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(.01)
    assert predicate(), 'Timed out waiting for ROS result'


class Provider(Node):
    def __init__(self):
        super().__init__('mock_motion_service_provider')
        self.requests = []
        self.next_id = 9007199254740993  # Exercises exact int64 -> string matching.
        self.response_ids = None
        self.accept = True
        self.automatic = False
        self.hold_response = False
        self.release = threading.Event()
        self.current = [.1, .2, .3]
        self.target_frame = 'base_link'
        self.stamp_offset = 0
        self.publish_tf = True
        self.targets = {
            'pick_approach': [.4, .1, .3], 'pick': [.4, .1, .2],
            'place_approach': [.5, .2, .3], 'place': [.5, .2, .2],
        }
        self.target_publishers = {
            name: self.create_publisher(PoseStamped, '/service_test/targets/' + name, 10)
            for name in self.targets
        }
        self.start_pub = self.create_publisher(String, '/service_test/start', 100)
        self.end_pub = self.create_publisher(String, '/service_test/end', 100)
        self.tf = TransformBroadcaster(self)
        callbacks = ReentrantCallbackGroup()
        self.service_servers = [self.create_service(
            PointToPointMotion, '/service_test/move',
            lambda request, response: self.call('move', request, response),
            callback_group=callbacks)]
        if HAS_GRIPPER:
            for name, service_type in (('pick', Pick), ('place', Place)):
                self.service_servers.append(self.create_service(
                    service_type, '/service_test/' + name,
                    lambda request, response, name=name: self.call(name, request, response),
                    callback_group=callbacks))
        self.create_timer(.02, self.publish)

    def publish(self):
        for name, publisher in self.target_publishers.items():
            pose = PoseStamped()
            pose.header.frame_id = self.target_frame
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.stamp.sec += self.stamp_offset
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = self.targets[name]
            pose.pose.orientation.w = 1.
            publisher.publish(pose)
        if self.publish_tf:
            transform = TransformStamped()
            transform.header.frame_id = 'base_link'
            transform.header.stamp = self.get_clock().now().to_msg()
            transform.child_frame_id = 'test_tcp'
            transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z = self.current
            transform.transform.rotation.w = 1.
            self.tf.sendTransform(transform)

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
        self.start_pub.publish(String(data=str(motion_id)))

    def end(self, motion_id):
        for operation, request, ids in self.requests:
            if ids and ids[-1] == motion_id and operation == 'move':
                position = request.g.pose.position
                self.current = [position.x, position.y, position.z]
        self.end_pub.publish(String(data=str(motion_id)))


class Rig:
    def __init__(self, full=False, response_timeout=2., execution_timeout=4., objects=None):
        self.provider = Provider()
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
                      seed_state_topic='/service_test/state', primitives=['move_a_b', 'pick', 'place'] if full else ['move_a_b'])
        config['services'].update(tcp_frame='test_tcp', start_topic='/service_test/start',
                                  end_topic='/service_test/end', response_timeout=response_timeout,
                                  execution_timeout=execution_timeout)
        if objects is not None:
            config['services']['objects'] = objects
        config['move_a_b']['service'] = '/service_test/move'
        config['move_a_b']['pose_timeout'] = .3
        config['move_a_b']['target_topics'] = {name: '/service_test/targets/' + name for name in self.provider.targets}
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
            wait_for(lambda: 'arm.at(pick)' in self.facts)
            time.sleep(.15)  # Populate measured TCP and target subscriptions.
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
        time.sleep(.05)  # Let the mock TF reflect the completed operation.

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
    if options.get('full') and not HAS_GRIPPER:
        pytest.skip('Pick/Place definitions have not arrived; use the test interface overlay')
    rclpy.init()
    test = None
    try:
        test = Rig(**options)
        yield test
    finally:
        if test is not None:
            test.close()
        rclpy.try_shutdown()


def test_move_forwards_current_tcp_exact_target_and_false_preplan(rig):
    rig.send('move_a_b(pick)')
    wait_for(lambda: len(rig.provider.requests) == 1)
    _, request, ids = rig.provider.requests[0]
    assert request.y0.header.frame_id == request.g.header.frame_id == 'base_link'
    assert [request.y0.pose.position.x, request.y0.pose.position.y, request.y0.pose.position.z] == [.1, .2, .3]
    assert request.g.pose.position.z == .2  # No hidden approach offset.
    assert request.max_vel == .05
    assert request.plan_y0_motion is False
    rig.provider.targets['pick'] = [.8, .8, .8]
    rig.provider.start(ids[0])
    wait_for(lambda: any('Started at /motion_start ID' in item['detail'] for item in rig.statuses))
    rig.provider.end(ids[1])
    time.sleep(.1)
    assert not rig.saw('move_a_b(pick)', 'succeeded')
    assert rig.facts['arm.at(pick)'] is False
    rig.provider.end(ids[-1])
    wait_for(lambda: rig.facts.get('arm.at(pick)') is True)
    assert rig.provider.requests[0][1].g.pose.position.z == .2


@pytest.mark.parametrize('ids', [None, [7]])
def test_early_end_before_response_and_before_start_is_not_lost(rig, ids):
    rig.provider.response_ids = ids
    rig.provider.hold_response = True
    rig.send('move_a_b(pick)')
    wait_for(lambda: rig.provider.requests)
    ids = rig.provider.requests[0][2]
    rig.provider.end(ids[-1])
    time.sleep(.1)
    assert not rig.saw('move_a_b(pick)', 'succeeded')
    rig.provider.release.set()
    wait_for(lambda: rig.saw('move_a_b(pick)', 'succeeded'))


def test_duplicate_commands_and_unrelated_events_do_not_complete_or_resubmit(rig):
    rig.send('move_a_b(pick)')
    wait_for(lambda: rig.provider.requests)
    for _ in range(3):
        rig.send('move_a_b(pick)')
    for text in ('-1', 'unrelated', '9223372036854775808', ''):
        rig.provider.end_pub.publish(String(data=text))
    rig.send('move_a_b(place)')
    wait_for(lambda: rig.saw('move_a_b(place)', 'rejected'))
    assert len(rig.provider.requests) == 1
    assert not rig.saw('move_a_b(pick)', 'succeeded')
    rig.provider.end(rig.provider.requests[0][2][-1])
    wait_for(lambda: rig.saw('move_a_b(pick)', 'succeeded'))
    rig.send('move_a_b(pick)')
    time.sleep(.1)
    assert len(rig.provider.requests) == 1


def test_old_completion_cannot_finish_next_request(rig):
    rig.provider.automatic = True
    rig.run('move_a_b(pick)')
    previous = rig.provider.requests[0][2][-1]
    rig.provider.automatic = False
    rig.send('move_a_b(place)')
    wait_for(lambda: len(rig.provider.requests) == 2)
    rig.provider.end(previous)
    time.sleep(.1)
    assert not rig.saw('move_a_b(place)', 'succeeded')
    rig.provider.end(rig.provider.requests[-1][2][-1])
    wait_for(lambda: rig.saw('move_a_b(place)', 'succeeded'))


@pytest.mark.parametrize('ids', [[], [5, 5]])
def test_invalid_accepted_ids_fail_and_keep_execution_slot(rig, ids):
    rig.provider.response_ids = ids
    rig.send('move_a_b(pick)')
    wait_for(lambda: rig.saw('move_a_b(pick)', 'failed'))
    rig.send('reset')
    wait_for(lambda: rig.saw('reset', 'rejected'))
    assert rig.facts['arm.at(pick)'] is False


def test_service_rejection_schedules_nothing_and_allows_reset(rig):
    rig.provider.accept = False
    rig.send('move_a_b(pick)')
    wait_for(lambda: rig.saw('move_a_b(pick)', 'failed'))
    rig.send('reset')
    wait_for(lambda: rig.saw('reset', 'succeeded'))
    rig.provider.accept = rig.provider.automatic = True
    rig.run('move_a_b(pick)')


def test_missing_event_publisher_prevents_dispatch(rig):
    rig.provider.destroy_publisher(rig.provider.end_pub)
    time.sleep(.3)
    rig.send('move_a_b(pick)')
    wait_for(lambda: rig.saw('move_a_b(pick)', 'failed'))
    assert not rig.provider.requests


@pytest.mark.parametrize('rig', [{'response_timeout': .2, 'execution_timeout': .5}], indirect=True)
def test_response_timeout_keeps_slot_until_late_response_and_end(rig):
    rig.provider.hold_response = True
    rig.send('move_a_b(pick)')
    wait_for(lambda: rig.saw('move_a_b(pick)', 'failed'))
    rig.send('reset')
    wait_for(lambda: rig.saw('reset', 'rejected'))
    rig.provider.end(rig.provider.requests[0][2][-1])
    rig.provider.release.set()
    time.sleep(.2)
    rig.send('reset')
    wait_for(lambda: rig.saw('reset', 'succeeded'))
    assert not rig.saw('move_a_b(pick)', 'succeeded')


@pytest.mark.parametrize('rig', [{'execution_timeout': .25}], indirect=True)
def test_missing_end_times_out_without_claiming_completion(rig):
    rig.send('move_a_b(pick)')
    wait_for(lambda: rig.saw('move_a_b(pick)', 'failed'))
    rig.send('reset')
    wait_for(lambda: rig.saw('reset', 'rejected'))
    assert not rig.saw('move_a_b(pick)', 'succeeded')


def test_cancel_reports_unsupported_stop_and_waits_for_remote_end(rig):
    rig.send('move_a_b(pick)')
    wait_for(lambda: rig.provider.requests)
    rig.send('cancel')
    wait_for(lambda: rig.saw('move_a_b(pick)', 'failed'))
    assert any('robot not stopped' in item['detail'] for item in rig.statuses)
    rig.send('reset')
    wait_for(lambda: rig.saw('reset', 'rejected'))
    rig.provider.end(rig.provider.requests[0][2][-1])
    time.sleep(.15)
    rig.send('reset')
    wait_for(lambda: rig.saw('reset', 'succeeded'))
    assert not rig.saw('move_a_b(pick)', 'cancelled')


@pytest.mark.parametrize('command', ['move_a_b', 'move_a_b(unknown)', 'move_a_b(pick,place)'])
def test_invalid_arguments_are_rejected_before_service_call(rig, command):
    rig.send(command)
    wait_for(lambda: rig.saw(command, 'failed'))
    assert not rig.provider.requests


@pytest.mark.parametrize('bad_target', ['frame', 'stamp', 'nan', 'tf'])
def test_bad_pose_or_missing_current_tcp_cannot_start_motion(rig, bad_target):
    if bad_target == 'frame':
        rig.provider.target_frame = 'world'
    elif bad_target == 'stamp':
        rig.provider.stamp_offset = -10
    elif bad_target == 'nan':
        rig.provider.targets['pick'][0] = float('nan')
    else:
        rig.provider.publish_tf = False
        time.sleep(.4)
    time.sleep(.1)
    rig.send('move_a_b(pick)')
    wait_for(lambda: rig.saw('move_a_b(pick)', 'failed'))
    assert not rig.provider.requests


SEQUENCE = ['move_a_b(pick_approach)', 'move_a_b(pick)', 'pick(workpiece,pick)',
            'move_a_b(pick_approach)', 'move_a_b(place_approach)', 'move_a_b(place)',
            'place(workpiece,place)', 'move_a_b(place_approach)']


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
    wait_for(lambda: rig.facts.get('object.placed(workpiece,place)') is True)
    assert rig.facts['object.held(workpiece)'] is False
    assert rig.facts['arm.at(place_approach)'] is True


@pytest.mark.parametrize('rig', [{'full': True}], indirect=True)
def test_gripper_requires_matching_object_and_arm_location(rig):
    rig.send('pick(workpiece,pick)')
    wait_for(lambda: rig.saw('pick(workpiece,pick)', 'failed'))
    assert not rig.provider.requests
    rig.send('reset')
    wait_for(lambda: rig.saw('reset', 'succeeded'))
    rig.provider.automatic = True
    rig.run('move_a_b(pick)')
    rig.send('place(workpiece,pick)')
    wait_for(lambda: rig.saw('place(workpiece,pick)', 'failed'))
    assert len(rig.provider.requests) == 1


@pytest.mark.parametrize('rig', [{'full': True, 'objects': ['red_connector', 'blue_peg']}], indirect=True)
def test_empty_gripper_requests_keep_symbolic_object_identity(rig):
    rig.provider.automatic = True
    rig.run('move_a_b(pick)')
    rig.run('pick(red_connector,pick)')
    wait_for(lambda: rig.facts.get('object.held(red_connector)') is True)
    assert rig.facts['object.held(blue_peg)'] is False
    rig.send('place(blue_peg,pick)')
    wait_for(lambda: rig.saw('place(blue_peg,pick)', 'failed'))
    assert [operation for operation, _, _ in rig.provider.requests] == ['move', 'pick']


@pytest.mark.parametrize('rig', [{'full': True}], indirect=True)
def test_named_seed_task_completes_against_mock_services(rig):
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
        source = (Path(get_package_share_directory('seed')) / '../../../../src/seed').resolve()
        for folder in ('LTM', 'learning'):
            shutil.copytree(source / folder, root / 'src/seed' / folder)
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
                seed.stdin.write('hardware_pick_place_demo\n')
                seed.stdin.flush()
                wait_for(lambda: len(rig.provider.requests) == 8, timeout=15)
                wait_for(lambda: rig.facts.get('object.placed(workpiece,place)') is True)
                def completed():
                    output.seek(0)
                    return 'hardware_pick_place_demo success!' in output.read()
                wait_for(completed)
            finally:
                seed.send_signal(signal.SIGINT)
                try:
                    seed.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    seed.kill()
                    seed.wait()
