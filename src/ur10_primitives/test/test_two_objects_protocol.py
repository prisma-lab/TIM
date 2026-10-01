"""Parameter propagation and object isolation through the real C++ plugins."""
import json
from pathlib import Path
import signal
import subprocess
import tempfile
import threading
import time

from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import PoseStamped
import pytest
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool
import yaml

from test_gripper_protocol import Controller, wait_for
from test_pick_place_protocol import Backend

OBJECTS = ('red_connector', 'blue_peg')
COMMANDS = ['move_a_b(red_pick)', 'pick(red_connector,red_pick)',
            'move_a_b(red_place)', 'place(red_connector,red_place)',
            'move_a_b(blue_pick)', 'pick(blue_peg,blue_pick)',
            'move_a_b(blue_place)', 'place(blue_peg,blue_place)']


class ObjectsBackend(Backend):
    def __init__(self):
        super().__init__()
        self.objects = {'red_connector': [.55, 0., .825], 'blue_peg': [.7, -.4, .825]}
        self.locations = {'red_pick': [.55, 0., .825], 'red_place': [.5, .35, .835],
                          'blue_pick': [.7, -.4, .825], 'blue_place': [.5, -.35, .835]}
        self.held_id = None
        self.emit = set(OBJECTS)
        self.object_grasps = []
        self.object_pubs = {}
        self.holding_pubs = {}
        self.location_pubs = {}
        self.grasp_services = []
        for obj in OBJECTS:
            prefix = '/ur10/two_objects/objects/' + obj
            self.object_pubs[obj] = self.create_publisher(PoseStamped, prefix + '/pose', 10)
            self.holding_pubs[obj] = self.create_publisher(Bool, prefix + '/holding',
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.grasp_services.append(self.create_service(SetBool, prefix + '/set_grasp',
                lambda req, res, obj=obj: self.object_grasp(obj, req, res)))
        for location in self.locations:
            self.location_pubs[location] = self.create_publisher(PoseStamped,
                '/ur10/two_objects/targets/' + location, 10)

    def publish(self):
        super().publish()
        if self.held_id:
            self.objects[self.held_id] = list(self.object)
        for obj in self.emit:
            self.object_pubs[obj].publish(self.pose(self.objects[obj]))
            self.holding_pubs[obj].publish(Bool(data=self.held_id == obj))
        if self.publish_targets:
            for name, pub in self.location_pubs.items():
                msg = self.pose(self.locations[name], self.frame)
                msg.header.stamp.sec += int(self.stamp_offset)
                pub.publish(msg)

    def object_grasp(self, obj, request, response):
        self.object_grasps.append((obj, request.data))
        if request.data:
            if self.held_id and self.held_id != obj:
                response.success = False
                return response
            self.held_id = obj
            self.object = list(self.tcp)
        else:
            if self.held_id != obj:
                response.success = False
                return response
            self.objects[obj] = list(self.tcp)
            self.objects[obj][2] = .815
            self.held_id = None
        self.held = self.held_id is not None
        response.success = True
        return response


class Rig:
    def __init__(self):
        self.backend = ObjectsBackend()
        self.controller = Controller('/two_object_test')
        self.observer = Node('two_object_observer')
        self.statuses = []
        self.states = {}
        self.pub = self.observer.create_publisher(String, '/two_object_test/command', 10)
        self.sub = self.observer.create_subscription(String, '/two_object_test/status',
            lambda msg: self.statuses.append(json.loads(msg.data)), 100)
        self.state_sub = self.observer.create_subscription(String, '/two_object_test/state',
            lambda msg: self.states.update({msg.data.lstrip('-'): not msg.data.startswith('-')}), 100)
        config = yaml.safe_load((Path(__file__).parents[1] / 'config/two_objects.yaml').read_text())['ur10_two_object_manager']['ros__parameters']
        config.update(use_sim_time=False, command_topic='/two_object_test/command',
                      status_topic='/two_object_test/status', seed_state_topic='/two_object_test/state')
        config['gripper'].update(action_name='/two_object_test/trajectory',
                                joint_states_topic='/two_object_test/joints', motion_duration=.1)
        config['manipulation'].update(target_timeout=.5, execution_timeout=4.)
        self.temp = tempfile.TemporaryDirectory()
        path = Path(self.temp.name) / 'params.yaml'
        path.write_text(yaml.safe_dump({'/**': {'ros__parameters': config}}))
        self.log = tempfile.TemporaryFile(mode='w+')
        exe = Path(get_package_prefix('primitive_manager')) / 'lib/primitive_manager/primitive_manager'
        self.process = subprocess.Popen([str(exe), '--ros-args', '--params-file', str(path)],
                                        stdout=self.log, stderr=subprocess.STDOUT)
        self.executor = MultiThreadedExecutor(num_threads=6)
        for node in (self.backend, self.controller, self.observer):
            self.executor.add_node(node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()
        try:
            wait_for(lambda: self.pub.get_subscription_count() == 1, timeout=8)
            wait_for(lambda: self.states.get('arm.at(red_pick)') is True, timeout=8)
        except AssertionError:
            self.log.seek(0)
            output = self.log.read()
            self.close()
            pytest.fail(output)

    def send(self, command):
        self.pub.publish(String(data=command))

    def saw(self, command, status):
        return any(item['command'] == command and item['status'] == status for item in self.statuses)

    def run(self, command):
        self.send(command)
        wait_for(lambda: self.saw(command, 'succeeded') or self.saw(command, 'failed'), timeout=8)
        assert self.saw(command, 'succeeded'), self.statuses

    def close(self):
        self.backend.release.set()
        self.controller.release.set()
        self.process.send_signal(signal.SIGINT)
        try:
            self.process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            self.process.kill()
            self.process.wait(timeout=3)
        self.executor.shutdown(timeout_sec=3)
        self.thread.join(timeout=3)
        self.backend.action.destroy()
        self.backend.execution.destroy()
        self.controller.server.destroy()
        for node in (self.backend, self.controller, self.observer):
            node.destroy_node()
        self.log.close()
        self.temp.cleanup()


@pytest.fixture
def rig():
    rclpy.init()
    test = Rig()
    try:
        yield test
    finally:
        test.close()
        rclpy.try_shutdown()


def test_two_object_sequence_preserves_parameters_and_distinct_facts(rig):
    for command in COMMANDS:
        rig.run(command)
    assert rig.backend.object_grasps == [('red_connector', True), ('red_connector', False),
                                        ('blue_peg', True), ('blue_peg', False)]
    assert len(rig.backend.transfers) == 4
    assert len(rig.backend.cartesian_requests) == 8
    ids = [scene.robot_state.attached_collision_objects[0].object.id for scene in rig.backend.scenes]
    assert ids == ['red_connector', 'red_connector', 'blue_peg', 'blue_peg']
    wait_for(lambda: rig.states.get('object.placed(red_connector,red_place)') is True)
    wait_for(lambda: rig.states.get('object.placed(blue_peg,blue_place)') is True)
    assert rig.states['object.placed(red_connector,blue_place)'] is False
    assert rig.states['object.placed(blue_peg,red_place)'] is False
    assert rig.states['object.held(red_connector)'] is False
    assert rig.states['object.held(blue_peg)'] is False


@pytest.mark.parametrize('command', ['pick', 'pick(red_connector)', 'pick(unknown,red_pick)',
                                     'pick(red_connector,unknown)', 'move_a_b(unknown)'])
def test_invalid_parameters_fail_before_moving(rig, command):
    rig.send(command)
    wait_for(lambda: rig.saw(command, 'failed'))
    assert not rig.backend.moves
    assert not rig.backend.object_grasps
    assert not rig.controller.requests


def test_pick_wrong_object_at_location_does_not_grasp(rig):
    rig.send('pick(blue_peg,red_pick)')
    wait_for(lambda: rig.saw('pick(blue_peg,red_pick)', 'failed'))
    assert not rig.backend.cartesian_requests
    assert not rig.backend.object_grasps


def test_red_feedback_cannot_satisfy_blue_pick_or_release_red_as_blue(rig):
    rig.run('pick(red_connector,red_pick)')
    wait_for(lambda: rig.states.get('object.held(red_connector)') is True)
    assert rig.states['object.held(blue_peg)'] is False
    rig.send('place(blue_peg,red_pick)')
    wait_for(lambda: rig.saw('place(blue_peg,red_pick)', 'failed'))
    assert rig.backend.held_id == 'red_connector'
    assert rig.backend.object_grasps == [('red_connector', True)]


def test_pick_requires_empty_gripper(rig):
    rig.run('pick(red_connector,red_pick)')
    rig.send('pick(blue_peg,blue_pick)')
    wait_for(lambda: rig.saw('pick(blue_peg,blue_pick)', 'failed'))
    assert rig.backend.object_grasps == [('red_connector', True)]


def test_missing_blue_feedback_is_not_replaced_by_fresh_red_feedback(rig):
    rig.backend.emit.remove('blue_peg')
    time.sleep(2.1)
    rig.send('move_a_b(blue_pick)')
    wait_for(lambda: rig.saw('move_a_b(blue_pick)', 'failed'))
    assert not rig.backend.moves


def test_pick_snapshots_selected_location(rig):
    rig.backend.hold_execution = True
    rig.send('pick(red_connector,red_pick)')
    wait_for(lambda: rig.backend.cartesian_executions == 1)
    rig.backend.locations['red_pick'] = [.9, .4, .825]
    time.sleep(.1)
    rig.backend.release.set()
    wait_for(lambda: rig.saw('pick(red_connector,red_pick)', 'succeeded'))
    assert all(abs(p[0] - .55) < 1e-6 for p in rig.backend.cartesian_requests)


def test_goal_waits_for_local_pick_to_finish_and_cancel_keeps_identity(rig):
    rig.backend.hold_execution = True
    rig.send('pick(red_connector,red_pick)')
    wait_for(lambda: rig.backend.cartesian_executions == 1)
    assert not rig.states['object.held(red_connector)']
    rig.send('cancel')
    wait_for(lambda: rig.saw('pick(red_connector,red_pick)', 'cancelled'))
    assert not rig.backend.object_grasps

