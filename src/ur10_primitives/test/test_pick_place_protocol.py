"""Real plugin manager with controllable planning/grasp backends."""
import json
from pathlib import Path
import signal
import subprocess
import tempfile
import threading
import time

from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.srv import ApplyPlanningScene, GetCartesianPath
from trajectory_msgs.msg import JointTrajectoryPoint
import pytest
import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool
import yaml

from test_gripper_protocol import Controller, wait_for


class Backend(Node):
    def __init__(self):
        super().__init__('mock_manipulation')
        self.held = False
        self.object = [0.55, 0.0, 0.825]
        self.tcp = [0.55, 0.0, 0.925]
        self.pick = [0.55, 0.0, 0.825]
        self.place = [0.9, 0.35, 0.835]
        self.orientation = [1.0, 0.0, 0.0, 0.0]
        self.frame = 'world'
        self.stamp_offset = 0.0
        self.publish_targets = True
        self.moves = []
        self.transfers = []
        self.cartesian_requests = []
        self.publish_tf = True
        self.hold_feedback = False
        self.tf = TransformBroadcaster(self)
        self.grasps = []
        self.hold = False
        self.abort = False
        self.release = threading.Event()
        self.cancelled = 0
        self.cartesian_fraction = 1.0
        self.cartesian_executions = 0
        self.hold_execution = False
        self.scenes = []
        self.held_pub = self.create_publisher(Bool, '/ur10/demo/holding', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.object_pub = self.create_publisher(PoseStamped, '/ur10/demo/object_pose', 10)
        self.target_pubs = {x:self.create_publisher(PoseStamped, '/ur10/targets/' + x, 10) for x in ('pick','place')}
        self.create_timer(0.02, self.publish)
        self.action = ActionServer(self, MoveGroup, '/move_action', execute_callback=self.move,
            cancel_callback=lambda _:CancelResponse.ACCEPT, callback_group=ReentrantCallbackGroup())
        self.create_service(SetBool, '/ur10/demo/set_grasp', self.grasp)
        self.create_service(ApplyPlanningScene, '/apply_planning_scene', lambda req,res:self.scene(req,res))
        self.create_service(GetCartesianPath, '/compute_cartesian_path', self.cartesian)
        self.execution = ActionServer(self, ExecuteTrajectory, '/execute_trajectory', execute_callback=self.execute,
            cancel_callback=lambda _:CancelResponse.ACCEPT, callback_group=ReentrantCallbackGroup())

    def pose(self, position, frame='world'):
        msg=PoseStamped();msg.header.frame_id=frame;msg.header.stamp=self.get_clock().now().to_msg()
        msg.pose.position.x,msg.pose.position.y,msg.pose.position.z=position
        msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w=self.orientation
        return msg

    def publish(self):
        if self.publish_tf:
            transform = TransformStamped()
            transform.header.frame_id = 'world'
            transform.header.stamp = self.get_clock().now().to_msg()
            transform.child_frame_id = 'tool0'
            transform.transform.translation.x = self.tcp[0]
            transform.transform.translation.y = self.tcp[1]
            transform.transform.translation.z = self.tcp[2] + .15
            transform.transform.rotation.x = 1.0
            transform.transform.rotation.w = 0.0
            self.tf.sendTransform(transform)
        self.held_pub.publish(Bool(data=self.held))
        self.object_pub.publish(self.pose(self.object))
        if self.publish_targets:
            for name,pub in self.target_pubs.items():
                msg=self.pose(getattr(self,name),self.frame)
                msg.header.stamp.sec += int(self.stamp_offset)
                pub.publish(msg)

    def move(self, handle):
        p=handle.request.request.goal_constraints[0].position_constraints[0].constraint_region.primitive_poses[0].position
        self.moves.append([p.x,p.y,p.z])
        self.transfers.append([p.x,p.y,p.z])
        while self.hold and not self.release.is_set():
            if handle.is_cancel_requested:
                self.cancelled += 1;handle.canceled();return MoveGroup.Result()
            time.sleep(.01)
        result=MoveGroup.Result()
        if self.abort:
            result.error_code.val=-1;handle.abort();return result
        if not self.hold_feedback:
            self.tcp=[p.x,p.y,p.z-.15]
        if self.held:self.object=list(self.tcp)
        result.error_code.val=1;handle.succeed();return result

    def grasp(self, request, response):
        self.grasps.append(request.data)
        self.held=request.data
        if self.held:self.object=list(self.tcp)
        else:self.object[2]=.815
        response.success=True
        return response

    def scene(self, request, response):
        self.scenes.append(request.scene)
        response.success=True
        return response

    def cartesian(self, request, response):
        assert request.avoid_collisions
        assert request.revolute_jump_threshold > 0
        p = request.waypoints[0].position
        self.moves.append([p.x, p.y, p.z])
        self.cartesian_requests.append([p.x, p.y, p.z])
        response.error_code.val = 1
        response.fraction = self.cartesian_fraction
        # Encode the requested endpoint for the fake executor, not real joints.
        response.solution.joint_trajectory.points = [JointTrajectoryPoint(positions=[p.x, p.y, p.z])]
        return response

    def execute(self, handle):
        self.cartesian_executions += 1
        while self.hold_execution and not self.release.is_set():
            if handle.is_cancel_requested:
                self.cancelled += 1
                handle.canceled()
                return ExecuteTrajectory.Result()
            time.sleep(.01)
        p = handle.request.trajectory.joint_trajectory.points[-1].positions
        self.tcp = [p[0], p[1], p[2] - .15]
        if self.held:
            self.object = list(self.tcp)
        result = ExecuteTrajectory.Result()
        result.error_code.val = 1
        handle.succeed()
        return result


class Rig:
    def __init__(self):
        self.backend=Backend()
        self.controller=Controller('/pick_test')
        self.observer=Node('pick_test_observer')
        self.statuses=[];self.states={}
        self.pub=self.observer.create_publisher(String,'/pick_test/command',10)
        self.sub=self.observer.create_subscription(String,'/pick_test/status',lambda m:self.statuses.append(json.loads(m.data)),100)
        self.state_sub=self.observer.create_subscription(String,'/pick_test/state',lambda m:self.states.update({m.data.lstrip('-'):not m.data.startswith('-')}),100)
        self.executor=MultiThreadedExecutor(num_threads=6)
        for node in (self.backend,self.controller,self.observer):self.executor.add_node(node)
        self.thread=threading.Thread(target=self.executor.spin,daemon=True);self.thread.start()
        config=yaml.safe_load((Path(__file__).parents[1]/'config/pick_place.yaml').read_text())['ur10_primitive_manager']['ros__parameters']
        config.update(use_sim_time=False,command_topic='/pick_test/command',status_topic='/pick_test/status',seed_state_topic='/pick_test/state')
        config['gripper'].update(action_name='/pick_test/trajectory',joint_states_topic='/pick_test/joints',motion_duration=.1)
        config['manipulation'].update(target_timeout=.5,execution_timeout=3.0)
        self.temp=tempfile.TemporaryDirectory();path=Path(self.temp.name)/'params.yaml';path.write_text(yaml.safe_dump({'/**':{'ros__parameters':config}}))
        self.log=tempfile.TemporaryFile(mode='w+')
        exe=Path(get_package_prefix('primitive_manager'))/'lib/primitive_manager/primitive_manager'
        self.process=subprocess.Popen([str(exe),'--ros-args','--params-file',str(path)],stdout=self.log,stderr=subprocess.STDOUT)
        wait_for(lambda:self.pub.get_subscription_count()==1)
        # Allow target/controller/grasp observations to reach the loaded plugins.
        time.sleep(.2)

    def send(self, command):self.pub.publish(String(data=command))
    def saw(self, command, status):return any(x['command']==command and x['status']==status for x in self.statuses)
    def close(self):
        self.backend.release.set();self.controller.release.set()
        self.process.send_signal(signal.SIGINT)
        try:self.process.wait(timeout=3)
        except subprocess.TimeoutExpired:self.process.kill();self.process.wait(timeout=3)
        self.executor.shutdown(timeout_sec=3);self.thread.join(timeout=3)
        self.backend.action.destroy();self.backend.execution.destroy();self.controller.server.destroy()
        for node in (self.backend,self.controller,self.observer):node.destroy_node()
        self.log.close();self.temp.cleanup()


@pytest.fixture
def rig():
    rclpy.init();test=Rig()
    try:yield test
    finally:test.close();rclpy.try_shutdown()


def run(rig, command):
    rig.send(command)
    wait_for(lambda: rig.saw(command, 'succeeded') or rig.saw(command, 'failed'))
    assert rig.saw(command, 'succeeded'), rig.statuses


def test_four_step_sequence_separates_transfer_from_grasp(rig):
    rig.backend.tcp = [0.0, 0.0, 1.0]
    time.sleep(.1)
    run(rig, 'move_a_b(pick)')
    assert len(rig.backend.transfers) == 1
    assert not rig.backend.cartesian_requests
    assert not rig.backend.grasps
    wait_for(lambda: rig.states.get('arm.at(pick)') is True)
    run(rig, 'pick')
    assert len(rig.backend.transfers) == 1  # Pick never requests a transfer.
    assert len(rig.backend.cartesian_requests) == 2  # Descent and lift.
    wait_for(lambda: rig.states.get('object.held') is True)
    rig.backend.place = [.5, .35, .835]
    time.sleep(.1)
    run(rig, 'move_a_b(place)')
    assert len(rig.backend.transfers) == 2
    assert rig.backend.grasps == [True]  # Moving a held object never releases it.
    wait_for(lambda: rig.states.get('arm.at(pick)') is False)
    wait_for(lambda: rig.states.get('arm.at(place)') is True)
    run(rig, 'place')
    assert len(rig.backend.transfers) == 2  # Place never requests a transfer.
    assert len(rig.backend.cartesian_requests) == 4
    assert rig.backend.grasps == [True, False]
    wait_for(lambda: rig.states.get('object.placed') is True)
    assert rig.backend.object[:2] == [.5, .35]


def test_pick_snapshots_target_during_local_motion(rig):
    rig.backend.hold_execution = True
    rig.send('pick')
    wait_for(lambda: rig.backend.cartesian_executions == 1)
    rig.backend.pick = [1.1, .4, .9]
    time.sleep(.1)
    rig.backend.release.set()
    wait_for(lambda: rig.saw('pick', 'succeeded'))
    assert not rig.backend.transfers
    assert all(abs(p[0] - .55) < 1e-6 for p in rig.backend.cartesian_requests)


def test_move_snapshots_target_and_observed_arrival_tracks_new_pose(rig):
    rig.backend.hold = True
    rig.send('move_a_b(place)')
    wait_for(lambda: len(rig.backend.transfers) == 1)
    old_target = list(rig.backend.place)
    rig.backend.place = [.5, -.35, .835]
    time.sleep(.1)
    rig.backend.release.set()
    wait_for(lambda: rig.saw('move_a_b(place)', 'succeeded'))
    assert rig.backend.tcp[:2] == old_target[:2]
    # Completion of an old invocation does not claim arrival at a changed target.
    wait_for(lambda: rig.states.get('arm.at(place)') is False)
    assert not rig.backend.grasps


@pytest.mark.parametrize('command', ['pick', 'move_a_b(pick)'])
@pytest.mark.parametrize('kind', ['stale', 'quaternion', 'frame', 'missing'])
def test_invalid_targets_never_request_motion(rig, kind, command):
    if kind == 'stale': rig.backend.stamp_offset = -10
    elif kind == 'quaternion': rig.backend.orientation = [0., 0., 0., 0.]
    elif kind == 'frame': rig.backend.frame = 'unknown_camera'
    else: rig.backend.publish_targets = False
    time.sleep(.65)
    rig.send(command); wait_for(lambda: rig.saw(command, 'failed'))
    assert not rig.backend.moves
    assert not rig.backend.grasps


@pytest.mark.parametrize('command,held', [('pick', False), ('place', True)])
def test_local_primitive_requires_arrival_first(rig, command, held):
    rig.backend.tcp = [0.0, 0.0, 1.0]
    rig.backend.held = held
    time.sleep(.1)
    rig.send(command); wait_for(lambda: rig.saw(command, 'failed'))
    assert not rig.backend.moves
    assert not rig.backend.grasps
    assert 'move_a_b' in rig.statuses[-1]['detail']


def test_place_requires_held_object(rig):
    rig.send('place'); wait_for(lambda: rig.saw('place', 'failed'))
    assert not rig.backend.moves


@pytest.mark.parametrize('command', ['move_a_b', 'move_a_b(unknown)', 'move_a_b(pick,place)', 'pick(extra)'])
def test_invalid_command_arguments_never_move(rig, command):
    rig.send(command); wait_for(lambda: rig.saw(command, 'failed'))
    assert not rig.backend.moves
    assert not rig.backend.grasps


def test_planner_failure_prevents_grasp(rig):
    rig.backend.abort = True
    rig.send('move_a_b(place)'); wait_for(lambda: rig.saw('move_a_b(place)', 'failed'))
    assert not rig.backend.grasps
    assert not rig.backend.cartesian_requests


def test_move_requires_measured_arrival_not_only_controller_success(rig):
    rig.backend.hold_feedback = True
    rig.send('move_a_b(place)'); wait_for(lambda: rig.saw('move_a_b(place)', 'failed'), timeout=5)
    assert not rig.saw('move_a_b(place)', 'succeeded')
    assert rig.states.get('arm.at(place)') is False


def test_cancel_waits_for_move_result(rig):
    rig.backend.hold = True
    rig.send('move_a_b(place)'); wait_for(lambda: len(rig.backend.transfers) == 1)
    rig.send('cancel'); wait_for(lambda: rig.saw('move_a_b(place)', 'cancelled'))
    assert rig.backend.cancelled == 1
    assert not rig.backend.grasps


@pytest.mark.parametrize('fraction', [.7, float('nan')])
def test_partial_or_invalid_cartesian_path_is_never_executed(rig, fraction):
    rig.backend.cartesian_fraction = fraction
    rig.send('pick'); wait_for(lambda: rig.saw('pick', 'failed'))
    assert rig.backend.cartesian_executions == 0
    assert not rig.backend.grasps


def test_cancel_waits_for_cartesian_execution(rig):
    rig.backend.hold_execution = True
    rig.send('pick'); wait_for(lambda: rig.backend.cartesian_executions == 1)
    rig.send('cancel'); wait_for(lambda: rig.saw('pick', 'cancelled'))
    assert rig.backend.cancelled == 1
    assert not rig.backend.grasps


def test_target_change_after_transfer_requires_a_new_transfer(rig):
    run(rig, 'move_a_b(pick)')
    rig.backend.pick = [.7, .0, .825]
    time.sleep(.1)
    rig.send('pick'); wait_for(lambda: rig.saw('pick', 'failed'))
    assert not rig.backend.cartesian_requests
    assert not rig.backend.grasps


def test_stale_tool_feedback_clears_arrival_and_prevents_transfer(rig):
    wait_for(lambda: rig.states.get('arm.at(pick)') is True)
    rig.backend.publish_tf = False
    time.sleep(2.2)
    wait_for(lambda: rig.states.get('arm.at(pick)') is False)
    rig.send('move_a_b(place)'); wait_for(lambda: rig.saw('move_a_b(place)', 'failed'))
    assert not rig.backend.moves
