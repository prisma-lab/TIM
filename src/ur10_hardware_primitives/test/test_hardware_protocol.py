"""Exercise actual C++ hardware plugins against deterministic ROS action servers."""
import json
from pathlib import Path
import signal
import subprocess
import tempfile
import threading
import time

from ament_index_python.packages import get_package_prefix
from control_msgs.action import GripperCommand
from control_msgs.msg import DynamicJointState, InterfaceValue
from geometry_msgs.msg import PoseStamped, TransformStamped
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.msg import AllowedCollisionEntry, CollisionObject
from moveit_msgs.srv import ApplyPlanningScene, GetCartesianPath, GetPlanningScene
import pytest
import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster
from trajectory_msgs.msg import JointTrajectoryPoint
import yaml


def wait_for(predicate, timeout=6):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if predicate():
            return
        time.sleep(.01)
    assert predicate(), 'Timed out waiting for ROS result'


class Backend(Node):
    def __init__(self):
        super().__init__('hardware_mock_backend')
        self.tcp = [.55, .0, .925]
        self.pick, self.place = [.55, .0, .825], [.5, .35, .835]
        self.orientation = [1., 0., 0., 0.]
        self.publish_targets = self.publish_tf = self.publish_gripper = True
        self.stamp_offset = 0
        self.fault, self.object, self.position = 0., 3., 0.
        self.detect_object = True
        self.missing_detection = False
        self.hold_move = self.hold_gripper = self.hold_cartesian = False
        self.ignore_move_cancel = False
        self.hold_feedback = False
        self.release = threading.Event()
        self.moves, self.grips, self.scenes = [], [], []
        self.scene_ok = True
        self.fraction = 1.
        self.executions = self.cancels = 0
        self.tf = TransformBroadcaster(self)
        self.targets = {k: self.create_publisher(PoseStamped, '/ur10/targets/' + k, 10)
                        for k in ('pick', 'place')}
        self.state_pub = self.create_publisher(DynamicJointState, '/dynamic_joint_states', 10)
        cb = ReentrantCallbackGroup()
        self.actions = [
            ActionServer(self, MoveGroup, '/move_action', execute_callback=self.move,
                         cancel_callback=lambda _: CancelResponse.ACCEPT, callback_group=cb),
            ActionServer(self, ExecuteTrajectory, '/execute_trajectory', execute_callback=self.execute,
                         cancel_callback=lambda _: CancelResponse.ACCEPT, callback_group=cb),
            ActionServer(self, GripperCommand, '/robotiq_gripper_controller/gripper_cmd',
                         execute_callback=self.gripper, cancel_callback=lambda _: CancelResponse.ACCEPT,
                         callback_group=cb)]
        self.create_service(GetCartesianPath, '/compute_cartesian_path', self.cartesian)
        self.create_service(ApplyPlanningScene, '/apply_planning_scene', self.scene)
        self.create_service(GetPlanningScene, '/get_planning_scene', self.query)
        self.create_timer(.02, self.publish)

    def pose(self, position):
        msg = PoseStamped()
        msg.header.frame_id = 'base_link'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.stamp.sec += self.stamp_offset
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = position
        q = msg.pose.orientation
        q.x, q.y, q.z, q.w = self.orientation
        return msg

    def publish(self):
        if self.publish_targets:
            for key, pub in self.targets.items():
                pub.publish(self.pose(getattr(self, key)))
        if self.publish_tf:
            tf = TransformStamped()
            tf.header.frame_id = 'base_link'
            tf.header.stamp = self.get_clock().now().to_msg()
            tf.child_frame_id = 'grasp_tcp'
            tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = self.tcp
            tf.transform.rotation.x = 1.
            tf.transform.rotation.w = 0.
            self.tf.sendTransform(tf)
        if self.publish_gripper:
            msg = DynamicJointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.joint_names = ['finger_joint']
            interfaces = ['position', 'object_status', 'gripper_fault']
            values = [self.position, self.object, self.fault]
            if self.missing_detection:
                interfaces.pop(1); values.pop(1)
            msg.interface_values = [InterfaceValue(interface_names=interfaces, values=values)]
            self.state_pub.publish(msg)

    def move(self, handle):
        p = handle.request.request.goal_constraints[0].position_constraints[0].constraint_region.primitive_poses[0].position
        self.moves.append([p.x, p.y, p.z])
        while self.hold_move and not self.release.is_set():
            if handle.is_cancel_requested and not self.ignore_move_cancel:
                self.cancels += 1
                handle.canceled()
                return MoveGroup.Result()
            time.sleep(.01)
        if not self.hold_feedback:
            self.tcp = [p.x, p.y, p.z]
        handle.succeed()
        result = MoveGroup.Result(); result.error_code.val = 1
        return result

    def gripper(self, handle):
        target = handle.request.command.position
        self.grips.append(target)
        self.position = .4 if target > .02 and self.detect_object else target
        self.object = 2. if target > .02 and self.detect_object else 3.
        while self.hold_gripper and target > .02 and not self.release.is_set():
            if handle.is_cancel_requested:
                self.cancels += 1
                handle.canceled()
                return GripperCommand.Result()
            time.sleep(.01)
        handle.succeed()
        return GripperCommand.Result(position=self.position, reached_goal=True, stalled=self.object == 2.)

    def cartesian(self, request, response):
        assert request.avoid_collisions
        p = request.waypoints[0].position
        response.fraction = self.fraction
        response.error_code.val = 1
        response.solution.joint_trajectory.points = [JointTrajectoryPoint(positions=[p.x, p.y, p.z])]
        return response

    def execute(self, handle):
        self.executions += 1
        while self.hold_cartesian and not self.release.is_set():
            if handle.is_cancel_requested:
                self.cancels += 1; handle.canceled(); return ExecuteTrajectory.Result()
            time.sleep(.01)
        self.tcp = list(handle.request.trajectory.joint_trajectory.points[-1].positions)
        handle.succeed()
        result = ExecuteTrajectory.Result(); result.error_code.val = 1
        return result

    def query(self, request, response):
        response.scene.allowed_collision_matrix.entry_names = ['link_a', 'link_b']
        response.scene.allowed_collision_matrix.entry_values = [
            AllowedCollisionEntry(enabled=[False, True]), AllowedCollisionEntry(enabled=[True, False])]
        return response

    def scene(self, request, response):
        self.scenes.append(request.scene)
        response.success = self.scene_ok
        return response


class Rig:
    def __init__(self):
        self.backend = Backend()
        self.observer = Node('hardware_test_observer')
        self.statuses, self.states = [], {}
        self.pub = self.observer.create_publisher(String, '/hardware_test/command', 10)
        self.alias_pub = self.observer.create_publisher(String, '/ur10/gripper/command', 10)
        self.sub = self.observer.create_subscription(String, '/hardware_test/status',
            lambda m: self.statuses.append(json.loads(m.data)), 100)
        self.state_sub = self.observer.create_subscription(String, '/hardware_test/state',
            lambda m: self.states.update({m.data.lstrip('-'): not m.data.startswith('-')}), 100)
        self.executor = MultiThreadedExecutor(num_threads=6)
        for node in (self.backend, self.observer): self.executor.add_node(node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True); self.thread.start()
        params = yaml.safe_load((Path(__file__).parents[1] / 'config/primitives.yaml').read_text())['ur10_primitive_manager']['ros__parameters']
        params.update(command_topic='/hardware_test/command', status_topic='/hardware_test/status',
                      seed_state_topic='/hardware_test/state')
        params['manipulation'].update(object_dimensions=[.05, .04, .03], execution_timeout=2., target_timeout=.5)
        params['gripper']['execution_timeout'] = 2.
        self.temp = tempfile.TemporaryDirectory()
        path = Path(self.temp.name) / 'params.yaml'; path.write_text(yaml.safe_dump({'/**': {'ros__parameters': params}}))
        self.log = tempfile.TemporaryFile(mode='w+')
        exe = Path(get_package_prefix('primitive_manager')) / 'lib/primitive_manager/primitive_manager'
        self.process = subprocess.Popen([str(exe), '--ros-args', '--params-file', str(path)], stdout=self.log, stderr=subprocess.STDOUT)
        try:
            wait_for(lambda: self.pub.get_subscription_count() == 1)
            time.sleep(.25)
        except Exception:
            self.log.seek(0); print(self.log.read()); self.close(); raise

    def send(self, command): self.pub.publish(String(data=command))
    def saw(self, command, status): return any(m['command'] == command and m['status'] == status for m in self.statuses)
    def close(self):
        self.backend.release.set()
        self.process.send_signal(signal.SIGINT)
        try: self.process.wait(timeout=3)
        except subprocess.TimeoutExpired: self.process.kill(); self.process.wait()
        self.executor.shutdown(timeout_sec=3); self.thread.join(timeout=3)
        for action in self.backend.actions: action.destroy()
        self.backend.destroy_node(); self.observer.destroy_node()
        self.log.close(); self.temp.cleanup()


@pytest.fixture
def rig():
    rclpy.init(); test = Rig()
    try: yield test
    finally: test.close(); rclpy.try_shutdown()


def run(rig, command):
    rig.send(command)
    wait_for(lambda: rig.saw(command, 'succeeded') or rig.saw(command, 'failed'))
    assert rig.saw(command, 'succeeded'), rig.statuses


def test_hardware_four_steps_and_collision_model(rig):
    rig.backend.tcp = [0., 0., 1.]
    time.sleep(.1)
    run(rig, 'move_a_b(pick)'); run(rig, 'pick')
    wait_for(lambda: rig.states.get('object.held'))
    run(rig, 'move_a_b(place)'); run(rig, 'place')
    wait_for(lambda: rig.states.get('object.placed'))
    assert len(rig.backend.moves) == 2 and rig.backend.executions == 4
    assert rig.backend.grips == [0., .695, 0.]
    assert any(s.robot_state.attached_collision_objects and
               s.robot_state.attached_collision_objects[0].object.operation == CollisionObject.ADD
               for s in rig.backend.scenes)
    last = rig.backend.scenes[-1]
    assert last.world.collision_objects[0].operation == CollisionObject.ADD
    assert last.robot_state.attached_collision_objects[0].object.operation == CollisionObject.REMOVE
    assert last.allowed_collision_matrix.entry_values[0].enabled[1]  # Preserve existing ACM.
    assert last.world.collision_objects[0].primitive_poses[0].position.x == rig.backend.place[0]


def test_full_closure_without_object_fails_before_lift(rig):
    rig.backend.detect_object = False
    rig.send('pick'); wait_for(lambda: rig.saw('pick', 'failed'))
    assert rig.backend.executions == 1
    assert not any(s.robot_state.attached_collision_objects for s in rig.backend.scenes)
    assert not rig.states.get('object.held', False)


@pytest.mark.parametrize('condition', ['stale', 'fault', 'missing_detection', 'unknown', 'nonfinite'])
def test_unhealthy_gripper_never_starts_motion(rig, condition):
    if condition == 'stale': rig.backend.publish_gripper = False
    elif condition == 'fault': rig.backend.fault = 9.
    elif condition == 'unknown': rig.backend.fault = 255.; rig.backend.object = 255.
    elif condition == 'nonfinite': rig.backend.position = float('nan')
    else: rig.backend.missing_detection = True
    time.sleep(.6)
    rig.send('move_a_b(place)'); wait_for(lambda: rig.saw('move_a_b(place)', 'failed'))
    assert not rig.backend.moves and not rig.backend.grips
    wait_for(lambda: rig.states.get('manipulation.failed') and rig.states.get('gripper.failed'))


def test_detection_loss_cancels_loaded_transfer(rig):
    rig.backend.object, rig.backend.position = 2., .4
    rig.backend.hold_move = True
    time.sleep(.1)
    rig.send('move_a_b(place)'); wait_for(lambda: len(rig.backend.moves) == 1)
    rig.backend.object = 3.
    wait_for(lambda: rig.saw('move_a_b(place)', 'failed'))
    wait_for(lambda: rig.backend.cancels == 1)


def test_controller_success_requires_observed_arrival(rig):
    rig.backend.hold_feedback = True
    rig.send('move_a_b(place)'); wait_for(lambda: rig.saw('move_a_b(place)', 'failed'))
    assert not rig.saw('move_a_b(place)', 'succeeded')


def test_cancel_and_alias_keep_one_execution_slot(rig):
    rig.backend.hold_move = rig.backend.ignore_move_cancel = True
    rig.send('move_a_b(place)'); wait_for(lambda: len(rig.backend.moves) == 1)
    rig.send('cancel'); wait_for(lambda: rig.saw('move_a_b(place)', 'cancelling'))
    rig.send('reset'); rig.alias_pub.publish(String(data='open_gripper'))
    wait_for(lambda: rig.saw('reset', 'rejected') and rig.saw('open_gripper', 'rejected'))
    assert not rig.backend.grips
    rig.backend.release.set(); wait_for(lambda: rig.saw('move_a_b(place)', 'cancelled'))


def test_cancel_after_grasp_reconciles_carried_geometry(rig):
    rig.backend.hold_gripper = True
    rig.send('pick'); wait_for(lambda: .695 in rig.backend.grips)
    time.sleep(.1); rig.send('cancel')
    wait_for(lambda: rig.saw('pick', 'cancelled'))
    assert rig.backend.executions == 1
    assert rig.backend.scenes[-1].robot_state.attached_collision_objects[0].object.operation == CollisionObject.ADD


@pytest.mark.parametrize('fraction', [.8, float('nan')])
def test_incomplete_cartesian_plan_never_executes(rig, fraction):
    rig.backend.fraction = fraction
    rig.send('pick'); wait_for(lambda: rig.saw('pick', 'failed'))
    assert rig.backend.executions == 0 and .695 not in rig.backend.grips


def test_scene_failure_prevents_descent(rig):
    rig.backend.scene_ok = False
    rig.send('pick'); wait_for(lambda: rig.saw('pick', 'failed'))
    assert rig.backend.executions == 0 and not rig.backend.grips


def test_place_requires_detected_object(rig):
    rig.send('place'); wait_for(lambda: rig.saw('place', 'failed'))
    assert not rig.backend.grips and rig.backend.executions == 0


@pytest.mark.parametrize('kind', ['timestamp', 'quaternion', 'missing'])
def test_invalid_external_targets_do_not_move(rig, kind):
    if kind == 'timestamp': rig.backend.stamp_offset = -10
    elif kind == 'quaternion': rig.backend.orientation = [0., 0., 0., 0.]
    else: rig.backend.publish_targets = False
    time.sleep(.6)
    rig.send('move_a_b(pick)'); wait_for(lambda: rig.saw('move_a_b(pick)', 'failed'))
    assert not rig.backend.moves


def test_gripper_commands_use_same_manager(rig):
    rig.alias_pub.publish(String(data='close_gripper'))
    wait_for(lambda: rig.saw('close_gripper', 'succeeded'))
    run(rig, 'open_gripper')
    assert rig.backend.grips == [.695, 0.]


def test_ambiguous_partial_closure_does_not_start_travel(rig):
    rig.backend.position, rig.backend.object = .3, 3.
    time.sleep(.1)
    rig.send('move_a_b(place)'); wait_for(lambda: rig.saw('move_a_b(place)', 'failed'))
    assert not rig.backend.moves


def test_initial_scene_includes_measured_workpiece_before_motion(rig):
    from ament_index_python.packages import get_package_share_directory as share
    cell = yaml.safe_load((Path(__file__).parents[1] / 'config/cell.yaml').read_text())
    cell.update(configured=True, robot_ip='127.0.0.1', serial_port='/dev/nonexistent-test',
        kinematics_file=str(Path(share('ur_description')) / 'config/ur10/default_kinematics.yaml'),
        mount_dimensions=[.05,.05,.02], object_dimensions=[.04,.06,.03],
        object_offset_xyz=[0., 0., .02], obstacles=[dict(id='test_table',
            dimensions=[1.,1.,.05], xyz=[.5,0.,-.1], rpy=[0.,0.,0.])])
    path = Path(rig.temp.name) / 'cell.yaml'; path.write_text(yaml.safe_dump(cell))
    script = Path(__file__).parents[1] / 'scripts/workcell_scene.py'
    result = subprocess.run(['python3', str(script), '--ros-args', '-p', f'cell_config:={path}'],
                            capture_output=True, text=True, timeout=15)
    assert result.returncode == 0, result.stderr
    world = {obj.id: obj for obj in rig.backend.scenes[-1].world.collision_objects}
    assert set(world) == {'test_table', 'workpiece'}
    # TCP is downward-facing in the fixture, so +Z in TCP is -Z in base_link.
    assert abs(world['workpiece'].primitive_poses[0].position.z - (rig.backend.pick[2] - .02)) < 1e-6
    assert not rig.backend.moves and not rig.backend.grips


def test_hardware_target_publisher_rejects_empty_template(rig):
    package = Path(__file__).parents[1]
    result = subprocess.run(['python3', str(package / 'scripts/target_publisher.py'), '--ros-args',
        '--params-file', str(package / 'config/targets.yaml')], capture_output=True, text=True, timeout=8)
    assert result.returncode == 1
    assert not rig.backend.moves


def test_hardware_target_publisher_uses_measured_values(rig):
    messages = []
    sub = rig.observer.create_subscription(PoseStamped, '/ur10/targets/pick',
        lambda msg: messages.append(msg) if msg.header.frame_id == 'test_frame' else None, 10)
    path = Path(rig.temp.name) / 'targets.yaml'
    pose = [.123, .234, .345, 0., 0., 0., 1.]
    path.write_text(yaml.safe_dump({'hardware_targets': {'ros__parameters': {
        'frame': 'test_frame', 'pick_pose': pose, 'place_pose': list(pose)}}}))
    publisher_log = tempfile.TemporaryFile(mode='w+')
    result = subprocess.Popen(['python3', str(Path(__file__).parents[1] / 'scripts/target_publisher.py'),
        '--ros-args', '--params-file', str(path)], stdout=publisher_log, stderr=subprocess.STDOUT)
    rig.executor.wake()
    try:
        wait_for(lambda: len(messages) >= 2)
        assert messages[-1].pose.position.x == pose[0]
        assert messages[-1].pose.orientation.w == 1.
        a, b = messages[0].header.stamp, messages[-1].header.stamp
        assert (b.sec, b.nanosec) > (a.sec, a.nanosec)
        assert not rig.backend.moves and not rig.backend.grips
    except Exception:
        publisher_log.seek(0); print(publisher_log.read()); print('Publisher exit:', result.poll())
        raise
    finally:
        result.send_signal(signal.SIGINT); result.wait(timeout=5)
        rig.observer.destroy_subscription(sub)
        publisher_log.close()
