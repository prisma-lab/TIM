#!/usr/bin/env python3
"""Manual Gazebo integration check. Use a free ROS domain and Gazebo master port.

Default: Fast Downward -> SEED. Set TWO_OBJECT_MODE=named for the named LTM task.
SEED logs and learned weights are isolated from the source checkout.
"""
import json
import math
import os
from pathlib import Path
import signal
import shutil
import subprocess
import tempfile
import time

import rclpy
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from controller_manager_msgs.srv import ListControllers
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String


# Keep SEED's learned weights and logs out of the shared source checkout.
seed_temp = tempfile.TemporaryDirectory(prefix='tim-seed-check-')
seed_root = Path(seed_temp.name)
seed_prefix = seed_root / 'install/seed'
(seed_prefix / 'share/ament_index/resource_index/packages').mkdir(parents=True)
(seed_prefix / 'share/ament_index/resource_index/packages/seed').touch()
(seed_prefix / 'share/seed').mkdir(parents=True)
seed_source = (Path(get_package_share_directory('seed')) / '../../../../src/seed').resolve()
for folder in ('LTM', 'learning'):
    shutil.copytree(seed_source / folder, seed_root / 'src/seed' / folder)
(seed_root / 'src/seed/log').mkdir()
seed_env = dict(os.environ)
seed_env['AMENT_PREFIX_PATH'] = str(seed_prefix) + ':' + seed_env.get('AMENT_PREFIX_PATH', '')
seed_binary = str(Path(get_package_prefix('seed')) / 'lib/seed/seed')

rclpy.init()
node = rclpy.create_node('two_object_scene_check')
poses = {}
completed = []
failures = []
states = {}
expected = ['move_a_b(red_pick)', 'pick(red_connector,red_pick)',
            'move_a_b(red_place)', 'place(red_connector,red_place)',
            'move_a_b(blue_pick)', 'pick(blue_peg,blue_pick)',
            'move_a_b(blue_place)', 'place(blue_peg,blue_place)']

def on_status(msg):
    data = json.loads(msg.data)
    if data['status'] == 'succeeded' and data['command'] not in completed:
        completed.append(data['command'])
        print('Completed:', data['command'], flush=True)
    elif data['status'] == 'failed':
        failures.append(data)

subscriptions = [node.create_subscription(PoseStamped, '/ur10/two_objects/objects/' + name + '/pose',
                 lambda msg, name=name: poses.update({name: msg.pose.position}), 10)
                 for name in ('red_connector', 'blue_peg')]
subscriptions.append(node.create_subscription(String, '/ur10/two_objects/status', on_status, 100))
subscriptions.append(node.create_subscription(String, '/seed_ur10_two_objects/state',
    lambda m: states.update({m.data.lstrip('-'): not m.data.startswith('-')}), 100))
launch_log = Path('/tmp/two-objects-launch.log')
seed_log = Path('/tmp/two-objects-seed.log')
launch_out = launch_log.open('w'); seed_out = seed_log.open('w')
processes = []

def wait_for(predicate, seconds, description):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if failures:
            raise RuntimeError(str(failures))
        if any(p.poll() is not None for p in processes):
            raise RuntimeError('A launch/SEED process stopped unexpectedly')
        if predicate():
            return
    raise TimeoutError(description)

try:
    processes.append(subprocess.Popen(['ros2', 'launch', 'ur10_primitives', 'two_objects.launch.py'],
                     stdout=launch_out, stderr=subprocess.STDOUT, start_new_session=True))
    wait_for(lambda: len(poses) == 2 and all(abs(p.z - .815) < .01 for p in poses.values()),
             60, 'Objects did not settle on table')
    client = node.create_client(ListControllers, '/controller_manager/list_controllers')
    wait_for(client.service_is_ready, 15, 'Controller manager unavailable')
    future = client.call_async(ListControllers.Request())
    wait_for(future.done, 10, 'Controller query timed out')
    controllers = {c.name: c.state for c in future.result().controller}
    assert all(controllers.get(c) == 'active' for c in
        ('joint_state_broadcaster','joint_trajectory_controller','robotiq_gripper_controller')), controllers
    print('Both objects and controllers ready', flush=True)
    seed = subprocess.Popen([seed_binary, 'ur10_two_objects'], stdin=subprocess.PIPE, env=seed_env,
                            stdout=seed_out, stderr=subprocess.STDOUT, text=True, start_new_session=True)
    processes.append(seed)
    wait_for(lambda: node.count_subscribers('/seed_ur10_two_objects/stream') > 0, 15, 'SEED unavailable')
    if os.environ.get('TWO_OBJECT_MODE') == 'named':
        seed.stdin.write('two_objects_demo\n'); seed.stdin.flush()
    else:
        base = get_package_share_directory('task_planner') + '/'
        result = subprocess.run(['ros2','run','task_planner','pddl_to_seed_two_objects',
            '--domain',base+'pddl/two_objects_domain.pddl', '--problem',base+'pddl/two_objects_problem.pddl',
            '--mapping',base+'config/two_objects_action_mapping.yaml','--execute'],capture_output=True,text=True,timeout=45)
        print(result.stdout, flush=True)
        assert result.returncode == 0, result.stderr
    wait_for(lambda: completed == expected, 240, 'Eight-step sequence did not finish')
    success_message = ('two_objects_demo success!' if os.environ.get('TWO_OBJECT_MODE') == 'named'
                       else 'sequence accomplished!')
    wait_for(lambda: success_message in seed_log.read_text(), 15, 'SEED did not recognize success')
    wait_for(lambda: states.get('object.placed(red_connector,red_place)') and states.get('object.placed(blue_peg,blue_place)'),
             10, 'Object-specific placement facts are missing')
    assert not states.get('object.held(red_connector)') and not states.get('object.held(blue_peg)'), states
    wait_for(lambda: math.dist([poses['red_connector'].x,poses['red_connector'].y,poses['red_connector'].z],[.5,.35,.815]) < .035
             and math.dist([poses['blue_peg'].x,poses['blue_peg'].y,poses['blue_peg'].z],[.5,-.35,.815]) < .035,
             10, 'Final object locations incorrect')
    print('PASS: both objects placed in order, distinct SEED goals satisfied', flush=True)
    print({name:(p.x,p.y,p.z) for name,p in poses.items()}, flush=True)
except Exception:
    print('LAUNCH LOG TAIL:\n'+launch_log.read_text()[-13000:],flush=True)
    print('SEED LOG TAIL:\n'+seed_log.read_text()[-7000:],flush=True)
    raise
finally:
    for proc in reversed(processes):
        if proc.poll() is None:
            proc.send_signal(signal.SIGINT)
            try: proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid,signal.SIGKILL);proc.wait()
    launch_out.close();seed_out.close();node.destroy_node();rclpy.shutdown()

seed_temp.cleanup()
