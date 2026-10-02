"""Pipeline boundaries and ROS delivery without a model server or robot."""
import json
from pathlib import Path
import time

import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String
from task_planner_msgs.msg import PlanningRequest

from task_planner import vlm_grounder_node as grounder
from task_planner import two_objects_planner_node as planner
from task_planner.fast_downward import PlanningError

PACKAGE = Path(__file__).resolve().parents[1]
PROBLEM = {
    'objects': [{'name': name, 'type': kind} for name, kind in (
        ('red_connector', 'object'), ('blue_peg', 'object'),
        ('home', 'location'), ('loc1', 'location'), ('loc2', 'location'),
        ('loc3', 'location'), ('loc4', 'location'))],
    'init': [
        {'predicate': 'eeAt', 'args': ['home']},
        {'predicate': 'gripperEmpty', 'args': []},
        {'predicate': 'objectAt', 'args': ['red_connector', 'loc2']},
        {'predicate': 'objectAt', 'args': ['blue_peg', 'loc1']},
        {'predicate': 'clearLoc', 'args': ['loc3']},
        {'predicate': 'clearLoc', 'args': ['loc4']}],
    'goal': [{'predicate': 'objectAt', 'args': ['red_connector', 'loc4']},
             {'predicate': 'objectAt', 'args': ['blue_peg', 'loc3']}],
}
ACTIONS = ['(move_a_b home loc2)', '(pick red_connector loc2)',
           '(move_a_b loc2 loc4)', '(place red_connector loc4)']


def settings(tmp_path):
    return {
        'domain': str(PACKAGE / 'pddl/serdar_two_objects_domain.pddl'),
        'scene': str(PACKAGE / 'resource/descriptions/scene.txt'),
        'goal': str(PACKAGE / 'resource/descriptions/goal.txt'),
        'predicate_definitions': str(PACKAGE / 'resource/descriptions/predicates.txt'),
        'model': 'test', 'endpoint': 'http://unused', 'vlm_timeout': 1.,
        'mode': 'full', 'context': '', 'output_directory': str(tmp_path),
    }


def test_grounding_serializes_validated_problem_and_optional_files(monkeypatch, tmp_path):
    monkeypatch.setattr(grounder, 'generate_problem', lambda *a, **kw: (PROBLEM, {}))
    domain, pddl = grounder.ground_image('unused.png', settings(tmp_path))
    assert '(domain manipulator-pick-place)' in domain
    assert '(objectAt red_connector loc4)' in pddl
    assert (tmp_path / 'generated_problem.pddl').read_text() == pddl
    assert json.loads((tmp_path / 'generated_problem.json').read_text()) == PROBLEM


def test_invalid_generation_does_not_write_problem(monkeypatch, tmp_path):
    invalid = {**PROBLEM, 'goal': [{'predicate': 'invented', 'args': []}]}
    monkeypatch.setattr(grounder, 'generate_problem', lambda *a, **kw: (invalid, {}))
    with pytest.raises(ValueError):
        grounder.ground_image('unused.png', settings(tmp_path))
    assert not list(tmp_path.iterdir())


def test_init_only_preserves_context(monkeypatch, tmp_path):
    context = tmp_path / 'context.json'
    context.write_text(json.dumps({'objects': PROBLEM['objects'], 'goal': PROBLEM['goal']}))
    options = {**settings(tmp_path), 'mode': 'init-only', 'context': str(context),
               'output_directory': ''}
    calls = []
    def generate(*args, **kwargs):
        calls.append(kwargs)
        return PROBLEM, {}
    monkeypatch.setattr(grounder, 'generate_problem', generate)
    grounder.ground_image('unused.png', options)
    assert calls[0]['context']['goal'] == PROBLEM['goal']
    assert calls[0]['mode'] == 'init-only'


@pytest.fixture
def ros():
    # Never connect test execution to the robot's SEED stream.
    rclpy.init(args=['--ros-args', '-p', 'seed_topic:=/test_vlm_pipeline/seed'])
    executor = SingleThreadedExecutor()
    nodes = []
    def add(node):
        nodes.append(node)
        executor.add_node(node)
        return node
    def wait(predicate):
        deadline = time.monotonic() + 5
        while not predicate() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.02)
        assert predicate(), 'ROS pipeline did not reach the expected state'
    yield add, wait
    for node in reversed(nodes):
        executor.remove_node(node)
        node.destroy_node()
    executor.shutdown()
    rclpy.try_shutdown()


@pytest.mark.parametrize('execute', [False, True])
def test_ros_grounding_to_mapped_plan(monkeypatch, ros, execute):
    add, wait = ros
    monkeypatch.setattr(grounder, 'generate_problem', lambda *a, **kw: (PROBLEM, {}))
    planned = []
    def run(domain, problem, **kwargs):
        planned.append((domain, problem))
        return ACTIONS
    monkeypatch.setattr(planner, 'run_fast_downward', run)
    p = add(planner.TwoObjectsPlanner())
    p.set_parameters([rclpy.parameter.Parameter('execute', value=execute)])
    g = add(grounder.VLMGrounder())
    observer = add(rclpy.create_node('pipeline_observer'))
    results, commands, requests = [], [], []
    subscriptions = [
        observer.create_subscription(String, '/two_objects_planner/result',
                                     lambda m: results.append(json.loads(m.data)), 10),
        observer.create_subscription(String, '/test_vlm_pipeline/seed',
                                     lambda m: commands.append(m.data), 10),
        observer.create_subscription(PlanningRequest, '/two_objects/planning_request',
                                     requests.append, 10),
    ]
    wait(lambda: g.publisher.get_subscription_count() >= 2 and
         p.result_publisher.get_subscription_count() > 0 and
         p.seed_publisher.get_subscription_count() > 0)
    g.image_callback(String(data='unused.png'))
    wait(lambda: bool(results))
    assert results[0]['submitted'] is execute
    assert 'pick(red_connector,red_pick)' in results[0]['sequence']
    assert planned[0][1] == requests[0].problem
    assert results[0]['request_id'] == requests[0].header.frame_id
    assert commands == ([results[0]['sequence']] if execute else [])
    # Repeated delivery of the same request must not submit twice.
    p.request_callback(requests[0])
    assert len(planned) == 1


@pytest.mark.parametrize('actions', [[], ['(unknown)']])
def test_empty_or_unmapped_plan_never_submits(monkeypatch, ros, actions):
    add, wait = ros
    monkeypatch.setattr(planner, 'run_fast_downward', lambda *a, **kw: actions)
    p = add(planner.TwoObjectsPlanner())
    options = {name: p.get_parameter(name).value for name in (
        'mapping', 'planner_executable', 'planner_build', 'planner_search',
        'planner_timeout', 'wait_for_seed')}
    options['execute'] = True
    # No SEED subscriber exists: an attempted submission would time out.
    if actions:
        with pytest.raises(PlanningError, match='No mapping'):
            p._plan_and_submit('domain', 'problem', options)
    else:
        result = p._plan_and_submit('domain', 'problem', options)
        assert result['empty_plan'] and not result['submitted']


def test_failed_grounding_publishes_no_request(monkeypatch, ros):
    add, wait = ros
    def fail(*args):
        raise RuntimeError('model unavailable')
    monkeypatch.setattr(grounder, 'ground_image', fail)
    g = add(grounder.VLMGrounder())
    observer = add(rclpy.create_node('failure_observer'))
    requests, statuses = [], []
    subscriptions = [
        observer.create_subscription(PlanningRequest, '/two_objects/planning_request', requests.append, 10),
        observer.create_subscription(String, '/vlm_grounder/status',
                                     lambda m: statuses.append(json.loads(m.data)), 10),
    ]
    wait(lambda: g.publisher.get_subscription_count() and
         g.status_publisher.get_subscription_count())
    g.image_callback(String(data='unused.png'))
    wait(lambda: any(s['status'] == 'failed' for s in statuses))
    assert not requests


def test_real_grounding_library_with_mocked_http(monkeypatch, tmp_path):
    import io
    from vlm_grounder.src import ollama_client
    payloads = []
    def response(request, timeout):
        payloads.append(json.loads(request.data))
        return io.BytesIO(json.dumps({'message': {'content': json.dumps(PROBLEM)}}).encode())
    monkeypatch.setattr(ollama_client.urllib.request, 'urlopen', response)
    image = tmp_path / 'scene.png'
    image.write_bytes(b'test image')
    domain, problem = grounder.ground_image(image, {**settings(tmp_path), 'output_directory': ''})
    assert '(gripperEmpty )' in problem
    assert payloads[0]['messages'][0]['images']
    assert not (tmp_path / 'generated_problem.pddl').exists()


def test_zero_argument_gripper_conflict_is_rejected(monkeypatch, tmp_path):
    invalid = {**PROBLEM, 'init': PROBLEM['init'] + [
        {'predicate': 'gripperHolding', 'args': ['red_connector']}]}
    monkeypatch.setattr(grounder, 'generate_problem', lambda *a, **kw: (invalid, {}))
    with pytest.raises(ValueError, match='hold an object and be empty'):
        grounder.ground_image('unused.png', settings(tmp_path))


def test_serdar_problem_with_actual_fast_downward(monkeypatch, tmp_path):
    import shutil
    if not shutil.which('fast-downward.py'):
        pytest.skip('Fast Downward unavailable')
    monkeypatch.setattr(grounder, 'generate_problem', lambda *a, **kw: (PROBLEM, {}))
    domain, pddl = grounder.ground_image('unused.png', settings(tmp_path))
    result = planner.plan_request(domain, pddl, {
        'mapping': str(PACKAGE / 'config/serdar_two_objects_action_mapping.yaml'),
        'planner_executable': 'fast-downward.py', 'planner_build': '',
        'planner_search': 'astar(blind())', 'planner_timeout': 30.})
    assert '(place red_connector loc4)' in result['actions']
    assert '(place blue_peg loc3)' in result['actions']
    assert 'place(red_connector,red_place)' in result['sequence']
    assert 'place(blue_peg,blue_place)' in result['sequence']
