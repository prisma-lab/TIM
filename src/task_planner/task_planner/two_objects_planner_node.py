"""Receive PDDL, map the complete plan, and optionally submit it to SEED."""
from collections import deque
import json
import math
from pathlib import Path
import time
from uuid import uuid4

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.duration import Duration
from std_msgs.msg import String
from task_planner_msgs.msg import PlanningRequest

from task_planner.fast_downward import PlanningError, run_fast_downward
from task_planner.pipeline_worker import PipelineWorker, spin_node
from task_planner.two_objects_to_seed import load_mapping, sequence_for


def plan_request(domain, problem, settings):
    mapping = load_mapping(settings['mapping'])
    actions = run_fast_downward(
        domain, problem, executable=settings['planner_executable'],
        build=settings['planner_build'] or None,
        search=settings['planner_search'], timeout=settings['planner_timeout'])
    return {'actions': actions, 'sequence': sequence_for(actions, mapping),
            'empty_plan': not actions, 'submitted': False}


class TwoObjectsPlanner(PipelineWorker):
    def __init__(self):
        super().__init__('two_objects_planner', '/two_objects_planner/status')
        share = Path(get_package_share_directory('task_planner'))
        defaults = {
            'mapping': str(share / 'config/two_objects_action_mapping_v1.yaml'),
            'planner_executable': 'fast-downward.py', 'planner_build': '',
            'planner_search': 'astar(blind())', 'planner_timeout': 30.0,
            'execute': False, 'seed_topic': '/seed_ur10_two_objects/stream',
            'wait_for_seed': 10.0, 'planning_topic': '/two_objects/planning_request',
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.seed_publisher = self.create_publisher(
            String, self.get_parameter('seed_topic').value, 10)
        self.result_publisher = self.create_publisher(String, '/two_objects_planner/result', 10)
        self.subscription = self.create_subscription(
            PlanningRequest, self.get_parameter('planning_topic').value, self.request_callback, 10)
        self._seen = deque(maxlen=256)

    def request_callback(self, msg):
        request_id = msg.header.frame_id or str(uuid4())
        if request_id in self._seen:
            self.status('rejected', request_id, detail='Duplicate request ID')
            return
        names = ('mapping', 'planner_executable', 'planner_build', 'planner_search',
                 'planner_timeout', 'execute', 'wait_for_seed')
        settings = {name: self.get_parameter(name).value for name in names}
        if self.start_work(request_id, self._plan_and_submit, msg.domain, msg.problem, settings):
            self._seen.append(request_id)

    def _plan_and_submit(self, domain, problem, settings):
        result = plan_request(domain, problem, settings)
        if settings['execute'] and result['sequence'] is not None:
            timeout = settings['wait_for_seed']
            if not math.isfinite(timeout) or timeout <= 0:
                raise PlanningError('wait_for_seed must be positive and finite')
            deadline = time.monotonic() + timeout
            while self.seed_publisher.get_subscription_count() == 0:
                if not rclpy.ok():
                    raise PlanningError('ROS shut down before submission')
                if time.monotonic() >= deadline:
                    raise PlanningError('No SEED subscriber; no sequence submitted')
                time.sleep(0.05)
            if not rclpy.ok():
                raise PlanningError('ROS shut down before submission')
            self.seed_publisher.publish(String(data=result['sequence']))
            if not self.seed_publisher.wait_for_all_acked(Duration(seconds=3)):
                raise PlanningError('SEED transport acknowledgement timed out; submission is uncertain. Inspect SEED before retrying.')
            result['submitted'] = True
        return result

    def finish_work(self, request_id, result):
        self.result_publisher.publish(String(data=json.dumps({'request_id': request_id, **result})))
        self.status('submitted' if result['submitted'] else 'planned', request_id,
                    empty_plan=result['empty_plan'])
        self.get_logger().info('SEED sequence: ' + str(result['sequence']))


def main(args=None):
    spin_node(TwoObjectsPlanner, args)
