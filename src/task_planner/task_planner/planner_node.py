"""ROS wrapper around Fast Downward for the existing TIM planning interface."""

import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from task_planner_msgs.msg import PlanningRequest

from task_planner.fast_downward import PlanningError, run_fast_downward


class TaskPlanner(Node):
    def __init__(self):
        super().__init__('task_planner')
        self.declare_parameter('planner_executable', 'fast-downward.py')
        self.declare_parameter('planner_build', '')
        self.declare_parameter('planner_search', 'astar(lmcut())')
        self.declare_parameter('planner_timeout', 30.0)
        self.subscription = self.create_subscription(
            PlanningRequest, '/planning_request', self.planning_request_callback, 10)
        self.publisher = self.create_publisher(String, '/planner_result', 10)
        self.status_publisher = self.create_publisher(String, '/planner_status', 10)

    def planning_request_callback(self, msg):
        self.get_logger().info('Received planning request')
        try:
            actions = run_fast_downward(
                msg.domain, msg.problem,
                executable=self.get_parameter('planner_executable').value,
                build=self.get_parameter('planner_build').value or None,
                search=self.get_parameter('planner_search').value,
                timeout=self.get_parameter('planner_timeout').value)
            status = {'status': 'succeeded', 'actions': actions, 'empty_plan': not actions}
            result = ','.join(actions)
            self.get_logger().info(f'Planner produced {len(actions)} actions')
        except PlanningError as error:
            # The legacy SEED plan behavior treats an empty result as failure.
            # Never send error prose where it expects executable PDDL actions.
            result = ''
            status = {'status': 'failed', 'detail': str(error)}
            self.get_logger().error(str(error))
        self.status_publisher.publish(String(data=json.dumps(status)))
        self.publisher.publish(String(data=result))


def main(args=None):
    rclpy.init(args=args)
    node = TaskPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
