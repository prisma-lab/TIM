import rclpy
from rclpy.node import Node

from task_planner_msgs.msg import PlanningRequest
from std_msgs.msg import String

class TaskPlanner(Node):
    def __init__(self):
        super().__init__('task_planner')

        self.subscription = self.create_subscription(
            PlanningRequest,
            '/planning_request',
            self.planning_request_callback,
            10)
        self.publisher = self.create_publisher(String, '/planner_result', 10)

    def planning_request_callback(self, msg):
        self.get_logger().info(f"Received planning request for domain and problem.")

        # Save PDDL files
        with open("/tmp/domain.pddl", "w") as f:
            f.write(msg.domain)
        with open("/tmp/problem.pddl", "w") as f:
            f.write(msg.problem)

        # Example: Call PlanSys2 or any PDDL planner here.
        # For demonstration, let's pretend you call an external planner (replace this!)
        plan = self.run_plansys2_planner("/tmp/domain.pddl", "/tmp/problem.pddl")
        
        result_msg = String()
        result_msg.data = plan
        self.publisher.publish(result_msg)
        self.get_logger().info("Published planning result.")

    def run_plansys2_planner(self, domain_path, problem_path):
        import subprocess
        try:
            # Call Fast Downward to solve the planning problem
            # The output plan will be written to 'sas_plan'
            result = subprocess.run(
                [
                    'fast-downward.py',
                    '--build',
                    '/home/user/ros2_ws/build/builds/release/bin',
                    domain_path,
                    problem_path,
                    '--search',
                    "astar(lmcut())"
                ],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                timeout=30
            )
            # Check if planner succeeded
            if result.returncode != 0:
                return f"Planner failed: {result.stderr}"
            
            # Read the output plan
            plan_path = 'sas_plan'
            try:
                with open(plan_path, 'r') as f:
                    plan = f.read()
                return plan
            except FileNotFoundError:
                return "Planner did not produce a plan file."
        except Exception as e:
            return f"Planner execution error: {e}"

def main(args=None):
    rclpy.init(args=args)
    node = TaskPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()