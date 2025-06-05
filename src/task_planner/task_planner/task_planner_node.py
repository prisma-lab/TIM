#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from task_planner_msgs.msg import PlanningRequest

# PlanSys2 service imports
from plansys2_msgs.srv import (
    GetDomain,
    GetProblem, 
    AddProblem,
    GetPlan
)

import threading
import time


class TaskPlannerNode(Node):
    def __init__(self):
        super().__init__('task_planner_node')
        
        # Create callback group for service calls
        self.callback_group = ReentrantCallbackGroup()
        
        # Create subscriber for planning requests
        self.planning_request_sub = self.create_subscription(
            PlanningRequest,
            '/planning_request',  
            self.planning_request_callback,
            10,
            callback_group=self.callback_group
        )
        
        # Create service clients for PlanSys2
        self.setup_plansys2_clients()
        
        self.get_logger().info('Task Planner Node initialized and ready to receive planning requests')
    
    def setup_plansys2_clients(self):
        """Initialize PlanSys2 service clients"""
        
        # Domain Expert clients
        self.get_domain_client = self.create_client(
            GetDomain, 
            '/domain_expert/get_domain',
            callback_group=self.callback_group
        )
        
        # Problem Expert clients  
        self.get_problem_client = self.create_client(
            GetProblem,
            '/problem_expert/get_problem',
            callback_group=self.callback_group
        )
        
        self.add_problem_client = self.create_client(
            AddProblem,
            '/problem_expert/add_problem',
            callback_group=self.callback_group
        )
        
        # Planner client
        self.get_plan_client = self.create_client(
            GetPlan,
            '/planner/get_plan',
            callback_group=self.callback_group
        )
        
        self.get_logger().info('PlanSys2 service clients created')
    
    def wait_for_services(self, timeout_sec=10.0):
        """Wait for all PlanSys2 services to become available"""
        services = [
            (self.get_domain_client, '/domain_expert/get_domain'),
            (self.get_problem_client, '/problem_expert/get_problem'),
            (self.add_problem_client, '/problem_expert/add_problem'),
            (self.get_plan_client, '/planner/get_plan')
        ]
        
        for client, service_name in services:
            if not client.wait_for_service(timeout_sec=timeout_sec):
                self.get_logger().warning(f'Service {service_name} not available after {timeout_sec} seconds')
                return False
                
        self.get_logger().info('All PlanSys2 services are available')
        return True
    
    def check_plansys2_ready(self):
        """Check if PlanSys2 is ready by verifying domain is loaded"""
        try:
            # Check if domain is loaded
            req = GetDomain.Request()
            future = self.get_domain_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            
            if future.result() is not None and future.result().success:
                self.get_logger().info('Domain is loaded in PlanSys2')
                return True
            else:
                self.get_logger().warning('Domain not loaded in PlanSys2')
                return False
                
        except Exception as e:
            self.get_logger().error(f'Error checking PlanSys2 readiness: {str(e)}')
            return False
    
    def planning_request_callback(self, msg):
        """Handle incoming planning requests"""
        self.get_logger().info(f'Received planning request for domain: {msg.domain[:50]}...')
        
        # Process the planning request in a separate thread to avoid blocking
        thread = threading.Thread(target=self.process_planning_request, args=(msg,))
        thread.start()
    
    def process_planning_request(self, request_msg):
        """Process the planning request using PlanSys2"""
        try:
            # Wait for services to be available
            if not self.wait_for_services():
                self.get_logger().error('PlanSys2 services not available')
                return
            
            # Check if PlanSys2 is properly initialized
            if not self.check_plansys2_ready():
                self.get_logger().error('PlanSys2 not ready - make sure domain is loaded')
                return
            
            # Step 1: Note - Clear operations may need to be handled differently
            # as Clear services might not be available in all PlanSys2 versions
            self.get_logger().info('Processing planning request...')
            
            # Step 2: Set domain from the request (if supported)
            self.get_logger().info('Domain provided in request (note: may need manual loading)')
            
            # Step 3: Set problem from the request  
            self.get_logger().info('Setting problem...')
            if not self.set_problem(request_msg.problem):
                self.get_logger().error('Failed to set problem')
                return
            
            # Step 4: Generate plan
            self.get_logger().info('Generating plan...')
            plan = self.generate_plan()
            
            if plan:
                self.get_logger().info('Plan generated successfully!')
                self.display_plan(plan)
            else:
                self.get_logger().error('Failed to generate plan')
                
        except Exception as e:
            self.get_logger().error(f'Error processing planning request: {str(e)}')
    
    def set_problem(self, problem_content):
        """Set the PDDL problem"""
        try:
            # Add a small delay to ensure services are ready
            time.sleep(1.0)
            
            req = AddProblem.Request()
            req.problem = problem_content
            
            self.get_logger().info('Sending problem to PlanSys2...')
            future = self.add_problem_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
            
            if future.result() is not None:
                if hasattr(future.result(), 'success') and future.result().success:
                    self.get_logger().info('Problem set successfully')
                    return True
                else:
                    # Check if there's an error_info field
                    if hasattr(future.result(), 'error_info'):
                        error_msg = future.result().error_info
                    else:
                        error_msg = 'Service call completed but success status unclear'
                    self.get_logger().error(f'Failed to set problem: {error_msg}')
                    self.get_logger().info(f'Service response: {future.result()}')
                    return False
            else:
                self.get_logger().error('Failed to set problem: No response from service')
                return False
                
        except Exception as e:
            self.get_logger().error(f'Error setting problem: {str(e)}')
            return False
    
    def generate_plan(self):
        """Generate a plan using PlanSys2 planner"""
        try:
            req = GetPlan.Request()
            
            future = self.get_plan_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
            
            if future.result() is not None and future.result().success:
                return future.result().plan
            else:
                error_msg = future.result().error_info if future.result() else 'Unknown error'
                self.get_logger().error(f'Failed to generate plan: {error_msg}')
                return None
                
        except Exception as e:
            self.get_logger().error(f'Error generating plan: {str(e)}')
            return None
    
    def display_plan(self, plan):
        """Display the generated plan"""
        if not plan or not plan.items:
            self.get_logger().info('Empty plan received')
            return
        
        self.get_logger().info('=== GENERATED PLAN ===')
        for i, item in enumerate(plan.items):
            self.get_logger().info(f'Step {i+1}: {item.action} (time: {item.time:.2f})')
        self.get_logger().info('=== END OF PLAN ===')


def main(args=None):
    rclpy.init(args=args)
    
    node = TaskPlannerNode()
    
    # Use MultiThreadedExecutor to handle concurrent service calls
    executor = MultiThreadedExecutor()
    
    try:
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
