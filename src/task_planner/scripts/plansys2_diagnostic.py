#!/usr/bin/env python3
"""
PlanSys2 Diagnostic Script
Run this to check the status of PlanSys2 services and components
"""

import rclpy
from rclpy.node import Node
from plansys2_msgs.srv import GetDomain, GetProblem, AddProblem


class PlanSys2Diagnostic(Node):
    def __init__(self):
        super().__init__('plansys2_diagnostic')
        
        # Create service clients
        self.get_domain_client = self.create_client(GetDomain, '/domain_expert/get_domain')
        self.get_problem_client = self.create_client(GetProblem, '/problem_expert/get_problem')
        self.add_problem_client = self.create_client(AddProblem, '/problem_expert/add_problem')
        
    def run_diagnostics(self):
        """Run comprehensive diagnostics"""
        print("=== PlanSys2 Diagnostic ===\n")
        
        # 1. Check service availability
        print("1. Checking service availability...")
        services = [
            (self.get_domain_client, '/domain_expert/get_domain'),
            (self.get_problem_client, '/problem_expert/get_problem'),
            (self.add_problem_client, '/problem_expert/add_problem')
        ]
        
        for client, service_name in services:
            if client.wait_for_service(timeout_sec=2.0):
                print(f"   ✓ {service_name} - Available")
            else:
                print(f"   ✗ {service_name} - Not Available")
        
        print()
        
        # 2. Check domain status
        print("2. Checking domain status...")
        try:
            req = GetDomain.Request()
            future = self.get_domain_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            
            if future.result() is not None:
                if hasattr(future.result(), 'success') and future.result().success:
                    print("   ✓ Domain loaded successfully")
                    if hasattr(future.result(), 'domain'):
                        domain_preview = future.result().domain[:100] + "..." if len(future.result().domain) > 100 else future.result().domain
                        print(f"   Domain preview: {domain_preview}")
                else:
                    print("   ✗ Domain not loaded or error occurred")
                    if hasattr(future.result(), 'error_info'):
                        print(f"   Error: {future.result().error_info}")
            else:
                print("   ✗ No response from domain service")
        except Exception as e:
            print(f"   ✗ Error checking domain: {e}")
        
        print()
        
        # 3. Check problem status
        print("3. Checking problem status...")
        try:
            req = GetProblem.Request()
            future = self.get_problem_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            
            if future.result() is not None:
                if hasattr(future.result(), 'success') and future.result().success:
                    print("   ✓ Problem service responsive")
                    if hasattr(future.result(), 'problem') and future.result().problem:
                        print("   Current problem is set")
                    else:
                        print("   No problem currently set")
                else:
                    print("   ✗ Problem service error")
                    if hasattr(future.result(), 'error_info'):
                        print(f"   Error: {future.result().error_info}")
            else:
                print("   ✗ No response from problem service")
        except Exception as e:
            print(f"   ✗ Error checking problem: {e}")
        
        print()
        
        # 4. Test simple problem addition
        print("4. Testing simple problem addition...")
        simple_problem = """
(define (problem test-problem)
  (:domain simple)
  (:objects 
    robot1 - robot
    loc1 loc2 - location)
  (:init 
    (at robot1 loc1)
    (adjacent loc1 loc2))
  (:goal (at robot1 loc2)))
"""
        
        try:
            req = AddProblem.Request()
            req.problem = simple_problem
            
            future = self.add_problem_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
            
            if future.result() is not None:
                print(f"   Service response received: {future.result()}")
                if hasattr(future.result(), 'success'):
                    if future.result().success:
                        print("   ✓ Problem added successfully")
                    else:
                        print("   ✗ Problem addition failed")
                        if hasattr(future.result(), 'error_info'):
                            print(f"   Error: {future.result().error_info}")
                else:
                    print("   ? Response received but no success field")
            else:
                print("   ✗ No response from add problem service")
        except Exception as e:
            print(f"   ✗ Error testing problem addition: {e}")
        
        print("\n=== End Diagnostic ===")


def main():
    rclpy.init()
    
    diagnostic = PlanSys2Diagnostic()
    diagnostic.run_diagnostics()
    
    diagnostic.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
