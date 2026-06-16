import os
import rclpy
from rclpy.node import Node
import links_and_nodes as ln
import numpy as np
import time

from sensor_msgs.msg import JointState
from std_msgs.msg import String

class TopicManager(Node):
    def __init__(self):
        super().__init__("Topic_manager")
        #ROS2 topic setup
        self.ros2_jnt_state_subscriber = self.create_subscription(JointState, 
                                                                "/joint_states", 
                                                                self.ros2_joint_state_callback,
                                                                10)
        
        self.ros2_task_status_subscriber = self.create_subscription(String, 
                                                                "/planner_result", 
                                                                self.ros2_task_status_callback,
                                                                1000)

        # Ln setup
        ln_clnt = ln.client("Topic_manager")

        # Create ln joint_state pub topic
        self.ln_joint_state_port = ln_clnt.publish("crf_ros2_interface_topics.robot_state", "crf_ros2_interface_topics/robot_state")
        self.ln_task_status_port = ln_clnt.publish("crf_ros2_interface_topics.task_status", "crf_ros2_interface_topics/task_status")

        # self.publish_params_to_ros2_timer = self.create_timer(1, self.publish_params_to_ros2)
        self.get_logger().info('Topic manager initialized!')

    def ros2_joint_state_callback(self, msg: JointState):
        # Read joint state from ROS2 and publish to ln
        for i, (pos) in enumerate(msg.position):
            self.ln_joint_state_port.packet.joint_state[i] = pos

        self.ln_joint_state_port.write()

    def ros2_task_status_callback(self, msg: String):
        # Read joint state from ROS2 and publish to ln
        self.get_logger().info("data  " + msg.data)
        # self.ln_task_status_port.packet.data = "\0" *10000
        self.ln_task_status_port.packet.data = msg.data.ljust(9999, '\0')
        self.get_logger().info("packet_data  " + self.ln_task_status_port.packet.data + "  " + str(time.time()))
        self.ln_task_status_port.write()
        
    # def publish_params_to_ros2(self):
    #     self.task_param_topic.read()
    #     param = self.task_param_topic.packet.task_param

        # TODO create ROS2 msg and send it

def main(args=None):
    rclpy.init(args=args)
    topic_mamanger = TopicManager()
    rclpy.spin(topic_mamanger)

    topic_mamanger.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
