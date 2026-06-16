import links_and_nodes as ln
import rclpy
from rclpy.node import Node
from inverse_msgs.srv import ExecuteSkill, LearnSkill, PointToPointMotion, ReachPosition
from inverse_msgs.srv import ListTopics, StartRecording, StopRecording
from geometry_msgs.msg import Pose, PoseStamped
import json

import logging
logging.basicConfig()

class ServiceClient(Node):

    def __init__(self):
        super().__init__('service_client')
        # Robot Manager
        self.execute_skill_client = self.create_client(ExecuteSkill, '/execute_skill')
        self.learn_skill_client = self.create_client(LearnSkill, '/learn_skill')
        self.point_to_point_motion_client = self.create_client(PointToPointMotion, '/point_to_point_motion')
        self.reach_position_client = self.create_client(ReachPosition, '/reach_position')

        # Data Collection
        self.list_topics_client = self.create_client(ListTopics, '/list_topics')
        self.start_recording_client = self.create_client(StartRecording, '/start_recording')
        self.stop_recording_client = self.create_client(StopRecording, '/stop_recording')

        while not self.learn_skill_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service...')

        self.execute_skill_req = ExecuteSkill.Request()
        self.learn_skill_req = LearnSkill.Request()
        self.point_to_point_motion_req = PointToPointMotion.Request()
        self.reach_position_req = ReachPosition.Request()

        self.list_topics_req = ListTopics.Request()
        self.start_recording_req = StartRecording.Request()
        self.stop_recording_req = StopRecording.Request()


    def send_execute_skill_request(self, skill_name, use_learned_initial_pose, use_learned_final_pose, initial_pose, final_pose, max_vel):     
        self.execute_skill_req.skill_name = skill_name
        self.execute_skill_req.use_learned_initial_pose = use_learned_initial_pose
        self.execute_skill_req.use_learned_final_pose = use_learned_final_pose
        self.execute_skill_req.initial_pose = initial_pose
        self.execute_skill_req.final_pose = final_pose
        self.execute_skill_req.max_vel = max_vel
        self.future = self.execute_skill_client.call_async(self.execute_skill_req)

    def send_learn_skill_request(self, skill_name, registration_duration_secs, num_basis):
        self.learn_skill_req.skill_name = skill_name
        self.learn_skill_req.registration_duration_secs = registration_duration_secs
        self.learn_skill_req.num_basis = num_basis
        self.future = self.learn_skill_client.call_async(self.learn_skill_req)

    def send_point_to_point_motion_request(self, yo, g, max_vel, plan_y0_motion):
        self.point_to_point_motion_req.yo = yo
        self.point_to_point_motion_req.yo = g
        self.point_to_point_motion_req.yo = max_vel
        self.point_to_point_motion_req.yo = plan_y0_motion
        self.future = self.get_tasks_client.call_async(self.point_to_point_motion_req)

    def send_reach_position_request(self, desired_pos, max_vel, immediate_execution):
        self.reach_position_req.desired_pos = desired_pos
        self.reach_position_req.max_vel = max_vel
        self.reach_position_req.immediate_execution = immediate_execution

        self.future = self.get_tasks_client.call_async(self.get_tasks_req)

    def send_get_available_topics_request(self):
        self.future = self.list_topics_client.call_async(self.list_topics_req)

    def send_start_recording_request(self, topics):
        self.stop_recording_req.topics = topics
        self.future = self.start_recording_client.call_async(self.start_recording_req)

    def send_stop_recording_request(self):
        self.future = self.stop_recording_client.call_async(self.stop_recording_req)

class LnToCrfRos2InterfaceServiceProvider(ln.service_provider):
    def __init__(self):

        self.clnt = ln.client(self.__class__.__name__)

        # Service provider
        ln.service_provider.__init__(self, self.clnt, "ln_to_crf_ros2_interface_services")
        self.svc_execute_skill = self.wrap_service_provider(
            "execute_skill", "crf_ros2_interface_services/execute_skill"
        )

        self.svc_learn_skill = self.wrap_service_provider(
            "learn_skill", "crf_ros2_interface_services/learn_skill"
        )

        self.svc_point_to_point_motion = self.wrap_service_provider(
            "point_to_point_motion", "crf_ros2_interface_services/point_to_point_motion"
        )

        self.svc_reach_position = self.wrap_service_provider(
            "reach_position", "crf_ros2_interface_services/reach_position"
        )

        self.svc_get_available_topics = self.wrap_service_provider(
            "get_topics", "crf_ros2_interface_services/get_available_topics"
        )

        self.svc_start_recording = self.wrap_service_provider(
            "start_recording", "crf_ros2_interface_services/start_recording"
        )

        self.svc_stop_recording = self.wrap_service_provider(
            "stop_recording", "crf_ros2_interface_services/stop_recording"
        )

        # self.clnt.handle_service_group_in_thread_pool(None, "main_pool")
                # Init logger
        self._logger = logging.getLogger(self.__class__.__name__)
        self._logger.setLevel("DEBUG")

        rclpy.init()
        self.client = ServiceClient()
        self.client.get_logger().info('Service manager initialized!')

    def vector_to_pose_stamped(self, vec, frame_id="base_link"):
        """
        Convert a 7D vector [x, y, z, qx, qy, qz, qw] into a PoseStamped.

        Args:
            vec (list/tuple): length-7 iterable
            frame_id (str): reference frame

        Returns:
            PoseStamped
        """

        if len(vec) != 7:
            raise ValueError("Input vector must have 7 elements: [x, y, z, qx, qy, qz, qw]")

        pose = PoseStamped()

        # # Header
        # pose.header.frame_id = frame_id
        # pose.header.stamp = self.get_clock().now().to_msg()

        # Position
        pose.pose.position.x = float(vec[0])
        pose.pose.position.y = float(vec[1])
        pose.pose.position.z = float(vec[2])

        # Orientation
        pose.pose.orientation.x = float(vec[3])
        pose.pose.orientation.y = float(vec[4])
        pose.pose.orientation.z = float(vec[5])
        pose.pose.orientation.w = float(vec[6])

        return pose
    
    def vector_to_pose(self, vec, frame_id="base_link"):
        """
        Convert a 7D vector [x, y, z, qx, qy, qz, qw] into a PoseStamped.

        Args:
            vec (list/tuple): length-7 iterable
            frame_id (str): reference frame

        Returns:
            PoseStamped
        """

        if len(vec) != 7:
            raise ValueError("Input vector must have 7 elements: [x, y, z, qx, qy, qz, qw]")

        pose = Pose()

        # # Header
        # pose.header.frame_id = frame_id
        # pose.header.stamp = self.get_clock().now().to_msg()

        # Position
        pose.position.x = float(vec[0])
        pose.position.y = float(vec[1])
        pose.position.z = float(vec[2])

        # Orientation
        pose.orientation.x = float(vec[3])
        pose.orientation.y = float(vec[4])
        pose.orientation.z = float(vec[5])
        pose.orientation.w = float(vec[6])

        return pose
    
    def pose_stamped_to_vec(self, pose):
        """
        Convert a PoseStamped into a 7D vector:
        [x, y, z, qx, qy, qz, qw]

        Args:
            pose (Pose)

        Returns:
            list[float]
        """

        if not isinstance(pose, Pose):
            raise TypeError("Input must be a Pose")

        p = pose.position
        q = pose.orientation

        vec = [
            float(p.x),
            float(p.y),
            float(p.z),
            float(q.x),
            float(q.y),
            float(q.z),
            float(q.w),
        ]

        return vec

    def execute_skill(self, skill_name, use_learned_initial_pose, use_learned_final_pose, initial_pose, final_pose, max_vel):
        self.client.send_execute_skill_request(skill_name, 
                                               bool(use_learned_initial_pose), 
                                               bool(use_learned_final_pose), 
                                               self.vector_to_pose_stamped(json.loads(initial_pose)), 
                                               self.vector_to_pose_stamped(json.loads(final_pose)), 
                                               max_vel)
        
        self.client.get_logger().info('Calling execute_skill service!')
        success = None
        while rclpy.ok():
            rclpy.spin_once(self.client)
            if self.client.future.done():
                try:
                    response = self.client.future.result()
                    if response.success:
                        success = "Success"
                except Exception as e:
                    self.client.get_logger().error(f'Service call failed: {e}')
                else:
                    self.client.get_logger().info(f'Received output: {response}')
                break

        return {
            "error_message": success
        }
    
    def learn_skill(self, skill_name, registration_duration_secs, num_basis):
        
        #print("I've got: ", skill_name, registration_duration_secs, num_basis)
        self.client.get_logger().info(skill_name, registration_duration_secs, num_basis)

        self.client.send_learn_skill_request(skill_name, registration_duration_secs, num_basis)
        self.client.get_logger().info('Calling learn_skill service!')
        success = None
        total_demos_time = None
        initial_pose = None
        final_pose = None
        while rclpy.ok():
            rclpy.spin_once(self.client)
            if self.client.future.done():
                try:
                    response = self.client.future.result()

                    if response.success:
                        success = "Success"
                    
                    total_demos_time = response.total_demonstration_time
                    initial_pose = self.pose_to_vec(response.initial_pose)
                    final_pose = self.pose_to_vec(response.final_pose)

                except Exception as e:
                    self.client.get_logger().error(f'Service call failed: {e}')
                else:
                    self.client.get_logger().info(f'Received output: {response}')
                break

        return {
            "error_message": success,
            "total_demonstration_time": total_demos_time,
            "initial_pose": initial_pose,
            "final_pose": final_pose
        }
    
    def point_to_point_motion(self, yo, g, max_vel, plan_y0_motion):
        
        self.client.send_execute_skill_request( yo, 
                                                g,
                                                max_vel, 
                                                plan_y0_motion )
        
        self.client.get_logger().info('Calling get_tasks service!')
        success = None
        while rclpy.ok():
            rclpy.spin_once(self.client)
            if self.client.future.done():
                try:
                    response = self.client.future.result()
                    if response.success:
                        success = "Success"

                except Exception as e:
                    self.client.get_logger().error(f'Service call failed: {e}')
                else:
                    self.client.get_logger().info(f'Received output: {response}')
                break


        return {
            "error_message": success
        }
    
    def reach_position(self, desired_pos, max_vel, immediate_execution):
        self.client.send_reach_position_request(desired_pos, max_vel, immediate_execution)

        self.client.get_logger().info('Calling get_tasks service!')
        success = None
        while rclpy.ok():
            rclpy.spin_once(self.client)
            if self.client.future.done():
                try:
                    response = self.client.future.result()
                    if response.success:
                        success = "Success"
                except Exception as e:
                    self.client.get_logger().error(f'Service call failed: {e}')
                else:
                    self.client.get_logger().info(f'Received output: {response}')
                break


        return {
            "error_message": success
        }
    
    def get_topics(self):
        self.client.send_reach_position_request()

        self.client.get_logger().info('Calling get_tasks service!')
        topics_list = None
        while rclpy.ok():
            rclpy.spin_once(self.client)
            if self.client.future.done():
                try:
                    response = self.client.future.result()
                    topics_list = response.topics 
                except Exception as e:
                    self.client.get_logger().error(f'Service call failed: {e}')
                else:
                    self.client.get_logger().info(f'Received output: {response}')
                break

        return {
            "error_message": "Success",
            "topics": topics_list
        }
    
    def start_recording(self, topics):
        self.client.send_reach_position_request(topics)

        self.client.get_logger().info('Calling get_tasks service!')
        success = None
        while rclpy.ok():
            rclpy.spin_once(self.client)
            if self.client.future.done():
                try:
                    response = self.client.future.result()
                    if response.success:
                        success = "Success"
                except Exception as e:
                    self.client.get_logger().error(f'Service call failed: {e}')
                else:
                    self.client.get_logger().info(f'Received output: {response}')
                break


        return {
            "error_message": success
        }
    
    def stop_recording(self):
        self.client.send_reach_position_request()

        self.client.get_logger().info('Calling get_tasks service!')
        success = None
        while rclpy.ok():
            rclpy.spin_once(self.client)
            if self.client.future.done():
                try:
                    response = self.client.future.result()
                    if response.success:
                        success = "Success"
                except Exception as e:
                    self.client.get_logger().error(f'Service call failed: {e}')
                else:
                    self.client.get_logger().info(f'Received output: {response}')
                break


        return {
            "error_message": success
        }
    
    def run(self):
        self.handle_service_group_requests()

def main(args=None):
    
    sp = LnToCrfRos2InterfaceServiceProvider()
    sp.run()

if __name__ == '__main__':
    main()
