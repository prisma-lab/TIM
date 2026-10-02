"""Run one slow operation at a time without blocking the ROS executor."""
from concurrent.futures import ThreadPoolExecutor
import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class PipelineWorker(Node):
    def __init__(self, name, status_topic):
        super().__init__(name)
        self._worker = ThreadPoolExecutor(max_workers=1, thread_name_prefix=name)
        self._future = None
        self._request_id = ''
        self.status_publisher = self.create_publisher(String, status_topic, 10)
        self._poll_timer = self.create_timer(0.1, self._poll)

    def status(self, state, request_id, **detail):
        data = {'status': state, 'request_id': request_id, **detail}
        self.status_publisher.publish(String(data=json.dumps(data)))
        self.get_logger().info(json.dumps(data))

    def start_work(self, request_id, operation, *args):
        if self._future is not None:
            self.status('rejected', request_id, detail='Node is busy; retry after its result.')
            return False
        self._request_id = request_id
        self.status('started', request_id)
        self._future = self._worker.submit(operation, *args)
        return True

    def _poll(self):
        if self._future is None or not self._future.done():
            return
        future, request_id = self._future, self._request_id
        self._future = None
        try:
            self.finish_work(request_id, future.result())
        except Exception as error:
            self.status('failed', request_id, detail=str(error))
            self.get_logger().error(str(error))

    def destroy_node(self):
        # HTTP/planner timeouts bound outstanding work; do not destroy publishers
        # while a worker could still be submitting a SEED sequence.
        self._worker.shutdown(wait=True, cancel_futures=True)
        return super().destroy_node()


def spin_node(node_type, args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = node_type()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
