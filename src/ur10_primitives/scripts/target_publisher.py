#!/usr/bin/env python3
"""Replaceable pose source. Positions can be changed with ros2 param set."""
from array import array
import math
import rclpy
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node


class Targets(Node):
    def __init__(self):
        super().__init__('pick_place_targets')
        self.declare_parameter('frame_id', 'world')
        self.declare_parameter('pick_position', [0.55, 0.0, 0.825])
        self.declare_parameter('place_position', [0.5, 0.35, 0.835])
        self.declare_parameter('orientation_xyzw', [1.0, 0.0, 0.0, 0.0])
        self.add_on_set_parameters_callback(self.validate)
        initial = self.validate([self.get_parameter(n) for n in (
            'frame_id', 'pick_position', 'place_position', 'orientation_xyzw')])
        if not initial.successful:
            raise ValueError(initial.reason)
        self.pubs = {name: self.create_publisher(PoseStamped, '/ur10/targets/' + name, 10)
                     for name in ('pick', 'place')}
        self.create_timer(0.2, self.publish)

    def validate(self, parameters):
        for p in parameters:
            if p.name == 'frame_id' and (not isinstance(p.value, str) or not p.value):
                return SetParametersResult(successful=False, reason='frame_id must be nonempty')
            if p.name in ('pick_position', 'place_position', 'orientation_xyzw'):
                size = 4 if p.name == 'orientation_xyzw' else 3
                if not isinstance(p.value, (list, tuple, array)) or len(p.value) != size or not all(math.isfinite(v) for v in p.value):
                    return SetParametersResult(successful=False, reason=f'{p.name} needs {size} finite values')
                if size == 4 and abs(sum(v*v for v in p.value) - 1.0) > 0.01:
                    return SetParametersResult(successful=False, reason='Orientation must be a unit quaternion')
        return SetParametersResult(successful=True)

    def publish(self):
        for name, pub in self.pubs.items():
            msg = PoseStamped()
            msg.header.frame_id = self.get_parameter('frame_id').value
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = self.get_parameter(name + '_position').value
            msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = self.get_parameter('orientation_xyzw').value
            pub.publish(msg)


def main():
    rclpy.init()
    node = Targets()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
