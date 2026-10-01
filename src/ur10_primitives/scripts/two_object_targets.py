#!/usr/bin/env python3
"""External numeric poses for the two-object simulation; never commands motion."""
import math
import re
from array import array

import rclpy
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.node import Node


class Targets(Node):
    def __init__(self):
        super().__init__('two_object_targets')
        self.declare_parameter('frame_id', 'world')
        self.declare_parameter('locations', ['red_pick', 'red_place', 'blue_pick', 'blue_place'],
                               ParameterDescriptor(read_only=True))
        self.names = self.get_parameter('locations').value
        if not self.names or len(set(self.names)) != len(self.names) or any(
                not re.fullmatch(r'[a-z][a-z0-9_]*', name) for name in self.names):
            raise ValueError('locations must be unique symbolic names')
        for name in self.names:
            self.declare_parameter(name, [float('nan')] * 7)
        self.add_on_set_parameters_callback(self.validate)
        result = self.validate([self.get_parameter(name) for name in ['frame_id'] + list(self.names)])
        if not result.successful:
            raise ValueError(result.reason)
        self.publishers_by_location = {
            name: self.create_publisher(PoseStamped, '/ur10/two_objects/targets/' + name, 10)
            for name in self.names}
        self.create_timer(0.2, self.publish)

    def validate(self, parameters):
        for parameter in parameters:
            value = parameter.value
            if parameter.name == 'frame_id' and (not isinstance(value, str) or not value):
                return SetParametersResult(successful=False, reason='frame_id must be nonempty')
            if parameter.name in self.names:
                if not isinstance(value, (list, tuple, array)) or len(value) != 7 or not all(math.isfinite(v) for v in value):
                    return SetParametersResult(successful=False, reason=parameter.name + ' needs seven finite values')
                if abs(sum(v*v for v in value[3:]) - 1.0) > 0.01:
                    return SetParametersResult(successful=False, reason='Quaternion must have unit length')
        return SetParametersResult(successful=True)

    def publish(self):
        for name, publisher in self.publishers_by_location.items():
            pose = self.get_parameter(name).value
            msg = PoseStamped()
            msg.header.frame_id = self.get_parameter('frame_id').value
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = pose[:3]
            msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = pose[3:]
            publisher.publish(msg)


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
