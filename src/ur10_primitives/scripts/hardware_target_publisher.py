#!/usr/bin/env python3
"""Capture the current TCP pose once and publish nearby, fixed hardware targets."""

from copy import deepcopy
import math

from geometry_msgs.msg import PoseStamped
from inverse_msgs.msg import TargetPoseArray
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


def make_targets(tcp_pose, spacing):
    """Offsets are in the incoming reference frame; tool orientation is preserved."""
    offsets = {
        'home': (0.0, 0.0, 0.0),
        'pick': (0.0, 0.0, 0.0),
        'pick_approach': (0.0, 0.0, spacing),
        'place': (spacing, 0.0, 0.0),
        'place_approach': (spacing, 0.0, spacing),
    }
    targets = TargetPoseArray()
    for name, (dx, dy, dz) in offsets.items():
        pose = deepcopy(tcp_pose)
        pose.pose.position.x += dx
        pose.pose.position.y += dy
        pose.pose.position.z += dz
        targets.names.append(name)
        targets.poses.append(pose)

        # print(pose)
    return targets


class HardwareTargetPublisher(Node):
    def __init__(self):
        super().__init__('hardware_target_publisher')
        self.spacing = self.declare_parameter(
            'spacing', 0.05, ParameterDescriptor(read_only=True)).value
        if not math.isfinite(self.spacing) or self.spacing <= 0.0:
            raise ValueError('spacing must be a positive, finite distance in metres')

        self.targets = None
        self.publisher = self.create_publisher(TargetPoseArray, '/target_poses', 1)
        self.subscription = self.create_subscription(
            PoseStamped, '/tcp_pose_broadcaster/pose', self.capture_pose,
            qos_profile_sensor_data)
        self.timer = self.create_timer(0.1, self.publish_targets)
        self.get_logger().info('Waiting for the current TCP pose on /tcp_pose_broadcaster/pose')

    def capture_pose(self, message):
        if self.targets is not None:
            return  # Targets stay fixed after capture, even when the TCP moves.

        position = message.pose.position
        # print(position)
        orientation = message.pose.orientation
        values = (position.x, position.y, position.z,
                  orientation.x, orientation.y, orientation.z, orientation.w)
        if not message.header.frame_id or not all(math.isfinite(value) for value in values):
            self.get_logger().warning('Ignoring TCP pose with a missing frame or nonfinite values',
                                      throttle_duration_sec=5.0)
            return
        if abs(sum(value * value for value in values[3:]) - 1.0) > 0.01:
            self.get_logger().warning('Ignoring TCP pose with a non-unit quaternion',
                                      throttle_duration_sec=5.0)
            return

        # Preserve the actual frame (normally "base"), never relabel it "base_link".
        self.targets = make_targets(message, self.spacing)
        self.get_logger().info(
            f'Captured TCP in {message.header.frame_id}: '
            f'x={position.x:.4f}, y={position.y:.4f}, z={position.z:.4f}. '
            f'Publishing fixed targets with {self.spacing:.3f} m +X/+Z spacing.')
        self.publish_targets()

    def publish_targets(self):
        if self.targets is None:
            return
        message = deepcopy(self.targets)
        stamp = self.get_clock().now().to_msg()
        for pose in message.poses:
            pose.header.stamp = stamp
        self.publisher.publish(message)


def main():
    rclpy.init()
    node = None
    try:
        node = HardwareTargetPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
