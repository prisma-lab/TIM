#!/usr/bin/env python3
"""Publish two explicitly configured physical TCP poses; no simulated defaults."""
import math
import rclpy
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.msg import ParameterDescriptor


def main():
    rclpy.init()
    node = rclpy.create_node('hardware_targets')
    try:
        fixed = ParameterDescriptor(read_only=True)
        frame = node.declare_parameter('frame', 'base_link', fixed).value
        if not frame:
            raise ValueError('frame cannot be empty')
        targets = {}
        for name in ('pick', 'place'):
            pose = node.declare_parameter(name + '_pose', [float('nan')] * 7, fixed).value
            if len(pose) != 7 or not all(math.isfinite(v) for v in pose):
                raise ValueError(f'Set {name}_pose to seven measured values: x y z qx qy qz qw')
            if abs(sum(v*v for v in pose[3:]) - 1) > .01:
                raise ValueError(f'{name}_pose needs a unit quaternion')
            targets[name] = (pose, node.create_publisher(PoseStamped, '/ur10/targets/' + name, 10))
        def publish():
            for pose, publisher in targets.values():
                msg = PoseStamped()
                msg.header.frame_id = frame
                msg.header.stamp = node.get_clock().now().to_msg()
                msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = pose[:3]
                q = msg.pose.orientation
                q.x, q.y, q.z, q.w = pose[3:]
                publisher.publish(msg)
        node.create_timer(.1, publish)
        node.get_logger().info('Publishing the configured physical grasp_tcp targets; no motion requested')
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as error:
        node.get_logger().error(str(error))
        return 1
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
