#!/usr/bin/env python3
"""Install the measured workcell collision geometry before enabling primitives."""
import math
import sys
import time
import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import CollisionObject
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs  # Registers PoseStamped transforms.
from ur10_hardware_primitives.configuration import load_cell


def initial_workpiece(node, cell):
    """Model the initial object before the first transfer, using its pick TCP pose."""
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    latest = []
    def received(msg):
        latest[:] = [msg, time.monotonic()]
    subscription = node.create_subscription(PoseStamped, '/ur10/targets/pick', received, 10)
    deadline = time.monotonic() + 60
    node.get_logger().info('Waiting for /ur10/targets/pick to model the initial workpiece')
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
            if not latest or time.monotonic() - latest[1] > 2:
                continue
            msg = latest[0]
            age = (node.get_clock().now() - rclpy.time.Time.from_msg(msg.header.stamp)).nanoseconds / 1e9
            if not msg.header.frame_id or not -.1 <= age <= 2 or (msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0):
                continue
            try:
                target = msg if msg.header.frame_id == 'base_link' else buffer.transform(msg, 'base_link')
            except Exception:
                continue
            pose = target.pose
            p, q = pose.position, pose.orientation
            if not all(math.isfinite(v) for v in (p.x, p.y, p.z, q.x, q.y, q.z, q.w)):
                continue
            if abs(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w - 1) > .01:
                continue
            # Compose grasp_tcp -> box with the measured base_link -> grasp_tcp.
            x, y, z = map(float, cell['object_offset_xyz'])
            tx, ty, tz = 2*(q.y*z-q.z*y), 2*(q.z*x-q.x*z), 2*(q.x*y-q.y*x)
            p.x += x + q.w*tx + q.y*tz - q.z*ty
            p.y += y + q.w*ty + q.z*tx - q.x*tz
            p.z += z + q.w*tz + q.x*ty - q.y*tx
            r, pitch, yaw = (float(v)/2 for v in cell['object_offset_rpy'])
            cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
            bx, by, bz, bw = sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy, cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy
            q.x, q.y, q.z, q.w = (q.w*bx+q.x*bw+q.y*bz-q.z*by,
                q.w*by-q.x*bz+q.y*bw+q.z*bx, q.w*bz+q.x*by-q.y*bx+q.z*bw,
                q.w*bw-q.x*bx-q.y*by-q.z*bz)
            obj = CollisionObject(id='workpiece', operation=CollisionObject.ADD)
            obj.header.frame_id = 'base_link'
            obj.primitives = [SolidPrimitive(type=SolidPrimitive.BOX,
                                            dimensions=list(map(float, cell['object_dimensions'])))]
            obj.primitive_poses = [pose]
            return obj
        raise RuntimeError('No valid pick target for initial workpiece; publish measured targets before starting primitives')
    finally:
        node.destroy_subscription(subscription)
        listener.unregister()


def main():
    rclpy.init()
    node = rclpy.create_node('hardware_workcell_scene')
    try:
        cell = load_cell(node.declare_parameter('cell_config', '').value)
        client = node.create_client(ApplyPlanningScene, '/apply_planning_scene')
        if not client.wait_for_service(timeout_sec=60):
            raise RuntimeError('MoveIt planning scene service unavailable')
        request = ApplyPlanningScene.Request()
        request.scene.is_diff = True
        request.scene.robot_state.is_diff = True
        for obstacle in cell['obstacles']:
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = map(float, obstacle['xyz'])
            r, p, y = (float(v) / 2 for v in obstacle['rpy'])
            cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
            pose.orientation.x = sr*cp*cy - cr*sp*sy
            pose.orientation.y = cr*sp*cy + sr*cp*sy
            pose.orientation.z = cr*cp*sy - sr*sp*cy
            pose.orientation.w = cr*cp*cy + sr*sp*sy
            obj = CollisionObject(id=obstacle['id'], operation=CollisionObject.ADD)
            obj.header.frame_id = 'base_link'
            obj.primitives = [SolidPrimitive(type=SolidPrimitive.BOX,
                                            dimensions=list(map(float, obstacle['dimensions'])))]
            obj.primitive_poses = [pose]
            request.scene.world.collision_objects.append(obj)
        request.scene.world.collision_objects.append(initial_workpiece(node, cell))
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=15)
        if not future.done() or not future.result().success:
            raise RuntimeError('MoveIt did not acknowledge the workcell collision geometry')
        node.get_logger().info('Measured workcell geometry installed')
        return 0
    except Exception as error:
        node.get_logger().error(str(error))
        return 1
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main())
