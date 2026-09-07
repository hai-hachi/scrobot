#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import Pose, PoseArray
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


def find_minimum_grid(court_length, court_width, effective_range, max_grid_size=20):
    best = None

    for nx in range(1, max_grid_size + 1):
        for ny in range(1, max_grid_size + 1):
            dx = court_length / nx
            dy = court_width / ny
            max_distance = 0.5 * math.hypot(dx, dy)

            if max_distance > effective_range:
                continue

            candidate = (nx * ny, max_distance, nx, ny, dx, dy)

            if best is None or candidate[:2] < best[:2]:
                best = candidate

    if best is None:
        raise RuntimeError('Could not generate patrol grid.')

    _, max_distance, nx, ny, dx, dy = best
    return nx, ny, dx, dy, max_distance


def generate_patrol_points(court_length, court_width, nx, ny):
    dx = court_length / nx
    dy = court_width / ny
    x_min = -court_length / 2.0
    y_min = -court_width / 2.0

    points = []

    for row in range(ny):
        y = y_min + dy / 2.0 + row * dy
        xs = [x_min + dx / 2.0 + col * dx for col in range(nx)]

        if row % 2 == 1:
            xs.reverse()

        for x in xs:
            yaw = 0.0 if row % 2 == 0 else math.pi
            points.append((x, y, yaw))

    return points


class PatrolPointsNode(Node):
    def __init__(self):
        super().__init__('patrol_points')

        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('court_length', 13.40)
        self.declare_parameter('court_width', 6.10)
        self.declare_parameter('camera_range', 3.0)
        self.declare_parameter('range_factor', 0.90)

        self.frame_id = str(self.get_parameter('frame_id').value)
        court_length = float(self.get_parameter('court_length').value)
        court_width = float(self.get_parameter('court_width').value)
        camera_range = float(self.get_parameter('camera_range').value)
        range_factor = float(self.get_parameter('range_factor').value)

        effective_range = camera_range * range_factor
        nx, ny, dx, dy, max_distance = find_minimum_grid(court_length, court_width, effective_range)
        self.points = generate_patrol_points(court_length, court_width, nx, ny)

        self.get_logger().info(f'Court: {court_length:.2f} x {court_width:.2f} m')
        self.get_logger().info(f'Camera range: {camera_range:.2f} m')
        self.get_logger().info(f'Effective range: {effective_range:.2f} m')
        self.get_logger().info(f'Grid: {nx} x {ny}, points: {len(self.points)}')
        self.get_logger().info(f'Cell size: {dx:.2f} x {dy:.2f} m')
        self.get_logger().info(f'Worst-case distance: {max_distance:.2f} m')

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(PoseArray, '/mission/patrol_points', qos)

        self.publish_points()

    def publish_points(self):
        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        for x, y, yaw in self.points:
            pose = Pose()
            pose.position.x = float(x)
            pose.position.y = float(y)
            pose.orientation.z = math.sin(yaw / 2.0)
            pose.orientation.w = math.cos(yaw / 2.0)
            msg.poses.append(pose)

        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PatrolPointsNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
