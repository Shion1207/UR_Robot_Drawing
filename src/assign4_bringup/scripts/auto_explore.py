#!/usr/bin/env python3
"""Drive a deterministic safe route through the compact assignment world."""

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def clamp(value, lower, upper):
    return max(lower, min(value, upper))


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class AutoExplore(Node):
    def __init__(self):
        super().__init__('assign4_auto_explore')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.create_subscription(
            LaserScan, 'scan', self.scan_callback, qos_profile_sensor_data
        )
        self.timer = self.create_timer(0.05, self.control)

        self.pose = None
        self.scan = None
        self.origin = None
        self.index = 0
        self.started = time.monotonic()
        self.last_progress = self.started
        self.last_distance = float('inf')

        # Safe route around the outer wall and both sides of the obstacles.
        self.waypoints = [
            (0.50, -2.20),
            (3.20, -2.20),
            (3.20, 2.35),
            (0.20, 2.35),
            (-1.65, 2.35),
            (-3.30, 2.35),
            (-3.30, 0.00),
            (-3.30, -2.20),
            (0.50, -2.20),
            (0.50, 0.00),
        ]

    def odom_callback(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y ** 2 + orientation.z ** 2),
        )
        self.pose = (position.x, position.y, yaw)
        if self.origin is None:
            self.origin = self.pose
            self.get_logger().info('Odometry ready; starting compact-world route.')

    def scan_callback(self, msg):
        self.scan = msg

    def local_to_odom(self, point):
        ox, oy, oyaw = self.origin
        px, py = point
        return (
            ox + math.cos(oyaw) * px - math.sin(oyaw) * py,
            oy + math.sin(oyaw) * px + math.cos(oyaw) * py,
        )

    def sector_min(self, start_fraction, end_fraction):
        if self.scan is None or not self.scan.ranges:
            return float('inf')
        count = len(self.scan.ranges)
        values = self.scan.ranges[
            int(count * start_fraction):int(count * end_fraction)
        ]
        valid = [value for value in values if math.isfinite(value)]
        return min(valid, default=float('inf'))

    def stop(self):
        self.publisher.publish(Twist())

    def control(self):
        if self.pose is None or self.scan is None:
            return
        if time.monotonic() - self.started > 210.0:
            self.get_logger().error('Automatic route timed out.')
            self.stop()
            rclpy.shutdown()
            return
        if self.index >= len(self.waypoints):
            self.stop()
            self.get_logger().info('Automatic mapping route completed.')
            rclpy.shutdown()
            return

        target_x, target_y = self.local_to_odom(self.waypoints[self.index])
        x, y, yaw = self.pose
        dx = target_x - x
        dy = target_y - y
        distance = math.hypot(dx, dy)

        if distance < 0.18:
            self.get_logger().info(
                f'Waypoint {self.index + 1}/{len(self.waypoints)} reached.'
            )
            self.index += 1
            self.last_distance = float('inf')
            self.last_progress = time.monotonic()
            self.stop()
            return

        if distance < self.last_distance - 0.08:
            self.last_distance = distance
            self.last_progress = time.monotonic()
        elif time.monotonic() - self.last_progress > 12.0:
            self.get_logger().warning('Skipping a waypoint that cannot be reached.')
            self.index += 1
            self.last_distance = float('inf')
            self.last_progress = time.monotonic()
            self.stop()
            return

        target_heading = math.atan2(dy, dx)
        heading_error = normalize_angle(target_heading - yaw)
        command = Twist()

        # The scan is ordered from -pi to +pi: front is near both ends.
        front = min(self.sector_min(0.0, 0.07), self.sector_min(0.93, 1.0))
        left = self.sector_min(0.55, 0.75)
        right = self.sector_min(0.25, 0.45)
        if front < 0.42:
            command.angular.z = 0.85 if left >= right else -0.85
        elif abs(heading_error) > 0.32:
            command.angular.z = clamp(1.7 * heading_error, -0.95, 0.95)
        else:
            command.linear.x = min(0.38, 0.18 + 0.12 * distance)
            command.angular.z = clamp(1.5 * heading_error, -0.55, 0.55)

        self.publisher.publish(command)


def main():
    rclpy.init()
    node = AutoExplore()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
