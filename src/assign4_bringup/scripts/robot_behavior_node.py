#!/usr/bin/env python3
"""ROS adapters for confirmed, finite hand-gesture behaviors."""
import math
import signal
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from gesture_logic import BehaviorController, scan_clearances


class RobotBehaviorNode(Node):
    def __init__(self):
        super().__init__('robot_behavior_node')
        defaults = dict(linear_speed=0.15, angular_speed=0.55, step_distance=0.10,
                        circle_radius=0.40, gesture_timeout=0.9, sensor_timeout=3.0,
                        selection_timeout=8.0, phase_timeout=240.0, progress_timeout=10.0,
                        stop_distance=0.50, rotation_clearance=0.30)
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.controller = BehaviorController(**{
            key: float(self.get_parameter(key).value) for key in defaults})
        self.velocity = self.create_publisher(Twist, '/cmd_vel', 1)
        status_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status = self.create_publisher(String, '/gesture_behavior/status', status_qos)
        self.create_subscription(String, '/detected_gesture', self.on_gesture, 1)
        self.create_subscription(Odometry, '/odom', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.last_status = ''
        # Deliberately wall-time: stopping must work when Gazebo /clock is paused.
        self.timer = self.create_timer(0.05, self.on_tick)
        self.get_logger().info('Chờ cử chỉ chọn và Thumb_Up xác nhận. Không chạy Nav2/teleop cùng lúc.')

    def on_gesture(self, msg):
        self.controller.gesture(msg.data, time.monotonic())
        if msg.data in ('Open_Palm', 'Closed_Fist'):
            self.velocity.publish(Twist())

    def on_odom(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        self.controller.update_pose(p.x, p.y, yaw, time.monotonic())

    def on_scan(self, msg):
        clearance = scan_clearances(msg.ranges, msg.angle_min, msg.angle_increment,
                                    msg.range_min, msg.range_max)
        self.controller.update_scan(*clearance, time.monotonic())

    def on_tick(self):
        v, w = self.controller.tick(time.monotonic())
        msg = Twist()
        msg.linear.x, msg.angular.z = float(v), float(w)
        self.velocity.publish(msg)
        status = self.controller.status
        self.status.publish(String(data=status))
        if status != self.last_status:
            self.get_logger().info(status)
            self.last_status = status

    def destroy_node(self):
        if rclpy.ok():
            for _ in range(3):
                self.velocity.publish(Twist())
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = RobotBehaviorNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # ros2 launch can forward SIGINT after the terminal already delivered it.
        # Ignore that second signal while ROS entities are being destroyed.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
