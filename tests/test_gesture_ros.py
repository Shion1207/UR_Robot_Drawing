"""Real ROS 2 publisher/subscriber integration in an isolated domain, no robot."""
import math
import os
from pathlib import Path
import sys
import tempfile
import time
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/assign4_bringup/scripts'))
try:
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String
    from robot_behavior_node import RobotBehaviorNode
except ImportError:
    rclpy = None


@unittest.skipIf(rclpy is None, 'source setup_local.bash to run the ROS integration test')
class RosGestureTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.logs = tempfile.TemporaryDirectory(prefix='gesture-ros-test-')
        cls.previous_logs = os.environ.get('ROS_LOG_DIR')
        os.environ['ROS_LOG_DIR'] = cls.logs.name

    @classmethod
    def tearDownClass(cls):
        if cls.previous_logs is None:
            os.environ.pop('ROS_LOG_DIR', None)
        else:
            os.environ['ROS_LOG_DIR'] = cls.previous_logs
        cls.logs.cleanup()

    def test_confirm_motion_palm_stop_and_missing_hand_watchdog(self):
        # A separate DDS domain prevents test velocity from reaching a running simulator.
        rclpy.init(domain_id=163)
        robot = RobotBehaviorNode()
        harness = Node('gesture_test_harness')
        executor = SingleThreadedExecutor()
        executor.add_node(robot)
        executor.add_node(harness)
        gestures = harness.create_publisher(String, '/detected_gesture', 1)
        scans = harness.create_publisher(LaserScan, '/scan', 1)
        odom = harness.create_publisher(Odometry, '/odom', 1)
        velocities = []
        harness.create_subscription(Twist, '/cmd_vel', lambda msg: velocities.append(msg.linear.x), 10)
        scan = LaserScan()
        scan.angle_min, scan.angle_increment = -math.pi, math.pi/180
        scan.range_min, scan.range_max = 0.1, 10.0
        scan.ranges = [5.0]*360
        pose = Odometry()
        pose.pose.pose.orientation.w = 1.0

        def pump(label, duration):
            deadline = time.monotonic()+duration
            while time.monotonic() < deadline:
                if label is not None:
                    gestures.publish(String(data=label))
                scans.publish(scan)
                odom.publish(pose)
                executor.spin_once(timeout_sec=0.015)

        try:
            pump('None', .4)
            velocities.clear()
            pump('Pointing_Up', .25)
            self.assertTrue(velocities)
            self.assertTrue(all(v == 0 for v in velocities))
            velocities.clear()
            pump('Thumb_Up', .3)
            self.assertTrue(any(v > 0 for v in velocities))
            pump('Open_Palm', .15)
            self.assertEqual(velocities[-1], 0.0)
            pump('Pointing_Up', .15)
            pump('Thumb_Up', .2)
            self.assertGreater(velocities[-1], 0)
            pump(None, 1.2)
            self.assertEqual(velocities[-1], 0.0)
            self.assertIsNone(robot.controller.active)
        finally:
            robot.destroy_node()
            harness.destroy_node()
            executor.shutdown()
            rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
