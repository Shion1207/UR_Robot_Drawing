"""Real ROS action protocol tests on an isolated DDS domain, no robot attached."""
import os
from pathlib import Path
import sys
import tempfile
import threading
import time
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src/assign7_bringup/scripts'))
try:
    import rclpy
    from rclpy.action import ActionServer, CancelResponse, GoalResponse
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from nav2_msgs.action import NavigateToPose
    from std_msgs.msg import String
    from voice_navigation_node import VoiceNavigation
except ImportError:
    rclpy = None


@unittest.skipIf(rclpy is None, 'source setup_local.bash before ROS integration tests')
class VoiceActionTests(unittest.TestCase):
    def test_order_unknown_and_cancellation_during_goal_acceptance(self):
        with tempfile.TemporaryDirectory(prefix='assign7-ros-') as logs:
            old_logs = os.environ.get('ROS_LOG_DIR')
            os.environ['ROS_LOG_DIR'] = logs
            rclpy.init(domain_id=176)
            node = VoiceNavigation()
            node.set_parameters([Parameter('require_localization', value=False)])
            harness = Node('assign7_action_harness')
            received, statuses = [], []
            delay_acceptance = threading.Event()
            def accept(request):
                if delay_acceptance.is_set():
                    time.sleep(.4)
                return GoalResponse.ACCEPT
            def execute(handle):
                received.append((round(handle.request.pose.pose.position.x,3),
                                 round(handle.request.pose.pose.position.y,3)))
                end = time.monotonic()+.4
                while time.monotonic() < end:
                    if handle.is_cancel_requested:
                        handle.canceled()
                        return NavigateToPose.Result()
                    time.sleep(.01)
                handle.succeed()
                return NavigateToPose.Result()
            server = ActionServer(harness, NavigateToPose, '/navigate_to_pose', execute,
                goal_callback=accept, cancel_callback=lambda _: CancelResponse.ACCEPT,
                callback_group=ReentrantCallbackGroup())
            publisher = harness.create_publisher(String, '/voice/text', 10)
            harness.create_subscription(String, '/voice/navigation_status', lambda m: statuses.append(m.data), 10)
            executor = MultiThreadedExecutor(num_threads=4)
            executor.add_node(node)
            executor.add_node(harness)
            thread = threading.Thread(target=executor.spin, daemon=True)
            thread.start()
            def wait_for(predicate, seconds=5):
                until = time.monotonic()+seconds
                while not predicate() and time.monotonic()<until:
                    time.sleep(.02)
                self.assertTrue(predicate(), (node.state, statuses[-8:], received))
            try:
                wait_for(lambda: node.client.server_is_ready() and publisher.get_subscription_count()>0)
                publisher.publish(String(data='Đi từ khu vực A đến khu vực B'))
                wait_for(lambda: len(received)==2 and node.state=='IDLE')
                self.assertEqual(received, [(-1.175,1.415), (2.725,1.915)])
                publisher.publish(String(data='Đi từ khu vực D đến khu vực B'))
                wait_for(lambda: any('không tồn tại' in s for s in statuses))
                self.assertEqual(len(received), 2)
                node.set_parameters([Parameter('require_localization', value=True)])
                publisher.publish(String(data='Đi đến hành lang'))
                wait_for(lambda: any('Chưa có TF' in s for s in statuses))
                self.assertEqual(len(received), 2)
                node.set_parameters([Parameter('require_localization', value=False)])
                delay_acceptance.set()
                publisher.publish(String(data='Đi từ hành lang đến phòng học'))
                wait_for(lambda: node.state=='SENDING')
                publisher.publish(String(data='Dừng lại'))
                wait_for(lambda: node.state=='CANCELLING')
                publisher.publish(String(data='Dừng lại'))
                time.sleep(.04)
                self.assertEqual(node.state, 'CANCELLING')
                wait_for(lambda: node.state=='IDLE')
                self.assertEqual(len(received), 3, 'Stop must prevent the second route goal')
            finally:
                # Let outstanding cancel/result callbacks finish before destroying clients.
                time.sleep(.3)
                executor.shutdown()
                thread.join(timeout=2)
                server.destroy()
                node.destroy_node()
                harness.destroy_node()
                rclpy.shutdown()
                if old_logs is None:
                    os.environ.pop('ROS_LOG_DIR', None)
                else:
                    os.environ['ROS_LOG_DIR'] = old_logs


if __name__=='__main__':
    unittest.main()
