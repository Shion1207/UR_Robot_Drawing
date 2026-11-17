"""ROS adapters exercised with real topics in isolated domain 189; no robot."""
from pathlib import Path
import sys
import threading
import time
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src/assign8_bringup/scripts'))
try:
    import rclpy
except ImportError:
    rclpy=None
if rclpy:
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String
    from emotion_motion_node import EmotionMotionNode


@unittest.skipIf(rclpy is None,'source setup_local.bash to test ROS adapters')
class RosTests(unittest.TestCase):
    def test_neutral_arms_expression_moves_and_loss_stops_real_cmd_vel(self):
        rclpy.init(domain_id=189)
        node=EmotionMotionNode(); harness=Node('emotion_test_harness')
        bridge=Node('cmd_vel_bridge')
        # The real simulator bridge permanently offers this telemetry publisher.
        bridge.create_publisher(Twist,'/cmd_vel',1)
        received=[]
        harness.create_subscription(Twist,'/cmd_vel',lambda m:received.append((m.linear.x,m.angular.z)),10)
        expression=harness.create_publisher(String,'/detected_emotion',1)
        odom=harness.create_publisher(Odometry,'/odom',10)
        scan=harness.create_publisher(LaserScan,'/scan',10)
        label=['neutral']; sequence=[0]
        def feed():
            sequence[0]+=1
            o=Odometry();o.header.stamp.sec=sequence[0];o.pose.pose.orientation.w=1.
            s=LaserScan();s.header.stamp.sec=sequence[0];s.header.frame_id='base_link';s.angle_min=-3.1415927;s.angle_increment=.017453293
            s.range_min=.1;s.range_max=8.;s.ranges=[5.]*360
            odom.publish(o);scan.publish(s);expression.publish(String(data=label[0]))
        harness.create_timer(.05,feed)
        executor=MultiThreadedExecutor(num_threads=2);executor.add_node(node);executor.add_node(harness)
        thread=threading.Thread(target=executor.spin,daemon=True);thread.start()
        def wait(test):
            until=time.monotonic()+5
            while time.monotonic()<until and not test():time.sleep(.02)
            self.assertTrue(test(),(node.controller.status,received[-5:]))
        try:
            wait(lambda:expression.get_subscription_count()>0 and len(received)>0)
            label[0]='happy';wait(lambda:any(v>0 and w>0 for v,w in received))
            label[0]='none';wait(lambda:node.controller.active is None and received[-1]==(0.,0.))
            received.clear();label[0]='happy';time.sleep(.25)
            self.assertFalse(any(v or w for v,w in received),'Loss must latch stop until neutral')
        finally:
            executor.shutdown();thread.join(timeout=2)
            node.destroy_node();harness.destroy_node();bridge.destroy_node();rclpy.shutdown()


if __name__=='__main__':unittest.main()
