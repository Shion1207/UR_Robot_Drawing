#!/usr/bin/env python3
"""Wall-time ROS safety adapter; no Nav2, never commands a physical robot by default."""
import math
import signal
import time
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener, TransformException
from emotion_logic import DirectMotionController, scan_clearances


class EmotionMotionNode(Node):
    def __init__(self):
        super().__init__('emotion_motion_node')
        defaults=dict(linear_speed=.12,angular_speed=.45,step_distance=.15,arc_distance=.35,
            arc_radius=.60,emotion_timeout=1.5,sensor_timeout=2.5,stop_distance=.50,
            rotation_clearance=.35,progress_timeout=10.,phase_timeout=180.,rearm_seconds=.4)
        for key,value in defaults.items():self.declare_parameter(key,value)
        self.controller=DirectMotionController(**{key:float(self.get_parameter(key).value) for key in defaults})
        self.tf=Buffer();self.listener=TransformListener(self.tf,self)
        self.velocity=self.create_publisher(Twist,'/cmd_vel',1)
        self.status=self.create_publisher(String,'/emotion/status',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(String,'/detected_emotion',lambda m:self.on_emotion(m),1)
        self.create_subscription(Odometry,'/odom',self.on_odom,qos_profile_sensor_data)
        self.create_subscription(LaserScan,'/scan',self.on_scan,qos_profile_sensor_data)
        self.create_service(Trigger,'/emotion/stop',self.on_stop)
        self.last_odom_stamp=self.last_scan_stamp=None
        self.last_status=''
        # use_sim_time must remain false: camera/safety stop during paused simulation.
        self.create_timer(.05,self.on_tick)
        self.get_logger().info('Direct control: show an expression; Neutral/no face stops. No Nav2/teleop.')

    def on_emotion(self,msg):
        self.controller.emotion(msg.data,time.monotonic())
        if not self.controller.active:self.velocity.publish(Twist())

    def on_odom(self,msg):
        stamp=(msg.header.stamp.sec,msg.header.stamp.nanosec)
        if stamp==self.last_odom_stamp:return
        self.last_odom_stamp=stamp
        p,q=msg.pose.pose.position,msg.pose.pose.orientation
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        self.controller.pose(p.x,p.y,yaw,time.monotonic())

    def on_scan(self,msg):
        stamp=(msg.header.stamp.sec,msg.header.stamp.nanosec)
        if stamp==self.last_scan_stamp:return
        self.last_scan_stamp=stamp
        yaw_offset,sensor_offset=0.,0.
        if msg.header.frame_id!='base_link':
            try:
                transform=self.tf.lookup_transform('base_link',msg.header.frame_id,rclpy.time.Time()).transform
                q=transform.rotation
                yaw_offset=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
                sensor_offset=math.hypot(transform.translation.x,transform.translation.y)
            except TransformException:
                self.controller.scan(0.,0.,0.,time.monotonic())
                self.controller.stop('missing scan -> base_link transform')
                return
        self.controller.scan(*scan_clearances(msg.ranges,msg.angle_min,msg.angle_increment,
            msg.range_min,msg.range_max,yaw_offset,sensor_offset),time.monotonic())

    def on_stop(self,request,response):
        self.controller.stop('manual stop');self.velocity.publish(Twist())
        response.success=True;response.message='Stopped. Change expression or show Neutral before restarting.'
        return response

    def on_tick(self):
        # Create 3 exposes its Gazebo telemetry as a ROS publisher on this topic.
        # Do not mistake that required bridge for a second external controller.
        other=[p for p in self.get_publishers_info_by_topic('/cmd_vel')
               if not (p.node_name==self.get_name() and p.node_namespace==self.get_namespace())
               and not (p.node_name=='cmd_vel_bridge' and p.node_namespace==self.get_namespace())]
        if other:
            self.controller.stop('another /cmd_vel publisher is active')
        v,w=self.controller.tick(time.monotonic())
        msg=Twist();msg.linear.x=float(v);msg.angular.z=float(w)
        self.velocity.publish(msg)
        self.status.publish(String(data=self.controller.status))
        if self.controller.status!=self.last_status:
            self.get_logger().info(self.controller.status);self.last_status=self.controller.status

    def destroy_node(self):
        if rclpy.ok():
            for _ in range(3):self.velocity.publish(Twist())
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args);node=None
    try:
        node=EmotionMotionNode();rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):pass
    finally:
        signal.signal(signal.SIGINT,signal.SIG_IGN)
        if node:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
