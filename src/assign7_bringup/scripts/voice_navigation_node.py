#!/usr/bin/env python3
"""Resolve complete voice routes, then visit every named area via Nav2 actions."""
import math
from pathlib import Path
import signal
import time

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from semantic_map import load_regions
from voice_commands import CommandError, normalize, parse_command


class VoiceNavigation(Node):
    def __init__(self):
        super().__init__('voice_navigation_node')
        share = Path(get_package_share_directory('assign7_bringup'))
        self.declare_parameter('regions_file', str(share/'semantic/regions.json'))
        self.declare_parameter('map_yaml', str(share/'maps/map.yaml'))
        self.declare_parameter('require_localization', True)
        data, _ = load_regions(self.get_parameter('regions_file').value, self.get_parameter('map_yaml').value)
        self.regions = data['regions']
        self.by_id = {r['id']: r for r in self.regions}
        self.client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.publisher = self.create_publisher(String, '/voice/navigation_status', 10)
        self.create_subscription(String, '/voice/text', self.on_command, 10)
        self.state, self.goal_handle = 'IDLE', None
        self.route, self.index = [], 0
        self.cancel_requested = False
        self.last_text, self.last_text_at = '', -math.inf
        self.last_feedback = 0.0
        self.create_timer(0.25, self.on_tick)
        self.status('Sẵn sàng. Nói: Đi đến phòng học; hoặc Đi từ khu vực A đến khu vực B.')

    def status(self, text):
        self.get_logger().info(text)
        self.publisher.publish(String(data=text))

    def localized(self):
        return (not self.get_parameter('require_localization').value or
                self.tf.can_transform('map', 'base_link', rclpy.time.Time()))

    def on_command(self, msg):
        try:
            route = parse_command(msg.data, self.regions)
        except CommandError as exc:
            self.status(f'Từ chối: {exc}')
            return
        if not route:
            self.cancel_requested = True
            if self.goal_handle:
                self.state = 'CANCELLING'
                self.goal_handle.cancel_goal_async().add_done_callback(self.on_cancel_response)
                self.status('Đã yêu cầu Nav2 dừng; đang chờ xác nhận hủy.')
            elif self.state in ('SENDING', 'CANCELLING'):
                self.state = 'CANCELLING'
                self.status('Đang gửi goal; sẽ hủy ngay khi Nav2 phản hồi.')
            else:
                self.state, self.route = 'IDLE', []
                self.status('Đã dừng chuỗi điều hướng.')
            return
        if self.state != 'IDLE':
            self.status('Robot đang thực hiện lệnh trước. Nói Dừng lại để hủy; chưa nhận lệnh mới.')
            return
        now, text = time.monotonic(), normalize(msg.data)
        if text == self.last_text and now-self.last_text_at < 2.0:
            return
        if not self.client.server_is_ready():
            self.status('Nav2 chưa sẵn sàng. Đợi navigation khởi động rồi nói lại.')
            return
        if not self.localized():
            self.status('Chưa có TF map -> base_link. Đặt 2D Pose Estimate đúng vị trí/hướng rồi nói lại.')
            return
        self.last_text, self.last_text_at = text, now
        self.route, self.index = route, 0
        self.cancel_requested, self.state = False, 'WAIT_NEXT'
        self.status('Đã hiểu lộ trình: '+' -> '.join(f"{i} ({self.by_id[i]['name']})" for i in route))

    def on_tick(self):
        if self.state != 'WAIT_NEXT':
            return
        if not self.localized():
            self.state = 'IDLE'
            self.status('Dừng chuỗi: mất localization trước goal tiếp theo.')
            return
        region = self.by_id[self.route[self.index]]
        x, y, yaw = region['goal']
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = float(x), float(y)
        goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
        self.state = 'SENDING'
        self.status(f"Gửi goal {self.index+1}/{len(self.route)}: {region['id']} — {region['name']} ({x:.2f}, {y:.2f})")
        self.client.send_goal_async(goal, feedback_callback=self.on_feedback).add_done_callback(self.on_response)

    def on_response(self, future):
        try:
            handle = future.result()
            if not handle.accepted:
                self.state = 'IDLE'
                self.status('Nav2 từ chối goal. Dừng chuỗi, không gửi đích kế tiếp.')
                return
            self.goal_handle = handle
            handle.get_result_async().add_done_callback(self.on_result)
            if self.cancel_requested:
                self.state = 'CANCELLING'
                handle.cancel_goal_async().add_done_callback(self.on_cancel_response)
            else:
                self.state = 'RUNNING'
        except Exception as exc:
            self.state, self.goal_handle = 'IDLE', None
            self.status(f'Lỗi gửi goal: {exc}')

    def on_cancel_response(self, future):
        try:
            if not future.result().goals_canceling:
                self.status('Nav2 chưa xác nhận hủy. Theo dõi robot; có thể nói Dừng lại lần nữa.')
        except Exception as exc:
            self.status(f'Lỗi yêu cầu hủy: {exc}')

    def on_feedback(self, msg):
        if time.monotonic()-self.last_feedback > 3:
            self.last_feedback = time.monotonic()
            self.status(f'Còn {msg.feedback.distance_remaining:.2f} m tới đích hiện tại.')

    def on_result(self, future):
        self.goal_handle = None
        try:
            status = future.result().status
        except Exception as exc:
            self.state = 'IDLE'
            self.status(f'Lỗi nhận kết quả: {exc}')
            return
        if self.cancel_requested or status != GoalStatus.STATUS_SUCCEEDED:
            self.state = 'IDLE'
            self.status(f'Kết thúc/hủy chuỗi, trạng thái Nav2={status}. Không tự gửi goal tiếp theo.')
            return
        self.status(f'Đã đến khu vực {self.route[self.index]}.')
        self.index += 1
        if self.index == len(self.route):
            self.state = 'IDLE'
            self.status('HOÀN THÀNH lộ trình: '+' -> '.join(self.route))
        else:
            self.state = 'WAIT_NEXT'


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = VoiceNavigation()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if node:
            if node.goal_handle and rclpy.ok():
                future = node.goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(node, future, timeout_sec=1.0)
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
