#!/usr/bin/env python3
"""Send a configurable sequence of NavigateToPose goals to Nav2."""

import math
from typing import List, Optional, Tuple

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray


class WaypointNavigator(Node):
    """Wait for AMCL, then visit all configured map-frame waypoints in order."""

    def __init__(self) -> None:
        super().__init__('waypoint_navigator')

        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('action_name', 'navigate_to_pose')
        self.declare_parameter('amcl_pose_topic', 'amcl_pose')
        self.declare_parameter('goals', [1.0, 0.0, 0.0])
        self.declare_parameter('goal_names', ['Goal 1'])
        self.declare_parameter('wait_for_initial_pose', True)
        self.declare_parameter('initial_delay', 3.0)
        self.declare_parameter('wait_between_goals', 2.0)
        self.declare_parameter('server_timeout', 90.0)
        self.declare_parameter('max_retries', 1)
        self.declare_parameter('continue_on_failure', False)

        self.frame_id = str(self.get_parameter('frame_id').value)
        self.wait_for_initial_pose = bool(
            self.get_parameter('wait_for_initial_pose').value)
        self.initial_delay = float(self.get_parameter('initial_delay').value)
        self.wait_between_goals = float(
            self.get_parameter('wait_between_goals').value)
        self.server_timeout = float(self.get_parameter('server_timeout').value)
        self.max_retries = int(self.get_parameter('max_retries').value)
        self.continue_on_failure = bool(
            self.get_parameter('continue_on_failure').value)

        flat_goals = [float(value) for value in self.get_parameter('goals').value]
        if not flat_goals or len(flat_goals) % 3 != 0:
            raise ValueError(
                'Tham số goals phải gồm các bộ ba [x, y, yaw, ...].')
        self.goals: List[Tuple[float, float, float]] = [
            tuple(flat_goals[index:index + 3])
            for index in range(0, len(flat_goals), 3)
        ]

        configured_names = [
            str(name) for name in self.get_parameter('goal_names').value]
        self.goal_names = [
            configured_names[index]
            if index < len(configured_names)
            else f'Goal {index + 1}'
            for index in range(len(self.goals))
        ]

        action_name = str(self.get_parameter('action_name').value)
        pose_topic = str(self.get_parameter('amcl_pose_topic').value)
        self.action_client = ActionClient(self, NavigateToPose, action_name)
        self.create_subscription(
            PoseWithCovarianceStamped,
            pose_topic,
            self._initial_pose_callback,
            10,
        )
        # RViz publishes the user's 2D Pose Estimate on /initialpose.  AMCL
        # normally follows with /amcl_pose, but that publish can be delayed on
        # a heavily loaded simulator.  Receiving either is sufficient to start
        # the delayed Nav2 sequence after the operator has localized the robot.
        self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self._initial_pose_callback,
            10,
        )

        marker_qos = QoSProfile(depth=1)
        marker_qos.reliability = ReliabilityPolicy.RELIABLE
        marker_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.marker_publisher = self.create_publisher(
            MarkerArray, 'waypoints', marker_qos)

        self.have_initial_pose = not self.wait_for_initial_pose
        self.goal_index = 0
        self.retry_count = 0
        self.goal_in_progress = False
        self.sequence_started = False
        self.ready_since: Optional[float] = None
        self.last_feedback_log = -10.0
        self.start_time = self._now_seconds()
        self.timer = self.create_timer(0.5, self._tick)

        self._publish_markers()
        self.get_logger().info(
            f'Đã nạp {len(self.goals)} goal. Đang chờ Nav2 và Initial Pose.')

    def _now_seconds(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _initial_pose_callback(self, _: PoseWithCovarianceStamped) -> None:
        if not self.have_initial_pose:
            self.have_initial_pose = True
            self.get_logger().info(
                'Đã nhận initial pose. Localization sẵn sàng để chạy tự động.')

    def _tick(self) -> None:
        if self.goal_in_progress or self.goal_index >= len(self.goals):
            return

        now = self._now_seconds()
        if not self.have_initial_pose:
            if now - self.start_time > 10.0:
                self.get_logger().warn(
                    'Chưa có /amcl_pose: hãy dùng 2D Pose Estimate trong RViz.',
                    throttle_duration_sec=10.0,
                )
            return

        if not self.action_client.server_is_ready():
            if now - self.start_time > self.server_timeout:
                self.get_logger().error(
                    'Nav2 action server chưa sẵn sàng. Kiểm tra navigation launch.')
                self.timer.cancel()
            else:
                self.get_logger().info(
                    'Đang chờ action /navigate_to_pose...',
                    throttle_duration_sec=5.0,
                )
            return

        if self.ready_since is None:
            self.ready_since = now
            delay = self.initial_delay if not self.sequence_started \
                else self.wait_between_goals
            self.get_logger().info(f'Bắt đầu goal tiếp theo sau {delay:.1f} giây.')
            return

        delay = self.initial_delay if not self.sequence_started \
            else self.wait_between_goals
        if now - self.ready_since < delay:
            return

        self.ready_since = None
        self.sequence_started = True
        self._send_current_goal()

    def _send_current_goal(self) -> None:
        x, y, yaw = self.goals[self.goal_index]
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self.frame_id
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.goal_in_progress = True
        self.get_logger().info(
            f'Gửi {self.goal_names[self.goal_index]}: '
            f'x={x:.2f}, y={y:.2f}, yaw={yaw:.2f} rad '
            f'(lần thử {self.retry_count + 1}/{self.max_retries + 1}).')
        future = self.action_client.send_goal_async(
            goal, feedback_callback=self._feedback_callback)
        future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:  # ROS future propagates transport errors here.
            self.get_logger().error(f'Lỗi khi gửi goal: {exc}')
            self._handle_failure()
            return

        if not goal_handle.accepted:
            self.get_logger().error('Nav2 từ chối goal.')
            self._handle_failure()
            return

        self.get_logger().info('Nav2 đã nhận goal, robot đang di chuyển.')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _feedback_callback(self, feedback_message) -> None:
        now = self._now_seconds()
        if now - self.last_feedback_log < 2.0:
            return
        self.last_feedback_log = now
        distance = feedback_message.feedback.distance_remaining
        self.get_logger().info(f'Còn cách goal khoảng {distance:.2f} m.')

    def _result_callback(self, future) -> None:
        try:
            status = future.result().status
        except Exception as exc:
            self.get_logger().error(f'Lỗi khi nhận kết quả goal: {exc}')
            self._handle_failure()
            return

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(
                f'Hoàn thành {self.goal_names[self.goal_index]}.')
            self.goal_index += 1
            self.retry_count = 0
            self.goal_in_progress = False
            if self.goal_index == len(self.goals):
                self.get_logger().info(
                    f'HOÀN THÀNH: robot đã đi đủ {len(self.goals)} goal liên tiếp.')
                self.timer.cancel()
            return

        status_names = {
            GoalStatus.STATUS_ABORTED: 'ABORTED',
            GoalStatus.STATUS_CANCELED: 'CANCELED',
        }
        self.get_logger().error(
            f'Goal kết thúc với trạng thái {status_names.get(status, str(status))}.')
        self._handle_failure()

    def _handle_failure(self) -> None:
        self.goal_in_progress = False
        if self.retry_count < self.max_retries:
            self.retry_count += 1
            self.ready_since = self._now_seconds()
            self.get_logger().warn('Sẽ thử lại goal hiện tại.')
            return

        if self.continue_on_failure:
            self.get_logger().warn('Bỏ qua goal lỗi và chuyển sang goal kế tiếp.')
            self.goal_index += 1
            self.retry_count = 0
            self.ready_since = self._now_seconds()
            return

        self.get_logger().error(
            'Dừng chuỗi goal để tránh robot tiếp tục khi navigation đang lỗi.')
        self.timer.cancel()

    def _publish_markers(self) -> None:
        markers = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        colors = [(0.1, 0.8, 0.2), (0.1, 0.4, 1.0), (1.0, 0.35, 0.1)]

        for index, ((x, y, _), name) in enumerate(zip(self.goals, self.goal_names)):
            red, green, blue = colors[index % len(colors)]

            point = Marker()
            point.header.frame_id = self.frame_id
            point.header.stamp = stamp
            point.ns = 'assignment_goals'
            point.id = index * 2
            point.type = Marker.SPHERE
            point.action = Marker.ADD
            point.pose.position.x = x
            point.pose.position.y = y
            point.pose.position.z = 0.12
            point.pose.orientation.w = 1.0
            point.scale.x = point.scale.y = point.scale.z = 0.25
            point.color.r = red
            point.color.g = green
            point.color.b = blue
            point.color.a = 0.95
            markers.markers.append(point)

            label = Marker()
            label.header.frame_id = self.frame_id
            label.header.stamp = stamp
            label.ns = 'assignment_goal_labels'
            label.id = index * 2 + 1
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = x
            label.pose.position.y = y
            label.pose.position.z = 0.42
            label.pose.orientation.w = 1.0
            label.scale.z = 0.22
            label.color.r = label.color.g = label.color.b = 0.05
            label.color.a = 1.0
            label.text = name
            markers.markers.append(label)

        self.marker_publisher.publish(markers)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WaypointNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

